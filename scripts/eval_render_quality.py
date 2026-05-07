#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
from PIL import Image


IMAGE_EXTENSIONS = {".bmp", ".jpeg", ".jpg", ".png", ".tif", ".tiff"}


def list_images(root):
    if not root.is_dir():
        return []
    return [
        path
        for path in sorted(root.rglob("*"))
        if path.is_file() and path.suffix.lower() in IMAGE_EXTENSIONS
    ]


def load_rgb(path):
    with Image.open(path) as image:
        return np.asarray(image.convert("RGB"), dtype=np.float64) / 255.0


def psnr(reference, candidate):
    mse = float(np.mean((reference - candidate) ** 2))
    if mse <= 0.0:
        return 100.0
    return 10.0 * math.log10(1.0 / mse)


def ssim_global(reference, candidate):
    values = []
    c1 = 0.01 ** 2
    c2 = 0.03 ** 2
    for channel in range(reference.shape[2]):
        x = reference[:, :, channel]
        y = candidate[:, :, channel]
        mean_x = float(np.mean(x))
        mean_y = float(np.mean(y))
        var_x = float(np.mean((x - mean_x) ** 2))
        var_y = float(np.mean((y - mean_y) ** 2))
        cov_xy = float(np.mean((x - mean_x) * (y - mean_y)))
        denominator = (mean_x * mean_x + mean_y * mean_y + c1) * (var_x + var_y + c2)
        values.append(((2.0 * mean_x * mean_y + c1) * (2.0 * cov_xy + c2)) / denominator)
    return float(np.mean(values))


def gaussian_window(window_size=11, sigma=1.5):
    offsets = np.arange(window_size, dtype=np.float64) - (window_size // 2)
    values = np.exp(-(offsets ** 2) / (2.0 * sigma * sigma))
    values /= np.sum(values)
    return values


SSIM_KERNEL_1D = gaussian_window()


def gaussian_filter_zero_padded(image):
    pad = SSIM_KERNEL_1D.size // 2
    padded_x = np.pad(image, ((0, 0), (pad, pad)), mode="constant")
    windows_x = np.lib.stride_tricks.sliding_window_view(padded_x, SSIM_KERNEL_1D.size, axis=1)
    filtered_x = np.tensordot(windows_x, SSIM_KERNEL_1D, axes=([-1], [0]))
    padded_y = np.pad(filtered_x, ((pad, pad), (0, 0)), mode="constant")
    windows_y = np.lib.stride_tricks.sliding_window_view(padded_y, SSIM_KERNEL_1D.size, axis=0)
    return np.tensordot(windows_y, SSIM_KERNEL_1D, axes=([-1], [0]))


def ssim_windowed(reference, candidate):
    """Match upstream loss_utils::ssim: 11x11 Gaussian conv2d with zero padding."""
    values = []
    c1 = 0.01 ** 2
    c2 = 0.03 ** 2
    for channel in range(reference.shape[2]):
        x = reference[:, :, channel]
        y = candidate[:, :, channel]
        mean_x = gaussian_filter_zero_padded(x)
        mean_y = gaussian_filter_zero_padded(y)
        mean_x_sq = mean_x * mean_x
        mean_y_sq = mean_y * mean_y
        mean_xy = mean_x * mean_y
        var_x = gaussian_filter_zero_padded(x * x) - mean_x_sq
        var_y = gaussian_filter_zero_padded(y * y) - mean_y_sq
        cov_xy = gaussian_filter_zero_padded(x * y) - mean_xy
        ssim_map = ((2.0 * mean_xy + c1) * (2.0 * cov_xy + c2)) / (
            (mean_x_sq + mean_y_sq + c1) * (var_x + var_y + c2)
        )
        values.append(float(np.mean(ssim_map)))
    return float(np.mean(values))


def load_lpips_model(model_path, requested_device, lpips_pytorch_root):
    try:
        import torch  # noqa: PLC0415
    except Exception as exc:  # noqa: BLE001
        return None, None, "", f"python torch unavailable: {exc}"
    errors = []
    device = requested_device
    if requested_device != "cpu" and not torch.cuda.is_available():
        errors.append(
            f"requested LPIPS device {requested_device} unavailable for torch "
            f"{getattr(torch, '__version__', 'unknown')}; falling back to cpu"
        )
        device = "cpu"
    if model_path and device != "cpu":
        try:
            model = torch.jit.load(str(model_path), map_location=device)
            model.eval()
            return torch, model, device, None
        except Exception as exc:  # noqa: BLE001
            errors.append(f"TorchScript LPIPS load failed: {exc}")
    elif model_path:
        errors.append("TorchScript LPIPS skipped on CPU because the upstream trace pins CUDA tensors")

    if lpips_pytorch_root and lpips_pytorch_root.is_dir():
        try:
            sys.path.insert(0, str(lpips_pytorch_root))
            from lpipsPyTorch import LPIPS  # noqa: PLC0415

            model = LPIPS(net_type="alex").to(device)
            model.eval()
            return torch, model, device, "; ".join(errors) if errors else None
        except Exception as exc:  # noqa: BLE001
            errors.append(f"lpipsPyTorch fallback failed: {exc}")

    return torch, None, device, "; ".join(errors) if errors else "LPIPS model unavailable"


def lpips_value(torch, model, reference, candidate, device):
    ref = torch.from_numpy(reference.astype(np.float32)).permute(2, 0, 1).unsqueeze(0).to(device)
    cand = torch.from_numpy(candidate.astype(np.float32)).permute(2, 0, 1).unsqueeze(0).to(device)
    with torch.no_grad():
        value = model(cand, ref).detach().cpu().reshape(-1)[0].item()
    return float(value)


def summarize(items, prefix, all_items):
    selected = [item for item in items if item["name"].startswith(prefix)]
    source = prefix.rstrip("_")
    if not selected:
        selected = all_items
        source = "all_matched_no_" + prefix.rstrip("_") + "_frames"
    if not selected:
        return {
            "source": source,
            "count": 0,
            "psnr": None,
            "ssim": None,
            "global_ssim": None,
            "lpips": None,
        }
    lpips_values = [item["lpips"] for item in selected if item["lpips"] is not None]
    return {
        "source": source,
        "count": len(selected),
        "psnr": float(np.mean([item["psnr"] for item in selected])),
        "ssim": float(np.mean([item["ssim"] for item in selected])),
        "global_ssim": float(np.mean([item["global_ssim"] for item in selected])),
        "lpips": float(np.mean(lpips_values)) if lpips_values else None,
    }


def compute_quality(args):
    render_dir = Path(args.render_dir).expanduser().resolve()
    reference_dir = Path(args.reference_dir).expanduser().resolve()
    lpips_model = Path(args.lpips_model).expanduser().resolve() if args.lpips_model else None
    lpips_pytorch_root = (
        Path(args.lpips_pytorch_root).expanduser().resolve()
        if args.lpips_pytorch_root
        else (lpips_model.parent if lpips_model else None)
    )

    reference_by_name = {path.name: path for path in list_images(reference_dir)}
    pairs = []
    for render_path in list_images(render_dir):
        reference_path = reference_by_name.get(render_path.name)
        if reference_path is not None:
            pairs.append((render_path, reference_path))
        if args.max_images > 0 and len(pairs) >= args.max_images:
            break

    lpips_error = None
    torch = None
    lpips = None
    lpips_device = args.lpips_device
    if lpips_model and lpips_model.is_file():
        torch, lpips, lpips_device, lpips_error = load_lpips_model(
            lpips_model, args.lpips_device, lpips_pytorch_root
        )
    elif lpips_model:
        lpips_error = f"LPIPS model is missing: {lpips_model}"
        torch, lpips, lpips_device, fallback_error = load_lpips_model(
            None, args.lpips_device, lpips_pytorch_root
        )
        if fallback_error:
            lpips_error += "; " + fallback_error

    items = []
    errors = []
    lpips_errors = []
    for render_path, reference_path in pairs:
        try:
            candidate = load_rgb(render_path)
            reference = load_rgb(reference_path)
            if candidate.shape != reference.shape:
                raise ValueError(f"shape mismatch render={candidate.shape} reference={reference.shape}")
            item = {
                "name": render_path.name,
                "render": str(render_path),
                "reference": str(reference_path),
                "psnr": psnr(reference, candidate),
                "ssim": ssim_windowed(reference, candidate),
                "global_ssim": ssim_global(reference, candidate),
                "lpips": None,
            }
            if torch is not None and lpips is not None:
                try:
                    item["lpips"] = lpips_value(torch, lpips, reference, candidate, lpips_device)
                except Exception as exc:  # noqa: BLE001
                    lpips_errors.append(f"{render_path.name}: {exc}")
                    lpips = None
            items.append(item)
        except Exception as exc:  # noqa: BLE001
            errors.append(f"{render_path.name}: {exc}")

    train = summarize(items, "train_", items)
    novel = summarize(items, "test_", items)
    quality = {
        "schema": "gaussian_lic_render_quality/v1",
        "render_dir": str(render_dir),
        "reference_dir": str(reference_dir),
        "matched_pairs": len(items),
        "failed_pairs": errors,
        "train_source": train["source"],
        "novel_source": novel["source"],
        "train_frame_count": train["count"],
        "novel_frame_count": novel["count"],
        "train_psnr": train["psnr"],
        "train_ssim": train["ssim"],
        "train_global_ssim": train["global_ssim"],
        "train_lpips": train["lpips"],
        "novel_psnr": novel["psnr"],
        "novel_ssim": novel["ssim"],
        "novel_global_ssim": novel["global_ssim"],
        "novel_lpips": novel["lpips"],
        "ssim_method": "gaussian_lic_windowed_11x11_sigma1.5_zero_pad",
        "lpips_model": str(lpips_model) if lpips_model else "",
        "lpips_requested_device": args.lpips_device,
        "lpips_device": lpips_device if torch is not None and lpips is not None else "",
        "lpips_error": "; ".join([part for part in [lpips_error, *lpips_errors] if part]),
        "pairs": items,
    }
    return quality


def main():
    parser = argparse.ArgumentParser(description="Evaluate rendered RGB images against same-name GT images.")
    parser.add_argument("--result-dir", required=True, help="Directory containing metrics.json to update.")
    parser.add_argument("--render-dir", required=True, help="Rendered image directory.")
    parser.add_argument("--reference-dir", required=True, help="Ground-truth image directory.")
    parser.add_argument("--metrics-name", default="metrics.json")
    parser.add_argument(
        "--lpips-model",
        default="external/Gaussian-LIC/src/lpips/lpips_alex.pt",
        help="TorchScript LPIPS model path. LPIPS stays null if Python torch is unavailable.",
    )
    parser.add_argument("--lpips-device", default="cpu")
    parser.add_argument(
        "--lpips-pytorch-root",
        default="external/Gaussian-LIC/src/lpips",
        help="Directory containing lpipsPyTorch/ for fallback LPIPS evaluation.",
    )
    parser.add_argument("--max-images", type=int, default=0)
    args = parser.parse_args()

    result_dir = Path(args.result_dir).expanduser().resolve()
    metrics_path = result_dir / args.metrics_name
    if not metrics_path.is_file():
        raise SystemExit(f"missing metrics file: {metrics_path}")

    quality = compute_quality(args)
    metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
    previous = metrics.get("quality", {})
    if isinstance(previous, dict) and "final_gaussians" in previous and "final_gaussians" not in quality:
        quality["final_gaussians"] = previous["final_gaussians"]
    metrics["quality"] = quality
    metrics_path.write_text(json.dumps(metrics, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    report = {
        "ok": quality["matched_pairs"] > 0 and not quality["failed_pairs"],
        "metrics": str(metrics_path),
        "quality": quality,
    }
    print(json.dumps(report, indent=2, sort_keys=True))
    raise SystemExit(0 if report["ok"] else 1)


if __name__ == "__main__":
    main()
