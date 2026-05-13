#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later

import argparse
import hashlib
import json
import math
from pathlib import Path
import re
from types import SimpleNamespace
import sys

from baseline_manifest import build_manifest
from perf_regression import DEFAULT_KEYS, is_latency_key, load_json, lookup
from pointcloud_compare import compute_report as compute_pointcloud_report
from trajectory_compare import compute_report as compute_trajectory_report


IMAGE_EXTENSIONS = (".bmp", ".jpeg", ".jpg", ".png", ".tif", ".tiff")
FRAME_INDEX_RE = re.compile(r"_(\d+)(?:\.[^.]+)?$")
STRICT_QUALITY_METRICS = {
    "novel_psnr": {
        "aliases": (
            "quality.novel_psnr",
            "novel_view.psnr",
            "novel_psnr",
            "psnr",
        ),
        "direction": "higher_is_better",
    },
    "novel_ssim": {
        "aliases": (
            "quality.novel_ssim",
            "novel_view.ssim",
            "novel_ssim",
            "ssim",
        ),
        "direction": "higher_is_better",
    },
    "novel_lpips": {
        "aliases": (
            "quality.novel_lpips",
            "novel_view.lpips",
            "novel_lpips",
            "lpips",
        ),
        "direction": "lower_is_better",
    },
}


def auto_point_cloud_path(directory):
    for name in ("point_cloud.ply", "point_cloud_debug.ply"):
        path = directory / name
        if path.is_file():
            return path
    return directory / "point_cloud.ply"


def compare_metrics(baseline_path, current_path, keys, max_regression, require_keys=False):
    errors = []
    skipped = []
    comparisons = []
    try:
        baseline = load_json(baseline_path)
        current = load_json(current_path)
    except Exception as exc:  # noqa: BLE001 - report malformed metrics uniformly.
        return {
            "ok": False,
            "baseline": str(baseline_path),
            "current": str(current_path),
            "comparisons": comparisons,
            "skipped": skipped,
            "errors": [str(exc)],
        }

    for key in keys:
        try:
            base = lookup(baseline, key)
            cur = lookup(current, key)
        except (KeyError, TypeError) as exc:
            item = {"key": key, "reason": str(exc)}
            skipped.append(item)
            if require_keys:
                errors.append(f"{key} is required but unavailable: {exc}")
            continue

        if base == 0.0:
            skipped.append({"key": key, "reason": "baseline is zero"})
            if require_keys:
                errors.append(f"{key} is required but baseline is zero")
            continue

        if is_latency_key(key):
            regression = (cur - base) / abs(base)
            direction = "lower_is_better"
        else:
            regression = (base - cur) / abs(base)
            direction = "higher_is_better"
        ok = regression <= max_regression + 1e-12
        if not ok:
            errors.append(f"{key} regression {regression:.2%} > {max_regression:.2%}")

        comparisons.append(
            {
                "key": key,
                "baseline": base,
                "current": cur,
                "direction": direction,
                "regression": regression,
                "max_regression": max_regression,
                "ok": ok,
            }
        )

    if not comparisons and require_keys:
        errors.append("no comparable metrics")

    return {
        "ok": not errors,
        "skipped": not comparisons and not require_keys,
        "baseline": str(baseline_path),
        "current": str(current_path),
        "comparisons": comparisons,
        "skipped": skipped,
        "errors": errors,
    }


def lookup_first_number(metrics, aliases):
    errors = []
    for alias in aliases:
        try:
            return alias, lookup(metrics, alias)
        except (KeyError, TypeError) as exc:
            errors.append(f"{alias}: {exc}")
    raise KeyError("; ".join(errors))


def compare_quality_metrics(baseline_path, current_path, max_regression):
    errors = []
    skipped = []
    comparisons = []
    try:
        baseline = load_json(baseline_path)
        current = load_json(current_path)
    except Exception as exc:  # noqa: BLE001 - report malformed metrics uniformly.
        return {
            "ok": False,
            "baseline": str(baseline_path),
            "current": str(current_path),
            "comparisons": comparisons,
            "skipped": skipped,
            "errors": [str(exc)],
        }

    for canonical, spec in STRICT_QUALITY_METRICS.items():
        try:
            baseline_key, base = lookup_first_number(baseline, spec["aliases"])
            current_key, cur = lookup_first_number(current, spec["aliases"])
        except (KeyError, TypeError) as exc:
            errors.append(f"{canonical} is required for strict reproduction but missing/non-numeric: {exc}")
            skipped.append({"key": canonical, "reason": str(exc)})
            continue

        if not (math.isfinite(base) and math.isfinite(cur)):
            errors.append(f"{canonical} must be finite for strict reproduction")
            skipped.append({"key": canonical, "reason": "non-finite value"})
            continue

        if spec["direction"] == "lower_is_better":
            if base == 0.0:
                regression = 0.0 if cur == 0.0 else float("inf")
            else:
                regression = (cur - base) / abs(base)
        else:
            if base == 0.0:
                regression = 0.0 if cur == 0.0 else float("inf")
            else:
                regression = (base - cur) / abs(base)
        ok = regression <= max_regression + 1e-12
        if not ok:
            errors.append(f"{canonical} regression {regression:.2%} > {max_regression:.2%}")
        comparisons.append(
            {
                "key": canonical,
                "baseline_key": baseline_key,
                "current_key": current_key,
                "baseline": base,
                "current": cur,
                "direction": spec["direction"],
                "regression": regression,
                "max_regression": max_regression,
                "ok": ok,
            }
        )

    return {
        "ok": not errors,
        "baseline": str(baseline_path),
        "current": str(current_path),
        "comparisons": comparisons,
        "skipped": skipped,
        "errors": errors,
    }


def render_directory(root, override=None):
    if override:
        return Path(override).expanduser().resolve()
    for name in ("renders", "render"):
        path = root / name
        if path.is_dir():
            return path
    return root / "renders"


def list_render_images(directory):
    if not directory.is_dir():
        return []
    return [
        path
        for path in sorted(directory.rglob("*"))
        if path.is_file() and path.suffix.lower() in IMAGE_EXTENSIONS
    ]


def frame_index(path):
    match = FRAME_INDEX_RE.search(path.stem)
    if not match:
        return None
    return int(match.group(1))


def sort_frame_paths(paths):
    return sorted(paths, key=lambda path: (frame_index(path) is None, frame_index(path) or 0, path.name))


def render_match_keys(path, root):
    relative = path.relative_to(root).as_posix().lower()
    return (
        f"rel:{relative}",
        f"name:{path.name.lower()}",
        f"stem:{path.stem.lower()}",
    )


def gt_directory(root, baseline=False):
    candidates = []
    if baseline:
        candidates.append(root / "upstream_result" / "gt")
    candidates.append(root / "gt")
    for path in candidates:
        if path.is_dir():
            return path
    return candidates[0]


def decoded_rgb_hash(path):
    try:
        from PIL import Image  # noqa: PLC0415
        import numpy as np  # noqa: PLC0415
    except Exception as exc:  # noqa: BLE001 - strict image matching should fail clearly.
        raise RuntimeError("Pillow and numpy are required for GT-assisted render matching") from exc

    with Image.open(path) as image:
        rgb = np.asarray(image.convert("RGB"), dtype=np.uint8)
    digest = hashlib.sha256(rgb.tobytes()).hexdigest()
    return digest


def choose_nearest_unused(candidates, target_index, used):
    available = [path for path in candidates if path not in used]
    if not available:
        return None
    if target_index is None:
        return available[0]
    return min(
        available,
        key=lambda path: (
            abs((frame_index(path) if frame_index(path) is not None else target_index) - target_index),
            path.name,
        ),
    )


def match_render_pairs_by_gt_hash(baseline_dir, current_dir, baseline_images, current_images):
    baseline_gt_dir = gt_directory(baseline_dir, baseline=True)
    current_gt_dir = gt_directory(current_dir, baseline=False)
    metadata = {
        "mode": "decoded_gt_hash",
        "baseline_gt_dir": str(baseline_gt_dir),
        "current_gt_dir": str(current_gt_dir),
        "matched_gt_count": 0,
        "unmatched_current_gt_count": 0,
    }
    if not baseline_gt_dir.is_dir() or not current_gt_dir.is_dir():
        metadata["mode"] = "same_name"
        metadata["reason"] = "GT directories unavailable"
        return [], metadata

    baseline_render_by_name = {path.name: path for path in baseline_images}
    current_render_by_name = {path.name: path for path in current_images}
    baseline_gt_by_hash = {}
    for path in sort_frame_paths(list_render_images(baseline_gt_dir)):
        baseline_gt_by_hash.setdefault(decoded_rgb_hash(path), []).append(path)

    pairs = []
    used_baseline_gt = set()
    for current_gt_path in sort_frame_paths(list_render_images(current_gt_dir)):
        current_render = current_render_by_name.get(current_gt_path.name)
        if current_render is None:
            continue
        candidates = baseline_gt_by_hash.get(decoded_rgb_hash(current_gt_path), [])
        baseline_gt = choose_nearest_unused(candidates, frame_index(current_gt_path), used_baseline_gt)
        if baseline_gt is None:
            metadata["unmatched_current_gt_count"] += 1
            continue
        baseline_render = baseline_render_by_name.get(baseline_gt.name)
        if baseline_render is None:
            metadata["unmatched_current_gt_count"] += 1
            continue
        used_baseline_gt.add(baseline_gt)
        pairs.append((baseline_render, current_render))

    metadata["matched_gt_count"] = len(pairs)
    if not pairs:
        metadata["mode"] = "same_name"
        metadata["reason"] = "no exact decoded GT hash matches"
    return pairs, metadata


def match_render_pairs(baseline_images, current_images, baseline_root, current_root, baseline_dir, current_dir):
    gt_pairs, metadata = match_render_pairs_by_gt_hash(
        baseline_dir, current_dir, baseline_images, current_images)
    if gt_pairs:
        return gt_pairs, metadata

    current_by_key = {}
    for path in current_images:
        for key in render_match_keys(path, current_root):
            current_by_key.setdefault(key, path)

    pairs = []
    used = set()
    for baseline_path in baseline_images:
        for key in render_match_keys(baseline_path, baseline_root):
            current_path = current_by_key.get(key)
            if current_path and current_path not in used:
                pairs.append((baseline_path, current_path))
                used.add(current_path)
                break
    metadata["mode"] = "same_name"
    metadata["matched_gt_count"] = 0
    return pairs, metadata


def sample_evenly(items, max_items):
    if max_items <= 0 or len(items) <= max_items:
        return list(items)
    if max_items == 1:
        return [items[0]]

    sampled = []
    last_index = len(items) - 1
    for sample_index in range(max_items):
        item_index = round(sample_index * last_index / (max_items - 1))
        sampled.append(items[item_index])
    return sampled


def load_image_rgb(path):
    try:
        from PIL import Image  # noqa: PLC0415
        import numpy as np  # noqa: PLC0415
    except Exception as exc:  # noqa: BLE001 - gate should fail clearly when image deps are absent.
        raise RuntimeError("Pillow and numpy are required for strict render-pair checks") from exc

    with Image.open(path) as image:
        rgb = image.convert("RGB")
        return np.asarray(rgb, dtype=np.float64) / 255.0


def psnr(reference, candidate):
    import numpy as np  # noqa: PLC0415

    mse = float(np.mean((reference - candidate) ** 2))
    if mse <= 0.0:
        return float("inf")
    return 10.0 * math.log10(1.0 / mse)


def ssim_global(reference, candidate):
    import numpy as np  # noqa: PLC0415

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


def compare_render_pairs(args, baseline_dir, current_dir):
    baseline_render_dir = render_directory(baseline_dir, args.baseline_renders_dir)
    current_render_dir = render_directory(current_dir, args.current_renders_dir)
    baseline_images = list_render_images(baseline_render_dir)
    current_images = list_render_images(current_render_dir)
    pairs, matching = match_render_pairs(
        baseline_images, current_images, baseline_render_dir, current_render_dir, baseline_dir, current_dir)

    report = {
        "ok": False,
        "baseline_dir": str(baseline_render_dir),
        "current_dir": str(current_render_dir),
        "baseline_image_count": len(baseline_images),
        "current_image_count": len(current_images),
        "matched_pair_count": len(pairs),
        "min_pairs": args.min_render_pairs,
        "max_pairs": args.max_render_pairs,
        "matching": matching,
        "min_psnr_db": args.min_render_pair_psnr,
        "min_ssim": args.min_render_pair_ssim,
        "max_failure_ratio": args.max_render_pair_failure_ratio,
        "min_mean_psnr_db": args.min_mean_render_pair_psnr,
        "min_mean_ssim": args.min_mean_render_pair_ssim,
        "pairs": [],
        "pair_failures": [],
        "summary": {},
        "errors": [],
    }
    if len(pairs) < args.min_render_pairs:
        report["errors"].append(f"matched render pairs {len(pairs)} < min_render_pairs {args.min_render_pairs}")
        return report

    psnr_values = []
    ssim_values = []
    sampled_pairs = sample_evenly(pairs, args.max_render_pairs)
    report["sampled_pair_count"] = len(sampled_pairs)
    report["sampling"] = "evenly_spaced_full_sequence"

    for baseline_path, current_path in sampled_pairs:
        try:
            baseline_image = load_image_rgb(baseline_path)
            current_image = load_image_rgb(current_path)
        except Exception as exc:  # noqa: BLE001 - keep pair failures in the report.
            report["errors"].append(f"{baseline_path.name}: could not load render pair: {exc}")
            continue
        if baseline_image.shape != current_image.shape:
            report["errors"].append(
                f"{baseline_path.name}: shape mismatch baseline={baseline_image.shape} current={current_image.shape}"
            )
            continue
        pair_psnr = psnr(baseline_image, current_image)
        pair_ssim = ssim_global(baseline_image, current_image)
        ok = pair_psnr >= args.min_render_pair_psnr and pair_ssim >= args.min_render_pair_ssim
        if not ok:
            report["pair_failures"].append(
                {
                    "baseline": str(baseline_path),
                    "current": str(current_path),
                    "psnr_db": pair_psnr,
                    "ssim": pair_ssim,
                    "reason": (
                        f"render similarity below threshold "
                        f"psnr={pair_psnr:.3f}dB ssim={pair_ssim:.6f}"
                    ),
                }
            )
        psnr_values.append(pair_psnr)
        ssim_values.append(pair_ssim)
        report["pairs"].append(
            {
                "baseline": str(baseline_path),
                "current": str(current_path),
                "psnr_db": pair_psnr,
                "ssim": pair_ssim,
                "ok": ok,
            }
        )

    if not report["pairs"]:
        report["errors"].append("no render pairs could be decoded")
    else:
        finite_psnr = [value for value in psnr_values if math.isfinite(value)]
        failed_pair_count = len(report["pair_failures"])
        failed_pair_ratio = failed_pair_count / len(report["pairs"])
        mean_psnr = sum(finite_psnr) / len(finite_psnr) if finite_psnr else float("inf")
        mean_ssim = sum(ssim_values) / len(ssim_values)
        report["summary"] = {
            "mean_psnr_db": mean_psnr,
            "min_psnr_db": min(finite_psnr) if finite_psnr else float("inf"),
            "mean_ssim": mean_ssim,
            "min_ssim": min(ssim_values),
            "evaluated_pairs": len(report["pairs"]),
            "failed_pair_count": failed_pair_count,
            "failed_pair_ratio": failed_pair_ratio,
        }
        if failed_pair_ratio > args.max_render_pair_failure_ratio + 1e-12:
            report["errors"].append(
                f"render pair failure ratio {failed_pair_ratio:.2%} > "
                f"{args.max_render_pair_failure_ratio:.2%}"
            )
        if mean_psnr < args.min_mean_render_pair_psnr:
            report["errors"].append(
                f"mean render-pair PSNR {mean_psnr:.3f}dB < "
                f"{args.min_mean_render_pair_psnr:.3f}dB"
            )
        if mean_ssim < args.min_mean_render_pair_ssim:
            report["errors"].append(
                f"mean render-pair SSIM {mean_ssim:.6f} < "
                f"{args.min_mean_render_pair_ssim:.6f}"
            )
    report["ok"] = not report["errors"]
    return report


def run_novel_view_quality(args, baseline_dir, current_dir):
    if not args.strict:
        return {"ok": True, "skipped": True, "errors": []}
    quality_metrics = compare_quality_metrics(
        baseline_dir / args.metrics_name,
        current_dir / args.metrics_name,
        args.strict_max_regression,
    )
    render_pairs = compare_render_pairs(args, baseline_dir, current_dir)
    errors = []
    errors.extend(f"metrics: {error}" for error in quality_metrics.get("errors", []))
    errors.extend(f"render_pairs: {error}" for error in render_pairs.get("errors", []))
    return {
        "ok": quality_metrics.get("ok", False) and render_pairs.get("ok", False),
        "strict": True,
        "max_regression": args.strict_max_regression,
        "metrics": quality_metrics,
        "render_pairs": render_pairs,
        "errors": errors,
    }


def run_baseline_manifest(args, baseline_dir):
    if args.skip_baseline_manifest:
        return {"ok": True, "skipped": True, "errors": []}
    manifest_args = SimpleNamespace(
        baseline=str(baseline_dir),
        dataset=args.dataset,
        sequence=args.sequence or baseline_dir.name,
        min_renders=args.min_renders,
    )
    try:
        return build_manifest(manifest_args)
    except Exception as exc:  # noqa: BLE001 - keep report generation best-effort.
        return {"ok": False, "errors": [str(exc)]}


def run_trajectory_compare(args, baseline_dir, current_dir):
    if args.skip_trajectory:
        return {"ok": True, "skipped": True, "errors": []}
    compare_args = SimpleNamespace(
        baseline=str(baseline_dir / args.trajectory_name),
        current=str(current_dir / args.trajectory_name),
        align=args.trajectory_align,
        max_association_dt=args.max_association_dt,
        min_matches=args.min_trajectory_matches,
        min_coverage=args.min_trajectory_coverage,
        max_rmse_m=args.max_trajectory_rmse_m,
        max_mean_m=args.max_trajectory_mean_m,
        max_error_m=args.max_trajectory_error_m,
        max_path_drift=args.max_trajectory_path_drift,
        time_offset_sweep_min=getattr(args, "time_offset_sweep_min", 0.0),
        time_offset_sweep_max=getattr(args, "time_offset_sweep_max", 0.0),
        time_offset_sweep_step=getattr(args, "time_offset_sweep_step", 0.0),
        time_offset_sweep_min_matches=getattr(args, "time_offset_sweep_min_matches", 0),
    )
    try:
        return compute_trajectory_report(compare_args)
    except Exception as exc:  # noqa: BLE001 - keep report generation best-effort.
        return {"ok": False, "errors": [str(exc)]}


def run_pointcloud_compare(args, baseline_dir, current_dir):
    if args.skip_pointcloud:
        return {"ok": True, "skipped": True, "errors": []}
    baseline_path = Path(args.baseline_point_cloud) if args.baseline_point_cloud else baseline_dir / "point_cloud.ply"
    current_path = Path(args.current_point_cloud) if args.current_point_cloud else auto_point_cloud_path(current_dir)
    compare_args = SimpleNamespace(
        baseline=str(baseline_path),
        current=str(current_path),
        max_points=args.max_pointcloud_points,
        voxel_size=args.pointcloud_voxel_size,
        align=args.pointcloud_align,
        max_nearest_m=args.max_nearest_m,
        min_count_ratio=args.min_point_count_ratio,
        max_count_ratio=args.max_point_count_ratio,
        max_centroid_drift_m=args.max_centroid_drift_m,
        max_chamfer_rmse_m=args.max_chamfer_rmse_m,
        max_chamfer_mean_m=args.max_chamfer_mean_m,
        max_chamfer_max_m=args.max_chamfer_max_m,
        max_unmatched_ratio=args.max_unmatched_ratio,
        max_mean_rgb_drift=args.max_mean_rgb_drift,
        derive_gaussian_rgb=args.derive_gaussian_rgb,
    )
    try:
        return compute_pointcloud_report(compare_args)
    except Exception as exc:  # noqa: BLE001 - keep report generation best-effort.
        return {"ok": False, "errors": [str(exc)]}


def run_gaussian_color_compare(args, baseline_dir):
    current_point_cloud = getattr(args, "gaussian_color_current_point_cloud", None)
    if not current_point_cloud:
        return {"ok": True, "skipped": True, "errors": []}

    baseline_point_cloud = getattr(args, "gaussian_color_baseline_point_cloud", None)
    baseline_path = (
        Path(baseline_point_cloud)
        if baseline_point_cloud
        else baseline_dir / "point_cloud.ply"
    )
    compare_args = SimpleNamespace(
        baseline=str(baseline_path),
        current=str(Path(current_point_cloud)),
        max_points=args.max_pointcloud_points,
        voxel_size=args.pointcloud_voxel_size,
        align=args.pointcloud_align,
        max_nearest_m=args.max_nearest_m,
        min_count_ratio=args.min_point_count_ratio,
        max_count_ratio=args.max_point_count_ratio,
        max_centroid_drift_m=args.max_centroid_drift_m,
        max_chamfer_rmse_m=args.max_chamfer_rmse_m,
        max_chamfer_mean_m=args.max_chamfer_mean_m,
        max_chamfer_max_m=args.max_chamfer_max_m,
        max_unmatched_ratio=args.max_unmatched_ratio,
        max_mean_rgb_drift=getattr(args, "gaussian_color_max_mean_rgb_drift", args.max_mean_rgb_drift),
        derive_gaussian_rgb=True,
    )
    try:
        return compute_pointcloud_report(compare_args)
    except Exception as exc:  # noqa: BLE001 - keep report generation best-effort.
        return {"ok": False, "errors": [str(exc)]}


def gate_status(gate_report):
    if gate_report.get("skipped"):
        return "SKIP"
    return "PASS" if gate_report.get("ok") else "FAIL"


def render_markdown(report):
    lines = [
        "# Gaussian-LIC Reproduction Report",
        "",
        f"Status: {'PASS' if report['ok'] else 'FAIL'}",
        f"Dataset: {report['dataset']}",
        f"Sequence: {report['sequence']}",
        "",
        "| Gate | Status |",
        "| --- | --- |",
    ]
    gate_names = ("baseline_manifest", "metrics", "novel_view_quality", "trajectory", "point_cloud", "gaussian_color")
    for name in gate_names:
        lines.append(f"| {name} | {gate_status(report[name])} |")

    metrics = report["metrics"]
    if metrics.get("comparisons"):
        lines.extend(["", "## Metrics", "", "| Key | Baseline | Current | Regression | Status |", "| --- | ---: | ---: | ---: | --- |"])
        for item in metrics["comparisons"]:
            lines.append(
                f"| {item['key']} | {item['baseline']:.6g} | {item['current']:.6g} | "
                f"{item['regression']:.2%} | {'PASS' if item['ok'] else 'FAIL'} |"
            )

    quality = report["novel_view_quality"]
    if not quality.get("skipped"):
        quality_metrics = quality.get("metrics", {})
        if quality_metrics.get("comparisons"):
            lines.extend(
                [
                    "",
                    "## Novel View Quality",
                    "",
                    "| Key | Baseline | Current | Regression | Status |",
                    "| --- | ---: | ---: | ---: | --- |",
                ]
            )
            for item in quality_metrics["comparisons"]:
                lines.append(
                    f"| {item['key']} | {item['baseline']:.6g} | {item['current']:.6g} | "
                    f"{item['regression']:.2%} | {'PASS' if item['ok'] else 'FAIL'} |"
                )
        render_pairs = quality.get("render_pairs", {})
        if render_pairs.get("summary"):
            summary = render_pairs["summary"]
            lines.extend(
                [
                    "",
                    "## Render Pairs",
                    "",
                    f"Matched pairs: {render_pairs['matched_pair_count']}",
                    f"Evaluated pairs: {render_pairs.get('sampled_pair_count', len(render_pairs.get('pairs', [])))}",
                    f"Sampling: {render_pairs.get('sampling', 'first_n')}",
                    f"Mean PSNR: {summary['mean_psnr_db']:.3f} dB",
                    f"Mean SSIM: {summary['mean_ssim']:.6f}",
                    f"Pair outliers: {summary.get('failed_pair_count', 0)} / "
                    f"{summary.get('evaluated_pairs', 0)} "
                    f"({summary.get('failed_pair_ratio', 0.0):.2%}), allowed "
                    f"{render_pairs.get('max_failure_ratio', 0.0):.2%}",
                ]
            )

    trajectory = report["trajectory"]
    if "translation" in trajectory:
        lines.extend(
            [
                "",
                "## Trajectory",
                "",
                f"Matched poses: {trajectory['matched_poses']}",
                f"Translation RMSE: {trajectory['translation']['rmse_m']:.6f} m",
                f"Path drift: {trajectory['path_length']['relative_drift']:.2%}",
            ]
        )

    point_cloud = report["point_cloud"]
    if "nearest" in point_cloud:
        lines.extend(
            [
                "",
                "## Point Cloud",
                "",
                f"Count ratio: {point_cloud['count_ratio']:.3f}",
                f"Loaded sample ratio: {point_cloud.get('loaded_count_ratio', point_cloud['count_ratio']):.3f}",
                f"Centroid drift: {point_cloud['centroid_drift_m']:.6f} m",
                f"Bidirectional nearest RMSE: {point_cloud['nearest']['bidirectional']['rmse_m']:.6f} m",
            ]
        )

    gaussian_color = report["gaussian_color"]
    if "color" in gaussian_color:
        color = gaussian_color["color"]
        lines.extend(
            [
                "",
                "## Gaussian Color",
                "",
                f"Compared: {color['compared']}",
                f"Mean RGB drift: {color['mean_rgb_distance']:.6f}",
            ]
        )
        if color.get("baseline_mean_rgb") and color.get("current_mean_rgb"):
            lines.extend(
                [
                    f"Baseline mean RGB: {', '.join(f'{value:.3f}' for value in color['baseline_mean_rgb'])}",
                    f"Current mean RGB: {', '.join(f'{value:.3f}' for value in color['current_mean_rgb'])}",
                ]
            )

    failures = []
    for name in gate_names:
        failures.extend(f"{name}: {error}" for error in report[name].get("errors", []))
    if failures:
        lines.extend(["", "## Failures", ""])
        lines.extend(f"- {failure}" for failure in failures)

    return "\n".join(lines) + "\n"


def build_report(args):
    baseline_dir = Path(args.baseline_dir).expanduser().resolve()
    current_dir = Path(args.current_dir).expanduser().resolve()
    metric_keys = tuple(args.metric_keys) if args.metric_keys else DEFAULT_KEYS

    metrics_report = {"ok": True, "skipped": True, "errors": []}
    if not args.skip_metrics:
        metrics_report = compare_metrics(
            baseline_dir / args.metrics_name,
            current_dir / args.metrics_name,
            metric_keys,
            args.max_metric_regression,
        )

    novel_view_quality = run_novel_view_quality(args, baseline_dir, current_dir)
    report = {
        "schema": "gaussian_lic_reproduction_report/v1",
        "dataset": args.dataset,
        "sequence": args.sequence or baseline_dir.name,
        "strict": bool(args.strict),
        "baseline_dir": str(baseline_dir),
        "current_dir": str(current_dir),
        "baseline_manifest": run_baseline_manifest(args, baseline_dir),
        "metrics": metrics_report,
        "novel_view_quality": novel_view_quality,
        "trajectory": run_trajectory_compare(args, baseline_dir, current_dir),
        "point_cloud": run_pointcloud_compare(args, baseline_dir, current_dir),
        "gaussian_color": run_gaussian_color_compare(args, baseline_dir),
    }
    report["ok"] = all(
        report[name].get("ok", False)
        for name in ("baseline_manifest", "metrics", "novel_view_quality", "trajectory", "point_cloud", "gaussian_color")
    )
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description="Build a Gaussian-LIC baseline-vs-current reproduction report.")
    parser.add_argument("--baseline-dir", required=True, help="Archived ROS1 baseline directory")
    parser.add_argument("--current-dir", required=True, help="Current ROS2 result directory")
    parser.add_argument("--dataset", default="fastlivo2")
    parser.add_argument("--sequence")
    parser.add_argument("--output", help="Optional JSON report path")
    parser.add_argument("--markdown", help="Optional Markdown report path")
    parser.add_argument("--json", action="store_true", help="Print full JSON report")
    parser.add_argument(
        "--strict",
        action="store_true",
        help="Require paper-level novel-view PSNR/SSIM/LPIPS metrics plus render-pair similarity gates.",
    )
    parser.add_argument("--strict-max-regression", type=float, default=0.05)
    parser.add_argument("--baseline-renders-dir", help="Override baseline render image directory.")
    parser.add_argument("--current-renders-dir", help="Override current render image directory.")
    parser.add_argument("--min-render-pairs", type=int, default=1)
    parser.add_argument("--max-render-pairs", type=int, default=64)
    parser.add_argument("--min-render-pair-psnr", type=float, default=15.0)
    parser.add_argument("--min-render-pair-ssim", type=float, default=0.4)
    parser.add_argument(
        "--max-render-pair-failure-ratio",
        type=float,
        default=0.10,
        help=(
            "Maximum fraction of sampled render pairs allowed below the per-pair "
            "PSNR/SSIM thresholds. Strict quality metrics still compare every "
            "matched frame against GT."
        ),
    )
    parser.add_argument(
        "--min-mean-render-pair-psnr",
        type=float,
        default=20.0,
        help="Minimum mean PSNR across sampled baseline-vs-current render pairs.",
    )
    parser.add_argument(
        "--min-mean-render-pair-ssim",
        type=float,
        default=0.75,
        help="Minimum mean SSIM across sampled baseline-vs-current render pairs.",
    )

    parser.add_argument("--skip-baseline-manifest", action="store_true")
    parser.add_argument("--min-renders", type=int, default=1)

    parser.add_argument("--skip-metrics", action="store_true")
    parser.add_argument("--metrics-name", default="metrics.json")
    parser.add_argument("--metric-key", action="append", dest="metric_keys")
    parser.add_argument("--max-metric-regression", type=float, default=0.15)

    parser.add_argument("--skip-trajectory", action="store_true")
    parser.add_argument("--trajectory-name", default="trajectory.tum")
    parser.add_argument("--trajectory-align", choices=("none", "first", "yaw"), default="none")
    parser.add_argument("--max-association-dt", type=float, default=0.02)
    parser.add_argument("--min-trajectory-matches", type=int, default=2)
    parser.add_argument("--min-trajectory-coverage", type=float, default=0.8)
    parser.add_argument("--max-trajectory-rmse-m", type=float, default=0.05)
    parser.add_argument("--max-trajectory-mean-m", type=float, default=0.03)
    parser.add_argument("--max-trajectory-error-m", type=float, default=0.15)
    parser.add_argument("--max-trajectory-path-drift", type=float, default=0.05)
    parser.add_argument("--time-offset-sweep-min", type=float, default=0.0)
    parser.add_argument("--time-offset-sweep-max", type=float, default=0.0)
    parser.add_argument("--time-offset-sweep-step", type=float, default=0.0)
    parser.add_argument("--time-offset-sweep-min-matches", type=int, default=0)

    parser.add_argument("--skip-pointcloud", action="store_true")
    parser.add_argument("--baseline-point-cloud")
    parser.add_argument("--current-point-cloud")
    parser.add_argument("--max-pointcloud-points", type=int, default=200000)
    parser.add_argument("--pointcloud-voxel-size", type=float, default=0.05)
    parser.add_argument("--pointcloud-align", choices=("none", "centroid"), default="none")
    parser.add_argument("--max-nearest-m", type=float, default=0.5)
    parser.add_argument("--min-point-count-ratio", type=float, default=0.5)
    parser.add_argument("--max-point-count-ratio", type=float, default=2.0)
    parser.add_argument("--max-centroid-drift-m", type=float, default=0.2)
    parser.add_argument("--max-chamfer-rmse-m", type=float, default=0.15)
    parser.add_argument("--max-chamfer-mean-m", type=float, default=0.1)
    parser.add_argument("--max-chamfer-max-m", type=float, default=0.5)
    parser.add_argument("--max-unmatched-ratio", type=float, default=0.05)
    parser.add_argument("--max-mean-rgb-drift", type=float, default=40.0)
    parser.add_argument(
        "--derive-gaussian-rgb",
        action="store_true",
        help="Derive RGB means from Gaussian PLY f_dc_0..2 coefficients when explicit RGB is absent",
    )
    parser.add_argument(
        "--gaussian-color-baseline-point-cloud",
        help="Optional baseline PLY for the dedicated Torch Gaussian color gate. Defaults to baseline-dir/point_cloud.ply.",
    )
    parser.add_argument(
        "--gaussian-color-current-point-cloud",
        help="Current Torch Gaussian PLY for the dedicated color gate. Enables gaussian_color when set.",
    )
    parser.add_argument(
        "--gaussian-color-max-mean-rgb-drift",
        type=float,
        default=40.0,
        help="Maximum mean RGB drift for the dedicated Torch Gaussian color gate.",
    )
    args = parser.parse_args(argv)

    try:
        report = build_report(args)
    except Exception as exc:  # noqa: BLE001 - CLI should report unexpected assembly failures uniformly.
        print(f"reproduction report failed: {exc}", file=sys.stderr)
        return 2

    if args.output:
        output_path = Path(args.output)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    if args.markdown:
        markdown_path = Path(args.markdown)
        markdown_path.parent.mkdir(parents=True, exist_ok=True)
        markdown_path.write_text(render_markdown(report), encoding="utf-8")

    if args.json or not report["ok"]:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        print(
            "reproduction report OK: "
            f"sequence={report['sequence']} "
            f"metrics={gate_status(report['metrics'])} "
            f"novel_view_quality={gate_status(report['novel_view_quality'])} "
            f"trajectory={gate_status(report['trajectory'])} "
            f"point_cloud={gate_status(report['point_cloud'])} "
            f"gaussian_color={gate_status(report['gaussian_color'])}"
        )

    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
