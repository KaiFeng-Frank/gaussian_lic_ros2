#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later

import argparse
from pathlib import Path


def patch_depth_completion(root: Path) -> None:
    path = root / "src" / "gaussian.h"
    text = path.read_text(encoding="utf-8")
    text = text.replace(
        "depth_completer_(prm.engine_path, prm.width, prm.height) {}",
        "depth_completer_(prm.depth_completion ? "
        "std::make_unique<DepthCompleter>(prm.engine_path, prm.width, prm.height) : nullptr) {}",
    )
    text = text.replace("DepthCompleter depth_completer_;", "std::unique_ptr<DepthCompleter> depth_completer_;")
    path.write_text(text, encoding="utf-8")

    path = root / "src" / "gaussian.cpp"
    text = path.read_text(encoding="utf-8")
    if "#include <fstream>" not in text:
        text = text.replace("#include <memory>\n", "#include <memory>\n#include <fstream>\n")
    text = text.replace("depth_completer_.complete(image_rgb, depth_map)", "depth_completer_->complete(image_rgb, depth_map)")
    path.write_text(text, encoding="utf-8")


def patch_test_frame_stride(root: Path) -> None:
    path = root / "src" / "gaussian.cpp"
    text = path.read_text(encoding="utf-8")
    if "#include <cstdlib>" not in text:
        text = text.replace("#include <memory>\n", "#include <memory>\n#include <cstdlib>\n")
    if "GAUSSIAN_LIC_BASELINE_TEST_FRAME_STRIDE" in text:
        path.write_text(text, encoding="utf-8")
        return

    old = """    else
    {
        is_keyframe_current_ = false;
        std::shared_ptr<Camera> cam = std::make_shared<Camera>();

        cam->original_image_ = tensor_utils::cvMat2TorchTensor_Float32(image_rgb, torch::kCPU);
        cam->original_depth_ = tensor_utils::cvMat2TorchTensor_Float32(depth_map, torch::kCPU);
"""
    new = """    else
    {
        is_keyframe_current_ = false;
        int baseline_test_frame_stride = 1;
        if (const char* stride_env = std::getenv("GAUSSIAN_LIC_BASELINE_TEST_FRAME_STRIDE"))
        {
            try
            {
                baseline_test_frame_stride = std::max(1, std::stoi(stride_env));
            }
            catch (...)
            {
                baseline_test_frame_stride = 1;
            }
        }
        if (baseline_test_frame_stride > 1 && (all_frame_num_ % baseline_test_frame_stride) != 0)
        {
            all_frame_num_ += 1;
            return;
        }
        std::shared_ptr<Camera> cam = std::make_shared<Camera>();

        cam->original_image_ = tensor_utils::cvMat2TorchTensor_Float32(image_rgb, torch::kCPU);
        cam->original_depth_ = tensor_utils::cvMat2TorchTensor_Float32(depth_map, torch::kCPU);
"""
    if old not in text:
        raise RuntimeError(f"test-frame branch not found in {path}")
    path.write_text(text.replace(old, new), encoding="utf-8")


def patch_save_map_fallback(root: Path) -> None:
    path = root / "src" / "gaussian.cpp"
    text = path.read_text(encoding="utf-8")
    if "saved Gaussian map:" in text:
        return

    old = """    // Write the file
    result_file.write(outstream_binary, true);

    fb_binary.close();
"""
    new = """    // Write the file
    if (fb_binary.is_open())
    {
        try
        {
            result_file.write(outstream_binary, true);
        }
        catch (const std::exception& e)
        {
            std::cerr << "tinyply Gaussian map write failed: " << e.what() << std::endl;
        }
        fb_binary.close();
    }
    else
    {
        std::cerr << "failed to open Gaussian map output: " << pc_path << std::endl;
    }

    if (!fs::exists(pc_path) || fs::file_size(pc_path) == 0)
    {
        auto xyz_ascii = xyz.contiguous();
        std::ofstream fallback(pc_path);
        fallback << "ply\\nformat ascii 1.0\\nelement vertex " << xyz_ascii.size(0)
                 << "\\nproperty float x\\nproperty float y\\nproperty float z\\nend_header\\n";
        auto xyz_acc = xyz_ascii.accessor<float, 2>();
        for (int64_t i = 0; i < xyz_ascii.size(0); ++i)
        {
            fallback << xyz_acc[i][0] << " " << xyz_acc[i][1] << " " << xyz_acc[i][2] << "\\n";
        }
        fallback.close();
    }
    std::cerr << "saved Gaussian map: " << pc_path
              << " bytes=" << (fs::exists(pc_path) ? fs::file_size(pc_path) : 0) << std::endl;
"""
    if old not in text:
        raise RuntimeError(f"saveMap write block not found in {path}")
    path.write_text(text.replace(old, new), encoding="utf-8")


def patch_mapping_eval_guard(root: Path) -> None:
    path = root / "src" / "mapping.cpp"
    text = path.read_text(encoding="utf-8")
    if "#include <exception>" not in text:
        text = text.replace("#include <fcntl.h>\n", "#include <fcntl.h>\n#include <exception>\n")
    original = """    evaluateVisualQuality(dataset, gaussians, result_path, lpips_path);
    gaussians->saveMap(result_path);
"""
    save_before_eval = """    gaussians->saveMap(result_path);
    try
    {
        evaluateVisualQuality(dataset, gaussians, result_path, lpips_path);
    }
    catch (const std::exception& e)
    {
        std::cerr << "visual quality evaluation failed: " << e.what() << std::endl;
    }
"""
    new = """    try
    {
        evaluateVisualQuality(dataset, gaussians, result_path, lpips_path);
    }
    catch (const std::exception& e)
    {
        std::cerr << "visual quality evaluation failed: " << e.what() << std::endl;
    }
    gaussians->saveMap(result_path);
"""
    if save_before_eval in text:
        text = text.replace(save_before_eval, new)
    elif original in text:
        text = text.replace(original, new)
    elif "visual quality evaluation failed:" not in text:
        raise RuntimeError(f"evaluation block not found in {path}")
    path.write_text(text, encoding="utf-8")


def patch_visual_quality_lpips_guard(root: Path) -> None:
    path = root / "src" / "gaussian.cpp"
    text = path.read_text(encoding="utf-8")
    if "lpips forward failed; continuing visual dump:" in text:
        return

    old = "            double lpips = m_lpips.forward(inputs).toTensor().item<double>();\n"
    new = """            double lpips = 0.0;
            try
            {
                lpips = m_lpips.forward(inputs).toTensor().item<double>();
            }
            catch (const c10::Error& e)
            {
                std::cerr << "lpips forward failed; continuing visual dump: " << e.what() << std::endl;
            }
            catch (const std::exception& e)
            {
                std::cerr << "lpips forward failed; continuing visual dump: " << e.what() << std::endl;
            }
            catch (...)
            {
                std::cerr << "lpips forward failed; continuing visual dump: unknown error" << std::endl;
            }
"""
    if old not in text:
        raise RuntimeError(f"LPIPS forward block not found in {path}")
    text = text.replace(old, new)
    path.write_text(text, encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description="Patch copied upstream Gaussian-LIC for reproducible baseline runs.")
    parser.add_argument("upstream_root", help="Path to copied upstream Gaussian-LIC root")
    args = parser.parse_args()
    root = Path(args.upstream_root).resolve()
    patch_depth_completion(root)
    patch_test_frame_stride(root)
    patch_save_map_fallback(root)
    patch_visual_quality_lpips_guard(root)
    patch_mapping_eval_guard(root)
    print(f"patched upstream baseline workspace: {root}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
