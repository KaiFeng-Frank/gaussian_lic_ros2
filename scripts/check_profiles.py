#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later

from pathlib import Path
import sys

try:
    import yaml
except ImportError as exc:
    raise SystemExit("PyYAML is required; run with the ROS2 system Python") from exc


ROOT = Path(__file__).resolve().parents[1]
CONFIG_DIR = ROOT / "src" / "gaussian_lic_bringup" / "config"
MAPPER_INPUT_STREAMS = ("pointcloud", "pose", "image", "camera_info", "depth", "imu")

REQUIRED_PARAMS = {
    "image_topic": str,
    "camera_info_topic": str,
    "depth_topic": str,
    "pose_topic": str,
    "pointcloud_topic": str,
    "imu_topic": str,
    "sync_tolerance_sec": (int, float),
    "max_queue_size": int,
    "sensor_qos_reliability": str,
    "sensor_qos_history": str,
    "sensor_qos_depth": int,
    "process_period_ms": int,
    "select_every_k_frame": int,
    "require_depth_topic": bool,
    "fx": (int, float),
    "fy": (int, float),
    "cx": (int, float),
    "cy": (int, float),
    "width": int,
    "height": int,
    "depth_completion": bool,
    "depth_completion_engine_path": str,
    "patch_size": int,
    "max_depth": (int, float),
    "sh_degree": int,
    "white_background": bool,
    "random_background": bool,
    "convert_SHs_python": bool,
    "compute_cov3D_python": bool,
    "lambda_erank": (int, float),
    "scaling_scale": (int, float),
    "position_lr": (int, float),
    "feature_lr": (int, float),
    "opacity_lr": (int, float),
    "scaling_lr": (int, float),
    "rotation_lr": (int, float),
    "lambda_dssim": (int, float),
    "optimize_depth": bool,
    "lambda_depth": (int, float),
    "iteration_decay": bool,
    "apply_exposure": bool,
    "exposure_lr": (int, float),
    "gaussian_init_min_points": int,
    "gaussian_init_min_keyframes": int,
    "enable_torch_camera_conversion": bool,
    "enable_torch_gaussian_init": bool,
    "enable_torch_gaussian_extend": bool,
    "enable_torch_gaussian_extend_visibility_filter": bool,
    "torch_gaussian_extend_alpha_threshold": (int, float),
    "enable_torch_gaussian_optimization": bool,
    "torch_gaussian_optimization_steps": int,
    "torch_gaussian_optimization_max_samples": int,
    "torch_gaussian_optimization_sampling": str,
    "torch_gaussian_optimization_seed": int,
    "enable_torch_gaussian_pruning": bool,
    "torch_gaussian_prune_min_opacity": (int, float),
    "torch_gaussian_max_foreground": int,
    "torch_gaussian_prune_count_policy": str,
    "torch_gaussian_prune_max_world_scale": (int, float),
    "enable_torch_gaussian_densification": bool,
    "torch_gaussian_opacity_reset_interval": int,
    "skybox_points_num": int,
    "skybox_radius": (int, float),
    "torch_gaussian_device": str,
    "gaussian_map_chunk_size": int,
    "max_path_length": int,
    "max_map_points": int,
    "publish_rendered_preview": bool,
    "render_mode": str,
    "active_profile": str,
    "odometry_topic": str,
    "path_topic": str,
    "rendered_image_topic": str,
    "rendered_feedback_topic": str,
    "rendered_feedback_qos_reliability": str,
    "rendered_feedback_qos_durability": str,
    "rendered_feedback_qos_depth": int,
    "map_points_topic": str,
    "gaussian_map_topic": str,
    "save_map_service": str,
    "status_topic": str,
    "world_frame": str,
    "publish_tf": bool,
    "camera_frame": str,
}

for stream in MAPPER_INPUT_STREAMS:
    REQUIRED_PARAMS[f"{stream}_qos_reliability"] = str
    REQUIRED_PARAMS[f"{stream}_qos_history"] = str
    REQUIRED_PARAMS[f"{stream}_qos_depth"] = int

TOPIC_KEYS = {
    "image_topic",
    "camera_info_topic",
    "depth_topic",
    "pose_topic",
    "pointcloud_topic",
    "imu_topic",
    "odometry_topic",
    "path_topic",
    "rendered_image_topic",
    "rendered_feedback_topic",
    "map_points_topic",
    "gaussian_map_topic",
    "save_map_service",
    "status_topic",
}


def load_params(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError("top-level YAML value must be a mapping")
    try:
        params = data["mapping_node"]["ros__parameters"]
    except KeyError as exc:
        raise ValueError("missing mapping_node.ros__parameters") from exc
    if not isinstance(params, dict):
        raise ValueError("mapping_node.ros__parameters must be a mapping")
    return params


def check_profile(path: Path) -> list[str]:
    errors: list[str] = []
    try:
        params = load_params(path)
    except Exception as exc:  # noqa: BLE001 - report all profile failures uniformly.
        return [str(exc)]

    for key, expected_type in REQUIRED_PARAMS.items():
        if key not in params:
            errors.append(f"missing {key}")
            continue
        if not isinstance(params[key], expected_type):
            errors.append(f"{key} has type {type(params[key]).__name__}, expected {expected_type}")

    for key in TOPIC_KEYS:
        value = params.get(key)
        if isinstance(value, str) and not value.startswith("/"):
            errors.append(f"{key} must be an absolute ROS topic/service name")

    if params.get("sensor_qos_reliability") not in {"best_effort", "reliable"}:
        errors.append("sensor_qos_reliability must be best_effort or reliable")
    if params.get("sensor_qos_history") not in {"keep_last", "keep_all"}:
        errors.append("sensor_qos_history must be keep_last or keep_all")
    for stream in MAPPER_INPUT_STREAMS:
        reliability = params.get(f"{stream}_qos_reliability")
        history = params.get(f"{stream}_qos_history")
        if reliability not in {"best_effort", "reliable"}:
            errors.append(f"{stream}_qos_reliability must be best_effort or reliable")
        if history not in {"keep_last", "keep_all"}:
            errors.append(f"{stream}_qos_history must be keep_last or keep_all")
    if params.get("render_mode") not in {"debug_cpu", "debug_input", "rasterizer", "off"}:
        errors.append("render_mode must be debug_cpu, debug_input, rasterizer, or off")
    if params.get("torch_gaussian_prune_count_policy") not in {"opacity", "uniform"}:
        errors.append("torch_gaussian_prune_count_policy must be opacity or uniform")
    if "rendered_image_mode" in params:
        errors.append("rendered_image_mode is deprecated; use render_mode")

    for key in (
        "fx",
        "fy",
        "sync_tolerance_sec",
        "max_depth",
        "scaling_scale",
        "position_lr",
        "feature_lr",
        "opacity_lr",
        "scaling_lr",
        "rotation_lr",
        "exposure_lr",
    ):
        value = params.get(key)
        if isinstance(value, (int, float)) and value <= 0:
            errors.append(f"{key} must be positive")

    for key in ("lambda_erank", "lambda_dssim", "lambda_depth"):
        value = params.get(key)
        if isinstance(value, (int, float)) and value < 0:
            errors.append(f"{key} must be non-negative")

    for key in (
        "torch_gaussian_extend_alpha_threshold",
        "torch_gaussian_prune_min_opacity",
    ):
        value = params.get(key)
        if isinstance(value, (int, float)) and not 0.0 <= value <= 1.0:
            errors.append(f"{key} must be in [0, 1]")

    value = params.get("torch_gaussian_prune_max_world_scale")
    if isinstance(value, (int, float)) and value < 0:
        errors.append("torch_gaussian_prune_max_world_scale must be non-negative")

    for key in (
        "max_queue_size",
        "sensor_qos_depth",
        "pointcloud_qos_depth",
        "pose_qos_depth",
        "image_qos_depth",
        "camera_info_qos_depth",
        "depth_qos_depth",
        "imu_qos_depth",
        "process_period_ms",
        "select_every_k_frame",
        "width",
        "height",
        "patch_size",
        "gaussian_init_min_points",
        "gaussian_init_min_keyframes",
        "gaussian_map_chunk_size",
    ):
        value = params.get(key)
        if isinstance(value, int) and value <= 0:
            errors.append(f"{key} must be positive")

    for key in (
        "torch_gaussian_optimization_steps",
        "torch_gaussian_optimization_max_samples",
        "torch_gaussian_optimization_seed",
        "torch_gaussian_max_foreground",
        "torch_gaussian_opacity_reset_interval",
    ):
        value = params.get(key)
        if isinstance(value, int) and value < 0:
            errors.append(f"{key} must be non-negative")

    sampling = params.get("torch_gaussian_optimization_sampling")
    allowed_sampling = {"upstream_random", "random", "even", "latest_even", "deterministic_even"}
    if isinstance(sampling, str) and sampling not in allowed_sampling:
        errors.append(
            "torch_gaussian_optimization_sampling must be one of "
            f"{', '.join(sorted(allowed_sampling))}"
        )

    value = params.get("skybox_points_num")
    if isinstance(value, int) and value < 0:
        errors.append("skybox_points_num must be non-negative")
    value = params.get("skybox_radius")
    if isinstance(value, (int, float)) and value <= 0:
        errors.append("skybox_radius must be positive")

    return errors


def main() -> int:
    paths = sorted(CONFIG_DIR.glob("*.yaml"))
    if not paths:
        print(f"no profiles found in {CONFIG_DIR}", file=sys.stderr)
        return 1

    failed = False
    for path in paths:
        errors = check_profile(path)
        if errors:
            failed = True
            print(f"[profile] {path.name}: FAIL", file=sys.stderr)
            for error in errors:
                print(f"  - {error}", file=sys.stderr)
        else:
            print(f"[profile] {path.name}: OK")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
