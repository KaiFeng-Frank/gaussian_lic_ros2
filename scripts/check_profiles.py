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
    "fx": (int, float),
    "fy": (int, float),
    "cx": (int, float),
    "cy": (int, float),
    "sh_degree": int,
    "scaling_scale": (int, float),
    "gaussian_init_min_points": int,
    "gaussian_init_min_keyframes": int,
    "enable_torch_gaussian_extend": bool,
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
    "map_points_topic": str,
    "gaussian_map_topic": str,
    "save_map_service": str,
    "status_topic": str,
    "world_frame": str,
    "publish_tf": bool,
    "camera_frame": str,
}

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
    if params.get("render_mode") not in {"debug_cpu", "debug_input", "rasterizer", "off"}:
        errors.append("render_mode must be debug_cpu, debug_input, rasterizer, or off")
    if "rendered_image_mode" in params:
        errors.append("rendered_image_mode is deprecated; use render_mode")

    for key in ("fx", "fy", "sync_tolerance_sec", "scaling_scale"):
        value = params.get(key)
        if isinstance(value, (int, float)) and value <= 0:
            errors.append(f"{key} must be positive")

    for key in (
        "max_queue_size",
        "sensor_qos_depth",
        "process_period_ms",
        "select_every_k_frame",
        "gaussian_init_min_points",
        "gaussian_init_min_keyframes",
        "gaussian_map_chunk_size",
    ):
        value = params.get(key)
        if isinstance(value, int) and value <= 0:
            errors.append(f"{key} must be positive")

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
