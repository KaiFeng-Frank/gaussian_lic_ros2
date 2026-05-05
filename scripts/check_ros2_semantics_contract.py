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
MAPPING_NODE = ROOT / "src" / "gaussian_lic_mapping" / "src" / "mapping_node.cpp"
FRONTEND_ADAPTER = ROOT / "src" / "gaussian_lic_frontend" / "src" / "lic2_contract_adapter_node.cpp"
RUN_BAG_LAUNCH = ROOT / "src" / "gaussian_lic_bringup" / "launch" / "run_bag.launch.py"
TRACKING_LAUNCH = ROOT / "src" / "gaussian_lic_bringup" / "launch" / "tracking.launch.py"
TRACKING_NODE = ROOT / "src" / "gaussian_lic_tracking" / "src" / "tracking_node.cpp"
TRACKING_STATUS_MSG = ROOT / "src" / "gaussian_lic_msgs" / "msg" / "TrackingStatus.msg"
SEMANTICS_DOC = ROOT / "docs" / "ROS2_SEMANTICS.md"
TIMING_AUDIT = ROOT / "scripts" / "rosbag2_timing_audit.py"


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def mapping_params(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    return data.get("mapping_node", {}).get("ros__parameters", {})


def source_files() -> list[Path]:
    roots = [
        ROOT / "src" / "gaussian_lic_mapping",
        ROOT / "src" / "gaussian_lic_frontend",
        ROOT / "src" / "gaussian_lic_tracking",
    ]
    paths: list[Path] = []
    for root in roots:
        paths.extend(root.rglob("*.cpp"))
        paths.extend(root.rglob("*.hpp"))
        paths.extend(root.rglob("*.h"))
    return paths


def main() -> int:
    errors: list[str] = []

    if not SEMANTICS_DOC.is_file():
        errors.append("docs/ROS2_SEMANTICS.md is missing")
    if not TIMING_AUDIT.is_file():
        errors.append("scripts/rosbag2_timing_audit.py is missing")

    mapping_text = read(MAPPING_NODE)
    frontend_text = read(FRONTEND_ADAPTER)
    launch_text = read(RUN_BAG_LAUNCH)
    tracking_launch_text = read(TRACKING_LAUNCH)
    tracking_node_text = read(TRACKING_NODE)
    tracking_status_msg_text = read(TRACKING_STATUS_MSG)
    timing_audit_text = read(TIMING_AUDIT)

    if "stamp_to_sec" in mapping_text:
        errors.append("mapping_node still exposes stamp_to_sec; use int64 nanoseconds for sync math")
    if "sync_tolerance_nsec_" not in mapping_text:
        errors.append("mapping_node does not retain a nanosecond sync tolerance")
    if "const double frame_time" in mapping_text:
        errors.append("mapping_node frame synchronization regressed to double seconds")

    if "stamp_to_sec" in frontend_text:
        errors.append("lic2_contract_adapter still exposes stamp_to_sec; use int64 nanoseconds")
    if "last_imu_stamp_sec_" in frontend_text:
        errors.append("lic2_contract_adapter IMU integration still stores double-second stamps")
    if "last_imu_stamp_nsec_" not in frontend_text:
        errors.append("lic2_contract_adapter does not store IMU stamps in nanoseconds")

    if 'executable="component_container_mt"' in launch_text or "component_container_mt" in launch_text:
        errors.append("run_bag.launch.py must default strict composition to single-threaded component_container")
    if 'executable="component_container"' not in launch_text:
        errors.append("run_bag.launch.py does not use the single-threaded component_container")
    if '"--clock"' not in launch_text:
        errors.append("run_bag.launch.py rosbag2 replay must publish /clock")
    if '"--read-ahead-queue-size", "100"' not in launch_text:
        errors.append("run_bag.launch.py strict replay must use bounded read-ahead queue size 100")
    if '"use_sim_time", default_value="true"' not in launch_text:
        errors.append("run_bag.launch.py must default use_sim_time to true")
    if "global_time_regressions" not in timing_audit_text or "strict-storage" not in timing_audit_text:
        errors.append("rosbag2_timing_audit.py must check timestamp regressions and strict storage mode")

    required_tracking_launch_args = [
        "tracking_status_topic",
        "serialize_callbacks",
        "sensor_qos_reliability",
        "sensor_qos_depth",
        "enable_lidar_plane_factor",
        "enable_visual_alignment_window_factor",
        "enable_se3_photometric_window_factor",
        "trajectory_control_interval_ns",
        "enable_sliding_window_optimizer",
        "lidar_robust_kernel_m",
        "lidar_plane_min_neighbors",
        "lidar_plane_max_condition",
        "lidar_to_imu_translation_m",
        "lidar_to_imu_rpy_rad",
        "camera_to_imu_translation_m",
        "camera_to_imu_rpy_rad",
        "visual_factor_max_dt_ns",
        "depth_frame_cache_size",
        "rendered_frame_cache_size",
        "visual_alignment_meters_per_pixel",
        "visual_alignment_window_weight",
        "visual_alignment_huber_delta_m",
        "sliding_window_max_states",
        "sliding_window_max_iterations",
        "sliding_window_imu_weight",
        "sliding_window_pose_translation_weight",
        "sliding_window_pose_rotation_weight",
        "enable_sliding_window_smoothness_factor",
        "sliding_window_smoothness_rotation_weight",
        "sliding_window_smoothness_position_weight",
        "sliding_window_smoothness_velocity_weight",
        "sliding_window_smoothness_bias_weight",
    ]
    for argument in required_tracking_launch_args:
        if f'DeclareLaunchArgument("{argument}"' not in tracking_launch_text:
            errors.append(f"tracking.launch.py must expose {argument}")
        if f'"{argument}": {argument}' not in tracking_launch_text:
            errors.append(f"tracking.launch.py must pass {argument} into tracking_node")

    if 'declare_parameter<bool>("serialize_callbacks", true)' not in tracking_node_text:
        errors.append("tracking_node must default serialize_callbacks to true")
    if 'declare_parameter<bool>("enable_sliding_window_optimizer", true)' not in tracking_node_text:
        errors.append("tracking_node must default production sliding-window BA to true")
    if 'DeclareLaunchArgument("enable_sliding_window_optimizer", default_value="true")' not in tracking_launch_text:
        errors.append("tracking.launch.py must default production sliding-window BA to true")
    if 'DeclareLaunchArgument("enable_visual_alignment_window_factor", default_value="true")' not in tracking_launch_text:
        errors.append("tracking.launch.py must default visual alignment window factors to true")
    if 'declare_parameter<double>("visual_alignment_huber_delta_m", 0.05)' not in tracking_node_text:
        errors.append("tracking_node must default visual alignment Huber delta to 0.05m")
    if 'DeclareLaunchArgument("visual_alignment_huber_delta_m", default_value="0.05")' not in tracking_launch_text:
        errors.append("tracking.launch.py must expose visual alignment Huber delta")
    if 'DeclareLaunchArgument("enable_se3_photometric_window_factor", default_value="true")' not in tracking_launch_text:
        errors.append("tracking.launch.py must default SE3 photometric window factors to true")
    if 'declare_parameter<int>("depth_frame_cache_size", 8)' not in tracking_node_text:
        errors.append("tracking_node must default the visual depth-frame cache size to 8")
    if 'DeclareLaunchArgument("depth_frame_cache_size", default_value="8")' not in tracking_launch_text:
        errors.append("tracking.launch.py must default the visual depth-frame cache size to 8")
    if 'declare_parameter<int>("rendered_frame_cache_size", 8)' not in tracking_node_text:
        errors.append("tracking_node must default the visual rendered-frame cache size to 8")
    if 'DeclareLaunchArgument("rendered_frame_cache_size", default_value="8")' not in tracking_launch_text:
        errors.append("tracking.launch.py must default the visual rendered-frame cache size to 8")
    if 'declare_parameter<bool>("enable_sliding_window_smoothness_factor", true)' not in tracking_node_text:
        errors.append("tracking_node must default trajectory smoothness BA factors to true")
    if 'DeclareLaunchArgument("enable_sliding_window_smoothness_factor", default_value="true")' not in tracking_launch_text:
        errors.append("tracking.launch.py must default trajectory smoothness BA factors to true")
    if "run_serialized_callback" not in tracking_node_text or "std::scoped_lock<std::mutex>" not in tracking_node_text:
        errors.append("tracking_node callbacks must pass through the serialization guard")
    for field in [
        "signed_nanosecond_time_math_enabled",
        "last_image_stamp_ns",
        "last_pointcloud_stamp_ns",
        "last_imu_stamp_ns",
        "sliding_window_imu_reanchors",
        "sliding_window_smoothness_factors",
        "trajectory_control_poses",
        "trajectory_deskew_queries",
        "trajectory_deskew_hits",
    ]:
        if field not in tracking_status_msg_text or field not in tracking_node_text:
            errors.append(f"tracking status must publish signed-nanosecond field {field}")

    for path in sorted(CONFIG_DIR.glob("*.yaml")):
        params = mapping_params(path)
        if params.get("sensor_qos_reliability") != "best_effort":
            errors.append(f"{path.name}: sensor_qos_reliability must default to best_effort")
        if params.get("sensor_qos_history") != "keep_last":
            errors.append(f"{path.name}: sensor_qos_history must default to keep_last")
        if params.get("sensor_qos_depth") != 5:
            errors.append(f"{path.name}: sensor_qos_depth must default to 5")

    for path in source_files():
        text = read(path)
        if "rclcpp::MultiThreadedExecutor" in text:
            errors.append(f"{path.relative_to(ROOT)} uses MultiThreadedExecutor in strict estimator code")
        if "lookupTransform" in text:
            errors.append(f"{path.relative_to(ROOT)} consumes tf2 lookupTransform without a semantic wrapper")

    if errors:
        print("[ros2-semantics] FAIL", file=sys.stderr)
        for error in errors:
            print(f"  - {error}", file=sys.stderr)
        return 1

    print("[ros2-semantics] OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
