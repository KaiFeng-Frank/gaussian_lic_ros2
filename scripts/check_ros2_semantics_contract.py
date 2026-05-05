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
    if "valid_camera_info_intrinsics" not in mapping_text or "std::isfinite(msg.k[2])" not in mapping_text:
        errors.append("mapping_node must reject non-finite CameraInfo intrinsics at the input boundary")
    nanosecond_guard = "stamp.nanosec >= static_cast<uint32_t>(kNanosecondsPerSecond)"
    if nanosecond_guard not in mapping_text:
        errors.append("mapping_node stamp_to_nsec must reject ROS2 stamps with nanosec >= 1e9")
    for suffix in ("_qos_reliability", "_qos_history", "_qos_depth"):
        if f"prefix + \"{suffix}\"" not in mapping_text:
            errors.append(f"mapping_node declare_topic_qos must expose per-stream {suffix}")
    for stream in ("pointcloud", "pose", "image", "camera_info", "depth", "imu"):
        if f'declare_topic_qos("{stream}")' not in mapping_text:
            errors.append(f"mapping_node must declare per-stream QoS for {stream}")

    if "stamp_to_sec" in frontend_text:
        errors.append("lic2_contract_adapter still exposes stamp_to_sec; use int64 nanoseconds")
    if "last_imu_stamp_sec_" in frontend_text:
        errors.append("lic2_contract_adapter IMU integration still stores double-second stamps")
    if "last_imu_stamp_nsec_" not in frontend_text:
        errors.append("lic2_contract_adapter does not store IMU stamps in nanoseconds")
    if nanosecond_guard not in frontend_text:
        errors.append("lic2_contract_adapter stamp_to_nsec must reject ROS2 stamps with nanosec >= 1e9")
    for suffix in ("_qos_reliability", "_qos_history", "_qos_depth"):
        if f"prefix + \"{suffix}\"" not in frontend_text:
            errors.append(f"lic2_contract_adapter declare_topic_qos must expose per-stream {suffix}")
    for stream in (
        "raw_image",
        "raw_camera_info",
        "raw_depth",
        "raw_pointcloud",
        "raw_imu",
        "pose_stamped",
        "raw_odometry",
        "image",
        "camera_info",
        "depth",
        "pointcloud",
        "pose",
        "imu",
        "frontend_odometry",
    ):
        if f'declare_topic_qos("{stream}")' not in frontend_text:
            errors.append(f"lic2_contract_adapter must declare per-stream QoS for {stream}")

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
        "sensor_qos_history",
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
        "sliding_window_max_rotation_step_rad",
        "sliding_window_max_translation_step_m",
        "sliding_window_max_velocity_step_mps",
        "sliding_window_max_bias_step",
        "sliding_window_max_normal_equation_condition",
        "sliding_window_min_normal_equation_rank_ratio",
        "sliding_window_imu_weight",
        "sliding_window_imu_rotation_weight",
        "sliding_window_imu_velocity_weight",
        "sliding_window_imu_position_weight",
        "sliding_window_pose_translation_weight",
        "sliding_window_pose_rotation_weight",
        "enable_sliding_window_smoothness_factor",
        "se3_photometric_factor_huber_delta",
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

    required_tracking_qos_streams = [
        "raw_image",
        "raw_camera_info",
        "raw_depth",
        "raw_pointcloud",
        "raw_imu",
        "image",
        "camera_info",
        "depth",
        "pointcloud",
        "pose",
        "frontend_odometry",
    ]
    if "qos_launch_configs" not in tracking_launch_text or "**qos_launch_configs" not in tracking_launch_text:
        errors.append("tracking.launch.py must pass per-stream tracking QoS overrides into tracking_node")
    for stream in required_tracking_qos_streams:
        if f'"{stream}"' not in tracking_launch_text:
            errors.append(f"tracking.launch.py must include {stream} in per-stream QoS launch overrides")
        if f'declare_topic_qos("{stream}")' not in tracking_node_text:
            errors.append(f"tracking_node must declare {stream} per-stream QoS parameters")
        if f'make_sensor_qos("{stream}"' not in tracking_node_text:
            errors.append(f"tracking_node must apply {stream} per-stream QoS")

    if 'declare_parameter<bool>("serialize_callbacks", true)' not in tracking_node_text:
        errors.append("tracking_node must default serialize_callbacks to true")
    if (
        'declare_parameter<std::string>("sensor_qos_history", "keep_last")' not in tracking_node_text
        or "sensor_qos_history" not in tracking_status_msg_text
    ):
        errors.append("tracking_node must expose and publish sensor_qos_history")
    if "valid_camera_info_intrinsics" not in tracking_node_text or "std::isfinite(msg.k[2])" not in tracking_node_text:
        errors.append("tracking_node must reject non-finite CameraInfo intrinsics at the input boundary")
    if "vector3_from_parameter" not in tracking_node_text or "quaternion_from_rpy_parameter" not in tracking_node_text:
        errors.append("tracking_node must validate finite LiDAR/camera extrinsic parameters")
    if "finite_nonnegative_parameter" not in tracking_node_text or 'vector3_from_parameter("imu_gravity_w"' not in tracking_node_text:
        errors.append("tracking_node must validate finite IMU gravity and LiDAR keyframe thresholds")
    if (
        "integer_parameter_at_least" not in tracking_node_text
        or "finite_positive_parameter" not in tracking_node_text
        or "finite_unit_interval_parameter" not in tracking_node_text
        or "se3_photometric_max_samples must be >= se3_photometric_min_samples" not in tracking_node_text
        or "se3_photometric_max_depth_m must be greater than se3_photometric_min_depth_m" not in tracking_node_text
    ):
        errors.append("tracking_node must hard-validate tracking window, LiDAR, visual, and BA numeric parameter ranges")
    if 'declare_parameter<bool>("enable_sliding_window_optimizer", true)' not in tracking_node_text:
        errors.append("tracking_node must default production sliding-window BA to true")
    if 'DeclareLaunchArgument("enable_sliding_window_optimizer", default_value="true")' not in tracking_launch_text:
        errors.append("tracking.launch.py must default production sliding-window BA to true")
    for needle, message in [
        ('declare_parameter<double>("sliding_window_max_rotation_step_rad", 0.5)', "tracking_node must default rotation BA step limit to 0.5 rad"),
        ('declare_parameter<double>("sliding_window_max_translation_step_m", 1.0)', "tracking_node must default translation BA step limit to 1.0m"),
        ('declare_parameter<double>("sliding_window_max_velocity_step_mps", 5.0)', "tracking_node must default velocity BA step limit to 5.0m/s"),
        ('declare_parameter<double>("sliding_window_max_bias_step", 1.0)', "tracking_node must default bias BA step limit to 1.0"),
        ('declare_parameter<double>("sliding_window_max_normal_equation_condition", 1.0e12)', "tracking_node must default normal-equation condition guard to 1e12"),
        ('declare_parameter<double>("sliding_window_min_normal_equation_rank_ratio", 0.0)', "tracking_node must default normal-equation rank-ratio guard to 0.0"),
        ('declare_parameter<double>("sliding_window_imu_rotation_weight", 1.0)', "tracking_node must expose IMU rotation residual weight"),
        ('declare_parameter<double>("sliding_window_imu_velocity_weight", 1.0)', "tracking_node must expose IMU velocity residual weight"),
        ('declare_parameter<double>("sliding_window_imu_position_weight", 1.0)', "tracking_node must expose IMU position residual weight"),
        ('DeclareLaunchArgument("sliding_window_max_rotation_step_rad", default_value="0.5")', "tracking.launch.py must expose rotation BA step limit"),
        ('DeclareLaunchArgument("sliding_window_max_translation_step_m", default_value="1.0")', "tracking.launch.py must expose translation BA step limit"),
        ('DeclareLaunchArgument("sliding_window_max_velocity_step_mps", default_value="5.0")', "tracking.launch.py must expose velocity BA step limit"),
        ('DeclareLaunchArgument("sliding_window_max_bias_step", default_value="1.0")', "tracking.launch.py must expose bias BA step limit"),
        ('DeclareLaunchArgument("sliding_window_max_normal_equation_condition", default_value="1000000000000.0")', "tracking.launch.py must expose normal-equation condition guard"),
        ('DeclareLaunchArgument("sliding_window_min_normal_equation_rank_ratio", default_value="0.0")', "tracking.launch.py must expose normal-equation rank-ratio guard"),
        ('DeclareLaunchArgument("sliding_window_imu_rotation_weight", default_value="1.0")', "tracking.launch.py must expose IMU rotation residual weight"),
        ('DeclareLaunchArgument("sliding_window_imu_velocity_weight", default_value="1.0")', "tracking.launch.py must expose IMU velocity residual weight"),
        ('DeclareLaunchArgument("sliding_window_imu_position_weight", default_value="1.0")', "tracking.launch.py must expose IMU position residual weight"),
    ]:
        if needle not in tracking_node_text and needle not in tracking_launch_text:
            errors.append(message)
    if 'DeclareLaunchArgument("enable_visual_alignment_window_factor", default_value="true")' not in tracking_launch_text:
        errors.append("tracking.launch.py must default visual alignment window factors to true")
    if 'declare_parameter<double>("visual_alignment_huber_delta_m", 0.05)' not in tracking_node_text:
        errors.append("tracking_node must default visual alignment Huber delta to 0.05m")
    if 'DeclareLaunchArgument("visual_alignment_huber_delta_m", default_value="0.05")' not in tracking_launch_text:
        errors.append("tracking.launch.py must expose visual alignment Huber delta")
    if 'DeclareLaunchArgument("enable_se3_photometric_window_factor", default_value="true")' not in tracking_launch_text:
        errors.append("tracking.launch.py must default SE3 photometric window factors to true")
    if 'declare_parameter<double>("se3_photometric_factor_huber_delta", 1.0)' not in tracking_node_text:
        errors.append("tracking_node must default SE3 photometric factor Huber delta to 1.0")
    if 'DeclareLaunchArgument("se3_photometric_factor_huber_delta", default_value="1.0")' not in tracking_launch_text:
        errors.append("tracking.launch.py must expose SE3 photometric factor Huber delta")
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
    if "accept_stream_stamp" not in tracking_node_text or "non-monotonic stamp" not in tracking_node_text:
        errors.append("tracking_node must reject non-monotonic raw stream stamps before estimator mutation")
    if "imu_invalid_measurements_" not in tracking_node_text or "non-finite angular velocity" not in tracking_node_text:
        errors.append("tracking_node must reject non-finite IMU measurements before propagation")
    for field in [
        "signed_nanosecond_time_math_enabled",
        "last_image_stamp_ns",
        "last_pointcloud_stamp_ns",
        "last_imu_stamp_ns",
        "image_stamp_regressions",
        "depth_stamp_regressions",
        "rendered_stamp_regressions",
        "pointcloud_stamp_regressions",
        "imu_stamp_regressions",
        "imu_invalid_measurements",
        "sliding_window_imu_reanchors",
        "sliding_window_smoothness_factors",
        "sliding_window_accepted_steps",
        "sliding_window_rejected_steps",
        "sliding_window_limited_steps",
        "sliding_window_point_factor_skip_count",
        "sliding_window_plane_factor_skip_count",
        "sliding_window_visual_factor_skip_count",
        "sliding_window_se3_photometric_factor_skip_count",
        "sliding_window_smoothness_factor_skip_count",
        "sliding_window_imu_factor_skip_count",
        "sliding_window_optimization_skip_count",
        "sliding_window_normal_equation_rows",
        "sliding_window_normal_equation_cols",
        "sliding_window_normal_equation_rank",
        "sliding_window_normal_equation_min_singular_value",
        "sliding_window_normal_equation_max_singular_value",
        "sliding_window_normal_equation_condition_number",
        "sliding_window_normal_equation_degenerate",
        "sliding_window_last_step_norm",
        "sliding_window_last_step_scale",
        "sliding_window_last_damping",
        "trajectory_control_poses",
        "trajectory_deskew_queries",
        "trajectory_deskew_hits",
        "visual_rendered_cache_size",
        "visual_rendered_match_delta_ns",
        "visual_depth_cache_size",
        "visual_depth_match_delta_ns",
        "visual_alignment_pending_stale_drops",
        "visual_se3_photometric_pending_stale_drops",
        "visual_se3_photometric_rejected_depth",
        "visual_se3_photometric_rejected_gradient",
        "visual_se3_photometric_rejected_residual",
    ]:
        if field not in tracking_status_msg_text or field not in tracking_node_text:
            errors.append(f"tracking status must publish frontend health field {field}")

    for path in sorted(CONFIG_DIR.glob("*.yaml")):
        params = mapping_params(path)
        if params.get("sensor_qos_reliability") != "best_effort":
            errors.append(f"{path.name}: sensor_qos_reliability must default to best_effort")
        if params.get("sensor_qos_history") != "keep_last":
            errors.append(f"{path.name}: sensor_qos_history must default to keep_last")
        if params.get("sensor_qos_depth") != 5:
            errors.append(f"{path.name}: sensor_qos_depth must default to 5")
        for stream in ("pointcloud", "pose", "image", "camera_info", "depth", "imu"):
            if params.get(f"{stream}_qos_reliability") != "best_effort":
                errors.append(f"{path.name}: {stream}_qos_reliability must default to best_effort")
            if params.get(f"{stream}_qos_history") != "keep_last":
                errors.append(f"{path.name}: {stream}_qos_history must default to keep_last")
            if params.get(f"{stream}_qos_depth") != 5:
                errors.append(f"{path.name}: {stream}_qos_depth must default to 5")

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
