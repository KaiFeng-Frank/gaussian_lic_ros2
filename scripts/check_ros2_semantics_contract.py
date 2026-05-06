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
SLIDING_WINDOW_OPTIMIZER = ROOT / "src" / "gaussian_lic_tracking" / "src" / "sliding_window_optimizer.cpp"
SLIDING_WINDOW_HEADER = ROOT / "src" / "gaussian_lic_tracking" / "include" / "gaussian_lic_tracking" / "sliding_window_optimizer.hpp"
IMU_PREINTEGRATOR = ROOT / "src" / "gaussian_lic_tracking" / "src" / "imu_preintegrator.cpp"
TRACKING_STATUS_MSG = ROOT / "src" / "gaussian_lic_msgs" / "msg" / "TrackingStatus.msg"
NATIVE_TRACKING_REPORT = ROOT / "scripts" / "run_native_tracking_bag_report.sh"
SYNTHETIC_GS_FRAME_PUB = (
    ROOT / "src" / "gaussian_lic_tools" / "gaussian_lic_tools" / "synthetic_gs_frame_pub.py"
)
TRACKING_SMOKE_TEST = ROOT / "scripts" / "tracking_smoke_test.sh"
SEMANTICS_DOC = ROOT / "docs" / "ROS2_SEMANTICS.md"
TIMING_AUDIT = ROOT / "scripts" / "rosbag2_timing_audit.py"
CI_WORKFLOW = ROOT / ".github" / "workflows" / "ci.yaml"


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
    if not CI_WORKFLOW.is_file():
        errors.append(".github/workflows/ci.yaml is missing")

    mapping_text = read(MAPPING_NODE)
    frontend_text = read(FRONTEND_ADAPTER)
    launch_text = read(RUN_BAG_LAUNCH)
    tracking_launch_text = read(TRACKING_LAUNCH)
    tracking_node_text = read(TRACKING_NODE)
    sliding_window_text = read(SLIDING_WINDOW_OPTIMIZER)
    sliding_window_header_text = read(SLIDING_WINDOW_HEADER)
    imu_preintegrator_text = read(IMU_PREINTEGRATOR)
    tracking_status_msg_text = read(TRACKING_STATUS_MSG)
    native_tracking_report_text = read(NATIVE_TRACKING_REPORT)
    synthetic_pub_text = read(SYNTHETIC_GS_FRAME_PUB)
    tracking_smoke_text = read(TRACKING_SMOKE_TEST)
    timing_audit_text = read(TIMING_AUDIT)
    ci_workflow_text = read(CI_WORKFLOW)

    if "humble" in ci_workflow_text.lower():
        errors.append("CI workflow must stay Jazzy-only; do not add ROS2 Humble to the build matrix")
    if "ros_distro: [jazzy]" not in ci_workflow_text:
        errors.append("CI workflow build matrix must be exactly ros_distro: [jazzy]")

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
    if "header_stamp_regressions" not in timing_audit_text or "rosbag2_py.SequentialReader" not in timing_audit_text:
        errors.append("rosbag2_timing_audit.py must inspect MCAP/header stamp order when ROS2 readers are available")
    if "gaussian_lic_frontend_raw_visual_timing_audit" not in read(ROOT / "scripts" / "verify_workspace.sh"):
        errors.append("verify_workspace.sh must audit frontend visual rosbag header stamp order")

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
        "visual_depth_max_dt_ns",
        "depth_frame_cache_size",
        "sparse_lidar_depth_dilation_px",
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
        "se3_photometric_min_hessian_rank",
        "se3_photometric_max_hessian_condition",
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
        ('declare_parameter<double>("sliding_window_max_feedback_translation_m", 1.0)', "tracking_node must default translation BA feedback limit to 1.0m"),
        ('declare_parameter<double>("sliding_window_max_feedback_rotation_rad", 0.5)', "tracking_node must default rotation BA feedback limit to 0.5rad"),
        ('declare_parameter<double>("sliding_window_max_feedback_velocity_mps", 5.0)', "tracking_node must default velocity BA feedback limit to 5.0m/s"),
        ('declare_parameter<double>("sliding_window_max_normal_equation_condition", 1.0e13)', "tracking_node must default normal-equation condition guard to 1e13"),
        ('declare_parameter<double>("sliding_window_min_normal_equation_rank_ratio", 0.8)', "tracking_node must default normal-equation rank-ratio guard to 0.8"),
        ('declare_parameter<double>("sliding_window_imu_rotation_weight", 1.0)', "tracking_node must expose IMU rotation residual weight"),
        ('declare_parameter<double>("sliding_window_imu_velocity_weight", 1.0)', "tracking_node must expose IMU velocity residual weight"),
        ('declare_parameter<double>("sliding_window_imu_position_weight", 1.0)', "tracking_node must expose IMU position residual weight"),
        ('DeclareLaunchArgument("sliding_window_max_rotation_step_rad", default_value="0.5")', "tracking.launch.py must expose rotation BA step limit"),
        ('DeclareLaunchArgument("sliding_window_max_translation_step_m", default_value="1.0")', "tracking.launch.py must expose translation BA step limit"),
        ('DeclareLaunchArgument("sliding_window_max_velocity_step_mps", default_value="5.0")', "tracking.launch.py must expose velocity BA step limit"),
        ('DeclareLaunchArgument("sliding_window_max_bias_step", default_value="1.0")', "tracking.launch.py must expose bias BA step limit"),
        ('DeclareLaunchArgument("sliding_window_max_feedback_translation_m", default_value="1.0")', "tracking.launch.py must expose translation BA feedback limit"),
        ('DeclareLaunchArgument("sliding_window_max_feedback_rotation_rad", default_value="0.5")', "tracking.launch.py must expose rotation BA feedback limit"),
        ('DeclareLaunchArgument("sliding_window_max_feedback_velocity_mps", default_value="5.0")', "tracking.launch.py must expose velocity BA feedback limit"),
        ('DeclareLaunchArgument("sliding_window_max_normal_equation_condition", default_value="10000000000000.0")', "tracking.launch.py must expose normal-equation condition guard"),
        ('DeclareLaunchArgument("sliding_window_min_normal_equation_rank_ratio", default_value="0.8")', "tracking.launch.py must expose normal-equation rank-ratio guard"),
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
    if 'declare_parameter<int64_t>("visual_depth_max_dt_ns", 0LL)' not in tracking_node_text:
        errors.append("tracking_node must expose a depth-specific visual freshness window")
    if 'DeclareLaunchArgument("visual_depth_max_dt_ns", default_value="0")' not in tracking_launch_text:
        errors.append("tracking.launch.py must expose the depth-specific visual freshness window")
    if "max_visual_depth_delta_ns()" not in tracking_node_text:
        errors.append("tracking_node must use the depth-specific visual freshness window for depth selection")
    if 'declare_parameter<double>("se3_photometric_factor_huber_delta", 1.0)' not in tracking_node_text:
        errors.append("tracking_node must default SE3 photometric factor Huber delta to 1.0")
    if 'DeclareLaunchArgument("se3_photometric_factor_huber_delta", default_value="1.0")' not in tracking_launch_text:
        errors.append("tracking.launch.py must expose SE3 photometric factor Huber delta")
    if 'declare_parameter<int>("se3_photometric_min_hessian_rank", 3)' not in tracking_node_text:
        errors.append("tracking_node must default the SE3 photometric Hessian rank gate to 3")
    if 'DeclareLaunchArgument("se3_photometric_min_hessian_rank", default_value="3")' not in tracking_launch_text:
        errors.append("tracking.launch.py must expose the SE3 photometric Hessian rank gate")
    if 'declare_parameter<double>("se3_photometric_max_hessian_condition", 1.0e12)' not in tracking_node_text:
        errors.append("tracking_node must default the SE3 photometric Hessian condition gate to 1e12")
    if (
        "se3_photometric_max_hessian_condition" not in tracking_launch_text
        or 'default_value="1000000000000.0"' not in tracking_launch_text
    ):
        errors.append("tracking.launch.py must expose the SE3 photometric Hessian condition gate")
    if "se3_photometric_hessian_is_healthy" not in tracking_node_text:
        errors.append("tracking_node must reject degenerate SE3 photometric Hessians before BA")
    if 'declare_parameter<int>("depth_frame_cache_size", 8)' not in tracking_node_text:
        errors.append("tracking_node must default the visual depth-frame cache size to 8")
    if 'DeclareLaunchArgument("depth_frame_cache_size", default_value="8")' not in tracking_launch_text:
        errors.append("tracking.launch.py must default the visual depth-frame cache size to 8")
    if 'declare_parameter<int>("sparse_lidar_depth_dilation_px", 1)' not in tracking_node_text:
        errors.append("tracking_node must default sparse LiDAR depth dilation to 1px")
    if 'DeclareLaunchArgument("sparse_lidar_depth_dilation_px", default_value="1")' not in tracking_launch_text:
        errors.append("tracking.launch.py must expose sparse LiDAR depth dilation")
    if "visual_depth_dilation_px" not in tracking_status_msg_text or \
            "status.visual_depth_dilation_px" not in tracking_node_text:
        errors.append("TrackingStatus must publish the sparse LiDAR depth dilation radius")
    for field_name in (
        "visual_se3_photometric_total_batches",
        "visual_se3_photometric_valid_batches",
        "visual_se3_photometric_insufficient_sample_batches",
        "visual_se3_photometric_degenerate_batches",
        "visual_se3_photometric_total_candidates",
        "visual_se3_photometric_total_samples",
        "visual_se3_photometric_hessian_rank",
        "visual_se3_photometric_hessian_condition_number",
        "visual_se3_photometric_last_accepted_hessian_rank",
        "visual_se3_photometric_last_accepted_hessian_condition_number",
    ):
        if field_name not in tracking_status_msg_text or f"status.{field_name}" not in tracking_node_text:
            errors.append(f"TrackingStatus must publish cumulative SE3 photometric diagnostics: {field_name}")
    if 'declare_parameter<int>("rendered_frame_cache_size", 8)' not in tracking_node_text:
        errors.append("tracking_node must default the visual rendered-frame cache size to 8")
    if 'DeclareLaunchArgument("rendered_frame_cache_size", default_value="8")' not in tracking_launch_text:
        errors.append("tracking.launch.py must default the visual rendered-frame cache size to 8")
    if 'declare_parameter<int>("observed_frame_cache_size", 64)' not in tracking_node_text:
        errors.append("tracking_node must keep a delayed observed-image cache for async mapper feedback")
    if 'DeclareLaunchArgument("observed_frame_cache_size", default_value="64")' not in tracking_launch_text:
        errors.append("tracking.launch.py must expose the observed-image cache size")
    if 'declare_parameter<int>("visual_pending_factor_queue_size", 64)' not in tracking_node_text:
        errors.append("tracking_node must queue delayed visual factors instead of keeping a single pending slot")
    if 'DeclareLaunchArgument("visual_pending_factor_queue_size", default_value="64")' not in tracking_launch_text:
        errors.append("tracking.launch.py must expose the visual pending factor queue size")
    if "select_visual_factor_reference" not in tracking_node_text:
        errors.append("tracking_node must attach delayed visual factors to active window states")
    if "observed_frame_cache_size:=128" not in native_tracking_report_text:
        errors.append("native tracking real-bag report must enlarge the observed-frame cache")
    if "VISUAL_DEPTH_FRAME_CACHE_SIZE=64" not in native_tracking_report_text or \
            'depth_frame_cache_size:="${VISUAL_DEPTH_FRAME_CACHE_SIZE}"' not in native_tracking_report_text:
        errors.append("native tracking real-bag report must enlarge the visual depth-frame cache")
    if "VISUAL_PENDING_FACTOR_QUEUE_SIZE=128" not in native_tracking_report_text or \
            'visual_pending_factor_queue_size:="${VISUAL_PENDING_FACTOR_QUEUE_SIZE}"' not in native_tracking_report_text:
        errors.append("native tracking real-bag report must enlarge the visual pending-factor queue")
    if "VISUAL_FACTOR_MAX_DT_NS=300000000" not in native_tracking_report_text or \
            'visual_factor_max_dt_ns:="${VISUAL_FACTOR_MAX_DT_NS}"' not in native_tracking_report_text or \
            '"visual_factor_max_dt_ns": visual_factor_max_dt_ns' not in native_tracking_report_text:
        errors.append("native tracking real-bag report must widen and record the visual BA pairing window")
    if "VISUAL_DEPTH_MAX_DT_NS=0" not in native_tracking_report_text or \
            'visual_depth_max_dt_ns:="${VISUAL_DEPTH_MAX_DT_NS}"' not in native_tracking_report_text or \
            '"visual_depth_max_dt_ns": visual_depth_max_dt_ns' not in native_tracking_report_text:
        errors.append("native tracking real-bag report must expose and record the sparse-depth visual BA pairing window")
    if "VISUAL_DEPTH_DILATION_PX=5" not in native_tracking_report_text or \
            'sparse_lidar_depth_dilation_px:="${VISUAL_DEPTH_DILATION_PX}"' not in native_tracking_report_text or \
            '"visual_depth_dilation_px": visual_depth_dilation_px' not in native_tracking_report_text:
        errors.append("native tracking real-bag report must enable and record sparse LiDAR depth dilation")
    if "SE3_PHOTOMETRIC_MIN_SAMPLES=8" not in native_tracking_report_text or \
            'se3_photometric_min_samples:="${SE3_PHOTOMETRIC_MIN_SAMPLES}"' not in native_tracking_report_text or \
            '"se3_photometric_min_samples": se3_photometric_min_samples' not in native_tracking_report_text:
        errors.append("native tracking real-bag report must tighten and record the sparse-depth SE3 sample gate")
    if "SE3_PHOTOMETRIC_MIN_HESSIAN_RANK=3" not in native_tracking_report_text or \
            'se3_photometric_min_hessian_rank:="${SE3_PHOTOMETRIC_MIN_HESSIAN_RANK}"' not in native_tracking_report_text or \
            '"se3_photometric_min_hessian_rank": se3_photometric_min_hessian_rank' not in native_tracking_report_text:
        errors.append("native tracking real-bag report must enforce and record the SE3 Hessian rank gate")
    if "SE3_PHOTOMETRIC_MAX_HESSIAN_CONDITION=1000000000000.0" not in native_tracking_report_text or \
            'se3_photometric_max_hessian_condition:="${SE3_PHOTOMETRIC_MAX_HESSIAN_CONDITION}"' not in native_tracking_report_text or \
            '"se3_photometric_max_hessian_condition": se3_photometric_max_hessian_condition' not in native_tracking_report_text:
        errors.append("native tracking real-bag report must enforce and record the SE3 Hessian condition gate")
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
    if "lidar_invalid_points_" not in tracking_node_text or "invalid_point_times" not in tracking_node_text:
        errors.append("tracking_node must publish LiDAR invalid point and point-time counters")
    if "lidar_invalid_frames_" not in tracking_node_text:
        errors.append("tracking_node must publish malformed LiDAR frame counters")
    if "lidar_out_of_range_point_times_" not in tracking_node_text or "lidar_max_abs_point_time_offset_s" not in tracking_node_text:
        errors.append("tracking_node must gate out-of-range LiDAR per-point time offsets")
    if '"scan_order"' not in tracking_node_text or "lidar_scan_order_duration_s" not in tracking_node_text:
        errors.append("tracking_node must expose explicit scan-order LiDAR deskew fallback")
    if 'DeclareLaunchArgument("lidar_scan_order_duration_s", default_value="0.1")' not in tracking_launch_text:
        errors.append("tracking.launch.py must expose scan-order LiDAR deskew duration")
    if "--enable-scan-order-deskew" not in native_tracking_report_text or "--require-deskew" not in native_tracking_report_text:
        errors.append("native tracking real-bag report must expose explicit scan-order deskew gates")
    if "camera_info_invalid_intrinsics_" not in tracking_node_text or "image_invalid_frames_" not in tracking_node_text:
        errors.append("tracking_node must publish invalid camera/image/depth/rendered-frame counters")
    if "sliding_window_invalid_optimized_states_" not in tracking_node_text or "valid_sliding_window_state" not in tracking_node_text:
        errors.append("tracking_node must reject invalid optimized sliding-window states before feedback")
    if "sliding_window_feedback_update_count_" not in tracking_node_text or "last_sliding_window_feedback_stamp_ns_" not in tracking_node_text:
        errors.append("tracking_node must publish accepted sliding-window feedback health")
    if "invalid_candidate_steps" not in sliding_window_text or "states_are_finite(candidate_states)" not in sliding_window_text:
        errors.append("sliding_window_optimizer must explicitly reject invalid candidate states")
    if "linearization_failure_count" not in sliding_window_text or "linear_solve_failure_count" not in sliding_window_text:
        errors.append("sliding_window_optimizer must publish linearization/linear-solve failure counters")
    if "sample.weight > 1.0" not in read(ROOT / "src" / "gaussian_lic_tracking" / "src" / "visual_factor.cpp"):
        errors.append("visual SE3 photometric sample weights must be bounded to (0, 1]")
    if "trajectory_control_pose_skip_count_" not in tracking_node_text:
        errors.append("tracking_node must publish trajectory-control pose rejection counters")
    if "last_sliding_window_imu_preintegration_samples_" not in tracking_node_text:
        errors.append("tracking_node must publish last consumed IMU preintegration block health")
    if "sliding_window_imu_max_extrapolation_s" not in tracking_node_text or "preintegration span must match factor timestamps" not in sliding_window_text:
        errors.append("tracking_node/optimizer must gate IMU preintegration span against factor timestamps")
    if "sample.stamp_ns == output.end_stamp_ns_" not in read(ROOT / "src" / "gaussian_lic_tracking" / "src" / "imu_preintegrator.cpp"):
        errors.append("IMU preintegrator must preserve an auto-start sample during bias re-integration")
    if "orphan_factor_count" not in sliding_window_text or "count_orphan_factors" not in sliding_window_text:
        errors.append("sliding_window_optimizer must expose and gate orphan factor references")
    if "schur_marginalization_count_" not in sliding_window_text or "fallback_marginalization_prior_count_" not in sliding_window_text:
        errors.append("sliding_window_optimizer must publish Schur/fallback marginalization health counters")
    if "max_state_gap_s" not in sliding_window_text or "state_gap_degenerate" not in sliding_window_text:
        errors.append("sliding_window_optimizer must expose and gate oversized state time gaps")
    if "dense prior stamps must match reference-state stamps" not in sliding_window_text or "dense prior stamps must be strictly increasing" not in sliding_window_text:
        errors.append("sliding_window_optimizer must validate dense-prior stamp/reference consistency")
    if "*existing = normalized" not in sliding_window_text:
        errors.append("sliding_window_optimizer must replace same-stamp pose/state priors instead of accumulating duplicates")
    if "candidate.from_stamp_ns == factor.from_stamp_ns" not in sliding_window_text or "candidate.previous_stamp_ns == factor.previous_stamp_ns" not in sliding_window_text:
        errors.append("sliding_window_optimizer must replace duplicate IMU spans and smoothness triplets instead of accumulating residual weight")
    if "source_id{0}" not in sliding_window_header_text or "candidate.source_id == factor.source_id" not in sliding_window_text:
        errors.append("sliding_window_optimizer must distinguish same-stamp factor sources before replacement")
    if "imu_factor_replacement_count_" not in sliding_window_text or "sliding_window_smoothness_factor_replacement_count" not in tracking_node_text:
        errors.append("tracking status must publish duplicate IMU/smoothness replacement counters")
    if "so3_left_jacobian_inverse" not in sliding_window_text:
        errors.append("smoothness rotation rows must use closed-form SO(3) Jacobian blocks")
    if "smoothness_rotation_jacobian" in sliding_window_text and "const double epsilon)" in sliding_window_text:
        errors.append("smoothness rotation rows must not depend on local finite-difference epsilon")
    if "return relative_rotation_vector(measured_q, predicted_q);" not in sliding_window_text:
        errors.append("sliding-window rotation priors must use SO(3) log-map residuals")
    if "rotation_residual_left_perturbation_jacobian" not in sliding_window_text:
        errors.append("sliding-window rotation priors must use SO(3) left-Jacobian inverse blocks")
    if "quaternion_log_vector_autodiff(error)" not in sliding_window_text:
        errors.append("IMU bias Jacobian residual must use SO(3) log-map AutoDiff, not 2*quaternion-vector")
    if "effective_imu_sqrt_information" not in sliding_window_text or "IMU factor sqrt-information must be finite" not in sliding_window_text:
        errors.append("IMU preintegration factors must support finite full 9x9 sqrt-information whitening")
    if "return quaternion_log_vector(" not in imu_preintegrator_text:
        errors.append("IMU preintegration rotation residual must use SO(3) log-map residuals")
    if 'declare_parameter("imu_samples_per_frame", 3)' not in synthetic_pub_text:
        errors.append("synthetic_gs_frame_pub must publish at least three IMU samples per frame by default")
    if 'declare_parameter("imu_stamp_lead_ns", 10000000)' not in synthetic_pub_text:
        errors.append("synthetic_gs_frame_pub must keep IMU samples before pose stamps by default")
    if "last_frame_stamp_ns" not in synthetic_pub_text or "last_imu_stamp_ns" not in synthetic_pub_text:
        errors.append("synthetic_gs_frame_pub must keep monotonic IMU/frame stamp state")
    if "start_ns = self.last_frame_stamp_ns + lead_ns" not in synthetic_pub_text:
        errors.append("synthetic_gs_frame_pub must place IMU samples inside the previous/current frame interval")
    if 'run_id="${ROS_DOMAIN_ID}_${BASHPID}"' not in tracking_smoke_text:
        errors.append("tracking_smoke_test must isolate status/log files by ROS_DOMAIN_ID and BASHPID")
    if 'status_file="/tmp/gaussian_lic_tracking_smoke_status_${run_id}.txt"' not in tracking_smoke_text:
        errors.append("tracking_smoke_test must use a per-run status file")
    if 'legacy_status_file=/tmp/gaussian_lic_tracking_smoke_status.txt' not in tracking_smoke_text:
        errors.append("tracking_smoke_test must preserve the legacy status file for manual inspection")
    if 'DeclareLaunchArgument("sliding_window_max_state_gap_s", default_value="1.0")' not in tracking_launch_text:
        errors.append("tracking.launch.py must expose the sliding-window max state gap")
    if 'DeclareLaunchArgument("sliding_window_imu_max_extrapolation_s", default_value="0.02")' not in tracking_launch_text:
        errors.append("tracking.launch.py must expose bounded IMU preintegration extrapolation")
    if 'DeclareLaunchArgument("enable_pointcloud_imu_wait", default_value="true")' not in tracking_launch_text:
        errors.append("tracking.launch.py must expose default-enabled pointcloud/IMU wait queue")
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
        "external_odometry_prior_stamp_regressions",
        "imu_invalid_measurements",
        "external_odometry_prior_invalid_messages",
        "camera_info_invalid_intrinsics",
        "image_invalid_frames",
        "depth_invalid_frames",
        "rendered_invalid_frames",
        "lidar_invalid_points",
        "lidar_invalid_frames",
        "lidar_invalid_point_times",
        "lidar_out_of_range_point_times",
        "last_lidar_max_abs_point_time_offset_s",
        "pointcloud_imu_wait_queue_size",
        "pointcloud_imu_wait_deferred",
        "pointcloud_imu_wait_released",
        "pointcloud_imu_wait_dropped",
        "sliding_window_imu_reanchors",
        "sliding_window_total_imu_factors",
        "sliding_window_total_imu_preintegration_samples",
        "sliding_window_total_imu_preintegration_dt_s",
        "sliding_window_total_visual_factors",
        "sliding_window_total_se3_photometric_factors",
        "sliding_window_smoothness_factors",
        "sliding_window_accepted_steps",
        "sliding_window_rejected_steps",
        "sliding_window_limited_steps",
        "sliding_window_point_factor_skip_count",
        "sliding_window_plane_factor_skip_count",
        "sliding_window_visual_factor_skip_count",
        "sliding_window_se3_photometric_factor_skip_count",
        "sliding_window_smoothness_factor_skip_count",
        "sliding_window_orphan_factors",
        "sliding_window_imu_factor_skip_count",
        "sliding_window_imu_factor_replacement_count",
        "sliding_window_point_factor_replacement_count",
        "sliding_window_plane_factor_replacement_count",
        "sliding_window_visual_factor_replacement_count",
        "sliding_window_se3_photometric_factor_replacement_count",
        "sliding_window_smoothness_factor_replacement_count",
        "sliding_window_imu_time_gap_skip_count",
        "sliding_window_last_imu_preintegration_samples",
        "sliding_window_last_imu_preintegration_dt_s",
        "sliding_window_last_imu_preintegration_extrapolated_dt_s",
        "sliding_window_last_imu_preintegration_start_stamp_ns",
        "sliding_window_last_imu_preintegration_end_stamp_ns",
        "sliding_window_optimization_skip_count",
        "sliding_window_invalid_optimized_states",
        "sliding_window_last_optimization_duration_ms",
        "sliding_window_feedback_updates",
        "sliding_window_last_feedback_stamp_ns",
        "sliding_window_last_feedback_translation_delta_m",
        "sliding_window_last_feedback_rotation_delta_rad",
        "sliding_window_last_feedback_velocity_delta_mps",
        "sliding_window_max_feedback_translation_m",
        "sliding_window_max_feedback_rotation_rad",
        "sliding_window_max_feedback_velocity_mps",
        "sliding_window_schur_marginalizations",
        "sliding_window_fallback_marginalization_priors",
        "sliding_window_normal_equation_rows",
        "sliding_window_normal_equation_cols",
        "sliding_window_normal_equation_rank",
        "sliding_window_numeric_jacobian_blocks",
        "sliding_window_numeric_jacobian_columns",
        "sliding_window_normal_equation_rank_ratio",
        "sliding_window_min_normal_equation_rank_ratio",
        "sliding_window_max_normal_equation_condition",
        "sliding_window_min_state_dt_s",
        "sliding_window_max_state_dt_s",
        "sliding_window_normal_equation_min_singular_value",
        "sliding_window_normal_equation_max_singular_value",
        "sliding_window_normal_equation_condition_number",
        "sliding_window_normal_equation_degenerate",
        "sliding_window_state_gap_degenerate",
        "sliding_window_last_step_norm",
        "sliding_window_last_step_scale",
        "sliding_window_last_damping",
        "sliding_window_invalid_candidate_steps",
        "sliding_window_linearization_failure_count",
        "sliding_window_linear_solve_failure_count",
        "sliding_window_imu_cost",
        "sliding_window_pose_prior_cost",
        "sliding_window_state_prior_cost",
        "sliding_window_dense_prior_cost",
        "sliding_window_point_factor_cost",
        "sliding_window_plane_factor_cost",
        "sliding_window_visual_factor_cost",
        "sliding_window_se3_photometric_factor_cost",
        "sliding_window_smoothness_factor_cost",
        "trajectory_control_poses",
        "trajectory_deskew_queries",
        "trajectory_deskew_hits",
        "trajectory_control_pose_skip_count",
        "external_odometry_priors_received",
        "external_odometry_prior_matches",
        "external_odometry_prior_misses",
        "last_external_odometry_prior_stamp_ns",
        "visual_rendered_cache_size",
        "visual_rendered_match_delta_ns",
        "visual_depth_cache_size",
        "visual_depth_match_delta_ns",
        "lidar_spatial_index_voxels",
        "lidar_spatial_index_voxel_size_m",
        "visual_alignment_pending_queue_size",
        "visual_se3_photometric_pending_queue_size",
        "visual_alignment_pending_stale_drops",
        "visual_se3_photometric_pending_stale_drops",
        "last_window_point_confidence_mean",
        "last_window_point_confidence_min",
        "last_window_plane_confidence_mean",
        "last_window_plane_confidence_min",
        "visual_se3_photometric_rejected_depth",
        "visual_se3_photometric_rejected_gradient",
        "visual_se3_photometric_rejected_residual",
        "visual_se3_photometric_degenerate_batches",
        "visual_se3_photometric_hessian_rank",
        "visual_se3_photometric_hessian_min_singular_value",
        "visual_se3_photometric_hessian_max_singular_value",
        "visual_se3_photometric_hessian_condition_number",
        "visual_se3_photometric_last_accepted_hessian_rank",
        "visual_se3_photometric_last_accepted_hessian_min_singular_value",
        "visual_se3_photometric_last_accepted_hessian_max_singular_value",
        "visual_se3_photometric_last_accepted_hessian_condition_number",
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
