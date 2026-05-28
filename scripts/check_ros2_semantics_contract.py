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
MAPPING_STATUS_MSG = ROOT / "src" / "gaussian_lic_msgs" / "msg" / "MappingStatus.msg"
NATIVE_TRACKING_REPORT = ROOT / "scripts" / "run_native_tracking_bag_report.sh"
SYNTHETIC_GS_FRAME_PUB = (
    ROOT / "src" / "gaussian_lic_tools" / "gaussian_lic_tools" / "synthetic_gs_frame_pub.py"
)
NATIVE_TRACKING_RECORDER = (
    ROOT / "src" / "gaussian_lic_tools" / "gaussian_lic_tools" / "native_tracking_recorder.py"
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
    mapping_status_msg_text = read(MAPPING_STATUS_MSG)
    native_tracking_report_text = read(NATIVE_TRACKING_REPORT)
    native_tracking_recorder_text = read(NATIVE_TRACKING_RECORDER)
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
    if 'declare_parameter<std::string>("sync_anchor_stream", "pointcloud")' not in mapping_text:
        errors.append("mapping_node must default frame synchronization to pointcloud anchoring")
    if "pop_aligned_image_anchor_locked" not in mapping_text:
        errors.append("mapping_node must expose an image-anchored sync path for mapper feedback continuity tests")
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
        "enable_rendered_feedback_contract",
        "rendered_feedback_topic",
        "enable_visual_factor_time_interpolation",
        "enable_visual_cache_reconciliation",
        "visual_depth_max_dt_ns",
        "depth_frame_cache_size",
        "sparse_lidar_depth_dilation_px",
        "rendered_frame_cache_size",
        "visual_alignment_score_mode",
        "visual_alignment_factor_source",
        "visual_factor_source_id_mode",
        "visual_factor_reference_stamp_mode",
        "enable_visual_watermark_pair_scheduler",
        "visual_watermark_pair_scheduler_max_pairs_per_pointcloud",
        "enable_rendered_feedback_watermark_queue",
        "enable_visual_marginalization_prior",
        "visual_marginalization_prior_zero_bias_columns",
        "enable_rendered_feedback_source_pose_reference",
        "visual_alignment_meters_per_pixel",
        "visual_alignment_window_weight",
        "visual_alignment_huber_delta_m",
        "visual_alignment_saturation_margin_px",
        "visual_alignment_saturated_weight_scale",
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
        "enable_sliding_window_delayed_published_multihop_relative_translation_factor",
        "sliding_window_delayed_published_multihop_start_after_s",
        "sliding_window_delayed_published_multihop_max_factors",
        "sliding_window_relative_motion_history_source",
        "sliding_window_relative_motion_history_published_after_s",
        "enable_sliding_window_smoothness_factor",
        "se3_photometric_factor_huber_delta",
        "se3_photometric_min_hessian_rank",
        "se3_photometric_max_hessian_condition",
        "se3_photometric_min_sample_inlier_ratio",
        "se3_photometric_max_mean_abs_residual_for_factor",
        "se3_photometric_coverage_grid_cols",
        "se3_photometric_coverage_grid_rows",
        "se3_photometric_min_coverage_tiles",
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
        'declare_parameter<std::string>("sliding_window_relative_motion_history_source", "pre_ba")'
        not in tracking_node_text
        or "parse_relative_motion_history_source" not in tracking_node_text
        or "RelativeMotionHistorySource::kPublished" not in tracking_node_text
        or "RelativeMotionHistorySource::kPublishedAfterWarmup" not in tracking_node_text
        or "sliding_window_relative_motion_history_source" not in tracking_status_msg_text
        or "sliding_window_relative_motion_history_published_after_s" not in tracking_status_msg_text
        or "sliding_window_delayed_published_multihop_relative_factors" not in tracking_status_msg_text
        or "sliding_window_delayed_published_multihop_max_factors" not in tracking_status_msg_text
        or "status.sliding_window_delayed_published_multihop_relative_factors" not in tracking_node_text
        or "status.sliding_window_delayed_published_multihop_max_factors" not in tracking_node_text
        or "factor.source_id = source_id" not in tracking_node_text
        or "candidate.source_id == normalized.source_id" not in sliding_window_text
        or "enable_sliding_window_delayed_published_multihop_relative_translation_factor" not in native_tracking_report_text
        or "sliding_window_delayed_published_multihop_start_after_s" not in native_tracking_report_text
        or "sliding_window_delayed_published_multihop_max_factors" not in native_tracking_report_text
        or "SLIDING_WINDOW_RELATIVE_MOTION_HISTORY_SOURCE_REPORT" not in native_tracking_report_text
        or "SLIDING_WINDOW_RELATIVE_MOTION_HISTORY_PUBLISHED_AFTER_S_REPORT" not in native_tracking_report_text
    ):
        errors.append(
            "tracking_node must expose a default-pre_ba relative-motion history source and publish/report it"
        )
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
        ('declare_parameter<std::string>("sliding_window_bias_feedback_ownership", "optimized")', "tracking_node must default bias feedback ownership to optimized"),
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
        ('DeclareLaunchArgument(\n                "sliding_window_bias_feedback_ownership",\n                default_value="optimized",\n            )', "tracking.launch.py must expose bias feedback ownership"),
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
    if 'declare_parameter<std::string>("visual_alignment_score_mode", "rmse")' not in tracking_node_text:
        errors.append("tracking_node must default visual alignment score mode to rmse")
    if 'declare_parameter<std::string>("visual_alignment_factor_source", "search")' not in tracking_node_text:
        errors.append("tracking_node must default visual alignment factor source to search")
    if 'declare_parameter<std::string>("visual_factor_source_id_mode", "legacy_8bit")' not in tracking_node_text:
        errors.append("tracking_node must default visual source-id hashing to the accepted legacy_8bit mode")
    if "uint64_t visual_factor_source_id" not in tracking_node_text or "VisualFactorSourceIdMode::kFull64Bit" not in tracking_node_text:
        errors.append("tracking_node must keep a default-off full_64bit visual source-id diagnostic")
    if "visual_pair_sources_match" not in tracking_node_text or \
            "rendered.rendered_feedback_frame_index" not in tracking_node_text or \
            "rendered.rendered_feedback_preview_index" not in tracking_node_text:
        errors.append(
            "tracking_node must use typed RenderedFeedback frame/preview indices for visual pair/factor identity"
        )
    if 'declare_parameter<std::string>("visual_factor_reference_stamp_mode", "observed")' not in tracking_node_text:
        errors.append("tracking_node must default visual factor reference stamps to observed")
    if "VisualFactorReferenceStampMode::kRendered" not in tracking_node_text:
        errors.append("tracking_node must expose rendered-stamp visual factor ownership as a default-off diagnostic")
    if 'declare_parameter<bool>("enable_rendered_feedback_contract", false)' not in tracking_node_text:
        errors.append("tracking_node must keep typed rendered-feedback subscription disabled by default")
    if 'DeclareLaunchArgument("enable_rendered_feedback_contract", default_value="false")' not in tracking_launch_text:
        errors.append("tracking.launch.py must expose typed rendered-feedback subscription as default-off")
    if "msg/RenderedFeedback.msg" not in (ROOT / "src" / "gaussian_lic_msgs" / "CMakeLists.txt").read_text():
        errors.append("gaussian_lic_msgs must generate the typed RenderedFeedback contract")
    rendered_feedback_msg = (ROOT / "src" / "gaussian_lic_msgs" / "msg" / "RenderedFeedback.msg").read_text()
    if "sensor_msgs/Image observed_depth_image" not in rendered_feedback_msg:
        errors.append("RenderedFeedback must carry the mapper-owned observed depth image")
    if "geometry_msgs/Pose source_pose" not in rendered_feedback_msg:
        errors.append("RenderedFeedback must carry the mapper source pose used for rendering")
    if "feedback.observed_depth_image = make_observed_feedback_depth_message(frame)" not in mapping_text:
        errors.append("mapping_node must embed observed depth in typed RenderedFeedback")
    if "observed.has_embedded_depth = true" not in tracking_node_text or \
            "visual_depth_embedded_observed_matches_" not in tracking_node_text:
        errors.append("tracking_node must use embedded RenderedFeedback depth before consulting the depth cache")
    if 'declare_parameter<bool>("enable_rendered_feedback_source_pose_reference", false)' not in tracking_node_text:
        errors.append("tracking_node must keep rendered-feedback source-pose references default-off")
    if 'DeclareLaunchArgument("enable_rendered_feedback_source_pose_reference", default_value="false")' not in tracking_launch_text:
        errors.append("tracking.launch.py must expose rendered-feedback source-pose references as default-off")
    if "uint64_t source_id{0}" not in sliding_window_header_text:
        errors.append("sliding-window factors must carry 64-bit source ids to avoid replacement collisions")
    if 'DeclareLaunchArgument("visual_alignment_huber_delta_m", default_value="0.05")' not in tracking_launch_text:
        errors.append("tracking.launch.py must expose visual alignment Huber delta")
    if 'DeclareLaunchArgument("visual_alignment_score_mode", default_value="rmse")' not in tracking_launch_text:
        errors.append("tracking.launch.py must expose visual alignment score mode")
    if 'DeclareLaunchArgument("visual_alignment_factor_source", default_value="search")' not in tracking_launch_text:
        errors.append("tracking.launch.py must expose visual alignment factor source")
    if 'DeclareLaunchArgument("visual_factor_source_id_mode", default_value="legacy_8bit")' not in tracking_launch_text:
        errors.append("tracking.launch.py must expose the default-off visual source-id mode")
    if 'DeclareLaunchArgument("visual_factor_reference_stamp_mode", default_value="observed")' not in tracking_launch_text:
        errors.append("tracking.launch.py must expose visual factor reference-stamp ownership")
    if "visual_factor_continuity" not in native_tracking_report_text:
        errors.append("native tracking report must summarize per-bin visual/SE3 factor continuity")
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
    if 'declare_parameter<double>("se3_photometric_min_sample_inlier_ratio", 0.25)' not in tracking_node_text:
        errors.append("tracking_node must default the SE3 photometric sample inlier gate to 0.25")
    if 'DeclareLaunchArgument("se3_photometric_min_sample_inlier_ratio", default_value="0.25")' not in tracking_launch_text:
        errors.append("tracking.launch.py must expose the SE3 photometric sample inlier gate")
    if "se3_photometric_sample_quality_is_healthy" not in tracking_node_text:
        errors.append("tracking_node must reject low-quality SE3 photometric sample batches before BA")
    if 'declare_parameter<int>("se3_photometric_coverage_grid_cols", 4)' not in tracking_node_text or \
            'declare_parameter<int>("se3_photometric_coverage_grid_rows", 4)' not in tracking_node_text or \
            'declare_parameter<int>("se3_photometric_min_coverage_tiles", 4)' not in tracking_node_text:
        errors.append("tracking_node must default the SE3 photometric spatial coverage gate to 4x4/min 4 tiles")
    if 'DeclareLaunchArgument("se3_photometric_coverage_grid_cols", default_value="4")' not in tracking_launch_text or \
            'DeclareLaunchArgument("se3_photometric_coverage_grid_rows", default_value="4")' not in tracking_launch_text or \
            'DeclareLaunchArgument("se3_photometric_min_coverage_tiles", default_value="4")' not in tracking_launch_text:
        errors.append("tracking.launch.py must expose the SE3 photometric spatial coverage gate")
    if "coverage_tiles < static_cast<size_t>(se3_photometric_min_coverage_tiles_)" not in tracking_node_text:
        errors.append("tracking_node must reject spatially clustered SE3 photometric samples before BA")
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
        "visual_depth_embedded_observed_matches",
        "visual_depth_observed_stamp_matches",
        "visual_depth_source_pointcloud_fallback_queries",
        "visual_depth_source_pointcloud_fallback_matches",
        "visual_depth_source_pointcloud_fallback_misses",
    ):
        if field_name not in tracking_status_msg_text or f"status.{field_name}" not in tracking_node_text:
            errors.append(f"TrackingStatus must publish source-index depth selection diagnostics: {field_name}")
    if "select_depth_frame_for_visual_pair" not in tracking_node_text or \
            "rendered.rendered_feedback_pointcloud_stamp_ns" not in tracking_node_text:
        errors.append(
            "tracking_node must prefer observed-stamp depth and fall back to "
            "source-pointcloud depth for rendered-feedback SE3 factors")
    if "visual_marginalization_prior_zero_bias_columns" not in tracking_status_msg_text or \
            "status.visual_marginalization_prior_zero_bias_columns" not in tracking_node_text:
        errors.append("TrackingStatus must expose visual marginalization-prior bias projection")
    if "visual_marginalization_prior_zero_bias_columns" not in tracking_node_text or \
            "visual_marginalization_prior_zero_bias_columns" not in sliding_window_text or \
            "visual_marginalization_prior_zero_bias_columns" not in sliding_window_header_text:
        errors.append("visual marginalization priors must support zeroing IMU bias columns")
    if 'DeclareLaunchArgument("visual_marginalization_prior_zero_bias_columns", default_value="false")' not in tracking_launch_text:
        errors.append("tracking.launch.py must expose visual-prior bias-column projection as default-off")
    if "--visual-marginalization-prior-zero-bias-columns" not in native_tracking_report_text:
        errors.append("native tracking bag report must expose the visual-prior bias-column projector")
    for field_name in (
        "visual_se3_photometric_total_batches",
        "visual_se3_photometric_valid_batches",
        "visual_se3_photometric_insufficient_sample_batches",
        "visual_se3_photometric_degenerate_batches",
        "visual_se3_photometric_quality_rejected_batches",
        "visual_se3_photometric_total_candidates",
        "visual_se3_photometric_total_samples",
        "visual_se3_photometric_sampled_depth",
        "visual_se3_photometric_sample_inlier_ratio",
        "visual_se3_photometric_coverage_tiles",
        "visual_se3_photometric_coverage_total_tiles",
        "visual_se3_photometric_hessian_rank",
        "visual_se3_photometric_hessian_condition_number",
        "visual_se3_photometric_last_accepted_hessian_rank",
        "visual_se3_photometric_last_accepted_hessian_condition_number",
        "visual_se3_photometric_last_accepted_sampled_depth",
        "visual_se3_photometric_last_accepted_samples",
        "visual_se3_photometric_last_accepted_sample_inlier_ratio",
        "visual_se3_photometric_last_accepted_coverage_tiles",
        "visual_se3_photometric_last_accepted_coverage_total_tiles",
        "visual_se3_photometric_last_accepted_mean_abs_residual",
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
    for field_name in (
        "visual_observed_cache_size",
        "visual_observed_match_delta_ns",
        "visual_observed_miss_count",
        "visual_observed_stale_count",
        "visual_observed_size_mismatch_count",
        "visual_pair_processed_count",
        "visual_pair_duplicate_count",
        "rendered_feedback_contract_enabled",
        "num_rendered_feedbacks",
        "last_rendered_feedback_frame_index",
        "last_rendered_feedback_preview_index",
        "rendered_feedback_frame_index_regressions",
        "rendered_feedback_preview_index_regressions",
        "rendered_feedback_frame_index_gap_count",
        "rendered_feedback_preview_index_gap_count",
        "rendered_feedback_frame_index_missing",
        "rendered_feedback_preview_index_missing",
        "rendered_feedback_duplicate_source_ids",
        "last_rendered_feedback_observed_delta_ns",
        "last_rendered_feedback_pose_delta_ns",
        "last_rendered_feedback_pointcloud_delta_ns",
        "last_rendered_feedback_reference_stamp_ns",
        "last_rendered_feedback_oldest_active_state_delta_ns",
        "last_rendered_feedback_newest_active_state_delta_ns",
        "rendered_feedback_before_active_window",
        "rendered_feedback_after_active_window",
        "visual_factor_reference_stamp_mode",
        "visual_factor_time_interpolation_enabled",
        "visual_cache_reconciliation_enabled",
        "visual_pair_monotonic_unique_enabled",
        "visual_watermark_pair_scheduler_enabled",
        "visual_watermark_pair_scheduler_processed_pairs",
        "visual_watermark_pair_scheduler_deferred_pairs",
        "visual_callback_factor_ingest_enabled",
        "rendered_feedback_watermark_queue_enabled",
        "rendered_feedback_watermark_queue_size",
        "rendered_feedback_watermark_processed_pairs",
        "rendered_feedback_watermark_deferred_pairs",
        "rendered_feedback_watermark_queue_drops",
        "rendered_feedback_watermark_reordered_pairs",
        "defer_future_visual_factors_until_active_enabled",
        "visual_adaptive_state_retention_enabled",
        "visual_render_backlog_frames",
        "visual_expired_factor_projection_enabled",
        "visual_marginalization_prior_enabled",
        "visual_alignment_expired_projected_factors",
        "visual_se3_photometric_expired_projected_factors",
        "visual_expired_projection_skipped_factors",
        "visual_alignment_marginalization_priors",
        "visual_se3_photometric_marginalization_priors",
        "visual_marginalization_prior_skipped_factors",
        "sliding_window_marginalized_backsubstitutions",
        "sliding_window_marginalized_backsubstitution_chain_updates",
        "sliding_window_marginalized_backsubstitution_interpolations",
        "sliding_window_visual_marginalization_priors",
        "sliding_window_se3_photometric_marginalization_priors",
        "sliding_window_effective_max_states",
        "visual_alignment_pending_queue_trim_drops",
        "visual_se3_photometric_pending_queue_trim_drops",
        "visual_alignment_pending_expired_drops",
        "visual_se3_photometric_pending_expired_drops",
        "visual_alignment_pending_future_deferrals",
        "visual_se3_photometric_pending_future_deferrals",
        "visual_alignment_interpolated_factors",
        "visual_se3_photometric_interpolated_factors",
        "visual_cache_reconciled_pairs",
        "visual_cache_reconciled_saturated_pairs",
        "visual_cache_reconciled_alignment_skipped_pairs",
        "visual_cache_reconciled_alignment_photometric_fallback_pairs",
        "visual_cache_reconciled_alignment_photometric_disagreement_pairs",
    ):
        if field_name not in tracking_status_msg_text or f"status.{field_name}" not in tracking_node_text:
            errors.append(f"TrackingStatus must publish rendered-to-observed cache diagnostics: {field_name}")
    if "select_visual_factor_reference" not in tracking_node_text:
        errors.append("tracking_node must attach delayed visual factors to active window states")
    if 'declare_parameter<bool>("visual_pair_monotonic_unique", false)' not in tracking_node_text:
        errors.append("tracking_node must expose direct visual-pair one-to-one semantics as default-off")
    if 'declare_parameter<bool>("enable_visual_watermark_pair_scheduler", false)' not in tracking_node_text:
        errors.append("tracking_node must expose deterministic visual watermark pair scheduling as default-off")
    if 'declare_parameter<bool>("enable_visual_callback_factor_ingest", false)' not in tracking_node_text:
        errors.append("tracking_node must expose callback-time visual factor ingestion as default-off")
    if 'declare_parameter<bool>("enable_rendered_feedback_watermark_queue", false)' not in tracking_node_text:
        errors.append("tracking_node must expose typed rendered-feedback watermark queueing as default-off")
    if 'declare_parameter<bool>("defer_future_visual_factors_until_active", false)' not in tracking_node_text:
        errors.append("tracking_node must expose future visual-factor deferral as default-off")
    if 'DeclareLaunchArgument("enable_rendered_feedback_watermark_queue", default_value="false")' not in tracking_launch_text:
        errors.append("tracking.launch.py must expose typed rendered-feedback watermark queueing as default-off")
    if 'DeclareLaunchArgument("defer_future_visual_factors_until_active", default_value="false")' not in tracking_launch_text:
        errors.append("tracking.launch.py must expose future visual-factor deferral as default-off")
    if 'declare_parameter<bool>("enable_visual_adaptive_state_retention", false)' not in tracking_node_text:
        errors.append("tracking_node must expose adaptive visual state retention as default-off")
    if 'declare_parameter<bool>("enable_visual_expired_factor_projection", false)' not in tracking_node_text:
        errors.append("tracking_node must expose expired visual factor projection as default-off")
    if 'declare_parameter<bool>("enable_visual_marginalization_prior", false)' not in tracking_node_text:
        errors.append("tracking_node must expose marginalized visual/SE3 prior conversion as default-off")
    if 'declare_parameter<bool>("visual_marginalization_prior_zero_bias_columns", false)' not in tracking_node_text:
        errors.append("tracking_node must expose visual-prior bias-column projection as default-off")
    if 'DeclareLaunchArgument("visual_pair_monotonic_unique", default_value="false")' not in tracking_launch_text:
        errors.append("tracking.launch.py must expose direct visual-pair one-to-one semantics")
    if 'DeclareLaunchArgument("enable_visual_watermark_pair_scheduler", default_value="false")' not in tracking_launch_text:
        errors.append("tracking.launch.py must expose deterministic visual watermark pair scheduling")
    if 'DeclareLaunchArgument("enable_visual_callback_factor_ingest", default_value="false")' not in tracking_launch_text:
        errors.append("tracking.launch.py must expose callback-time visual factor ingestion")
    if 'DeclareLaunchArgument("enable_visual_adaptive_state_retention", default_value="false")' not in tracking_launch_text:
        errors.append("tracking.launch.py must expose adaptive visual state retention")
    if 'DeclareLaunchArgument("enable_visual_expired_factor_projection", default_value="false")' not in tracking_launch_text:
        errors.append("tracking.launch.py must expose expired visual factor projection")
    if 'DeclareLaunchArgument("enable_visual_marginalization_prior", default_value="false")' not in tracking_launch_text:
        errors.append("tracking.launch.py must expose marginalized visual/SE3 prior conversion")
    if "--enable-visual-pair-monotonic-unique" not in native_tracking_report_text:
        errors.append("native tracking report must expose direct visual-pair one-to-one semantics")
    if "--enable-visual-watermark-pair-scheduler" not in native_tracking_report_text:
        errors.append("native tracking report must expose deterministic visual watermark pair scheduling")
    if "--enable-visual-callback-factor-ingest" not in native_tracking_report_text:
        errors.append("native tracking report must expose callback-time visual factor ingestion")
    if "--enable-rendered-feedback-watermark-queue" not in native_tracking_report_text:
        errors.append("native tracking report must expose typed rendered-feedback watermark queueing")
    if "--enable-visual-adaptive-state-retention" not in native_tracking_report_text:
        errors.append("native tracking report must expose adaptive visual state retention")
    if "--enable-visual-expired-factor-projection" not in native_tracking_report_text:
        errors.append("native tracking report must expose expired visual factor projection")
    if "--enable-visual-marginalization-prior" not in native_tracking_report_text:
        errors.append("native tracking report must expose marginalized visual/SE3 prior conversion")
    if "--visual-factor-reference-stamp-mode" not in native_tracking_report_text:
        errors.append("native tracking report must expose visual factor reference-stamp ownership")
    if "OBSERVED_FRAME_CACHE_SIZE=128" not in native_tracking_report_text or \
            'observed_frame_cache_size:="${OBSERVED_FRAME_CACHE_SIZE}"' not in native_tracking_report_text:
        errors.append("native tracking real-bag report must expose the enlarged observed-frame cache")
    if "--observed-frame-cache-size" not in native_tracking_report_text or \
            "observed_frame_cache_size" not in native_tracking_report_text:
        errors.append("native tracking real-bag report must record visual cache-size hypotheses")
    if 'stop_process_group "${record_pid}" INT' not in native_tracking_report_text:
        errors.append("native tracking real-bag report must stop the recorder with SIGINT for final metrics flush")
    if 'stop_process_group "${record_pid}" TERM' in native_tracking_report_text:
        errors.append("native tracking real-bag report must not stop the recorder with SIGTERM")
    if "canonical_float()" not in native_tracking_report_text or "MAPPER_FEEDBACK_MAX_DEPTH" not in native_tracking_report_text:
        errors.append("native tracking real-bag report must canonicalize mapper double parameters for ros2 CLI overrides")
    if (
        "--post-play-drain-target-poses" not in native_tracking_report_text
        or "--post-play-drain-target-visual-factors" not in native_tracking_report_text
        or "--post-play-drain-target-se3-factors" not in native_tracking_report_text
        or "wait_for_recorder_drain" not in native_tracking_report_text
        or "post_play_drain_target_poses" not in native_tracking_report_text
        or "post_play_drain_target_visual_factors" not in native_tracking_report_text
        or "post_play_drain_target_se3_factors" not in native_tracking_report_text
    ):
        errors.append("native tracking real-bag report must support pose and visual/SE3 evidence drain waits before recorder shutdown")
    for field_name in (
        "pointcloud_messages",
        "pose_messages",
        "image_messages",
        "depth_messages",
        "aligned_frames",
        "converted_frames",
        "dropped_pointcloud_messages",
        "dropped_pose_messages",
        "dropped_image_messages",
        "dropped_depth_messages",
        "pending_pointcloud_messages",
        "pending_pose_messages",
        "pending_image_messages",
        "pending_depth_messages",
        "rendered_preview_count",
        "rendered_feedback_published",
        "rendered_feedback_publish_errors",
        "render_error_count",
        "sync_anchor_stream",
        "rendered_feedback_source_stream",
        "image_pose_feedback_published",
        "image_pose_feedback_image_messages",
        "image_pose_feedback_pose_messages",
        "image_pose_feedback_pose_window_drops",
        "pending_image_pose_feedback_images",
        "pending_image_pose_feedback_poses",
    ):
        if field_name not in mapping_status_msg_text or f"msg.{field_name}" not in mapping_text:
            errors.append(f"MappingStatus must publish mapper feedback liveness diagnostics: {field_name}")
    if (
        'declare_parameter<std::string>("rendered_feedback_source_stream", "aligned_frame")'
        not in mapping_text
    ):
        errors.append("mapping_node must expose image_pose rendered-feedback source as default-off")
    if '"rendered_feedback_source_stream"' not in launch_text:
        errors.append("run_bag.launch.py must expose rendered_feedback_source_stream")
    if "--mapper-feedback-rendered-feedback-source-stream" not in native_tracking_report_text:
        errors.append("native tracking real-bag report must expose mapper rendered-feedback source stream")
    for suffix in ("image-topic", "pose-topic"):
        if f"--mapper-feedback-rendered-feedback-{suffix}" not in native_tracking_report_text:
            errors.append(
                "native tracking real-bag report must expose independent "
                f"image_pose feedback {suffix}"
            )
    if "--mapper-feedback-rendered-feedback-image-qos-depth" not in native_tracking_report_text:
        errors.append("native tracking real-bag report must expose image_pose feedback image QoS depth")
    for topic_param in ("rendered_feedback_image_topic", "rendered_feedback_pose_topic"):
        if f'"{topic_param}"' not in launch_text or f"{topic_param}_" not in mapping_text:
            errors.append(f"image_pose feedback must expose independent topic param: {topic_param}")
    for prefix in ("rendered_feedback_image", "rendered_feedback_pose"):
        if f'declare_topic_qos("{prefix}"' not in mapping_text:
            errors.append(f"image_pose feedback must declare independent DDS QoS prefix: {prefix}")
        if f'"{prefix}_qos_depth"' not in launch_text:
            errors.append(f"image_pose feedback must expose independent DDS QoS depth: {prefix}")
    if "--mapper-feedback-sync-anchor" not in native_tracking_report_text or \
            "mapper_feedback_sync_anchor_stream" not in native_tracking_report_text:
        errors.append("native tracking real-bag report must expose and record the mapper feedback sync anchor")
    if '"sync_anchor_stream"' not in launch_text or 'default_value="pointcloud"' not in launch_text:
        errors.append("run_bag.launch.py must expose mapper sync_anchor_stream with pointcloud default")
    if "++rendered_preview_count_" not in mapping_text:
        errors.append("mapping_node must count successful rendered preview publications")
    if "++rendered_feedback_published_" not in mapping_text:
        errors.append("mapping_node must count successful typed rendered-feedback publications separately")
    if '"rendered_preview_count"' not in native_tracking_report_text:
        errors.append("native tracking real-bag report must gate mapper rendered-preview liveness")
    if (
        "make_rendered_feedback_qos" not in mapping_text
        or "make_rendered_feedback_qos" not in tracking_node_text
    ):
        errors.append("typed rendered feedback must use its own QoS, separate from preview image QoS")
    for qos_name in (
        "rendered_feedback_qos_reliability",
        "rendered_feedback_qos_durability",
        "rendered_feedback_qos_depth",
    ):
        if qos_name not in mapping_text or qos_name not in tracking_node_text:
            errors.append(f"typed rendered feedback QoS must be declared by mapper and tracker: {qos_name}")
        if (
            f'"{qos_name}"' not in launch_text
            or f'"{qos_name}"' not in tracking_launch_text
            or f"--{qos_name.replace('_', '-')}" not in native_tracking_report_text
        ):
            errors.append(f"launch/report tools must expose typed rendered feedback QoS: {qos_name}")
    if (
        "import signal" not in native_tracking_recorder_text
        or "signal.SIGTERM" not in native_tracking_recorder_text
        or "node.flush(final=True)" not in native_tracking_recorder_text
    ):
        errors.append("native_tracking_recorder must convert shutdown signals into a final binned-summary flush")
    if "--enable-rendered-feedback-source-pose-reference" not in native_tracking_report_text or \
            'enable_rendered_feedback_source_pose_reference:="${ENABLE_RENDERED_FEEDBACK_SOURCE_POSE_REFERENCE}"' not in native_tracking_report_text:
        errors.append("native tracking real-bag report must expose rendered-feedback source-pose references")
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
    if "ENABLE_VISUAL_FACTOR_TIME_INTERPOLATION=false" not in native_tracking_report_text or \
            'enable_visual_factor_time_interpolation:="${ENABLE_VISUAL_FACTOR_TIME_INTERPOLATION}"' not in native_tracking_report_text or \
            '"enable_visual_factor_time_interpolation": enable_visual_factor_time_interpolation' not in native_tracking_report_text:
        errors.append("native tracking real-bag report must expose and record visual factor time interpolation")
    if "ENABLE_VISUAL_CACHE_RECONCILIATION=false" not in native_tracking_report_text or \
            'enable_visual_cache_reconciliation:="${ENABLE_VISUAL_CACHE_RECONCILIATION}"' not in native_tracking_report_text or \
            '"enable_visual_cache_reconciliation": enable_visual_cache_reconciliation' not in native_tracking_report_text:
        errors.append("native tracking real-bag report must expose and record visual cache reconciliation")
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
    if "SE3_PHOTOMETRIC_MIN_SAMPLE_INLIER_RATIO=0.25" not in native_tracking_report_text or \
            'se3_photometric_min_sample_inlier_ratio:="${SE3_PHOTOMETRIC_MIN_SAMPLE_INLIER_RATIO}"' not in native_tracking_report_text or \
            '"se3_photometric_min_sample_inlier_ratio": se3_photometric_min_sample_inlier_ratio' not in native_tracking_report_text:
        errors.append("native tracking real-bag report must enforce and record the SE3 sample inlier gate")
    if "SE3_PHOTOMETRIC_COVERAGE_GRID_COLS=4" not in native_tracking_report_text or \
            "SE3_PHOTOMETRIC_COVERAGE_GRID_ROWS=4" not in native_tracking_report_text or \
            "SE3_PHOTOMETRIC_MIN_COVERAGE_TILES=4" not in native_tracking_report_text or \
            'se3_photometric_min_coverage_tiles:="${SE3_PHOTOMETRIC_MIN_COVERAGE_TILES}"' not in native_tracking_report_text or \
            '"se3_photometric_min_coverage_tiles": se3_photometric_min_coverage_tiles' not in native_tracking_report_text:
        errors.append("native tracking real-bag report must enforce and record the SE3 spatial coverage gate")
    if 'declare_parameter<bool>("enable_sliding_window_smoothness_factor", true)' not in tracking_node_text:
        errors.append("tracking_node must default trajectory smoothness BA factors to true")
    if 'DeclareLaunchArgument("enable_sliding_window_smoothness_factor", default_value="true")' not in tracking_launch_text:
        errors.append("tracking.launch.py must default trajectory smoothness BA factors to true")
    for argument in (
        "sliding_window_smoothness_motion_target_min_visual_factors",
        "sliding_window_smoothness_motion_target_min_se3_photometric_factors",
        "sliding_window_smoothness_motion_target_recent_window",
        "sliding_window_smoothness_motion_target_min_recent_visual_factors",
        "sliding_window_smoothness_motion_target_min_recent_se3_photometric_factors",
        "sliding_window_smoothness_motion_target_start_after_s",
        "sliding_window_smoothness_motion_target_max_rotation_rate_delta_radps",
        "sliding_window_smoothness_motion_target_max_position_rate_delta_mps",
        "sliding_window_smoothness_motion_target_max_velocity_acceleration_delta_mps2",
    ):
        if argument not in tracking_node_text or argument not in tracking_launch_text:
            errors.append(f"tracking_node and tracking.launch.py must expose bounded smoothness target parameter {argument}")
        if argument not in native_tracking_report_text:
            errors.append(f"native tracking real-bag report must archive bounded smoothness target parameter {argument}")
    for field in (
        "sliding_window_smoothness_motion_target_applied_count",
        "sliding_window_smoothness_motion_target_support_skip_count",
        "sliding_window_smoothness_motion_target_recent_support_skip_count",
        "sliding_window_smoothness_motion_target_warmup_skip_count",
        "sliding_window_smoothness_motion_target_history_miss_count",
        "sliding_window_smoothness_motion_target_invalid_count",
        "sliding_window_smoothness_motion_target_clamp_count",
        "sliding_window_smoothness_motion_target_last_rotation_rate_delta_norm",
        "sliding_window_smoothness_motion_target_last_position_rate_delta_norm",
        "sliding_window_smoothness_motion_target_last_velocity_acceleration_delta_norm",
        "sliding_window_smoothness_motion_target_max_rotation_rate_delta_norm",
        "sliding_window_smoothness_motion_target_max_position_rate_delta_norm",
        "sliding_window_smoothness_motion_target_max_velocity_acceleration_delta_norm",
        "sliding_window_smoothness_motion_target_recent_visual_factors",
        "sliding_window_smoothness_motion_target_recent_se3_photometric_factors",
    ):
        if field not in tracking_status_msg_text or f"status.{field}" not in tracking_node_text:
            errors.append(f"TrackingStatus must publish bounded smoothness target diagnostic {field}")
    if "--min-motion-target-delta-per-status-bin" not in native_tracking_report_text or \
            "motion_target_continuity" not in native_tracking_report_text:
        errors.append("native tracking real-bag report must gate and summarize smoothness motion-target continuity")
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
    if 'declare_parameter<bool>("sliding_window_sync_guarded_pose_state", false)' not in tracking_node_text:
        errors.append("tracking_node must keep guarded-pose state sync default-off until full-window evidence passes")
    if 'declare_parameter<double>("sliding_window_guarded_pose_prior_translation_weight", 0.0)' not in tracking_node_text or \
            'declare_parameter<double>("sliding_window_guarded_pose_prior_rotation_weight", 0.0)' not in tracking_node_text:
        errors.append("tracking_node must keep guarded-pose soft prior default-off until full-window evidence passes")
    if "sliding_window_guarded_state_syncs" not in tracking_status_msg_text or "status.sliding_window_guarded_state_syncs" not in tracking_node_text:
        errors.append("TrackingStatus must publish guarded-pose state sync count")
    if "sliding_window_guarded_pose_priors" not in tracking_status_msg_text or "status.sliding_window_guarded_pose_priors" not in tracking_node_text:
        errors.append("TrackingStatus must publish guarded-pose soft-prior count")
    if "sliding_window_sync_guarded_pose_state" not in tracking_launch_text or "--sliding-window-sync-guarded-pose-state" not in native_tracking_report_text:
        errors.append("guarded-pose state sync must be exposed only as an explicit report/launch hypothesis")
    if "sliding_window_guarded_pose_prior_translation_weight" not in tracking_launch_text or \
            "--sliding-window-guarded-pose-prior-weights" not in native_tracking_report_text:
        errors.append("guarded-pose soft prior must be exposed only as an explicit report/launch hypothesis")
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
        "pointcloud_imu_wait_stale_dropped",
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
        "sliding_window_smoothness_motion_target_applied_count",
        "sliding_window_smoothness_motion_target_support_skip_count",
        "sliding_window_smoothness_motion_target_recent_support_skip_count",
        "sliding_window_smoothness_motion_target_warmup_skip_count",
        "sliding_window_smoothness_motion_target_history_miss_count",
        "sliding_window_smoothness_motion_target_invalid_count",
        "sliding_window_smoothness_motion_target_clamp_count",
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
        "rendered_feedback_source_pose_reference_enabled",
        "rendered_feedback_source_pose_reference_factors",
        "rendered_feedback_source_pose_invalid",
        "rendered_feedback_embedded_depth_pairs",
        "rendered_feedback_embedded_depth_invalid",
        "visual_depth_embedded_observed_matches",
        "sliding_window_bias_feedback_ownership",
        "sliding_window_bias_feedback_ownership_holds",
        "sliding_window_schur_marginalizations",
        "sliding_window_fallback_marginalization_priors",
        "sliding_window_marginalized_backsubstitutions",
        "sliding_window_marginalized_backsubstitution_chain_updates",
        "sliding_window_marginalized_backsubstitution_interpolations",
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
        "sliding_window_smoothness_motion_target_last_rotation_rate_delta_norm",
        "sliding_window_smoothness_motion_target_last_position_rate_delta_norm",
        "sliding_window_smoothness_motion_target_last_velocity_acceleration_delta_norm",
        "sliding_window_smoothness_motion_target_max_rotation_rate_delta_norm",
        "sliding_window_smoothness_motion_target_max_position_rate_delta_norm",
        "sliding_window_smoothness_motion_target_max_velocity_acceleration_delta_norm",
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
        "visual_alignment_saturated",
        "visual_alignment_saturated_count",
        "visual_alignment_effective_weight",
        "last_window_point_confidence_mean",
        "last_window_point_confidence_min",
        "last_window_plane_confidence_mean",
        "last_window_plane_confidence_min",
        "visual_se3_photometric_rejected_depth",
        "visual_se3_photometric_rejected_gradient",
        "visual_se3_photometric_rejected_residual",
        "visual_se3_photometric_degenerate_batches",
        "visual_se3_photometric_quality_rejected_batches",
        "visual_se3_photometric_sampled_depth",
        "visual_se3_photometric_sample_inlier_ratio",
        "visual_se3_photometric_coverage_tiles",
        "visual_se3_photometric_coverage_total_tiles",
        "visual_se3_photometric_hessian_rank",
        "visual_se3_photometric_hessian_min_singular_value",
        "visual_se3_photometric_hessian_max_singular_value",
        "visual_se3_photometric_hessian_condition_number",
        "visual_se3_photometric_last_accepted_hessian_rank",
        "visual_se3_photometric_last_accepted_hessian_min_singular_value",
        "visual_se3_photometric_last_accepted_hessian_max_singular_value",
        "visual_se3_photometric_last_accepted_hessian_condition_number",
        "visual_se3_photometric_last_accepted_sampled_depth",
        "visual_se3_photometric_last_accepted_samples",
        "visual_se3_photometric_last_accepted_sample_inlier_ratio",
        "visual_se3_photometric_last_accepted_coverage_tiles",
        "visual_se3_photometric_last_accepted_coverage_total_tiles",
        "visual_se3_photometric_last_accepted_mean_abs_residual",
        "visual_se3_photometric_last_accepted_step_norm",
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
