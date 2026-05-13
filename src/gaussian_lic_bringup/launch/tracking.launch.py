# SPDX-License-Identifier: GPL-3.0-or-later

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    publish_tf = LaunchConfiguration("publish_tf")
    raw_image_topic = LaunchConfiguration("raw_image_topic")
    raw_camera_info_topic = LaunchConfiguration("raw_camera_info_topic")
    raw_depth_topic = LaunchConfiguration("raw_depth_topic")
    raw_pointcloud_topic = LaunchConfiguration("raw_pointcloud_topic")
    raw_imu_topic = LaunchConfiguration("raw_imu_topic")
    external_odometry_prior_topic = LaunchConfiguration("external_odometry_prior_topic")
    enable_pointcloud_imu_wait = LaunchConfiguration("enable_pointcloud_imu_wait")
    pointcloud_imu_wait_tolerance_ns = LaunchConfiguration("pointcloud_imu_wait_tolerance_ns")
    pointcloud_imu_wait_queue_size = LaunchConfiguration("pointcloud_imu_wait_queue_size")
    tracking_status_topic = LaunchConfiguration("tracking_status_topic")
    rendered_image_topic = LaunchConfiguration("rendered_image_topic")
    gaussian_map_topic = LaunchConfiguration("gaussian_map_topic")
    serialize_callbacks = LaunchConfiguration("serialize_callbacks")
    sensor_qos_reliability = LaunchConfiguration("sensor_qos_reliability")
    sensor_qos_history = LaunchConfiguration("sensor_qos_history")
    sensor_qos_depth = LaunchConfiguration("sensor_qos_depth")
    qos_streams = (
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
    )
    qos_launch_configs = {
        f"{stream}_{suffix}": LaunchConfiguration(f"{stream}_{suffix}")
        for stream in qos_streams
        for suffix in ("qos_reliability", "qos_history", "qos_depth")
    }
    qos_launch_arguments = [
        DeclareLaunchArgument(
            f"{stream}_{suffix}",
            default_value={
                "qos_reliability": "best_effort",
                "qos_history": "keep_last",
                "qos_depth": "5",
            }[suffix],
        )
        for stream in qos_streams
        for suffix in ("qos_reliability", "qos_history", "qos_depth")
    ]
    enable_lio_factor = LaunchConfiguration("enable_lio_factor")
    enable_external_odometry_prior = LaunchConfiguration("enable_external_odometry_prior")
    external_odometry_prior_max_dt_ns = LaunchConfiguration("external_odometry_prior_max_dt_ns")
    external_odometry_prior_cache_size = LaunchConfiguration("external_odometry_prior_cache_size")
    external_odometry_prior_translation_weight = LaunchConfiguration(
        "external_odometry_prior_translation_weight"
    )
    external_odometry_prior_rotation_weight = LaunchConfiguration(
        "external_odometry_prior_rotation_weight"
    )
    enable_lidar_plane_factor = LaunchConfiguration("enable_lidar_plane_factor")
    lidar_min_points = LaunchConfiguration("lidar_min_points")
    lidar_max_frame_points = LaunchConfiguration("lidar_max_frame_points")
    lidar_max_map_points = LaunchConfiguration("lidar_max_map_points")
    lidar_nearest_distance_m = LaunchConfiguration("lidar_nearest_distance_m")
    lidar_correction_gain = LaunchConfiguration("lidar_correction_gain")
    lidar_max_correction_m = LaunchConfiguration("lidar_max_correction_m")
    lidar_max_rotation_rad = LaunchConfiguration("lidar_max_rotation_rad")
    lidar_robust_kernel_m = LaunchConfiguration("lidar_robust_kernel_m")
    lidar_pose_factor_iterations = LaunchConfiguration("lidar_pose_factor_iterations")
    lidar_plane_min_neighbors = LaunchConfiguration("lidar_plane_min_neighbors")
    lidar_plane_max_condition = LaunchConfiguration("lidar_plane_max_condition")
    lidar_keyframe_translation_m = LaunchConfiguration("lidar_keyframe_translation_m")
    lidar_to_imu_translation_m = LaunchConfiguration("lidar_to_imu_translation_m")
    lidar_to_imu_rpy_rad = LaunchConfiguration("lidar_to_imu_rpy_rad")
    enable_lidar_deskew = LaunchConfiguration("enable_lidar_deskew")
    enable_visual_factor = LaunchConfiguration("enable_visual_factor")
    visual_factor_max_dt_ns = LaunchConfiguration("visual_factor_max_dt_ns")
    visual_depth_max_dt_ns = LaunchConfiguration("visual_depth_max_dt_ns")
    depth_frame_cache_size = LaunchConfiguration("depth_frame_cache_size")
    sparse_lidar_depth_dilation_px = LaunchConfiguration("sparse_lidar_depth_dilation_px")
    rendered_frame_cache_size = LaunchConfiguration("rendered_frame_cache_size")
    observed_frame_cache_size = LaunchConfiguration("observed_frame_cache_size")
    visual_pending_factor_queue_size = LaunchConfiguration("visual_pending_factor_queue_size")
    camera_to_imu_translation_m = LaunchConfiguration("camera_to_imu_translation_m")
    camera_to_imu_rpy_rad = LaunchConfiguration("camera_to_imu_rpy_rad")
    visual_alignment_max_shift_px = LaunchConfiguration("visual_alignment_max_shift_px")
    enable_visual_alignment_window_factor = LaunchConfiguration("enable_visual_alignment_window_factor")
    visual_alignment_meters_per_pixel = LaunchConfiguration("visual_alignment_meters_per_pixel")
    visual_alignment_window_weight = LaunchConfiguration("visual_alignment_window_weight")
    visual_alignment_huber_delta_m = LaunchConfiguration("visual_alignment_huber_delta_m")
    enable_se3_photometric_window_factor = LaunchConfiguration("enable_se3_photometric_window_factor")
    se3_photometric_window_weight = LaunchConfiguration("se3_photometric_window_weight")
    se3_photometric_factor_huber_delta = LaunchConfiguration("se3_photometric_factor_huber_delta")
    se3_photometric_max_samples = LaunchConfiguration("se3_photometric_max_samples")
    se3_photometric_min_samples = LaunchConfiguration("se3_photometric_min_samples")
    se3_photometric_min_hessian_rank = LaunchConfiguration("se3_photometric_min_hessian_rank")
    se3_photometric_max_hessian_condition = LaunchConfiguration(
        "se3_photometric_max_hessian_condition"
    )
    se3_photometric_min_sample_inlier_ratio = LaunchConfiguration(
        "se3_photometric_min_sample_inlier_ratio"
    )
    se3_photometric_max_mean_abs_residual_for_factor = LaunchConfiguration(
        "se3_photometric_max_mean_abs_residual_for_factor"
    )
    se3_photometric_coverage_grid_cols = LaunchConfiguration(
        "se3_photometric_coverage_grid_cols"
    )
    se3_photometric_coverage_grid_rows = LaunchConfiguration(
        "se3_photometric_coverage_grid_rows"
    )
    se3_photometric_min_coverage_tiles = LaunchConfiguration(
        "se3_photometric_min_coverage_tiles"
    )
    se3_photometric_min_depth_m = LaunchConfiguration("se3_photometric_min_depth_m")
    se3_photometric_max_depth_m = LaunchConfiguration("se3_photometric_max_depth_m")
    se3_photometric_min_gradient = LaunchConfiguration("se3_photometric_min_gradient")
    se3_photometric_huber_delta = LaunchConfiguration("se3_photometric_huber_delta")
    se3_photometric_max_abs_residual = LaunchConfiguration("se3_photometric_max_abs_residual")
    enable_gaussian_snapshot = LaunchConfiguration("enable_gaussian_snapshot")
    tracking_max_pose_step_m = LaunchConfiguration("tracking_max_pose_step_m")
    trajectory_control_interval_ns = LaunchConfiguration("trajectory_control_interval_ns")
    enable_sliding_window_optimizer = LaunchConfiguration("enable_sliding_window_optimizer")
    sliding_window_max_states = LaunchConfiguration("sliding_window_max_states")
    sliding_window_optimize_every_n_frames = LaunchConfiguration("sliding_window_optimize_every_n_frames")
    sliding_window_max_iterations = LaunchConfiguration("sliding_window_max_iterations")
    sliding_window_max_rotation_step_rad = LaunchConfiguration("sliding_window_max_rotation_step_rad")
    sliding_window_max_translation_step_m = LaunchConfiguration("sliding_window_max_translation_step_m")
    sliding_window_max_velocity_step_mps = LaunchConfiguration("sliding_window_max_velocity_step_mps")
    sliding_window_max_bias_step = LaunchConfiguration("sliding_window_max_bias_step")
    sliding_window_max_feedback_translation_m = LaunchConfiguration("sliding_window_max_feedback_translation_m")
    sliding_window_max_feedback_rotation_rad = LaunchConfiguration("sliding_window_max_feedback_rotation_rad")
    sliding_window_max_feedback_velocity_mps = LaunchConfiguration("sliding_window_max_feedback_velocity_mps")
    sliding_window_max_feedback_velocity_norm_mps = LaunchConfiguration(
        "sliding_window_max_feedback_velocity_norm_mps"
    )
    sliding_window_max_feedback_gyro_bias_norm = LaunchConfiguration(
        "sliding_window_max_feedback_gyro_bias_norm"
    )
    sliding_window_max_feedback_accel_bias_norm = LaunchConfiguration(
        "sliding_window_max_feedback_accel_bias_norm"
    )
    sliding_window_max_normal_equation_condition = LaunchConfiguration(
        "sliding_window_max_normal_equation_condition"
    )
    sliding_window_min_normal_equation_rank_ratio = LaunchConfiguration(
        "sliding_window_min_normal_equation_rank_ratio"
    )
    sliding_window_max_state_gap_s = LaunchConfiguration("sliding_window_max_state_gap_s")
    sliding_window_imu_weight = LaunchConfiguration("sliding_window_imu_weight")
    sliding_window_imu_rotation_weight = LaunchConfiguration("sliding_window_imu_rotation_weight")
    sliding_window_imu_velocity_weight = LaunchConfiguration("sliding_window_imu_velocity_weight")
    sliding_window_imu_position_weight = LaunchConfiguration("sliding_window_imu_position_weight")
    sliding_window_imu_max_extrapolation_s = LaunchConfiguration("sliding_window_imu_max_extrapolation_s")
    sliding_window_bias_weight = LaunchConfiguration("sliding_window_bias_weight")
    sliding_window_pose_translation_weight = LaunchConfiguration("sliding_window_pose_translation_weight")
    sliding_window_pose_rotation_weight = LaunchConfiguration("sliding_window_pose_rotation_weight")
    enable_sliding_window_smoothness_factor = LaunchConfiguration("enable_sliding_window_smoothness_factor")
    sliding_window_smoothness_rotation_weight = LaunchConfiguration("sliding_window_smoothness_rotation_weight")
    sliding_window_smoothness_position_weight = LaunchConfiguration("sliding_window_smoothness_position_weight")
    sliding_window_smoothness_velocity_weight = LaunchConfiguration("sliding_window_smoothness_velocity_weight")
    sliding_window_smoothness_bias_weight = LaunchConfiguration("sliding_window_smoothness_bias_weight")
    imu_history_size = LaunchConfiguration("imu_history_size")
    imu_linear_acceleration_scale = LaunchConfiguration("imu_linear_acceleration_scale")
    enable_gaussian_snapshot_lidar_factor = LaunchConfiguration("enable_gaussian_snapshot_lidar_factor")
    enable_gaussian_snapshot_lidar_plane_factor = LaunchConfiguration(
        "enable_gaussian_snapshot_lidar_plane_factor"
    )
    gaussian_snapshot_qos_depth = LaunchConfiguration("gaussian_snapshot_qos_depth")
    gaussian_snapshot_lidar_factor_weight = LaunchConfiguration("gaussian_snapshot_lidar_factor_weight")
    gaussian_snapshot_lidar_plane_factor_weight = LaunchConfiguration(
        "gaussian_snapshot_lidar_plane_factor_weight"
    )
    gaussian_snapshot_lidar_min_opacity = LaunchConfiguration("gaussian_snapshot_lidar_min_opacity")
    gaussian_snapshot_lidar_plane_min_anisotropy = LaunchConfiguration(
        "gaussian_snapshot_lidar_plane_min_anisotropy"
    )
    enable_pre_lio_tracking_step_guard = LaunchConfiguration("enable_pre_lio_tracking_step_guard")
    enable_post_ba_tracking_step_guard = LaunchConfiguration("enable_post_ba_tracking_step_guard")
    pre_lio_tracking_max_pose_step_m = LaunchConfiguration("pre_lio_tracking_max_pose_step_m")
    post_ba_tracking_max_pose_step_m = LaunchConfiguration("post_ba_tracking_max_pose_step_m")
    tracking_step_guard_velocity_scale = LaunchConfiguration("tracking_step_guard_velocity_scale")
    tracking_step_guard_acceleration_mps2 = LaunchConfiguration("tracking_step_guard_acceleration_mps2")
    tracking_step_guard_max_velocity_mps = LaunchConfiguration("tracking_step_guard_max_velocity_mps")
    tracking_step_guard_margin_m = LaunchConfiguration("tracking_step_guard_margin_m")
    lidar_time_field = LaunchConfiguration("lidar_time_field")
    lidar_time_unit = LaunchConfiguration("lidar_time_unit")
    lidar_time_mode = LaunchConfiguration("lidar_time_mode")
    lidar_scan_order_duration_s = LaunchConfiguration("lidar_scan_order_duration_s")
    lidar_max_abs_point_time_offset_s = LaunchConfiguration("lidar_max_abs_point_time_offset_s")

    return LaunchDescription(
        [
            DeclareLaunchArgument("publish_tf", default_value="false"),
            DeclareLaunchArgument("raw_image_topic", default_value="/camera/image"),
            DeclareLaunchArgument("raw_camera_info_topic", default_value="/camera/camera_info"),
            DeclareLaunchArgument("raw_depth_topic", default_value="/camera/depth"),
            DeclareLaunchArgument("raw_pointcloud_topic", default_value="/livox/lidar"),
            DeclareLaunchArgument("raw_imu_topic", default_value="/imu"),
            DeclareLaunchArgument(
                "external_odometry_prior_topic",
                default_value="/gaussian_lic/frontend/input_odometry",
            ),
            DeclareLaunchArgument("enable_pointcloud_imu_wait", default_value="true"),
            DeclareLaunchArgument("pointcloud_imu_wait_tolerance_ns", default_value="0"),
            DeclareLaunchArgument("pointcloud_imu_wait_queue_size", default_value="4"),
            DeclareLaunchArgument("tracking_status_topic", default_value="/gaussian_lic/frontend/status"),
            DeclareLaunchArgument("rendered_image_topic", default_value="/gaussian_lic/rendered_image"),
            DeclareLaunchArgument("gaussian_map_topic", default_value="/gaussian_lic/gaussian_map"),
            DeclareLaunchArgument("serialize_callbacks", default_value="true"),
            DeclareLaunchArgument("sensor_qos_reliability", default_value="best_effort"),
            DeclareLaunchArgument("sensor_qos_history", default_value="keep_last"),
            DeclareLaunchArgument("sensor_qos_depth", default_value="5"),
            *qos_launch_arguments,
            DeclareLaunchArgument("enable_lio_factor", default_value="true"),
            DeclareLaunchArgument("enable_external_odometry_prior", default_value="false"),
            DeclareLaunchArgument("external_odometry_prior_max_dt_ns", default_value="100000000"),
            DeclareLaunchArgument("external_odometry_prior_cache_size", default_value="128"),
            DeclareLaunchArgument("external_odometry_prior_translation_weight", default_value="4.0"),
            DeclareLaunchArgument("external_odometry_prior_rotation_weight", default_value="4.0"),
            DeclareLaunchArgument("enable_lidar_plane_factor", default_value="true"),
            DeclareLaunchArgument("lidar_min_points", default_value="32"),
            DeclareLaunchArgument("lidar_max_frame_points", default_value="2000"),
            DeclareLaunchArgument("lidar_max_map_points", default_value="20000"),
            DeclareLaunchArgument("lidar_nearest_distance_m", default_value="0.35"),
            DeclareLaunchArgument("lidar_correction_gain", default_value="0.7"),
            DeclareLaunchArgument("lidar_max_correction_m", default_value="0.25"),
            DeclareLaunchArgument("lidar_max_rotation_rad", default_value="0.08"),
            DeclareLaunchArgument("lidar_robust_kernel_m", default_value="0.15"),
            DeclareLaunchArgument("lidar_pose_factor_iterations", default_value="1"),
            DeclareLaunchArgument("lidar_plane_min_neighbors", default_value="5"),
            DeclareLaunchArgument("lidar_plane_max_condition", default_value="0.2"),
            DeclareLaunchArgument("lidar_keyframe_translation_m", default_value="0.25"),
            DeclareLaunchArgument("lidar_to_imu_translation_m", default_value="[0.0, 0.0, 0.0]"),
            DeclareLaunchArgument("lidar_to_imu_rpy_rad", default_value="[0.0, 0.0, 0.0]"),
            DeclareLaunchArgument("enable_lidar_deskew", default_value="true"),
            DeclareLaunchArgument("lidar_time_field", default_value="auto"),
            DeclareLaunchArgument("lidar_time_unit", default_value="auto"),
            DeclareLaunchArgument("lidar_time_mode", default_value="auto"),
            DeclareLaunchArgument("lidar_scan_order_duration_s", default_value="0.1"),
            DeclareLaunchArgument("lidar_max_abs_point_time_offset_s", default_value="0.25"),
            DeclareLaunchArgument("enable_visual_factor", default_value="true"),
            DeclareLaunchArgument("visual_factor_max_dt_ns", default_value="150000000"),
            DeclareLaunchArgument("visual_depth_max_dt_ns", default_value="0"),
            DeclareLaunchArgument("depth_frame_cache_size", default_value="8"),
            DeclareLaunchArgument("sparse_lidar_depth_dilation_px", default_value="1"),
            DeclareLaunchArgument("rendered_frame_cache_size", default_value="8"),
            DeclareLaunchArgument("observed_frame_cache_size", default_value="64"),
            DeclareLaunchArgument("visual_pending_factor_queue_size", default_value="64"),
            DeclareLaunchArgument("camera_to_imu_translation_m", default_value="[0.0, 0.0, 0.0]"),
            DeclareLaunchArgument("camera_to_imu_rpy_rad", default_value="[0.0, 0.0, 0.0]"),
            DeclareLaunchArgument("visual_alignment_max_shift_px", default_value="8"),
            DeclareLaunchArgument("enable_visual_alignment_window_factor", default_value="true"),
            DeclareLaunchArgument("visual_alignment_meters_per_pixel", default_value="0.01"),
            DeclareLaunchArgument("visual_alignment_window_weight", default_value="1.0"),
            DeclareLaunchArgument("visual_alignment_huber_delta_m", default_value="0.05"),
            DeclareLaunchArgument("enable_se3_photometric_window_factor", default_value="true"),
            DeclareLaunchArgument("se3_photometric_window_weight", default_value="0.5"),
            DeclareLaunchArgument("se3_photometric_factor_huber_delta", default_value="1.0"),
            DeclareLaunchArgument("se3_photometric_max_samples", default_value="2000"),
            DeclareLaunchArgument("se3_photometric_min_samples", default_value="16"),
            DeclareLaunchArgument("se3_photometric_min_hessian_rank", default_value="3"),
            DeclareLaunchArgument("se3_photometric_max_hessian_condition", default_value="1000000000000.0"),
            DeclareLaunchArgument("se3_photometric_min_sample_inlier_ratio", default_value="0.25"),
            DeclareLaunchArgument("se3_photometric_max_mean_abs_residual_for_factor", default_value="0.0"),
            DeclareLaunchArgument("se3_photometric_coverage_grid_cols", default_value="4"),
            DeclareLaunchArgument("se3_photometric_coverage_grid_rows", default_value="4"),
            DeclareLaunchArgument("se3_photometric_min_coverage_tiles", default_value="4"),
            DeclareLaunchArgument("se3_photometric_min_depth_m", default_value="0.05"),
            DeclareLaunchArgument("se3_photometric_max_depth_m", default_value="200.0"),
            DeclareLaunchArgument("se3_photometric_min_gradient", default_value="0.0001"),
            DeclareLaunchArgument("se3_photometric_huber_delta", default_value="0.15"),
            DeclareLaunchArgument("se3_photometric_max_abs_residual", default_value="1.0"),
            DeclareLaunchArgument("enable_gaussian_snapshot", default_value="true"),
            DeclareLaunchArgument("tracking_max_pose_step_m", default_value="0.25"),
            DeclareLaunchArgument("enable_pre_lio_tracking_step_guard", default_value="true"),
            DeclareLaunchArgument("enable_post_ba_tracking_step_guard", default_value="true"),
            DeclareLaunchArgument("pre_lio_tracking_max_pose_step_m", default_value="0.0"),
            DeclareLaunchArgument("post_ba_tracking_max_pose_step_m", default_value="0.0"),
            DeclareLaunchArgument("tracking_step_guard_velocity_scale", default_value="0.0"),
            DeclareLaunchArgument("tracking_step_guard_acceleration_mps2", default_value="0.0"),
            DeclareLaunchArgument("tracking_step_guard_max_velocity_mps", default_value="0.0"),
            DeclareLaunchArgument("tracking_step_guard_margin_m", default_value="0.0"),
            DeclareLaunchArgument("trajectory_control_interval_ns", default_value="50000000"),
            DeclareLaunchArgument("enable_sliding_window_optimizer", default_value="true"),
            DeclareLaunchArgument("sliding_window_max_states", default_value="12"),
            DeclareLaunchArgument("sliding_window_optimize_every_n_frames", default_value="1"),
            DeclareLaunchArgument("sliding_window_max_iterations", default_value="3"),
            DeclareLaunchArgument("sliding_window_max_rotation_step_rad", default_value="0.5"),
            DeclareLaunchArgument("sliding_window_max_translation_step_m", default_value="1.0"),
            DeclareLaunchArgument("sliding_window_max_velocity_step_mps", default_value="5.0"),
            DeclareLaunchArgument("sliding_window_max_bias_step", default_value="1.0"),
            DeclareLaunchArgument("sliding_window_max_feedback_translation_m", default_value="1.0"),
            DeclareLaunchArgument("sliding_window_max_feedback_rotation_rad", default_value="0.5"),
            DeclareLaunchArgument("sliding_window_max_feedback_velocity_mps", default_value="5.0"),
            DeclareLaunchArgument("sliding_window_max_feedback_velocity_norm_mps", default_value="5.0"),
            DeclareLaunchArgument("sliding_window_max_feedback_gyro_bias_norm", default_value="0.5"),
            DeclareLaunchArgument("sliding_window_max_feedback_accel_bias_norm", default_value="2.5"),
            DeclareLaunchArgument("sliding_window_max_normal_equation_condition", default_value="10000000000000.0"),
            DeclareLaunchArgument("sliding_window_min_normal_equation_rank_ratio", default_value="0.8"),
            DeclareLaunchArgument("sliding_window_max_state_gap_s", default_value="1.0"),
            DeclareLaunchArgument("sliding_window_imu_weight", default_value="1.0"),
            DeclareLaunchArgument("sliding_window_imu_rotation_weight", default_value="1.0"),
            DeclareLaunchArgument("sliding_window_imu_velocity_weight", default_value="1.0"),
            DeclareLaunchArgument("sliding_window_imu_position_weight", default_value="1.0"),
            DeclareLaunchArgument("sliding_window_imu_max_extrapolation_s", default_value="0.02"),
            DeclareLaunchArgument("sliding_window_bias_weight", default_value="1.0"),
            DeclareLaunchArgument("sliding_window_pose_translation_weight", default_value="2.0"),
            DeclareLaunchArgument("sliding_window_pose_rotation_weight", default_value="2.0"),
            DeclareLaunchArgument("enable_sliding_window_smoothness_factor", default_value="true"),
            DeclareLaunchArgument("sliding_window_smoothness_rotation_weight", default_value="0.1"),
            DeclareLaunchArgument("sliding_window_smoothness_position_weight", default_value="0.1"),
            DeclareLaunchArgument("sliding_window_smoothness_velocity_weight", default_value="0.1"),
            DeclareLaunchArgument("sliding_window_smoothness_bias_weight", default_value="0.1"),
            DeclareLaunchArgument("imu_history_size", default_value="12000"),
            DeclareLaunchArgument("imu_linear_acceleration_scale", default_value="1.0"),
            DeclareLaunchArgument("enable_gaussian_snapshot_lidar_factor", default_value="true"),
            DeclareLaunchArgument("enable_gaussian_snapshot_lidar_plane_factor", default_value="false"),
            DeclareLaunchArgument("gaussian_snapshot_qos_depth", default_value="64"),
            DeclareLaunchArgument("gaussian_snapshot_lidar_factor_weight", default_value="1.0"),
            DeclareLaunchArgument("gaussian_snapshot_lidar_plane_factor_weight", default_value="1.0"),
            DeclareLaunchArgument("gaussian_snapshot_lidar_min_opacity", default_value="0.01"),
            DeclareLaunchArgument("gaussian_snapshot_lidar_plane_min_anisotropy", default_value="0.25"),
            Node(
                package="gaussian_lic_tracking",
                executable="tracking_node",
                name="tracking_node",
                output="screen",
                parameters=[
                    {
                        "publish_tf": publish_tf,
                        "world_frame": "map",
                        "child_frame": "base_link",
                        "raw_image_topic": raw_image_topic,
                        "raw_camera_info_topic": raw_camera_info_topic,
                        "raw_depth_topic": raw_depth_topic,
                        "raw_pointcloud_topic": raw_pointcloud_topic,
                        "raw_imu_topic": raw_imu_topic,
                        "external_odometry_prior_topic": external_odometry_prior_topic,
                        "enable_pointcloud_imu_wait": enable_pointcloud_imu_wait,
                        "pointcloud_imu_wait_tolerance_ns": pointcloud_imu_wait_tolerance_ns,
                        "pointcloud_imu_wait_queue_size": pointcloud_imu_wait_queue_size,
                        "tracking_status_topic": tracking_status_topic,
                        "rendered_image_topic": rendered_image_topic,
                        "gaussian_map_topic": gaussian_map_topic,
                        "serialize_callbacks": serialize_callbacks,
                        "sensor_qos_reliability": sensor_qos_reliability,
                        "sensor_qos_history": sensor_qos_history,
                        "sensor_qos_depth": sensor_qos_depth,
                        **qos_launch_configs,
                        "enable_lio_factor": enable_lio_factor,
                        "enable_external_odometry_prior": enable_external_odometry_prior,
                        "external_odometry_prior_max_dt_ns": external_odometry_prior_max_dt_ns,
                        "external_odometry_prior_cache_size": external_odometry_prior_cache_size,
                        "external_odometry_prior_translation_weight": (
                            external_odometry_prior_translation_weight
                        ),
                        "external_odometry_prior_rotation_weight": (
                            external_odometry_prior_rotation_weight
                        ),
                        "enable_lidar_plane_factor": enable_lidar_plane_factor,
                        "lidar_min_points": lidar_min_points,
                        "lidar_max_frame_points": lidar_max_frame_points,
                        "lidar_max_map_points": lidar_max_map_points,
                        "lidar_nearest_distance_m": lidar_nearest_distance_m,
                        "lidar_correction_gain": lidar_correction_gain,
                        "lidar_max_correction_m": lidar_max_correction_m,
                        "lidar_max_rotation_rad": lidar_max_rotation_rad,
                        "lidar_robust_kernel_m": lidar_robust_kernel_m,
                        "lidar_pose_factor_iterations": lidar_pose_factor_iterations,
                        "lidar_plane_min_neighbors": lidar_plane_min_neighbors,
                        "lidar_plane_max_condition": lidar_plane_max_condition,
                        "lidar_keyframe_translation_m": lidar_keyframe_translation_m,
                        "lidar_to_imu_translation_m": lidar_to_imu_translation_m,
                        "lidar_to_imu_rpy_rad": lidar_to_imu_rpy_rad,
                        "enable_lidar_deskew": enable_lidar_deskew,
                        "lidar_time_field": lidar_time_field,
                        "lidar_time_unit": lidar_time_unit,
                        "lidar_time_mode": lidar_time_mode,
                        "lidar_scan_order_duration_s": lidar_scan_order_duration_s,
                        "lidar_max_abs_point_time_offset_s": lidar_max_abs_point_time_offset_s,
                        "enable_visual_factor": enable_visual_factor,
                        "visual_factor_max_dt_ns": visual_factor_max_dt_ns,
                        "visual_depth_max_dt_ns": visual_depth_max_dt_ns,
                        "depth_frame_cache_size": depth_frame_cache_size,
                        "sparse_lidar_depth_dilation_px": sparse_lidar_depth_dilation_px,
                        "rendered_frame_cache_size": rendered_frame_cache_size,
                        "observed_frame_cache_size": observed_frame_cache_size,
                        "visual_pending_factor_queue_size": visual_pending_factor_queue_size,
                        "camera_to_imu_translation_m": camera_to_imu_translation_m,
                        "camera_to_imu_rpy_rad": camera_to_imu_rpy_rad,
                        "visual_alignment_max_shift_px": visual_alignment_max_shift_px,
                        "enable_visual_alignment_window_factor": enable_visual_alignment_window_factor,
                        "visual_alignment_meters_per_pixel": visual_alignment_meters_per_pixel,
                        "visual_alignment_window_weight": visual_alignment_window_weight,
                        "visual_alignment_huber_delta_m": visual_alignment_huber_delta_m,
                        "enable_se3_photometric_window_factor": enable_se3_photometric_window_factor,
                        "se3_photometric_window_weight": se3_photometric_window_weight,
                        "se3_photometric_factor_huber_delta": se3_photometric_factor_huber_delta,
                        "se3_photometric_max_samples": se3_photometric_max_samples,
                        "se3_photometric_min_samples": se3_photometric_min_samples,
                        "se3_photometric_min_hessian_rank": se3_photometric_min_hessian_rank,
                        "se3_photometric_max_hessian_condition": se3_photometric_max_hessian_condition,
                        "se3_photometric_min_sample_inlier_ratio": se3_photometric_min_sample_inlier_ratio,
                        "se3_photometric_max_mean_abs_residual_for_factor": se3_photometric_max_mean_abs_residual_for_factor,
                        "se3_photometric_coverage_grid_cols": se3_photometric_coverage_grid_cols,
                        "se3_photometric_coverage_grid_rows": se3_photometric_coverage_grid_rows,
                        "se3_photometric_min_coverage_tiles": se3_photometric_min_coverage_tiles,
                        "se3_photometric_min_depth_m": se3_photometric_min_depth_m,
                        "se3_photometric_max_depth_m": se3_photometric_max_depth_m,
                        "se3_photometric_min_gradient": se3_photometric_min_gradient,
                        "se3_photometric_huber_delta": se3_photometric_huber_delta,
                        "se3_photometric_max_abs_residual": se3_photometric_max_abs_residual,
                        "enable_gaussian_snapshot": enable_gaussian_snapshot,
                        "tracking_max_pose_step_m": tracking_max_pose_step_m,
                        "enable_pre_lio_tracking_step_guard": enable_pre_lio_tracking_step_guard,
                        "enable_post_ba_tracking_step_guard": enable_post_ba_tracking_step_guard,
                        "pre_lio_tracking_max_pose_step_m": pre_lio_tracking_max_pose_step_m,
                        "post_ba_tracking_max_pose_step_m": post_ba_tracking_max_pose_step_m,
                        "tracking_step_guard_velocity_scale": tracking_step_guard_velocity_scale,
                        "tracking_step_guard_acceleration_mps2": tracking_step_guard_acceleration_mps2,
                        "tracking_step_guard_max_velocity_mps": tracking_step_guard_max_velocity_mps,
                        "tracking_step_guard_margin_m": tracking_step_guard_margin_m,
                        "trajectory_control_interval_ns": trajectory_control_interval_ns,
                        "enable_sliding_window_optimizer": enable_sliding_window_optimizer,
                        "sliding_window_max_states": sliding_window_max_states,
                        "sliding_window_optimize_every_n_frames": sliding_window_optimize_every_n_frames,
                        "sliding_window_max_iterations": sliding_window_max_iterations,
                        "sliding_window_max_rotation_step_rad": sliding_window_max_rotation_step_rad,
                        "sliding_window_max_translation_step_m": sliding_window_max_translation_step_m,
                        "sliding_window_max_velocity_step_mps": sliding_window_max_velocity_step_mps,
                        "sliding_window_max_bias_step": sliding_window_max_bias_step,
                        "sliding_window_max_feedback_translation_m": sliding_window_max_feedback_translation_m,
                        "sliding_window_max_feedback_rotation_rad": sliding_window_max_feedback_rotation_rad,
                        "sliding_window_max_feedback_velocity_mps": sliding_window_max_feedback_velocity_mps,
                        "sliding_window_max_feedback_velocity_norm_mps": sliding_window_max_feedback_velocity_norm_mps,
                        "sliding_window_max_feedback_gyro_bias_norm": sliding_window_max_feedback_gyro_bias_norm,
                        "sliding_window_max_feedback_accel_bias_norm": sliding_window_max_feedback_accel_bias_norm,
                        "sliding_window_max_normal_equation_condition": sliding_window_max_normal_equation_condition,
                        "sliding_window_min_normal_equation_rank_ratio": sliding_window_min_normal_equation_rank_ratio,
                        "sliding_window_max_state_gap_s": sliding_window_max_state_gap_s,
                        "sliding_window_imu_weight": sliding_window_imu_weight,
                        "sliding_window_imu_rotation_weight": sliding_window_imu_rotation_weight,
                        "sliding_window_imu_velocity_weight": sliding_window_imu_velocity_weight,
                        "sliding_window_imu_position_weight": sliding_window_imu_position_weight,
                        "sliding_window_imu_max_extrapolation_s": sliding_window_imu_max_extrapolation_s,
                        "sliding_window_bias_weight": sliding_window_bias_weight,
                        "sliding_window_pose_translation_weight": sliding_window_pose_translation_weight,
                        "sliding_window_pose_rotation_weight": sliding_window_pose_rotation_weight,
                        "enable_sliding_window_smoothness_factor": enable_sliding_window_smoothness_factor,
                        "sliding_window_smoothness_rotation_weight": sliding_window_smoothness_rotation_weight,
                        "sliding_window_smoothness_position_weight": sliding_window_smoothness_position_weight,
                        "sliding_window_smoothness_velocity_weight": sliding_window_smoothness_velocity_weight,
                        "sliding_window_smoothness_bias_weight": sliding_window_smoothness_bias_weight,
                        "imu_history_size": imu_history_size,
                        "imu_linear_acceleration_scale": imu_linear_acceleration_scale,
                        "enable_gaussian_snapshot_lidar_factor": enable_gaussian_snapshot_lidar_factor,
                        "enable_gaussian_snapshot_lidar_plane_factor": (
                            enable_gaussian_snapshot_lidar_plane_factor
                        ),
                        "gaussian_snapshot_qos_depth": gaussian_snapshot_qos_depth,
                        "gaussian_snapshot_lidar_factor_weight": gaussian_snapshot_lidar_factor_weight,
                        "gaussian_snapshot_lidar_plane_factor_weight": (
                            gaussian_snapshot_lidar_plane_factor_weight
                        ),
                        "gaussian_snapshot_lidar_min_opacity": gaussian_snapshot_lidar_min_opacity,
                        "gaussian_snapshot_lidar_plane_min_anisotropy": (
                            gaussian_snapshot_lidar_plane_min_anisotropy
                        ),
                    }
                ],
            ),
        ]
    )
