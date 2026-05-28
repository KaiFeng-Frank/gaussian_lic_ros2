# SPDX-License-Identifier: GPL-3.0-or-later

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
    rendered_feedback_topic = LaunchConfiguration("rendered_feedback_topic")
    enable_rendered_feedback_contract = LaunchConfiguration("enable_rendered_feedback_contract")
    rendered_image_qos_reliability = LaunchConfiguration("rendered_image_qos_reliability")
    rendered_image_qos_durability = LaunchConfiguration("rendered_image_qos_durability")
    rendered_image_qos_depth = LaunchConfiguration("rendered_image_qos_depth")
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
    lidar_window_point_factor_weight = LaunchConfiguration("lidar_window_point_factor_weight")
    lidar_window_plane_factor_weight = LaunchConfiguration("lidar_window_plane_factor_weight")
    lidar_window_confidence_power = LaunchConfiguration("lidar_window_confidence_power")
    lidar_plane_min_neighbors = LaunchConfiguration("lidar_plane_min_neighbors")
    lidar_plane_max_condition = LaunchConfiguration("lidar_plane_max_condition")
    lidar_keyframe_translation_m = LaunchConfiguration("lidar_keyframe_translation_m")
    lidar_to_imu_translation_m = LaunchConfiguration("lidar_to_imu_translation_m")
    lidar_to_imu_rpy_rad = LaunchConfiguration("lidar_to_imu_rpy_rad")
    enable_lidar_deskew = LaunchConfiguration("enable_lidar_deskew")
    enable_visual_factor = LaunchConfiguration("enable_visual_factor")
    visual_factor_max_dt_ns = LaunchConfiguration("visual_factor_max_dt_ns")
    enable_visual_factor_time_interpolation = LaunchConfiguration(
        "enable_visual_factor_time_interpolation"
    )
    enable_visual_cache_reconciliation = LaunchConfiguration("enable_visual_cache_reconciliation")
    visual_cache_reconciliation_monotonic_unique = LaunchConfiguration(
        "visual_cache_reconciliation_monotonic_unique"
    )
    visual_pair_monotonic_unique = LaunchConfiguration("visual_pair_monotonic_unique")
    enable_visual_callback_factor_ingest = LaunchConfiguration(
        "enable_visual_callback_factor_ingest"
    )
    enable_rendered_feedback_watermark_queue = LaunchConfiguration(
        "enable_rendered_feedback_watermark_queue"
    )
    defer_future_visual_factors_until_active = LaunchConfiguration(
        "defer_future_visual_factors_until_active"
    )
    enable_visual_adaptive_state_retention = LaunchConfiguration(
        "enable_visual_adaptive_state_retention"
    )
    visual_adaptive_state_retention_margin_states = LaunchConfiguration(
        "visual_adaptive_state_retention_margin_states"
    )
    visual_adaptive_state_retention_max_states = LaunchConfiguration(
        "visual_adaptive_state_retention_max_states"
    )
    enable_visual_expired_factor_projection = LaunchConfiguration(
        "enable_visual_expired_factor_projection"
    )
    enable_visual_marginalization_prior = LaunchConfiguration(
        "enable_visual_marginalization_prior"
    )
    visual_marginalization_prior_zero_bias_columns = LaunchConfiguration(
        "visual_marginalization_prior_zero_bias_columns"
    )
    visual_expired_factor_projection_max_age_s = LaunchConfiguration(
        "visual_expired_factor_projection_max_age_s"
    )
    visual_cache_reconciliation_defer_to_pointcloud = LaunchConfiguration(
        "visual_cache_reconciliation_defer_to_pointcloud"
    )
    visual_pair_processing_defer_to_pointcloud = LaunchConfiguration(
        "visual_pair_processing_defer_to_pointcloud"
    )
    visual_depth_max_dt_ns = LaunchConfiguration("visual_depth_max_dt_ns")
    depth_frame_cache_size = LaunchConfiguration("depth_frame_cache_size")
    sparse_lidar_depth_dilation_px = LaunchConfiguration("sparse_lidar_depth_dilation_px")
    rendered_frame_cache_size = LaunchConfiguration("rendered_frame_cache_size")
    observed_frame_cache_size = LaunchConfiguration("observed_frame_cache_size")
    visual_pending_factor_queue_size = LaunchConfiguration("visual_pending_factor_queue_size")
    enable_visual_factor_quality_weighting = LaunchConfiguration(
        "enable_visual_factor_quality_weighting"
    )
    visual_factor_quality_min_weight_scale = LaunchConfiguration(
        "visual_factor_quality_min_weight_scale"
    )
    enable_visual_factor_quality_selection = LaunchConfiguration(
        "enable_visual_factor_quality_selection"
    )
    enable_visual_factor_quality_reference_cap = LaunchConfiguration(
        "enable_visual_factor_quality_reference_cap"
    )
    visual_factor_quality_selection_max_per_reference = LaunchConfiguration(
        "visual_factor_quality_selection_max_per_reference"
    )
    visual_factor_quality_selection_start_after_s = LaunchConfiguration(
        "visual_factor_quality_selection_start_after_s"
    )
    camera_to_imu_translation_m = LaunchConfiguration("camera_to_imu_translation_m")
    camera_to_imu_rpy_rad = LaunchConfiguration("camera_to_imu_rpy_rad")
    visual_alignment_max_shift_px = LaunchConfiguration("visual_alignment_max_shift_px")
    visual_alignment_score_mode = LaunchConfiguration("visual_alignment_score_mode")
    visual_alignment_factor_source = LaunchConfiguration("visual_alignment_factor_source")
    visual_factor_source_id_mode = LaunchConfiguration("visual_factor_source_id_mode")
    visual_factor_reference_stamp_mode = LaunchConfiguration(
        "visual_factor_reference_stamp_mode"
    )
    enable_visual_watermark_pair_scheduler = LaunchConfiguration("enable_visual_watermark_pair_scheduler")
    visual_watermark_pair_scheduler_max_pairs_per_pointcloud = LaunchConfiguration("visual_watermark_pair_scheduler_max_pairs_per_pointcloud")
    enable_visual_alignment_window_factor = LaunchConfiguration("enable_visual_alignment_window_factor")
    visual_alignment_meters_per_pixel = LaunchConfiguration("visual_alignment_meters_per_pixel")
    visual_alignment_window_weight = LaunchConfiguration("visual_alignment_window_weight")
    visual_alignment_huber_delta_m = LaunchConfiguration("visual_alignment_huber_delta_m")
    visual_alignment_saturation_margin_px = LaunchConfiguration(
        "visual_alignment_saturation_margin_px"
    )
    visual_alignment_saturated_weight_scale = LaunchConfiguration(
        "visual_alignment_saturated_weight_scale"
    )
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
    se3_photometric_rank_samples_by_gradient = LaunchConfiguration(
        "se3_photometric_rank_samples_by_gradient"
    )
    se3_photometric_use_rendered_gradient = LaunchConfiguration(
        "se3_photometric_use_rendered_gradient"
    )
    se3_photometric_huber_delta = LaunchConfiguration("se3_photometric_huber_delta")
    se3_photometric_max_abs_residual = LaunchConfiguration("se3_photometric_max_abs_residual")
    enable_se3_photometric_pose_correction = LaunchConfiguration(
        "enable_se3_photometric_pose_correction"
    )
    se3_photometric_pose_correction_gain = LaunchConfiguration(
        "se3_photometric_pose_correction_gain"
    )
    se3_photometric_pose_correction_max_translation_m = LaunchConfiguration(
        "se3_photometric_pose_correction_max_translation_m"
    )
    se3_photometric_pose_correction_max_rotation_rad = LaunchConfiguration(
        "se3_photometric_pose_correction_max_rotation_rad"
    )
    se3_photometric_pose_correction_max_dt_ns = LaunchConfiguration(
        "se3_photometric_pose_correction_max_dt_ns"
    )
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
    sliding_window_max_feedback_gyro_bias_step = LaunchConfiguration(
        "sliding_window_max_feedback_gyro_bias_step"
    )
    sliding_window_max_feedback_accel_bias_step = LaunchConfiguration(
        "sliding_window_max_feedback_accel_bias_step"
    )
    sliding_window_min_bias_feedback_visual_factors = LaunchConfiguration(
        "sliding_window_min_bias_feedback_visual_factors"
    )
    sliding_window_sync_guarded_pose_state = LaunchConfiguration(
        "sliding_window_sync_guarded_pose_state"
    )
    sliding_window_guarded_pose_prior_translation_weight = LaunchConfiguration(
        "sliding_window_guarded_pose_prior_translation_weight"
    )
    sliding_window_guarded_pose_prior_rotation_weight = LaunchConfiguration(
        "sliding_window_guarded_pose_prior_rotation_weight"
    )
    sliding_window_max_normal_equation_condition = LaunchConfiguration(
        "sliding_window_max_normal_equation_condition"
    )
    sliding_window_min_normal_equation_rank_ratio = LaunchConfiguration(
        "sliding_window_min_normal_equation_rank_ratio"
    )
    sliding_window_max_state_gap_s = LaunchConfiguration("sliding_window_max_state_gap_s")
    sliding_window_marginalization_prior_weight = LaunchConfiguration(
        "sliding_window_marginalization_prior_weight"
    )
    sliding_window_imu_weight = LaunchConfiguration("sliding_window_imu_weight")
    sliding_window_imu_rotation_weight = LaunchConfiguration("sliding_window_imu_rotation_weight")
    sliding_window_imu_velocity_weight = LaunchConfiguration("sliding_window_imu_velocity_weight")
    sliding_window_imu_position_weight = LaunchConfiguration("sliding_window_imu_position_weight")
    sliding_window_imu_velocity_prior_weight = LaunchConfiguration(
        "sliding_window_imu_velocity_prior_weight"
    )
    sliding_window_gyro_bias_prior_weight = LaunchConfiguration(
        "sliding_window_gyro_bias_prior_weight"
    )
    sliding_window_accel_bias_prior_weight = LaunchConfiguration(
        "sliding_window_accel_bias_prior_weight"
    )
    sliding_window_imu_max_extrapolation_s = LaunchConfiguration("sliding_window_imu_max_extrapolation_s")
    sliding_window_bias_weight = LaunchConfiguration("sliding_window_bias_weight")
    sliding_window_gyro_bias_weight = LaunchConfiguration("sliding_window_gyro_bias_weight")
    sliding_window_accel_bias_weight = LaunchConfiguration("sliding_window_accel_bias_weight")
    sliding_window_bias_random_walk_reference_dt_s = LaunchConfiguration(
        "sliding_window_bias_random_walk_reference_dt_s"
    )
    sliding_window_gyro_bias_random_walk_sigma = LaunchConfiguration(
        "sliding_window_gyro_bias_random_walk_sigma"
    )
    sliding_window_accel_bias_random_walk_sigma = LaunchConfiguration(
        "sliding_window_accel_bias_random_walk_sigma"
    )
    sliding_window_pose_translation_weight = LaunchConfiguration("sliding_window_pose_translation_weight")
    sliding_window_pose_rotation_weight = LaunchConfiguration("sliding_window_pose_rotation_weight")
    enable_sliding_window_smoothness_factor = LaunchConfiguration("enable_sliding_window_smoothness_factor")
    sliding_window_smoothness_rotation_weight = LaunchConfiguration("sliding_window_smoothness_rotation_weight")
    sliding_window_smoothness_position_weight = LaunchConfiguration("sliding_window_smoothness_position_weight")
    sliding_window_smoothness_velocity_weight = LaunchConfiguration("sliding_window_smoothness_velocity_weight")
    sliding_window_smoothness_position_velocity_weight = LaunchConfiguration(
        "sliding_window_smoothness_position_velocity_weight"
    )
    sliding_window_smoothness_bias_weight = LaunchConfiguration("sliding_window_smoothness_bias_weight")
    sliding_window_smoothness_use_motion_targets = LaunchConfiguration(
        "sliding_window_smoothness_use_motion_targets"
    )
    sliding_window_smoothness_motion_target_min_visual_factors = LaunchConfiguration(
        "sliding_window_smoothness_motion_target_min_visual_factors"
    )
    sliding_window_smoothness_motion_target_min_se3_photometric_factors = LaunchConfiguration(
        "sliding_window_smoothness_motion_target_min_se3_photometric_factors"
    )
    sliding_window_smoothness_motion_target_recent_window = LaunchConfiguration(
        "sliding_window_smoothness_motion_target_recent_window"
    )
    sliding_window_smoothness_motion_target_min_recent_visual_factors = LaunchConfiguration(
        "sliding_window_smoothness_motion_target_min_recent_visual_factors"
    )
    sliding_window_smoothness_motion_target_min_recent_se3_photometric_factors = (
        LaunchConfiguration(
            "sliding_window_smoothness_motion_target_min_recent_se3_photometric_factors"
        )
    )
    sliding_window_smoothness_motion_target_start_after_s = LaunchConfiguration(
        "sliding_window_smoothness_motion_target_start_after_s"
    )
    sliding_window_smoothness_motion_target_max_rotation_rate_delta_radps = LaunchConfiguration(
        "sliding_window_smoothness_motion_target_max_rotation_rate_delta_radps"
    )
    sliding_window_smoothness_motion_target_max_position_rate_delta_mps = LaunchConfiguration(
        "sliding_window_smoothness_motion_target_max_position_rate_delta_mps"
    )
    sliding_window_smoothness_motion_target_max_velocity_acceleration_delta_mps2 = (
        LaunchConfiguration(
            "sliding_window_smoothness_motion_target_max_velocity_acceleration_delta_mps2"
        )
    )
    enable_sliding_window_relative_translation_factor = LaunchConfiguration(
        "enable_sliding_window_relative_translation_factor"
    )
    sliding_window_relative_translation_weight = LaunchConfiguration(
        "sliding_window_relative_translation_weight"
    )
    sliding_window_relative_translation_huber_delta_m = LaunchConfiguration(
        "sliding_window_relative_translation_huber_delta_m"
    )
    sliding_window_relative_translation_in_from_frame = LaunchConfiguration(
        "sliding_window_relative_translation_in_from_frame"
    )
    sliding_window_relative_rotation_weight = LaunchConfiguration(
        "sliding_window_relative_rotation_weight"
    )
    sliding_window_relative_rotation_huber_delta_rad = LaunchConfiguration(
        "sliding_window_relative_rotation_huber_delta_rad"
    )
    enable_sliding_window_relative_distance_factor = LaunchConfiguration(
        "enable_sliding_window_relative_distance_factor"
    )
    sliding_window_relative_distance_weight = LaunchConfiguration(
        "sliding_window_relative_distance_weight"
    )
    sliding_window_relative_distance_huber_delta_m = LaunchConfiguration(
        "sliding_window_relative_distance_huber_delta_m"
    )
    enable_sliding_window_multihop_relative_translation_factor = LaunchConfiguration(
        "enable_sliding_window_multihop_relative_translation_factor"
    )
    sliding_window_multihop_relative_translation_weight = LaunchConfiguration(
        "sliding_window_multihop_relative_translation_weight"
    )
    sliding_window_multihop_relative_translation_huber_delta_m = LaunchConfiguration(
        "sliding_window_multihop_relative_translation_huber_delta_m"
    )
    sliding_window_multihop_relative_translation_in_from_frame = LaunchConfiguration(
        "sliding_window_multihop_relative_translation_in_from_frame"
    )
    sliding_window_multihop_relative_rotation_weight = LaunchConfiguration(
        "sliding_window_multihop_relative_rotation_weight"
    )
    sliding_window_multihop_relative_rotation_huber_delta_rad = LaunchConfiguration(
        "sliding_window_multihop_relative_rotation_huber_delta_rad"
    )
    enable_sliding_window_multihop_relative_distance_factor = LaunchConfiguration(
        "enable_sliding_window_multihop_relative_distance_factor"
    )
    sliding_window_multihop_relative_distance_weight = LaunchConfiguration(
        "sliding_window_multihop_relative_distance_weight"
    )
    sliding_window_multihop_relative_distance_huber_delta_m = LaunchConfiguration(
        "sliding_window_multihop_relative_distance_huber_delta_m"
    )
    sliding_window_multihop_relative_translation_min_dt_s = LaunchConfiguration(
        "sliding_window_multihop_relative_translation_min_dt_s"
    )
    sliding_window_multihop_relative_translation_max_dt_s = LaunchConfiguration(
        "sliding_window_multihop_relative_translation_max_dt_s"
    )
    sliding_window_multihop_relative_translation_max_factors = LaunchConfiguration(
        "sliding_window_multihop_relative_translation_max_factors"
    )
    enable_sliding_window_delayed_published_multihop_relative_translation_factor = LaunchConfiguration(
        "enable_sliding_window_delayed_published_multihop_relative_translation_factor"
    )
    sliding_window_delayed_published_multihop_start_after_s = LaunchConfiguration(
        "sliding_window_delayed_published_multihop_start_after_s"
    )
    sliding_window_delayed_published_multihop_max_factors = LaunchConfiguration(
        "sliding_window_delayed_published_multihop_max_factors"
    )
    sliding_window_relative_motion_history_source = LaunchConfiguration(
        "sliding_window_relative_motion_history_source"
    )
    sliding_window_relative_motion_history_published_after_s = LaunchConfiguration(
        "sliding_window_relative_motion_history_published_after_s"
    )
    imu_history_size = LaunchConfiguration("imu_history_size")
    imu_linear_acceleration_scale = LaunchConfiguration("imu_linear_acceleration_scale")
    enable_gaussian_snapshot_lidar_factor = LaunchConfiguration("enable_gaussian_snapshot_lidar_factor")
    enable_gaussian_snapshot_lidar_plane_factor = LaunchConfiguration(
        "enable_gaussian_snapshot_lidar_plane_factor"
    )
    gaussian_snapshot_qos_depth = LaunchConfiguration("gaussian_snapshot_qos_depth")
    gaussian_snapshot_lidar_factor_weight = LaunchConfiguration("gaussian_snapshot_lidar_factor_weight")
    gaussian_snapshot_lidar_nearest_distance_m = LaunchConfiguration(
        "gaussian_snapshot_lidar_nearest_distance_m"
    )
    gaussian_snapshot_lidar_residual_preweight = LaunchConfiguration(
        "gaussian_snapshot_lidar_residual_preweight"
    )
    enable_gaussian_snapshot_lidar_pose_correction = LaunchConfiguration(
        "enable_gaussian_snapshot_lidar_pose_correction"
    )
    gaussian_snapshot_lidar_pose_correction_gain = LaunchConfiguration(
        "gaussian_snapshot_lidar_pose_correction_gain"
    )
    gaussian_snapshot_lidar_pose_correction_max_translation_m = LaunchConfiguration(
        "gaussian_snapshot_lidar_pose_correction_max_translation_m"
    )
    gaussian_snapshot_lidar_pose_correction_max_rotation_rad = LaunchConfiguration(
        "gaussian_snapshot_lidar_pose_correction_max_rotation_rad"
    )
    gaussian_snapshot_lidar_pose_correction_min_match_ratio = LaunchConfiguration(
        "gaussian_snapshot_lidar_pose_correction_min_match_ratio"
    )
    gaussian_snapshot_lidar_pose_correction_max_mean_residual_m = LaunchConfiguration(
        "gaussian_snapshot_lidar_pose_correction_max_mean_residual_m"
    )
    gaussian_snapshot_lidar_pose_correction_coverage_grid_cols = LaunchConfiguration(
        "gaussian_snapshot_lidar_pose_correction_coverage_grid_cols"
    )
    gaussian_snapshot_lidar_pose_correction_coverage_grid_rows = LaunchConfiguration(
        "gaussian_snapshot_lidar_pose_correction_coverage_grid_rows"
    )
    gaussian_snapshot_lidar_pose_correction_min_coverage_tiles = LaunchConfiguration(
        "gaussian_snapshot_lidar_pose_correction_min_coverage_tiles"
    )
    gaussian_snapshot_lidar_pose_correction_bidirectional_max_distance_m = LaunchConfiguration(
        "gaussian_snapshot_lidar_pose_correction_bidirectional_max_distance_m"
    )
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
    post_ba_step_guard_confidence_max_pose_step_m = LaunchConfiguration(
        "post_ba_step_guard_confidence_max_pose_step_m"
    )
    post_ba_step_guard_confidence_warmup_marginalizations = LaunchConfiguration(
        "post_ba_step_guard_confidence_warmup_marginalizations"
    )
    post_ba_step_guard_min_lidar_confidence = LaunchConfiguration(
        "post_ba_step_guard_min_lidar_confidence"
    )
    post_ba_step_guard_min_visual_inlier_ratio = LaunchConfiguration(
        "post_ba_step_guard_min_visual_inlier_ratio"
    )
    post_ba_step_guard_max_visual_residual = LaunchConfiguration(
        "post_ba_step_guard_max_visual_residual"
    )
    post_ba_step_guard_min_visual_coverage_tiles = LaunchConfiguration(
        "post_ba_step_guard_min_visual_coverage_tiles"
    )
    post_ba_step_guard_reject_to_pre_ba_over_m = LaunchConfiguration(
        "post_ba_step_guard_reject_to_pre_ba_over_m"
    )
    post_ba_step_guard_pre_ba_agreement_max_pose_step_m = LaunchConfiguration(
        "post_ba_step_guard_pre_ba_agreement_max_pose_step_m"
    )
    post_ba_step_guard_pre_ba_agreement_late_start_marginalizations = LaunchConfiguration(
        "post_ba_step_guard_pre_ba_agreement_late_start_marginalizations"
    )
    post_ba_step_guard_pre_ba_agreement_late_max_pose_step_m = LaunchConfiguration(
        "post_ba_step_guard_pre_ba_agreement_late_max_pose_step_m"
    )
    post_ba_step_guard_pre_ba_agreement_min_cosine = LaunchConfiguration(
        "post_ba_step_guard_pre_ba_agreement_min_cosine"
    )
    post_ba_step_guard_pre_ba_agreement_max_delta_m = LaunchConfiguration(
        "post_ba_step_guard_pre_ba_agreement_max_delta_m"
    )
    post_ba_step_guard_pre_ba_agreement_margin_m = LaunchConfiguration(
        "post_ba_step_guard_pre_ba_agreement_margin_m"
    )
    post_ba_step_guard_pre_ba_blend_on_clamp = LaunchConfiguration(
        "post_ba_step_guard_pre_ba_blend_on_clamp"
    )
    tracking_step_guard_velocity_scale = LaunchConfiguration("tracking_step_guard_velocity_scale")
    pre_lio_tracking_step_guard_velocity_scale = LaunchConfiguration(
        "pre_lio_tracking_step_guard_velocity_scale"
    )
    post_ba_tracking_step_guard_velocity_scale = LaunchConfiguration(
        "post_ba_tracking_step_guard_velocity_scale"
    )
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
            DeclareLaunchArgument("rendered_feedback_topic", default_value="/gaussian_lic/rendered_feedback"),
            DeclareLaunchArgument("enable_rendered_feedback_contract", default_value="false"),
            DeclareLaunchArgument("rendered_image_qos_reliability", default_value="reliable"),
            DeclareLaunchArgument("rendered_image_qos_durability", default_value="transient_local"),
            DeclareLaunchArgument("rendered_image_qos_depth", default_value="1"),
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
            DeclareLaunchArgument("lidar_window_point_factor_weight", default_value="1.0"),
            DeclareLaunchArgument("lidar_window_plane_factor_weight", default_value="1.0"),
            DeclareLaunchArgument("lidar_window_confidence_power", default_value="1.0"),
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
            DeclareLaunchArgument("enable_visual_factor_time_interpolation", default_value="false"),
            DeclareLaunchArgument("enable_visual_cache_reconciliation", default_value="false"),
            DeclareLaunchArgument("visual_cache_reconciliation_monotonic_unique", default_value="false"),
            DeclareLaunchArgument("visual_pair_monotonic_unique", default_value="false"),
            DeclareLaunchArgument("enable_visual_callback_factor_ingest", default_value="false"),
            DeclareLaunchArgument("enable_rendered_feedback_watermark_queue", default_value="false"),
            DeclareLaunchArgument("defer_future_visual_factors_until_active", default_value="false"),
            DeclareLaunchArgument("enable_visual_adaptive_state_retention", default_value="false"),
            DeclareLaunchArgument("visual_adaptive_state_retention_margin_states", default_value="4"),
            DeclareLaunchArgument("visual_adaptive_state_retention_max_states", default_value="64"),
            DeclareLaunchArgument("enable_visual_expired_factor_projection", default_value="false"),
            DeclareLaunchArgument("enable_visual_marginalization_prior", default_value="false"),
            DeclareLaunchArgument("visual_marginalization_prior_zero_bias_columns", default_value="false"),
            DeclareLaunchArgument("visual_expired_factor_projection_max_age_s", default_value="5.0"),
            DeclareLaunchArgument("visual_cache_reconciliation_defer_to_pointcloud", default_value="false"),
            DeclareLaunchArgument("visual_pair_processing_defer_to_pointcloud", default_value="false"),
            DeclareLaunchArgument("visual_depth_max_dt_ns", default_value="0"),
            DeclareLaunchArgument("depth_frame_cache_size", default_value="8"),
            DeclareLaunchArgument("sparse_lidar_depth_dilation_px", default_value="1"),
            DeclareLaunchArgument("rendered_frame_cache_size", default_value="8"),
            DeclareLaunchArgument("observed_frame_cache_size", default_value="64"),
            DeclareLaunchArgument("visual_pending_factor_queue_size", default_value="64"),
            DeclareLaunchArgument("enable_visual_factor_quality_weighting", default_value="false"),
            DeclareLaunchArgument("visual_factor_quality_min_weight_scale", default_value="0.25"),
            DeclareLaunchArgument("enable_visual_factor_quality_selection", default_value="false"),
            DeclareLaunchArgument("enable_visual_factor_quality_reference_cap", default_value="true"),
            DeclareLaunchArgument(
                "visual_factor_quality_selection_max_per_reference", default_value="2"
            ),
            DeclareLaunchArgument("visual_factor_quality_selection_start_after_s", default_value="0.0"),
            DeclareLaunchArgument("camera_to_imu_translation_m", default_value="[0.0, 0.0, 0.0]"),
            DeclareLaunchArgument("camera_to_imu_rpy_rad", default_value="[0.0, 0.0, 0.0]"),
            DeclareLaunchArgument("visual_alignment_max_shift_px", default_value="8"),
            DeclareLaunchArgument("visual_alignment_score_mode", default_value="rmse"),
            DeclareLaunchArgument("visual_alignment_factor_source", default_value="search"),
            DeclareLaunchArgument("visual_factor_source_id_mode", default_value="legacy_8bit"),
            DeclareLaunchArgument("visual_factor_reference_stamp_mode", default_value="observed"),
            DeclareLaunchArgument("enable_visual_watermark_pair_scheduler", default_value="false"),
            DeclareLaunchArgument("visual_watermark_pair_scheduler_max_pairs_per_pointcloud", default_value="2"),
            DeclareLaunchArgument("enable_visual_alignment_window_factor", default_value="true"),
            DeclareLaunchArgument("visual_alignment_meters_per_pixel", default_value="0.01"),
            DeclareLaunchArgument("visual_alignment_window_weight", default_value="1.0"),
            DeclareLaunchArgument("visual_alignment_huber_delta_m", default_value="0.05"),
            DeclareLaunchArgument("visual_alignment_saturation_margin_px", default_value="0.0"),
            DeclareLaunchArgument("visual_alignment_saturated_weight_scale", default_value="1.0"),
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
            DeclareLaunchArgument("se3_photometric_rank_samples_by_gradient", default_value="false"),
            DeclareLaunchArgument("se3_photometric_use_rendered_gradient", default_value="false"),
            DeclareLaunchArgument("se3_photometric_huber_delta", default_value="0.15"),
            DeclareLaunchArgument("se3_photometric_max_abs_residual", default_value="1.0"),
            DeclareLaunchArgument("enable_se3_photometric_pose_correction", default_value="false"),
            DeclareLaunchArgument("se3_photometric_pose_correction_gain", default_value="0.1"),
            DeclareLaunchArgument("se3_photometric_pose_correction_max_translation_m", default_value="0.02"),
            DeclareLaunchArgument("se3_photometric_pose_correction_max_rotation_rad", default_value="0.01"),
            DeclareLaunchArgument("se3_photometric_pose_correction_max_dt_ns", default_value="0"),
            DeclareLaunchArgument("enable_gaussian_snapshot", default_value="true"),
            DeclareLaunchArgument("tracking_max_pose_step_m", default_value="0.25"),
            DeclareLaunchArgument("enable_pre_lio_tracking_step_guard", default_value="true"),
            DeclareLaunchArgument("enable_post_ba_tracking_step_guard", default_value="true"),
            DeclareLaunchArgument("pre_lio_tracking_max_pose_step_m", default_value="0.0"),
            DeclareLaunchArgument("post_ba_tracking_max_pose_step_m", default_value="0.0"),
            DeclareLaunchArgument("post_ba_step_guard_confidence_max_pose_step_m", default_value="0.0"),
            DeclareLaunchArgument(
                "post_ba_step_guard_confidence_warmup_marginalizations",
                default_value="0",
            ),
            DeclareLaunchArgument("post_ba_step_guard_min_lidar_confidence", default_value="0.6"),
            DeclareLaunchArgument("post_ba_step_guard_min_visual_inlier_ratio", default_value="0.85"),
            DeclareLaunchArgument("post_ba_step_guard_max_visual_residual", default_value="0.3"),
            DeclareLaunchArgument("post_ba_step_guard_min_visual_coverage_tiles", default_value="8"),
            DeclareLaunchArgument("post_ba_step_guard_reject_to_pre_ba_over_m", default_value="0.0"),
            DeclareLaunchArgument("post_ba_step_guard_pre_ba_agreement_max_pose_step_m", default_value="0.0"),
            DeclareLaunchArgument(
                "post_ba_step_guard_pre_ba_agreement_late_start_marginalizations",
                default_value="0",
            ),
            DeclareLaunchArgument(
                "post_ba_step_guard_pre_ba_agreement_late_max_pose_step_m",
                default_value="0.0",
            ),
            DeclareLaunchArgument("post_ba_step_guard_pre_ba_agreement_min_cosine", default_value="0.85"),
            DeclareLaunchArgument("post_ba_step_guard_pre_ba_agreement_max_delta_m", default_value="0.05"),
            DeclareLaunchArgument("post_ba_step_guard_pre_ba_agreement_margin_m", default_value="0.0"),
            DeclareLaunchArgument("post_ba_step_guard_pre_ba_blend_on_clamp", default_value="0.0"),
            DeclareLaunchArgument("tracking_step_guard_velocity_scale", default_value="0.0"),
            DeclareLaunchArgument("pre_lio_tracking_step_guard_velocity_scale", default_value="0.0"),
            DeclareLaunchArgument("post_ba_tracking_step_guard_velocity_scale", default_value="0.0"),
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
            DeclareLaunchArgument("sliding_window_max_feedback_gyro_bias_step", default_value="0.0"),
            DeclareLaunchArgument("sliding_window_max_feedback_accel_bias_step", default_value="0.0"),
            DeclareLaunchArgument("sliding_window_min_bias_feedback_visual_factors", default_value="0"),
            DeclareLaunchArgument("sliding_window_sync_guarded_pose_state", default_value="false"),
            DeclareLaunchArgument(
                "sliding_window_guarded_pose_prior_translation_weight",
                default_value="0.0",
            ),
            DeclareLaunchArgument(
                "sliding_window_guarded_pose_prior_rotation_weight",
                default_value="0.0",
            ),
            DeclareLaunchArgument("sliding_window_max_normal_equation_condition", default_value="10000000000000.0"),
            DeclareLaunchArgument("sliding_window_min_normal_equation_rank_ratio", default_value="0.8"),
            DeclareLaunchArgument("sliding_window_max_state_gap_s", default_value="1.0"),
            DeclareLaunchArgument("sliding_window_marginalization_prior_weight", default_value="1.0"),
            DeclareLaunchArgument("sliding_window_imu_weight", default_value="1.0"),
            DeclareLaunchArgument("sliding_window_imu_rotation_weight", default_value="1.0"),
            DeclareLaunchArgument("sliding_window_imu_velocity_weight", default_value="1.0"),
            DeclareLaunchArgument("sliding_window_imu_position_weight", default_value="1.0"),
            DeclareLaunchArgument("sliding_window_imu_velocity_prior_weight", default_value="0.0"),
            DeclareLaunchArgument("sliding_window_gyro_bias_prior_weight", default_value="0.0"),
            DeclareLaunchArgument("sliding_window_accel_bias_prior_weight", default_value="0.0"),
            DeclareLaunchArgument("sliding_window_imu_max_extrapolation_s", default_value="0.02"),
            DeclareLaunchArgument("sliding_window_bias_weight", default_value="1.0"),
            DeclareLaunchArgument("sliding_window_gyro_bias_weight", default_value="1.0"),
            DeclareLaunchArgument("sliding_window_accel_bias_weight", default_value="1.0"),
            DeclareLaunchArgument("sliding_window_bias_random_walk_reference_dt_s", default_value="0.0"),
            DeclareLaunchArgument("sliding_window_gyro_bias_random_walk_sigma", default_value="0.0"),
            DeclareLaunchArgument("sliding_window_accel_bias_random_walk_sigma", default_value="0.0"),
            DeclareLaunchArgument("sliding_window_pose_translation_weight", default_value="2.0"),
            DeclareLaunchArgument("sliding_window_pose_rotation_weight", default_value="2.0"),
            DeclareLaunchArgument("enable_sliding_window_smoothness_factor", default_value="true"),
            DeclareLaunchArgument("sliding_window_smoothness_rotation_weight", default_value="0.1"),
            DeclareLaunchArgument("sliding_window_smoothness_position_weight", default_value="0.1"),
            DeclareLaunchArgument("sliding_window_smoothness_velocity_weight", default_value="0.1"),
            DeclareLaunchArgument("sliding_window_smoothness_position_velocity_weight", default_value="0.0"),
            DeclareLaunchArgument("sliding_window_smoothness_bias_weight", default_value="0.1"),
            DeclareLaunchArgument("sliding_window_smoothness_use_motion_targets", default_value="false"),
            DeclareLaunchArgument(
                "sliding_window_smoothness_motion_target_min_visual_factors",
                default_value="0",
            ),
            DeclareLaunchArgument(
                "sliding_window_smoothness_motion_target_min_se3_photometric_factors",
                default_value="0",
            ),
            DeclareLaunchArgument(
                "sliding_window_smoothness_motion_target_recent_window",
                default_value="0",
            ),
            DeclareLaunchArgument(
                "sliding_window_smoothness_motion_target_min_recent_visual_factors",
                default_value="0",
            ),
            DeclareLaunchArgument(
                "sliding_window_smoothness_motion_target_min_recent_se3_photometric_factors",
                default_value="0",
            ),
            DeclareLaunchArgument(
                "sliding_window_smoothness_motion_target_start_after_s",
                default_value="0.0",
            ),
            DeclareLaunchArgument(
                "sliding_window_smoothness_motion_target_max_rotation_rate_delta_radps",
                default_value="0.25",
            ),
            DeclareLaunchArgument(
                "sliding_window_smoothness_motion_target_max_position_rate_delta_mps",
                default_value="0.5",
            ),
            DeclareLaunchArgument(
                "sliding_window_smoothness_motion_target_max_velocity_acceleration_delta_mps2",
                default_value="2.0",
            ),
            DeclareLaunchArgument("enable_sliding_window_relative_translation_factor", default_value="false"),
            DeclareLaunchArgument("sliding_window_relative_translation_weight", default_value="0.0"),
            DeclareLaunchArgument("sliding_window_relative_translation_huber_delta_m", default_value="0.1"),
            DeclareLaunchArgument("sliding_window_relative_translation_in_from_frame", default_value="false"),
            DeclareLaunchArgument("sliding_window_relative_rotation_weight", default_value="0.0"),
            DeclareLaunchArgument("sliding_window_relative_rotation_huber_delta_rad", default_value="0.05"),
            DeclareLaunchArgument("enable_sliding_window_relative_distance_factor", default_value="false"),
            DeclareLaunchArgument("sliding_window_relative_distance_weight", default_value="0.0"),
            DeclareLaunchArgument("sliding_window_relative_distance_huber_delta_m", default_value="0.1"),
            DeclareLaunchArgument(
                "enable_sliding_window_multihop_relative_translation_factor",
                default_value="false",
            ),
            DeclareLaunchArgument("sliding_window_multihop_relative_translation_weight", default_value="0.0"),
            DeclareLaunchArgument(
                "sliding_window_multihop_relative_translation_huber_delta_m",
                default_value="0.15",
            ),
            DeclareLaunchArgument(
                "sliding_window_multihop_relative_translation_in_from_frame",
                default_value="false",
            ),
            DeclareLaunchArgument("sliding_window_multihop_relative_rotation_weight", default_value="0.0"),
            DeclareLaunchArgument(
                "sliding_window_multihop_relative_rotation_huber_delta_rad",
                default_value="0.08",
            ),
            DeclareLaunchArgument(
                "enable_sliding_window_multihop_relative_distance_factor",
                default_value="false",
            ),
            DeclareLaunchArgument("sliding_window_multihop_relative_distance_weight", default_value="0.0"),
            DeclareLaunchArgument(
                "sliding_window_multihop_relative_distance_huber_delta_m",
                default_value="0.15",
            ),
            DeclareLaunchArgument("sliding_window_multihop_relative_translation_min_dt_s", default_value="0.45"),
            DeclareLaunchArgument("sliding_window_multihop_relative_translation_max_dt_s", default_value="1.05"),
            DeclareLaunchArgument("sliding_window_multihop_relative_translation_max_factors", default_value="1"),
            DeclareLaunchArgument("enable_sliding_window_delayed_published_multihop_relative_translation_factor", default_value="false"),
            DeclareLaunchArgument("sliding_window_delayed_published_multihop_start_after_s", default_value="0.0"),
            DeclareLaunchArgument("sliding_window_delayed_published_multihop_max_factors", default_value="1"),
            DeclareLaunchArgument("sliding_window_relative_motion_history_source", default_value="pre_ba"),
            DeclareLaunchArgument("sliding_window_relative_motion_history_published_after_s", default_value="0.0"),
            DeclareLaunchArgument("imu_history_size", default_value="12000"),
            DeclareLaunchArgument("imu_linear_acceleration_scale", default_value="1.0"),
            DeclareLaunchArgument("enable_gaussian_snapshot_lidar_factor", default_value="true"),
            DeclareLaunchArgument("enable_gaussian_snapshot_lidar_plane_factor", default_value="false"),
            DeclareLaunchArgument("gaussian_snapshot_qos_depth", default_value="64"),
            DeclareLaunchArgument("gaussian_snapshot_lidar_factor_weight", default_value="1.0"),
            DeclareLaunchArgument("gaussian_snapshot_lidar_nearest_distance_m", default_value="0.0"),
            DeclareLaunchArgument("gaussian_snapshot_lidar_residual_preweight", default_value="true"),
            DeclareLaunchArgument("enable_gaussian_snapshot_lidar_pose_correction", default_value="false"),
            DeclareLaunchArgument("gaussian_snapshot_lidar_pose_correction_gain", default_value="0.3"),
            DeclareLaunchArgument(
                "gaussian_snapshot_lidar_pose_correction_max_translation_m",
                default_value="0.05",
            ),
            DeclareLaunchArgument(
                "gaussian_snapshot_lidar_pose_correction_max_rotation_rad",
                default_value="0.02",
            ),
            DeclareLaunchArgument("gaussian_snapshot_lidar_pose_correction_min_match_ratio", default_value="0.0"),
            DeclareLaunchArgument("gaussian_snapshot_lidar_pose_correction_max_mean_residual_m", default_value="0.0"),
            DeclareLaunchArgument("gaussian_snapshot_lidar_pose_correction_coverage_grid_cols", default_value="1"),
            DeclareLaunchArgument("gaussian_snapshot_lidar_pose_correction_coverage_grid_rows", default_value="1"),
            DeclareLaunchArgument("gaussian_snapshot_lidar_pose_correction_min_coverage_tiles", default_value="0"),
            DeclareLaunchArgument(
                "gaussian_snapshot_lidar_pose_correction_bidirectional_max_distance_m",
                default_value="0.0",
            ),
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
                        "rendered_feedback_topic": rendered_feedback_topic,
                        "enable_rendered_feedback_contract": enable_rendered_feedback_contract,
                        "rendered_image_qos_reliability": rendered_image_qos_reliability,
                        "rendered_image_qos_durability": rendered_image_qos_durability,
                        "rendered_image_qos_depth": rendered_image_qos_depth,
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
                        "lidar_window_point_factor_weight": lidar_window_point_factor_weight,
                        "lidar_window_plane_factor_weight": lidar_window_plane_factor_weight,
                        "lidar_window_confidence_power": lidar_window_confidence_power,
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
                        "enable_visual_factor_time_interpolation": enable_visual_factor_time_interpolation,
                        "enable_visual_cache_reconciliation": enable_visual_cache_reconciliation,
                        "visual_cache_reconciliation_monotonic_unique": (
                            visual_cache_reconciliation_monotonic_unique
                        ),
                        "visual_pair_monotonic_unique": visual_pair_monotonic_unique,
                        "enable_visual_callback_factor_ingest": enable_visual_callback_factor_ingest,
                        "enable_rendered_feedback_watermark_queue": enable_rendered_feedback_watermark_queue,
                        "defer_future_visual_factors_until_active": (
                            defer_future_visual_factors_until_active
                        ),
                        "enable_visual_adaptive_state_retention": (
                            enable_visual_adaptive_state_retention
                        ),
                        "visual_adaptive_state_retention_margin_states": (
                            visual_adaptive_state_retention_margin_states
                        ),
                        "visual_adaptive_state_retention_max_states": (
                            visual_adaptive_state_retention_max_states
                        ),
                        "enable_visual_expired_factor_projection": (
                            enable_visual_expired_factor_projection
                        ),
                        "enable_visual_marginalization_prior": enable_visual_marginalization_prior,
                        "visual_marginalization_prior_zero_bias_columns": visual_marginalization_prior_zero_bias_columns,
                        "visual_expired_factor_projection_max_age_s": (
                            visual_expired_factor_projection_max_age_s
                        ),
                        "visual_cache_reconciliation_defer_to_pointcloud": (
                            visual_cache_reconciliation_defer_to_pointcloud
                        ),
                        "visual_pair_processing_defer_to_pointcloud": (
                            visual_pair_processing_defer_to_pointcloud
                        ),
                        "visual_depth_max_dt_ns": visual_depth_max_dt_ns,
                        "depth_frame_cache_size": depth_frame_cache_size,
                        "sparse_lidar_depth_dilation_px": sparse_lidar_depth_dilation_px,
                        "rendered_frame_cache_size": rendered_frame_cache_size,
                        "observed_frame_cache_size": observed_frame_cache_size,
                        "visual_pending_factor_queue_size": visual_pending_factor_queue_size,
                        "enable_visual_factor_quality_weighting": (
                            enable_visual_factor_quality_weighting
                        ),
                        "visual_factor_quality_min_weight_scale": (
                            ParameterValue(
                                visual_factor_quality_min_weight_scale,
                                value_type=float,
                            )
                        ),
                        "enable_visual_factor_quality_selection": enable_visual_factor_quality_selection,
                        "enable_visual_factor_quality_reference_cap": (
                            enable_visual_factor_quality_reference_cap
                        ),
                        "visual_factor_quality_selection_max_per_reference": (
                            visual_factor_quality_selection_max_per_reference
                        ),
                        "visual_factor_quality_selection_start_after_s": (
                            ParameterValue(
                                visual_factor_quality_selection_start_after_s,
                                value_type=float,
                            )
                        ),
                        "camera_to_imu_translation_m": camera_to_imu_translation_m,
                        "camera_to_imu_rpy_rad": camera_to_imu_rpy_rad,
                        "visual_alignment_max_shift_px": visual_alignment_max_shift_px,
                        "visual_alignment_score_mode": visual_alignment_score_mode,
                        "visual_alignment_factor_source": visual_alignment_factor_source,
                        "visual_factor_source_id_mode": visual_factor_source_id_mode,
                        "visual_factor_reference_stamp_mode": visual_factor_reference_stamp_mode,
                        "enable_visual_watermark_pair_scheduler": enable_visual_watermark_pair_scheduler,
                        "visual_watermark_pair_scheduler_max_pairs_per_pointcloud": visual_watermark_pair_scheduler_max_pairs_per_pointcloud,
                        "enable_visual_alignment_window_factor": enable_visual_alignment_window_factor,
                        "visual_alignment_meters_per_pixel": visual_alignment_meters_per_pixel,
                        "visual_alignment_window_weight": visual_alignment_window_weight,
                        "visual_alignment_huber_delta_m": visual_alignment_huber_delta_m,
                        "visual_alignment_saturation_margin_px": visual_alignment_saturation_margin_px,
                        "visual_alignment_saturated_weight_scale": visual_alignment_saturated_weight_scale,
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
                        "se3_photometric_rank_samples_by_gradient": (
                            se3_photometric_rank_samples_by_gradient
                        ),
                        "se3_photometric_use_rendered_gradient": (
                            se3_photometric_use_rendered_gradient
                        ),
                        "se3_photometric_huber_delta": se3_photometric_huber_delta,
                        "se3_photometric_max_abs_residual": se3_photometric_max_abs_residual,
                        "enable_se3_photometric_pose_correction": (
                            enable_se3_photometric_pose_correction
                        ),
                        "se3_photometric_pose_correction_gain": (
                            se3_photometric_pose_correction_gain
                        ),
                        "se3_photometric_pose_correction_max_translation_m": (
                            se3_photometric_pose_correction_max_translation_m
                        ),
                        "se3_photometric_pose_correction_max_rotation_rad": (
                            se3_photometric_pose_correction_max_rotation_rad
                        ),
                        "se3_photometric_pose_correction_max_dt_ns": (
                            se3_photometric_pose_correction_max_dt_ns
                        ),
                        "enable_gaussian_snapshot": enable_gaussian_snapshot,
                        "tracking_max_pose_step_m": tracking_max_pose_step_m,
                        "enable_pre_lio_tracking_step_guard": enable_pre_lio_tracking_step_guard,
                        "enable_post_ba_tracking_step_guard": enable_post_ba_tracking_step_guard,
                        "pre_lio_tracking_max_pose_step_m": pre_lio_tracking_max_pose_step_m,
                        "post_ba_tracking_max_pose_step_m": post_ba_tracking_max_pose_step_m,
                        "post_ba_step_guard_confidence_max_pose_step_m": (
                            post_ba_step_guard_confidence_max_pose_step_m
                        ),
                        "post_ba_step_guard_confidence_warmup_marginalizations": (
                            post_ba_step_guard_confidence_warmup_marginalizations
                        ),
                        "post_ba_step_guard_min_lidar_confidence": (
                            post_ba_step_guard_min_lidar_confidence
                        ),
                        "post_ba_step_guard_min_visual_inlier_ratio": (
                            post_ba_step_guard_min_visual_inlier_ratio
                        ),
                        "post_ba_step_guard_max_visual_residual": (
                            post_ba_step_guard_max_visual_residual
                        ),
                        "post_ba_step_guard_min_visual_coverage_tiles": (
                            post_ba_step_guard_min_visual_coverage_tiles
                        ),
                        "post_ba_step_guard_reject_to_pre_ba_over_m": (
                            post_ba_step_guard_reject_to_pre_ba_over_m
                        ),
                        "post_ba_step_guard_pre_ba_agreement_max_pose_step_m": (
                            post_ba_step_guard_pre_ba_agreement_max_pose_step_m
                        ),
                        "post_ba_step_guard_pre_ba_agreement_late_start_marginalizations": (
                            post_ba_step_guard_pre_ba_agreement_late_start_marginalizations
                        ),
                        "post_ba_step_guard_pre_ba_agreement_late_max_pose_step_m": (
                            post_ba_step_guard_pre_ba_agreement_late_max_pose_step_m
                        ),
                        "post_ba_step_guard_pre_ba_agreement_min_cosine": (
                            post_ba_step_guard_pre_ba_agreement_min_cosine
                        ),
                        "post_ba_step_guard_pre_ba_agreement_max_delta_m": (
                            post_ba_step_guard_pre_ba_agreement_max_delta_m
                        ),
                        "post_ba_step_guard_pre_ba_agreement_margin_m": (
                            post_ba_step_guard_pre_ba_agreement_margin_m
                        ),
                        "post_ba_step_guard_pre_ba_blend_on_clamp": (
                            post_ba_step_guard_pre_ba_blend_on_clamp
                        ),
                        "tracking_step_guard_velocity_scale": tracking_step_guard_velocity_scale,
                        "pre_lio_tracking_step_guard_velocity_scale": (
                            pre_lio_tracking_step_guard_velocity_scale
                        ),
                        "post_ba_tracking_step_guard_velocity_scale": (
                            post_ba_tracking_step_guard_velocity_scale
                        ),
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
                        "sliding_window_max_feedback_gyro_bias_step": sliding_window_max_feedback_gyro_bias_step,
                        "sliding_window_max_feedback_accel_bias_step": sliding_window_max_feedback_accel_bias_step,
                        "sliding_window_min_bias_feedback_visual_factors": (
                            sliding_window_min_bias_feedback_visual_factors
                        ),
                        "sliding_window_sync_guarded_pose_state": (
                            sliding_window_sync_guarded_pose_state
                        ),
                        "sliding_window_guarded_pose_prior_translation_weight": (
                            sliding_window_guarded_pose_prior_translation_weight
                        ),
                        "sliding_window_guarded_pose_prior_rotation_weight": (
                            sliding_window_guarded_pose_prior_rotation_weight
                        ),
                        "sliding_window_max_normal_equation_condition": sliding_window_max_normal_equation_condition,
                        "sliding_window_min_normal_equation_rank_ratio": sliding_window_min_normal_equation_rank_ratio,
                        "sliding_window_max_state_gap_s": sliding_window_max_state_gap_s,
                        "sliding_window_marginalization_prior_weight": (
                            sliding_window_marginalization_prior_weight
                        ),
                        "sliding_window_imu_weight": sliding_window_imu_weight,
                        "sliding_window_imu_rotation_weight": sliding_window_imu_rotation_weight,
                        "sliding_window_imu_velocity_weight": sliding_window_imu_velocity_weight,
                        "sliding_window_imu_position_weight": sliding_window_imu_position_weight,
                        "sliding_window_imu_velocity_prior_weight": (
                            sliding_window_imu_velocity_prior_weight
                        ),
                        "sliding_window_gyro_bias_prior_weight": (
                            sliding_window_gyro_bias_prior_weight
                        ),
                        "sliding_window_accel_bias_prior_weight": (
                            sliding_window_accel_bias_prior_weight
                        ),
                        "sliding_window_imu_max_extrapolation_s": sliding_window_imu_max_extrapolation_s,
                        "sliding_window_bias_weight": sliding_window_bias_weight,
                        "sliding_window_gyro_bias_weight": sliding_window_gyro_bias_weight,
                        "sliding_window_accel_bias_weight": sliding_window_accel_bias_weight,
                        "sliding_window_bias_random_walk_reference_dt_s": (
                            sliding_window_bias_random_walk_reference_dt_s
                        ),
                        "sliding_window_gyro_bias_random_walk_sigma": (
                            sliding_window_gyro_bias_random_walk_sigma
                        ),
                        "sliding_window_accel_bias_random_walk_sigma": (
                            sliding_window_accel_bias_random_walk_sigma
                        ),
                        "sliding_window_pose_translation_weight": sliding_window_pose_translation_weight,
                        "sliding_window_pose_rotation_weight": sliding_window_pose_rotation_weight,
                        "enable_sliding_window_smoothness_factor": enable_sliding_window_smoothness_factor,
                        "sliding_window_smoothness_rotation_weight": sliding_window_smoothness_rotation_weight,
                        "sliding_window_smoothness_position_weight": sliding_window_smoothness_position_weight,
                        "sliding_window_smoothness_velocity_weight": sliding_window_smoothness_velocity_weight,
                        "sliding_window_smoothness_position_velocity_weight": (
                            sliding_window_smoothness_position_velocity_weight
                        ),
                        "sliding_window_smoothness_bias_weight": sliding_window_smoothness_bias_weight,
                        "sliding_window_smoothness_use_motion_targets": (
                            sliding_window_smoothness_use_motion_targets
                        ),
                        "sliding_window_smoothness_motion_target_min_visual_factors": (
                            sliding_window_smoothness_motion_target_min_visual_factors
                        ),
                        "sliding_window_smoothness_motion_target_min_se3_photometric_factors": (
                            sliding_window_smoothness_motion_target_min_se3_photometric_factors
                        ),
                        "sliding_window_smoothness_motion_target_recent_window": (
                            sliding_window_smoothness_motion_target_recent_window
                        ),
                        "sliding_window_smoothness_motion_target_min_recent_visual_factors": (
                            sliding_window_smoothness_motion_target_min_recent_visual_factors
                        ),
                        "sliding_window_smoothness_motion_target_min_recent_se3_photometric_factors": (
                            sliding_window_smoothness_motion_target_min_recent_se3_photometric_factors
                        ),
                        "sliding_window_smoothness_motion_target_start_after_s": (
                            ParameterValue(
                                sliding_window_smoothness_motion_target_start_after_s,
                                value_type=float,
                            )
                        ),
                        "sliding_window_smoothness_motion_target_max_rotation_rate_delta_radps": (
                            ParameterValue(
                                sliding_window_smoothness_motion_target_max_rotation_rate_delta_radps,
                                value_type=float,
                            )
                        ),
                        "sliding_window_smoothness_motion_target_max_position_rate_delta_mps": (
                            ParameterValue(
                                sliding_window_smoothness_motion_target_max_position_rate_delta_mps,
                                value_type=float,
                            )
                        ),
                        "sliding_window_smoothness_motion_target_max_velocity_acceleration_delta_mps2": (
                            ParameterValue(
                                sliding_window_smoothness_motion_target_max_velocity_acceleration_delta_mps2,
                                value_type=float,
                            )
                        ),
                        "enable_sliding_window_relative_translation_factor": (
                            enable_sliding_window_relative_translation_factor
                        ),
                        "sliding_window_relative_translation_weight": (
                            sliding_window_relative_translation_weight
                        ),
                        "sliding_window_relative_translation_huber_delta_m": (
                            sliding_window_relative_translation_huber_delta_m
                        ),
                        "sliding_window_relative_translation_in_from_frame": (
                            sliding_window_relative_translation_in_from_frame
                        ),
                        "sliding_window_relative_rotation_weight": (
                            sliding_window_relative_rotation_weight
                        ),
                        "sliding_window_relative_rotation_huber_delta_rad": (
                            sliding_window_relative_rotation_huber_delta_rad
                        ),
                        "enable_sliding_window_relative_distance_factor": (
                            enable_sliding_window_relative_distance_factor
                        ),
                        "sliding_window_relative_distance_weight": (
                            sliding_window_relative_distance_weight
                        ),
                        "sliding_window_relative_distance_huber_delta_m": (
                            sliding_window_relative_distance_huber_delta_m
                        ),
                        "enable_sliding_window_multihop_relative_translation_factor": (
                            enable_sliding_window_multihop_relative_translation_factor
                        ),
                        "sliding_window_multihop_relative_translation_weight": (
                            sliding_window_multihop_relative_translation_weight
                        ),
                        "sliding_window_multihop_relative_translation_huber_delta_m": (
                            sliding_window_multihop_relative_translation_huber_delta_m
                        ),
                        "sliding_window_multihop_relative_translation_in_from_frame": (
                            sliding_window_multihop_relative_translation_in_from_frame
                        ),
                        "sliding_window_multihop_relative_rotation_weight": (
                            sliding_window_multihop_relative_rotation_weight
                        ),
                        "sliding_window_multihop_relative_rotation_huber_delta_rad": (
                            sliding_window_multihop_relative_rotation_huber_delta_rad
                        ),
                        "enable_sliding_window_multihop_relative_distance_factor": (
                            enable_sliding_window_multihop_relative_distance_factor
                        ),
                        "sliding_window_multihop_relative_distance_weight": (
                            sliding_window_multihop_relative_distance_weight
                        ),
                        "sliding_window_multihop_relative_distance_huber_delta_m": (
                            sliding_window_multihop_relative_distance_huber_delta_m
                        ),
                        "sliding_window_multihop_relative_translation_min_dt_s": (
                            sliding_window_multihop_relative_translation_min_dt_s
                        ),
                        "sliding_window_multihop_relative_translation_max_dt_s": (
                            sliding_window_multihop_relative_translation_max_dt_s
                        ),
                        "sliding_window_multihop_relative_translation_max_factors": (
                            sliding_window_multihop_relative_translation_max_factors
                        ),
                        "enable_sliding_window_delayed_published_multihop_relative_translation_factor": enable_sliding_window_delayed_published_multihop_relative_translation_factor,
                        "sliding_window_delayed_published_multihop_start_after_s": sliding_window_delayed_published_multihop_start_after_s,
                        "sliding_window_delayed_published_multihop_max_factors": sliding_window_delayed_published_multihop_max_factors,
                        "sliding_window_relative_motion_history_source": sliding_window_relative_motion_history_source,
                        "sliding_window_relative_motion_history_published_after_s": sliding_window_relative_motion_history_published_after_s,
                        "imu_history_size": imu_history_size,
                        "imu_linear_acceleration_scale": imu_linear_acceleration_scale,
                        "enable_gaussian_snapshot_lidar_factor": enable_gaussian_snapshot_lidar_factor,
                        "enable_gaussian_snapshot_lidar_plane_factor": (
                            enable_gaussian_snapshot_lidar_plane_factor
                        ),
                        "gaussian_snapshot_qos_depth": gaussian_snapshot_qos_depth,
                        "gaussian_snapshot_lidar_factor_weight": gaussian_snapshot_lidar_factor_weight,
                        "gaussian_snapshot_lidar_nearest_distance_m": (
                            gaussian_snapshot_lidar_nearest_distance_m
                        ),
                        "gaussian_snapshot_lidar_residual_preweight": (
                            gaussian_snapshot_lidar_residual_preweight
                        ),
                        "enable_gaussian_snapshot_lidar_pose_correction": (
                            enable_gaussian_snapshot_lidar_pose_correction
                        ),
                        "gaussian_snapshot_lidar_pose_correction_gain": (
                            gaussian_snapshot_lidar_pose_correction_gain
                        ),
                        "gaussian_snapshot_lidar_pose_correction_max_translation_m": (
                            gaussian_snapshot_lidar_pose_correction_max_translation_m
                        ),
                        "gaussian_snapshot_lidar_pose_correction_max_rotation_rad": (
                            gaussian_snapshot_lidar_pose_correction_max_rotation_rad
                        ),
                        "gaussian_snapshot_lidar_pose_correction_min_match_ratio": (
                            gaussian_snapshot_lidar_pose_correction_min_match_ratio
                        ),
                        "gaussian_snapshot_lidar_pose_correction_max_mean_residual_m": (
                            gaussian_snapshot_lidar_pose_correction_max_mean_residual_m
                        ),
                        "gaussian_snapshot_lidar_pose_correction_coverage_grid_cols": (
                            gaussian_snapshot_lidar_pose_correction_coverage_grid_cols
                        ),
                        "gaussian_snapshot_lidar_pose_correction_coverage_grid_rows": (
                            gaussian_snapshot_lidar_pose_correction_coverage_grid_rows
                        ),
                        "gaussian_snapshot_lidar_pose_correction_min_coverage_tiles": (
                            gaussian_snapshot_lidar_pose_correction_min_coverage_tiles
                        ),
                        "gaussian_snapshot_lidar_pose_correction_bidirectional_max_distance_m": (
                            gaussian_snapshot_lidar_pose_correction_bidirectional_max_distance_m
                        ),
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
