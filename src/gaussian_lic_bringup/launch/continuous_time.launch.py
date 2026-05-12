# SPDX-License-Identifier: GPL-3.0-or-later
#
# Launches the standalone continuous-time tracking node introduced as part of
# the Coco-LIC port. Runs alongside (not inside) the existing tracking_node
# so it can be evaluated against real bags without changing the default
# discrete-state tracker behavior.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    raw_imu_topic = LaunchConfiguration("raw_imu_topic")
    raw_pointcloud_topic = LaunchConfiguration("raw_pointcloud_topic")
    external_odometry_prior_topic = LaunchConfiguration("external_odometry_prior_topic")
    enable_external_odometry_prior = LaunchConfiguration("enable_external_odometry_prior")
    enable_external_odometry_position_factors = LaunchConfiguration(
        "enable_external_odometry_position_factors"
    )
    enable_external_odometry_orientation_factors = LaunchConfiguration(
        "enable_external_odometry_orientation_factors"
    )
    external_odometry_position_factor_weight = LaunchConfiguration(
        "external_odometry_position_factor_weight"
    )
    external_odometry_position_factor_huber_delta_m = LaunchConfiguration(
        "external_odometry_position_factor_huber_delta_m"
    )
    external_odometry_orientation_factor_weight = LaunchConfiguration(
        "external_odometry_orientation_factor_weight"
    )
    external_odometry_orientation_factor_huber_delta_rad = LaunchConfiguration(
        "external_odometry_orientation_factor_huber_delta_rad"
    )
    odometry_topic = LaunchConfiguration("odometry_topic")
    path_topic = LaunchConfiguration("path_topic")
    knot_interval_seconds = LaunchConfiguration("knot_interval_seconds")
    window_knot_count = LaunchConfiguration("window_knot_count")
    marginalize_oldest_count = LaunchConfiguration("marginalize_oldest_count")
    max_iterations_per_step = LaunchConfiguration("max_iterations_per_step")
    imu_info_gyro = LaunchConfiguration("imu_info_gyro")
    imu_info_accel = LaunchConfiguration("imu_info_accel")
    ceres_initial_trust_region_radius = LaunchConfiguration(
        "ceres_initial_trust_region_radius"
    )
    ceres_max_trust_region_radius = LaunchConfiguration("ceres_max_trust_region_radius")
    position_smoothness_weight = LaunchConfiguration("position_smoothness_weight")
    position_smoothness_huber_delta_m = LaunchConfiguration(
        "position_smoothness_huber_delta_m"
    )
    lidar_huber_delta_m = LaunchConfiguration("lidar_huber_delta_m")
    step_period_seconds = LaunchConfiguration("step_period_seconds")
    diagnostic_log_period_steps = LaunchConfiguration("diagnostic_log_period_steps")
    seed_min_imu_count = LaunchConfiguration("seed_min_imu_count")
    hold_gravity_constant = LaunchConfiguration("hold_gravity_constant")
    hold_accel_bias_constant = LaunchConfiguration("hold_accel_bias_constant")
    hold_gyro_bias_constant = LaunchConfiguration("hold_gyro_bias_constant")
    enable_startup_bias_autocal = LaunchConfiguration("enable_startup_bias_autocal")
    imu_linear_acceleration_scale = LaunchConfiguration("imu_linear_acceleration_scale")
    max_position_update_m = LaunchConfiguration("max_position_update_m")
    max_rotation_update_rad = LaunchConfiguration("max_rotation_update_rad")
    update_gate_edge_knot_margin = LaunchConfiguration("update_gate_edge_knot_margin")
    position_extrapolation_damping = LaunchConfiguration("position_extrapolation_damping")
    apply_position_update_on_rotation_reject = LaunchConfiguration(
        "apply_position_update_on_rotation_reject"
    )
    apply_limited_rotation_update = LaunchConfiguration("apply_limited_rotation_update")
    scale_position_with_limited_rotation = LaunchConfiguration(
        "scale_position_with_limited_rotation"
    )
    pointcloud_enable = LaunchConfiguration("pointcloud_enable")
    pointcloud_subsample_stride = LaunchConfiguration("pointcloud_subsample_stride")
    pointcloud_max_points_per_msg = LaunchConfiguration("pointcloud_max_points_per_msg")
    pointcloud_wait_queue_max_size = LaunchConfiguration("pointcloud_wait_queue_max_size")
    pointcloud_min_range_m = LaunchConfiguration("pointcloud_min_range_m")
    pointcloud_max_range_m = LaunchConfiguration("pointcloud_max_range_m")
    pointcloud_factor_weight = LaunchConfiguration("pointcloud_factor_weight")
    enable_lidar_pose_prior_factor = LaunchConfiguration("enable_lidar_pose_prior_factor")
    lidar_pose_prior_position_weight = LaunchConfiguration(
        "lidar_pose_prior_position_weight"
    )
    lidar_pose_prior_orientation_weight = LaunchConfiguration(
        "lidar_pose_prior_orientation_weight"
    )
    lidar_pose_prior_position_huber_delta_m = LaunchConfiguration(
        "lidar_pose_prior_position_huber_delta_m"
    )
    lidar_pose_prior_orientation_huber_delta_rad = LaunchConfiguration(
        "lidar_pose_prior_orientation_huber_delta_rad"
    )
    lidar_pose_factor_keyframe_stride = LaunchConfiguration(
        "lidar_pose_factor_keyframe_stride"
    )
    body_frame_id = LaunchConfiguration("body_frame_id")
    world_frame_id = LaunchConfiguration("world_frame_id")

    declarations = [
        DeclareLaunchArgument("raw_imu_topic", default_value="/imu_for_gs"),
        DeclareLaunchArgument(
            "raw_pointcloud_topic", default_value="/points_for_gs"
        ),
        DeclareLaunchArgument("external_odometry_prior_topic", default_value=""),
        DeclareLaunchArgument("enable_external_odometry_prior", default_value="false"),
        DeclareLaunchArgument("enable_external_odometry_position_factors", default_value="false"),
        DeclareLaunchArgument("enable_external_odometry_orientation_factors", default_value="false"),
        DeclareLaunchArgument("external_odometry_position_factor_weight", default_value="1.0"),
        DeclareLaunchArgument("external_odometry_position_factor_huber_delta_m", default_value="0.25"),
        DeclareLaunchArgument("external_odometry_orientation_factor_weight", default_value="1.0"),
        DeclareLaunchArgument("external_odometry_orientation_factor_huber_delta_rad", default_value="0.25"),
        DeclareLaunchArgument(
            "odometry_topic",
            default_value="/gaussian_lic/continuous_time/odometry",
        ),
        DeclareLaunchArgument(
            "path_topic",
            default_value="/gaussian_lic/continuous_time/path",
        ),
        DeclareLaunchArgument("knot_interval_seconds", default_value="0.05"),
        DeclareLaunchArgument("window_knot_count", default_value="8"),
        DeclareLaunchArgument("marginalize_oldest_count", default_value="1"),
        DeclareLaunchArgument("max_iterations_per_step", default_value="1"),
        DeclareLaunchArgument("imu_info_gyro", default_value="10.0"),
        DeclareLaunchArgument("imu_info_accel", default_value="1.0"),
        DeclareLaunchArgument("ceres_initial_trust_region_radius", default_value="0.0"),
        DeclareLaunchArgument("ceres_max_trust_region_radius", default_value="0.0"),
        DeclareLaunchArgument("position_smoothness_weight", default_value="0.0"),
        DeclareLaunchArgument("position_smoothness_huber_delta_m", default_value="0.0"),
        DeclareLaunchArgument("lidar_huber_delta_m", default_value="0.10"),
        DeclareLaunchArgument("step_period_seconds", default_value="0.10"),
        DeclareLaunchArgument("diagnostic_log_period_steps", default_value="50"),
        DeclareLaunchArgument("seed_min_imu_count", default_value="25"),
        DeclareLaunchArgument("hold_gravity_constant", default_value="true"),
        DeclareLaunchArgument("hold_accel_bias_constant", default_value="false"),
        DeclareLaunchArgument("hold_gyro_bias_constant", default_value="false"),
        DeclareLaunchArgument("enable_startup_bias_autocal", default_value="true"),
        DeclareLaunchArgument("imu_linear_acceleration_scale", default_value="1.0"),
        DeclareLaunchArgument("max_position_update_m", default_value="2.0"),
        DeclareLaunchArgument("max_rotation_update_rad", default_value="0.50"),
        DeclareLaunchArgument("update_gate_edge_knot_margin", default_value="0"),
        DeclareLaunchArgument("position_extrapolation_damping", default_value="0.0"),
        DeclareLaunchArgument(
            "apply_position_update_on_rotation_reject", default_value="false"
        ),
        DeclareLaunchArgument("apply_limited_rotation_update", default_value="false"),
        DeclareLaunchArgument("scale_position_with_limited_rotation", default_value="true"),
        DeclareLaunchArgument("pointcloud_enable", default_value="true"),
        DeclareLaunchArgument("pointcloud_subsample_stride", default_value="50"),
        DeclareLaunchArgument("pointcloud_max_points_per_msg", default_value="256"),
        DeclareLaunchArgument("pointcloud_wait_queue_max_size", default_value="100"),
        DeclareLaunchArgument("pointcloud_min_range_m", default_value="0.3"),
        DeclareLaunchArgument("pointcloud_max_range_m", default_value="30.0"),
        DeclareLaunchArgument("pointcloud_factor_weight", default_value="0.1"),
        DeclareLaunchArgument("enable_lidar_pose_prior_factor", default_value="false"),
        DeclareLaunchArgument("lidar_pose_prior_position_weight", default_value="1.0"),
        DeclareLaunchArgument("lidar_pose_prior_orientation_weight", default_value="1.0"),
        DeclareLaunchArgument("lidar_pose_prior_position_huber_delta_m", default_value="0.25"),
        DeclareLaunchArgument("lidar_pose_prior_orientation_huber_delta_rad", default_value="0.25"),
        DeclareLaunchArgument("lidar_pose_factor_keyframe_stride", default_value="5"),
        DeclareLaunchArgument("lidar_pose_factor_min_points", default_value="32"),
        DeclareLaunchArgument("lidar_pose_factor_max_frame_points", default_value="2000"),
        DeclareLaunchArgument("lidar_pose_factor_max_map_points", default_value="20000"),
        DeclareLaunchArgument("lidar_pose_factor_nearest_distance_m", default_value="0.35"),
        DeclareLaunchArgument("lidar_pose_factor_correction_gain", default_value="0.7"),
        DeclareLaunchArgument("lidar_pose_factor_max_correction_m", default_value="0.25"),
        DeclareLaunchArgument("lidar_pose_factor_max_rotation_rad", default_value="0.08"),
        DeclareLaunchArgument("lidar_pose_factor_robust_kernel_m", default_value="0.15"),
        DeclareLaunchArgument("enable_lidar_plane_normal_factor", default_value="false"),
        DeclareLaunchArgument("lidar_plane_normal_factor_weight", default_value="0.1"),
        DeclareLaunchArgument("lidar_plane_normal_huber_delta_rad", default_value="0.10"),
        DeclareLaunchArgument("enable_voxel_plane_extraction", default_value="false"),
        DeclareLaunchArgument("enable_persistent_plane_map", default_value="true"),
        DeclareLaunchArgument("enable_persistent_point_map", default_value="false"),
        DeclareLaunchArgument(
            "persistent_map_update_requires_accepted_solve", default_value="false"
        ),
        DeclareLaunchArgument("persistent_point_map_nearest_distance_m", default_value="0.35"),
        DeclareLaunchArgument("persistent_point_map_factor_weight", default_value="0.05"),
        DeclareLaunchArgument("persistent_point_map_subsample_stride", default_value="20"),
        DeclareLaunchArgument("persistent_point_map_max_points", default_value="20000"),
        DeclareLaunchArgument("persistent_point_map_max_correspondences", default_value="64"),
        DeclareLaunchArgument("voxel_plane_size_m", default_value="0.5"),
        DeclareLaunchArgument("voxel_plane_min_points", default_value="8"),
        DeclareLaunchArgument("voxel_plane_eigen_ratio", default_value="0.05"),
        DeclareLaunchArgument("voxel_plane_max_inlier_m", default_value="0.1"),
        DeclareLaunchArgument("voxel_plane_max_correspondences", default_value="64"),
        DeclareLaunchArgument("persistent_plane_map_max_planes", default_value="512"),
        DeclareLaunchArgument("persistent_plane_map_match_distance_m", default_value="0.25"),
        DeclareLaunchArgument("persistent_plane_map_min_normal_dot", default_value="0.95"),
        DeclareLaunchArgument("persistent_plane_map_min_observations_for_match", default_value="3"),
        DeclareLaunchArgument("body_frame_id", default_value="imu_link"),
        DeclareLaunchArgument("world_frame_id", default_value="map"),
    ]

    node = Node(
        package="gaussian_lic_tracking",
        executable="continuous_time_node",
        name="continuous_time_node",
        output="screen",
        parameters=[
            {
                "raw_imu_topic": raw_imu_topic,
                "raw_pointcloud_topic": raw_pointcloud_topic,
                "external_odometry_prior_topic": external_odometry_prior_topic,
                "enable_external_odometry_prior": enable_external_odometry_prior,
                "enable_external_odometry_position_factors": enable_external_odometry_position_factors,
                "enable_external_odometry_orientation_factors": enable_external_odometry_orientation_factors,
                "external_odometry_position_factor_weight": external_odometry_position_factor_weight,
                "external_odometry_position_factor_huber_delta_m": external_odometry_position_factor_huber_delta_m,
                "external_odometry_orientation_factor_weight": external_odometry_orientation_factor_weight,
                "external_odometry_orientation_factor_huber_delta_rad": external_odometry_orientation_factor_huber_delta_rad,
                "odometry_topic": odometry_topic,
                "path_topic": path_topic,
                "knot_interval_seconds": knot_interval_seconds,
                "window_knot_count": window_knot_count,
                "marginalize_oldest_count": marginalize_oldest_count,
                "max_iterations_per_step": max_iterations_per_step,
                "imu_info_gyro": imu_info_gyro,
                "imu_info_accel": imu_info_accel,
                "ceres_initial_trust_region_radius": ceres_initial_trust_region_radius,
                "ceres_max_trust_region_radius": ceres_max_trust_region_radius,
                "position_smoothness_weight": position_smoothness_weight,
                "position_smoothness_huber_delta_m": position_smoothness_huber_delta_m,
                "lidar_huber_delta_m": lidar_huber_delta_m,
                "step_period_seconds": step_period_seconds,
                "diagnostic_log_period_steps": diagnostic_log_period_steps,
                "seed_min_imu_count": seed_min_imu_count,
                "hold_gravity_constant": hold_gravity_constant,
                "hold_accel_bias_constant": hold_accel_bias_constant,
                "hold_gyro_bias_constant": hold_gyro_bias_constant,
                "enable_startup_bias_autocal": enable_startup_bias_autocal,
                "imu_linear_acceleration_scale": imu_linear_acceleration_scale,
                "max_position_update_m": max_position_update_m,
                "max_rotation_update_rad": max_rotation_update_rad,
                "update_gate_edge_knot_margin": update_gate_edge_knot_margin,
                "position_extrapolation_damping": position_extrapolation_damping,
                "apply_position_update_on_rotation_reject": apply_position_update_on_rotation_reject,
                "apply_limited_rotation_update": apply_limited_rotation_update,
                "scale_position_with_limited_rotation": scale_position_with_limited_rotation,
                "pointcloud_enable": pointcloud_enable,
                "pointcloud_subsample_stride": pointcloud_subsample_stride,
                "pointcloud_max_points_per_msg": pointcloud_max_points_per_msg,
                "pointcloud_wait_queue_max_size": pointcloud_wait_queue_max_size,
                "pointcloud_min_range_m": pointcloud_min_range_m,
                "pointcloud_max_range_m": pointcloud_max_range_m,
                "pointcloud_factor_weight": pointcloud_factor_weight,
                "enable_lidar_pose_prior_factor": enable_lidar_pose_prior_factor,
                "lidar_pose_prior_position_weight": lidar_pose_prior_position_weight,
                "lidar_pose_prior_orientation_weight": lidar_pose_prior_orientation_weight,
                "lidar_pose_prior_position_huber_delta_m": lidar_pose_prior_position_huber_delta_m,
                "lidar_pose_prior_orientation_huber_delta_rad": lidar_pose_prior_orientation_huber_delta_rad,
                "lidar_pose_factor_keyframe_stride": lidar_pose_factor_keyframe_stride,
                "lidar_pose_factor_min_points": LaunchConfiguration("lidar_pose_factor_min_points"),
                "lidar_pose_factor_max_frame_points": LaunchConfiguration("lidar_pose_factor_max_frame_points"),
                "lidar_pose_factor_max_map_points": LaunchConfiguration("lidar_pose_factor_max_map_points"),
                "lidar_pose_factor_nearest_distance_m": LaunchConfiguration("lidar_pose_factor_nearest_distance_m"),
                "lidar_pose_factor_correction_gain": LaunchConfiguration("lidar_pose_factor_correction_gain"),
                "lidar_pose_factor_max_correction_m": LaunchConfiguration("lidar_pose_factor_max_correction_m"),
                "lidar_pose_factor_max_rotation_rad": LaunchConfiguration("lidar_pose_factor_max_rotation_rad"),
                "lidar_pose_factor_robust_kernel_m": LaunchConfiguration("lidar_pose_factor_robust_kernel_m"),
                "enable_lidar_plane_normal_factor": LaunchConfiguration("enable_lidar_plane_normal_factor"),
                "lidar_plane_normal_factor_weight": LaunchConfiguration("lidar_plane_normal_factor_weight"),
                "lidar_plane_normal_huber_delta_rad": LaunchConfiguration("lidar_plane_normal_huber_delta_rad"),
                "enable_voxel_plane_extraction": LaunchConfiguration("enable_voxel_plane_extraction"),
                "enable_persistent_plane_map": LaunchConfiguration("enable_persistent_plane_map"),
                "enable_persistent_point_map": LaunchConfiguration("enable_persistent_point_map"),
                "persistent_map_update_requires_accepted_solve": LaunchConfiguration(
                    "persistent_map_update_requires_accepted_solve"
                ),
                "persistent_point_map_nearest_distance_m": LaunchConfiguration("persistent_point_map_nearest_distance_m"),
                "persistent_point_map_factor_weight": LaunchConfiguration("persistent_point_map_factor_weight"),
                "persistent_point_map_subsample_stride": LaunchConfiguration("persistent_point_map_subsample_stride"),
                "persistent_point_map_max_points": LaunchConfiguration("persistent_point_map_max_points"),
                "persistent_point_map_max_correspondences": LaunchConfiguration("persistent_point_map_max_correspondences"),
                "voxel_plane_size_m": LaunchConfiguration("voxel_plane_size_m"),
                "voxel_plane_min_points": LaunchConfiguration("voxel_plane_min_points"),
                "voxel_plane_eigen_ratio": LaunchConfiguration("voxel_plane_eigen_ratio"),
                "voxel_plane_max_inlier_m": LaunchConfiguration("voxel_plane_max_inlier_m"),
                "voxel_plane_max_correspondences": LaunchConfiguration("voxel_plane_max_correspondences"),
                "persistent_plane_map_max_planes": LaunchConfiguration("persistent_plane_map_max_planes"),
                "persistent_plane_map_match_distance_m": LaunchConfiguration("persistent_plane_map_match_distance_m"),
                "persistent_plane_map_min_normal_dot": LaunchConfiguration("persistent_plane_map_min_normal_dot"),
                "persistent_plane_map_min_observations_for_match": LaunchConfiguration("persistent_plane_map_min_observations_for_match"),
                "body_frame_id": body_frame_id,
                "world_frame_id": world_frame_id,
            }
        ],
    )

    return LaunchDescription([*declarations, node])
