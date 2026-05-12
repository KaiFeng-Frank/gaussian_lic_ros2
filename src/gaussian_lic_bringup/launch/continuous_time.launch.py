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
    odometry_topic = LaunchConfiguration("odometry_topic")
    path_topic = LaunchConfiguration("path_topic")
    knot_interval_seconds = LaunchConfiguration("knot_interval_seconds")
    window_knot_count = LaunchConfiguration("window_knot_count")
    marginalize_oldest_count = LaunchConfiguration("marginalize_oldest_count")
    max_iterations_per_step = LaunchConfiguration("max_iterations_per_step")
    step_period_seconds = LaunchConfiguration("step_period_seconds")
    seed_min_imu_count = LaunchConfiguration("seed_min_imu_count")
    hold_gravity_constant = LaunchConfiguration("hold_gravity_constant")
    hold_accel_bias_constant = LaunchConfiguration("hold_accel_bias_constant")
    hold_gyro_bias_constant = LaunchConfiguration("hold_gyro_bias_constant")
    max_position_update_m = LaunchConfiguration("max_position_update_m")
    max_rotation_update_rad = LaunchConfiguration("max_rotation_update_rad")
    pointcloud_enable = LaunchConfiguration("pointcloud_enable")
    pointcloud_subsample_stride = LaunchConfiguration("pointcloud_subsample_stride")
    pointcloud_max_points_per_msg = LaunchConfiguration("pointcloud_max_points_per_msg")
    pointcloud_min_range_m = LaunchConfiguration("pointcloud_min_range_m")
    pointcloud_max_range_m = LaunchConfiguration("pointcloud_max_range_m")
    pointcloud_factor_weight = LaunchConfiguration("pointcloud_factor_weight")
    body_frame_id = LaunchConfiguration("body_frame_id")
    world_frame_id = LaunchConfiguration("world_frame_id")

    declarations = [
        DeclareLaunchArgument("raw_imu_topic", default_value="/imu_for_gs"),
        DeclareLaunchArgument(
            "raw_pointcloud_topic", default_value="/points_for_gs"
        ),
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
        DeclareLaunchArgument("max_iterations_per_step", default_value="12"),
        DeclareLaunchArgument("step_period_seconds", default_value="0.10"),
        DeclareLaunchArgument("seed_min_imu_count", default_value="25"),
        DeclareLaunchArgument("hold_gravity_constant", default_value="true"),
        DeclareLaunchArgument("hold_accel_bias_constant", default_value="false"),
        DeclareLaunchArgument("hold_gyro_bias_constant", default_value="false"),
        DeclareLaunchArgument("max_position_update_m", default_value="2.0"),
        DeclareLaunchArgument("max_rotation_update_rad", default_value="0.50"),
        DeclareLaunchArgument("pointcloud_enable", default_value="true"),
        DeclareLaunchArgument("pointcloud_subsample_stride", default_value="50"),
        DeclareLaunchArgument("pointcloud_max_points_per_msg", default_value="256"),
        DeclareLaunchArgument("pointcloud_min_range_m", default_value="0.3"),
        DeclareLaunchArgument("pointcloud_max_range_m", default_value="30.0"),
        DeclareLaunchArgument("pointcloud_factor_weight", default_value="1.0"),
        DeclareLaunchArgument("enable_voxel_plane_extraction", default_value="false"),
        DeclareLaunchArgument("enable_persistent_plane_map", default_value="true"),
        DeclareLaunchArgument("voxel_plane_size_m", default_value="0.5"),
        DeclareLaunchArgument("voxel_plane_min_points", default_value="8"),
        DeclareLaunchArgument("voxel_plane_eigen_ratio", default_value="0.05"),
        DeclareLaunchArgument("voxel_plane_max_inlier_m", default_value="0.1"),
        DeclareLaunchArgument("voxel_plane_max_correspondences", default_value="64"),
        DeclareLaunchArgument("persistent_plane_map_max_planes", default_value="512"),
        DeclareLaunchArgument("persistent_plane_map_match_distance_m", default_value="0.25"),
        DeclareLaunchArgument("persistent_plane_map_min_normal_dot", default_value="0.95"),
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
                "odometry_topic": odometry_topic,
                "path_topic": path_topic,
                "knot_interval_seconds": knot_interval_seconds,
                "window_knot_count": window_knot_count,
                "marginalize_oldest_count": marginalize_oldest_count,
                "max_iterations_per_step": max_iterations_per_step,
                "step_period_seconds": step_period_seconds,
                "seed_min_imu_count": seed_min_imu_count,
                "hold_gravity_constant": hold_gravity_constant,
                "hold_accel_bias_constant": hold_accel_bias_constant,
                "hold_gyro_bias_constant": hold_gyro_bias_constant,
                "max_position_update_m": max_position_update_m,
                "max_rotation_update_rad": max_rotation_update_rad,
                "pointcloud_enable": pointcloud_enable,
                "pointcloud_subsample_stride": pointcloud_subsample_stride,
                "pointcloud_max_points_per_msg": pointcloud_max_points_per_msg,
                "pointcloud_min_range_m": pointcloud_min_range_m,
                "pointcloud_max_range_m": pointcloud_max_range_m,
                "pointcloud_factor_weight": pointcloud_factor_weight,
                "enable_voxel_plane_extraction": LaunchConfiguration("enable_voxel_plane_extraction"),
                "enable_persistent_plane_map": LaunchConfiguration("enable_persistent_plane_map"),
                "voxel_plane_size_m": LaunchConfiguration("voxel_plane_size_m"),
                "voxel_plane_min_points": LaunchConfiguration("voxel_plane_min_points"),
                "voxel_plane_eigen_ratio": LaunchConfiguration("voxel_plane_eigen_ratio"),
                "voxel_plane_max_inlier_m": LaunchConfiguration("voxel_plane_max_inlier_m"),
                "voxel_plane_max_correspondences": LaunchConfiguration("voxel_plane_max_correspondences"),
                "persistent_plane_map_max_planes": LaunchConfiguration("persistent_plane_map_max_planes"),
                "persistent_plane_map_match_distance_m": LaunchConfiguration("persistent_plane_map_match_distance_m"),
                "persistent_plane_map_min_normal_dot": LaunchConfiguration("persistent_plane_map_min_normal_dot"),
                "body_frame_id": body_frame_id,
                "world_frame_id": world_frame_id,
            }
        ],
    )

    return LaunchDescription([*declarations, node])
