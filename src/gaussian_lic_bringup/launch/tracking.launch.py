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
    rendered_image_topic = LaunchConfiguration("rendered_image_topic")
    gaussian_map_topic = LaunchConfiguration("gaussian_map_topic")
    enable_lio_factor = LaunchConfiguration("enable_lio_factor")
    enable_lidar_deskew = LaunchConfiguration("enable_lidar_deskew")
    enable_visual_factor = LaunchConfiguration("enable_visual_factor")
    enable_gaussian_snapshot = LaunchConfiguration("enable_gaussian_snapshot")
    enable_sliding_window_optimizer = LaunchConfiguration("enable_sliding_window_optimizer")
    sliding_window_bias_weight = LaunchConfiguration("sliding_window_bias_weight")
    lidar_time_field = LaunchConfiguration("lidar_time_field")
    lidar_time_unit = LaunchConfiguration("lidar_time_unit")
    lidar_time_mode = LaunchConfiguration("lidar_time_mode")

    return LaunchDescription(
        [
            DeclareLaunchArgument("publish_tf", default_value="false"),
            DeclareLaunchArgument("raw_image_topic", default_value="/camera/image"),
            DeclareLaunchArgument("raw_camera_info_topic", default_value="/camera/camera_info"),
            DeclareLaunchArgument("raw_depth_topic", default_value="/camera/depth"),
            DeclareLaunchArgument("raw_pointcloud_topic", default_value="/livox/lidar"),
            DeclareLaunchArgument("raw_imu_topic", default_value="/imu"),
            DeclareLaunchArgument("rendered_image_topic", default_value="/gaussian_lic/rendered_image"),
            DeclareLaunchArgument("gaussian_map_topic", default_value="/gaussian_lic/gaussian_map"),
            DeclareLaunchArgument("enable_lio_factor", default_value="true"),
            DeclareLaunchArgument("enable_lidar_deskew", default_value="true"),
            DeclareLaunchArgument("lidar_time_field", default_value="auto"),
            DeclareLaunchArgument("lidar_time_unit", default_value="auto"),
            DeclareLaunchArgument("lidar_time_mode", default_value="auto"),
            DeclareLaunchArgument("enable_visual_factor", default_value="true"),
            DeclareLaunchArgument("enable_gaussian_snapshot", default_value="true"),
            DeclareLaunchArgument("enable_sliding_window_optimizer", default_value="false"),
            DeclareLaunchArgument("sliding_window_bias_weight", default_value="1.0"),
            Node(
                package="gaussian_lic_tracking",
                executable="tracking_node",
                name="tracking_node",
                output="screen",
                parameters=[
                    {
                        "publish_tf": publish_tf,
                        "sensor_qos_reliability": "best_effort",
                        "sensor_qos_depth": 5,
                        "world_frame": "map",
                        "child_frame": "base_link",
                        "raw_image_topic": raw_image_topic,
                        "raw_camera_info_topic": raw_camera_info_topic,
                        "raw_depth_topic": raw_depth_topic,
                        "raw_pointcloud_topic": raw_pointcloud_topic,
                        "raw_imu_topic": raw_imu_topic,
                        "rendered_image_topic": rendered_image_topic,
                        "gaussian_map_topic": gaussian_map_topic,
                        "enable_lio_factor": enable_lio_factor,
                        "enable_lidar_deskew": enable_lidar_deskew,
                        "lidar_time_field": lidar_time_field,
                        "lidar_time_unit": lidar_time_unit,
                        "lidar_time_mode": lidar_time_mode,
                        "enable_visual_factor": enable_visual_factor,
                        "enable_gaussian_snapshot": enable_gaussian_snapshot,
                        "enable_sliding_window_optimizer": enable_sliding_window_optimizer,
                        "sliding_window_bias_weight": sliding_window_bias_weight,
                    }
                ],
            ),
        ]
    )
