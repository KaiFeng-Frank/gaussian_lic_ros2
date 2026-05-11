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
    body_frame_id = LaunchConfiguration("body_frame_id")
    world_frame_id = LaunchConfiguration("world_frame_id")

    declarations = [
        DeclareLaunchArgument("raw_imu_topic", default_value="/imu_for_gs"),
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
                "body_frame_id": body_frame_id,
                "world_frame_id": world_frame_id,
            }
        ],
    )

    return LaunchDescription([*declarations, node])
