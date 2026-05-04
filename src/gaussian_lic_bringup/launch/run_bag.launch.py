# SPDX-License-Identifier: GPL-3.0-or-later

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    bag = LaunchConfiguration("bag")
    config = LaunchConfiguration("config")
    play_bag = LaunchConfiguration("play_bag")
    loop_bag = LaunchConfiguration("loop_bag")
    stub_mode = LaunchConfiguration("stub_mode")
    synthetic_input = LaunchConfiguration("synthetic_input")
    frontend_adapter = LaunchConfiguration("frontend_adapter")
    adapter_identity_pose_fallback = LaunchConfiguration("adapter_identity_pose_fallback")
    adapter_imu_pose_fallback = LaunchConfiguration("adapter_imu_pose_fallback")
    adapter_pointcloud_transform_profile = LaunchConfiguration("adapter_pointcloud_transform_profile")
    synthetic_pose_output_mode = LaunchConfiguration("synthetic_pose_output_mode")
    synthetic_pointcloud_color_mode = LaunchConfiguration("synthetic_pointcloud_color_mode")
    synthetic_point_color_rgb = LaunchConfiguration("synthetic_point_color_rgb")
    synthetic_image_color_rgb = LaunchConfiguration("synthetic_image_color_rgb")
    synthetic_publish_depth = LaunchConfiguration("synthetic_publish_depth")
    enable_torch_camera_conversion = LaunchConfiguration("enable_torch_camera_conversion")
    enable_torch_gaussian_init = LaunchConfiguration("enable_torch_gaussian_init")
    enable_torch_gaussian_extend = LaunchConfiguration("enable_torch_gaussian_extend")
    enable_torch_gaussian_optimization = LaunchConfiguration("enable_torch_gaussian_optimization")
    torch_gaussian_optimization_steps = LaunchConfiguration("torch_gaussian_optimization_steps")
    torch_gaussian_optimization_max_samples = LaunchConfiguration("torch_gaussian_optimization_max_samples")
    enable_torch_gaussian_pruning = LaunchConfiguration("enable_torch_gaussian_pruning")
    torch_gaussian_prune_min_opacity = LaunchConfiguration("torch_gaussian_prune_min_opacity")
    torch_gaussian_max_foreground = LaunchConfiguration("torch_gaussian_max_foreground")
    torch_gaussian_device = LaunchConfiguration("torch_gaussian_device")
    sensor_qos_reliability = LaunchConfiguration("sensor_qos_reliability")
    sensor_qos_history = LaunchConfiguration("sensor_qos_history")
    sensor_qos_depth = LaunchConfiguration("sensor_qos_depth")
    require_depth_topic = LaunchConfiguration("require_depth_topic")
    render_mode = LaunchConfiguration("render_mode")
    rendered_image_mode = LaunchConfiguration("rendered_image_mode")
    publish_tf = LaunchConfiguration("publish_tf")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_composition = LaunchConfiguration("use_composition")
    rviz = LaunchConfiguration("rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    default_config = PathJoinSubstitution([
        FindPackageShare("gaussian_lic_bringup"),
        "config",
        "default.yaml",
    ])
    default_rviz_config = PathJoinSubstitution([
        FindPackageShare("gaussian_lic_bringup"),
        "rviz",
        "gaussian_lic.rviz",
    ])
    mapping_parameters = [
        config,
        {
            "use_sim_time": use_sim_time,
            "enable_torch_camera_conversion": enable_torch_camera_conversion,
            "enable_torch_gaussian_init": enable_torch_gaussian_init,
            "enable_torch_gaussian_extend": enable_torch_gaussian_extend,
            "enable_torch_gaussian_optimization": enable_torch_gaussian_optimization,
            "torch_gaussian_optimization_steps": torch_gaussian_optimization_steps,
            "torch_gaussian_optimization_max_samples": torch_gaussian_optimization_max_samples,
            "enable_torch_gaussian_pruning": enable_torch_gaussian_pruning,
            "torch_gaussian_prune_min_opacity": torch_gaussian_prune_min_opacity,
            "torch_gaussian_max_foreground": torch_gaussian_max_foreground,
            "torch_gaussian_device": torch_gaussian_device,
            "sensor_qos_reliability": sensor_qos_reliability,
            "sensor_qos_history": sensor_qos_history,
            "sensor_qos_depth": sensor_qos_depth,
            "require_depth_topic": require_depth_topic,
            "render_mode": render_mode,
            "rendered_image_mode": rendered_image_mode,
            "publish_tf": publish_tf,
        },
    ]
    synthetic_parameters = [
        config,
        {
            "use_sim_time": use_sim_time,
            "pointcloud_color_mode": synthetic_pointcloud_color_mode,
            "point_color_rgb": synthetic_point_color_rgb,
            "image_color_rgb": synthetic_image_color_rgb,
            "publish_depth": synthetic_publish_depth,
            "pose_output_mode": synthetic_pose_output_mode,
        },
    ]
    synthetic_raw_parameters = [
        config,
        {
            "use_sim_time": use_sim_time,
            "pointcloud_topic": "/livox/lidar",
            "pose_topic": "/gaussian_lic/frontend/pose",
            "odometry_topic": "/gaussian_lic/frontend/input_odometry",
            "image_topic": "/camera/image",
            "camera_info_topic": "/camera/camera_info",
            "depth_topic": "/camera/depth",
            "imu_topic": "/imu",
            "pointcloud_color_mode": synthetic_pointcloud_color_mode,
            "point_color_rgb": synthetic_point_color_rgb,
            "image_color_rgb": synthetic_image_color_rgb,
            "publish_depth": synthetic_publish_depth,
            "pose_output_mode": synthetic_pose_output_mode,
        },
    ]
    adapter_parameters = [
        config,
        {
            "use_sim_time": use_sim_time,
            "sensor_qos_reliability": sensor_qos_reliability,
            "sensor_qos_history": sensor_qos_history,
            "sensor_qos_depth": sensor_qos_depth,
            "identity_pose_fallback": adapter_identity_pose_fallback,
            "imu_pose_fallback": adapter_imu_pose_fallback,
            "pointcloud_transform_profile": adapter_pointcloud_transform_profile,
            "publish_tf": publish_tf,
        },
    ]
    native_node_condition = IfCondition(PythonExpression([
        "'", stub_mode, "'.lower() == 'false' and '", use_composition, "'.lower() == 'false'",
    ]))
    composition_condition = IfCondition(PythonExpression([
        "'", stub_mode, "'.lower() == 'false' and '", use_composition, "'.lower() == 'true'",
    ]))
    play_bag_once_condition = IfCondition(PythonExpression([
        "'", play_bag, "'.lower() == 'true' and '", loop_bag, "'.lower() == 'false'",
    ]))
    play_bag_loop_condition = IfCondition(PythonExpression([
        "'", play_bag, "'.lower() == 'true' and '", loop_bag, "'.lower() == 'true'",
    ]))
    frontend_adapter_condition = IfCondition(frontend_adapter)
    synthetic_mapper_condition = IfCondition(PythonExpression([
        "'", synthetic_input, "'.lower() == 'true' and '", frontend_adapter, "'.lower() == 'false'",
    ]))
    synthetic_raw_condition = IfCondition(PythonExpression([
        "'", synthetic_input, "'.lower() == 'true' and '", frontend_adapter, "'.lower() == 'true'",
    ]))

    return LaunchDescription([
        DeclareLaunchArgument("bag", default_value="", description="rosbag2 directory to replay"),
        DeclareLaunchArgument("config", default_value=default_config, description="Parameter YAML file"),
        DeclareLaunchArgument("play_bag", default_value="false", description="Replay the bag argument"),
        DeclareLaunchArgument("loop_bag", default_value="false", description="Loop rosbag2 playback until launch shutdown"),
        DeclareLaunchArgument("stub_mode", default_value="true", description="Run smoke-test nodes instead of native mapper"),
        DeclareLaunchArgument("synthetic_input", default_value="false", description="Publish synthetic synchronized mapper inputs"),
        DeclareLaunchArgument(
            "frontend_adapter",
            default_value="false",
            description="Run the LIC2 frontend contract adapter between raw sensor topics and mapper topics",
        ),
        DeclareLaunchArgument(
            "adapter_identity_pose_fallback",
            default_value="false",
            description="Let the adapter publish identity poses from point-cloud stamps when no odometry is available",
        ),
        DeclareLaunchArgument(
            "adapter_imu_pose_fallback",
            default_value="false",
            description="Let the adapter integrate IMU gyro orientation for pose fallback when no odometry is available",
        ),
        DeclareLaunchArgument(
            "adapter_pointcloud_transform_profile",
            default_value="identity",
            description="Static adapter pointcloud transform profile: identity or fastlivo2",
        ),
        DeclareLaunchArgument(
            "synthetic_pose_output_mode",
            default_value="pose_stamped",
            description="Synthetic pose output mode: pose_stamped, odometry, both, or none",
        ),
        DeclareLaunchArgument(
            "synthetic_pointcloud_color_mode",
            default_value="packed_rgb",
            description="Synthetic PointCloud2 color mode: packed_rgb, rgb_fields, or none",
        ),
        DeclareLaunchArgument(
            "synthetic_point_color_rgb",
            default_value="255,32,16",
            description="Synthetic point RGB as red,green,blue",
        ),
        DeclareLaunchArgument(
            "synthetic_image_color_rgb",
            default_value="0,0,0",
            description="Synthetic image RGB as red,green,blue",
        ),
        DeclareLaunchArgument(
            "synthetic_publish_depth",
            default_value="true",
            description="Publish synthetic depth_topic frames",
        ),
        DeclareLaunchArgument(
            "enable_torch_camera_conversion",
            default_value="false",
            description="Enable optional TorchCamera creation when mapping_node was built with torch support",
        ),
        DeclareLaunchArgument(
            "enable_torch_gaussian_init",
            default_value="false",
            description="Enable optional Torch Gaussian map initialization when mapping_node was built with torch support",
        ),
        DeclareLaunchArgument(
            "enable_torch_gaussian_extend",
            default_value="true",
            description="Append new keyframe pending points to the Torch Gaussian map after initialization",
        ),
        DeclareLaunchArgument(
            "enable_torch_gaussian_optimization",
            default_value="false",
            description="Run the optional Torch photometric Gaussian tensor update on keyframes",
        ),
        DeclareLaunchArgument(
            "torch_gaussian_optimization_steps",
            default_value="0",
            description="Photometric optimizer steps per keyframe when enabled",
        ),
        DeclareLaunchArgument(
            "torch_gaussian_optimization_max_samples",
            default_value="4096",
            description="Maximum visible foreground Gaussians supervised per keyframe optimization",
        ),
        DeclareLaunchArgument(
            "enable_torch_gaussian_pruning",
            default_value="false",
            description="Prune low-opacity or excess foreground Gaussians after keyframe updates",
        ),
        DeclareLaunchArgument(
            "torch_gaussian_prune_min_opacity",
            default_value="0.005",
            description="Minimum sigmoid opacity retained by Torch Gaussian pruning",
        ),
        DeclareLaunchArgument(
            "torch_gaussian_max_foreground",
            default_value="0",
            description="Maximum foreground Gaussians retained by pruning; 0 disables count cap",
        ),
        DeclareLaunchArgument(
            "torch_gaussian_device",
            default_value="cpu",
            description="Torch device for Gaussian initialization: cpu, cuda, cuda:0, or auto",
        ),
        DeclareLaunchArgument(
            "sensor_qos_reliability",
            default_value="best_effort",
            description="Input sensor QoS reliability: best_effort or reliable",
        ),
        DeclareLaunchArgument(
            "sensor_qos_history",
            default_value="keep_last",
            description="Input sensor QoS history: keep_last or keep_all",
        ),
        DeclareLaunchArgument(
            "sensor_qos_depth",
            default_value="5",
            description="Input sensor QoS keep-last depth",
        ),
        DeclareLaunchArgument(
            "require_depth_topic",
            default_value="true",
            description="Require depth_topic in frame synchronization; false uses sparse point-projected depth",
        ),
        DeclareLaunchArgument(
            "render_mode",
            default_value="debug_cpu",
            description="Rendered output mode: debug_cpu, debug_input, rasterizer, or off",
        ),
        DeclareLaunchArgument(
            "rendered_image_mode",
            default_value="",
            description="Deprecated alias: projected_map/input/auto. Prefer render_mode.",
        ),
        DeclareLaunchArgument(
            "publish_tf",
            default_value="false",
            description="Broadcast world_frame -> camera_frame TF from converted poses",
        ),
        DeclareLaunchArgument("use_sim_time", default_value="true", description="Use /clock from rosbag2"),
        DeclareLaunchArgument(
            "use_composition",
            default_value="false",
            description="Load mapping_node as an rclcpp component in a component_container_mt",
        ),
        DeclareLaunchArgument("rviz", default_value="false", description="Start RViz2 with Gaussian-LIC displays"),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz_config, description="RViz2 config file"),

        ExecuteProcess(
            cmd=["ros2", "bag", "play", bag, "--clock"],
            output="screen",
            condition=play_bag_once_condition,
        ),

        ExecuteProcess(
            cmd=["ros2", "bag", "play", bag, "--clock", "--loop"],
            output="screen",
            condition=play_bag_loop_condition,
        ),

        Node(
            package="gaussian_lic_tools",
            executable="topic_probe",
            name="topic_probe",
            output="screen",
            parameters=[config, {"use_sim_time": use_sim_time}],
            condition=IfCondition(stub_mode),
        ),

        Node(
            package="gaussian_lic_tools",
            executable="status_stub",
            name="status_stub",
            output="screen",
            parameters=[config, {"use_sim_time": use_sim_time}],
            condition=IfCondition(stub_mode),
        ),

        Node(
            package="gaussian_lic_mapping",
            executable="mapping_node",
            name="mapping_node",
            output="screen",
            parameters=mapping_parameters,
            condition=native_node_condition,
        ),

        Node(
            package="gaussian_lic_frontend",
            executable="lic2_contract_adapter",
            name="lic2_contract_adapter",
            output="screen",
            parameters=adapter_parameters,
            condition=frontend_adapter_condition,
        ),

        ComposableNodeContainer(
            package="rclcpp_components",
            executable="component_container_mt",
            name="gaussian_lic_container",
            namespace="",
            output="screen",
            composable_node_descriptions=[
                ComposableNode(
                    package="gaussian_lic_mapping",
                    plugin="MappingNode",
                    name="mapping_node",
                    parameters=mapping_parameters,
                ),
            ],
            condition=composition_condition,
        ),

        Node(
            package="gaussian_lic_tools",
            executable="synthetic_gs_frame_pub",
            name="synthetic_gs_frame_pub",
            output="screen",
            parameters=synthetic_parameters,
            condition=synthetic_mapper_condition,
        ),

        Node(
            package="gaussian_lic_tools",
            executable="synthetic_gs_frame_pub",
            name="synthetic_raw_frame_pub",
            output="screen",
            parameters=synthetic_raw_parameters,
            condition=synthetic_raw_condition,
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": use_sim_time}],
            condition=IfCondition(rviz),
        ),
    ])
