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
    adapter_rotate_pointcloud_with_imu_pose = LaunchConfiguration("adapter_rotate_pointcloud_with_imu_pose")
    adapter_pointcloud_use_stamp_imu_orientation = LaunchConfiguration("adapter_pointcloud_use_stamp_imu_orientation")
    adapter_imu_orientation_history_size = LaunchConfiguration("adapter_imu_orientation_history_size")
    adapter_sync_image_to_pointcloud = LaunchConfiguration("adapter_sync_image_to_pointcloud")
    adapter_visual_sync_policy = LaunchConfiguration("adapter_visual_sync_policy")
    adapter_pointcloud_transform_profile = LaunchConfiguration("adapter_pointcloud_transform_profile")
    adapter_pointcloud_filter_min_z = LaunchConfiguration("adapter_pointcloud_filter_min_z")
    adapter_pointcloud_filter_max_z = LaunchConfiguration("adapter_pointcloud_filter_max_z")
    adapter_pointcloud_filter_min_points = LaunchConfiguration("adapter_pointcloud_filter_min_points")
    adapter_raw_pointcloud_topic = LaunchConfiguration("adapter_raw_pointcloud_topic")
    livox_custom_bridge = LaunchConfiguration("livox_custom_bridge")
    livox_custom_topic = LaunchConfiguration("livox_custom_topic")
    livox_pointcloud_topic = LaunchConfiguration("livox_pointcloud_topic")
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
    torch_gaussian_optimization_sampling = LaunchConfiguration("torch_gaussian_optimization_sampling")
    torch_gaussian_optimization_seed = LaunchConfiguration("torch_gaussian_optimization_seed")
    enable_torch_gaussian_pruning = LaunchConfiguration("enable_torch_gaussian_pruning")
    enable_torch_gaussian_densification = LaunchConfiguration("enable_torch_gaussian_densification")
    torch_gaussian_prune_min_opacity = LaunchConfiguration("torch_gaussian_prune_min_opacity")
    torch_gaussian_max_foreground = LaunchConfiguration("torch_gaussian_max_foreground")
    torch_gaussian_prune_count_policy = LaunchConfiguration("torch_gaussian_prune_count_policy")
    torch_gaussian_prune_max_world_scale = LaunchConfiguration("torch_gaussian_prune_max_world_scale")
    enable_torch_gaussian_extend_visibility_filter = LaunchConfiguration("enable_torch_gaussian_extend_visibility_filter")
    torch_gaussian_extend_alpha_threshold = LaunchConfiguration("torch_gaussian_extend_alpha_threshold")
    torch_gaussian_opacity_reset_interval = LaunchConfiguration("torch_gaussian_opacity_reset_interval")
    torch_gaussian_device = LaunchConfiguration("torch_gaussian_device")
    publish_gaussian_map = LaunchConfiguration("publish_gaussian_map")
    gaussian_map_publish_min_interval_sec = LaunchConfiguration("gaussian_map_publish_min_interval_sec")
    gaussian_map_publish_on_empty_extend = LaunchConfiguration("gaussian_map_publish_on_empty_extend")
    save_map_render_evaluation = LaunchConfiguration("save_map_render_evaluation")
    test_frame_stride = LaunchConfiguration("test_frame_stride")
    sensor_qos_reliability = LaunchConfiguration("sensor_qos_reliability")
    sensor_qos_history = LaunchConfiguration("sensor_qos_history")
    sensor_qos_depth = LaunchConfiguration("sensor_qos_depth")
    mapping_qos_streams = ("pointcloud", "pose", "image", "camera_info", "depth", "imu")
    adapter_qos_streams = (
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
    )
    qos_launch_configs = {
        f"{stream}_qos_{suffix}": LaunchConfiguration(f"{stream}_qos_{suffix}")
        for stream in sorted(set(mapping_qos_streams + adapter_qos_streams))
        for suffix in ("reliability", "history", "depth")
    }
    qos_launch_arguments = [
        DeclareLaunchArgument(
            f"{stream}_qos_{suffix}",
            default_value={
                "reliability": "best_effort",
                "history": "keep_last",
                "depth": "5",
            }[suffix],
            description=f"{stream} QoS {suffix}",
        )
        for stream in sorted(set(mapping_qos_streams + adapter_qos_streams))
        for suffix in ("reliability", "history", "depth")
    ]
    require_depth_topic = LaunchConfiguration("require_depth_topic")
    sync_anchor_stream = LaunchConfiguration("sync_anchor_stream")
    render_mode = LaunchConfiguration("render_mode")
    rendered_feedback_topic = LaunchConfiguration("rendered_feedback_topic")
    rendered_feedback_source_stream = LaunchConfiguration("rendered_feedback_source_stream")
    publish_rendered_feedback_before_update = LaunchConfiguration(
        "publish_rendered_feedback_before_update"
    )
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
            "torch_gaussian_optimization_sampling": torch_gaussian_optimization_sampling,
            "torch_gaussian_optimization_seed": torch_gaussian_optimization_seed,
            "enable_torch_gaussian_pruning": enable_torch_gaussian_pruning,
            "enable_torch_gaussian_densification": enable_torch_gaussian_densification,
            "torch_gaussian_prune_min_opacity": torch_gaussian_prune_min_opacity,
            "torch_gaussian_max_foreground": torch_gaussian_max_foreground,
            "torch_gaussian_prune_count_policy": torch_gaussian_prune_count_policy,
            "torch_gaussian_prune_max_world_scale": torch_gaussian_prune_max_world_scale,
            "enable_torch_gaussian_extend_visibility_filter": enable_torch_gaussian_extend_visibility_filter,
            "torch_gaussian_extend_alpha_threshold": torch_gaussian_extend_alpha_threshold,
            "torch_gaussian_opacity_reset_interval": torch_gaussian_opacity_reset_interval,
            "torch_gaussian_device": torch_gaussian_device,
            "publish_gaussian_map": publish_gaussian_map,
            "gaussian_map_publish_min_interval_sec": gaussian_map_publish_min_interval_sec,
            "gaussian_map_publish_on_empty_extend": gaussian_map_publish_on_empty_extend,
            "save_map_render_evaluation": save_map_render_evaluation,
            "test_frame_stride": test_frame_stride,
            "sensor_qos_reliability": sensor_qos_reliability,
            "sensor_qos_history": sensor_qos_history,
            "sensor_qos_depth": sensor_qos_depth,
            **{
                f"{stream}_qos_{suffix}": qos_launch_configs[f"{stream}_qos_{suffix}"]
                for stream in mapping_qos_streams
                for suffix in ("reliability", "history", "depth")
            },
            "require_depth_topic": require_depth_topic,
            "sync_anchor_stream": sync_anchor_stream,
            "render_mode": render_mode,
            "rendered_feedback_topic": rendered_feedback_topic,
            "rendered_feedback_source_stream": rendered_feedback_source_stream,
            "publish_rendered_feedback_before_update": publish_rendered_feedback_before_update,
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
            **{
                f"{stream}_qos_{suffix}": qos_launch_configs[f"{stream}_qos_{suffix}"]
                for stream in adapter_qos_streams
                for suffix in ("reliability", "history", "depth")
            },
            "raw_pointcloud_topic": adapter_raw_pointcloud_topic,
            "identity_pose_fallback": adapter_identity_pose_fallback,
            "imu_pose_fallback": adapter_imu_pose_fallback,
            "rotate_pointcloud_with_imu_pose": adapter_rotate_pointcloud_with_imu_pose,
            "pointcloud_use_stamp_imu_orientation": adapter_pointcloud_use_stamp_imu_orientation,
            "imu_orientation_history_size": adapter_imu_orientation_history_size,
            "sync_image_to_pointcloud": adapter_sync_image_to_pointcloud,
            "visual_sync_policy": adapter_visual_sync_policy,
            "pointcloud_transform_profile": adapter_pointcloud_transform_profile,
            "pointcloud_filter_min_z": adapter_pointcloud_filter_min_z,
            "pointcloud_filter_max_z": adapter_pointcloud_filter_max_z,
            "pointcloud_filter_min_points": adapter_pointcloud_filter_min_points,
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
            "adapter_rotate_pointcloud_with_imu_pose",
            default_value="true",
            description="Rotate adapter point clouds into the IMU fallback world frame, matching the ROS1 mapper-contract converter",
        ),
        DeclareLaunchArgument(
            "adapter_pointcloud_use_stamp_imu_orientation",
            default_value="true",
            description="Use the latest integrated IMU orientation at or before each point-cloud stamp instead of callback-time latest orientation",
        ),
        DeclareLaunchArgument(
            "adapter_imu_orientation_history_size",
            default_value="50000",
            description="Maximum integrated IMU orientation samples retained for point-cloud stamp lookup",
        ),
        DeclareLaunchArgument(
            "adapter_sync_image_to_pointcloud",
            default_value="false",
            description="Re-stamp the latest raw image/camera_info to each point-cloud stamp before mapper output",
        ),
        DeclareLaunchArgument(
            "adapter_visual_sync_policy",
            default_value="latest_before",
            description="Visual sync policy when adapter_sync_image_to_pointcloud is true: latest_before, nearest, or latest",
        ),
        DeclareLaunchArgument(
            "adapter_pointcloud_transform_profile",
            default_value="identity",
            description="Static adapter pointcloud transform profile: identity or fastlivo2",
        ),
        DeclareLaunchArgument(
            "adapter_pointcloud_filter_min_z",
            default_value="-1.7976931348623157e+308",
            description="Drop adapter point-cloud samples with transformed z <= this value; huge negative disables.",
        ),
        DeclareLaunchArgument(
            "adapter_pointcloud_filter_max_z",
            default_value="0.0",
            description="Drop adapter point-cloud samples with transformed z above this value; 0 disables.",
        ),
        DeclareLaunchArgument(
            "adapter_pointcloud_filter_min_points",
            default_value="0",
            description="Drop the whole adapter point cloud if fewer samples survive filtering; 0 disables.",
        ),
        DeclareLaunchArgument(
            "adapter_raw_pointcloud_topic",
            default_value="/livox/lidar",
            description="Raw PointCloud2 topic consumed by the LIC2 contract adapter",
        ),
        DeclareLaunchArgument(
            "livox_custom_bridge",
            default_value="false",
            description="Bridge livox_ros_driver2/CustomMsg packets to PointCloud2 before the adapter",
        ),
        DeclareLaunchArgument(
            "livox_custom_topic",
            default_value="/livox/lidar",
            description="Livox CustomMsg topic consumed by livox_custom_to_pointcloud2",
        ),
        DeclareLaunchArgument(
            "livox_pointcloud_topic",
            default_value="/livox/lidar/points",
            description="PointCloud2 topic published by livox_custom_to_pointcloud2",
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
            default_value="true",
            description="Enable optional TorchCamera creation when mapping_node was built with torch support",
        ),
        DeclareLaunchArgument(
            "enable_torch_gaussian_init",
            default_value="true",
            description="Enable optional Torch Gaussian map initialization when mapping_node was built with torch support",
        ),
        DeclareLaunchArgument(
            "enable_torch_gaussian_extend",
            default_value="true",
            description="Append new keyframe pending points to the Torch Gaussian map after initialization",
        ),
        DeclareLaunchArgument(
            "enable_torch_gaussian_extend_visibility_filter",
            default_value="true",
            description="Filter Torch Gaussian extension points to current-view alpha holes, matching upstream Gaussian-LIC extend()",
        ),
        DeclareLaunchArgument(
            "torch_gaussian_extend_alpha_threshold",
            default_value="0.99",
            description="Rendered alpha threshold below which pending points may be inserted during Torch Gaussian extension",
        ),
        DeclareLaunchArgument(
            "enable_torch_gaussian_optimization",
            default_value="true",
            description="Run the optional Torch photometric Gaussian tensor update on keyframes",
        ),
        DeclareLaunchArgument(
            "torch_gaussian_optimization_steps",
            default_value="100",
            description="Max accumulated train-frame optimizer samples per keyframe when enabled",
        ),
        DeclareLaunchArgument(
            "torch_gaussian_optimization_max_samples",
            default_value="4096",
            description="Maximum visible foreground Gaussians supervised per keyframe optimization",
        ),
        DeclareLaunchArgument(
            "torch_gaussian_optimization_sampling",
            default_value="upstream_random",
            description="Training-frame optimization sampling: upstream_random, even, or latest_even",
        ),
        DeclareLaunchArgument(
            "torch_gaussian_optimization_seed",
            default_value="20260505",
            description="Random seed for upstream_random optimization sampling; 0 uses std::random_device",
        ),
        DeclareLaunchArgument(
            "enable_torch_gaussian_pruning",
            default_value="true",
            description="Prune low-opacity or excess foreground Gaussians after keyframe updates",
        ),
        DeclareLaunchArgument(
            "enable_torch_gaussian_densification",
            default_value="false",
            description="Enable gradient-aware Gaussian densification after keyframe updates",
        ),
        DeclareLaunchArgument(
            "torch_gaussian_prune_min_opacity",
            default_value="0.005",
            description="Minimum sigmoid opacity retained by Torch Gaussian pruning",
        ),
        DeclareLaunchArgument(
            "torch_gaussian_max_foreground",
            default_value="1500000",
            description="Maximum foreground Gaussians retained by pruning; 0 disables count cap",
        ),
        DeclareLaunchArgument(
            "torch_gaussian_prune_count_policy",
            default_value="uniform",
            description="Foreground count-cap policy: opacity keeps highest-opacity Gaussians; uniform preserves insertion-order spatial coverage",
        ),
        DeclareLaunchArgument(
            "torch_gaussian_prune_max_world_scale",
            default_value="0.0",
            description="Maximum foreground Gaussian world scale retained by pruning; 0 disables this gate",
        ),
        DeclareLaunchArgument(
            "torch_gaussian_opacity_reset_interval",
            default_value="0",
            description="Optimization steps between foreground opacity resets; 0 disables this non-upstream recovery heuristic",
        ),
        DeclareLaunchArgument(
            "torch_gaussian_device",
            default_value="cuda",
            description="Torch device for Gaussian initialization: cpu, cuda, cuda:0, or auto",
        ),
        DeclareLaunchArgument(
            "publish_gaussian_map",
            default_value="true",
            description="Publish full GaussianArray chunks for visualization; strict metric runs can disable this to avoid GPU-to-CPU map transfer overhead",
        ),
        DeclareLaunchArgument(
            "gaussian_map_publish_min_interval_sec",
            default_value="0.0",
            description="Minimum simulated-time interval between full GaussianArray publications; 0 disables throttling.",
        ),
        DeclareLaunchArgument(
            "gaussian_map_publish_on_empty_extend",
            default_value="true",
            description="Publish GaussianArray after keyframes that insert no new Gaussians. Disable for mapper-feedback runs to avoid repeated full-map chunks.",
        ),
        DeclareLaunchArgument(
            "save_map_render_evaluation",
            default_value="false",
            description="When SaveMap is called, render final train/test camera records to renders/, gt/, and render_depth/.",
        ),
        DeclareLaunchArgument(
            "test_frame_stride",
            default_value="1",
            description="Store every Nth non-keyframe test camera for final render evaluation; all keyframes and point clouds are still processed.",
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
        *qos_launch_arguments,
        DeclareLaunchArgument(
            "require_depth_topic",
            default_value="true",
            description="Require depth_topic in frame synchronization; false uses sparse point-projected depth",
        ),
        DeclareLaunchArgument(
            "sync_anchor_stream",
            default_value="pointcloud",
            description="Frame synchronization anchor stream: pointcloud or image",
        ),
        DeclareLaunchArgument(
            "render_mode",
            default_value="rasterizer",
            description="Rendered output mode: debug_cpu, debug_input, rasterizer, or off",
        ),
        DeclareLaunchArgument(
            "rendered_feedback_topic",
            default_value="/gaussian_lic/rendered_feedback",
            description="Typed rendered-feedback topic carrying image plus mapper/source stamps",
        ),
        DeclareLaunchArgument(
            "rendered_feedback_source_stream",
            default_value="aligned_frame",
            description="Typed rendered-feedback source stream: aligned_frame or image_pose",
        ),
        DeclareLaunchArgument(
            "publish_rendered_feedback_before_update",
            default_value="false",
            description="Diagnostic: publish typed rendered feedback before Torch/Gaussian update",
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
            description="Load mapping_node as an rclcpp component in a single-threaded component_container",
        ),
        DeclareLaunchArgument("rviz", default_value="false", description="Start RViz2 with Gaussian-LIC displays"),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz_config, description="RViz2 config file"),

        ExecuteProcess(
            cmd=["ros2", "bag", "play", bag, "--clock", "--read-ahead-queue-size", "100"],
            output="screen",
            condition=play_bag_once_condition,
        ),

        ExecuteProcess(
            cmd=["ros2", "bag", "play", bag, "--clock", "--read-ahead-queue-size", "100", "--loop"],
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

        Node(
            package="gaussian_lic_frontend",
            executable="livox_custom_to_pointcloud2",
            name="livox_custom_to_pointcloud2",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "input_topic": livox_custom_topic,
                    "output_topic": livox_pointcloud_topic,
                    "sensor_qos_reliability": sensor_qos_reliability,
                    "sensor_qos_depth": sensor_qos_depth,
                },
            ],
            condition=IfCondition(livox_custom_bridge),
        ),

        ComposableNodeContainer(
            package="rclcpp_components",
            executable="component_container",
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
