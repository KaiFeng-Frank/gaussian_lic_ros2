// SPDX-License-Identifier: GPL-3.0-or-later

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <gaussian_lic_tracking/gaussian_snapshot.hpp>
#include <gaussian_lic_tracking/imu_propagator.hpp>
#include <gaussian_lic_tracking/lidar_deskew.hpp>
#include <gaussian_lic_tracking/lidar_factor.hpp>
#include <gaussian_lic_tracking/sliding_window_optimizer.hpp>
#include <gaussian_lic_tracking/time.hpp>
#include <gaussian_lic_tracking/visual_factor.hpp>
#include <gaussian_lic_msgs/msg/gaussian_array.hpp>
#include <gaussian_lic_msgs/msg/tracking_status.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>

class TrackingNode final : public rclcpp::Node
{
public:
  explicit TrackingNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("tracking_node", options)
  {
    raw_image_topic_ = declare_parameter<std::string>("raw_image_topic", "/camera/image");
    raw_camera_info_topic_ = declare_parameter<std::string>("raw_camera_info_topic", "/camera/camera_info");
    raw_depth_topic_ = declare_parameter<std::string>("raw_depth_topic", "/camera/depth");
    raw_pointcloud_topic_ = declare_parameter<std::string>("raw_pointcloud_topic", "/livox/lidar");
    raw_imu_topic_ = declare_parameter<std::string>("raw_imu_topic", "/imu");
    image_topic_ = declare_parameter<std::string>("image_topic", "/image_for_gs");
    camera_info_topic_ = declare_parameter<std::string>("camera_info_topic", "/camera_info_for_gs");
    depth_topic_ = declare_parameter<std::string>("depth_topic", "/depth_for_gs");
    pointcloud_topic_ = declare_parameter<std::string>("pointcloud_topic", "/points_for_gs");
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/pose_for_gs");
    odometry_topic_ = declare_parameter<std::string>("odometry_topic", "/gaussian_lic/frontend/odometry");
    path_topic_ = declare_parameter<std::string>("path_topic", "/gaussian_lic/frontend/path");
    tracking_status_topic_ = declare_parameter<std::string>(
      "tracking_status_topic", "/gaussian_lic/frontend/status");
    rendered_image_topic_ = declare_parameter<std::string>("rendered_image_topic", "/gaussian_lic/rendered_image");
    gaussian_map_topic_ = declare_parameter<std::string>("gaussian_map_topic", "/gaussian_lic/gaussian_map");
    world_frame_ = declare_parameter<std::string>("world_frame", "map");
    child_frame_ = declare_parameter<std::string>("child_frame", "base_link");
    publish_tf_ = declare_parameter<bool>("publish_tf", false);
    max_path_length_ = integer_parameter_at_least(
      "max_path_length", declare_parameter<int>("max_path_length", 5000), 1);
    sensor_qos_depth_ = integer_parameter_at_least(
      "sensor_qos_depth", declare_parameter<int>("sensor_qos_depth", 5), 1);
    sensor_qos_reliability_ = declare_parameter<std::string>("sensor_qos_reliability", "best_effort");
    sensor_qos_history_ = declare_parameter<std::string>("sensor_qos_history", "keep_last");
    raw_image_qos_ = declare_topic_qos("raw_image");
    raw_camera_info_qos_ = declare_topic_qos("raw_camera_info");
    raw_depth_qos_ = declare_topic_qos("raw_depth");
    raw_pointcloud_qos_ = declare_topic_qos("raw_pointcloud");
    raw_imu_qos_ = declare_topic_qos("raw_imu");
    image_qos_ = declare_topic_qos("image");
    camera_info_qos_ = declare_topic_qos("camera_info");
    depth_qos_ = declare_topic_qos("depth");
    pointcloud_qos_ = declare_topic_qos("pointcloud");
    pose_qos_ = declare_topic_qos("pose");
    frontend_odometry_qos_ = declare_topic_qos("frontend_odometry");
    serialize_callbacks_ = declare_parameter<bool>("serialize_callbacks", true);
    enable_visual_factor_ = declare_parameter<bool>("enable_visual_factor", true);
    enable_gaussian_snapshot_ = declare_parameter<bool>("enable_gaussian_snapshot", true);
    visual_max_pixels_ = integer_parameter_at_least(
      "visual_max_pixels", declare_parameter<int>("visual_max_pixels", 200000), 1);
    visual_factor_max_dt_ns_ = integer_parameter_at_least(
      "visual_factor_max_dt_ns",
      declare_parameter<int64_t>("visual_factor_max_dt_ns", 150000000LL), 0LL);
    depth_frame_cache_size_ = integer_parameter_at_least(
      "depth_frame_cache_size", declare_parameter<int>("depth_frame_cache_size", 8), 1);
    rendered_frame_cache_size_ = integer_parameter_at_least(
      "rendered_frame_cache_size", declare_parameter<int>("rendered_frame_cache_size", 8), 1);
    const auto camera_to_imu_translation = declare_parameter<std::vector<double>>(
      "camera_to_imu_translation_m", std::vector<double>{0.0, 0.0, 0.0});
    const auto camera_to_imu_rpy = declare_parameter<std::vector<double>>(
      "camera_to_imu_rpy_rad", std::vector<double>{0.0, 0.0, 0.0});
    p_i_c_ = vector3_from_parameter("camera_to_imu_translation_m", camera_to_imu_translation);
    q_i_c_ = quaternion_from_rpy_parameter("camera_to_imu_rpy_rad", camera_to_imu_rpy);
    visual_alignment_max_shift_px_ = integer_parameter_at_least(
      "visual_alignment_max_shift_px",
      declare_parameter<int>("visual_alignment_max_shift_px", 8), 0);
    enable_visual_alignment_window_factor_ =
      declare_parameter<bool>("enable_visual_alignment_window_factor", true);
    visual_alignment_meters_per_pixel_ = finite_positive_parameter(
      "visual_alignment_meters_per_pixel",
      declare_parameter<double>("visual_alignment_meters_per_pixel", 0.01));
    visual_alignment_window_weight_ = finite_positive_parameter(
      "visual_alignment_window_weight",
      declare_parameter<double>("visual_alignment_window_weight", 1.0));
    visual_alignment_huber_delta_m_ = finite_nonnegative_parameter(
      "visual_alignment_huber_delta_m",
      declare_parameter<double>("visual_alignment_huber_delta_m", 0.05));
    enable_se3_photometric_window_factor_ =
      declare_parameter<bool>("enable_se3_photometric_window_factor", true);
    se3_photometric_window_weight_ = finite_positive_parameter(
      "se3_photometric_window_weight",
      declare_parameter<double>("se3_photometric_window_weight", 1.0));
    se3_photometric_factor_huber_delta_ = finite_nonnegative_parameter(
      "se3_photometric_factor_huber_delta",
      declare_parameter<double>("se3_photometric_factor_huber_delta", 1.0));
    se3_photometric_max_samples_ = integer_parameter_at_least(
      "se3_photometric_max_samples",
      declare_parameter<int>("se3_photometric_max_samples", 2000), 1);
    se3_photometric_min_samples_ = integer_parameter_at_least(
      "se3_photometric_min_samples",
      declare_parameter<int>("se3_photometric_min_samples", 16), 1);
    if (se3_photometric_max_samples_ < se3_photometric_min_samples_) {
      throw std::runtime_error("se3_photometric_max_samples must be >= se3_photometric_min_samples");
    }
    se3_photometric_min_depth_m_ = finite_positive_parameter(
      "se3_photometric_min_depth_m",
      declare_parameter<double>("se3_photometric_min_depth_m", 0.05));
    se3_photometric_max_depth_m_ = finite_positive_parameter(
      "se3_photometric_max_depth_m",
      declare_parameter<double>("se3_photometric_max_depth_m", 200.0));
    if (se3_photometric_max_depth_m_ <= se3_photometric_min_depth_m_) {
      throw std::runtime_error("se3_photometric_max_depth_m must be greater than se3_photometric_min_depth_m");
    }
    se3_photometric_min_gradient_ = finite_nonnegative_parameter(
      "se3_photometric_min_gradient",
      declare_parameter<double>("se3_photometric_min_gradient", 1.0e-4));
    se3_photometric_huber_delta_ = finite_nonnegative_parameter(
      "se3_photometric_huber_delta",
      declare_parameter<double>("se3_photometric_huber_delta", 0.15));
    se3_photometric_max_abs_residual_ = finite_nonnegative_parameter(
      "se3_photometric_max_abs_residual",
      declare_parameter<double>("se3_photometric_max_abs_residual", 1.0));
    enable_lio_factor_ = declare_parameter<bool>("enable_lio_factor", true);
    enable_lidar_plane_factor_ = declare_parameter<bool>("enable_lidar_plane_factor", true);
    lidar_min_points_ = integer_parameter_at_least(
      "lidar_min_points", declare_parameter<int>("lidar_min_points", 32), 1);
    lidar_max_frame_points_ = integer_parameter_at_least(
      "lidar_max_frame_points", declare_parameter<int>("lidar_max_frame_points", 2000), 1);
    lidar_max_map_points_ = integer_parameter_at_least(
      "lidar_max_map_points", declare_parameter<int>("lidar_max_map_points", 20000), 1);
    lidar_nearest_distance_m_ = finite_positive_parameter(
      "lidar_nearest_distance_m",
      declare_parameter<double>("lidar_nearest_distance_m", 0.35));
    lidar_correction_gain_ = finite_nonnegative_parameter(
      "lidar_correction_gain",
      declare_parameter<double>("lidar_correction_gain", 0.7));
    lidar_max_correction_m_ = finite_nonnegative_parameter(
      "lidar_max_correction_m",
      declare_parameter<double>("lidar_max_correction_m", 0.25));
    lidar_max_rotation_rad_ = finite_nonnegative_parameter(
      "lidar_max_rotation_rad",
      declare_parameter<double>("lidar_max_rotation_rad", 0.08));
    lidar_robust_kernel_m_ = finite_nonnegative_parameter(
      "lidar_robust_kernel_m",
      declare_parameter<double>("lidar_robust_kernel_m", 0.15));
    lidar_plane_min_neighbors_ = integer_parameter_at_least(
      "lidar_plane_min_neighbors",
      declare_parameter<int>("lidar_plane_min_neighbors", 5), 3);
    lidar_plane_max_condition_ = finite_positive_parameter(
      "lidar_plane_max_condition",
      declare_parameter<double>("lidar_plane_max_condition", 0.2));
    lidar_keyframe_translation_m_ = finite_nonnegative_parameter(
      "lidar_keyframe_translation_m",
      declare_parameter<double>("lidar_keyframe_translation_m", 0.25));
    const auto lidar_to_imu_translation = declare_parameter<std::vector<double>>(
      "lidar_to_imu_translation_m", std::vector<double>{0.0, 0.0, 0.0});
    const auto lidar_to_imu_rpy = declare_parameter<std::vector<double>>(
      "lidar_to_imu_rpy_rad", std::vector<double>{0.0, 0.0, 0.0});
    p_i_l_ = vector3_from_parameter("lidar_to_imu_translation_m", lidar_to_imu_translation);
    q_i_l_ = quaternion_from_rpy_parameter("lidar_to_imu_rpy_rad", lidar_to_imu_rpy);
    enable_lidar_deskew_ = declare_parameter<bool>("enable_lidar_deskew", true);
    lidar_time_field_ = declare_parameter<std::string>("lidar_time_field", "auto");
    lidar_time_unit_ = declare_parameter<std::string>("lidar_time_unit", "auto");
    lidar_time_mode_ = declare_parameter<std::string>("lidar_time_mode", "auto");
    imu_history_size_ = integer_parameter_at_least(
      "imu_history_size", declare_parameter<int>("imu_history_size", 4000), 2);
    trajectory_control_interval_ns_ = integer_parameter_at_least(
      "trajectory_control_interval_ns",
      declare_parameter<int64_t>("trajectory_control_interval_ns", 50000000LL), 1LL);
    enable_sliding_window_optimizer_ = declare_parameter<bool>("enable_sliding_window_optimizer", true);
    enable_gaussian_snapshot_lidar_factor_ =
      declare_parameter<bool>("enable_gaussian_snapshot_lidar_factor", true);
    gaussian_snapshot_lidar_min_opacity_ = finite_nonnegative_parameter(
      "gaussian_snapshot_lidar_min_opacity",
      declare_parameter<double>("gaussian_snapshot_lidar_min_opacity", 0.01));
    sliding_window_max_states_ = integer_parameter_at_least(
      "sliding_window_max_states",
      declare_parameter<int>("sliding_window_max_states", 12), 2);
    sliding_window_max_iterations_ = integer_parameter_at_least(
      "sliding_window_max_iterations",
      declare_parameter<int>("sliding_window_max_iterations", 3), 1);
    sliding_window_max_rotation_step_rad_ = finite_nonnegative_parameter(
      "sliding_window_max_rotation_step_rad",
      declare_parameter<double>("sliding_window_max_rotation_step_rad", 0.5));
    sliding_window_max_translation_step_m_ = finite_nonnegative_parameter(
      "sliding_window_max_translation_step_m",
      declare_parameter<double>("sliding_window_max_translation_step_m", 1.0));
    sliding_window_max_velocity_step_mps_ = finite_nonnegative_parameter(
      "sliding_window_max_velocity_step_mps",
      declare_parameter<double>("sliding_window_max_velocity_step_mps", 5.0));
    sliding_window_max_bias_step_ = finite_nonnegative_parameter(
      "sliding_window_max_bias_step",
      declare_parameter<double>("sliding_window_max_bias_step", 1.0));
    sliding_window_max_normal_equation_condition_ = finite_positive_parameter(
      "sliding_window_max_normal_equation_condition",
      declare_parameter<double>("sliding_window_max_normal_equation_condition", 1.0e12));
    sliding_window_min_normal_equation_rank_ratio_ = finite_unit_interval_parameter(
      "sliding_window_min_normal_equation_rank_ratio",
      declare_parameter<double>("sliding_window_min_normal_equation_rank_ratio", 0.0));
    sliding_window_imu_weight_ = finite_nonnegative_parameter(
      "sliding_window_imu_weight",
      declare_parameter<double>("sliding_window_imu_weight", 1.0));
    sliding_window_imu_rotation_weight_ = finite_nonnegative_parameter(
      "sliding_window_imu_rotation_weight",
      declare_parameter<double>("sliding_window_imu_rotation_weight", 1.0));
    sliding_window_imu_velocity_weight_ = finite_nonnegative_parameter(
      "sliding_window_imu_velocity_weight",
      declare_parameter<double>("sliding_window_imu_velocity_weight", 1.0));
    sliding_window_imu_position_weight_ = finite_nonnegative_parameter(
      "sliding_window_imu_position_weight",
      declare_parameter<double>("sliding_window_imu_position_weight", 1.0));
    sliding_window_bias_weight_ = finite_nonnegative_parameter(
      "sliding_window_bias_weight",
      declare_parameter<double>("sliding_window_bias_weight", 1.0));
    sliding_window_pose_translation_weight_ = finite_nonnegative_parameter(
      "sliding_window_pose_translation_weight",
      declare_parameter<double>("sliding_window_pose_translation_weight", 2.0));
    sliding_window_pose_rotation_weight_ = finite_nonnegative_parameter(
      "sliding_window_pose_rotation_weight",
      declare_parameter<double>("sliding_window_pose_rotation_weight", 2.0));
    enable_sliding_window_smoothness_factor_ =
      declare_parameter<bool>("enable_sliding_window_smoothness_factor", true);
    sliding_window_smoothness_rotation_weight_ = finite_nonnegative_parameter(
      "sliding_window_smoothness_rotation_weight",
      declare_parameter<double>("sliding_window_smoothness_rotation_weight", 0.1));
    sliding_window_smoothness_position_weight_ = finite_nonnegative_parameter(
      "sliding_window_smoothness_position_weight",
      declare_parameter<double>("sliding_window_smoothness_position_weight", 0.1));
    sliding_window_smoothness_velocity_weight_ = finite_nonnegative_parameter(
      "sliding_window_smoothness_velocity_weight",
      declare_parameter<double>("sliding_window_smoothness_velocity_weight", 0.1));
    sliding_window_smoothness_bias_weight_ = finite_nonnegative_parameter(
      "sliding_window_smoothness_bias_weight",
      declare_parameter<double>("sliding_window_smoothness_bias_weight", 0.1));
    const auto gravity =
      declare_parameter<std::vector<double>>("imu_gravity_w", std::vector<double>{0.0, 0.0, 0.0});
    imu_propagator_.set_gravity_w(vector3_from_parameter("imu_gravity_w", gravity));
    imu_propagator_.set_max_history_size(static_cast<size_t>(imu_history_size_));
    trajectory_manager_.set_control_interval_ns(trajectory_control_interval_ns_);
    gaussian_lic_tracking::SlidingWindowConfig window_config;
    window_config.max_states = static_cast<size_t>(sliding_window_max_states_);
    window_config.max_iterations = static_cast<size_t>(sliding_window_max_iterations_);
    window_config.max_rotation_step_rad = sliding_window_max_rotation_step_rad_;
    window_config.max_translation_step_m = sliding_window_max_translation_step_m_;
    window_config.max_velocity_step_mps = sliding_window_max_velocity_step_mps_;
    window_config.max_bias_step = sliding_window_max_bias_step_;
    window_config.max_normal_equation_condition = sliding_window_max_normal_equation_condition_;
    window_config.min_normal_equation_rank_ratio = sliding_window_min_normal_equation_rank_ratio_;
    sliding_window_optimizer_.set_config(window_config);

    gaussian_lic_tracking::LidarFactorConfig lidar_config;
    lidar_config.min_points = static_cast<size_t>(lidar_min_points_);
    lidar_config.max_frame_points = static_cast<size_t>(lidar_max_frame_points_);
    lidar_config.max_map_points = static_cast<size_t>(lidar_max_map_points_);
    lidar_config.nearest_distance_m = lidar_nearest_distance_m_;
    lidar_config.correction_gain = lidar_correction_gain_;
    lidar_config.max_correction_m = lidar_max_correction_m_;
    lidar_config.max_rotation_rad = lidar_max_rotation_rad_;
    lidar_config.robust_kernel_m = lidar_robust_kernel_m_;
    lidar_config.plane_min_neighbors = static_cast<size_t>(lidar_plane_min_neighbors_);
    lidar_config.plane_max_condition = lidar_plane_max_condition_;
    lidar_factor_.set_config(lidar_config);
    visual_factor_.set_max_pixels(static_cast<size_t>(visual_max_pixels_));

    const auto raw_image_qos = make_sensor_qos("raw_image", raw_image_qos_);
    const auto raw_camera_info_qos = make_sensor_qos("raw_camera_info", raw_camera_info_qos_);
    const auto raw_depth_qos = make_sensor_qos("raw_depth", raw_depth_qos_);
    const auto raw_pointcloud_qos = make_sensor_qos("raw_pointcloud", raw_pointcloud_qos_);
    const auto raw_imu_qos = make_sensor_qos("raw_imu", raw_imu_qos_);
    const auto image_qos = make_sensor_qos("image", image_qos_);
    const auto camera_info_qos = make_sensor_qos("camera_info", camera_info_qos_);
    const auto depth_qos = make_sensor_qos("depth", depth_qos_);
    const auto pointcloud_qos = make_sensor_qos("pointcloud", pointcloud_qos_);
    const auto pose_qos = make_sensor_qos("pose", pose_qos_);
    const auto frontend_odometry_qos = make_sensor_qos("frontend_odometry", frontend_odometry_qos_);
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      raw_image_topic_, raw_image_qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
        run_serialized_callback([this, msg]() {
          handle_image(*msg);
        });
      });
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      raw_camera_info_topic_, raw_camera_info_qos,
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
        run_serialized_callback([this, msg]() {
          handle_camera_info(*msg);
        });
      });
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      raw_depth_topic_, raw_depth_qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
        run_serialized_callback([this, msg]() {
          handle_depth(*msg);
        });
      });
    pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      raw_pointcloud_topic_, raw_pointcloud_qos,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        run_serialized_callback([this, msg]() {
          handle_pointcloud(*msg);
        });
      });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      raw_imu_topic_, raw_imu_qos,
      [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
        run_serialized_callback([this, msg]() {
          handle_imu(*msg);
        });
      });

    image_pub_ = create_publisher<sensor_msgs::msg::Image>(image_topic_, image_qos);
    camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, camera_info_qos);
    depth_pub_ = create_publisher<sensor_msgs::msg::Image>(depth_topic_, depth_qos);
    pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic_, pointcloud_qos);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_, pose_qos);
    odometry_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      odometry_topic_, frontend_odometry_qos);
    path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic_, rclcpp::QoS(1).transient_local().reliable());
    tracking_status_pub_ = create_publisher<gaussian_lic_msgs::msg::TrackingStatus>(
      tracking_status_topic_, rclcpp::QoS(1).transient_local().reliable());
    rendered_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      rendered_image_topic_, rclcpp::QoS(1).transient_local().reliable(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
        run_serialized_callback([this, msg]() {
          handle_rendered_image(*msg);
        });
      });
    if (enable_gaussian_snapshot_) {
      gaussian_map_sub_ = create_subscription<gaussian_lic_msgs::msg::GaussianArray>(
        gaussian_map_topic_, rclcpp::QoS(1).transient_local().reliable(),
        [this](gaussian_lic_msgs::msg::GaussianArray::ConstSharedPtr msg) {
          run_serialized_callback([this, msg]() {
            handle_gaussian_snapshot(*msg);
          });
        });
    }
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }
  }

private:
  struct DecodedLidarPoint
  {
    Eigen::Vector3d point_i{Eigen::Vector3d::Zero()};
    int64_t stamp_ns{0};
    bool has_stamp{false};
    size_t index{0};
  };

  struct PointCloudFields
  {
    const sensor_msgs::msg::PointField * x_field{nullptr};
    const sensor_msgs::msg::PointField * y_field{nullptr};
    const sensor_msgs::msg::PointField * z_field{nullptr};
    const sensor_msgs::msg::PointField * time_field{nullptr};
    int x_offset{-1};
    int y_offset{-1};
    int z_offset{-1};
    bool xyz_writable{false};
  };

  struct DepthFrame
  {
    int64_t stamp_ns{0};
    size_t width{0};
    size_t height{0};
    std::vector<float> depth_m;
  };

  struct Se3PhotometricSampleBatch
  {
    std::vector<gaussian_lic_tracking::VisualSe3PhotometricSample> samples;
    size_t candidate_pixels{0};
    size_t accepted_pixels{0};
    size_t rejected_depth_pixels{0};
    size_t rejected_gradient_pixels{0};
    size_t rejected_residual_pixels{0};
    double mean_abs_residual{0.0};
  };

  struct QosProfileParams
  {
    std::string reliability{"best_effort"};
    std::string history{"keep_last"};
    int depth{5};
  };

  template<typename CallbackT>
  void run_serialized_callback(CallbackT && callback)
  {
    if (serialize_callbacks_) {
      std::scoped_lock<std::mutex> lock(callback_mutex_);
      std::forward<CallbackT>(callback)();
      return;
    }
    std::forward<CallbackT>(callback)();
  }

  QosProfileParams declare_topic_qos(const std::string & prefix)
  {
    QosProfileParams params;
    params.reliability = declare_parameter<std::string>(
      prefix + "_qos_reliability", sensor_qos_reliability_);
    params.history = declare_parameter<std::string>(
      prefix + "_qos_history", sensor_qos_history_);
    params.depth = integer_parameter_at_least(
      (prefix + "_qos_depth").c_str(),
      declare_parameter<int>(prefix + "_qos_depth", sensor_qos_depth_), 1);
    return params;
  }

  rclcpp::QoS make_sensor_qos(const char * stream_name, const QosProfileParams & params) const
  {
    rclcpp::QoS qos(static_cast<size_t>(params.depth));
    if (params.history == "keep_all") {
      qos.keep_all();
    } else if (params.history == "keep_last") {
      qos.keep_last(static_cast<size_t>(params.depth));
    } else {
      throw std::runtime_error(
        std::string(stream_name) + "_qos_history must be keep_last or keep_all, got " +
        params.history);
    }
    qos.durability_volatile();
    if (params.reliability == "reliable") {
      qos.reliable();
    } else if (params.reliability == "best_effort") {
      qos.best_effort();
    } else {
      throw std::runtime_error(
        std::string(stream_name) + "_qos_reliability must be best_effort or reliable, got " +
        params.reliability);
    }
    return qos;
  }

  void handle_imu(const sensor_msgs::msg::Imu & msg)
  {
    try {
      const int64_t stamp_ns = gaussian_lic_tracking::stamp_to_nanoseconds(msg.header.stamp);
      if (!accept_stream_stamp("imu", stamp_ns, last_imu_input_stamp_ns_, imu_stamp_regressions_, false)) {
        return;
      }
      const Eigen::Vector3d angular_velocity{
        msg.angular_velocity.x,
        msg.angular_velocity.y,
        msg.angular_velocity.z};
      const Eigen::Vector3d linear_acceleration{
        msg.linear_acceleration.x,
        msg.linear_acceleration.y,
        msg.linear_acceleration.z};
      if (!angular_velocity.allFinite() || !linear_acceleration.allFinite()) {
        ++imu_invalid_measurements_;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "dropping IMU message with non-finite angular velocity or linear acceleration");
        return;
      }
      last_imu_stamp_ns_ = stamp_ns;
      ++num_raw_imus_;
      imu_propagator_.add_measurement(stamp_ns, angular_velocity, linear_acceleration);
      if (enable_sliding_window_optimizer_) {
        sliding_window_preintegrator_.add_measurement(
          stamp_ns, angular_velocity, linear_acceleration);
        sliding_window_preintegrator_initialized_ = true;
      }
    } catch (const std::exception & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "failed to propagate IMU tracking state: %s", ex.what());
    }
  }

  void handle_camera_info(const sensor_msgs::msg::CameraInfo & msg)
  {
    camera_info_pub_->publish(msg);
    if (valid_camera_info_intrinsics(msg)) {
      camera_intrinsics_.fx = msg.k[0];
      camera_intrinsics_.fy = msg.k[4];
      camera_intrinsics_.cx = msg.k[2];
      camera_intrinsics_.cy = msg.k[5];
      has_camera_intrinsics_ = true;
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "ignoring CameraInfo with invalid intrinsics");
    }
  }

  static bool valid_camera_info_intrinsics(const sensor_msgs::msg::CameraInfo & msg)
  {
    return std::isfinite(msg.k[0]) && std::isfinite(msg.k[4]) &&
           std::isfinite(msg.k[2]) && std::isfinite(msg.k[5]) &&
           msg.k[0] > 0.0 && msg.k[4] > 0.0;
  }

  static Eigen::Vector3d vector3_from_parameter(
    const char * parameter_name,
    const std::vector<double> & values)
  {
    if (values.size() != 3U) {
      throw std::runtime_error(std::string(parameter_name) + " must contain exactly 3 values");
    }
    if (!std::isfinite(values[0]) || !std::isfinite(values[1]) || !std::isfinite(values[2])) {
      throw std::runtime_error(std::string(parameter_name) + " must contain only finite values");
    }
    return Eigen::Vector3d{values[0], values[1], values[2]};
  }

  static Eigen::Quaterniond quaternion_from_rpy_parameter(
    const char * parameter_name,
    const std::vector<double> & values)
  {
    if (values.size() != 3U) {
      throw std::runtime_error(std::string(parameter_name) + " must contain exactly 3 values");
    }
    if (!std::isfinite(values[0]) || !std::isfinite(values[1]) || !std::isfinite(values[2])) {
      throw std::runtime_error(std::string(parameter_name) + " must contain only finite values");
    }
    const Eigen::Quaterniond q = (
      Eigen::AngleAxisd(values[2], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(values[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(values[0], Eigen::Vector3d::UnitX())).normalized();
    if (!q.coeffs().allFinite() || q.norm() <= std::numeric_limits<double>::epsilon()) {
      throw std::runtime_error(std::string(parameter_name) + " produced an invalid quaternion");
    }
    return q;
  }

  static double finite_nonnegative_parameter(const char * parameter_name, const double value)
  {
    if (!std::isfinite(value) || value < 0.0) {
      throw std::runtime_error(std::string(parameter_name) + " must be finite and non-negative");
    }
    return value;
  }

  static double finite_positive_parameter(const char * parameter_name, const double value)
  {
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::runtime_error(std::string(parameter_name) + " must be finite and positive");
    }
    return value;
  }

  static double finite_unit_interval_parameter(const char * parameter_name, const double value)
  {
    if (!std::isfinite(value) || value < 0.0 || value > 1.0) {
      throw std::runtime_error(std::string(parameter_name) + " must be finite and in [0, 1]");
    }
    return value;
  }

  template<typename ValueT, typename MinimumT>
  static ValueT integer_parameter_at_least(
    const char * parameter_name,
    const ValueT value,
    const MinimumT minimum_value)
  {
    const ValueT typed_minimum = static_cast<ValueT>(minimum_value);
    if (value < typed_minimum) {
      throw std::runtime_error(std::string(parameter_name) + " is below its minimum value");
    }
    return value;
  }

  bool accept_stream_stamp(
    const char * stream_name,
    const int64_t stamp_ns,
    std::optional<int64_t> & last_seen_stamp_ns,
    uint64_t & regression_count,
    const bool allow_equal_stamp)
  {
    if (last_seen_stamp_ns.has_value()) {
      const bool out_of_order = allow_equal_stamp
        ? stamp_ns < last_seen_stamp_ns.value()
        : stamp_ns <= last_seen_stamp_ns.value();
      if (out_of_order) {
        ++regression_count;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "dropping %s message with non-monotonic stamp: current=%ld previous=%ld",
          stream_name, static_cast<long>(stamp_ns), static_cast<long>(last_seen_stamp_ns.value()));
        return false;
      }
    }
    last_seen_stamp_ns = stamp_ns;
    return true;
  }

  void handle_depth(const sensor_msgs::msg::Image & msg)
  {
    const int64_t stamp_ns = gaussian_lic_tracking::stamp_to_nanoseconds(msg.header.stamp);
    if (!accept_stream_stamp("depth", stamp_ns, last_depth_input_stamp_ns_, depth_stamp_regressions_, true)) {
      return;
    }
    depth_pub_->publish(msg);
    DepthFrame decoded;
    if (decode_depth_image(msg, decoded)) {
      cache_depth_frame(std::move(decoded));
    }
  }

  void handle_image(const sensor_msgs::msg::Image & msg)
  {
    const int64_t stamp_ns = gaussian_lic_tracking::stamp_to_nanoseconds(msg.header.stamp);
    if (!accept_stream_stamp("image", stamp_ns, last_image_input_stamp_ns_, image_stamp_regressions_, true)) {
      return;
    }
    last_image_stamp_ns_ = stamp_ns;
    ++num_raw_images_;
    image_pub_->publish(msg);
    if (!enable_visual_factor_) {
      return;
    }
    gaussian_lic_tracking::VisualFrame observed;
    if (!decode_image_gray(msg, observed)) {
      return;
    }
    int64_t rendered_match_delta_ns = 0;
    bool rendered_cache_had_size_match = false;
    const gaussian_lic_tracking::VisualFrame * rendered_frame =
      select_rendered_frame_for_stamp(
      observed.stamp_ns,
      observed.width,
      observed.height,
      &rendered_match_delta_ns,
      &rendered_cache_had_size_match);
    last_visual_rendered_cache_size_ = rendered_frame_cache_.size();
    last_visual_rendered_match_delta_ns_ = rendered_frame == nullptr ? 0 : rendered_match_delta_ns;
    if (rendered_frame == nullptr) {
      if (rendered_frame_cache_.empty()) {
        ++visual_rendered_miss_count_;
      } else if (!rendered_cache_had_size_match) {
        ++visual_rendered_size_mismatch_count_;
      } else {
        ++visual_rendered_stale_count_;
      }
    }
    if (rendered_frame != nullptr) {
      last_visual_residual_ = visual_factor_.evaluate(*rendered_frame, observed);
      last_visual_alignment_ = visual_factor_.estimate_translation(
        *rendered_frame,
        observed,
        visual_alignment_max_shift_px_);
      last_visual_photometric_linearization_ =
        visual_factor_.linearize_translation(*rendered_frame, observed);
      if (enable_se3_photometric_window_factor_) {
        const auto se3_samples = build_se3_photometric_samples(*rendered_frame, observed);
        last_visual_se3_photometric_candidate_pixels_ = se3_samples.candidate_pixels;
        last_visual_se3_photometric_accepted_pixels_ = se3_samples.accepted_pixels;
        last_visual_se3_photometric_rejected_depth_pixels_ = se3_samples.rejected_depth_pixels;
        last_visual_se3_photometric_rejected_gradient_pixels_ = se3_samples.rejected_gradient_pixels;
        last_visual_se3_photometric_rejected_residual_pixels_ = se3_samples.rejected_residual_pixels;
        last_visual_se3_photometric_mean_abs_residual_ = se3_samples.mean_abs_residual;
        last_visual_se3_photometric_linearization_ =
          gaussian_lic_tracking::linearize_se3_photometric_samples(camera_intrinsics_, se3_samples.samples);
        if (last_visual_se3_photometric_linearization_.valid &&
          last_visual_se3_photometric_linearization_.sample_count >=
          static_cast<size_t>(se3_photometric_min_samples_))
        {
          pending_visual_se3_photometric_linearization_ = last_visual_se3_photometric_linearization_;
          pending_visual_se3_photometric_stamp_ns_ = observed.stamp_ns;
          has_pending_visual_se3_photometric_ = true;
        }
      }
      if (last_visual_alignment_.valid) {
        pending_visual_alignment_ = last_visual_alignment_;
        pending_visual_alignment_stamp_ns_ = observed.stamp_ns;
        has_pending_visual_alignment_ = true;
      }
      if (last_visual_residual_.valid) {
        RCLCPP_DEBUG_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "visual residual pixels=%zu mae=%.6f rmse=%.6f align_valid=%s dx=%d dy=%d align_rmse=%.6f photo_valid=%s photo_step=(%.4f, %.4f) se3_photo_valid=%s se3_samples=%zu/%zu se3_mae=%.6f",
          last_visual_residual_.compared_pixels,
          last_visual_residual_.mean_abs_error,
          last_visual_residual_.rmse,
          last_visual_alignment_.valid ? "true" : "false",
          last_visual_alignment_.dx,
          last_visual_alignment_.dy,
          last_visual_alignment_.rmse,
          last_visual_photometric_linearization_.valid ? "true" : "false",
          last_visual_photometric_linearization_.gauss_newton_step.x(),
          last_visual_photometric_linearization_.gauss_newton_step.y(),
          last_visual_se3_photometric_linearization_.valid ? "true" : "false",
          last_visual_se3_photometric_linearization_.sample_count,
          last_visual_se3_photometric_candidate_pixels_,
          last_visual_se3_photometric_mean_abs_residual_);
      }
    }
  }

  void handle_rendered_image(const sensor_msgs::msg::Image & msg)
  {
    if (!enable_visual_factor_) {
      return;
    }
    const int64_t stamp_ns = gaussian_lic_tracking::stamp_to_nanoseconds(msg.header.stamp);
    if (!accept_stream_stamp(
        "rendered_image", stamp_ns, last_rendered_input_stamp_ns_, rendered_stamp_regressions_, true))
    {
      return;
    }
    gaussian_lic_tracking::VisualFrame rendered;
    if (decode_image_gray(msg, rendered)) {
      cache_rendered_frame(std::move(rendered));
    }
  }

  void handle_gaussian_snapshot(const gaussian_lic_msgs::msg::GaussianArray & msg)
  {
    const bool accepted = gaussian_snapshot_.ingest(msg);
    last_gaussian_snapshot_stamp_ns_ = gaussian_snapshot_.stamp_ns();
    last_gaussian_total_count_ = gaussian_snapshot_.expected_total_count();
    last_gaussian_chunk_count_ = gaussian_snapshot_.expected_chunk_count();
    gaussian_snapshot_chunks_received_ = static_cast<uint32_t>(gaussian_snapshot_.received_chunk_count());
    last_gaussian_chunk_size_ = msg.gaussians.size();
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "received Gaussian snapshot chunk %u/%u total=%u chunk_size=%zu accepted=%s complete=%s cached=%zu mean_opacity=%.4f",
      msg.chunk_index + 1U,
      msg.chunk_count,
      msg.total_count,
      msg.gaussians.size(),
      accepted ? "true" : "false",
      gaussian_snapshot_.complete() ? "true" : "false",
      gaussian_snapshot_.point_count(),
      gaussian_snapshot_.mean_opacity());
  }

  void handle_pointcloud(const sensor_msgs::msg::PointCloud2 & msg)
  {
    gaussian_lic_tracking::TrajectoryPose tracking_pose;
    tracking_pose.stamp_ns = gaussian_lic_tracking::stamp_to_nanoseconds(msg.header.stamp);
    if (!accept_stream_stamp(
        "pointcloud", tracking_pose.stamp_ns, last_pointcloud_input_stamp_ns_,
        pointcloud_stamp_regressions_, false))
    {
      return;
    }
    ++num_raw_pointclouds_;
    last_pointcloud_stamp_ns_ = tracking_pose.stamp_ns;
    tracking_pose.q_w_i = Eigen::Quaterniond::Identity();
    if (imu_propagator_.initialized()) {
      const auto & state = imu_propagator_.state();
      tracking_pose.p_w_i = state.p_w_i;
      tracking_pose.q_w_i = state.q_w_i;
      tracking_pose.v_w_i = state.v_w_i;
    }

    sensor_msgs::msg::PointCloud2 output_cloud = msg;
    std::vector<Eigen::Vector3d> lidar_points;
    if (enable_lio_factor_ || enable_lidar_deskew_) {
      PointCloudFields fields;
      auto decoded_points = decode_pointcloud(msg, fields);
      lidar_points.reserve(decoded_points.size());
      for (auto & point : decoded_points) {
        point.point_i = q_i_l_ * point.point_i + p_i_l_;
        lidar_points.push_back(point.point_i);
      }
      last_lidar_points_ = lidar_points.size();
      if (enable_lidar_deskew_ && fields.xyz_writable && !decoded_points.empty()) {
        const auto deskew_result = deskew_decoded_points(decoded_points, tracking_pose);
        if (deskew_result.deskewed_count > 0U) {
          lidar_points = deskew_result.points_i;
          write_deskewed_points(output_cloud, fields, decoded_points, deskew_result.points_i);
          RCLCPP_DEBUG_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "deskewed %zu/%zu LiDAR points with max offset %.6fs",
            deskew_result.deskewed_count,
            decoded_points.size(),
            deskew_result.max_abs_time_offset_s);
        }
      }
    }
    pointcloud_pub_->publish(output_cloud);

    std::vector<gaussian_lic_tracking::SlidingWindowPointToPointFactor> window_point_factors;
    std::vector<gaussian_lic_tracking::SlidingWindowPointToPlaneFactor> window_plane_factors;
    size_t window_point_correspondences = 0U;
    size_t window_plane_correspondences = 0U;
    if (enable_lio_factor_) {
      const auto correction = lidar_factor_.compute_pose_correction(lidar_points, tracking_pose);
      if (correction.applied) {
        tracking_pose.p_w_i += correction.delta_p_w;
        tracking_pose.q_w_i = (correction.delta_q * tracking_pose.q_w_i).normalized();
      }
      if (enable_sliding_window_optimizer_) {
        auto lidar_window_factor = lidar_factor_.build_point_to_point_factor(lidar_points, tracking_pose);
        if (!lidar_window_factor.frame_points_i.empty()) {
          window_point_correspondences += lidar_window_factor.frame_points_i.size();
          window_point_factors.push_back(std::move(lidar_window_factor));
        }
        if (enable_lidar_plane_factor_) {
          auto lidar_plane_factor = lidar_factor_.build_point_to_plane_factor(lidar_points, tracking_pose);
          if (!lidar_plane_factor.frame_points_i.empty()) {
            window_plane_correspondences += lidar_plane_factor.frame_points_i.size();
            window_plane_factors.push_back(std::move(lidar_plane_factor));
          }
        }
        if (enable_gaussian_snapshot_lidar_factor_ && gaussian_snapshot_.complete()) {
          auto gaussian_window_factor = gaussian_snapshot_.build_point_to_point_factor(
            lidar_points,
            tracking_pose,
            static_cast<size_t>(lidar_min_points_),
            static_cast<size_t>(lidar_max_frame_points_),
            lidar_nearest_distance_m_,
            gaussian_snapshot_lidar_min_opacity_);
          if (!gaussian_window_factor.frame_points_i.empty()) {
            window_point_correspondences += gaussian_window_factor.frame_points_i.size();
            window_point_factors.push_back(std::move(gaussian_window_factor));
          }
        }
      }
      last_window_point_correspondences_ = window_point_correspondences;
      last_window_plane_correspondences_ = window_plane_correspondences;
      total_window_point_correspondences_ += window_point_correspondences;
      total_window_plane_correspondences_ += window_plane_correspondences;
      last_lidar_matches_ = std::max(
        correction.matched_points, window_point_correspondences + window_plane_correspondences);
      last_lidar_mean_residual_m_ = correction.mean_residual_m;
      if (should_insert_lidar_keyframe(tracking_pose, lidar_points.size())) {
        lidar_factor_.insert_keyframe(lidar_points, tracking_pose);
        ++num_lidar_keyframes_;
        last_lidar_keyframe_pose_ = tracking_pose;
        has_lidar_keyframe_ = true;
      }
    }

    std::vector<gaussian_lic_tracking::SlidingWindowVisualAlignmentFactor> visual_window_factors;
    std::vector<gaussian_lic_tracking::SlidingWindowSe3PhotometricFactor> se3_photometric_factors;
    if (enable_sliding_window_optimizer_ && enable_visual_alignment_window_factor_ &&
      has_pending_visual_alignment_)
    {
      if (pending_factor_stamp_is_fresh(pending_visual_alignment_stamp_ns_, tracking_pose.stamp_ns)) {
        gaussian_lic_tracking::SlidingWindowVisualAlignmentFactor visual_factor;
        visual_factor.stamp_ns = tracking_pose.stamp_ns;
        visual_factor.reference_p_w_i = tracking_pose.p_w_i;
        visual_factor.measured_shift_px = Eigen::Vector2d{
          pending_visual_alignment_.subpixel_dx,
          pending_visual_alignment_.subpixel_dy};
        visual_factor.meters_per_pixel = visual_alignment_meters_per_pixel_;
        visual_factor.weight = visual_alignment_window_weight_;
        visual_factor.huber_delta_m = visual_alignment_huber_delta_m_;
        visual_window_factors.push_back(visual_factor);
      } else {
        ++visual_alignment_pending_stale_drops_;
      }
      has_pending_visual_alignment_ = false;
      pending_visual_alignment_stamp_ns_.reset();
    }
    if (enable_sliding_window_optimizer_ && enable_se3_photometric_window_factor_ &&
      has_pending_visual_se3_photometric_)
    {
      if (pending_factor_stamp_is_fresh(
          pending_visual_se3_photometric_stamp_ns_, tracking_pose.stamp_ns))
      {
        gaussian_lic_tracking::SlidingWindowSe3PhotometricFactor factor;
        factor.stamp_ns = tracking_pose.stamp_ns;
        factor.reference_p_w_i = tracking_pose.p_w_i;
        factor.reference_q_w_i = tracking_pose.q_w_i;
        factor.target_delta = gaussian_lic_tracking::transform_camera_delta_to_body(
          q_i_c_,
          p_i_c_,
          pending_visual_se3_photometric_linearization_.gauss_newton_step);
        const Eigen::Matrix<double, 6, 6> body_hessian =
          gaussian_lic_tracking::transform_camera_information_to_body(
          q_i_c_,
          p_i_c_,
          pending_visual_se3_photometric_linearization_.hessian);
        factor.sqrt_information =
          gaussian_lic_tracking::sqrt_information_from_hessian(body_hessian);
        factor.weight = se3_photometric_window_weight_;
        factor.huber_delta = se3_photometric_factor_huber_delta_;
        if (factor.sqrt_information.norm() > std::numeric_limits<double>::epsilon()) {
          se3_photometric_factors.push_back(factor);
        } else {
          ++sliding_window_se3_photometric_factor_skip_count_;
        }
      } else {
        ++visual_se3_photometric_pending_stale_drops_;
      }
      has_pending_visual_se3_photometric_ = false;
      pending_visual_se3_photometric_stamp_ns_.reset();
    }

    if (enable_sliding_window_optimizer_) {
      tracking_pose = update_sliding_window(
        tracking_pose,
        window_point_factors,
        window_plane_factors,
        visual_window_factors,
        se3_photometric_factors);
    }
    append_trajectory_control_pose(tracking_pose);

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = msg.header.stamp;
    pose.header.frame_id = world_frame_;
    pose.pose.position.x = tracking_pose.p_w_i.x();
    pose.pose.position.y = tracking_pose.p_w_i.y();
    pose.pose.position.z = tracking_pose.p_w_i.z();
    pose.pose.orientation.x = tracking_pose.q_w_i.x();
    pose.pose.orientation.y = tracking_pose.q_w_i.y();
    pose.pose.orientation.z = tracking_pose.q_w_i.z();
    pose.pose.orientation.w = tracking_pose.q_w_i.w();
    pose_pub_->publish(pose);

    nav_msgs::msg::Odometry odom;
    odom.header = pose.header;
    odom.child_frame_id = child_frame_;
    odom.pose.pose = pose.pose;
    odom.twist.twist.linear.x = tracking_pose.v_w_i.x();
    odom.twist.twist.linear.y = tracking_pose.v_w_i.y();
    odom.twist.twist.linear.z = tracking_pose.v_w_i.z();
    odometry_pub_->publish(odom);
    ++num_published_poses_;

    path_.header = pose.header;
    path_.poses.push_back(pose);
    while (max_path_length_ > 0 && path_.poses.size() > static_cast<size_t>(max_path_length_)) {
      path_.poses.erase(path_.poses.begin());
    }
    path_pub_->publish(path_);

    if (tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header = pose.header;
      tf.child_frame_id = child_frame_;
      tf.transform.translation.x = pose.pose.position.x;
      tf.transform.translation.y = pose.pose.position.y;
      tf.transform.translation.z = pose.pose.position.z;
      tf.transform.rotation = pose.pose.orientation;
      tf_broadcaster_->sendTransform(tf);
    }
    publish_tracking_status(msg.header.stamp);
  }

  gaussian_lic_tracking::TrajectoryPose update_sliding_window(
    const gaussian_lic_tracking::TrajectoryPose & input_pose,
    const std::vector<gaussian_lic_tracking::SlidingWindowPointToPointFactor> & point_factors,
    const std::vector<gaussian_lic_tracking::SlidingWindowPointToPlaneFactor> & plane_factors,
    const std::vector<gaussian_lic_tracking::SlidingWindowVisualAlignmentFactor> & visual_factors,
    const std::vector<gaussian_lic_tracking::SlidingWindowSe3PhotometricFactor> & se3_photometric_factors)
  {
    gaussian_lic_tracking::TrajectoryPose output_pose = input_pose;
    gaussian_lic_tracking::ImuState imu_state;
    if (!imu_propagator_.query_state(input_pose.stamp_ns, imu_state)) {
      if (!imu_propagator_.initialized()) {
        return output_pose;
      }
      imu_state = imu_propagator_.state();
    }

    gaussian_lic_tracking::SlidingWindowState state;
    state.stamp_ns = input_pose.stamp_ns;
    state.p_w_i = input_pose.p_w_i;
    state.q_w_i = input_pose.q_w_i;
    state.v_w_i = imu_state.v_w_i;
    state.gyro_bias = sliding_window_bias_.gyro;
    state.accel_bias = sliding_window_bias_.accel;
    state.fixed = !has_sliding_window_state_;
    sliding_window_optimizer_.add_or_update_state(state);

    gaussian_lic_tracking::SlidingWindowPosePrior prior;
    prior.stamp_ns = input_pose.stamp_ns;
    prior.p_w_i = input_pose.p_w_i;
    prior.q_w_i = input_pose.q_w_i;
    prior.translation_weight = sliding_window_pose_translation_weight_;
    prior.rotation_weight = sliding_window_pose_rotation_weight_;
    sliding_window_optimizer_.add_pose_prior(prior);
    bool window_factor_added = false;
    for (const auto & point_factor : point_factors) {
      try {
        sliding_window_optimizer_.add_point_to_point_factor(point_factor);
        window_factor_added = true;
      } catch (const std::exception & ex) {
        ++sliding_window_point_factor_skip_count_;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "sliding window point factor skipped: %s", ex.what());
      }
    }
    for (const auto & plane_factor : plane_factors) {
      try {
        sliding_window_optimizer_.add_point_to_plane_factor(plane_factor);
        window_factor_added = true;
      } catch (const std::exception & ex) {
        ++sliding_window_plane_factor_skip_count_;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "sliding window plane factor skipped: %s", ex.what());
      }
    }
    for (const auto & visual_factor : visual_factors) {
      try {
        sliding_window_optimizer_.add_visual_alignment_factor(visual_factor);
        window_factor_added = true;
      } catch (const std::exception & ex) {
        ++sliding_window_visual_factor_skip_count_;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "sliding window visual factor skipped: %s", ex.what());
      }
    }
    for (const auto & se3_factor : se3_photometric_factors) {
      try {
        sliding_window_optimizer_.add_se3_photometric_factor(se3_factor);
        window_factor_added = true;
      } catch (const std::exception & ex) {
        ++sliding_window_se3_photometric_factor_skip_count_;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "sliding window SE3 photometric factor skipped: %s", ex.what());
      }
    }
    if (enable_sliding_window_smoothness_factor_ &&
      previous_sliding_window_stamp_ns_.has_value() && has_sliding_window_state_)
    {
      gaussian_lic_tracking::SlidingWindowTrajectorySmoothnessFactor factor;
      factor.previous_stamp_ns = previous_sliding_window_stamp_ns_.value();
      factor.current_stamp_ns = last_sliding_window_stamp_ns_;
      factor.next_stamp_ns = input_pose.stamp_ns;
      factor.rotation_rate_weight = sliding_window_smoothness_rotation_weight_;
      factor.position_rate_weight = sliding_window_smoothness_position_weight_;
      factor.velocity_acceleration_weight = sliding_window_smoothness_velocity_weight_;
      factor.gyro_bias_rate_weight = sliding_window_smoothness_bias_weight_;
      factor.accel_bias_rate_weight = sliding_window_smoothness_bias_weight_;
      try {
        sliding_window_optimizer_.add_trajectory_smoothness_factor(factor);
        window_factor_added = true;
      } catch (const std::exception & ex) {
        ++sliding_window_smoothness_factor_skip_count_;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "sliding window trajectory smoothness factor skipped: %s", ex.what());
      }
    }

    if (has_sliding_window_state_ && sliding_window_preintegrator_initialized_ &&
      sliding_window_preintegrator_.delta_t_s() > 0.0)
    {
      gaussian_lic_tracking::SlidingWindowImuFactor factor;
      factor.from_stamp_ns = last_sliding_window_stamp_ns_;
      factor.to_stamp_ns = input_pose.stamp_ns;
      factor.preintegration = sliding_window_preintegrator_;
      factor.gravity_w = imu_propagator_.gravity_w();
      factor.weight = sliding_window_imu_weight_;
      factor.rotation_weight = sliding_window_imu_rotation_weight_;
      factor.velocity_weight = sliding_window_imu_velocity_weight_;
      factor.position_weight = sliding_window_imu_position_weight_;
      factor.bias_weight = sliding_window_bias_weight_;
      try {
        sliding_window_optimizer_.add_imu_factor(factor);
        window_factor_added = true;
      } catch (const std::exception & ex) {
        ++sliding_window_imu_factor_skip_count_;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "sliding window IMU factor skipped: %s", ex.what());
      }
    }

    if (has_sliding_window_state_ && window_factor_added) {
      try {
        const auto summary = sliding_window_optimizer_.optimize();
        last_sliding_window_summary_ = summary;
        has_last_sliding_window_summary_ = true;
        gaussian_lic_tracking::SlidingWindowState optimized;
        if (sliding_window_optimizer_.get_state(input_pose.stamp_ns, optimized)) {
          output_pose.p_w_i = optimized.p_w_i;
          output_pose.q_w_i = optimized.q_w_i;
          output_pose.v_w_i = optimized.v_w_i;
          sliding_window_bias_.gyro = optimized.gyro_bias;
          sliding_window_bias_.accel = optimized.accel_bias;
          if (!imu_propagator_.initialized() ||
            imu_propagator_.state().stamp_ns <= optimized.stamp_ns)
          {
            gaussian_lic_tracking::ImuState corrected_state;
            corrected_state.stamp_ns = optimized.stamp_ns;
            corrected_state.p_w_i = optimized.p_w_i;
            corrected_state.q_w_i = optimized.q_w_i;
            corrected_state.v_w_i = optimized.v_w_i;
            corrected_state.gyro_bias = optimized.gyro_bias;
            corrected_state.accel_bias = optimized.accel_bias;
            imu_propagator_.reset(corrected_state);
            ++num_sliding_window_imu_reanchors_;
          }
        }
        sync_optimized_trajectory_controls();
        RCLCPP_DEBUG_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "sliding window states=%zu imu=%zu pose_priors=%zu dense_priors=%zu point=%zu plane=%zu visual=%zu se3_photo=%zu smooth=%zu cost %.6g -> %.6g",
          summary.state_count,
          summary.imu_factor_count,
          summary.pose_prior_count,
          summary.dense_prior_count,
          summary.point_factor_count,
          summary.plane_factor_count,
          summary.visual_factor_count,
          summary.se3_photometric_factor_count,
          summary.smoothness_factor_count,
          summary.initial_cost,
          summary.final_cost);
      } catch (const std::exception & ex) {
        ++sliding_window_optimization_skip_count_;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "sliding window optimization skipped: %s", ex.what());
      }
    }

    if (has_sliding_window_state_) {
      previous_sliding_window_stamp_ns_ = last_sliding_window_stamp_ns_;
    }
    has_sliding_window_state_ = true;
    last_sliding_window_stamp_ns_ = input_pose.stamp_ns;
    sliding_window_preintegrator_.reset(input_pose.stamp_ns, sliding_window_bias_);
    sliding_window_preintegrator_initialized_ = true;
    return output_pose;
  }

  void append_trajectory_control_pose(const gaussian_lic_tracking::TrajectoryPose & pose)
  {
    try {
      trajectory_manager_.add_or_update_control_pose(pose);
      if (!last_trajectory_control_stamp_ns_.has_value() ||
        pose.stamp_ns > last_trajectory_control_stamp_ns_.value())
      {
        last_trajectory_control_stamp_ns_ = pose.stamp_ns;
      }
    } catch (const std::exception & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "trajectory control pose skipped: %s", ex.what());
    }
  }

  void sync_optimized_trajectory_controls()
  {
    for (const auto & state : sliding_window_optimizer_.states()) {
      gaussian_lic_tracking::TrajectoryPose pose;
      pose.stamp_ns = state.stamp_ns;
      pose.p_w_i = state.p_w_i;
      pose.q_w_i = state.q_w_i;
      pose.v_w_i = state.v_w_i;
      append_trajectory_control_pose(pose);
    }
  }

  bool pending_factor_stamp_is_fresh(
    const std::optional<int64_t> & factor_stamp_ns,
    const int64_t target_stamp_ns) const
  {
    if (!factor_stamp_ns.has_value()) {
      return false;
    }
    return stamp_delta_is_within(factor_stamp_ns.value(), target_stamp_ns, visual_factor_max_dt_ns_);
  }

  static bool stamp_delta_is_within(
    const int64_t lhs_stamp_ns,
    const int64_t rhs_stamp_ns,
    const int64_t max_delta_ns)
  {
    const int64_t delta_ns = stamp_delta_ns(lhs_stamp_ns, rhs_stamp_ns);
    return delta_ns <= max_delta_ns;
  }

  static int64_t stamp_delta_ns(const int64_t lhs_stamp_ns, const int64_t rhs_stamp_ns)
  {
    return lhs_stamp_ns > rhs_stamp_ns
      ? lhs_stamp_ns - rhs_stamp_ns
      : rhs_stamp_ns - lhs_stamp_ns;
  }

  void cache_depth_frame(DepthFrame frame)
  {
    const auto insert_it = std::lower_bound(
      depth_frame_cache_.begin(), depth_frame_cache_.end(), frame.stamp_ns,
      [](const DepthFrame & cached, const int64_t stamp_ns) {
        return cached.stamp_ns < stamp_ns;
      });
    if (insert_it != depth_frame_cache_.end() && insert_it->stamp_ns == frame.stamp_ns) {
      *insert_it = std::move(frame);
    } else {
      depth_frame_cache_.insert(insert_it, std::move(frame));
    }

    const auto max_cache_size = static_cast<size_t>(depth_frame_cache_size_);
    while (depth_frame_cache_.size() > max_cache_size) {
      depth_frame_cache_.pop_front();
    }
  }

  void cache_rendered_frame(gaussian_lic_tracking::VisualFrame frame)
  {
    const auto insert_it = std::lower_bound(
      rendered_frame_cache_.begin(), rendered_frame_cache_.end(), frame.stamp_ns,
      [](const gaussian_lic_tracking::VisualFrame & cached, const int64_t stamp_ns) {
        return cached.stamp_ns < stamp_ns;
      });
    if (insert_it != rendered_frame_cache_.end() && insert_it->stamp_ns == frame.stamp_ns) {
      *insert_it = std::move(frame);
    } else {
      rendered_frame_cache_.insert(insert_it, std::move(frame));
    }

    const auto max_cache_size = static_cast<size_t>(rendered_frame_cache_size_);
    while (rendered_frame_cache_.size() > max_cache_size) {
      rendered_frame_cache_.pop_front();
    }
  }

  const DepthFrame * select_depth_frame_for_stamp(
    const int64_t image_stamp_ns,
    const size_t width,
    const size_t height,
    int64_t * selected_delta_ns = nullptr,
    bool * cache_had_size_match = nullptr) const
  {
    const DepthFrame * best = nullptr;
    int64_t best_delta_ns = std::numeric_limits<int64_t>::max();
    bool had_size_match = false;
    for (const auto & frame : depth_frame_cache_) {
      if (frame.width != width || frame.height != height) {
        continue;
      }
      had_size_match = true;
      const int64_t delta_ns = stamp_delta_ns(frame.stamp_ns, image_stamp_ns);
      if (delta_ns <= std::max<int64_t>(visual_factor_max_dt_ns_, 0LL) &&
        delta_ns < best_delta_ns)
      {
        best = &frame;
        best_delta_ns = delta_ns;
      }
    }
    if (selected_delta_ns != nullptr) {
      *selected_delta_ns = best == nullptr ? 0 : best_delta_ns;
    }
    if (cache_had_size_match != nullptr) {
      *cache_had_size_match = had_size_match;
    }
    return best;
  }

  const gaussian_lic_tracking::VisualFrame * select_rendered_frame_for_stamp(
    const int64_t image_stamp_ns,
    const size_t width,
    const size_t height,
    int64_t * selected_delta_ns = nullptr,
    bool * cache_had_size_match = nullptr) const
  {
    const gaussian_lic_tracking::VisualFrame * best = nullptr;
    int64_t best_delta_ns = std::numeric_limits<int64_t>::max();
    bool had_size_match = false;
    for (const auto & frame : rendered_frame_cache_) {
      if (frame.width != width || frame.height != height) {
        continue;
      }
      had_size_match = true;
      const int64_t delta_ns = stamp_delta_ns(frame.stamp_ns, image_stamp_ns);
      if (delta_ns <= std::max<int64_t>(visual_factor_max_dt_ns_, 0LL) &&
        delta_ns < best_delta_ns)
      {
        best = &frame;
        best_delta_ns = delta_ns;
      }
    }
    if (selected_delta_ns != nullptr) {
      *selected_delta_ns = best == nullptr ? 0 : best_delta_ns;
    }
    if (cache_had_size_match != nullptr) {
      *cache_had_size_match = had_size_match;
    }
    return best;
  }

  bool decode_image_gray(
    const sensor_msgs::msg::Image & msg,
    gaussian_lic_tracking::VisualFrame & frame) const
  {
    int channels = 0;
    int r_offset = 0;
    int g_offset = 0;
    int b_offset = 0;
    if (msg.encoding == "mono8" || msg.encoding == "8UC1") {
      channels = 1;
    } else if (msg.encoding == "rgb8") {
      channels = 3;
      r_offset = 0;
      g_offset = 1;
      b_offset = 2;
    } else if (msg.encoding == "bgr8") {
      channels = 3;
      r_offset = 2;
      g_offset = 1;
      b_offset = 0;
    } else if (msg.encoding == "rgba8") {
      channels = 4;
      r_offset = 0;
      g_offset = 1;
      b_offset = 2;
    } else if (msg.encoding == "bgra8") {
      channels = 4;
      r_offset = 2;
      g_offset = 1;
      b_offset = 0;
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "image encoding %s is not supported by the native tracking visual factor",
        msg.encoding.c_str());
      return false;
    }

    const size_t width = static_cast<size_t>(msg.width);
    const size_t height = static_cast<size_t>(msg.height);
    if (width == 0U || height == 0U || msg.step < width * static_cast<size_t>(channels)) {
      return false;
    }
    if (msg.data.size() < static_cast<size_t>(msg.step) * height) {
      return false;
    }

    frame.stamp_ns = gaussian_lic_tracking::stamp_to_nanoseconds(msg.header.stamp);
    frame.width = width;
    frame.height = height;
    frame.gray.assign(width * height, 0.0F);
    for (size_t row = 0; row < height; ++row) {
      const size_t row_offset = row * static_cast<size_t>(msg.step);
      for (size_t col = 0; col < width; ++col) {
        const size_t base = row_offset + col * static_cast<size_t>(channels);
        if (channels == 1) {
          frame.gray[row * width + col] = static_cast<float>(msg.data[base]) / 255.0F;
        } else {
          const float red = static_cast<float>(msg.data[base + static_cast<size_t>(r_offset)]);
          const float green = static_cast<float>(msg.data[base + static_cast<size_t>(g_offset)]);
          const float blue = static_cast<float>(msg.data[base + static_cast<size_t>(b_offset)]);
          frame.gray[row * width + col] = (0.299F * red + 0.587F * green + 0.114F * blue) / 255.0F;
        }
      }
    }
    return true;
  }

  bool decode_depth_image(const sensor_msgs::msg::Image & msg, DepthFrame & frame) const
  {
    const size_t width = static_cast<size_t>(msg.width);
    const size_t height = static_cast<size_t>(msg.height);
    if (width == 0U || height == 0U || msg.is_bigendian) {
      return false;
    }
    frame.stamp_ns = gaussian_lic_tracking::stamp_to_nanoseconds(msg.header.stamp);
    frame.width = width;
    frame.height = height;
    frame.depth_m.assign(width * height, 0.0F);
    if (msg.encoding == "32FC1") {
      if (msg.step < width * sizeof(float) || msg.data.size() < msg.step * height) {
        return false;
      }
      for (size_t row = 0; row < height; ++row) {
        const size_t row_offset = row * static_cast<size_t>(msg.step);
        for (size_t col = 0; col < width; ++col) {
          float value = 0.0F;
          std::memcpy(&value, &msg.data[row_offset + col * sizeof(float)], sizeof(value));
          frame.depth_m[row * width + col] = value;
        }
      }
      return true;
    }
    if (msg.encoding == "16UC1") {
      if (msg.step < width * sizeof(uint16_t) || msg.data.size() < msg.step * height) {
        return false;
      }
      for (size_t row = 0; row < height; ++row) {
        const size_t row_offset = row * static_cast<size_t>(msg.step);
        for (size_t col = 0; col < width; ++col) {
          uint16_t value = 0U;
          std::memcpy(&value, &msg.data[row_offset + col * sizeof(uint16_t)], sizeof(value));
          frame.depth_m[row * width + col] = static_cast<float>(value) * 0.001F;
        }
      }
      return true;
    }
    return false;
  }

  Se3PhotometricSampleBatch build_se3_photometric_samples(
    const gaussian_lic_tracking::VisualFrame & rendered,
    const gaussian_lic_tracking::VisualFrame & observed)
  {
    Se3PhotometricSampleBatch batch;
    int64_t depth_match_delta_ns = 0;
    bool depth_cache_had_size_match = false;
    const DepthFrame * depth_frame =
      select_depth_frame_for_stamp(
      observed.stamp_ns,
      observed.width,
      observed.height,
      &depth_match_delta_ns,
      &depth_cache_had_size_match);
    last_visual_depth_cache_size_ = depth_frame_cache_.size();
    last_visual_depth_match_delta_ns_ = depth_frame == nullptr ? 0 : depth_match_delta_ns;
    if (depth_frame == nullptr) {
      if (depth_frame_cache_.empty()) {
        ++visual_depth_miss_count_;
      } else if (!depth_cache_had_size_match) {
        ++visual_depth_size_mismatch_count_;
      } else {
        ++visual_depth_stale_count_;
      }
    }
    if (!has_camera_intrinsics_ || depth_frame == nullptr ||
      rendered.width < 3U || rendered.height < 3U ||
      rendered.width != observed.width || rendered.height != observed.height)
    {
      return batch;
    }
    const size_t pixel_count = observed.width * observed.height;
    if (rendered.gray.size() != pixel_count || observed.gray.size() != pixel_count ||
      depth_frame->depth_m.size() != pixel_count)
    {
      return batch;
    }
    const size_t max_samples = static_cast<size_t>(se3_photometric_max_samples_);
    const size_t stride = pixel_count > max_samples
      ? static_cast<size_t>(std::ceil(static_cast<double>(pixel_count) / static_cast<double>(max_samples)))
      : 1U;
    batch.samples.reserve(std::min(pixel_count, max_samples));
    double abs_residual_sum = 0.0;
    for (size_t index = 0; index < pixel_count; index += stride) {
      const size_t x = index % observed.width;
      const size_t y = index / observed.width;
      if (x == 0U || y == 0U || x + 1U >= observed.width || y + 1U >= observed.height) {
        continue;
      }
      ++batch.candidate_pixels;
      const float depth = depth_frame->depth_m[index];
      if (!std::isfinite(depth) ||
        static_cast<double>(depth) < se3_photometric_min_depth_m_ ||
        static_cast<double>(depth) > se3_photometric_max_depth_m_)
      {
        ++batch.rejected_depth_pixels;
        continue;
      }
      auto at = [&observed](const size_t px, const size_t py) {
          return static_cast<double>(observed.gray[py * observed.width + px]);
        };
      gaussian_lic_tracking::VisualSe3PhotometricSample sample;
      const double z = static_cast<double>(depth);
      sample.point_camera = Eigen::Vector3d{
        (static_cast<double>(x) - camera_intrinsics_.cx) * z / camera_intrinsics_.fx,
        (static_cast<double>(y) - camera_intrinsics_.cy) * z / camera_intrinsics_.fy,
        z};
      sample.image_gradient = Eigen::Vector2d{
        0.5 * (at(x + 1U, y) - at(x - 1U, y)),
        0.5 * (at(x, y + 1U) - at(x, y - 1U))};
      sample.residual = static_cast<double>(observed.gray[index] - rendered.gray[index]);
      const double gradient_norm = sample.image_gradient.norm();
      if (!std::isfinite(gradient_norm) || gradient_norm < se3_photometric_min_gradient_) {
        ++batch.rejected_gradient_pixels;
        continue;
      }
      const double abs_residual = std::abs(sample.residual);
      if (!std::isfinite(abs_residual) ||
        (se3_photometric_max_abs_residual_ > 0.0 &&
        abs_residual > se3_photometric_max_abs_residual_))
      {
        ++batch.rejected_residual_pixels;
        continue;
      }
      sample.weight = 1.0;
      if (se3_photometric_huber_delta_ > 0.0 && abs_residual > se3_photometric_huber_delta_) {
        sample.weight = se3_photometric_huber_delta_ / abs_residual;
      }
      batch.samples.push_back(sample);
      ++batch.accepted_pixels;
      abs_residual_sum += abs_residual;
    }
    if (batch.accepted_pixels > 0U) {
      batch.mean_abs_residual = abs_residual_sum / static_cast<double>(batch.accepted_pixels);
    }
    return batch;
  }

  const sensor_msgs::msg::PointField * find_field(
    const sensor_msgs::msg::PointCloud2 & msg,
    const std::string & field_name) const
  {
    const auto it = std::find_if(
      msg.fields.begin(), msg.fields.end(),
      [&field_name](const sensor_msgs::msg::PointField & field) {
        return field.name == field_name;
      });
    return it == msg.fields.end() ? nullptr : &(*it);
  }

  const sensor_msgs::msg::PointField * find_time_field(const sensor_msgs::msg::PointCloud2 & msg) const
  {
    if (!lidar_time_field_.empty() && lidar_time_field_ != "auto") {
      return find_field(msg, lidar_time_field_);
    }
    for (const std::string & field_name : {"offset_time", "time", "timestamp", "t"}) {
      if (const auto * field = find_field(msg, field_name); field != nullptr) {
        return field;
      }
    }
    return nullptr;
  }

  static bool read_numeric_field(
    const uint8_t * base,
    const sensor_msgs::msg::PointField & field,
    double & value)
  {
    switch (field.datatype) {
      case sensor_msgs::msg::PointField::INT8: {
          int8_t raw = 0;
          std::memcpy(&raw, base + field.offset, sizeof(raw));
          value = static_cast<double>(raw);
          return true;
        }
      case sensor_msgs::msg::PointField::UINT8: {
          uint8_t raw = 0;
          std::memcpy(&raw, base + field.offset, sizeof(raw));
          value = static_cast<double>(raw);
          return true;
        }
      case sensor_msgs::msg::PointField::INT16: {
          int16_t raw = 0;
          std::memcpy(&raw, base + field.offset, sizeof(raw));
          value = static_cast<double>(raw);
          return true;
        }
      case sensor_msgs::msg::PointField::UINT16: {
          uint16_t raw = 0;
          std::memcpy(&raw, base + field.offset, sizeof(raw));
          value = static_cast<double>(raw);
          return true;
        }
      case sensor_msgs::msg::PointField::INT32: {
          int32_t raw = 0;
          std::memcpy(&raw, base + field.offset, sizeof(raw));
          value = static_cast<double>(raw);
          return true;
        }
      case sensor_msgs::msg::PointField::UINT32: {
          uint32_t raw = 0;
          std::memcpy(&raw, base + field.offset, sizeof(raw));
          value = static_cast<double>(raw);
          return true;
        }
      case sensor_msgs::msg::PointField::FLOAT32: {
          float raw = 0.0F;
          std::memcpy(&raw, base + field.offset, sizeof(raw));
          value = static_cast<double>(raw);
          return true;
        }
      case sensor_msgs::msg::PointField::FLOAT64: {
          double raw = 0.0;
          std::memcpy(&raw, base + field.offset, sizeof(raw));
          value = raw;
          return true;
        }
      default:
        return false;
    }
  }

  static size_t point_field_scalar_size(const sensor_msgs::msg::PointField & field)
  {
    switch (field.datatype) {
      case sensor_msgs::msg::PointField::INT8:
      case sensor_msgs::msg::PointField::UINT8:
        return 1U;
      case sensor_msgs::msg::PointField::INT16:
      case sensor_msgs::msg::PointField::UINT16:
        return 2U;
      case sensor_msgs::msg::PointField::INT32:
      case sensor_msgs::msg::PointField::UINT32:
      case sensor_msgs::msg::PointField::FLOAT32:
        return 4U;
      case sensor_msgs::msg::PointField::FLOAT64:
        return 8U;
      default:
        return 0U;
    }
  }

  std::optional<int64_t> decode_point_stamp_ns(
    const double raw_time,
    const std::string & field_name,
    const int64_t cloud_stamp_ns) const
  {
    if (!std::isfinite(raw_time)) {
      return std::nullopt;
    }
    auto scale_to_ns = [](const double value, const double scale) {
        return static_cast<int64_t>(std::llround(value * scale));
      };
    int64_t time_ns = 0;
    bool offset_mode = true;
    if (lidar_time_unit_ == "seconds") {
      time_ns = scale_to_ns(raw_time, 1.0e9);
    } else if (lidar_time_unit_ == "milliseconds") {
      time_ns = scale_to_ns(raw_time, 1.0e6);
    } else if (lidar_time_unit_ == "microseconds") {
      time_ns = scale_to_ns(raw_time, 1.0e3);
    } else if (lidar_time_unit_ == "nanoseconds") {
      time_ns = static_cast<int64_t>(std::llround(raw_time));
    } else {
      const double abs_time = std::abs(raw_time);
      if (field_name == "offset_time") {
        time_ns = static_cast<int64_t>(std::llround(raw_time));
      } else if (abs_time > 1.0e17) {
        time_ns = static_cast<int64_t>(std::llround(raw_time));
        offset_mode = false;
      } else if (abs_time > 1.0e14) {
        time_ns = scale_to_ns(raw_time, 1.0e3);
        offset_mode = false;
      } else if ((field_name == "timestamp" || field_name == "t") && abs_time > 1.0e8) {
        time_ns = scale_to_ns(raw_time, 1.0e9);
        offset_mode = false;
      } else {
        time_ns = scale_to_ns(raw_time, 1.0e9);
      }
    }

    if (lidar_time_mode_ == "absolute") {
      offset_mode = false;
    } else if (lidar_time_mode_ == "offset") {
      offset_mode = true;
    }
    return offset_mode ? cloud_stamp_ns + time_ns : time_ns;
  }

  std::vector<DecodedLidarPoint> decode_pointcloud(
    const sensor_msgs::msg::PointCloud2 & msg,
    PointCloudFields & fields)
  {
    std::vector<DecodedLidarPoint> points;
    if (msg.is_bigendian) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "big-endian PointCloud2 is not supported by the native tracking LiDAR factor");
      return points;
    }

    fields = PointCloudFields{};
    for (const auto & field : msg.fields) {
      if (field.name == "x") {
        fields.x_field = &field;
        fields.x_offset = static_cast<int>(field.offset);
      } else if (field.name == "y") {
        fields.y_field = &field;
        fields.y_offset = static_cast<int>(field.offset);
      } else if (field.name == "z") {
        fields.z_field = &field;
        fields.z_offset = static_cast<int>(field.offset);
      }
    }
    fields.time_field = find_time_field(msg);
    fields.xyz_writable =
      fields.x_field != nullptr && fields.y_field != nullptr && fields.z_field != nullptr &&
      fields.x_field->datatype == sensor_msgs::msg::PointField::FLOAT32 &&
      fields.y_field->datatype == sensor_msgs::msg::PointField::FLOAT32 &&
      fields.z_field->datatype == sensor_msgs::msg::PointField::FLOAT32;
    if (fields.x_offset < 0 || fields.y_offset < 0 || fields.z_offset < 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "PointCloud2 must expose numeric x/y/z fields for the native tracking LiDAR factor");
      return points;
    }

    uint32_t max_offset = 0U;
    for (const auto * field : {fields.x_field, fields.y_field, fields.z_field, fields.time_field}) {
      if (field == nullptr) {
        continue;
      }
      const size_t scalar_size = point_field_scalar_size(*field);
      if (scalar_size == 0U) {
        continue;
      }
      max_offset = std::max(max_offset, field->offset + static_cast<uint32_t>(scalar_size));
    }
    if (msg.point_step < max_offset || msg.point_step == 0U) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "PointCloud2 point_step is too small for x/y/z fields");
      return points;
    }

    const size_t count = static_cast<size_t>(msg.width) * static_cast<size_t>(msg.height);
    points.reserve(count);
    const int64_t cloud_stamp_ns = gaussian_lic_tracking::stamp_to_nanoseconds(msg.header.stamp);
    for (size_t index = 0; index < count; ++index) {
      const size_t base = index * static_cast<size_t>(msg.point_step);
      if (base + max_offset > msg.data.size()) {
        break;
      }
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
      const uint8_t * point_base = msg.data.data() + base;
      if (!read_numeric_field(point_base, *fields.x_field, x) ||
        !read_numeric_field(point_base, *fields.y_field, y) ||
        !read_numeric_field(point_base, *fields.z_field, z))
      {
        continue;
      }
      if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
        DecodedLidarPoint point;
        point.point_i = Eigen::Vector3d{x, y, z};
        point.index = index;
        if (fields.time_field != nullptr) {
          double raw_time = 0.0;
          if (read_numeric_field(point_base, *fields.time_field, raw_time)) {
            const auto stamp_ns = decode_point_stamp_ns(raw_time, fields.time_field->name, cloud_stamp_ns);
            if (stamp_ns.has_value()) {
              point.stamp_ns = stamp_ns.value();
              point.has_stamp = true;
            }
          }
        }
        points.push_back(point);
      }
    }
    return points;
  }

  gaussian_lic_tracking::LidarDeskewResult deskew_decoded_points(
    const std::vector<DecodedLidarPoint> & points,
    const gaussian_lic_tracking::TrajectoryPose & reference_pose)
  {
    std::vector<gaussian_lic_tracking::TimedLidarPoint> timed_points;
    timed_points.reserve(points.size());
    for (const auto & point : points) {
      timed_points.push_back(
        gaussian_lic_tracking::TimedLidarPoint{point.point_i, point.stamp_ns, point.has_stamp});
    }
    return gaussian_lic_tracking::deskew_lidar_points(
      timed_points,
      reference_pose,
      [this](const int64_t stamp_ns, gaussian_lic_tracking::TrajectoryPose & pose) {
        ++trajectory_deskew_queries_;
        if (trajectory_manager_.query_pose(stamp_ns, pose)) {
          ++trajectory_deskew_hits_;
          return true;
        }
        gaussian_lic_tracking::ImuState state;
        if (!imu_propagator_.query_state(stamp_ns, state)) {
          return false;
        }
        pose.stamp_ns = state.stamp_ns;
        pose.p_w_i = state.p_w_i;
        pose.q_w_i = state.q_w_i;
        pose.v_w_i = state.v_w_i;
        return true;
      });
  }

  void write_deskewed_points(
    sensor_msgs::msg::PointCloud2 & msg,
    const PointCloudFields & fields,
    const std::vector<DecodedLidarPoint> & decoded_points,
    const std::vector<Eigen::Vector3d> & deskewed_points) const
  {
    if (!fields.xyz_writable || decoded_points.size() != deskewed_points.size()) {
      return;
    }
    for (size_t i = 0; i < decoded_points.size(); ++i) {
      const size_t base = decoded_points[i].index * static_cast<size_t>(msg.point_step);
      if (base + static_cast<size_t>(std::max({fields.x_offset, fields.y_offset, fields.z_offset})) + sizeof(float) >
        msg.data.size())
      {
        continue;
      }
      const float x = static_cast<float>(deskewed_points[i].x());
      const float y = static_cast<float>(deskewed_points[i].y());
      const float z = static_cast<float>(deskewed_points[i].z());
      std::memcpy(msg.data.data() + base + static_cast<size_t>(fields.x_offset), &x, sizeof(float));
      std::memcpy(msg.data.data() + base + static_cast<size_t>(fields.y_offset), &y, sizeof(float));
      std::memcpy(msg.data.data() + base + static_cast<size_t>(fields.z_offset), &z, sizeof(float));
    }
  }

  bool should_insert_lidar_keyframe(
    const gaussian_lic_tracking::TrajectoryPose & pose,
    const size_t point_count) const
  {
    const auto & config = lidar_factor_.config();
    if (point_count < config.min_points) {
      return false;
    }
    if (!has_lidar_keyframe_ || lidar_factor_.map_size() < config.min_points) {
      return true;
    }
    return (pose.p_w_i - last_lidar_keyframe_pose_.p_w_i).norm() >= lidar_keyframe_translation_m_;
  }

  void publish_tracking_status(const builtin_interfaces::msg::Time & stamp)
  {
    gaussian_lic_msgs::msg::TrackingStatus status;
    status.header.stamp = stamp;
    status.header.frame_id = world_frame_;
    status.executor_callback_serialization_enabled = serialize_callbacks_;
    status.sensor_qos_reliability = sensor_qos_reliability_;
    status.sensor_qos_history = sensor_qos_history_;
    status.sensor_qos_depth = static_cast<uint32_t>(sensor_qos_depth_);
    status.signed_nanosecond_time_math_enabled = true;
    status.last_image_stamp_ns = last_image_stamp_ns_;
    status.last_pointcloud_stamp_ns = last_pointcloud_stamp_ns_;
    status.last_imu_stamp_ns = last_imu_stamp_ns_;
    status.image_stamp_regressions = image_stamp_regressions_;
    status.depth_stamp_regressions = depth_stamp_regressions_;
    status.rendered_stamp_regressions = rendered_stamp_regressions_;
    status.pointcloud_stamp_regressions = pointcloud_stamp_regressions_;
    status.imu_stamp_regressions = imu_stamp_regressions_;
    status.imu_invalid_measurements = imu_invalid_measurements_;
    if (num_published_poses_ == 0U) {
      status.state = gaussian_lic_msgs::msg::TrackingStatus::STATE_INITIALIZING;
      status.status_text = "initializing";
    } else if (enable_sliding_window_optimizer_ && !has_last_sliding_window_summary_) {
      status.state = gaussian_lic_msgs::msg::TrackingStatus::STATE_DEGRADED;
      status.status_text = "tracking_waiting_for_sliding_window";
    } else if (enable_sliding_window_optimizer_ &&
      last_sliding_window_summary_.normal_equation_degenerate)
    {
      status.state = gaussian_lic_msgs::msg::TrackingStatus::STATE_DEGRADED;
      status.status_text = "tracking_degraded_sliding_window_degenerate";
    } else {
      status.state = gaussian_lic_msgs::msg::TrackingStatus::STATE_TRACKING;
      status.status_text = enable_sliding_window_optimizer_ ? "tracking_with_sliding_window" : "tracking";
    }

    status.num_raw_images = num_raw_images_;
    status.num_raw_pointclouds = num_raw_pointclouds_;
    status.num_raw_imus = num_raw_imus_;
    status.num_published_poses = num_published_poses_;

    status.num_lidar_keyframes = num_lidar_keyframes_;
    status.lidar_map_points = static_cast<uint64_t>(lidar_factor_.map_size());
    status.last_lidar_points = static_cast<uint64_t>(last_lidar_points_);
    status.last_lidar_matches = static_cast<uint64_t>(last_lidar_matches_);
    status.last_window_point_correspondences =
      static_cast<uint64_t>(last_window_point_correspondences_);
    status.last_window_plane_correspondences =
      static_cast<uint64_t>(last_window_plane_correspondences_);
    status.total_window_point_correspondences =
      static_cast<uint64_t>(total_window_point_correspondences_);
    status.total_window_plane_correspondences =
      static_cast<uint64_t>(total_window_plane_correspondences_);
    status.last_lidar_mean_residual_m = last_lidar_mean_residual_m_;

    const auto & summary = last_sliding_window_summary_;
    status.sliding_window_enabled = enable_sliding_window_optimizer_;
    status.sliding_window_states = has_last_sliding_window_summary_
      ? static_cast<uint64_t>(summary.state_count)
      : static_cast<uint64_t>(sliding_window_optimizer_.states().size());
    status.sliding_window_imu_factors = static_cast<uint64_t>(summary.imu_factor_count);
    status.sliding_window_pose_priors = static_cast<uint64_t>(summary.pose_prior_count);
    status.sliding_window_dense_priors = static_cast<uint64_t>(summary.dense_prior_count);
    status.sliding_window_point_factors = static_cast<uint64_t>(summary.point_factor_count);
    status.sliding_window_plane_factors = static_cast<uint64_t>(summary.plane_factor_count);
    status.sliding_window_visual_factors = static_cast<uint64_t>(summary.visual_factor_count);
    status.sliding_window_se3_photometric_factors =
      static_cast<uint64_t>(summary.se3_photometric_factor_count);
    status.sliding_window_smoothness_factors =
      static_cast<uint64_t>(summary.smoothness_factor_count);
    status.sliding_window_point_factor_skip_count = sliding_window_point_factor_skip_count_;
    status.sliding_window_plane_factor_skip_count = sliding_window_plane_factor_skip_count_;
    status.sliding_window_visual_factor_skip_count = sliding_window_visual_factor_skip_count_;
    status.sliding_window_se3_photometric_factor_skip_count =
      sliding_window_se3_photometric_factor_skip_count_;
    status.sliding_window_smoothness_factor_skip_count =
      sliding_window_smoothness_factor_skip_count_;
    status.sliding_window_imu_factor_skip_count = sliding_window_imu_factor_skip_count_;
    status.sliding_window_optimization_skip_count = sliding_window_optimization_skip_count_;
    status.sliding_window_marginalized_states = static_cast<uint64_t>(summary.marginalized_state_count);
    status.sliding_window_dense_prior_rows = static_cast<uint64_t>(summary.dense_prior_rows);
    status.sliding_window_dense_prior_cols = static_cast<uint64_t>(summary.dense_prior_cols);
    status.sliding_window_dense_prior_rank = static_cast<uint64_t>(summary.dense_prior_rank);
    status.sliding_window_normal_equation_rows =
      static_cast<uint64_t>(summary.normal_equation_rows);
    status.sliding_window_normal_equation_cols =
      static_cast<uint64_t>(summary.normal_equation_cols);
    status.sliding_window_normal_equation_rank =
      static_cast<uint64_t>(summary.normal_equation_rank);
    status.sliding_window_iterations = static_cast<uint64_t>(summary.iterations);
    status.sliding_window_accepted_steps = static_cast<uint64_t>(summary.accepted_steps);
    status.sliding_window_rejected_steps = static_cast<uint64_t>(summary.rejected_steps);
    status.sliding_window_limited_steps = static_cast<uint64_t>(summary.limited_steps);
    status.sliding_window_initial_cost = summary.initial_cost;
    status.sliding_window_final_cost = summary.final_cost;
    status.sliding_window_last_step_norm = summary.last_step_norm;
    status.sliding_window_last_step_scale = summary.last_step_scale;
    status.sliding_window_last_damping = summary.last_damping;
    status.sliding_window_dense_prior_min_singular_value = summary.dense_prior_min_singular_value;
    status.sliding_window_dense_prior_max_singular_value = summary.dense_prior_max_singular_value;
    status.sliding_window_normal_equation_min_singular_value =
      summary.normal_equation_min_singular_value;
    status.sliding_window_normal_equation_max_singular_value =
      summary.normal_equation_max_singular_value;
    status.sliding_window_normal_equation_condition_number =
      summary.normal_equation_condition_number;
    status.sliding_window_gyro_bias_norm = summary.gyro_bias_norm;
    status.sliding_window_accel_bias_norm = summary.accel_bias_norm;
    status.sliding_window_gyro_bias_observability = summary.gyro_bias_observability;
    status.sliding_window_accel_bias_observability = summary.accel_bias_observability;
    status.sliding_window_converged = summary.converged;
    status.sliding_window_normal_equation_degenerate = summary.normal_equation_degenerate;
    status.sliding_window_imu_reanchors = num_sliding_window_imu_reanchors_;
    status.trajectory_control_poses = static_cast<uint64_t>(trajectory_manager_.size());
    status.trajectory_deskew_queries = trajectory_deskew_queries_;
    status.trajectory_deskew_hits = trajectory_deskew_hits_;

    status.gaussian_snapshot_points = static_cast<uint64_t>(gaussian_snapshot_.point_count());
    status.gaussian_snapshot_expected_total = last_gaussian_total_count_;
    status.gaussian_snapshot_chunks_received = gaussian_snapshot_chunks_received_;
    status.gaussian_snapshot_expected_chunks = last_gaussian_chunk_count_;
    status.gaussian_snapshot_complete = gaussian_snapshot_.complete();

    status.visual_factor_enabled = enable_visual_factor_;
    status.visual_rendered_cache_size = static_cast<uint64_t>(last_visual_rendered_cache_size_);
    status.visual_rendered_match_delta_ns = last_visual_rendered_match_delta_ns_;
    status.visual_rendered_miss_count = visual_rendered_miss_count_;
    status.visual_rendered_stale_count = visual_rendered_stale_count_;
    status.visual_rendered_size_mismatch_count = visual_rendered_size_mismatch_count_;
    status.visual_depth_cache_size = static_cast<uint64_t>(last_visual_depth_cache_size_);
    status.visual_depth_match_delta_ns = last_visual_depth_match_delta_ns_;
    status.visual_depth_miss_count = visual_depth_miss_count_;
    status.visual_depth_stale_count = visual_depth_stale_count_;
    status.visual_depth_size_mismatch_count = visual_depth_size_mismatch_count_;
    status.visual_alignment_pending_stale_drops = visual_alignment_pending_stale_drops_;
    status.visual_se3_photometric_pending_stale_drops = visual_se3_photometric_pending_stale_drops_;
    status.visual_alignment_valid = last_visual_alignment_.valid;
    status.visual_rmse = last_visual_residual_.valid ? last_visual_residual_.rmse : 0.0;
    status.visual_subpixel_dx = last_visual_alignment_.valid ? last_visual_alignment_.subpixel_dx : 0.0;
    status.visual_subpixel_dy = last_visual_alignment_.valid ? last_visual_alignment_.subpixel_dy : 0.0;
    status.visual_photometric_valid = last_visual_photometric_linearization_.valid;
    status.visual_photometric_pixels =
      static_cast<uint64_t>(last_visual_photometric_linearization_.compared_pixels);
    status.visual_photometric_cost = last_visual_photometric_linearization_.valid
      ? last_visual_photometric_linearization_.cost
      : 0.0;
    status.visual_photometric_step_dx = last_visual_photometric_linearization_.valid
      ? last_visual_photometric_linearization_.gauss_newton_step.x()
      : 0.0;
    status.visual_photometric_step_dy = last_visual_photometric_linearization_.valid
      ? last_visual_photometric_linearization_.gauss_newton_step.y()
      : 0.0;
    const bool se3_photometric_valid = last_visual_se3_photometric_linearization_.valid &&
      last_visual_se3_photometric_linearization_.sample_count >=
      static_cast<size_t>(se3_photometric_min_samples_);
    status.visual_se3_photometric_valid = se3_photometric_valid;
    status.visual_se3_photometric_candidates =
      static_cast<uint64_t>(last_visual_se3_photometric_candidate_pixels_);
    status.visual_se3_photometric_samples =
      static_cast<uint64_t>(last_visual_se3_photometric_accepted_pixels_);
    status.visual_se3_photometric_rejected_depth =
      static_cast<uint64_t>(last_visual_se3_photometric_rejected_depth_pixels_);
    status.visual_se3_photometric_rejected_gradient =
      static_cast<uint64_t>(last_visual_se3_photometric_rejected_gradient_pixels_);
    status.visual_se3_photometric_rejected_residual =
      static_cast<uint64_t>(last_visual_se3_photometric_rejected_residual_pixels_);
    status.visual_se3_photometric_inlier_ratio = last_visual_se3_photometric_candidate_pixels_ > 0U
      ? static_cast<double>(last_visual_se3_photometric_accepted_pixels_) /
      static_cast<double>(last_visual_se3_photometric_candidate_pixels_)
      : 0.0;
    status.visual_se3_photometric_mean_abs_residual =
      last_visual_se3_photometric_mean_abs_residual_;
    status.visual_se3_photometric_cost = se3_photometric_valid
      ? last_visual_se3_photometric_linearization_.cost
      : 0.0;
    status.visual_se3_photometric_step_norm = se3_photometric_valid
      ? last_visual_se3_photometric_linearization_.gauss_newton_step.norm()
      : 0.0;
    tracking_status_pub_->publish(status);
  }

  std::string raw_image_topic_;
  std::string raw_camera_info_topic_;
  std::string raw_depth_topic_;
  std::string raw_pointcloud_topic_;
  std::string raw_imu_topic_;
  std::string image_topic_;
  std::string camera_info_topic_;
  std::string depth_topic_;
  std::string pointcloud_topic_;
  std::string pose_topic_;
  std::string odometry_topic_;
  std::string path_topic_;
  std::string tracking_status_topic_;
  std::string rendered_image_topic_;
  std::string gaussian_map_topic_;
  std::string world_frame_;
  std::string child_frame_;
  bool publish_tf_{false};
  int max_path_length_{5000};
  int sensor_qos_depth_{5};
  std::string sensor_qos_reliability_{"best_effort"};
  std::string sensor_qos_history_{"keep_last"};
  QosProfileParams raw_image_qos_;
  QosProfileParams raw_camera_info_qos_;
  QosProfileParams raw_depth_qos_;
  QosProfileParams raw_pointcloud_qos_;
  QosProfileParams raw_imu_qos_;
  QosProfileParams image_qos_;
  QosProfileParams camera_info_qos_;
  QosProfileParams depth_qos_;
  QosProfileParams pointcloud_qos_;
  QosProfileParams pose_qos_;
  QosProfileParams frontend_odometry_qos_;
  bool serialize_callbacks_{true};
  bool enable_visual_factor_{true};
  bool enable_gaussian_snapshot_{true};
  int visual_max_pixels_{200000};
  int64_t visual_factor_max_dt_ns_{150000000LL};
  int depth_frame_cache_size_{8};
  int rendered_frame_cache_size_{8};
  Eigen::Vector3d p_i_c_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond q_i_c_{Eigen::Quaterniond::Identity()};
  int visual_alignment_max_shift_px_{8};
  bool enable_visual_alignment_window_factor_{true};
  double visual_alignment_meters_per_pixel_{0.01};
  double visual_alignment_window_weight_{1.0};
  double visual_alignment_huber_delta_m_{0.05};
  bool enable_se3_photometric_window_factor_{true};
  double se3_photometric_window_weight_{1.0};
  double se3_photometric_factor_huber_delta_{1.0};
  int se3_photometric_max_samples_{2000};
  int se3_photometric_min_samples_{16};
  double se3_photometric_min_depth_m_{0.05};
  double se3_photometric_max_depth_m_{200.0};
  double se3_photometric_min_gradient_{1.0e-4};
  double se3_photometric_huber_delta_{0.15};
  double se3_photometric_max_abs_residual_{1.0};
  bool enable_lio_factor_{true};
  bool enable_lidar_plane_factor_{true};
  bool enable_lidar_deskew_{true};
  bool enable_sliding_window_optimizer_{true};
  bool enable_gaussian_snapshot_lidar_factor_{true};
  std::string lidar_time_field_{"auto"};
  std::string lidar_time_unit_{"auto"};
  std::string lidar_time_mode_{"auto"};
  int imu_history_size_{4000};
  int64_t trajectory_control_interval_ns_{50000000LL};
  int sliding_window_max_states_{12};
  int sliding_window_max_iterations_{3};
  double sliding_window_max_rotation_step_rad_{0.5};
  double sliding_window_max_translation_step_m_{1.0};
  double sliding_window_max_velocity_step_mps_{5.0};
  double sliding_window_max_bias_step_{1.0};
  double sliding_window_max_normal_equation_condition_{1.0e12};
  double sliding_window_min_normal_equation_rank_ratio_{0.0};
  double gaussian_snapshot_lidar_min_opacity_{0.01};
  double sliding_window_imu_weight_{1.0};
  double sliding_window_imu_rotation_weight_{1.0};
  double sliding_window_imu_velocity_weight_{1.0};
  double sliding_window_imu_position_weight_{1.0};
  double sliding_window_bias_weight_{1.0};
  double sliding_window_pose_translation_weight_{2.0};
  double sliding_window_pose_rotation_weight_{2.0};
  bool enable_sliding_window_smoothness_factor_{true};
  double sliding_window_smoothness_rotation_weight_{0.1};
  double sliding_window_smoothness_position_weight_{0.1};
  double sliding_window_smoothness_velocity_weight_{0.1};
  double sliding_window_smoothness_bias_weight_{0.1};
  int lidar_min_points_{32};
  int lidar_max_frame_points_{2000};
  int lidar_max_map_points_{20000};
  double lidar_nearest_distance_m_{0.35};
  double lidar_correction_gain_{0.7};
  double lidar_max_correction_m_{0.25};
  double lidar_max_rotation_rad_{0.08};
  double lidar_robust_kernel_m_{0.15};
  int lidar_plane_min_neighbors_{5};
  double lidar_plane_max_condition_{0.2};
  double lidar_keyframe_translation_m_{0.25};
  Eigen::Vector3d p_i_l_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond q_i_l_{Eigen::Quaterniond::Identity()};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rendered_image_sub_;
  rclcpp::Subscription<gaussian_lic_msgs::msg::GaussianArray>::SharedPtr gaussian_map_sub_;
  std::mutex callback_mutex_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<gaussian_lic_msgs::msg::TrackingStatus>::SharedPtr tracking_status_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  gaussian_lic_tracking::ImuPropagator imu_propagator_;
  gaussian_lic_tracking::TrajectoryManager trajectory_manager_;
  gaussian_lic_tracking::SlidingWindowOptimizer sliding_window_optimizer_;
  gaussian_lic_tracking::ImuPreintegrator sliding_window_preintegrator_;
  gaussian_lic_tracking::ImuBias sliding_window_bias_;
  bool sliding_window_preintegrator_initialized_{false};
  bool has_sliding_window_state_{false};
  int64_t last_sliding_window_stamp_ns_{0};
  std::optional<int64_t> previous_sliding_window_stamp_ns_;
  std::optional<int64_t> last_trajectory_control_stamp_ns_;
  uint64_t num_sliding_window_imu_reanchors_{0};
  uint64_t sliding_window_point_factor_skip_count_{0};
  uint64_t sliding_window_plane_factor_skip_count_{0};
  uint64_t sliding_window_visual_factor_skip_count_{0};
  uint64_t sliding_window_se3_photometric_factor_skip_count_{0};
  uint64_t sliding_window_smoothness_factor_skip_count_{0};
  uint64_t sliding_window_imu_factor_skip_count_{0};
  uint64_t sliding_window_optimization_skip_count_{0};
  uint64_t trajectory_deskew_queries_{0};
  uint64_t trajectory_deskew_hits_{0};
  gaussian_lic_tracking::LidarFactor lidar_factor_;
  gaussian_lic_tracking::TrajectoryPose last_lidar_keyframe_pose_;
  bool has_lidar_keyframe_{false};
  gaussian_lic_tracking::GaussianSnapshot gaussian_snapshot_;
  gaussian_lic_tracking::VisualFactor visual_factor_;
  std::deque<gaussian_lic_tracking::VisualFrame> rendered_frame_cache_;
  size_t last_visual_rendered_cache_size_{0};
  int64_t last_visual_rendered_match_delta_ns_{0};
  uint64_t visual_rendered_miss_count_{0};
  uint64_t visual_rendered_stale_count_{0};
  uint64_t visual_rendered_size_mismatch_count_{0};
  gaussian_lic_tracking::VisualResidual last_visual_residual_;
  gaussian_lic_tracking::VisualAlignment last_visual_alignment_;
  gaussian_lic_tracking::VisualPhotometricLinearization last_visual_photometric_linearization_;
  gaussian_lic_tracking::VisualSe3PhotometricLinearization last_visual_se3_photometric_linearization_;
  gaussian_lic_tracking::VisualSe3PhotometricLinearization pending_visual_se3_photometric_linearization_;
  gaussian_lic_tracking::VisualAlignment pending_visual_alignment_;
  std::optional<int64_t> pending_visual_alignment_stamp_ns_;
  std::optional<int64_t> pending_visual_se3_photometric_stamp_ns_;
  size_t last_visual_se3_photometric_candidate_pixels_{0};
  size_t last_visual_se3_photometric_accepted_pixels_{0};
  size_t last_visual_se3_photometric_rejected_depth_pixels_{0};
  size_t last_visual_se3_photometric_rejected_gradient_pixels_{0};
  size_t last_visual_se3_photometric_rejected_residual_pixels_{0};
  double last_visual_se3_photometric_mean_abs_residual_{0.0};
  gaussian_lic_tracking::VisualCameraIntrinsics camera_intrinsics_;
  std::deque<DepthFrame> depth_frame_cache_;
  size_t last_visual_depth_cache_size_{0};
  int64_t last_visual_depth_match_delta_ns_{0};
  uint64_t visual_depth_miss_count_{0};
  uint64_t visual_depth_stale_count_{0};
  uint64_t visual_depth_size_mismatch_count_{0};
  bool has_camera_intrinsics_{false};
  bool has_pending_visual_se3_photometric_{false};
  bool has_pending_visual_alignment_{false};
  uint64_t visual_alignment_pending_stale_drops_{0};
  uint64_t visual_se3_photometric_pending_stale_drops_{0};
  gaussian_lic_tracking::SlidingWindowSummary last_sliding_window_summary_;
  bool has_last_sliding_window_summary_{false};
  uint64_t num_raw_images_{0};
  uint64_t num_raw_pointclouds_{0};
  uint64_t num_raw_imus_{0};
  uint64_t num_published_poses_{0};
  int64_t last_image_stamp_ns_{0};
  int64_t last_pointcloud_stamp_ns_{0};
  int64_t last_imu_stamp_ns_{0};
  std::optional<int64_t> last_image_input_stamp_ns_;
  std::optional<int64_t> last_depth_input_stamp_ns_;
  std::optional<int64_t> last_rendered_input_stamp_ns_;
  std::optional<int64_t> last_pointcloud_input_stamp_ns_;
  std::optional<int64_t> last_imu_input_stamp_ns_;
  uint64_t image_stamp_regressions_{0};
  uint64_t depth_stamp_regressions_{0};
  uint64_t rendered_stamp_regressions_{0};
  uint64_t pointcloud_stamp_regressions_{0};
  uint64_t imu_stamp_regressions_{0};
  uint64_t imu_invalid_measurements_{0};
  uint64_t num_lidar_keyframes_{0};
  size_t last_lidar_points_{0};
  size_t last_lidar_matches_{0};
  size_t last_window_point_correspondences_{0};
  size_t last_window_plane_correspondences_{0};
  uint64_t total_window_point_correspondences_{0};
  uint64_t total_window_plane_correspondences_{0};
  double last_lidar_mean_residual_m_{0.0};
  int64_t last_gaussian_snapshot_stamp_ns_{0};
  uint32_t last_gaussian_total_count_{0};
  uint32_t last_gaussian_chunk_count_{0};
  uint32_t gaussian_snapshot_chunks_received_{0};
  size_t last_gaussian_chunk_size_{0};
  nav_msgs::msg::Path path_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrackingNode>());
  rclcpp::shutdown();
  return 0;
}
