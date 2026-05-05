// SPDX-License-Identifier: GPL-3.0-or-later

#include <algorithm>
#include <cmath>
#include <cstring>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <gaussian_lic_tracking/gaussian_snapshot.hpp>
#include <gaussian_lic_tracking/imu_propagator.hpp>
#include <gaussian_lic_tracking/lidar_deskew.hpp>
#include <gaussian_lic_tracking/lidar_factor.hpp>
#include <gaussian_lic_tracking/sliding_window_optimizer.hpp>
#include <gaussian_lic_tracking/time.hpp>
#include <gaussian_lic_tracking/visual_factor.hpp>
#include <gaussian_lic_msgs/msg/gaussian_array.hpp>
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
    rendered_image_topic_ = declare_parameter<std::string>("rendered_image_topic", "/gaussian_lic/rendered_image");
    gaussian_map_topic_ = declare_parameter<std::string>("gaussian_map_topic", "/gaussian_lic/gaussian_map");
    world_frame_ = declare_parameter<std::string>("world_frame", "map");
    child_frame_ = declare_parameter<std::string>("child_frame", "base_link");
    publish_tf_ = declare_parameter<bool>("publish_tf", false);
    max_path_length_ = declare_parameter<int>("max_path_length", 5000);
    sensor_qos_depth_ = declare_parameter<int>("sensor_qos_depth", 5);
    sensor_qos_reliability_ = declare_parameter<std::string>("sensor_qos_reliability", "best_effort");
    enable_visual_factor_ = declare_parameter<bool>("enable_visual_factor", true);
    enable_gaussian_snapshot_ = declare_parameter<bool>("enable_gaussian_snapshot", true);
    visual_max_pixels_ = declare_parameter<int>("visual_max_pixels", 200000);
    enable_lio_factor_ = declare_parameter<bool>("enable_lio_factor", true);
    lidar_min_points_ = declare_parameter<int>("lidar_min_points", 32);
    lidar_max_frame_points_ = declare_parameter<int>("lidar_max_frame_points", 2000);
    lidar_max_map_points_ = declare_parameter<int>("lidar_max_map_points", 20000);
    lidar_nearest_distance_m_ = declare_parameter<double>("lidar_nearest_distance_m", 0.35);
    lidar_correction_gain_ = declare_parameter<double>("lidar_correction_gain", 0.7);
    lidar_max_correction_m_ = declare_parameter<double>("lidar_max_correction_m", 0.25);
    lidar_max_rotation_rad_ = declare_parameter<double>("lidar_max_rotation_rad", 0.08);
    lidar_keyframe_translation_m_ = declare_parameter<double>("lidar_keyframe_translation_m", 0.25);
    enable_lidar_deskew_ = declare_parameter<bool>("enable_lidar_deskew", true);
    lidar_time_field_ = declare_parameter<std::string>("lidar_time_field", "auto");
    lidar_time_unit_ = declare_parameter<std::string>("lidar_time_unit", "auto");
    lidar_time_mode_ = declare_parameter<std::string>("lidar_time_mode", "auto");
    imu_history_size_ = declare_parameter<int>("imu_history_size", 4000);
    enable_sliding_window_optimizer_ = declare_parameter<bool>("enable_sliding_window_optimizer", false);
    sliding_window_max_states_ = declare_parameter<int>("sliding_window_max_states", 12);
    sliding_window_max_iterations_ = declare_parameter<int>("sliding_window_max_iterations", 3);
    sliding_window_imu_weight_ = declare_parameter<double>("sliding_window_imu_weight", 1.0);
    sliding_window_bias_weight_ = declare_parameter<double>("sliding_window_bias_weight", 1.0);
    sliding_window_pose_translation_weight_ =
      declare_parameter<double>("sliding_window_pose_translation_weight", 2.0);
    sliding_window_pose_rotation_weight_ =
      declare_parameter<double>("sliding_window_pose_rotation_weight", 2.0);
    const auto gravity = declare_parameter<std::vector<double>>("imu_gravity_w", std::vector<double>{0.0, 0.0, 0.0});
    if (gravity.size() == 3U) {
      imu_propagator_.set_gravity_w(Eigen::Vector3d{gravity[0], gravity[1], gravity[2]});
    }
    imu_propagator_.set_max_history_size(static_cast<size_t>(std::max(imu_history_size_, 2)));
    gaussian_lic_tracking::SlidingWindowConfig window_config;
    window_config.max_states = static_cast<size_t>(std::max(sliding_window_max_states_, 2));
    window_config.max_iterations = static_cast<size_t>(std::max(sliding_window_max_iterations_, 1));
    sliding_window_optimizer_.set_config(window_config);

    gaussian_lic_tracking::LidarFactorConfig lidar_config;
    lidar_config.min_points = static_cast<size_t>(std::max(lidar_min_points_, 1));
    lidar_config.max_frame_points = static_cast<size_t>(std::max(lidar_max_frame_points_, 1));
    lidar_config.max_map_points = static_cast<size_t>(std::max(lidar_max_map_points_, 1));
    lidar_config.nearest_distance_m = lidar_nearest_distance_m_;
    lidar_config.correction_gain = lidar_correction_gain_;
    lidar_config.max_correction_m = lidar_max_correction_m_;
    lidar_config.max_rotation_rad = lidar_max_rotation_rad_;
    lidar_factor_.set_config(lidar_config);
    visual_factor_.set_max_pixels(static_cast<size_t>(std::max(visual_max_pixels_, 1)));

    auto qos = make_sensor_qos();
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      raw_image_topic_, qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
        handle_image(*msg);
      });
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      raw_camera_info_topic_, qos,
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
        camera_info_pub_->publish(*msg);
      });
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      raw_depth_topic_, qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
        depth_pub_->publish(*msg);
      });
    pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      raw_pointcloud_topic_, qos,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        handle_pointcloud(*msg);
      });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      raw_imu_topic_, qos,
      [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
        handle_imu(*msg);
      });

    image_pub_ = create_publisher<sensor_msgs::msg::Image>(image_topic_, qos);
    camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic_, qos);
    depth_pub_ = create_publisher<sensor_msgs::msg::Image>(depth_topic_, qos);
    pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud_topic_, qos);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_, 20);
    odometry_pub_ = create_publisher<nav_msgs::msg::Odometry>(odometry_topic_, 20);
    path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic_, rclcpp::QoS(1).transient_local().reliable());
    rendered_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      rendered_image_topic_, rclcpp::QoS(1).transient_local().reliable(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
        handle_rendered_image(*msg);
      });
    if (enable_gaussian_snapshot_) {
      gaussian_map_sub_ = create_subscription<gaussian_lic_msgs::msg::GaussianArray>(
        gaussian_map_topic_, rclcpp::QoS(1).transient_local().reliable(),
        [this](gaussian_lic_msgs::msg::GaussianArray::ConstSharedPtr msg) {
          handle_gaussian_snapshot(*msg);
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

  rclcpp::QoS make_sensor_qos() const
  {
    rclcpp::QoS qos(static_cast<size_t>(std::max(sensor_qos_depth_, 1)));
    qos.keep_last(static_cast<size_t>(std::max(sensor_qos_depth_, 1)));
    qos.durability_volatile();
    if (sensor_qos_reliability_ == "reliable") {
      qos.reliable();
    } else {
      qos.best_effort();
    }
    return qos;
  }

  void handle_imu(const sensor_msgs::msg::Imu & msg)
  {
    try {
      const int64_t stamp_ns = gaussian_lic_tracking::stamp_to_nanoseconds(msg.header.stamp);
      imu_propagator_.add_measurement(
        stamp_ns,
        Eigen::Vector3d{
          msg.angular_velocity.x,
          msg.angular_velocity.y,
          msg.angular_velocity.z},
        Eigen::Vector3d{
          msg.linear_acceleration.x,
          msg.linear_acceleration.y,
          msg.linear_acceleration.z});
      if (enable_sliding_window_optimizer_) {
        sliding_window_preintegrator_.add_measurement(
          stamp_ns,
          Eigen::Vector3d{
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z},
          Eigen::Vector3d{
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z});
        sliding_window_preintegrator_initialized_ = true;
      }
    } catch (const std::exception & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "failed to propagate IMU tracking state: %s", ex.what());
    }
  }

  void handle_image(const sensor_msgs::msg::Image & msg)
  {
    image_pub_->publish(msg);
    if (!enable_visual_factor_) {
      return;
    }
    gaussian_lic_tracking::VisualFrame observed;
    if (!decode_image_gray(msg, observed)) {
      return;
    }
    if (has_rendered_frame_) {
      last_visual_residual_ = visual_factor_.evaluate(latest_rendered_frame_, observed);
      if (last_visual_residual_.valid) {
        RCLCPP_DEBUG_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "visual residual pixels=%zu mae=%.6f rmse=%.6f",
          last_visual_residual_.compared_pixels,
          last_visual_residual_.mean_abs_error,
          last_visual_residual_.rmse);
      }
    }
  }

  void handle_rendered_image(const sensor_msgs::msg::Image & msg)
  {
    if (!enable_visual_factor_) {
      return;
    }
    gaussian_lic_tracking::VisualFrame rendered;
    if (decode_image_gray(msg, rendered)) {
      latest_rendered_frame_ = std::move(rendered);
      has_rendered_frame_ = true;
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
    tracking_pose.q_w_i = Eigen::Quaterniond::Identity();
    if (imu_propagator_.initialized()) {
      const auto & state = imu_propagator_.state();
      tracking_pose.p_w_i = state.p_w_i;
      tracking_pose.q_w_i = state.q_w_i;
    }

    sensor_msgs::msg::PointCloud2 output_cloud = msg;
    std::vector<Eigen::Vector3d> lidar_points;
    if (enable_lio_factor_ || enable_lidar_deskew_) {
      PointCloudFields fields;
      const auto decoded_points = decode_pointcloud(msg, fields);
      lidar_points.reserve(decoded_points.size());
      for (const auto & point : decoded_points) {
        lidar_points.push_back(point.point_i);
      }
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

    gaussian_lic_tracking::SlidingWindowPointToPointFactor lidar_window_factor;
    bool has_lidar_window_factor = false;
    if (enable_lio_factor_) {
      if (enable_sliding_window_optimizer_) {
        lidar_window_factor = lidar_factor_.build_point_to_point_factor(lidar_points, tracking_pose);
        has_lidar_window_factor = !lidar_window_factor.frame_points_i.empty();
      }
      const auto correction = lidar_factor_.compute_pose_correction(lidar_points, tracking_pose);
      if (correction.applied) {
        tracking_pose.p_w_i += correction.delta_p_w;
        tracking_pose.q_w_i = (correction.delta_q * tracking_pose.q_w_i).normalized();
      }
      if (should_insert_lidar_keyframe(tracking_pose, lidar_points.size())) {
        lidar_factor_.insert_keyframe(lidar_points, tracking_pose);
        last_lidar_keyframe_pose_ = tracking_pose;
        has_lidar_keyframe_ = true;
      }
    }

    if (enable_sliding_window_optimizer_) {
      tracking_pose = update_sliding_window(
        tracking_pose,
        has_lidar_window_factor ? &lidar_window_factor : nullptr);
    }

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
    if (imu_propagator_.initialized()) {
      const auto & state = imu_propagator_.state();
      odom.twist.twist.linear.x = state.v_w_i.x();
      odom.twist.twist.linear.y = state.v_w_i.y();
      odom.twist.twist.linear.z = state.v_w_i.z();
    }
    odometry_pub_->publish(odom);

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
  }

  gaussian_lic_tracking::TrajectoryPose update_sliding_window(
    const gaussian_lic_tracking::TrajectoryPose & input_pose,
    const gaussian_lic_tracking::SlidingWindowPointToPointFactor * lidar_factor)
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
    if (lidar_factor != nullptr) {
      try {
        sliding_window_optimizer_.add_point_to_point_factor(*lidar_factor);
      } catch (const std::exception & ex) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "sliding window LiDAR factor skipped: %s", ex.what());
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
      factor.bias_weight = sliding_window_bias_weight_;
      try {
        sliding_window_optimizer_.add_imu_factor(factor);
        const auto summary = sliding_window_optimizer_.optimize();
        gaussian_lic_tracking::SlidingWindowState optimized;
        if (sliding_window_optimizer_.get_state(input_pose.stamp_ns, optimized)) {
          output_pose.p_w_i = optimized.p_w_i;
          output_pose.q_w_i = optimized.q_w_i;
          sliding_window_bias_.gyro = optimized.gyro_bias;
          sliding_window_bias_.accel = optimized.accel_bias;
        }
        RCLCPP_DEBUG_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "sliding window states=%zu imu_factors=%zu priors=%zu cost %.6g -> %.6g",
          summary.state_count,
          summary.imu_factor_count,
          summary.pose_prior_count,
          summary.initial_cost,
          summary.final_cost);
      } catch (const std::exception & ex) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "sliding window optimization skipped: %s", ex.what());
      }
    }

    has_sliding_window_state_ = true;
    last_sliding_window_stamp_ns_ = input_pose.stamp_ns;
    sliding_window_preintegrator_.reset(input_pose.stamp_ns, sliding_window_bias_);
    sliding_window_preintegrator_initialized_ = true;
    return output_pose;
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
    const gaussian_lic_tracking::TrajectoryPose & reference_pose) const
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
        gaussian_lic_tracking::ImuState state;
        if (!imu_propagator_.query_state(stamp_ns, state)) {
          return false;
        }
        pose.stamp_ns = state.stamp_ns;
        pose.p_w_i = state.p_w_i;
        pose.q_w_i = state.q_w_i;
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
  std::string rendered_image_topic_;
  std::string gaussian_map_topic_;
  std::string world_frame_;
  std::string child_frame_;
  bool publish_tf_{false};
  int max_path_length_{5000};
  int sensor_qos_depth_{5};
  std::string sensor_qos_reliability_{"best_effort"};
  bool enable_visual_factor_{true};
  bool enable_gaussian_snapshot_{true};
  int visual_max_pixels_{200000};
  bool enable_lio_factor_{true};
  bool enable_lidar_deskew_{true};
  bool enable_sliding_window_optimizer_{false};
  std::string lidar_time_field_{"auto"};
  std::string lidar_time_unit_{"auto"};
  std::string lidar_time_mode_{"auto"};
  int imu_history_size_{4000};
  int sliding_window_max_states_{12};
  int sliding_window_max_iterations_{3};
  double sliding_window_imu_weight_{1.0};
  double sliding_window_bias_weight_{1.0};
  double sliding_window_pose_translation_weight_{2.0};
  double sliding_window_pose_rotation_weight_{2.0};
  int lidar_min_points_{32};
  int lidar_max_frame_points_{2000};
  int lidar_max_map_points_{20000};
  double lidar_nearest_distance_m_{0.35};
  double lidar_correction_gain_{0.7};
  double lidar_max_correction_m_{0.25};
  double lidar_max_rotation_rad_{0.08};
  double lidar_keyframe_translation_m_{0.25};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rendered_image_sub_;
  rclcpp::Subscription<gaussian_lic_msgs::msg::GaussianArray>::SharedPtr gaussian_map_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  gaussian_lic_tracking::ImuPropagator imu_propagator_;
  gaussian_lic_tracking::SlidingWindowOptimizer sliding_window_optimizer_;
  gaussian_lic_tracking::ImuPreintegrator sliding_window_preintegrator_;
  gaussian_lic_tracking::ImuBias sliding_window_bias_;
  bool sliding_window_preintegrator_initialized_{false};
  bool has_sliding_window_state_{false};
  int64_t last_sliding_window_stamp_ns_{0};
  gaussian_lic_tracking::LidarFactor lidar_factor_;
  gaussian_lic_tracking::TrajectoryPose last_lidar_keyframe_pose_;
  bool has_lidar_keyframe_{false};
  gaussian_lic_tracking::GaussianSnapshot gaussian_snapshot_;
  gaussian_lic_tracking::VisualFactor visual_factor_;
  gaussian_lic_tracking::VisualFrame latest_rendered_frame_;
  gaussian_lic_tracking::VisualResidual last_visual_residual_;
  bool has_rendered_frame_{false};
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
