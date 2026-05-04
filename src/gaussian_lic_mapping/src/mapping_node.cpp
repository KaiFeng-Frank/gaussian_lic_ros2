// SPDX-License-Identifier: GPL-3.0-or-later

#include <chrono>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <deque>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include <builtin_interfaces/msg/time.hpp>
#include <gaussian_lic_msgs/msg/gaussian.hpp>
#include <gaussian_lic_msgs/msg/gaussian_array.hpp>
#include <gaussian_lic_msgs/msg/mapping_status.hpp>
#include <gaussian_lic_msgs/srv/save_map.hpp>
#include <gaussian_lic_mapping/backend_config.hpp>
#include <gaussian_lic_mapping/frame_data.hpp>
#include <gaussian_lic_mapping/mapper_dataset.hpp>
#ifdef GAUSSIAN_LIC_ENABLE_TORCH
#include <gaussian_lic_mapping/torch_backend.hpp>
#endif
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#ifdef GAUSSIAN_LIC_MAPPING_COMPOSITION
#include <rclcpp_components/register_node_macro.hpp>
#endif
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <tf2_ros/transform_broadcaster.h>

using namespace std::chrono_literals;
using gaussian_lic_mapping::AlignedRosFrame;
using gaussian_lic_mapping::MapperDataset;
using gaussian_lic_mapping::MapperFrameData;

class MappingNode final : public rclcpp::Node
{
public:
  explicit MappingNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("mapping_node", options)
  {
    image_topic_ = declare_parameter<std::string>("image_topic", "/image_for_gs");
    camera_info_topic_ = declare_parameter<std::string>("camera_info_topic", "/camera_info_for_gs");
    depth_topic_ = declare_parameter<std::string>("depth_topic", "/depth_for_gs");
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/pose_for_gs");
    pointcloud_topic_ = declare_parameter<std::string>("pointcloud_topic", "/points_for_gs");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu_for_gs");
    status_topic_ = declare_parameter<std::string>("status_topic", "/gaussian_lic/status");
    odometry_topic_ = declare_parameter<std::string>("odometry_topic", "/gaussian_lic/odometry");
    path_topic_ = declare_parameter<std::string>("path_topic", "/gaussian_lic/path");
    map_points_topic_ = declare_parameter<std::string>("map_points_topic", "/gaussian_lic/map_points");
    rendered_image_topic_ = declare_parameter<std::string>("rendered_image_topic", "/gaussian_lic/rendered_image");
    gaussian_map_topic_ = declare_parameter<std::string>("gaussian_map_topic", "/gaussian_lic/gaussian_map");
    save_map_service_ = declare_parameter<std::string>("save_map_service", "/gaussian_lic/save_map");
    world_frame_ = declare_parameter<std::string>("world_frame", "map");
    camera_frame_ = declare_parameter<std::string>("camera_frame", "camera");
    publish_tf_ = declare_parameter<bool>("publish_tf", false);
    sync_tolerance_sec_ = declare_parameter<double>("sync_tolerance_sec", 0.01);
    max_queue_size_ = declare_parameter<int>("max_queue_size", 10000);
    sensor_qos_reliability_ = declare_parameter<std::string>("sensor_qos_reliability", "best_effort");
    sensor_qos_history_ = declare_parameter<std::string>("sensor_qos_history", "keep_last");
    sensor_qos_depth_ = declare_parameter<int>("sensor_qos_depth", 5);
    process_period_ms_ = declare_parameter<int>("process_period_ms", 5);
    select_every_k_frame_ = declare_parameter<int>("select_every_k_frame", 8);
    require_depth_topic_ = declare_parameter<bool>("require_depth_topic", true);
    fx_ = declare_parameter<double>("fx", 1.0);
    fy_ = declare_parameter<double>("fy", 1.0);
    cx_ = declare_parameter<double>("cx", 0.5);
    cy_ = declare_parameter<double>("cy", 0.5);
    backend_config_.width = declare_parameter<int>("width", 0);
    backend_config_.height = declare_parameter<int>("height", 0);
    backend_config_.depth_completion = declare_parameter<bool>("depth_completion", false);
    backend_config_.patch_size = declare_parameter<int>("patch_size", 10);
    backend_config_.max_depth = declare_parameter<double>("max_depth", 20.0);
#ifdef GAUSSIAN_LIC_ENABLE_TORCH
    enable_torch_camera_conversion_ = declare_parameter<bool>("enable_torch_camera_conversion", false);
    enable_torch_gaussian_init_ = declare_parameter<bool>("enable_torch_gaussian_init", false);
    enable_torch_gaussian_extend_ = declare_parameter<bool>("enable_torch_gaussian_extend", true);
#else
    declare_parameter<bool>("enable_torch_camera_conversion", false);
    declare_parameter<bool>("enable_torch_gaussian_init", false);
    declare_parameter<bool>("enable_torch_gaussian_extend", true);
#endif
    gaussian_init_min_points_ = declare_parameter<int>("gaussian_init_min_points", 1);
    gaussian_init_min_keyframes_ = declare_parameter<int>("gaussian_init_min_keyframes", 1);
    backend_config_.sh_degree = declare_parameter<int>("sh_degree", 3);
    backend_config_.white_background = declare_parameter<bool>("white_background", false);
    backend_config_.random_background = declare_parameter<bool>("random_background", false);
    backend_config_.convert_shs_python = declare_parameter<bool>("convert_SHs_python", false);
    backend_config_.compute_cov3d_python = declare_parameter<bool>("compute_cov3D_python", false);
    backend_config_.lambda_erank = declare_parameter<double>("lambda_erank", 0.0);
    backend_config_.scaling_scale = declare_parameter<double>("scaling_scale", 1.0);
    backend_config_.position_lr = declare_parameter<double>("position_lr", 0.00016);
    backend_config_.feature_lr = declare_parameter<double>("feature_lr", 0.005);
    backend_config_.opacity_lr = declare_parameter<double>("opacity_lr", 0.05);
    backend_config_.scaling_lr = declare_parameter<double>("scaling_lr", 0.005);
    backend_config_.rotation_lr = declare_parameter<double>("rotation_lr", 0.001);
    backend_config_.lambda_dssim = declare_parameter<double>("lambda_dssim", 0.2);
    backend_config_.optimize_depth = declare_parameter<bool>("optimize_depth", true);
    backend_config_.lambda_depth = declare_parameter<double>("lambda_depth", 0.005);
    backend_config_.iteration_decay = declare_parameter<bool>("iteration_decay", false);
    backend_config_.apply_exposure = declare_parameter<bool>("apply_exposure", false);
    backend_config_.exposure_lr = declare_parameter<double>("exposure_lr", 0.001);
    backend_config_.skybox_points_num = declare_parameter<int>("skybox_points_num", 0);
    backend_config_.skybox_radius = declare_parameter<double>("skybox_radius", 1000.0);
    torch_gaussian_device_name_ = declare_parameter<std::string>("torch_gaussian_device", "cpu");
    gaussian_map_chunk_size_ = declare_parameter<int>("gaussian_map_chunk_size", 1024);
    max_path_length_ = declare_parameter<int>("max_path_length", 5000);
    max_map_points_ = declare_parameter<int>("max_map_points", 200000);
    publish_rendered_preview_ = declare_parameter<bool>("publish_rendered_preview", true);
    active_profile_ = declare_parameter<std::string>("active_profile", "default");
    render_mode_ = declare_parameter<std::string>("render_mode", "debug_cpu");
    rendered_image_mode_ = declare_parameter<std::string>("rendered_image_mode", "");
    if (!rendered_image_mode_.empty()) {
      render_mode_ = legacy_render_mode_to_render_mode(rendered_image_mode_);
      RCLCPP_WARN(
        get_logger(),
        "rendered_image_mode is deprecated; use render_mode:=debug_cpu, rasterizer, debug_input, or off");
    }

    auto sensor_qos = make_sensor_qos();
    points_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic_, sensor_qos,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        ++pointcloud_count_;
        {
          std::scoped_lock lock(buffer_mutex_);
          push_bounded(point_buf_, msg, dropped_pointcloud_count_);
        }
      });

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic_, sensor_qos,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
        ++pose_count_;
        {
          std::scoped_lock lock(buffer_mutex_);
          push_bounded(pose_buf_, msg, dropped_pose_count_);
        }
      });

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, sensor_qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
        ++image_count_;
        {
          std::scoped_lock lock(buffer_mutex_);
          push_bounded(image_buf_, msg, dropped_image_count_);
        }
      });

    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, sensor_qos,
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
        ++camera_info_count_;
        if (msg->k[0] <= 0.0 || msg->k[4] <= 0.0) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "ignoring CameraInfo with non-positive focal length");
          return;
        }

        std::scoped_lock lock(intrinsics_mutex_);
        fx_ = msg->k[0];
        fy_ = msg->k[4];
        cx_ = msg->k[2];
        cy_ = msg->k[5];
        intrinsics_source_ = "camera_info";
        camera_info_received_ = true;
      });

    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      depth_topic_, sensor_qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
        ++depth_count_;
        {
          std::scoped_lock lock(buffer_mutex_);
          push_bounded(depth_buf_, msg, dropped_depth_count_);
        }
      });

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, sensor_qos,
      [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
        ++imu_count_;
        last_imu_stamp_ = msg->header.stamp;
      });

    status_pub_ = create_publisher<gaussian_lic_msgs::msg::MappingStatus>(status_topic_, 10);
    odometry_pub_ = create_publisher<nav_msgs::msg::Odometry>(odometry_topic_, 20);
    path_pub_ = create_publisher<nav_msgs::msg::Path>(
      path_topic_, rclcpp::QoS(1).transient_local().reliable());
    map_points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(map_points_topic_, 10);
    rendered_image_pub_ = create_publisher<sensor_msgs::msg::Image>(
      rendered_image_topic_, rclcpp::QoS(1).transient_local().reliable());
    gaussian_map_pub_ = create_publisher<gaussian_lic_msgs::msg::GaussianArray>(
      gaussian_map_topic_, rclcpp::QoS(1).transient_local().reliable());
    save_map_srv_ = create_service<gaussian_lic_msgs::srv::SaveMap>(
      save_map_service_,
      [this](
        const std::shared_ptr<gaussian_lic_msgs::srv::SaveMap::Request> request,
        std::shared_ptr<gaussian_lic_msgs::srv::SaveMap::Response> response) {
        handle_save_map(request, response);
      });
    status_timer_ = create_wall_timer(1s, [this]() { publish_status(); });
    process_timer_ = create_wall_timer(
      std::chrono::milliseconds(process_period_ms_),
      [this]() { process_available_frames(); });
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    RCLCPP_INFO(get_logger(), "Native mapping scaffold started");
    RCLCPP_INFO(get_logger(), "Subscribing to %s, %s, %s, %s, %s",
      pointcloud_topic_.c_str(), pose_topic_.c_str(), image_topic_.c_str(), depth_topic_.c_str(),
      imu_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Subscribing to camera intrinsics on %s",
      camera_info_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Publishing Gaussian map chunks to %s",
      gaussian_map_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Publishing odometry/path/map points to %s, %s, %s",
      odometry_topic_.c_str(), path_topic_.c_str(), map_points_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Publishing rendered image preview to %s (%s, render_mode=%s)",
      rendered_image_topic_.c_str(), publish_rendered_preview_ ? "enabled" : "disabled",
      render_mode_.c_str());
    RCLCPP_INFO(get_logger(), "TF publishing %s (%s -> %s)",
      publish_tf_ ? "enabled" : "disabled", world_frame_.c_str(), camera_frame_.c_str());
    RCLCPP_INFO(get_logger(), "Save-map service available at %s",
      save_map_service_.c_str());
    RCLCPP_INFO(get_logger(), "Frame sync tolerance %.3f sec, max queue %d",
      sync_tolerance_sec_, max_queue_size_);
    RCLCPP_INFO(get_logger(), "Depth topic synchronization %s",
      require_depth_topic_ ? "required" : "optional; projected point depth fallback enabled");
    RCLCPP_INFO(get_logger(), "Sensor QoS reliability=%s history=%s depth=%d",
      sensor_qos_reliability_.c_str(), sensor_qos_history_.c_str(), sensor_qos_depth_);
#ifdef GAUSSIAN_LIC_ENABLE_TORCH
    RCLCPP_INFO(get_logger(), "Torch camera conversion %s",
      enable_torch_camera_conversion_ ? "enabled" : "disabled");
    RCLCPP_INFO(get_logger(), "Torch Gaussian initialization %s, min points %d, device %s",
      enable_torch_gaussian_init_ ? "enabled" : "disabled",
      gaussian_init_min_points_, torch_gaussian_device_name_.c_str());
    RCLCPP_INFO(get_logger(), "Torch Gaussian extension %s, min keyframes %d, skybox points %d",
      enable_torch_gaussian_extend_ ? "enabled" : "disabled",
      gaussian_init_min_keyframes_, backend_config_.skybox_points_num);
#else
    RCLCPP_INFO(get_logger(), "Torch camera conversion unavailable in this build");
    RCLCPP_INFO(get_logger(), "Torch Gaussian initialization unavailable in this build");
#endif
    RCLCPP_INFO(
      get_logger(),
      "Gaussian backend profile: size=%dx%d sh=%d scaling=%.3f depth_completion=%s "
      "patch=%d max_depth=%.2f optimize_depth=%s lr(pos=%.6f feat=%.6f opacity=%.6f scale=%.6f rot=%.6f)",
      backend_config_.width, backend_config_.height, backend_config_.sh_degree,
      backend_config_.scaling_scale, backend_config_.depth_completion ? "true" : "false",
      backend_config_.patch_size, backend_config_.max_depth,
      backend_config_.optimize_depth ? "true" : "false",
      backend_config_.position_lr, backend_config_.feature_lr, backend_config_.opacity_lr,
      backend_config_.scaling_lr, backend_config_.rotation_lr);
    if (backend_config_.depth_completion) {
      RCLCPP_WARN(
        get_logger(),
        "depth_completion is configured but the TensorRT/SPNet backend is not ported in this ROS2 "
        "slice; using the provided depth topic only");
    }
  }

private:
  struct Intrinsics
  {
    double fx{1.0};
    double fy{1.0};
    double cx{0.5};
    double cy{0.5};
    std::string source{"params"};
  };

  static std::string normalized_qos_token(std::string value)
  {
    std::transform(
      value.begin(), value.end(), value.begin(),
      [](const unsigned char c) { return static_cast<char>(std::tolower(c)); });
    value.erase(
      std::remove_if(
        value.begin(), value.end(),
        [](const char c) { return c == '-' || c == ' ' || c == '_'; }),
      value.end());
    return value;
  }

  static std::string legacy_render_mode_to_render_mode(const std::string & value)
  {
    const std::string mode = normalized_qos_token(value);
    if (mode == "projectedmap" || mode == "auto") {
      return "debug_cpu";
    }
    if (mode == "input") {
      return "debug_input";
    }
    if (mode == "debugcpu" || mode == "debuginput" || mode == "rasterizer" || mode == "off") {
      return value;
    }
    return value;
  }

  uint8_t render_mode_status_value() const
  {
    const std::string mode = normalized_qos_token(render_mode_);
    if (mode == "off") {
      return gaussian_lic_msgs::msg::MappingStatus::RENDER_MODE_OFF;
    }
    if (mode == "debuginput") {
      return gaussian_lic_msgs::msg::MappingStatus::RENDER_MODE_DEBUG_INPUT;
    }
    if (mode == "rasterizer") {
      return gaussian_lic_msgs::msg::MappingStatus::RENDER_MODE_RASTERIZER;
    }
    return gaussian_lic_msgs::msg::MappingStatus::RENDER_MODE_DEBUG_CPU;
  }

  rclcpp::QoS make_sensor_qos() const
  {
    const auto depth = static_cast<size_t>(std::max(sensor_qos_depth_, 1));
    rclcpp::QoS qos{rclcpp::KeepLast(depth)};
    qos.durability_volatile();

    const std::string reliability = normalized_qos_token(sensor_qos_reliability_);
    if (reliability == "besteffort") {
      qos.best_effort();
    } else if (reliability == "reliable") {
      qos.reliable();
    } else {
      throw std::runtime_error(
        "sensor_qos_reliability must be best_effort or reliable, got " + sensor_qos_reliability_);
    }

    const std::string history = normalized_qos_token(sensor_qos_history_);
    if (history == "keeplast") {
      qos.keep_last(depth);
    } else if (history == "keepall") {
      qos.keep_all();
    } else {
      throw std::runtime_error(
        "sensor_qos_history must be keep_last or keep_all, got " + sensor_qos_history_);
    }

    return qos;
  }

  template<typename PtrT>
  void push_bounded(
    std::deque<PtrT> & queue,
    const PtrT & msg,
    uint64_t & dropped_count)
  {
    queue.push_back(msg);
    while (static_cast<int>(queue.size()) > max_queue_size_) {
      queue.pop_front();
      ++dropped_count;
    }
  }

  static double stamp_to_sec(const builtin_interfaces::msg::Time & stamp)
  {
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
  }

  static uint8_t color_channel_to_u8(const float value)
  {
    const float clamped = std::clamp(value, 0.0F, 1.0F);
    return static_cast<uint8_t>(std::lround(clamped * 255.0F));
  }

  static float sigmoid(const float value)
  {
    return 1.0F / (1.0F + std::exp(-value));
  }

  static float sh_dc_to_rgb(const float value)
  {
    constexpr float c0 = 0.28209479177387814F;
    return std::clamp(0.5F + c0 * value, 0.0F, 1.0F);
  }

  static sensor_msgs::msg::PointField make_point_field(
    const std::string & name,
    const uint32_t offset,
    const uint8_t datatype)
  {
    sensor_msgs::msg::PointField field;
    field.name = name;
    field.offset = offset;
    field.datatype = datatype;
    field.count = 1;
    return field;
  }

  Intrinsics current_intrinsics() const
  {
    std::scoped_lock lock(intrinsics_mutex_);
    return Intrinsics{fx_, fy_, cx_, cy_, intrinsics_source_};
  }

  template<typename PtrT>
  bool trim_until_near(
    std::deque<PtrT> & queue,
    const double frame_time,
    uint64_t & dropped_count)
  {
    while (!queue.empty() && stamp_to_sec(queue.front()->header.stamp) < frame_time - sync_tolerance_sec_) {
      queue.pop_front();
      ++dropped_count;
    }
    return !queue.empty();
  }

  enum class AlignResult
  {
    kNoData,
    kDropped,
    kAligned
  };

  AlignResult pop_aligned_locked(AlignedRosFrame & out)
  {
    if (point_buf_.empty() || pose_buf_.empty() || image_buf_.empty() ||
      (require_depth_topic_ && depth_buf_.empty()))
    {
      return AlignResult::kNoData;
    }

    const double frame_time = stamp_to_sec(point_buf_.front()->header.stamp);

    if (!trim_until_near(pose_buf_, frame_time, dropped_pose_count_) ||
      !trim_until_near(image_buf_, frame_time, dropped_image_count_))
    {
      return AlignResult::kNoData;
    }

    sensor_msgs::msg::Image::ConstSharedPtr aligned_depth;
    if (!depth_buf_.empty()) {
      (void)trim_until_near(depth_buf_, frame_time, dropped_depth_count_);
    }
    if (!depth_buf_.empty()) {
      const bool depth_too_new =
        stamp_to_sec(depth_buf_.front()->header.stamp) > frame_time + sync_tolerance_sec_;
      if (!depth_too_new) {
        aligned_depth = depth_buf_.front();
      } else if (require_depth_topic_) {
        point_buf_.pop_front();
        ++dropped_pointcloud_count_;
        return AlignResult::kDropped;
      }
    } else if (require_depth_topic_) {
      return AlignResult::kNoData;
    }

    const bool pose_too_new =
      stamp_to_sec(pose_buf_.front()->header.stamp) > frame_time + sync_tolerance_sec_;
    const bool image_too_new =
      stamp_to_sec(image_buf_.front()->header.stamp) > frame_time + sync_tolerance_sec_;

    if (pose_too_new || image_too_new) {
      point_buf_.pop_front();
      ++dropped_pointcloud_count_;
      return AlignResult::kDropped;
    }

    last_aligned_stamp_ = point_buf_.front()->header.stamp;
    out.pointcloud = point_buf_.front();
    out.pose = pose_buf_.front();
    out.image = image_buf_.front();
    out.depth = aligned_depth;
    point_buf_.pop_front();
    pose_buf_.pop_front();
    image_buf_.pop_front();
    if (aligned_depth) {
      depth_buf_.pop_front();
    }
    ++aligned_frame_count_;
    return AlignResult::kAligned;
  }

  void process_available_frames()
  {
    int processed = 0;
    while (processed < 32) {
      AlignedRosFrame frame;
      AlignResult result = AlignResult::kNoData;
      {
        std::scoped_lock lock(buffer_mutex_);
        result = pop_aligned_locked(frame);
      }

      if (result == AlignResult::kNoData) {
        break;
      }
      if (result == AlignResult::kDropped) {
        ++processed;
        continue;
      }

      try {
        const auto iteration_start = std::chrono::steady_clock::now();
        const auto intrinsics = current_intrinsics();
        const gaussian_lic_mapping::CameraIntrinsics frame_intrinsics{
          intrinsics.fx, intrinsics.fy, intrinsics.cx, intrinsics.cy};
        MapperFrameData frame_data = gaussian_lic_mapping::convert_aligned_frame(
          frame, dataset_.all_frame_count(), select_every_k_frame_, frame_intrinsics);
        record_converted_frame(std::move(frame_data));
        record_iteration_timing(iteration_start);
      } catch (const std::exception & ex) {
        ++conversion_error_count_;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "failed to convert aligned frame: %s", ex.what());
      }

      ++processed;
    }
  }

  void record_iteration_timing(const std::chrono::steady_clock::time_point & iteration_start)
  {
    const std::chrono::duration<double, std::milli> elapsed =
      std::chrono::steady_clock::now() - iteration_start;
    last_mapping_latency_ms_ = static_cast<float>(elapsed.count());
    cumulative_iteration_ms_ += elapsed.count();
    ++iteration_timing_count_;
    mean_iteration_ms_ = static_cast<float>(
      cumulative_iteration_ms_ / static_cast<double>(iteration_timing_count_));
  }

  void record_converted_frame(MapperFrameData && frame_data)
  {
    ++converted_frame_count_;
    last_image_width_ = frame_data.width;
    last_image_height_ = frame_data.height;
    last_points_in_frame_ = frame_data.points.size();
    last_image_rgb_float_ = frame_data.image_rgb_float.clone();
    last_q_wc_ = frame_data.q_wc;
    last_t_wc_ = frame_data.t_wc;
    const auto frame_stamp = frame_data.stamp;
    publish_tracking_outputs(frame_data);
    const auto & record = dataset_.add_frame(std::move(frame_data));
    dataset_.trim_map_points(max_map_points_ > 0 ? static_cast<size_t>(max_map_points_) : 0U);
    map_points_pub_->publish(make_map_points_message(frame_stamp));
    (void)record;
    if (publish_rendered_preview_ && normalized_qos_token(render_mode_) != "off") {
      try {
        rendered_image_pub_->publish(make_rendered_preview_message(frame_stamp));
      } catch (const std::exception & ex) {
        ++render_error_count_;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "failed to publish rendered preview: %s", ex.what());
      }
    }
#ifdef GAUSSIAN_LIC_ENABLE_TORCH
    const auto intrinsics = current_intrinsics();
    if (enable_torch_camera_conversion_) {
      try {
        const auto camera = gaussian_lic_mapping::make_torch_camera(
          record, intrinsics.fx, intrinsics.fy, intrinsics.cx, intrinsics.cy, torch::kCPU, torch::kCPU);
        ++torch_camera_count_;
        std::ostringstream image_dims;
        std::ostringstream depth_dims;
        image_dims << camera.original_image.sizes();
        depth_dims << camera.original_depth.sizes();
        last_torch_image_dims_ = image_dims.str();
        last_torch_depth_dims_ = depth_dims.str();
      } catch (const std::exception & ex) {
        ++torch_camera_error_count_;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "failed to create TorchCamera: %s", ex.what());
      }
    }
    maybe_update_torch_gaussians(record.is_keyframe);
#endif
  }

#ifdef GAUSSIAN_LIC_ENABLE_TORCH
  torch::Device resolve_torch_gaussian_device() const
  {
    std::string device_name = torch_gaussian_device_name_;
    if (device_name.empty() || device_name == "auto") {
      device_name = torch::cuda::is_available() ? "cuda" : "cpu";
    }

    torch::Device device(device_name);
    if (device.is_cuda() && !torch::cuda::is_available()) {
      throw std::runtime_error("CUDA torch device requested but CUDA is not available");
    }
    return device;
  }

  void refresh_torch_gaussian_status(const torch::Device & device)
  {
    torch_gaussian_count_ = torch_gaussian_map_.foreground_count + torch_gaussian_map_.skybox_count;
    torch_gaussian_device_label_ = device.str();

    std::ostringstream xyz_dims;
    std::ostringstream feature_dims;
    xyz_dims << torch_gaussian_map_.xyz.sizes();
    feature_dims << torch_gaussian_map_.features_dc.sizes();
    last_torch_gaussian_xyz_dims_ = xyz_dims.str();
    last_torch_gaussian_feature_dims_ = feature_dims.str();
  }

  void maybe_update_torch_gaussians(const bool is_keyframe)
  {
    if (!enable_torch_gaussian_init_ || !is_keyframe) {
      return;
    }
    if (dataset_.pending_point_count() < static_cast<size_t>(std::max(gaussian_init_min_points_, 1))) {
      return;
    }
    if (dataset_.train_frame_count() < static_cast<size_t>(std::max(gaussian_init_min_keyframes_, 1))) {
      return;
    }

    try {
      const auto intrinsics = current_intrinsics();
      const auto device = resolve_torch_gaussian_device();
      if (!torch_gaussian_initialized_) {
        torch_gaussian_map_ = gaussian_lic_mapping::initialize_gaussian_map(
          dataset_, backend_config_, intrinsics.fx, intrinsics.fy, device);
        torch_gaussian_initialized_ = true;
        last_torch_gaussian_inserted_ = torch_gaussian_map_.foreground_count;
        ++torch_gaussian_init_count_;
        refresh_torch_gaussian_status(device);

        dataset_.clear_pending_points();
        publish_torch_gaussian_map();
        RCLCPP_INFO(
          get_logger(),
          "Initialized Torch Gaussian map: foreground=%zu skybox=%zu xyz=%s features_dc=%s device=%s",
          torch_gaussian_map_.foreground_count, torch_gaussian_map_.skybox_count,
          last_torch_gaussian_xyz_dims_.c_str(), last_torch_gaussian_feature_dims_.c_str(),
          torch_gaussian_device_label_.c_str());
        return;
      }

      if (!enable_torch_gaussian_extend_) {
        dataset_.clear_pending_points();
        return;
      }

      last_torch_gaussian_inserted_ = gaussian_lic_mapping::append_pending_points_to_gaussian_map(
        torch_gaussian_map_, dataset_, backend_config_, intrinsics.fx, intrinsics.fy, device);
      ++torch_gaussian_extend_count_;
      refresh_torch_gaussian_status(device);
      dataset_.clear_pending_points();
      publish_torch_gaussian_map();
      RCLCPP_INFO(
        get_logger(),
        "Extended Torch Gaussian map: inserted=%zu foreground=%zu skybox=%zu xyz=%s device=%s",
        last_torch_gaussian_inserted_, torch_gaussian_map_.foreground_count,
        torch_gaussian_map_.skybox_count, last_torch_gaussian_xyz_dims_.c_str(),
        torch_gaussian_device_label_.c_str());
    } catch (const std::exception & ex) {
      if (torch_gaussian_initialized_) {
        ++torch_gaussian_extend_error_count_;
      } else {
        ++torch_gaussian_init_error_count_;
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "failed to update Torch Gaussian map: %s", ex.what());
    }
  }

  void publish_torch_gaussian_map()
  {
    if (!torch_gaussian_initialized_ || torch_gaussian_count_ == 0) {
      return;
    }

    torch::NoGradGuard no_grad;
    const auto xyz = torch_gaussian_map_.xyz.detach().to(torch::kCPU).contiguous();
    const auto features_dc = torch_gaussian_map_.features_dc.detach().to(torch::kCPU).contiguous();
    const auto features_rest = torch_gaussian_map_.features_rest.detach().to(torch::kCPU).contiguous();
    const auto scaling = torch_gaussian_map_.scaling.detach().to(torch::kCPU).contiguous();
    const auto rotation = torch_gaussian_map_.rotation.detach().to(torch::kCPU).contiguous();
    const auto opacity = torch_gaussian_map_.opacity.detach().to(torch::kCPU).contiguous();

    const int64_t point_count = xyz.size(0);
    const size_t total_count = static_cast<size_t>(point_count);
    const size_t chunk_size = static_cast<size_t>(std::max(gaussian_map_chunk_size_, 1));
    const size_t chunk_count = (total_count + chunk_size - 1U) / chunk_size;

    const auto xyz_a = xyz.accessor<float, 2>();
    const auto dc_a = features_dc.accessor<float, 3>();
    const auto rest_a = features_rest.accessor<float, 3>();
    const auto scaling_a = scaling.accessor<float, 2>();
    const auto rotation_a = rotation.accessor<float, 2>();
    const auto opacity_a = opacity.accessor<float, 2>();

    const uint32_t total_count_msg = static_cast<uint32_t>(
      std::min<size_t>(total_count, std::numeric_limits<uint32_t>::max()));
    const uint32_t chunk_count_msg = static_cast<uint32_t>(
      std::min<size_t>(chunk_count, std::numeric_limits<uint32_t>::max()));
    for (size_t chunk_index = 0; chunk_index < chunk_count; ++chunk_index) {
      const size_t begin = chunk_index * chunk_size;
      const size_t end = std::min(begin + chunk_size, total_count);

      gaussian_lic_msgs::msg::GaussianArray message;
      message.header.stamp = now();
      message.header.frame_id = world_frame_;
      message.total_count = total_count_msg;
      message.chunk_index = static_cast<uint32_t>(
        std::min<size_t>(chunk_index, std::numeric_limits<uint32_t>::max()));
      message.chunk_count = chunk_count_msg;
      message.gaussians.reserve(end - begin);

      for (size_t i = begin; i < end; ++i) {
        gaussian_lic_msgs::msg::Gaussian gaussian;
        gaussian.id = static_cast<uint32_t>(std::min<size_t>(i, std::numeric_limits<uint32_t>::max()));
        gaussian.xyz[0] = xyz_a[i][0];
        gaussian.xyz[1] = xyz_a[i][1];
        gaussian.xyz[2] = xyz_a[i][2];

        gaussian.rotation_xyzw[0] = rotation_a[i][1];
        gaussian.rotation_xyzw[1] = rotation_a[i][2];
        gaussian.rotation_xyzw[2] = rotation_a[i][3];
        gaussian.rotation_xyzw[3] = rotation_a[i][0];

        gaussian.scale[0] = std::exp(scaling_a[i][0]);
        gaussian.scale[1] = std::exp(scaling_a[i][1]);
        gaussian.scale[2] = std::exp(scaling_a[i][2]);
        gaussian.opacity = 1.0F / (1.0F + std::exp(-opacity_a[i][0]));
        gaussian.confidence = 1.0F;
        gaussian.flags = 0;

        const int64_t rest_coeff_count = features_rest.size(1);
        gaussian.spherical_harmonics.reserve(static_cast<size_t>(1 + rest_coeff_count) * 3U);
        gaussian.spherical_harmonics.push_back(dc_a[i][0][0]);
        gaussian.spherical_harmonics.push_back(dc_a[i][0][1]);
        gaussian.spherical_harmonics.push_back(dc_a[i][0][2]);
        for (int64_t coeff = 0; coeff < rest_coeff_count; ++coeff) {
          gaussian.spherical_harmonics.push_back(rest_a[i][coeff][0]);
          gaussian.spherical_harmonics.push_back(rest_a[i][coeff][1]);
          gaussian.spherical_harmonics.push_back(rest_a[i][coeff][2]);
        }
        message.gaussians.push_back(std::move(gaussian));
      }

      gaussian_map_pub_->publish(message);
    }
  }

  void write_torch_gaussian_ply(
    const std::filesystem::path & output_path,
    const bool include_skybox) const
  {
    if (!torch_gaussian_initialized_ || torch_gaussian_count_ == 0) {
      throw std::runtime_error("no initialized Gaussian map is available");
    }

    torch::NoGradGuard no_grad;
    const int64_t start_index = include_skybox ?
      0 : static_cast<int64_t>(torch_gaussian_map_.skybox_count);
    const auto slice = torch::indexing::Slice(start_index, torch::indexing::None);
    const auto xyz = torch_gaussian_map_.xyz.index({slice}).detach().to(torch::kCPU).contiguous();
    const auto features_dc = torch_gaussian_map_.features_dc.index({slice})
      .detach().transpose(1, 2).flatten(1).contiguous().to(torch::kCPU);
    const auto features_rest = torch_gaussian_map_.features_rest.index({slice})
      .detach().transpose(1, 2).flatten(1).contiguous().to(torch::kCPU);
    const auto opacity = torch_gaussian_map_.opacity.index({slice}).detach().to(torch::kCPU).contiguous();
    const auto scaling = torch_gaussian_map_.scaling.index({slice}).detach().to(torch::kCPU).contiguous();
    const auto rotation = torch_gaussian_map_.rotation.index({slice}).detach().to(torch::kCPU).contiguous();

    const int64_t point_count = xyz.size(0);
    if (point_count == 0) {
      throw std::runtime_error("Gaussian map has no foreground points to save");
    }

    std::ofstream out(output_path);
    if (!out.is_open()) {
      throw std::runtime_error("failed to open " + output_path.string());
    }

    out << "ply\n";
    out << "format ascii 1.0\n";
    out << "element vertex " << point_count << "\n";
    out << "property float x\n";
    out << "property float y\n";
    out << "property float z\n";
    for (int64_t i = 0; i < features_dc.size(1); ++i) {
      out << "property float f_dc_" << i << "\n";
    }
    for (int64_t i = 0; i < features_rest.size(1); ++i) {
      out << "property float f_rest_" << i << "\n";
    }
    out << "property float opacity\n";
    for (int64_t i = 0; i < scaling.size(1); ++i) {
      out << "property float scale_" << i << "\n";
    }
    for (int64_t i = 0; i < rotation.size(1); ++i) {
      out << "property float rot_" << i << "\n";
    }
    out << "end_header\n";
    out << std::setprecision(9);

    const auto xyz_a = xyz.accessor<float, 2>();
    const auto dc_a = features_dc.accessor<float, 2>();
    const auto rest_a = features_rest.accessor<float, 2>();
    const auto opacity_a = opacity.accessor<float, 2>();
    const auto scaling_a = scaling.accessor<float, 2>();
    const auto rotation_a = rotation.accessor<float, 2>();

    for (int64_t row = 0; row < point_count; ++row) {
      out << xyz_a[row][0] << " " << xyz_a[row][1] << " " << xyz_a[row][2];
      for (int64_t i = 0; i < features_dc.size(1); ++i) {
        out << " " << dc_a[row][i];
      }
      for (int64_t i = 0; i < features_rest.size(1); ++i) {
        out << " " << rest_a[row][i];
      }
      out << " " << opacity_a[row][0];
      for (int64_t i = 0; i < scaling.size(1); ++i) {
        out << " " << scaling_a[row][i];
      }
      for (int64_t i = 0; i < rotation.size(1); ++i) {
        out << " " << rotation_a[row][i];
      }
      out << "\n";
    }
  }
#endif

  std::filesystem::path resolve_debug_map_path(const std::string & request_path) const
  {
    if (request_path.empty()) {
      throw std::runtime_error("save path is empty");
    }

    std::filesystem::path output_path(request_path);
    if (output_path.extension() == ".ply") {
      if (output_path.has_parent_path()) {
        std::filesystem::create_directories(output_path.parent_path());
      }
      return output_path;
    }

    std::filesystem::create_directories(output_path);
    return output_path / "point_cloud.ply";
  }

  void write_debug_map_points_ply(const std::filesystem::path & output_path) const
  {
    const auto & points = dataset_.map_points_world();
    const auto & colors = dataset_.map_colors_rgb();
    const size_t point_count = std::min(points.size(), colors.size());
    if (point_count == 0) {
      throw std::runtime_error("no accumulated debug map points are available");
    }

    std::ofstream out(output_path);
    if (!out.is_open()) {
      throw std::runtime_error("failed to open " + output_path.string());
    }

    out << "ply\n";
    out << "format ascii 1.0\n";
    out << "element vertex " << point_count << "\n";
    out << "property float x\n";
    out << "property float y\n";
    out << "property float z\n";
    out << "property uchar red\n";
    out << "property uchar green\n";
    out << "property uchar blue\n";
    out << "end_header\n";
    out << std::setprecision(9);

    for (size_t i = 0; i < point_count; ++i) {
      const auto & point = points[i];
      const auto & color = colors[i];
      out << point.x() << " " << point.y() << " " << point.z() << " "
          << static_cast<int>(color_channel_to_u8(color.x())) << " "
          << static_cast<int>(color_channel_to_u8(color.y())) << " "
          << static_cast<int>(color_channel_to_u8(color.z())) << "\n";
    }
  }

  void handle_save_map(
    const std::shared_ptr<gaussian_lic_msgs::srv::SaveMap::Request> request,
    std::shared_ptr<gaussian_lic_msgs::srv::SaveMap::Response> response)
  {
#ifdef GAUSSIAN_LIC_ENABLE_TORCH
    try {
      const auto output_path = resolve_debug_map_path(request->path);
      if (torch_gaussian_initialized_ && torch_gaussian_count_ > 0) {
        write_torch_gaussian_ply(output_path, request->include_skybox);
        response->success = true;
        response->message = "saved Gaussian map to " + output_path.string();
        return;
      }

      write_debug_map_points_ply(output_path);
      response->success = true;
      response->message = "saved accumulated debug map points to " + output_path.string();
    } catch (const std::exception & ex) {
      response->success = false;
      response->message = ex.what();
    }
#else
    try {
      const auto output_path = resolve_debug_map_path(request->path);
      write_debug_map_points_ply(output_path);
      response->success = true;
      response->message = "saved accumulated debug map points to " + output_path.string();
    } catch (const std::exception & ex) {
      response->success = false;
      response->message = ex.what();
    }
#endif
  }

  sensor_msgs::msg::PointCloud2 make_map_points_message(
    const builtin_interfaces::msg::Time & stamp) const
  {
    const auto & points = dataset_.map_points_world();
    const auto & colors = dataset_.map_colors_rgb();
    const size_t point_count = std::min(points.size(), colors.size());

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = stamp;
    cloud.header.frame_id = world_frame_;
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(
      std::min<size_t>(point_count, std::numeric_limits<uint32_t>::max()));
    cloud.is_bigendian = false;
    cloud.is_dense = false;
    cloud.point_step = 16;
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.fields = {
      make_point_field("x", 0, sensor_msgs::msg::PointField::FLOAT32),
      make_point_field("y", 4, sensor_msgs::msg::PointField::FLOAT32),
      make_point_field("z", 8, sensor_msgs::msg::PointField::FLOAT32),
      make_point_field("rgb", 12, sensor_msgs::msg::PointField::FLOAT32),
    };
    cloud.data.resize(static_cast<size_t>(cloud.row_step));

    for (uint32_t i = 0; i < cloud.width; ++i) {
      const auto & point = points[i];
      const auto & color = colors[i];
      uint8_t * base = cloud.data.data() + static_cast<size_t>(i) * cloud.point_step;
      const float x = point.x();
      const float y = point.y();
      const float z = point.z();
      std::memcpy(base + 0, &x, sizeof(float));
      std::memcpy(base + 4, &y, sizeof(float));
      std::memcpy(base + 8, &z, sizeof(float));

      const uint32_t rgb =
        (static_cast<uint32_t>(color_channel_to_u8(color.x())) << 16U) |
        (static_cast<uint32_t>(color_channel_to_u8(color.y())) << 8U) |
        static_cast<uint32_t>(color_channel_to_u8(color.z()));
      float rgb_float = 0.0F;
      std::memcpy(&rgb_float, &rgb, sizeof(uint32_t));
      std::memcpy(base + 12, &rgb_float, sizeof(float));
    }

    return cloud;
  }

  sensor_msgs::msg::Image make_blank_rendered_image_message(
    const builtin_interfaces::msg::Time & stamp) const
  {
    sensor_msgs::msg::Image image;
    image.header.stamp = stamp;
    image.header.frame_id = camera_frame_;
    image.height = static_cast<uint32_t>(std::max(last_image_height_, 0));
    image.width = static_cast<uint32_t>(std::max(last_image_width_, 0));
    image.encoding = sensor_msgs::image_encodings::RGB8;
    image.is_bigendian = false;
    image.step = image.width * 3U;
    image.data.resize(static_cast<size_t>(image.height) * image.step);
    return image;
  }

  sensor_msgs::msg::Image make_input_preview_message(
    const builtin_interfaces::msg::Time & stamp) const
  {
    sensor_msgs::msg::Image image = make_blank_rendered_image_message(stamp);
    const cv::Mat & rgb_float = last_image_rgb_float_;

    if (rgb_float.empty()) {
      return image;
    }
    if (rgb_float.type() != CV_32FC3) {
      throw std::runtime_error("rendered preview expects RGB float image data");
    }

    for (uint32_t row = 0; row < image.height; ++row) {
      const auto * src = rgb_float.ptr<cv::Vec3f>(static_cast<int>(row));
      uint8_t * dst = image.data.data() + static_cast<size_t>(row) * image.step;
      for (uint32_t col = 0; col < image.width; ++col) {
        const cv::Vec3f & rgb = src[col];
        dst[col * 3U + 0U] = color_channel_to_u8(rgb[0]);
        dst[col * 3U + 1U] = color_channel_to_u8(rgb[1]);
        dst[col * 3U + 2U] = color_channel_to_u8(rgb[2]);
      }
    }

    return image;
  }

  sensor_msgs::msg::Image make_projected_map_preview_message(
    const builtin_interfaces::msg::Time & stamp) const
  {
    sensor_msgs::msg::Image image = make_blank_rendered_image_message(stamp);
    if (image.width == 0 || image.height == 0) {
      return image;
    }

    const auto & points = dataset_.map_points_world();
    const auto & colors = dataset_.map_colors_rgb();
    const size_t point_count = std::min(points.size(), colors.size());
    if (point_count == 0) {
      return make_input_preview_message(stamp);
    }

    const auto intrinsics = current_intrinsics();
    const Eigen::Matrix3d r_cw = last_q_wc_.toRotationMatrix().transpose();
    std::vector<float> z_buffer(static_cast<size_t>(image.width) * image.height,
      std::numeric_limits<float>::infinity());
    size_t visible_count = 0;

    for (size_t i = 0; i < point_count; ++i) {
      const Eigen::Vector3d xyz_world = points[i].cast<double>();
      const Eigen::Vector3d xyz_cam = r_cw * (xyz_world - last_t_wc_);
      if (xyz_cam.z() <= 0.0) {
        continue;
      }

      const double u_float = intrinsics.fx * xyz_cam.x() / xyz_cam.z() + intrinsics.cx;
      const double v_float = intrinsics.fy * xyz_cam.y() / xyz_cam.z() + intrinsics.cy;
      if (!std::isfinite(u_float) || !std::isfinite(v_float)) {
        continue;
      }

      const int u = static_cast<int>(std::floor(u_float));
      const int v = static_cast<int>(std::floor(v_float));
      if (u < 0 || v < 0 || u >= static_cast<int>(image.width) || v >= static_cast<int>(image.height)) {
        continue;
      }

      const size_t offset = static_cast<size_t>(v) * image.width + static_cast<size_t>(u);
      const float depth = static_cast<float>(xyz_cam.z());
      if (depth >= z_buffer[offset]) {
        continue;
      }

      z_buffer[offset] = depth;
      const Eigen::Vector3f & color = colors[i];
      uint8_t * dst = image.data.data() + offset * 3U;
      dst[0] = color_channel_to_u8(color.x());
      dst[1] = color_channel_to_u8(color.y());
      dst[2] = color_channel_to_u8(color.z());
      ++visible_count;
    }

    if (visible_count == 0) {
      return make_input_preview_message(stamp);
    }
    return image;
  }

#ifdef GAUSSIAN_LIC_ENABLE_TORCH
  sensor_msgs::msg::Image make_torch_gaussian_preview_message(
    const builtin_interfaces::msg::Time & stamp) const
  {
    sensor_msgs::msg::Image image = make_input_preview_message(stamp);
    if (
      image.width == 0 || image.height == 0 || !torch_gaussian_initialized_ ||
      torch_gaussian_count_ == 0)
    {
      return make_projected_map_preview_message(stamp);
    }

    torch::NoGradGuard no_grad;
    const auto xyz = torch_gaussian_map_.xyz.detach().to(torch::kCPU).contiguous();
    const auto features_dc = torch_gaussian_map_.features_dc.detach().to(torch::kCPU).contiguous();
    const auto scaling = torch_gaussian_map_.scaling.detach().to(torch::kCPU).contiguous();
    const auto opacity = torch_gaussian_map_.opacity.detach().to(torch::kCPU).contiguous();
    if (xyz.dim() != 2 || xyz.size(1) < 3 || features_dc.dim() != 3 || features_dc.size(2) < 3) {
      return make_projected_map_preview_message(stamp);
    }

    const auto xyz_a = xyz.accessor<float, 2>();
    const auto dc_a = features_dc.accessor<float, 3>();
    const auto scaling_a = scaling.accessor<float, 2>();
    const auto opacity_a = opacity.accessor<float, 2>();
    const auto intrinsics = current_intrinsics();
    const Eigen::Matrix3d r_cw = last_q_wc_.toRotationMatrix().transpose();
    std::vector<float> z_buffer(static_cast<size_t>(image.width) * image.height,
      std::numeric_limits<float>::infinity());
    size_t visible_count = 0;

    const int64_t start_index = static_cast<int64_t>(torch_gaussian_map_.skybox_count);
    for (int64_t i = start_index; i < xyz.size(0); ++i) {
      const Eigen::Vector3d xyz_world{
        static_cast<double>(xyz_a[i][0]),
        static_cast<double>(xyz_a[i][1]),
        static_cast<double>(xyz_a[i][2])};
      const Eigen::Vector3d xyz_cam = r_cw * (xyz_world - last_t_wc_);
      if (xyz_cam.z() <= 0.0) {
        continue;
      }

      const double u_float = intrinsics.fx * xyz_cam.x() / xyz_cam.z() + intrinsics.cx;
      const double v_float = intrinsics.fy * xyz_cam.y() / xyz_cam.z() + intrinsics.cy;
      if (!std::isfinite(u_float) || !std::isfinite(v_float)) {
        continue;
      }

      const float scale_x = std::exp(scaling_a[i][0]);
      const float scale_y = std::exp(scaling_a[i][1]);
      const float scale_seed = std::max(scale_x, scale_y);
      const float projected_radius = static_cast<float>(
        intrinsics.fx * static_cast<double>(scale_seed) / xyz_cam.z());
      const int radius = static_cast<int>(std::ceil(std::clamp(projected_radius, 1.0F, 8.0F)));
      const float sigma = std::max(static_cast<float>(radius) * 0.5F, 0.5F);
      const float alpha_seed = std::clamp(sigmoid(opacity_a[i][0]), 0.05F, 0.95F);
      const float rgb[3] = {
        sh_dc_to_rgb(dc_a[i][0][0]),
        sh_dc_to_rgb(dc_a[i][0][1]),
        sh_dc_to_rgb(dc_a[i][0][2])};

      const int u_center = static_cast<int>(std::lround(u_float));
      const int v_center = static_cast<int>(std::lround(v_float));
      for (int dv = -radius; dv <= radius; ++dv) {
        const int v = v_center + dv;
        if (v < 0 || v >= static_cast<int>(image.height)) {
          continue;
        }
        for (int du = -radius; du <= radius; ++du) {
          const int u = u_center + du;
          if (u < 0 || u >= static_cast<int>(image.width)) {
            continue;
          }
          const float d2 = static_cast<float>(du * du + dv * dv);
          if (d2 > static_cast<float>(radius * radius)) {
            continue;
          }

          const size_t offset = static_cast<size_t>(v) * image.width + static_cast<size_t>(u);
          const float depth = static_cast<float>(xyz_cam.z());
          if (depth > z_buffer[offset] + static_cast<float>(scale_seed)) {
            continue;
          }
          z_buffer[offset] = std::min(z_buffer[offset], depth);
          const float alpha = alpha_seed * std::exp(-0.5F * d2 / (sigma * sigma));
          uint8_t * dst = image.data.data() + offset * 3U;
          for (size_t channel = 0; channel < 3U; ++channel) {
            const float current = static_cast<float>(dst[channel]) / 255.0F;
            dst[channel] = color_channel_to_u8((1.0F - alpha) * current + alpha * rgb[channel]);
          }
          ++visible_count;
        }
      }
    }

    if (visible_count == 0) {
      return make_projected_map_preview_message(stamp);
    }
    return image;
  }
#endif

  sensor_msgs::msg::Image make_rendered_preview_message(
    const builtin_interfaces::msg::Time & stamp) const
  {
    const std::string mode = normalized_qos_token(render_mode_);
    if (mode == "off") {
      return sensor_msgs::msg::Image{};
    }
    if (mode == "debuginput") {
      return make_input_preview_message(stamp);
    }
    if (mode == "debugcpu") {
      return make_projected_map_preview_message(stamp);
    }
    if (mode == "rasterizer") {
#ifdef GAUSSIAN_LIC_ENABLE_TORCH
      return make_torch_gaussian_preview_message(stamp);
#else
      throw std::runtime_error(
        "render_mode=rasterizer requested, but the Gaussian rasterizer backend is not ported yet");
#endif
    }
    throw std::runtime_error(
      "render_mode must be debug_cpu, debug_input, rasterizer, or off, got " + render_mode_);
  }

  void publish_tracking_outputs(const MapperFrameData & frame_data)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = frame_data.stamp;
    pose.header.frame_id = world_frame_;
    pose.pose.position.x = frame_data.t_wc.x();
    pose.pose.position.y = frame_data.t_wc.y();
    pose.pose.position.z = frame_data.t_wc.z();
    pose.pose.orientation.x = frame_data.q_wc.x();
    pose.pose.orientation.y = frame_data.q_wc.y();
    pose.pose.orientation.z = frame_data.q_wc.z();
    pose.pose.orientation.w = frame_data.q_wc.w();

    nav_msgs::msg::Odometry odometry;
    odometry.header = pose.header;
    odometry.child_frame_id = camera_frame_;
    odometry.pose.pose = pose.pose;
    odometry_pub_->publish(odometry);

    if (tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = pose.header;
      transform.child_frame_id = camera_frame_;
      transform.transform.translation.x = pose.pose.position.x;
      transform.transform.translation.y = pose.pose.position.y;
      transform.transform.translation.z = pose.pose.position.z;
      transform.transform.rotation = pose.pose.orientation;
      tf_broadcaster_->sendTransform(transform);
    }

    path_msg_.header.stamp = frame_data.stamp;
    path_msg_.header.frame_id = world_frame_;
    path_msg_.poses.push_back(pose);
    while (max_path_length_ > 0 && path_msg_.poses.size() > static_cast<size_t>(max_path_length_)) {
      path_msg_.poses.erase(path_msg_.poses.begin());
    }
    path_pub_->publish(path_msg_);
  }

  void publish_status()
  {
    const auto rate_time = std::chrono::steady_clock::now();
    const std::chrono::duration<double> rate_dt = rate_time - last_rate_sample_time_;
    if (rate_dt.count() > 0.0) {
      last_tracking_hz_ =
        static_cast<float>(
          static_cast<double>(aligned_frame_count_ - last_rate_aligned_count_) / rate_dt.count());
      last_mapping_hz_ =
        static_cast<float>(
          static_cast<double>(converted_frame_count_ - last_rate_converted_count_) / rate_dt.count());
      last_rate_sample_time_ = rate_time;
      last_rate_aligned_count_ = aligned_frame_count_;
      last_rate_converted_count_ = converted_frame_count_;
    }

    size_t q_points = 0;
    size_t q_pose = 0;
    size_t q_image = 0;
    size_t q_depth = 0;
    {
      std::scoped_lock lock(buffer_mutex_);
      q_points = point_buf_.size();
      q_pose = pose_buf_.size();
      q_image = image_buf_.size();
      q_depth = depth_buf_.size();
    }

    gaussian_lic_msgs::msg::MappingStatus msg;
    msg.header.stamp = now();
    msg.header.frame_id = "map";
    msg.state = aligned_frame_count_ > 0 ?
      gaussian_lic_msgs::msg::MappingStatus::STATE_ACTIVE :
      gaussian_lic_msgs::msg::MappingStatus::STATE_INACTIVE;
    msg.tracking_state = aligned_frame_count_ > 0 ?
      gaussian_lic_msgs::msg::MappingStatus::STATE_ACTIVE :
      gaussian_lic_msgs::msg::MappingStatus::STATE_INACTIVE;
    msg.mapping_state = converted_frame_count_ > 0 ?
      gaussian_lic_msgs::msg::MappingStatus::STATE_ACTIVE :
      gaussian_lic_msgs::msg::MappingStatus::STATE_INACTIVE;
    msg.render_mode = render_mode_status_value();
#ifdef GAUSSIAN_LIC_ENABLE_TORCH
    if (torch_gaussian_initialized_) {
      msg.status_text = "ROS2 frame conversion active; Torch Gaussian map updating on keyframes";
    } else if (enable_torch_gaussian_init_) {
      msg.status_text = "ROS2 frame conversion active; waiting for Torch Gaussian initialization";
    } else {
      msg.status_text = "ROS2 frame conversion active; Torch Gaussian initialization disabled";
    }
    msg.num_gaussians = torch_gaussian_count_;
#else
    msg.status_text = "ROS2 frame conversion active; Gaussian-LIC torch core not linked";
    msg.num_gaussians = 0;
#endif
    msg.num_keyframes = dataset_.train_frame_count();
    msg.num_tracking_frames = aligned_frame_count_;
    msg.num_mapping_frames = converted_frame_count_;
    msg.num_dropped_messages =
      dropped_pointcloud_count_ + dropped_pose_count_ + dropped_image_count_ + dropped_depth_count_;
    msg.num_errors =
      conversion_error_count_ + torch_camera_error_count_ + torch_gaussian_init_error_count_ +
      torch_gaussian_extend_error_count_ + render_error_count_;
    msg.tracking_hz = last_tracking_hz_;
    msg.mapping_hz = last_mapping_hz_;
    msg.tracking_latency_ms = 0.0F;
    msg.mapping_latency_ms = last_mapping_latency_ms_;
    msg.mean_iteration_ms = mean_iteration_ms_;
    msg.gpu_memory_mb = 0.0F;
    msg.active_profile = active_profile_;
    status_pub_->publish(msg);

    const auto intrinsics = current_intrinsics();
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "received points=%lu pose=%lu image=%lu depth=%lu imu=%lu | aligned=%lu | "
      "converted=%lu train=%zu test=%zu dataset_frames=%zu last_image=%dx%d "
      "last_points=%zu pending_points=%zu map_points=%zu total_points=%zu | "
      "rate tracking=%.2fHz mapping=%.2fHz | "
      "torch_cameras=%lu torch_errors=%lu torch_image=%s torch_depth=%s | "
      "torch_gaussians=%zu gaussian_inits=%lu gaussian_extends=%lu last_inserted=%zu "
      "gaussian_init_errors=%lu gaussian_extend_errors=%lu gaussian_xyz=%s gaussian_features=%s "
      "gaussian_device=%s | latency last=%.3fms mean=%.3fms | "
      "camera_info=%lu intrinsics=%s fx=%.3f fy=%.3f cx=%.3f cy=%.3f | "
      "dropped points=%lu pose=%lu image=%lu depth=%lu skipped_depth=%lu conversion_errors=%lu | "
      "queues=%zu/%zu/%zu/%zu",
      pointcloud_count_, pose_count_, image_count_, depth_count_, imu_count_, aligned_frame_count_,
      converted_frame_count_, dataset_.train_frame_count(), dataset_.test_frame_count(),
      dataset_.all_frame_count(), last_image_width_, last_image_height_,
      last_points_in_frame_, dataset_.pending_point_count(), dataset_.map_point_count(),
      dataset_.total_point_count(),
      last_tracking_hz_, last_mapping_hz_,
      torch_camera_count_, torch_camera_error_count_, last_torch_image_dims_.c_str(),
      last_torch_depth_dims_.c_str(),
      torch_gaussian_count_, torch_gaussian_init_count_, torch_gaussian_extend_count_,
      last_torch_gaussian_inserted_, torch_gaussian_init_error_count_,
      torch_gaussian_extend_error_count_, last_torch_gaussian_xyz_dims_.c_str(),
      last_torch_gaussian_feature_dims_.c_str(), torch_gaussian_device_label_.c_str(),
      last_mapping_latency_ms_, mean_iteration_ms_,
      camera_info_count_, intrinsics.source.c_str(), intrinsics.fx, intrinsics.fy, intrinsics.cx, intrinsics.cy,
      dropped_pointcloud_count_, dropped_pose_count_, dropped_image_count_, dropped_depth_count_,
      dataset_.skipped_nonpositive_depth_count(), conversion_error_count_,
      q_points, q_pose, q_image, q_depth);
  }

  std::string image_topic_;
  std::string camera_info_topic_;
  std::string depth_topic_;
  std::string pose_topic_;
  std::string pointcloud_topic_;
  std::string imu_topic_;
  std::string status_topic_;
  std::string odometry_topic_;
  std::string path_topic_;
  std::string map_points_topic_;
  std::string rendered_image_topic_;
  std::string gaussian_map_topic_;
  std::string save_map_service_;
  std::string world_frame_{"map"};
  std::string camera_frame_{"camera"};
  bool publish_tf_{false};
  double sync_tolerance_sec_{0.01};
  int max_queue_size_{10000};
  std::string sensor_qos_reliability_{"best_effort"};
  std::string sensor_qos_history_{"keep_last"};
  int sensor_qos_depth_{5};
  int process_period_ms_{5};
  int select_every_k_frame_{8};
  bool require_depth_topic_{true};
  double fx_{1.0};
  double fy_{1.0};
  double cx_{0.5};
  double cy_{0.5};
  mutable std::mutex intrinsics_mutex_;
  bool camera_info_received_{false};
  std::string intrinsics_source_{"params"};
  bool enable_torch_camera_conversion_{false};
  bool enable_torch_gaussian_init_{false};
  bool enable_torch_gaussian_extend_{true};
  bool publish_rendered_preview_{true};
  std::string active_profile_{"default"};
  std::string render_mode_{"debug_cpu"};
  std::string rendered_image_mode_;
  int gaussian_init_min_points_{1};
  int gaussian_init_min_keyframes_{1};
  gaussian_lic_mapping::GaussianBackendConfig backend_config_;
  std::string torch_gaussian_device_name_{"cpu"};
  int gaussian_map_chunk_size_{1024};
  int max_path_length_{5000};
  int max_map_points_{200000};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<gaussian_lic_msgs::msg::MappingStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_points_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rendered_image_pub_;
  rclcpp::Publisher<gaussian_lic_msgs::msg::GaussianArray>::SharedPtr gaussian_map_pub_;
  rclcpp::Service<gaussian_lic_msgs::srv::SaveMap>::SharedPtr save_map_srv_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr process_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  MapperDataset dataset_;
  nav_msgs::msg::Path path_msg_;

  std::mutex buffer_mutex_;
  std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr> point_buf_;
  std::deque<geometry_msgs::msg::PoseStamped::ConstSharedPtr> pose_buf_;
  std::deque<sensor_msgs::msg::Image::ConstSharedPtr> image_buf_;
  std::deque<sensor_msgs::msg::Image::ConstSharedPtr> depth_buf_;

  builtin_interfaces::msg::Time last_aligned_stamp_;
  builtin_interfaces::msg::Time last_imu_stamp_;
  std::chrono::steady_clock::time_point last_rate_sample_time_{std::chrono::steady_clock::now()};
  uint64_t last_rate_aligned_count_{0};
  uint64_t last_rate_converted_count_{0};
  float last_tracking_hz_{0.0F};
  float last_mapping_hz_{0.0F};
  float last_mapping_latency_ms_{0.0F};
  float mean_iteration_ms_{0.0F};
  double cumulative_iteration_ms_{0.0};
  uint64_t iteration_timing_count_{0};

  uint64_t pointcloud_count_{0};
  uint64_t pose_count_{0};
  uint64_t image_count_{0};
  uint64_t camera_info_count_{0};
  uint64_t depth_count_{0};
  uint64_t imu_count_{0};
  uint64_t aligned_frame_count_{0};
  uint64_t converted_frame_count_{0};
  uint64_t torch_camera_count_{0};
  uint64_t torch_camera_error_count_{0};
  uint64_t torch_gaussian_init_count_{0};
  uint64_t torch_gaussian_extend_count_{0};
  uint64_t torch_gaussian_init_error_count_{0};
  uint64_t torch_gaussian_extend_error_count_{0};
  uint64_t dropped_pointcloud_count_{0};
  uint64_t dropped_pose_count_{0};
  uint64_t dropped_image_count_{0};
  uint64_t dropped_depth_count_{0};
  uint64_t conversion_error_count_{0};
  uint64_t render_error_count_{0};
  int last_image_width_{0};
  int last_image_height_{0};
  cv::Mat last_image_rgb_float_;
  Eigen::Quaterniond last_q_wc_{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d last_t_wc_{Eigen::Vector3d::Zero()};
  size_t last_points_in_frame_{0};
  size_t torch_gaussian_count_{0};
  size_t last_torch_gaussian_inserted_{0};
  std::string last_torch_image_dims_{"n/a"};
  std::string last_torch_depth_dims_{"n/a"};
  std::string last_torch_gaussian_xyz_dims_{"n/a"};
  std::string last_torch_gaussian_feature_dims_{"n/a"};
  std::string torch_gaussian_device_label_{"n/a"};
#ifdef GAUSSIAN_LIC_ENABLE_TORCH
  bool torch_gaussian_initialized_{false};
  gaussian_lic_mapping::TorchGaussianMap torch_gaussian_map_;
#endif
};

#ifndef GAUSSIAN_LIC_MAPPING_COMPOSITION
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MappingNode>());
  rclcpp::shutdown();
  return 0;
}
#else
RCLCPP_COMPONENTS_REGISTER_NODE(MappingNode)
#endif
