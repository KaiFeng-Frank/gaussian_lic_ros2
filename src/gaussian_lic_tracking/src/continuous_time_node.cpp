// SPDX-License-Identifier: GPL-3.0-or-later
//
// Independent ROS2 node that exercises the newly-ported continuous-time
// tracker on real-world streams. Subscribes to a raw IMU topic, feeds the
// samples into `ContinuousTimeSlidingWindowEstimator`, periodically solves,
// and publishes the optimized body pose as a `nav_msgs/Odometry` message
// plus an aggregated `Path`.
//
// This node is deliberately separate from `tracking_node` so it can be
// validated against real bags without risking regressions in the existing
// 12/12 strict parity matrix.

#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <deque>
#include <fstream>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <gaussian_lic_msgs/msg/gaussian_array.hpp>
#include <gaussian_lic_tracking/gaussian_snapshot.hpp>
#include <gaussian_lic_tracking/lidar_factor.hpp>
#include <gaussian_lic_tracking/time.hpp>
#include <gaussian_lic_tracking/visual_factor.hpp>
#include <gaussian_lic_tracking/spline/continuous_time_sliding_window.hpp>
#include <gaussian_lic_tracking/spline/lidar_plane_extractor.hpp>
#include <gaussian_lic_tracking/spline/so3_ops.hpp>

namespace gaussian_lic_tracking
{

namespace
{

enum class VisualSe3PriorStatus
{
  kSkipped,
  kDepthMiss,
  kRejected,
  kAdded
};

const sensor_msgs::msg::PointField * find_pointcloud_field(
  const sensor_msgs::msg::PointCloud2 & msg,
  const std::string & field_name)
{
  const auto it = std::find_if(
    msg.fields.begin(), msg.fields.end(),
    [&field_name](const sensor_msgs::msg::PointField & field) {
      return field.name == field_name;
    });
  return it == msg.fields.end() ? nullptr : &(*it);
}

const sensor_msgs::msg::PointField * find_point_time_field(
  const sensor_msgs::msg::PointCloud2 & msg)
{
  for (const std::string & field_name : {"offset_time", "time", "timestamp", "t"}) {
    if (const auto * field = find_pointcloud_field(msg, field_name); field != nullptr) {
      return field;
    }
  }
  return nullptr;
}

size_t point_field_scalar_size(const sensor_msgs::msg::PointField & field)
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

bool read_point_numeric_field(
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
        uint32_t raw = 0U;
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

std::optional<int64_t> decode_point_stamp_ns(
  const sensor_msgs::msg::PointCloud2 & msg,
  const sensor_msgs::msg::PointField * time_field,
  const size_t point_index,
  const int64_t cloud_stamp_ns)
{
  const size_t point_count = static_cast<size_t>(msg.width) * static_cast<size_t>(msg.height);
  if (time_field == nullptr || msg.point_step == 0U || point_index >= point_count) {
    return std::nullopt;
  }
  const size_t scalar_size = point_field_scalar_size(*time_field);
  const size_t base = point_index * static_cast<size_t>(msg.point_step);
  if (scalar_size == 0U || base + time_field->offset + scalar_size > msg.data.size()) {
    return std::nullopt;
  }
  double raw_time = 0.0;
  if (!read_point_numeric_field(msg.data.data() + base, *time_field, raw_time) ||
    !std::isfinite(raw_time))
  {
    return std::nullopt;
  }

  auto scale_to_ns = [](const double value, const double scale) {
      return static_cast<int64_t>(std::llround(value * scale));
    };
  bool offset_mode = true;
  int64_t time_ns = 0;
  const double abs_time = std::abs(raw_time);
  if (time_field->name == "offset_time") {
    time_ns = static_cast<int64_t>(std::llround(raw_time));
  } else if (abs_time > 1.0e17) {
    time_ns = static_cast<int64_t>(std::llround(raw_time));
    offset_mode = false;
  } else if (abs_time > 1.0e14) {
    time_ns = scale_to_ns(raw_time, 1.0e3);
    offset_mode = false;
  } else if ((time_field->name == "timestamp" || time_field->name == "t") && abs_time > 1.0e8) {
    time_ns = scale_to_ns(raw_time, 1.0e9);
    offset_mode = false;
  } else {
    time_ns = scale_to_ns(raw_time, 1.0e9);
  }
  return offset_mode ? cloud_stamp_ns + time_ns : time_ns;
}

int64_t nearest_point_stamp_ns(
  const std::vector<Eigen::Vector3d> & points,
  const std::vector<int64_t> & point_stamps_ns,
  const Eigen::Vector3d & sample_point,
  const int64_t fallback_stamp_ns)
{
  if (points.empty() || points.size() != point_stamps_ns.size() || !sample_point.allFinite()) {
    return fallback_stamp_ns;
  }
  double best_distance_sq = std::numeric_limits<double>::infinity();
  int64_t best_stamp_ns = fallback_stamp_ns;
  for (size_t index = 0; index < points.size(); ++index) {
    const double distance_sq = (points[index] - sample_point).squaredNorm();
    if (distance_sq < best_distance_sq) {
      best_distance_sq = distance_sq;
      best_stamp_ns = point_stamps_ns[index];
    }
  }
  return best_stamp_ns;
}

int64_t pointcloud_required_pose_stamp_ns(
  const sensor_msgs::msg::PointCloud2 & msg,
  const int64_t cloud_stamp_ns,
  const double max_abs_point_time_offset_s)
{
  const auto * time_field = find_point_time_field(msg);
  if (time_field == nullptr) {
    return cloud_stamp_ns;
  }
  int64_t required_stamp_ns = cloud_stamp_ns;
  const size_t point_count = static_cast<size_t>(msg.width) * static_cast<size_t>(msg.height);
  for (size_t point_index = 0U; point_index < point_count; ++point_index) {
    const auto decoded_stamp = decode_point_stamp_ns(msg, time_field, point_index, cloud_stamp_ns);
    if (!decoded_stamp.has_value()) {
      continue;
    }
    const double abs_offset_s =
      std::abs(static_cast<double>(decoded_stamp.value() - cloud_stamp_ns) * 1.0e-9);
    if (abs_offset_s <= max_abs_point_time_offset_s) {
      required_stamp_ns = std::max(required_stamp_ns, decoded_stamp.value());
    }
  }
  return required_stamp_ns;
}

Eigen::Quaterniond quaternion_from_rotation_vector(const Eigen::Vector3d & rotation_vector)
{
  if (!rotation_vector.allFinite()) {
    return Eigen::Quaterniond::Identity();
  }
  const double angle = rotation_vector.norm();
  if (angle < 1.0e-12) {
    return Eigen::Quaterniond::Identity();
  }
  return Eigen::Quaterniond(Eigen::AngleAxisd(angle, rotation_vector / angle)).normalized();
}

bool decode_image_gray(const sensor_msgs::msg::Image & msg, VisualFrame & frame)
{
  if (msg.width == 0U || msg.height == 0U || msg.data.empty()) {
    return false;
  }
  const auto width = static_cast<size_t>(msg.width);
  const auto height = static_cast<size_t>(msg.height);
  const auto pixel_count = width * height;
  frame.stamp_ns = stamp_to_nanoseconds(msg.header.stamp);
  frame.width = width;
  frame.height = height;
  frame.gray.assign(pixel_count, 0.0F);

  const std::string & encoding = msg.encoding;
  if (encoding == "mono8" || encoding == "8UC1") {
    if (msg.step < width || msg.data.size() < msg.step * height) {
      return false;
    }
    for (size_t y = 0; y < height; ++y) {
      const size_t row = y * msg.step;
      for (size_t x = 0; x < width; ++x) {
        frame.gray[y * width + x] =
          static_cast<float>(msg.data[row + x]) / 255.0F;
      }
    }
    return true;
  }

  size_t channels = 0U;
  bool bgr_order = false;
  if (encoding == "rgb8") {
    channels = 3U;
  } else if (encoding == "bgr8") {
    channels = 3U;
    bgr_order = true;
  } else if (encoding == "rgba8") {
    channels = 4U;
  } else if (encoding == "bgra8") {
    channels = 4U;
    bgr_order = true;
  } else {
    return false;
  }
  if (msg.step < width * channels || msg.data.size() < msg.step * height) {
    return false;
  }
  for (size_t y = 0; y < height; ++y) {
    const size_t row = y * msg.step;
    for (size_t x = 0; x < width; ++x) {
      const size_t base = row + x * channels;
      const uint8_t c0 = msg.data[base + 0U];
      const uint8_t c1 = msg.data[base + 1U];
      const uint8_t c2 = msg.data[base + 2U];
      const double r = bgr_order ? static_cast<double>(c2) : static_cast<double>(c0);
      const double g = static_cast<double>(c1);
      const double b = bgr_order ? static_cast<double>(c0) : static_cast<double>(c2);
      frame.gray[y * width + x] =
        static_cast<float>((0.299 * r + 0.587 * g + 0.114 * b) / 255.0);
    }
  }
  return true;
}

}  // namespace

class ContinuousTimeNode : public rclcpp::Node
{
public:
  ContinuousTimeNode()
  : rclcpp::Node("continuous_time_node")
  {
    raw_imu_topic_ = declare_parameter<std::string>(
      "raw_imu_topic", "/imu_for_gs");
    raw_image_topic_ = declare_parameter<std::string>(
      "raw_image_topic", "/camera/image");
    raw_camera_info_topic_ = declare_parameter<std::string>(
      "raw_camera_info_topic", "/camera/camera_info");
    external_odometry_prior_topic_ = declare_parameter<std::string>(
      "external_odometry_prior_topic", "");
    enable_external_odometry_prior_ =
      declare_parameter<bool>("enable_external_odometry_prior", false);
    enable_external_odometry_position_factors_ =
      declare_parameter<bool>("enable_external_odometry_position_factors", false);
    enable_external_odometry_orientation_factors_ =
      declare_parameter<bool>("enable_external_odometry_orientation_factors", false);
    external_odometry_position_factor_weight_ =
      declare_parameter<double>("external_odometry_position_factor_weight", 1.0);
    external_odometry_position_factor_huber_delta_m_ =
      declare_parameter<double>("external_odometry_position_factor_huber_delta_m", 0.25);
    external_odometry_orientation_factor_weight_ =
      declare_parameter<double>("external_odometry_orientation_factor_weight", 1.0);
    external_odometry_orientation_factor_huber_delta_rad_ =
      declare_parameter<double>("external_odometry_orientation_factor_huber_delta_rad", 0.25);
    if (!std::isfinite(external_odometry_position_factor_weight_) ||
      external_odometry_position_factor_weight_ <= 0.0 ||
      !std::isfinite(external_odometry_position_factor_huber_delta_m_) ||
      external_odometry_position_factor_huber_delta_m_ < 0.0 ||
      !std::isfinite(external_odometry_orientation_factor_weight_) ||
      external_odometry_orientation_factor_weight_ <= 0.0 ||
      !std::isfinite(external_odometry_orientation_factor_huber_delta_rad_) ||
      external_odometry_orientation_factor_huber_delta_rad_ < 0.0)
    {
      throw std::runtime_error(
        "external odometry factor weight/huber parameters must be finite");
    }
    // Mount rotation: q_imu_in_prior, expressed in (x, y, z, w).
    // For datasets whose ground truth is in a robot-base frame that differs
    // from the IMU sensor frame (e.g. M2DGR uses a base frame rotated ~180°
    // about x relative to the IMU), this rotation gets right-multiplied
    // into the prior's pose before it becomes the seed orientation.
    const auto prior_to_imu_param = declare_parameter<std::vector<double>>(
      "prior_to_imu_rotation_xyzw", std::vector<double>{0.0, 0.0, 0.0, 1.0});
    if (prior_to_imu_param.size() == 4) {
      prior_to_imu_rotation_ = Eigen::Quaterniond(
        prior_to_imu_param[3],
        prior_to_imu_param[0],
        prior_to_imu_param[1],
        prior_to_imu_param[2]).normalized();
    }
    const auto camera_to_imu_param = declare_parameter<std::vector<double>>(
      "camera_to_imu_rotation_xyzw", std::vector<double>{0.0, 0.0, 0.0, 1.0});
    if (camera_to_imu_param.size() == 4) {
      camera_to_imu_rotation_ = Eigen::Quaterniond(
        camera_to_imu_param[3],
        camera_to_imu_param[0],
        camera_to_imu_param[1],
        camera_to_imu_param[2]).normalized();
    }
    const auto camera_to_imu_translation_param = declare_parameter<std::vector<double>>(
      "camera_to_imu_translation_m", std::vector<double>{0.0, 0.0, 0.0});
    if (camera_to_imu_translation_param.size() == 3) {
      camera_to_imu_translation_ = Eigen::Vector3d(
        camera_to_imu_translation_param[0],
        camera_to_imu_translation_param[1],
        camera_to_imu_translation_param[2]);
    }
    odometry_topic_ = declare_parameter<std::string>(
      "odometry_topic", "/gaussian_lic/continuous_time/odometry");
    path_topic_ = declare_parameter<std::string>(
      "path_topic", "/gaussian_lic/continuous_time/path");
    gaussian_map_topic_ = declare_parameter<std::string>(
      "gaussian_map_topic", "/gaussian_lic/gaussian_map");
    gaussian_snapshot_qos_depth_ = static_cast<int>(
      declare_parameter<int>("gaussian_snapshot_qos_depth", 64));
    body_frame_id_ = declare_parameter<std::string>("body_frame_id", "imu_link");
    world_frame_id_ = declare_parameter<std::string>("world_frame_id", "map");

    spline::ContinuousTimeSlidingWindowOptions options;
    options.dt_s = declare_parameter<double>("knot_interval_seconds", 0.05);
    options.window_knot_count =
      static_cast<int>(declare_parameter<int>("window_knot_count", 8));
    options.marginalize_oldest_count =
      static_cast<int>(declare_parameter<int>("marginalize_oldest_count", 1));
    options.max_iterations_per_step =
      static_cast<int>(declare_parameter<int>("max_iterations_per_step", 1));
    options.imu_info_gyro =
      declare_parameter<double>("imu_info_gyro", 10.0);
    options.imu_info_accel =
      declare_parameter<double>("imu_info_accel", 1.0);
    if (!std::isfinite(options.imu_info_gyro) || options.imu_info_gyro <= 0.0 ||
      !std::isfinite(options.imu_info_accel) || options.imu_info_accel <= 0.0)
    {
      throw std::runtime_error("imu_info_gyro and imu_info_accel must be finite and positive");
    }
    options.ceres_initial_trust_region_radius =
      declare_parameter<double>("ceres_initial_trust_region_radius", 0.0);
    options.ceres_max_trust_region_radius =
      declare_parameter<double>("ceres_max_trust_region_radius", 0.0);
    if (!std::isfinite(options.ceres_initial_trust_region_radius) ||
      options.ceres_initial_trust_region_radius < 0.0 ||
      !std::isfinite(options.ceres_max_trust_region_radius) ||
      options.ceres_max_trust_region_radius < 0.0)
    {
      throw std::runtime_error("Ceres trust-region radii must be finite and non-negative");
    }
    options.position_smoothness_weight =
      declare_parameter<double>("position_smoothness_weight", 0.0);
    options.position_smoothness_huber_delta_m =
      declare_parameter<double>("position_smoothness_huber_delta_m", 0.0);
    options.rotation_smoothness_weight =
      declare_parameter<double>("rotation_smoothness_weight", 0.0);
    options.rotation_smoothness_huber_delta_rad =
      declare_parameter<double>("rotation_smoothness_huber_delta_rad", 0.0);
    options.retained_knot_prior_count =
      declare_parameter<int>("retained_knot_prior_count", 0);
    options.retained_knot_position_prior_weight =
      declare_parameter<double>("retained_knot_position_prior_weight", 0.0);
    options.retained_knot_position_prior_huber_delta_m =
      declare_parameter<double>("retained_knot_position_prior_huber_delta_m", 0.0);
    options.retained_knot_orientation_prior_weight =
      declare_parameter<double>("retained_knot_orientation_prior_weight", 0.0);
    options.retained_knot_orientation_prior_huber_delta_rad =
      declare_parameter<double>("retained_knot_orientation_prior_huber_delta_rad", 0.0);
    options.enable_spline_orientation_marginalization_prior =
      declare_parameter<bool>("enable_spline_orientation_marginalization_prior", false);
    options.gyro_bias_prior_weight =
      declare_parameter<double>("gyro_bias_prior_weight", 0.0);
    options.gyro_bias_prior_huber_delta_radps =
      declare_parameter<double>("gyro_bias_prior_huber_delta_radps", 0.0);
    options.accel_bias_prior_weight =
      declare_parameter<double>("accel_bias_prior_weight", 0.0);
    options.accel_bias_prior_huber_delta_mps2 =
      declare_parameter<double>("accel_bias_prior_huber_delta_mps2", 0.0);
    options.bias_random_walk_reference_dt_s =
      declare_parameter<double>("bias_random_walk_reference_dt_s", 1.0);
    options.gyro_bias_random_walk_sigma_radps_per_sqrt_s =
      declare_parameter<double>("gyro_bias_random_walk_sigma_radps_per_sqrt_s", 0.0);
    options.accel_bias_random_walk_sigma_mps2_per_sqrt_s =
      declare_parameter<double>("accel_bias_random_walk_sigma_mps2_per_sqrt_s", 0.0);
    if (!std::isfinite(options.position_smoothness_weight) ||
      options.position_smoothness_weight < 0.0 ||
      !std::isfinite(options.position_smoothness_huber_delta_m) ||
      options.position_smoothness_huber_delta_m < 0.0 ||
      !std::isfinite(options.rotation_smoothness_weight) ||
      options.rotation_smoothness_weight < 0.0 ||
      !std::isfinite(options.rotation_smoothness_huber_delta_rad) ||
      options.rotation_smoothness_huber_delta_rad < 0.0 ||
      options.retained_knot_prior_count < 0 ||
      !std::isfinite(options.retained_knot_position_prior_weight) ||
      options.retained_knot_position_prior_weight < 0.0 ||
      !std::isfinite(options.retained_knot_position_prior_huber_delta_m) ||
      options.retained_knot_position_prior_huber_delta_m < 0.0 ||
      !std::isfinite(options.retained_knot_orientation_prior_weight) ||
      options.retained_knot_orientation_prior_weight < 0.0 ||
      !std::isfinite(options.retained_knot_orientation_prior_huber_delta_rad) ||
      options.retained_knot_orientation_prior_huber_delta_rad < 0.0)
    {
      throw std::runtime_error(
              "smoothness and retained knot prior parameters must be finite and non-negative");
    }
    if (!std::isfinite(options.gyro_bias_prior_weight) ||
      options.gyro_bias_prior_weight < 0.0 ||
      !std::isfinite(options.gyro_bias_prior_huber_delta_radps) ||
      options.gyro_bias_prior_huber_delta_radps < 0.0 ||
      !std::isfinite(options.accel_bias_prior_weight) ||
      options.accel_bias_prior_weight < 0.0 ||
      !std::isfinite(options.accel_bias_prior_huber_delta_mps2) ||
      options.accel_bias_prior_huber_delta_mps2 < 0.0)
    {
      throw std::runtime_error("bias prior parameters must be finite and non-negative");
    }
    if (!std::isfinite(options.bias_random_walk_reference_dt_s) ||
      options.bias_random_walk_reference_dt_s <= 0.0 ||
      !std::isfinite(options.gyro_bias_random_walk_sigma_radps_per_sqrt_s) ||
      options.gyro_bias_random_walk_sigma_radps_per_sqrt_s < 0.0 ||
      !std::isfinite(options.accel_bias_random_walk_sigma_mps2_per_sqrt_s) ||
      options.accel_bias_random_walk_sigma_mps2_per_sqrt_s < 0.0)
    {
      throw std::runtime_error("bias random-walk parameters must be finite and non-negative");
    }
    options.lidar_huber_delta_m =
      declare_parameter<double>("lidar_huber_delta_m", 0.10);
    lidar_huber_delta_m_ = options.lidar_huber_delta_m;
    const auto gravity_param =
      declare_parameter<std::vector<double>>(
      "gravity_world", std::vector<double>{0.0, 0.0, -9.81});
    if (gravity_param.size() == 3) {
      options.gravity_world =
        Eigen::Vector3d(gravity_param[0], gravity_param[1], gravity_param[2]);
    }
    options.hold_gravity_constant =
      declare_parameter<bool>("hold_gravity_constant", true);
    options.hold_accel_bias_constant =
      declare_parameter<bool>("hold_accel_bias_constant", false);
    options.hold_gyro_bias_constant =
      declare_parameter<bool>("hold_gyro_bias_constant", false);
    options.fixed_control_point_index =
      declare_parameter<int>("fixed_control_point_index", -1);
    if (options.fixed_control_point_index < -1) {
      throw std::runtime_error("fixed_control_point_index must be >= -1");
    }
    options.max_position_update_m =
      declare_parameter<double>("max_position_update_m", 2.0);
    options.max_rotation_update_rad =
      declare_parameter<double>("max_rotation_update_rad", 0.50);
    options.update_gate_edge_knot_margin =
      declare_parameter<int>("update_gate_edge_knot_margin", 0);
    options.position_extrapolation_damping =
      declare_parameter<double>("position_extrapolation_damping", 0.0);
    if (!std::isfinite(options.position_extrapolation_damping) ||
      options.position_extrapolation_damping < 0.0 ||
      options.position_extrapolation_damping > 1.0)
    {
      throw std::runtime_error("position_extrapolation_damping must be finite in [0, 1]");
    }
    options.apply_position_update_on_rotation_reject =
      declare_parameter<bool>("apply_position_update_on_rotation_reject", false);
    options.apply_limited_rotation_update =
      declare_parameter<bool>("apply_limited_rotation_update", false);
    options.apply_limited_position_update =
      declare_parameter<bool>("apply_limited_position_update", false);
    options.scale_position_with_limited_rotation =
      declare_parameter<bool>("scale_position_with_limited_rotation", true);
    enable_startup_bias_autocal_ =
      declare_parameter<bool>("enable_startup_bias_autocal", true);
    imu_linear_acceleration_scale_ =
      declare_parameter<double>("imu_linear_acceleration_scale", 1.0);
    if (!std::isfinite(imu_linear_acceleration_scale_) ||
      imu_linear_acceleration_scale_ <= 0.0)
    {
      throw std::runtime_error("imu_linear_acceleration_scale must be finite and positive");
    }

    step_period_seconds_ =
      declare_parameter<double>("step_period_seconds", 0.10);
    use_stamp_driven_steps_ =
      declare_parameter<bool>("use_stamp_driven_steps", false);
    max_stamp_driven_steps_per_callback_ =
      static_cast<int>(declare_parameter<int>("max_stamp_driven_steps_per_callback", 4));
    if (max_stamp_driven_steps_per_callback_ <= 0) {
      throw std::runtime_error("max_stamp_driven_steps_per_callback must be positive");
    }
    pose_output_period_seconds_ =
      declare_parameter<double>("pose_output_period_seconds", 0.0);
    if (!std::isfinite(pose_output_period_seconds_) ||
      pose_output_period_seconds_ < 0.0)
    {
      throw std::runtime_error("pose_output_period_seconds must be finite and non-negative");
    }
    diagnostic_log_period_steps_ =
      static_cast<int>(declare_parameter<int>("diagnostic_log_period_steps", 50));
    if (diagnostic_log_period_steps_ < 0) {
      throw std::runtime_error("diagnostic_log_period_steps must be >= 0");
    }
    seed_min_imu_count_ =
      static_cast<int>(declare_parameter<int>("seed_min_imu_count", 25));
    max_path_history_ =
      static_cast<int>(declare_parameter<int>("max_path_history", 5000));
    enable_visual_rotation_prior_ =
      declare_parameter<bool>("enable_visual_rotation_prior", false);
    visual_rotation_prior_weight_ =
      declare_parameter<double>("visual_rotation_prior_weight", 0.1);
    visual_rotation_prior_huber_delta_rad_ =
      declare_parameter<double>("visual_rotation_prior_huber_delta_rad", 0.05);
    visual_rotation_max_shift_px_ =
      static_cast<int>(declare_parameter<int>("visual_rotation_max_shift_px", 12));
    visual_rotation_min_pixels_ =
      static_cast<int>(declare_parameter<int>("visual_rotation_min_pixels", 2000));
    visual_rotation_max_pixels_ =
      static_cast<int>(declare_parameter<int>("visual_rotation_max_pixels", 20000));
    visual_rotation_frame_stride_ =
      static_cast<int>(declare_parameter<int>("visual_rotation_frame_stride", 3));
    visual_rotation_max_dt_ns_ =
      declare_parameter<int64_t>("visual_rotation_max_dt_ns", 200000000LL);
    visual_rotation_max_rmse_ =
      declare_parameter<double>("visual_rotation_max_rmse", 0.30);
    visual_rotation_pixel_to_rad_scale_ =
      declare_parameter<double>("visual_rotation_pixel_to_rad_scale", 1.0);
    visual_rotation_sign_ =
      declare_parameter<double>("visual_rotation_sign", 1.0);
    enable_visual_se3_prior_ =
      declare_parameter<bool>("enable_visual_se3_prior", false);
    visual_se3_position_weight_ =
      declare_parameter<double>("visual_se3_position_weight", 0.0);
    visual_se3_orientation_weight_ =
      declare_parameter<double>("visual_se3_orientation_weight", 0.0);
    visual_se3_velocity_weight_ =
      declare_parameter<double>("visual_se3_velocity_weight", 0.0);
    visual_se3_huber_delta_m_ =
      declare_parameter<double>("visual_se3_huber_delta_m", 0.05);
    visual_se3_huber_delta_rad_ =
      declare_parameter<double>("visual_se3_huber_delta_rad", 0.05);
    visual_se3_huber_delta_mps_ =
      declare_parameter<double>("visual_se3_huber_delta_mps", 0.10);
    visual_se3_max_samples_ =
      static_cast<int>(declare_parameter<int>("visual_se3_max_samples", 1000));
    visual_se3_min_samples_ =
      static_cast<int>(declare_parameter<int>("visual_se3_min_samples", 32));
    visual_se3_min_gradient_ =
      declare_parameter<double>("visual_se3_min_gradient", 1.0e-4);
    visual_se3_max_abs_residual_ =
      declare_parameter<double>("visual_se3_max_abs_residual", 0.5);
    visual_se3_huber_delta_intensity_ =
      declare_parameter<double>("visual_se3_huber_delta_intensity", 0.15);
    visual_se3_min_depth_m_ =
      declare_parameter<double>("visual_se3_min_depth_m", 0.05);
    visual_se3_max_depth_m_ =
      declare_parameter<double>("visual_se3_max_depth_m", 80.0);
    visual_se3_depth_dilation_px_ =
      static_cast<int>(declare_parameter<int>("visual_se3_depth_dilation_px", 2));
    visual_se3_depth_cache_size_ =
      static_cast<int>(declare_parameter<int>("visual_se3_depth_cache_size", 8));
    visual_se3_max_dt_ns_ =
      declare_parameter<int64_t>("visual_se3_max_dt_ns", 100000000LL);
    visual_se3_min_hessian_rank_ =
      static_cast<int>(declare_parameter<int>("visual_se3_min_hessian_rank", 4));
    visual_se3_max_hessian_condition_ =
      declare_parameter<double>("visual_se3_max_hessian_condition", 1.0e12);
    visual_se3_min_sample_inlier_ratio_ =
      declare_parameter<double>("visual_se3_min_sample_inlier_ratio", 0.20);
    visual_se3_coverage_grid_cols_ =
      static_cast<int>(declare_parameter<int>("visual_se3_coverage_grid_cols", 4));
    visual_se3_coverage_grid_rows_ =
      static_cast<int>(declare_parameter<int>("visual_se3_coverage_grid_rows", 4));
    visual_se3_min_coverage_tiles_ =
      static_cast<int>(declare_parameter<int>("visual_se3_min_coverage_tiles", 4));
    visual_se3_max_mean_abs_residual_ =
      declare_parameter<double>("visual_se3_max_mean_abs_residual", 0.0);
    visual_se3_max_translation_step_m_ =
      declare_parameter<double>("visual_se3_max_translation_step_m", 0.25);
    visual_se3_max_rotation_step_rad_ =
      declare_parameter<double>("visual_se3_max_rotation_step_rad", 0.15);
    visual_se3_delta_sign_ =
      declare_parameter<double>("visual_se3_delta_sign", 1.0);
    if (!std::isfinite(visual_rotation_prior_weight_) ||
      visual_rotation_prior_weight_ < 0.0 ||
      !std::isfinite(visual_rotation_prior_huber_delta_rad_) ||
      visual_rotation_prior_huber_delta_rad_ < 0.0 ||
      visual_rotation_max_shift_px_ < 0 ||
      visual_rotation_min_pixels_ < 0 ||
      visual_rotation_max_pixels_ <= 0 ||
      visual_rotation_frame_stride_ <= 0 ||
      visual_rotation_max_dt_ns_ < 0 ||
      !std::isfinite(visual_rotation_max_rmse_) ||
      visual_rotation_max_rmse_ < 0.0 ||
      !std::isfinite(visual_rotation_pixel_to_rad_scale_) ||
      visual_rotation_pixel_to_rad_scale_ < 0.0 ||
      !std::isfinite(visual_rotation_sign_))
    {
      throw std::runtime_error("visual rotation prior parameters are invalid");
    }
    if (!camera_to_imu_translation_.allFinite() ||
      !camera_to_imu_rotation_.coeffs().allFinite() ||
      camera_to_imu_rotation_.norm() <= 1.0e-9 ||
      !std::isfinite(visual_se3_position_weight_) ||
      visual_se3_position_weight_ < 0.0 ||
      !std::isfinite(visual_se3_orientation_weight_) ||
      visual_se3_orientation_weight_ < 0.0 ||
      !std::isfinite(visual_se3_velocity_weight_) ||
      visual_se3_velocity_weight_ < 0.0 ||
      !std::isfinite(visual_se3_huber_delta_m_) ||
      visual_se3_huber_delta_m_ < 0.0 ||
      !std::isfinite(visual_se3_huber_delta_rad_) ||
      visual_se3_huber_delta_rad_ < 0.0 ||
      !std::isfinite(visual_se3_huber_delta_mps_) ||
      visual_se3_huber_delta_mps_ < 0.0 ||
      visual_se3_max_samples_ <= 0 ||
      visual_se3_min_samples_ <= 0 ||
      visual_se3_max_samples_ < visual_se3_min_samples_ ||
      !std::isfinite(visual_se3_min_gradient_) ||
      visual_se3_min_gradient_ < 0.0 ||
      !std::isfinite(visual_se3_max_abs_residual_) ||
      visual_se3_max_abs_residual_ < 0.0 ||
      !std::isfinite(visual_se3_huber_delta_intensity_) ||
      visual_se3_huber_delta_intensity_ < 0.0 ||
      !std::isfinite(visual_se3_min_depth_m_) ||
      !std::isfinite(visual_se3_max_depth_m_) ||
      visual_se3_min_depth_m_ <= 0.0 ||
      visual_se3_max_depth_m_ <= visual_se3_min_depth_m_ ||
      visual_se3_depth_dilation_px_ < 0 ||
      visual_se3_depth_cache_size_ <= 0 ||
      visual_se3_max_dt_ns_ < 0 ||
      visual_se3_min_hessian_rank_ < 0 ||
      visual_se3_min_hessian_rank_ > 6 ||
      !std::isfinite(visual_se3_max_hessian_condition_) ||
      visual_se3_max_hessian_condition_ < 0.0 ||
      !std::isfinite(visual_se3_min_sample_inlier_ratio_) ||
      visual_se3_min_sample_inlier_ratio_ < 0.0 ||
      visual_se3_min_sample_inlier_ratio_ > 1.0 ||
      visual_se3_coverage_grid_cols_ <= 0 ||
      visual_se3_coverage_grid_rows_ <= 0 ||
      visual_se3_min_coverage_tiles_ <= 0 ||
      visual_se3_min_coverage_tiles_ >
      visual_se3_coverage_grid_cols_ * visual_se3_coverage_grid_rows_ ||
      !std::isfinite(visual_se3_max_mean_abs_residual_) ||
      visual_se3_max_mean_abs_residual_ < 0.0 ||
      !std::isfinite(visual_se3_max_translation_step_m_) ||
      visual_se3_max_translation_step_m_ < 0.0 ||
      !std::isfinite(visual_se3_max_rotation_step_rad_) ||
      visual_se3_max_rotation_step_rad_ < 0.0 ||
      !std::isfinite(visual_se3_delta_sign_))
    {
      throw std::runtime_error("visual SE3 prior parameters are invalid");
    }
    visual_factor_.set_max_pixels(static_cast<size_t>(visual_rotation_max_pixels_));

    output_tum_path_ = declare_parameter<std::string>("output_tum_path", "");
    if (!output_tum_path_.empty()) {
      output_tum_stream_.open(output_tum_path_, std::ios::out | std::ios::trunc);
      if (!output_tum_stream_) {
        RCLCPP_WARN(
          get_logger(),
          "could not open output_tum_path '%s' for writing",
          output_tum_path_.c_str());
      } else {
        output_tum_stream_ << "# stamp_s tx ty tz qx qy qz qw" << std::endl;
        output_tum_stream_.flush();
      }
    }

    raw_pointcloud_topic_ = declare_parameter<std::string>(
      "raw_pointcloud_topic", "/points_for_gs");
    pointcloud_enable_ =
      declare_parameter<bool>("pointcloud_enable", true);
    pointcloud_subsample_stride_ =
      static_cast<int>(declare_parameter<int>("pointcloud_subsample_stride", 50));
    pointcloud_max_points_per_msg_ =
      static_cast<int>(declare_parameter<int>("pointcloud_max_points_per_msg", 256));
    pointcloud_wait_queue_max_size_ =
      static_cast<int>(declare_parameter<int>("pointcloud_wait_queue_max_size", 100));
    if (pointcloud_wait_queue_max_size_ < 0) {
      throw std::runtime_error("pointcloud_wait_queue_max_size must be >= 0");
    }
    pointcloud_min_range_m_ =
      declare_parameter<double>("pointcloud_min_range_m", 0.3);
    pointcloud_max_range_m_ =
      declare_parameter<double>("pointcloud_max_range_m", 30.0);
    pointcloud_factor_weight_ =
      declare_parameter<double>("pointcloud_factor_weight", 0.1);
    pointcloud_use_lidar_scale_ =
      declare_parameter<bool>("pointcloud_use_lidar_scale", true);
    pointcloud_min_lidar_scale_ =
      declare_parameter<double>("pointcloud_min_lidar_scale", 0.1);
    if (!std::isfinite(pointcloud_factor_weight_) || pointcloud_factor_weight_ <= 0.0 ||
      !std::isfinite(pointcloud_min_lidar_scale_) ||
      pointcloud_min_lidar_scale_ < 0.0 ||
      pointcloud_min_lidar_scale_ > 1.0)
    {
      throw std::runtime_error("pointcloud factor weight/scale parameters are invalid");
    }
    enable_lidar_point_deskew_ =
      declare_parameter<bool>("enable_lidar_point_deskew", false);
    lidar_max_abs_point_time_offset_s_ =
      declare_parameter<double>("lidar_max_abs_point_time_offset_s", 0.25);
    lidar_max_deskew_delta_m_ =
      declare_parameter<double>("lidar_max_deskew_delta_m", 1.0);
    if (!std::isfinite(lidar_max_abs_point_time_offset_s_) ||
      lidar_max_abs_point_time_offset_s_ < 0.0)
    {
      throw std::runtime_error("lidar_max_abs_point_time_offset_s must be finite and non-negative");
    }
    if (!std::isfinite(lidar_max_deskew_delta_m_) || lidar_max_deskew_delta_m_ < 0.0) {
      throw std::runtime_error("lidar_max_deskew_delta_m must be finite and non-negative");
    }
    enable_lidar_pose_prior_factor_ =
      declare_parameter<bool>("enable_lidar_pose_prior_factor", false);
    lidar_pose_prior_position_weight_ =
      declare_parameter<double>("lidar_pose_prior_position_weight", 1.0);
    lidar_pose_prior_velocity_weight_ =
      declare_parameter<double>("lidar_pose_prior_velocity_weight", 0.0);
    lidar_pose_prior_acceleration_weight_ =
      declare_parameter<double>("lidar_pose_prior_acceleration_weight", 0.0);
    lidar_pose_prior_angular_velocity_weight_ =
      declare_parameter<double>("lidar_pose_prior_angular_velocity_weight", 0.0);
    lidar_pose_prior_orientation_weight_ =
      declare_parameter<double>("lidar_pose_prior_orientation_weight", 1.0);
    lidar_pose_prior_position_huber_delta_m_ =
      declare_parameter<double>("lidar_pose_prior_position_huber_delta_m", 0.25);
    lidar_pose_prior_velocity_huber_delta_mps_ =
      declare_parameter<double>("lidar_pose_prior_velocity_huber_delta_mps", 0.25);
    lidar_pose_prior_acceleration_huber_delta_mps2_ =
      declare_parameter<double>("lidar_pose_prior_acceleration_huber_delta_mps2", 0.50);
    lidar_pose_prior_angular_velocity_huber_delta_radps_ =
      declare_parameter<double>("lidar_pose_prior_angular_velocity_huber_delta_radps", 0.25);
    lidar_pose_prior_orientation_huber_delta_rad_ =
      declare_parameter<double>("lidar_pose_prior_orientation_huber_delta_rad", 0.25);
    lidar_pose_factor_keyframe_stride_ =
      static_cast<int>(declare_parameter<int>("lidar_pose_factor_keyframe_stride", 5));
    lidar_pose_factor_iterations_ =
      static_cast<int>(declare_parameter<int>("lidar_pose_factor_iterations", 1));
    enable_lidar_scan_to_scan_prior_ =
      declare_parameter<bool>("enable_lidar_scan_to_scan_prior", false);
    lidar_scan_to_scan_velocity_weight_ =
      declare_parameter<double>("lidar_scan_to_scan_velocity_weight", 0.0);
    lidar_scan_to_scan_acceleration_weight_ =
      declare_parameter<double>("lidar_scan_to_scan_acceleration_weight", 0.0);
    lidar_scan_to_scan_angular_velocity_weight_ =
      declare_parameter<double>("lidar_scan_to_scan_angular_velocity_weight", 0.0);
    lidar_scan_to_scan_position_weight_ =
      declare_parameter<double>("lidar_scan_to_scan_position_weight", 0.0);
    lidar_scan_to_scan_orientation_weight_ =
      declare_parameter<double>("lidar_scan_to_scan_orientation_weight", 0.0);
    lidar_scan_to_scan_velocity_huber_delta_mps_ =
      declare_parameter<double>("lidar_scan_to_scan_velocity_huber_delta_mps", 0.25);
    lidar_scan_to_scan_acceleration_huber_delta_mps2_ =
      declare_parameter<double>("lidar_scan_to_scan_acceleration_huber_delta_mps2", 0.50);
    lidar_scan_to_scan_angular_velocity_huber_delta_radps_ =
      declare_parameter<double>("lidar_scan_to_scan_angular_velocity_huber_delta_radps", 0.25);
    lidar_scan_to_scan_max_velocity_mps_ =
      declare_parameter<double>("lidar_scan_to_scan_max_velocity_mps", 0.0);
    lidar_scan_to_scan_max_acceleration_mps2_ =
      declare_parameter<double>("lidar_scan_to_scan_max_acceleration_mps2", 0.0);
    lidar_scan_to_scan_max_angular_velocity_radps_ =
      declare_parameter<double>("lidar_scan_to_scan_max_angular_velocity_radps", 0.0);
    lidar_scan_to_scan_relative_translation_gain_ =
      declare_parameter<double>("lidar_scan_to_scan_relative_translation_gain", 1.0);
    lidar_scan_to_scan_position_huber_delta_m_ =
      declare_parameter<double>("lidar_scan_to_scan_position_huber_delta_m", 0.25);
    lidar_scan_to_scan_orientation_huber_delta_rad_ =
      declare_parameter<double>("lidar_scan_to_scan_orientation_huber_delta_rad", 0.25);
    lidar_scan_to_scan_use_odometry_prediction_ =
      declare_parameter<bool>("lidar_scan_to_scan_use_odometry_prediction", false);
    lidar_scan_to_scan_use_point_to_plane_correction_ =
      declare_parameter<bool>("lidar_scan_to_scan_use_point_to_plane_correction", false);
    lidar_scan_to_scan_use_relative_pose_factor_ =
      declare_parameter<bool>("lidar_scan_to_scan_use_relative_pose_factor", false);
    lidar_scan_to_scan_min_target_prediction_ratio_ =
      declare_parameter<double>("lidar_scan_to_scan_min_target_prediction_ratio", 0.0);
    lidar_scan_to_scan_min_target_translation_m_ =
      declare_parameter<double>("lidar_scan_to_scan_min_target_translation_m", 0.0);
    lidar_scan_to_scan_use_prediction_on_small_target_ =
      declare_parameter<bool>("lidar_scan_to_scan_use_prediction_on_small_target", false);
    lidar_scan_to_scan_skip_translation_priors_on_small_target_ =
      declare_parameter<bool>("lidar_scan_to_scan_skip_translation_priors_on_small_target", false);
    lidar_scan_to_scan_dead_reckon_on_reject_ =
      declare_parameter<bool>("lidar_scan_to_scan_dead_reckon_on_reject", false);
    lidar_scan_to_scan_apply_pose_seed_ =
      declare_parameter<bool>("lidar_scan_to_scan_apply_pose_seed", false);
    lidar_scan_to_scan_store_corrected_pose_ =
      declare_parameter<bool>("lidar_scan_to_scan_store_corrected_pose", true);
    lidar_scan_to_scan_yaw_only_angular_velocity_ =
      declare_parameter<bool>("lidar_scan_to_scan_yaw_only_angular_velocity", false);
    lidar_scan_to_scan_pose_seed_position_gain_ =
      declare_parameter<double>("lidar_scan_to_scan_pose_seed_position_gain", 1.0);
    lidar_scan_to_scan_pose_seed_rotation_gain_ =
      declare_parameter<double>("lidar_scan_to_scan_pose_seed_rotation_gain", 1.0);
    if (!std::isfinite(lidar_pose_prior_position_weight_) ||
      lidar_pose_prior_position_weight_ < 0.0 ||
      !std::isfinite(lidar_pose_prior_velocity_weight_) ||
      lidar_pose_prior_velocity_weight_ < 0.0 ||
      !std::isfinite(lidar_pose_prior_acceleration_weight_) ||
      lidar_pose_prior_acceleration_weight_ < 0.0 ||
      !std::isfinite(lidar_pose_prior_angular_velocity_weight_) ||
      lidar_pose_prior_angular_velocity_weight_ < 0.0 ||
      !std::isfinite(lidar_pose_prior_orientation_weight_) ||
      lidar_pose_prior_orientation_weight_ < 0.0 ||
      !std::isfinite(lidar_pose_prior_position_huber_delta_m_) ||
      lidar_pose_prior_position_huber_delta_m_ < 0.0 ||
      !std::isfinite(lidar_pose_prior_velocity_huber_delta_mps_) ||
      lidar_pose_prior_velocity_huber_delta_mps_ < 0.0 ||
      !std::isfinite(lidar_pose_prior_acceleration_huber_delta_mps2_) ||
      lidar_pose_prior_acceleration_huber_delta_mps2_ < 0.0 ||
      !std::isfinite(lidar_pose_prior_angular_velocity_huber_delta_radps_) ||
      lidar_pose_prior_angular_velocity_huber_delta_radps_ < 0.0 ||
      !std::isfinite(lidar_pose_prior_orientation_huber_delta_rad_) ||
      lidar_pose_prior_orientation_huber_delta_rad_ < 0.0 ||
      !std::isfinite(lidar_scan_to_scan_velocity_weight_) ||
      lidar_scan_to_scan_velocity_weight_ < 0.0 ||
      !std::isfinite(lidar_scan_to_scan_acceleration_weight_) ||
      lidar_scan_to_scan_acceleration_weight_ < 0.0 ||
      !std::isfinite(lidar_scan_to_scan_angular_velocity_weight_) ||
      lidar_scan_to_scan_angular_velocity_weight_ < 0.0 ||
      !std::isfinite(lidar_scan_to_scan_position_weight_) ||
      lidar_scan_to_scan_position_weight_ < 0.0 ||
      !std::isfinite(lidar_scan_to_scan_orientation_weight_) ||
      lidar_scan_to_scan_orientation_weight_ < 0.0 ||
      !std::isfinite(lidar_scan_to_scan_velocity_huber_delta_mps_) ||
      lidar_scan_to_scan_velocity_huber_delta_mps_ < 0.0 ||
      !std::isfinite(lidar_scan_to_scan_acceleration_huber_delta_mps2_) ||
      lidar_scan_to_scan_acceleration_huber_delta_mps2_ < 0.0 ||
      !std::isfinite(lidar_scan_to_scan_angular_velocity_huber_delta_radps_) ||
      lidar_scan_to_scan_angular_velocity_huber_delta_radps_ < 0.0 ||
      !std::isfinite(lidar_scan_to_scan_max_velocity_mps_) ||
      lidar_scan_to_scan_max_velocity_mps_ < 0.0 ||
      !std::isfinite(lidar_scan_to_scan_max_acceleration_mps2_) ||
      lidar_scan_to_scan_max_acceleration_mps2_ < 0.0 ||
      !std::isfinite(lidar_scan_to_scan_max_angular_velocity_radps_) ||
      lidar_scan_to_scan_max_angular_velocity_radps_ < 0.0 ||
      !std::isfinite(lidar_scan_to_scan_relative_translation_gain_) ||
      lidar_scan_to_scan_relative_translation_gain_ < 0.0 ||
      !std::isfinite(lidar_scan_to_scan_min_target_prediction_ratio_) ||
      lidar_scan_to_scan_min_target_prediction_ratio_ < 0.0 ||
      lidar_scan_to_scan_min_target_prediction_ratio_ > 1.0 ||
      !std::isfinite(lidar_scan_to_scan_min_target_translation_m_) ||
      lidar_scan_to_scan_min_target_translation_m_ < 0.0 ||
      !std::isfinite(lidar_scan_to_scan_position_huber_delta_m_) ||
      lidar_scan_to_scan_position_huber_delta_m_ < 0.0 ||
      !std::isfinite(lidar_scan_to_scan_orientation_huber_delta_rad_) ||
      lidar_scan_to_scan_orientation_huber_delta_rad_ < 0.0 ||
      !std::isfinite(lidar_scan_to_scan_pose_seed_position_gain_) ||
      lidar_scan_to_scan_pose_seed_position_gain_ < 0.0 ||
      lidar_scan_to_scan_pose_seed_position_gain_ > 1.0 ||
      !std::isfinite(lidar_scan_to_scan_pose_seed_rotation_gain_) ||
      lidar_scan_to_scan_pose_seed_rotation_gain_ < 0.0 ||
      lidar_scan_to_scan_pose_seed_rotation_gain_ > 1.0 ||
      lidar_pose_factor_keyframe_stride_ <= 0 ||
      lidar_pose_factor_iterations_ <= 0)
    {
      throw std::runtime_error("LiDAR pose-prior factor parameters are invalid");
    }
    const int lidar_pose_min_points =
      declare_parameter<int>("lidar_pose_factor_min_points", 32);
    const int lidar_pose_max_frame_points =
      declare_parameter<int>("lidar_pose_factor_max_frame_points", 2000);
    const int lidar_pose_max_map_points =
      declare_parameter<int>("lidar_pose_factor_max_map_points", 20000);
    if (lidar_pose_min_points <= 0 ||
      lidar_pose_max_frame_points < 0 ||
      lidar_pose_max_map_points < 0)
    {
      throw std::runtime_error("LiDAR pose-prior map sizes are invalid");
    }
    LidarFactorConfig lidar_pose_config;
    lidar_pose_config.min_points = static_cast<size_t>(lidar_pose_min_points);
    lidar_pose_config.max_frame_points = static_cast<size_t>(lidar_pose_max_frame_points);
    lidar_pose_config.max_map_points = static_cast<size_t>(lidar_pose_max_map_points);
    lidar_pose_config.pose_iterations = static_cast<size_t>(lidar_pose_factor_iterations_);
    lidar_pose_config.nearest_distance_m =
      declare_parameter<double>("lidar_pose_factor_nearest_distance_m", 0.35);
    lidar_pose_config.correction_gain =
      declare_parameter<double>("lidar_pose_factor_correction_gain", 0.7);
    lidar_pose_config.max_correction_m =
      declare_parameter<double>("lidar_pose_factor_max_correction_m", 0.25);
    lidar_pose_config.max_rotation_rad =
      declare_parameter<double>("lidar_pose_factor_max_rotation_rad", 0.08);
    lidar_pose_config.robust_kernel_m =
      declare_parameter<double>("lidar_pose_factor_robust_kernel_m", 0.15);
    lidar_pose_factor_.set_config(lidar_pose_config);
    enable_lidar_plane_normal_factor_ =
      declare_parameter<bool>("enable_lidar_plane_normal_factor", false);
    lidar_plane_normal_factor_weight_ =
      declare_parameter<double>("lidar_plane_normal_factor_weight", 0.1);
    lidar_plane_normal_huber_delta_rad_ =
      declare_parameter<double>("lidar_plane_normal_huber_delta_rad", 0.10);
    if (!std::isfinite(lidar_plane_normal_factor_weight_) ||
      lidar_plane_normal_factor_weight_ <= 0.0 ||
      !std::isfinite(lidar_plane_normal_huber_delta_rad_) ||
      lidar_plane_normal_huber_delta_rad_ < 0.0)
    {
      throw std::runtime_error("LiDAR plane-normal factor parameters are invalid");
    }

    enable_imu_gravity_autocal_ =
      declare_parameter<bool>("enable_imu_gravity_autocal", true);
    // Default off for launches: voxel-plane factors are useful only when the
    // persistent world-frame plane map has enough observations to avoid the
    // old same-frame identity constraint. Parity scripts enable this path
    // explicitly while the tracker is still being RMSE-tuned.
    enable_voxel_plane_extraction_ =
      declare_parameter<bool>("enable_voxel_plane_extraction", false);
    enable_persistent_plane_map_ =
      declare_parameter<bool>("enable_persistent_plane_map", true);
    enable_persistent_point_map_ =
      declare_parameter<bool>("enable_persistent_point_map", false);
    enable_gaussian_snapshot_lidar_factor_ =
      declare_parameter<bool>("enable_gaussian_snapshot_lidar_factor", false);
    persistent_map_update_requires_accepted_solve_ =
      declare_parameter<bool>("persistent_map_update_requires_accepted_solve", false);
    defer_persistent_plane_map_updates_until_solved_ =
      declare_parameter<bool>("defer_persistent_plane_map_updates_until_solved", false);
    deferred_plane_map_update_max_queue_ = static_cast<int>(
      declare_parameter<int>("deferred_plane_map_update_max_queue", 200));
    if (deferred_plane_map_update_max_queue_ < 0) {
      throw std::runtime_error("deferred_plane_map_update_max_queue must be >= 0");
    }
    persistent_point_map_nearest_distance_m_ =
      declare_parameter<double>("persistent_point_map_nearest_distance_m", 0.35);
    persistent_point_map_merge_distance_m_ =
      declare_parameter<double>("persistent_point_map_merge_distance_m", 0.10);
    persistent_point_map_min_match_age_s_ =
      declare_parameter<double>("persistent_point_map_min_match_age_s", 0.25);
    persistent_point_map_factor_weight_ =
      declare_parameter<double>("persistent_point_map_factor_weight", 0.05);
    persistent_point_map_subsample_stride_ = static_cast<int>(
      declare_parameter<int>("persistent_point_map_subsample_stride", 20));
    persistent_point_map_max_points_ = static_cast<int>(
      declare_parameter<int>("persistent_point_map_max_points", 20000));
    persistent_point_map_max_correspondences_ = static_cast<int>(
      declare_parameter<int>("persistent_point_map_max_correspondences", 64));
    persistent_point_map_min_observations_for_match_ = static_cast<int>(
      declare_parameter<int>("persistent_point_map_min_observations_for_match", 3));
    gaussian_snapshot_lidar_factor_weight_ =
      declare_parameter<double>("gaussian_snapshot_lidar_factor_weight", 0.05);
    gaussian_snapshot_lidar_nearest_distance_m_ =
      declare_parameter<double>("gaussian_snapshot_lidar_nearest_distance_m", 0.35);
    gaussian_snapshot_lidar_min_opacity_ =
      declare_parameter<double>("gaussian_snapshot_lidar_min_opacity", 0.01);
    gaussian_snapshot_lidar_subsample_stride_ = static_cast<int>(
      declare_parameter<int>("gaussian_snapshot_lidar_subsample_stride", 20));
    gaussian_snapshot_map_subsample_stride_ = static_cast<int>(
      declare_parameter<int>("gaussian_snapshot_map_subsample_stride", 1));
    gaussian_snapshot_lidar_max_correspondences_ = static_cast<int>(
      declare_parameter<int>("gaussian_snapshot_lidar_max_correspondences", 64));
    if (!std::isfinite(persistent_point_map_nearest_distance_m_) ||
      persistent_point_map_nearest_distance_m_ <= 0.0 ||
      !std::isfinite(persistent_point_map_merge_distance_m_) ||
      persistent_point_map_merge_distance_m_ < 0.0 ||
      !std::isfinite(persistent_point_map_min_match_age_s_) ||
      persistent_point_map_min_match_age_s_ < 0.0 ||
      !std::isfinite(persistent_point_map_factor_weight_) ||
      persistent_point_map_factor_weight_ <= 0.0 ||
      persistent_point_map_subsample_stride_ < 1 ||
      persistent_point_map_max_correspondences_ < 0 ||
      persistent_point_map_min_observations_for_match_ < 1)
    {
      throw std::runtime_error("Persistent point-map parameters are invalid");
    }
    if (!std::isfinite(gaussian_snapshot_lidar_factor_weight_) ||
      gaussian_snapshot_lidar_factor_weight_ <= 0.0 ||
      !std::isfinite(gaussian_snapshot_lidar_nearest_distance_m_) ||
      gaussian_snapshot_lidar_nearest_distance_m_ <= 0.0 ||
      !std::isfinite(gaussian_snapshot_lidar_min_opacity_) ||
      gaussian_snapshot_lidar_min_opacity_ < 0.0 ||
      gaussian_snapshot_lidar_subsample_stride_ < 1 ||
      gaussian_snapshot_map_subsample_stride_ < 1 ||
      gaussian_snapshot_qos_depth_ < 1 ||
      gaussian_snapshot_lidar_max_correspondences_ < 0)
    {
      throw std::runtime_error("Gaussian snapshot LiDAR factor parameters are invalid");
    }
    spline::LidarPlaneExtractorOptions extractor_options;
    extractor_options.voxel_size_m =
      declare_parameter<double>("voxel_plane_size_m", 0.5);
    extractor_options.min_points_per_voxel = static_cast<int>(
      declare_parameter<int>("voxel_plane_min_points", 8));
    extractor_options.planar_eigenvalue_ratio =
      declare_parameter<double>("voxel_plane_eigen_ratio", 0.05);
    extractor_options.max_inlier_distance_m =
      declare_parameter<double>("voxel_plane_max_inlier_m", 0.1);
    extractor_options.max_correspondences = static_cast<int>(
      declare_parameter<int>("voxel_plane_max_correspondences", 64));
    extractor_options.min_range_m = pointcloud_min_range_m_;
    extractor_options.max_range_m = pointcloud_max_range_m_;
    plane_extractor_.set_options(extractor_options);
    spline::PersistentPlaneMapOptions plane_map_options;
    plane_map_options.max_planes = static_cast<int>(
      declare_parameter<int>("persistent_plane_map_max_planes", 512));
    plane_map_options.max_point_to_plane_distance_m =
      declare_parameter<double>("persistent_plane_map_match_distance_m", 0.25);
    plane_map_options.min_normal_dot =
      declare_parameter<double>("persistent_plane_map_min_normal_dot", 0.95);
    plane_map_options.min_observations_for_match = static_cast<int>(
      declare_parameter<int>("persistent_plane_map_min_observations_for_match", 3));
    persistent_plane_map_.set_options(plane_map_options);

    const auto plane_param = declare_parameter<std::vector<double>>(
      "lidar_ground_plane", std::vector<double>{0.0, 0.0, 1.0, 0.0});
    if (plane_param.size() == 4) {
      lidar_plane_ << plane_param[0], plane_param[1], plane_param[2], plane_param[3];
      const double n_norm = lidar_plane_.head<3>().norm();
      if (n_norm > 1.0e-9) {
        lidar_plane_ /= n_norm;
      }
    }

    const auto extrinsic_translation = declare_parameter<std::vector<double>>(
      "lidar_to_imu_translation", std::vector<double>{0.0, 0.0, 0.0});
    if (extrinsic_translation.size() == 3) {
      lidar_to_imu_translation_ = Eigen::Vector3d(
        extrinsic_translation[0],
        extrinsic_translation[1],
        extrinsic_translation[2]);
    }
    const auto extrinsic_rotation_xyzw = declare_parameter<std::vector<double>>(
      "lidar_to_imu_rotation_xyzw", std::vector<double>{0.0, 0.0, 0.0, 1.0});
    if (extrinsic_rotation_xyzw.size() == 4) {
      lidar_to_imu_rotation_ = Eigen::Quaterniond(
        extrinsic_rotation_xyzw[3],
        extrinsic_rotation_xyzw[0],
        extrinsic_rotation_xyzw[1],
        extrinsic_rotation_xyzw[2]).normalized();
    }

    estimator_ =
      std::make_unique<spline::ContinuousTimeSlidingWindowEstimator>(options);

    rclcpp::QoS imu_qos(rclcpp::KeepLast(200));
    imu_qos.best_effort();
    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      raw_imu_topic_, imu_qos,
      std::bind(&ContinuousTimeNode::on_imu, this, std::placeholders::_1));

    if (pointcloud_enable_) {
      rclcpp::QoS pc_qos(rclcpp::KeepLast(20));
      pc_qos.best_effort();
      pointcloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        raw_pointcloud_topic_, pc_qos,
        std::bind(&ContinuousTimeNode::on_pointcloud, this, std::placeholders::_1));
    }

    if (enable_visual_rotation_prior_ || enable_visual_se3_prior_) {
      rclcpp::QoS camera_qos(rclcpp::KeepLast(20));
      camera_qos.best_effort();
      camera_info_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        raw_camera_info_topic_, camera_qos,
        std::bind(&ContinuousTimeNode::on_camera_info, this, std::placeholders::_1));
      image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
        raw_image_topic_, camera_qos,
        std::bind(&ContinuousTimeNode::on_image, this, std::placeholders::_1));
    }

    if (enable_gaussian_snapshot_lidar_factor_) {
      rclcpp::QoS gaussian_qos(
        rclcpp::KeepLast(static_cast<size_t>(gaussian_snapshot_qos_depth_)));
      gaussian_qos.reliable();
      gaussian_qos.transient_local();
      gaussian_map_subscription_ =
        create_subscription<gaussian_lic_msgs::msg::GaussianArray>(
        gaussian_map_topic_, gaussian_qos,
        std::bind(&ContinuousTimeNode::on_gaussian_map, this, std::placeholders::_1));
    }

    if (enable_external_odometry_prior_ && !external_odometry_prior_topic_.empty()) {
      rclcpp::QoS prior_qos(rclcpp::KeepLast(20));
      prior_qos.reliable();
      external_odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
        external_odometry_prior_topic_, prior_qos,
        std::bind(&ContinuousTimeNode::on_external_odometry_prior, this, std::placeholders::_1));
    }

    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
      odometry_topic_, rclcpp::QoS(50));
    path_publisher_ = create_publisher<nav_msgs::msg::Path>(
      path_topic_, rclcpp::QoS(10));

    step_timer_ = create_wall_timer(
      std::chrono::duration<double>(step_period_seconds_),
      std::bind(&ContinuousTimeNode::on_step_timer, this));

    RCLCPP_INFO(
      get_logger(),
      "continuous_time_node ready (imu=%s, odom=%s, dt=%.3fs, window=%d knots, imu_accel_scale=%.5f)",
      raw_imu_topic_.c_str(), odometry_topic_.c_str(),
      options.dt_s, options.window_knot_count,
      imu_linear_acceleration_scale_);

    step_period_ns_ = static_cast<int64_t>(std::llround(step_period_seconds_ * 1.0e9));
    pose_output_period_ns_ =
      pose_output_period_seconds_ > 0.0 ?
      static_cast<int64_t>(std::llround(pose_output_period_seconds_ * 1.0e9)) : 0;
    if (pose_output_period_seconds_ > 0.0 && pose_output_period_ns_ <= 0) {
      throw std::runtime_error("pose_output_period_seconds rounds to a non-positive period");
    }
    knot_interval_ns_ =
      static_cast<int64_t>(std::llround(options.dt_s * 1.0e9));
  }

private:
  struct PendingPlaneMapUpdate
  {
    int64_t stamp_ns{0};
    std::vector<spline::ExtractedPlane> planes;
    spline::LidarExtrinsics extrinsics;
  };

  struct PersistentPointMapEntry
  {
    Eigen::Vector3d point_world{Eigen::Vector3d::Zero()};
    Eigen::Vector3d last_stamp_mean_world{Eigen::Vector3d::Zero()};
    int observations{1};
    int last_stamp_sample_count{1};
    int64_t first_stamp_ns{0};
    int64_t last_stamp_ns{0};
  };

  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    if (!msg) {
      return;
    }
    const int64_t stamp_ns = stamp_to_nanoseconds(msg->header.stamp);
    if (stamp_ns <= 0) {
      return;
    }
    if (!last_imu_stamp_valid_ || stamp_ns > last_imu_stamp_ns_) {
      last_imu_stamp_ns_ = stamp_ns;
      last_imu_stamp_valid_ = true;
    } else {
      // Non-monotonic stamp — drop to preserve estimator semantics.
      ++dropped_imu_count_;
      return;
    }

    spline::ImuSample sample;
    sample.gyro = Eigen::Vector3d(
      msg->angular_velocity.x,
      msg->angular_velocity.y,
      msg->angular_velocity.z);
    sample.accel = imu_linear_acceleration_scale_ * Eigen::Vector3d(
      msg->linear_acceleration.x,
      msg->linear_acceleration.y,
      msg->linear_acceleration.z);
    if (!sample.gyro.allFinite() || !sample.accel.allFinite()) {
      ++rejected_imu_count_;
      return;
    }

    std::lock_guard<std::mutex> lock(estimator_mutex_);
    if (!initialized_) {
      seed_imu_buffer_.push_back({stamp_ns, sample});
      if (static_cast<int>(seed_imu_buffer_.size()) >= seed_min_imu_count_) {
        seed_window_from_buffer();
        initialized_ = true;
      }
      return;
    }
    estimator_->add_imu_sample(stamp_ns, sample);
    ++accepted_imu_count_;
    maybe_run_stamp_driven_steps_locked(stamp_ns);
  }

  void on_external_odometry_prior(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!msg) {
      return;
    }
    const int64_t stamp_ns = stamp_to_nanoseconds(msg->header.stamp);
    if (stamp_ns <= 0) {
      ++rejected_prior_count_;
      return;
    }
    const auto & pose = msg->pose.pose;
    Eigen::Vector3d p(pose.position.x, pose.position.y, pose.position.z);
    Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x,
      pose.orientation.y, pose.orientation.z);
    if (!p.allFinite() || !q.coeffs().allFinite() ||
      q.norm() < 1.0e-6)
    {
      ++rejected_prior_count_;
      return;
    }
    std::lock_guard<std::mutex> lock(estimator_mutex_);
    if (initialized_) {
      if (enable_external_odometry_position_factors_) {
        add_external_position_prior_factor(stamp_ns, p);
      }
      if (enable_external_odometry_orientation_factors_) {
        add_external_orientation_prior_factor(
          stamp_ns, (q.normalized() * prior_to_imu_rotation_).normalized());
      }
      return;
    }
    latest_prior_position_ = p;
    latest_prior_orientation_ = q.normalized();
    have_prior_pose_ = true;
    ++accepted_prior_count_;
  }

  void add_external_position_prior_factor(
    int64_t stamp_ns,
    const Eigen::Vector3d & position_world)
  {
    if (!estimator_ || !position_world.allFinite()) {
      ++rejected_prior_count_;
      return;
    }
    estimator_->add_position_prior(
      stamp_ns, position_world,
      external_odometry_position_factor_weight_,
      external_odometry_position_factor_huber_delta_m_);
    ++accepted_prior_position_factor_messages_;
  }

  void add_external_orientation_prior_factor(
    int64_t stamp_ns,
    const Eigen::Quaterniond & q_world_body)
  {
    if (!estimator_ || !q_world_body.coeffs().allFinite() ||
      q_world_body.norm() <= 1.0e-9)
    {
      ++rejected_prior_count_;
      return;
    }
    estimator_->add_orientation_prior(
      stamp_ns, q_world_body.normalized(),
      external_odometry_orientation_factor_weight_,
      external_odometry_orientation_factor_huber_delta_rad_);
    ++accepted_prior_orientation_factor_messages_;
  }

  void on_camera_info(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    if (!msg || (!enable_visual_rotation_prior_ && !enable_visual_se3_prior_)) {
      return;
    }
    const double fx = msg->k[0];
    const double fy = msg->k[4];
    const double cx = msg->k[2];
    const double cy = msg->k[5];
    if (!std::isfinite(fx) || !std::isfinite(fy) ||
      !std::isfinite(cx) || !std::isfinite(cy) ||
      fx <= 0.0 || fy <= 0.0)
    {
      ++visual_rotation_rejected_frames_;
      return;
    }
    std::lock_guard<std::mutex> lock(estimator_mutex_);
    visual_intrinsics_.fx = fx;
    visual_intrinsics_.fy = fy;
    visual_intrinsics_.cx = cx;
    visual_intrinsics_.cy = cy;
    have_visual_intrinsics_ = true;
  }

  void on_image(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (!msg || (!enable_visual_rotation_prior_ && !enable_visual_se3_prior_)) {
      return;
    }
    VisualFrame frame;
    if (!decode_image_gray(*msg, frame) || frame.stamp_ns <= 0) {
      ++visual_rotation_rejected_frames_;
      return;
    }

    std::lock_guard<std::mutex> lock(estimator_mutex_);
    ++visual_rotation_image_frames_;
    if (!initialized_ || !estimator_ || !have_visual_intrinsics_) {
      last_visual_frame_ = std::move(frame);
      have_last_visual_frame_ = true;
      return;
    }
    if (!have_last_visual_frame_) {
      last_visual_frame_ = std::move(frame);
      have_last_visual_frame_ = true;
      return;
    }
    const auto visual_se3_status =
      maybe_add_visual_se3_prior_locked(last_visual_frame_, frame);
    if (visual_se3_status == VisualSe3PriorStatus::kDepthMiss) {
      pending_visual_se3_previous_frame_ = last_visual_frame_;
      pending_visual_se3_current_frame_ = frame;
      have_pending_visual_se3_pair_ = true;
    } else if (visual_se3_status != VisualSe3PriorStatus::kSkipped) {
      have_pending_visual_se3_pair_ = false;
    }
    if (!enable_visual_rotation_prior_) {
      last_visual_frame_ = std::move(frame);
      return;
    }
    if (
      visual_rotation_frame_stride_ > 1 &&
      visual_rotation_image_frames_ %
        static_cast<size_t>(visual_rotation_frame_stride_) != 0U)
    {
      last_visual_frame_ = std::move(frame);
      have_last_visual_frame_ = true;
      return;
    }
    const int64_t dt_ns = frame.stamp_ns - last_visual_frame_.stamp_ns;
    if (dt_ns <= 0 || (visual_rotation_max_dt_ns_ > 0 && dt_ns > visual_rotation_max_dt_ns_)) {
      ++visual_rotation_rejected_frames_;
      last_visual_frame_ = std::move(frame);
      return;
    }
    if (frame.width != last_visual_frame_.width || frame.height != last_visual_frame_.height) {
      ++visual_rotation_rejected_frames_;
      last_visual_frame_ = std::move(frame);
      have_visual_orientation_target_ = false;
      return;
    }

    const auto alignment =
      visual_factor_.estimate_translation(
      last_visual_frame_, frame, visual_rotation_max_shift_px_);
    if (!alignment.valid ||
      alignment.compared_pixels < static_cast<size_t>(visual_rotation_min_pixels_) ||
      (visual_rotation_max_rmse_ > 0.0 && alignment.rmse > visual_rotation_max_rmse_))
    {
      ++visual_rotation_rejected_frames_;
      last_visual_frame_ = std::move(frame);
      return;
    }

    if (!have_visual_orientation_target_) {
      Eigen::Quaterniond q_seed;
      Eigen::Vector3d p_seed;
      if (!estimator_->query_pose(last_visual_frame_.stamp_ns, q_seed, p_seed)) {
        ++visual_rotation_rejected_frames_;
        last_visual_frame_ = std::move(frame);
        return;
      }
      visual_orientation_target_ = q_seed.normalized();
      have_visual_orientation_target_ = true;
    }

    const double inv_fx = 1.0 / visual_intrinsics_.fx;
    const double inv_fy = 1.0 / visual_intrinsics_.fy;
    const Eigen::Vector3d rotation_camera =
      visual_rotation_sign_ * visual_rotation_pixel_to_rad_scale_ *
      Eigen::Vector3d(
      alignment.subpixel_dy * inv_fy,
      -alignment.subpixel_dx * inv_fx,
      0.0);
    if (!rotation_camera.allFinite()) {
      ++visual_rotation_rejected_frames_;
      last_visual_frame_ = std::move(frame);
      return;
    }
    const Eigen::Vector3d rotation_body = camera_to_imu_rotation_ * rotation_camera;
    const Eigen::Quaterniond delta_q =
      quaternion_from_rotation_vector(rotation_body);
    visual_orientation_target_ = (visual_orientation_target_ * delta_q).normalized();
    if (!visual_orientation_target_.coeffs().allFinite() ||
      visual_orientation_target_.norm() <= 1.0e-9)
    {
      have_visual_orientation_target_ = false;
      ++visual_rotation_rejected_frames_;
      last_visual_frame_ = std::move(frame);
      return;
    }

    if (visual_rotation_prior_weight_ > 0.0) {
      estimator_->add_orientation_prior(
        frame.stamp_ns, visual_orientation_target_,
        visual_rotation_prior_weight_,
        visual_rotation_prior_huber_delta_rad_);
      ++visual_rotation_prior_factors_;
    }
    last_visual_rotation_dx_px_ = alignment.subpixel_dx;
    last_visual_rotation_dy_px_ = alignment.subpixel_dy;
    last_visual_rotation_angle_rad_ = rotation_body.norm();
    last_visual_rotation_rmse_ = alignment.rmse;
    ++visual_rotation_accepted_frames_;
    last_visual_frame_ = std::move(frame);
  }

  void on_gaussian_map(const gaussian_lic_msgs::msg::GaussianArray::SharedPtr msg)
  {
    if (!msg || !enable_gaussian_snapshot_lidar_factor_) {
      return;
    }
    std::lock_guard<std::mutex> lock(estimator_mutex_);
    const int64_t previous_complete_stamp_ns =
      gaussian_snapshot_.complete() ? gaussian_snapshot_.stamp_ns() : 0;
    const bool accepted = gaussian_snapshot_.ingest(*msg);
    gaussian_snapshot_chunks_received_ = gaussian_snapshot_.received_chunk_count();
    gaussian_snapshot_expected_chunks_ = gaussian_snapshot_.expected_chunk_count();
    gaussian_snapshot_points_ = gaussian_snapshot_.point_count();
    if (
      accepted && gaussian_snapshot_.complete() &&
      gaussian_snapshot_.stamp_ns() != previous_complete_stamp_ns)
    {
      ++gaussian_snapshot_updates_;
    }
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "continuous-time Gaussian snapshot chunk %u/%u total=%u accepted=%s complete=%s cached=%zu mean_opacity=%.4f",
      msg->chunk_index + 1U,
      msg->chunk_count,
      msg->total_count,
      accepted ? "true" : "false",
      gaussian_snapshot_.complete() ? "true" : "false",
      gaussian_snapshot_.point_count(),
      gaussian_snapshot_.mean_opacity());
  }

  struct SparseDepthFrame
  {
    int64_t stamp_ns{0};
    size_t width{0};
    size_t height{0};
    std::vector<float> depth_m;
  };

  void cache_sparse_depth_frame_locked(
    int64_t stamp_ns,
    const std::vector<Eigen::Vector3d> & points_lidar,
    const spline::LidarExtrinsics & extrinsics)
  {
    if (!enable_visual_se3_prior_ || !have_visual_intrinsics_ ||
      last_visual_frame_.width == 0U || last_visual_frame_.height == 0U ||
      points_lidar.empty())
    {
      return;
    }
    SparseDepthFrame frame;
    frame.stamp_ns = stamp_ns;
    frame.width = last_visual_frame_.width;
    frame.height = last_visual_frame_.height;
    frame.depth_m.assign(frame.width * frame.height, std::numeric_limits<float>::quiet_NaN());
    const Eigen::Quaterniond q_imu_camera = camera_to_imu_rotation_.normalized();
    const Eigen::Quaterniond q_camera_imu = q_imu_camera.inverse();
    size_t projected = 0U;
    for (const auto & point_lidar : points_lidar) {
      if (!point_lidar.allFinite()) {
        continue;
      }
      const double range = point_lidar.norm();
      if (!std::isfinite(range) || range < pointcloud_min_range_m_ ||
        range > pointcloud_max_range_m_)
      {
        continue;
      }
      const Eigen::Vector3d point_imu =
        extrinsics.q_lidar_to_imu * point_lidar + extrinsics.p_lidar_in_imu;
      const Eigen::Vector3d point_camera =
        q_camera_imu * (point_imu - camera_to_imu_translation_);
      const double z = point_camera.z();
      if (!std::isfinite(z) || z < visual_se3_min_depth_m_ ||
        z > visual_se3_max_depth_m_)
      {
        continue;
      }
      const double u_f = visual_intrinsics_.fx * point_camera.x() / z + visual_intrinsics_.cx;
      const double v_f = visual_intrinsics_.fy * point_camera.y() / z + visual_intrinsics_.cy;
      if (!std::isfinite(u_f) || !std::isfinite(v_f)) {
        continue;
      }
      const auto u = static_cast<int64_t>(std::llround(u_f));
      const auto v = static_cast<int64_t>(std::llround(v_f));
      const int64_t dilation = static_cast<int64_t>(visual_se3_depth_dilation_px_);
      for (int64_t dy = -dilation; dy <= dilation; ++dy) {
        for (int64_t dx = -dilation; dx <= dilation; ++dx) {
          if (dx * dx + dy * dy > dilation * dilation) {
            continue;
          }
          const int64_t uu = u + dx;
          const int64_t vv = v + dy;
          if (uu < 0 || vv < 0 ||
            uu >= static_cast<int64_t>(frame.width) ||
            vv >= static_cast<int64_t>(frame.height))
          {
            continue;
          }
          const size_t index =
            static_cast<size_t>(vv) * frame.width + static_cast<size_t>(uu);
          const float depth = static_cast<float>(z);
          if (!std::isfinite(frame.depth_m[index]) || depth < frame.depth_m[index]) {
            if (!std::isfinite(frame.depth_m[index])) {
              ++projected;
            }
            frame.depth_m[index] = depth;
          }
        }
      }
    }
    if (projected < static_cast<size_t>(visual_se3_min_samples_)) {
      ++visual_se3_depth_rejected_frames_;
      return;
    }
    sparse_depth_cache_.push_back(std::move(frame));
    while (static_cast<int>(sparse_depth_cache_.size()) > visual_se3_depth_cache_size_) {
      sparse_depth_cache_.pop_front();
    }
    ++visual_se3_depth_frames_;
    retry_pending_visual_se3_prior_locked(stamp_ns);
  }

  void retry_pending_visual_se3_prior_locked(int64_t depth_stamp_ns)
  {
    if (!have_pending_visual_se3_pair_) {
      return;
    }
    const int64_t stamp_delta = pending_visual_se3_current_frame_.stamp_ns - depth_stamp_ns;
    const int64_t delta = stamp_delta >= 0 ? stamp_delta : -stamp_delta;
    if (visual_se3_max_dt_ns_ > 0 && delta > visual_se3_max_dt_ns_) {
      if (depth_stamp_ns > pending_visual_se3_current_frame_.stamp_ns) {
        have_pending_visual_se3_pair_ = false;
      }
      return;
    }
    const auto status = maybe_add_visual_se3_prior_locked(
      pending_visual_se3_previous_frame_, pending_visual_se3_current_frame_);
    if (status != VisualSe3PriorStatus::kDepthMiss) {
      have_pending_visual_se3_pair_ = false;
    }
  }

  const SparseDepthFrame * select_sparse_depth_frame_locked(
    int64_t stamp_ns,
    size_t width,
    size_t height,
    int64_t & match_delta_ns) const
  {
    const SparseDepthFrame * best = nullptr;
    int64_t best_delta = std::numeric_limits<int64_t>::max();
    for (const auto & frame : sparse_depth_cache_) {
      if (frame.width != width || frame.height != height) {
        continue;
      }
      const int64_t delta =
        frame.stamp_ns > stamp_ns ? frame.stamp_ns - stamp_ns : stamp_ns - frame.stamp_ns;
      if (visual_se3_max_dt_ns_ > 0 && delta > visual_se3_max_dt_ns_) {
        continue;
      }
      if (delta < best_delta) {
        best_delta = delta;
        best = &frame;
      }
    }
    match_delta_ns = best == nullptr ? 0 : best_delta;
    return best;
  }

  VisualSe3PriorStatus maybe_add_visual_se3_prior_locked(
    const VisualFrame & previous,
    const VisualFrame & current)
  {
    if (!enable_visual_se3_prior_ || !estimator_ ||
      !have_visual_intrinsics_ || current.width != previous.width ||
      current.height != previous.height)
    {
      return VisualSe3PriorStatus::kSkipped;
    }
    int64_t depth_delta_ns = 0;
    const SparseDepthFrame * depth_frame =
      select_sparse_depth_frame_locked(
      current.stamp_ns, current.width, current.height, depth_delta_ns);
    if (depth_frame == nullptr) {
      ++visual_se3_depth_miss_count_;
      return VisualSe3PriorStatus::kDepthMiss;
    }
    const size_t pixel_count = current.width * current.height;
    if (current.gray.size() != pixel_count ||
      previous.gray.size() != pixel_count ||
      depth_frame->depth_m.size() != pixel_count)
    {
      ++visual_se3_rejected_batches_;
      return VisualSe3PriorStatus::kRejected;
    }

    std::vector<VisualSe3PhotometricSample> samples;
    samples.reserve(static_cast<size_t>(visual_se3_max_samples_));
    size_t sampled_depth = 0U;
    size_t rejected_gradient = 0U;
    size_t rejected_residual = 0U;
    const size_t coverage_cols = static_cast<size_t>(visual_se3_coverage_grid_cols_);
    const size_t coverage_rows = static_cast<size_t>(visual_se3_coverage_grid_rows_);
    const size_t coverage_total_tiles = coverage_cols * coverage_rows;
    std::vector<bool> occupied_coverage_tiles(coverage_total_tiles, false);
    std::vector<std::vector<size_t>> valid_depth_tiles(coverage_total_tiles);
    const auto coverage_tile_for_pixel =
      [coverage_cols, coverage_rows, &current](const size_t x, const size_t y) {
        const size_t tile_x =
          std::min(coverage_cols - 1U, (x * coverage_cols) / current.width);
        const size_t tile_y =
          std::min(coverage_rows - 1U, (y * coverage_rows) / current.height);
        return tile_y * coverage_cols + tile_x;
      };
    size_t valid_depth_pixels = 0U;
    for (size_t index = 0; index < pixel_count; ++index) {
      const size_t x = index % current.width;
      const size_t y = index / current.width;
      if (x == 0U || y == 0U || x + 1U >= current.width || y + 1U >= current.height) {
        continue;
      }
      const float depth = depth_frame->depth_m[index];
      if (!std::isfinite(depth) ||
        static_cast<double>(depth) < visual_se3_min_depth_m_ ||
        static_cast<double>(depth) > visual_se3_max_depth_m_)
      {
        continue;
      }
      valid_depth_tiles[coverage_tile_for_pixel(x, y)].push_back(index);
      ++valid_depth_pixels;
    }
    if (valid_depth_pixels == 0U) {
      ++visual_se3_total_batches_;
      ++visual_se3_rejected_batches_;
      return VisualSe3PriorStatus::kRejected;
    }

    std::vector<size_t> tile_quotas(valid_depth_tiles.size(), 0U);
    size_t active_tiles = 0U;
    for (const auto & tile_indices : valid_depth_tiles) {
      if (!tile_indices.empty()) {
        ++active_tiles;
      }
    }
    size_t remaining_samples = std::min(
      valid_depth_pixels, static_cast<size_t>(visual_se3_max_samples_));
    while (remaining_samples > 0U && active_tiles > 0U) {
      const size_t share = std::max<size_t>(1U, remaining_samples / active_tiles);
      bool made_progress = false;
      for (size_t tile = 0; tile < valid_depth_tiles.size() && remaining_samples > 0U; ++tile) {
        const size_t tile_remaining = valid_depth_tiles[tile].size() - tile_quotas[tile];
        if (tile_remaining == 0U) {
          continue;
        }
        const size_t take = std::min(tile_remaining, share);
        tile_quotas[tile] += take;
        remaining_samples -= take;
        made_progress = true;
        if (tile_quotas[tile] == valid_depth_tiles[tile].size()) {
          --active_tiles;
        }
      }
      if (!made_progress) {
        break;
      }
    }
    std::vector<size_t> valid_depth_indices;
    valid_depth_indices.reserve(
      std::min(valid_depth_pixels, static_cast<size_t>(visual_se3_max_samples_)));
    for (size_t tile = 0; tile < valid_depth_tiles.size(); ++tile) {
      const auto & tile_indices = valid_depth_tiles[tile];
      const size_t quota = tile_quotas[tile];
      if (quota == 0U) {
        continue;
      }
      if (quota >= tile_indices.size()) {
        valid_depth_indices.insert(
          valid_depth_indices.end(), tile_indices.begin(), tile_indices.end());
        continue;
      }
      for (size_t i = 0; i < quota; ++i) {
        valid_depth_indices.push_back(tile_indices[(i * tile_indices.size()) / quota]);
      }
    }

    double abs_residual_sum = 0.0;
    for (const size_t index : valid_depth_indices) {
      const size_t x = index % current.width;
      const size_t y = index / current.width;
      const float depth = depth_frame->depth_m[index];
      ++sampled_depth;
      const auto at = [&current](const size_t px, const size_t py) {
          return static_cast<double>(current.gray[py * current.width + px]);
        };
      VisualSe3PhotometricSample sample;
      const double z = static_cast<double>(depth);
      sample.point_camera = Eigen::Vector3d{
        (static_cast<double>(x) - visual_intrinsics_.cx) * z / visual_intrinsics_.fx,
        (static_cast<double>(y) - visual_intrinsics_.cy) * z / visual_intrinsics_.fy,
        z};
      sample.image_gradient = Eigen::Vector2d{
        0.5 * (at(x + 1U, y) - at(x - 1U, y)),
        0.5 * (at(x, y + 1U) - at(x, y - 1U))};
      sample.residual =
        static_cast<double>(current.gray[index] - previous.gray[index]);
      const double gradient_norm = sample.image_gradient.norm();
      if (!std::isfinite(gradient_norm) || gradient_norm < visual_se3_min_gradient_) {
        ++rejected_gradient;
        continue;
      }
      const double abs_residual = std::abs(sample.residual);
      if (!std::isfinite(abs_residual) ||
        (visual_se3_max_abs_residual_ > 0.0 && abs_residual > visual_se3_max_abs_residual_))
      {
        ++rejected_residual;
        continue;
      }
      sample.weight = 1.0;
      if (visual_se3_huber_delta_intensity_ > 0.0 &&
        abs_residual > visual_se3_huber_delta_intensity_)
      {
        sample.weight = visual_se3_huber_delta_intensity_ / abs_residual;
      }
      occupied_coverage_tiles[coverage_tile_for_pixel(x, y)] = true;
      abs_residual_sum += abs_residual;
      samples.push_back(sample);
      if (samples.size() >= static_cast<size_t>(visual_se3_max_samples_)) {
        break;
      }
    }

    const size_t coverage_tiles = static_cast<size_t>(
      std::count(occupied_coverage_tiles.begin(), occupied_coverage_tiles.end(), true));
    const double mean_abs_residual = samples.empty()
      ? 0.0
      : abs_residual_sum / static_cast<double>(samples.size());
    ++visual_se3_total_batches_;
    visual_se3_sampled_depth_pixels_ += sampled_depth;
    visual_se3_rejected_gradient_pixels_ += rejected_gradient;
    visual_se3_rejected_residual_pixels_ += rejected_residual;
    last_visual_se3_coverage_tiles_ = coverage_tiles;
    last_visual_se3_coverage_total_tiles_ = coverage_total_tiles;
    last_visual_se3_mean_abs_residual_ = mean_abs_residual;
    if (sampled_depth == 0U ||
      samples.size() < static_cast<size_t>(visual_se3_min_samples_) ||
      static_cast<double>(samples.size()) / static_cast<double>(sampled_depth) <
      visual_se3_min_sample_inlier_ratio_ ||
      coverage_tiles < static_cast<size_t>(visual_se3_min_coverage_tiles_) ||
      (visual_se3_max_mean_abs_residual_ > 0.0 &&
      mean_abs_residual > visual_se3_max_mean_abs_residual_))
    {
      ++visual_se3_rejected_batches_;
      return VisualSe3PriorStatus::kRejected;
    }

    const auto linearization =
      linearize_se3_photometric_samples(visual_intrinsics_, samples);
    if (!linearization.valid ||
      linearization.sample_count < static_cast<size_t>(visual_se3_min_samples_) ||
      linearization.hessian_rank < static_cast<size_t>(visual_se3_min_hessian_rank_) ||
      (visual_se3_max_hessian_condition_ > 0.0 &&
      (linearization.hessian_condition_number <= 0.0 ||
      linearization.hessian_condition_number > visual_se3_max_hessian_condition_)))
    {
      ++visual_se3_degenerate_batches_;
      return VisualSe3PriorStatus::kRejected;
    }

    Eigen::Quaterniond q_prev;
    Eigen::Vector3d p_prev;
    if (!estimator_->query_pose(previous.stamp_ns, q_prev, p_prev)) {
      ++visual_se3_rejected_batches_;
      return VisualSe3PriorStatus::kRejected;
    }
    Eigen::Matrix<double, 6, 1> camera_delta =
      visual_se3_delta_sign_ * linearization.gauss_newton_step;
    Eigen::Matrix<double, 6, 1> body_delta =
      transform_camera_delta_to_body(
      camera_to_imu_rotation_, camera_to_imu_translation_, camera_delta);
    Eigen::Vector3d rotation_delta = body_delta.head<3>();
    Eigen::Vector3d translation_delta = body_delta.tail<3>();
    if (!rotation_delta.allFinite() || !translation_delta.allFinite()) {
      ++visual_se3_rejected_batches_;
      return VisualSe3PriorStatus::kRejected;
    }
    const double rotation_norm = rotation_delta.norm();
    const double translation_norm = translation_delta.norm();
    if ((visual_se3_max_rotation_step_rad_ > 0.0 &&
      rotation_norm > visual_se3_max_rotation_step_rad_) ||
      (visual_se3_max_translation_step_m_ > 0.0 &&
      translation_norm > visual_se3_max_translation_step_m_))
    {
      ++visual_se3_step_rejected_batches_;
      last_visual_se3_rotation_step_rad_ = rotation_norm;
      last_visual_se3_translation_step_m_ = translation_norm;
      return VisualSe3PriorStatus::kRejected;
    }
    const Eigen::Quaterniond target_q =
      (q_prev * quaternion_from_rotation_vector(rotation_delta)).normalized();
    const Eigen::Vector3d target_p = p_prev + q_prev * translation_delta;
    if (!target_q.coeffs().allFinite() || !target_p.allFinite()) {
      ++visual_se3_rejected_batches_;
      return VisualSe3PriorStatus::kRejected;
    }
    if (visual_se3_position_weight_ > 0.0) {
      estimator_->add_position_prior(
        current.stamp_ns, target_p,
        visual_se3_position_weight_, visual_se3_huber_delta_m_);
      ++visual_se3_position_priors_;
    }
    const double dt_s =
      static_cast<double>(current.stamp_ns - previous.stamp_ns) * 1.0e-9;
    if (visual_se3_velocity_weight_ > 0.0 && dt_s > 1.0e-6) {
      const Eigen::Vector3d target_velocity = q_prev * (translation_delta / dt_s);
      if (target_velocity.allFinite()) {
        estimator_->add_velocity_prior(
          current.stamp_ns, target_velocity,
          visual_se3_velocity_weight_, visual_se3_huber_delta_mps_);
        ++visual_se3_velocity_priors_;
      }
    }
    if (visual_se3_orientation_weight_ > 0.0) {
      estimator_->add_orientation_prior(
        current.stamp_ns, target_q,
        visual_se3_orientation_weight_, visual_se3_huber_delta_rad_);
      ++visual_se3_orientation_priors_;
    }
    ++visual_se3_valid_batches_;
    last_visual_se3_depth_delta_ns_ = depth_delta_ns;
    last_visual_se3_samples_ = linearization.sample_count;
    last_visual_se3_hessian_rank_ = linearization.hessian_rank;
    last_visual_se3_hessian_condition_ = linearization.hessian_condition_number;
    last_visual_se3_rotation_step_rad_ = rotation_norm;
    last_visual_se3_translation_step_m_ = translation_norm;
    return VisualSe3PriorStatus::kAdded;
  }

  void on_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (!msg || !pointcloud_enable_) {
      return;
    }
    const int64_t stamp_ns = stamp_to_nanoseconds(msg->header.stamp);
    if (stamp_ns <= 0) {
      return;
    }
    std::lock_guard<std::mutex> lock(estimator_mutex_);
    process_pointcloud_locked(msg, stamp_ns, true);
    maybe_run_stamp_driven_steps_locked(stamp_ns);
  }

  bool pointcloud_needs_pose_delay() const
  {
    // Pose-dependent factors must not silently degrade into same-scan
    // constraints before the spline window can answer the scan stamp.
    return enable_lidar_pose_prior_factor_ ||
           enable_lidar_scan_to_scan_prior_ ||
           enable_gaussian_snapshot_lidar_factor_ ||
           enable_lidar_plane_normal_factor_ ||
           enable_voxel_plane_extraction_ ||
           enable_persistent_plane_map_ ||
           enable_persistent_point_map_;
  }

  bool pointcloud_pose_ready_locked(int64_t stamp_ns) const
  {
    if (!pointcloud_needs_pose_delay() || !estimator_) {
      return true;
    }
    Eigen::Quaterniond q;
    Eigen::Vector3d p;
    return estimator_->query_pose(stamp_ns, q, p);
  }

  void enqueue_delayed_pointcloud_locked(
    const sensor_msgs::msg::PointCloud2::SharedPtr & msg)
  {
    if (pointcloud_wait_queue_max_size_ <= 0) {
      ++delayed_pointcloud_dropped_;
      return;
    }
    delayed_pointcloud_queue_.push_back(msg);
    ++delayed_pointcloud_deferred_;
    while (
      static_cast<int>(delayed_pointcloud_queue_.size()) >
      pointcloud_wait_queue_max_size_)
    {
      delayed_pointcloud_queue_.pop_front();
      ++delayed_pointcloud_dropped_;
    }
  }

  bool transform_plane_to_world(
    const spline::ExtractedPlane & plane,
    const spline::LidarExtrinsics & extrinsics,
    const Eigen::Quaterniond & q_w_i,
    const Eigen::Vector3d & p_w_i,
    Eigen::Vector3d & normal_world,
    Eigen::Vector3d & centroid_world,
    double & offset_world) const
  {
    if (!q_w_i.coeffs().allFinite() || !p_w_i.allFinite()) {
      return false;
    }
    const Eigen::Matrix3d R_l_i = extrinsics.q_lidar_to_imu.toRotationMatrix();
    const Eigen::Vector3d p_l_i = extrinsics.p_lidar_in_imu;
    const Eigen::Matrix3d R_i_w = q_w_i.toRotationMatrix();
    const Eigen::Matrix3d R_w_l = R_i_w * R_l_i;
    const Eigen::Vector3d p_w_l = R_i_w * p_l_i + p_w_i;
    normal_world = (R_w_l * plane.normal).normalized();
    centroid_world = R_w_l * plane.centroid + p_w_l;
    offset_world = plane.offset - normal_world.dot(p_w_l);
    return normal_world.allFinite() && centroid_world.allFinite() &&
           std::isfinite(offset_world);
  }

  bool add_persistent_plane_map_update(
    const spline::ExtractedPlane & plane,
    const spline::LidarExtrinsics & extrinsics,
    const Eigen::Quaterniond & q_w_i,
    const Eigen::Vector3d & p_w_i)
  {
    Eigen::Vector3d normal_world = Eigen::Vector3d::Zero();
    Eigen::Vector3d centroid_world = Eigen::Vector3d::Zero();
    double offset_world = 0.0;
    if (!transform_plane_to_world(
        plane, extrinsics, q_w_i, p_w_i,
        normal_world, centroid_world, offset_world))
    {
      return false;
    }
    return persistent_plane_map_.add_or_update(
      centroid_world, normal_world, offset_world).has_value();
  }

  void queue_deferred_plane_map_update(
    int64_t stamp_ns,
    const std::vector<spline::ExtractedPlane> & planes,
    const spline::LidarExtrinsics & extrinsics)
  {
    if (!defer_persistent_plane_map_updates_until_solved_ || planes.empty()) {
      return;
    }
    if (deferred_plane_map_update_max_queue_ == 0) {
      ++deferred_plane_map_update_dropped_;
      return;
    }
    while (
      static_cast<int>(deferred_plane_map_updates_.size()) >=
      deferred_plane_map_update_max_queue_)
    {
      deferred_plane_map_updates_.pop_front();
      ++deferred_plane_map_update_dropped_;
    }
    PendingPlaneMapUpdate update;
    update.stamp_ns = stamp_ns;
    update.planes = planes;
    update.extrinsics = extrinsics;
    deferred_plane_map_updates_.push_back(std::move(update));
    ++deferred_plane_map_update_enqueued_;
  }

  void apply_deferred_plane_map_updates_locked()
  {
    if (!estimator_ || deferred_plane_map_updates_.empty()) {
      return;
    }
    const auto & diagnostics = estimator_->diagnostics();
    const bool solve_update_allowed =
      !persistent_map_update_requires_accepted_solve_ ||
      diagnostics.steps_run == 0 ||
      diagnostics.last_step_update_accepted;
    if (!solve_update_allowed || diagnostics.last_step_update_rejected) {
      return;
    }

    std::deque<PendingPlaneMapUpdate> still_waiting;
    while (!deferred_plane_map_updates_.empty()) {
      auto update = std::move(deferred_plane_map_updates_.front());
      deferred_plane_map_updates_.pop_front();
      if (estimator_ && update.stamp_ns < estimator_->oldest_active_knot_stamp_ns()) {
        ++deferred_plane_map_update_dropped_;
        continue;
      }
      Eigen::Quaterniond q_w_i;
      Eigen::Vector3d p_w_i;
      if (!estimator_->query_pose(update.stamp_ns, q_w_i, p_w_i)) {
        still_waiting.push_back(std::move(update));
        continue;
      }
      for (const auto & plane : update.planes) {
        if (add_persistent_plane_map_update(plane, update.extrinsics, q_w_i, p_w_i)) {
          ++persistent_plane_map_updates_;
          ++deferred_plane_map_update_applied_;
        }
      }
    }
    deferred_plane_map_updates_ = std::move(still_waiting);
  }

  void process_pointcloud_locked(
    const sensor_msgs::msg::PointCloud2::SharedPtr & msg,
    int64_t stamp_ns,
    bool allow_delay)
  {
    if (!initialized_) {
      return;
    }
    const int64_t required_pose_stamp_ns = (allow_delay && enable_lidar_point_deskew_) ?
      pointcloud_required_pose_stamp_ns(*msg, stamp_ns, lidar_max_abs_point_time_offset_s_) :
      stamp_ns;
    if (allow_delay && !pointcloud_pose_ready_locked(required_pose_stamp_ns)) {
      enqueue_delayed_pointcloud_locked(msg);
      return;
    }
    spline::LidarExtrinsics extrinsics;
    extrinsics.q_lidar_to_imu = lidar_to_imu_rotation_;
    extrinsics.p_lidar_in_imu = lidar_to_imu_translation_;

    if (enable_voxel_plane_extraction_) {
      std::vector<Eigen::Vector3d> points;
      std::vector<int64_t> point_stamps_ns;
      points.reserve(static_cast<std::size_t>(msg->width) * msg->height / 4);
      point_stamps_ns.reserve(static_cast<std::size_t>(msg->width) * msg->height / 4);
      const auto * time_field = find_point_time_field(*msg);
      if (time_field != nullptr) {
        ++pointcloud_time_field_frames_;
      }
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
      size_t point_index = 0U;
      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++point_index) {
        const float fx = *iter_x;
        const float fy = *iter_y;
        const float fz = *iter_z;
        if (!std::isfinite(fx) || !std::isfinite(fy) || !std::isfinite(fz)) {
          continue;
        }
        int64_t point_stamp_ns = stamp_ns;
        if (time_field != nullptr) {
          const auto decoded_stamp = decode_point_stamp_ns(*msg, time_field, point_index, stamp_ns);
          if (decoded_stamp.has_value()) {
            const double abs_offset_s =
              std::abs(static_cast<double>(decoded_stamp.value() - stamp_ns) * 1.0e-9);
            pointcloud_last_max_abs_point_time_offset_s_ =
              std::max(pointcloud_last_max_abs_point_time_offset_s_, abs_offset_s);
            if (abs_offset_s <= lidar_max_abs_point_time_offset_s_) {
              point_stamp_ns = decoded_stamp.value();
              ++pointcloud_timed_points_;
            } else {
              ++pointcloud_out_of_range_point_times_;
            }
          } else {
            ++pointcloud_invalid_point_times_;
          }
        }
        points.emplace_back(
          static_cast<double>(fx),
          static_cast<double>(fy),
          static_cast<double>(fz));
        point_stamps_ns.push_back(point_stamp_ns);
      }
      const auto planes = plane_extractor_.extract(points);
      cache_sparse_depth_frame_locked(stamp_ns, points, extrinsics);

      // Transform each LiDAR-frame plane to world frame via the estimator's
      // current pose at the scan stamp. World-frame planes become stationary
      // constraints — the factor then truly pulls the trajectory back when
      // it drifts, instead of being a same-frame identity check.
      Eigen::Quaterniond q_b_w_at_scan = Eigen::Quaterniond::Identity();
      Eigen::Vector3d p_b_w_at_scan = Eigen::Vector3d::Zero();
      const bool have_scan_pose =
        estimator_->query_pose(stamp_ns, q_b_w_at_scan, p_b_w_at_scan);
      // Compose: world = R_b_w * R_l_b * p_lidar + R_b_w * p_l_b + p_b_w
      //                = R_w_l * p_lidar + p_w_l   (where R_w_l = R_b_w * R_l_b,
      //                                                  p_w_l = R_b_w * p_l_b + p_b_w)
      // Plane in LiDAR frame: n_l^T p_l + d_l = 0
      // After substituting p_l = R_w_l^T (p_w - p_w_l):
      //   n_l^T R_w_l^T (p_w - p_w_l) + d_l = 0
      //   (R_w_l n_l)^T p_w - (R_w_l n_l)^T p_w_l + d_l = 0
      // So n_world = R_w_l n_l, d_world = d_l - n_world^T p_w_l.
      const Eigen::Matrix3d R_l_b = extrinsics.q_lidar_to_imu.toRotationMatrix();
      const Eigen::Vector3d p_l_b = extrinsics.p_lidar_in_imu;
      const Eigen::Matrix3d R_b_w = q_b_w_at_scan.toRotationMatrix();
      const Eigen::Matrix3d R_w_l = R_b_w * R_l_b;
      const Eigen::Vector3d p_w_l = R_b_w * p_l_b + p_b_w_at_scan;
      const auto & diagnostics = estimator_->diagnostics();
      const bool persistent_map_update_allowed =
        !persistent_map_update_requires_accepted_solve_ ||
        diagnostics.steps_run == 0 ||
        diagnostics.last_step_update_accepted;
      const bool defer_plane_updates_for_scan =
        defer_persistent_plane_map_updates_until_solved_ &&
        persistent_plane_map_.size() > 0U;
      std::vector<spline::ExtractedPlane> deferred_planes;
      if (defer_plane_updates_for_scan) {
        deferred_planes.reserve(planes.size());
      }

      int accepted = 0;
      maybe_add_lidar_pose_prior_locked(
        stamp_ns, points, point_stamps_ns, q_b_w_at_scan, p_b_w_at_scan,
        extrinsics, have_scan_pose);
      maybe_add_lidar_scan_to_scan_prior_locked(
        stamp_ns, points, point_stamps_ns, q_b_w_at_scan, p_b_w_at_scan,
        extrinsics, have_scan_pose);
      if (enable_persistent_point_map_ && have_scan_pose) {
        accepted += add_persistent_point_map_correspondences(
          stamp_ns, points, R_w_l, p_w_l, extrinsics,
          persistent_map_update_allowed);
      }
      if (enable_gaussian_snapshot_lidar_factor_ && have_scan_pose) {
        accepted += add_gaussian_snapshot_correspondences(
          stamp_ns, points, q_b_w_at_scan, p_b_w_at_scan, extrinsics);
      }
      for (const auto & plane : planes) {
        Eigen::Vector3d n_world = Eigen::Vector3d::Zero();
        Eigen::Vector3d centroid_world = Eigen::Vector3d::Zero();
        double d_world = 0.0;
        bool has_world_plane = false;
        if (have_scan_pose) {
          has_world_plane = transform_plane_to_world(
            plane, extrinsics, q_b_w_at_scan, p_b_w_at_scan,
            n_world, centroid_world, d_world);
        }

        spline::LidarPointCorrespondence pc;
        pc.geometry = spline::LidarFeatureGeometry::kPlane;
        pc.point_lidar = plane.sample_point;
        if (enable_persistent_plane_map_ && has_world_plane) {
          const auto match = persistent_plane_map_.match(centroid_world, n_world);
          if (match) {
            pc.plane = match->plane;
            pc.scale = lidar_surface_feature_scale(
              match->point_to_plane_distance_m, plane.sample_point.norm());
            if (!lidar_feature_scale_is_accepted(pc.scale)) {
              if (defer_plane_updates_for_scan) {
                deferred_planes.push_back(plane);
              }
              continue;
            }
            const int64_t plane_stamp_ns =
              nearest_point_stamp_ns(points, point_stamps_ns, plane.sample_point, stamp_ns);
            estimator_->add_lidar_correspondence(
              plane_stamp_ns, pc, extrinsics, pointcloud_factor_weight_,
              lidar_huber_delta_m_);
            if (enable_lidar_plane_normal_factor_) {
              estimator_->add_lidar_plane_normal_correspondence(
                plane_stamp_ns, plane.normal, pc.plane.head<3>(), extrinsics,
                lidar_plane_normal_factor_weight_,
                lidar_plane_normal_huber_delta_rad_);
              ++persistent_plane_normal_factors_;
            }
            ++accepted;
            ++persistent_plane_map_matches_;
          }
          if (defer_plane_updates_for_scan) {
            deferred_planes.push_back(plane);
          } else if (persistent_map_update_allowed &&
            add_persistent_plane_map_update(plane, extrinsics, q_b_w_at_scan, p_b_w_at_scan))
          {
            ++persistent_plane_map_updates_;
          } else if (!persistent_map_update_allowed) {
            ++persistent_plane_map_update_skips_;
          }
          continue;
        }
        if (has_world_plane) {
          pc.plane.head<3>() = n_world;
          pc.plane[3] = d_world;
        } else {
          // Falling back to LiDAR-frame plane until the estimator has a
          // valid pose at the scan stamp — this still degenerates into
          // a same-frame identity for the first few frames.
          pc.plane.head<3>() = plane.normal;
          pc.plane[3] = plane.offset;
        }
        const Eigen::Vector3d point_for_scale =
          has_world_plane ? (R_w_l * pc.point_lidar + p_w_l) : pc.point_lidar;
        const double plane_residual =
          pc.plane.head<3>().dot(point_for_scale) + pc.plane[3];
        pc.scale = lidar_surface_feature_scale(plane_residual, pc.point_lidar.norm());
        if (!lidar_feature_scale_is_accepted(pc.scale)) {
          continue;
        }
        const int64_t plane_stamp_ns =
          nearest_point_stamp_ns(points, point_stamps_ns, plane.sample_point, stamp_ns);
        estimator_->add_lidar_correspondence(
          plane_stamp_ns, pc, extrinsics, pointcloud_factor_weight_,
          lidar_huber_delta_m_);
        ++accepted;
      }
      if (!deferred_planes.empty()) {
        queue_deferred_plane_map_update(stamp_ns, deferred_planes, extrinsics);
      }
      accepted_pointcloud_correspondences_ += static_cast<std::size_t>(accepted);
      ++pointcloud_messages_;
      return;
    }

    spline::LidarPointCorrespondence pc;
    pc.geometry = spline::LidarFeatureGeometry::kPlane;
    pc.plane = lidar_plane_;

    int accepted = 0;
    int stride_counter = 0;
    std::vector<Eigen::Vector3d> points;
    std::vector<int64_t> point_stamps_ns;
    if (enable_lidar_pose_prior_factor_ || enable_lidar_scan_to_scan_prior_ ||
      enable_visual_se3_prior_)
    {
      points.reserve(static_cast<std::size_t>(msg->width) * msg->height / 4);
    }
    point_stamps_ns.reserve(static_cast<std::size_t>(msg->width) * msg->height / 4);
    const auto * time_field = find_point_time_field(*msg);
    if (time_field != nullptr) {
      ++pointcloud_time_field_frames_;
    }
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
    size_t point_index = 0U;
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++point_index) {
      if (stride_counter++ % std::max(1, pointcloud_subsample_stride_) != 0) {
        continue;
      }
      const float fx = *iter_x;
      const float fy = *iter_y;
      const float fz = *iter_z;
      if (!std::isfinite(fx) || !std::isfinite(fy) || !std::isfinite(fz)) {
        continue;
      }
      const double rx = static_cast<double>(fx);
      const double ry = static_cast<double>(fy);
      const double rz = static_cast<double>(fz);
      const double range = std::sqrt(rx * rx + ry * ry + rz * rz);
      if (range < pointcloud_min_range_m_ || range > pointcloud_max_range_m_) {
        continue;
      }
      int64_t point_stamp_ns = stamp_ns;
      if (time_field != nullptr) {
        const auto decoded_stamp = decode_point_stamp_ns(*msg, time_field, point_index, stamp_ns);
        if (decoded_stamp.has_value()) {
          const double abs_offset_s =
            std::abs(static_cast<double>(decoded_stamp.value() - stamp_ns) * 1.0e-9);
          pointcloud_last_max_abs_point_time_offset_s_ =
            std::max(pointcloud_last_max_abs_point_time_offset_s_, abs_offset_s);
          if (abs_offset_s <= lidar_max_abs_point_time_offset_s_) {
            point_stamp_ns = decoded_stamp.value();
            ++pointcloud_timed_points_;
          } else {
            ++pointcloud_out_of_range_point_times_;
          }
        } else {
          ++pointcloud_invalid_point_times_;
        }
      }
      if (enable_lidar_pose_prior_factor_ || enable_lidar_scan_to_scan_prior_ ||
        enable_visual_se3_prior_)
      {
        points.emplace_back(rx, ry, rz);
        point_stamps_ns.push_back(point_stamp_ns);
      }
      pc.point_lidar = Eigen::Vector3d(rx, ry, rz);
      pc.scale = lidar_surface_feature_scale(
        pc.plane.head<3>().dot(pc.point_lidar) + pc.plane[3], range);
      if (!lidar_feature_scale_is_accepted(pc.scale)) {
        continue;
      }
      estimator_->add_lidar_correspondence(
        point_stamp_ns, pc, extrinsics, pointcloud_factor_weight_,
        lidar_huber_delta_m_);
      ++accepted;
      if (pointcloud_max_points_per_msg_ > 0 &&
        accepted >= pointcloud_max_points_per_msg_)
      {
        break;
      }
    }
    if (enable_lidar_pose_prior_factor_) {
      Eigen::Quaterniond q;
      Eigen::Vector3d p;
      const bool have_scan_pose = estimator_->query_pose(stamp_ns, q, p);
      maybe_add_lidar_pose_prior_locked(
        stamp_ns, points, point_stamps_ns, q, p, extrinsics, have_scan_pose);
      maybe_add_lidar_scan_to_scan_prior_locked(
        stamp_ns, points, point_stamps_ns, q, p, extrinsics, have_scan_pose);
    } else if (enable_lidar_scan_to_scan_prior_) {
      Eigen::Quaterniond q;
      Eigen::Vector3d p;
      const bool have_scan_pose = estimator_->query_pose(stamp_ns, q, p);
      maybe_add_lidar_scan_to_scan_prior_locked(
        stamp_ns, points, point_stamps_ns, q, p, extrinsics, have_scan_pose);
    }
    cache_sparse_depth_frame_locked(stamp_ns, points, extrinsics);
    accepted_pointcloud_correspondences_ += static_cast<std::size_t>(accepted);
    ++pointcloud_messages_;
  }

  void drain_delayed_pointclouds_locked()
  {
    if (delayed_pointcloud_queue_.empty()) {
      return;
    }
    std::deque<sensor_msgs::msg::PointCloud2::SharedPtr> still_waiting;
    while (!delayed_pointcloud_queue_.empty()) {
      auto msg = delayed_pointcloud_queue_.front();
      delayed_pointcloud_queue_.pop_front();
      if (!msg) {
        ++delayed_pointcloud_dropped_;
        continue;
      }
      const int64_t stamp_ns = stamp_to_nanoseconds(msg->header.stamp);
      if (stamp_ns <= 0) {
        ++delayed_pointcloud_dropped_;
        continue;
      }
      const int64_t required_pose_stamp_ns = enable_lidar_point_deskew_ ?
        pointcloud_required_pose_stamp_ns(*msg, stamp_ns, lidar_max_abs_point_time_offset_s_) :
        stamp_ns;
      if (pointcloud_pose_ready_locked(required_pose_stamp_ns)) {
        process_pointcloud_locked(msg, stamp_ns, false);
        ++delayed_pointcloud_released_;
        continue;
      }
      if (estimator_ && required_pose_stamp_ns < estimator_->oldest_active_knot_stamp_ns()) {
        ++delayed_pointcloud_dropped_;
        continue;
      }
      still_waiting.push_back(msg);
    }
    delayed_pointcloud_queue_ = std::move(still_waiting);
  }

  void on_step_timer()
  {
    std::lock_guard<std::mutex> lock(estimator_mutex_);
    if (!initialized_ || use_stamp_driven_steps_) {
      return;
    }
    run_estimator_step_locked();
  }

  void maybe_run_stamp_driven_steps_locked(int64_t stamp_ns)
  {
    if (!use_stamp_driven_steps_ || !initialized_ || step_period_ns_ <= 0 || stamp_ns <= 0) {
      return;
    }
    if (last_stamp_driven_step_ns_ == 0) {
      last_stamp_driven_step_ns_ = stamp_ns;
      return;
    }
    int steps = 0;
    while (
      stamp_ns - last_stamp_driven_step_ns_ >= step_period_ns_ &&
      steps < max_stamp_driven_steps_per_callback_)
    {
      last_stamp_driven_step_ns_ += step_period_ns_;
      run_estimator_step_locked();
      ++steps;
    }
    if (steps == max_stamp_driven_steps_per_callback_ &&
      stamp_ns - last_stamp_driven_step_ns_ >= step_period_ns_)
    {
      last_stamp_driven_step_ns_ = stamp_ns - step_period_ns_;
    }
  }

  void run_estimator_step_locked()
  {
    const bool stepped = estimator_->step();
    if (!stepped) {
      return;
    }
    apply_deferred_plane_map_updates_locked();
    drain_delayed_pointclouds_locked();
    const auto & diagnostics = estimator_->diagnostics();
    if (diagnostics.rejected_solver_steps > last_logged_rejected_solver_steps_) {
      last_logged_rejected_solver_steps_ = diagnostics.rejected_solver_steps;
      RCLCPP_WARN(
        get_logger(),
        "continuous-time solve update rejected: total=%zu max_dp=%.3f m max_dtheta=%.3f rad",
        diagnostics.rejected_solver_steps,
        diagnostics.last_rejected_position_update_m,
        diagnostics.last_rejected_rotation_update_rad);
    }
    if (
      diagnostics.rotation_limited_solver_steps >
      last_logged_rotation_limited_solver_steps_)
    {
      last_logged_rotation_limited_solver_steps_ =
        diagnostics.rotation_limited_solver_steps;
      RCLCPP_WARN(
        get_logger(),
        "continuous-time solve rotation update limited: total=%zu kept_rotation=true max_dp=%.3f m max_dtheta=%.3f rad",
        diagnostics.rotation_limited_solver_steps,
        diagnostics.last_rotation_limited_position_update_m,
        diagnostics.last_rotation_limited_rotation_update_rad);
    }
    if (
      diagnostics.position_limited_solver_steps >
      last_logged_position_limited_solver_steps_)
    {
      last_logged_position_limited_solver_steps_ =
        diagnostics.position_limited_solver_steps;
      RCLCPP_WARN(
        get_logger(),
        "continuous-time solve position update limited: total=%zu kept_position=true max_dp=%.3f m max_dtheta=%.3f rad",
        diagnostics.position_limited_solver_steps,
        diagnostics.last_position_limited_position_update_m,
        diagnostics.last_position_limited_rotation_update_rad);
    }
    log_runtime_diagnostics_if_due(diagnostics);
    publish_latest_pose();
  }

  void log_runtime_diagnostics_if_due(
    const spline::ContinuousTimeSlidingWindowDiagnostics & diagnostics)
  {
    if (diagnostic_log_period_steps_ == 0) {
      return;
    }
    if (
      last_logged_diagnostic_step_ != 0 &&
      diagnostics.steps_run < last_logged_diagnostic_step_ +
        static_cast<std::size_t>(diagnostic_log_period_steps_))
    {
      return;
    }
    last_logged_diagnostic_step_ = diagnostics.steps_run;
    const Eigen::Vector3d gyro_bias =
      estimator_ ? estimator_->gyro_bias() : Eigen::Vector3d::Zero();
    const Eigen::Vector3d accel_bias =
      estimator_ ? estimator_->accel_bias() : Eigen::Vector3d::Zero();
    const Eigen::Vector3d gravity_world =
      estimator_ ? estimator_->gravity_world() : Eigen::Vector3d::Zero();
    RCLCPP_INFO(
      get_logger(),
      "continuous-time diagnostics: steps=%zu imu_factors=%zu lidar_factors=%zu "
      "lidar_point_factors=%zu lidar_normal_factors=%zu "
      "position_smoothness_factors=%zu rotation_smoothness_factors=%zu "
      "retained_knot_position_priors=%zu retained_knot_orientation_priors=%zu "
      "spline_position_marginalization_priors=%zu spline_position_marginalization_rows=%zu "
      "spline_orientation_marginalization_priors=%zu spline_orientation_marginalization_rows=%zu "
      "accepted_steps=%zu "
      "gyro_bias_x=%.9g gyro_bias_y=%.9g gyro_bias_z=%.9g gyro_bias_norm=%.9g "
      "accel_bias_x=%.9g accel_bias_y=%.9g accel_bias_z=%.9g accel_bias_norm=%.9g "
      "gravity_x=%.9g gravity_y=%.9g gravity_z=%.9g gravity_norm=%.9g "
      "last_imu_factors=%zu last_lidar_factors=%zu "
      "last_lidar_point_factors=%zu last_lidar_normal_factors=%zu "
      "last_position_smoothness_factors=%zu last_rotation_smoothness_factors=%zu "
      "last_retained_knot_position_priors=%zu "
      "last_retained_knot_orientation_priors=%zu "
      "last_spline_position_marginalization_priors=%zu "
      "last_spline_position_marginalization_rows=%zu "
      "last_spline_orientation_marginalization_priors=%zu "
      "last_spline_orientation_marginalization_rows=%zu "
      "last_position_prior_factors=%zu last_velocity_prior_factors=%zu "
      "last_acceleration_prior_factors=%zu "
      "last_angular_velocity_prior_factors=%zu "
      "last_orientation_prior_factors=%zu "
      "last_gyro_bias_prior_factors=%zu last_accel_bias_prior_factors=%zu "
      "initial_cost=%.9g final_cost=%.9g initial_imu_cost=%.9g "
      "final_imu_cost=%.9g initial_lidar_cost=%.9g final_lidar_cost=%.9g "
      "initial_position_prior_cost=%.9g final_position_prior_cost=%.9g "
      "initial_velocity_prior_cost=%.9g final_velocity_prior_cost=%.9g "
      "initial_acceleration_prior_cost=%.9g final_acceleration_prior_cost=%.9g "
      "initial_orientation_prior_cost=%.9g final_orientation_prior_cost=%.9g "
      "initial_bias_prior_cost=%.9g final_bias_prior_cost=%.9g "
      "initial_smoothness_cost=%.9g final_smoothness_cost=%.9g "
      "effective_gyro_bias_prior_weight=%.9g effective_accel_bias_prior_weight=%.9g "
      "max_position_update_m=%.9g max_rotation_update_rad=%.9g "
      "position_prior_factors=%zu velocity_prior_factors=%zu "
      "acceleration_prior_factors=%zu "
      "angular_velocity_prior_factors=%zu "
      "orientation_prior_factors=%zu "
      "gyro_bias_prior_factors=%zu accel_bias_prior_factors=%zu "
      "imu_msgs=%zu dropped_imu=%zu rejected_imu=%zu pointcloud_msgs=%zu "
      "pointcloud_corr=%zu plane_matches=%zu plane_updates=%zu point_matches=%zu "
      "pointcloud_time_field_frames=%zu pointcloud_timed_points=%zu "
      "pointcloud_invalid_point_times=%zu pointcloud_out_of_range_point_times=%zu "
      "pointcloud_last_max_abs_point_time_offset_s=%.9g "
      "pointcloud_deskewed_points=%zu pointcloud_deskew_fallback_points=%zu "
      "pointcloud_last_max_deskew_m=%.9g "
      "plane_update_skips=%zu point_updates=%zu point_update_skips=%zu "
      "gaussian_snapshot_points=%zu "
      "gaussian_snapshot_chunks_received=%zu "
      "gaussian_snapshot_expected_chunks=%u "
      "gaussian_snapshot_updates=%zu "
      "gaussian_snapshot_matches=%zu "
      "gaussian_snapshot_factor_skips=%zu "
      "plane_normal_factors=%zu "
      "visual_rotation_images=%zu visual_rotation_accepted=%zu "
      "visual_rotation_rejected=%zu visual_rotation_priors=%zu "
      "visual_rotation_dx_px=%.9g visual_rotation_dy_px=%.9g "
      "visual_rotation_angle_rad=%.9g visual_rotation_rmse=%.9g "
      "visual_se_depth_frames=%zu visual_se_depth_miss=%zu "
      "visual_se_batches=%zu visual_se_valid=%zu visual_se_rejected=%zu "
      "visual_se_degenerate=%zu visual_se_step_rejected=%zu "
      "visual_se_position_priors=%zu visual_se_velocity_priors=%zu "
      "visual_se_orientation_priors=%zu "
      "visual_se_sampled_depth=%zu visual_se_rejected_gradient=%zu "
      "visual_se_rejected_residual=%zu visual_se_last_samples=%zu "
      "visual_se_last_rank=%zu visual_se_last_coverage_tiles=%zu "
      "visual_se_last_coverage_total=%zu visual_se_last_condition=%.9g "
      "visual_se_last_mean_abs_residual=%.9g "
      "visual_se_last_translation_step_m=%.9g "
      "visual_se_last_rotation_step_rad=%.9g "
      "visual_se_last_depth_delta_ns=%ld "
      "lidar_pose_priors=%zu lidar_pose_velocity_priors=%zu "
      "lidar_pose_acceleration_priors=%zu "
      "lidar_pose_angular_velocity_priors=%zu "
      "lidar_pose_matches=%zu lidar_pose_keyframes=%zu "
      "lidar_pose_rejected=%zu lidar_pose_last_residual=%.9g "
      "lidar_scan_to_scan_priors=%zu lidar_scan_to_scan_velocity_priors=%zu "
      "lidar_scan_to_scan_acceleration_priors=%zu "
      "lidar_scan_to_scan_angular_velocity_priors=%zu "
      "lidar_scan_to_scan_position_priors=%zu "
      "lidar_scan_to_scan_orientation_priors=%zu "
      "lidar_scan_to_scan_dead_reckoned=%zu "
      "lidar_scan_to_scan_pose_seed_updates=%zu "
      "lidar_scan_to_scan_pose_seed_rejected=%zu "
      "lidar_scan_to_scan_velocity_clamped=%zu "
      "lidar_scan_to_scan_acceleration_clamped=%zu "
      "lidar_scan_to_scan_angular_velocity_clamped=%zu "
      "lidar_scan_to_scan_point_to_plane=%zu "
      "lidar_scan_to_scan_point_to_point_fallback=%zu "
      "lidar_scan_to_scan_matches=%zu lidar_scan_to_scan_rejected=%zu "
      "lidar_scan_to_scan_last_residual=%.9g "
      "lidar_scan_to_scan_predicted_rel_m=%.9g "
      "lidar_scan_to_scan_correction_m=%.9g "
      "lidar_scan_to_scan_correction_rot_rad=%.9g "
      "lidar_scan_to_scan_raw_target_rel_m=%.9g "
      "lidar_scan_to_scan_target_rel_m=%.9g "
      "lidar_scan_to_scan_target_prediction_ratio=%.9g "
      "lidar_scan_to_scan_target_speed_mps=%.9g "
      "lidar_scan_to_scan_target_angular_radps=%.9g "
      "lidar_scan_to_scan_cumulative_target_path_m=%.9g "
      "lidar_scan_to_scan_small_translation_targets=%zu "
      "lidar_scan_to_scan_prediction_translation_fallbacks=%zu "
      "lidar_scan_to_scan_translation_priors_skipped=%zu "
      "prior_seed=%zu prior_position_factors=%zu "
      "prior_orientation_factors=%zu "
      "prior_rejected=%zu delayed_pc_deferred=%zu delayed_pc_released=%zu "
      "delayed_pc_dropped=%zu delayed_pc_pending=%zu "
      "deferred_plane_updates=%zu deferred_plane_applied=%zu "
      "deferred_plane_dropped=%zu deferred_plane_pending=%zu tum_lines=%zu "
      "rejected_steps=%zu invalid_rejections=%zu position_rejections=%zu "
      "rotation_rejections=%zu rotation_limited_steps=%zu position_limited_steps=%zu",
      diagnostics.steps_run,
      diagnostics.total_imu_factors,
      diagnostics.total_lidar_factors,
      diagnostics.total_lidar_point_factors,
      diagnostics.total_lidar_normal_factors,
      diagnostics.total_position_smoothness_factors,
      diagnostics.total_rotation_smoothness_factors,
      diagnostics.total_retained_knot_position_prior_factors,
      diagnostics.total_retained_knot_orientation_prior_factors,
      diagnostics.total_spline_marginalization_priors,
      diagnostics.total_spline_marginalization_prior_rows,
      diagnostics.total_spline_orientation_marginalization_priors,
      diagnostics.total_spline_orientation_marginalization_prior_rows,
      diagnostics.accepted_solver_steps,
      gyro_bias.x(),
      gyro_bias.y(),
      gyro_bias.z(),
      gyro_bias.norm(),
      accel_bias.x(),
      accel_bias.y(),
      accel_bias.z(),
      accel_bias.norm(),
      gravity_world.x(),
      gravity_world.y(),
      gravity_world.z(),
      gravity_world.norm(),
      diagnostics.last_step_imu_factors,
      diagnostics.last_step_lidar_factors,
      diagnostics.last_step_lidar_point_factors,
      diagnostics.last_step_lidar_normal_factors,
      diagnostics.last_step_position_smoothness_factors,
      diagnostics.last_step_rotation_smoothness_factors,
      diagnostics.last_step_retained_knot_position_prior_factors,
      diagnostics.last_step_retained_knot_orientation_prior_factors,
      diagnostics.last_step_spline_marginalization_prior_factors,
      diagnostics.last_step_spline_marginalization_prior_rows,
      diagnostics.last_step_spline_orientation_marginalization_prior_factors,
      diagnostics.last_step_spline_orientation_marginalization_prior_rows,
      diagnostics.last_step_position_prior_factors,
      diagnostics.last_step_velocity_prior_factors,
      diagnostics.last_step_acceleration_prior_factors,
      diagnostics.last_step_angular_velocity_prior_factors,
      diagnostics.last_step_orientation_prior_factors,
      diagnostics.last_step_gyro_bias_prior_factors,
      diagnostics.last_step_accel_bias_prior_factors,
      diagnostics.last_step_initial_cost,
      diagnostics.last_step_final_cost,
      diagnostics.last_step_initial_imu_cost,
      diagnostics.last_step_final_imu_cost,
      diagnostics.last_step_initial_lidar_cost,
      diagnostics.last_step_final_lidar_cost,
      diagnostics.last_step_initial_position_prior_cost,
      diagnostics.last_step_final_position_prior_cost,
      diagnostics.last_step_initial_velocity_prior_cost,
      diagnostics.last_step_final_velocity_prior_cost,
      diagnostics.last_step_initial_acceleration_prior_cost,
      diagnostics.last_step_final_acceleration_prior_cost,
      diagnostics.last_step_initial_orientation_prior_cost,
      diagnostics.last_step_final_orientation_prior_cost,
      diagnostics.last_step_initial_bias_prior_cost,
      diagnostics.last_step_final_bias_prior_cost,
      diagnostics.last_step_initial_smoothness_cost,
      diagnostics.last_step_final_smoothness_cost,
      diagnostics.last_step_effective_gyro_bias_prior_weight,
      diagnostics.last_step_effective_accel_bias_prior_weight,
      diagnostics.last_step_max_position_update_m,
      diagnostics.last_step_max_rotation_update_rad,
      diagnostics.total_position_prior_factors,
      diagnostics.total_velocity_prior_factors,
      diagnostics.total_acceleration_prior_factors,
      diagnostics.total_angular_velocity_prior_factors,
      diagnostics.total_orientation_prior_factors,
      diagnostics.total_gyro_bias_prior_factors,
      diagnostics.total_accel_bias_prior_factors,
      accepted_imu_count_,
      dropped_imu_count_,
      rejected_imu_count_,
      pointcloud_messages_,
      accepted_pointcloud_correspondences_,
      persistent_plane_map_matches_,
      persistent_plane_map_updates_,
      persistent_point_map_matches_,
      pointcloud_time_field_frames_,
      pointcloud_timed_points_,
      pointcloud_invalid_point_times_,
      pointcloud_out_of_range_point_times_,
      pointcloud_last_max_abs_point_time_offset_s_,
      pointcloud_deskewed_points_,
      pointcloud_deskew_fallback_points_,
      pointcloud_last_max_deskew_m_,
      persistent_plane_map_update_skips_,
      persistent_point_map_updates_,
      persistent_point_map_update_skips_,
      gaussian_snapshot_points_,
      gaussian_snapshot_chunks_received_,
      gaussian_snapshot_expected_chunks_,
      gaussian_snapshot_updates_,
      gaussian_snapshot_lidar_matches_,
      gaussian_snapshot_lidar_factor_skips_,
      persistent_plane_normal_factors_,
      visual_rotation_image_frames_,
      visual_rotation_accepted_frames_,
      visual_rotation_rejected_frames_,
      visual_rotation_prior_factors_,
      last_visual_rotation_dx_px_,
      last_visual_rotation_dy_px_,
      last_visual_rotation_angle_rad_,
      last_visual_rotation_rmse_,
      visual_se3_depth_frames_,
      visual_se3_depth_miss_count_,
      visual_se3_total_batches_,
      visual_se3_valid_batches_,
      visual_se3_rejected_batches_,
      visual_se3_degenerate_batches_,
      visual_se3_step_rejected_batches_,
      visual_se3_position_priors_,
      visual_se3_velocity_priors_,
      visual_se3_orientation_priors_,
      visual_se3_sampled_depth_pixels_,
      visual_se3_rejected_gradient_pixels_,
      visual_se3_rejected_residual_pixels_,
      last_visual_se3_samples_,
      last_visual_se3_hessian_rank_,
      last_visual_se3_coverage_tiles_,
      last_visual_se3_coverage_total_tiles_,
      last_visual_se3_hessian_condition_,
      last_visual_se3_mean_abs_residual_,
      last_visual_se3_translation_step_m_,
      last_visual_se3_rotation_step_rad_,
      static_cast<long>(last_visual_se3_depth_delta_ns_),
      lidar_pose_prior_factors_,
      lidar_pose_prior_velocity_factors_,
      lidar_pose_prior_acceleration_factors_,
      lidar_pose_prior_angular_velocity_factors_,
      lidar_pose_prior_matches_,
      lidar_pose_factor_keyframes_,
      lidar_pose_prior_rejected_,
      lidar_pose_prior_last_mean_residual_m_,
      lidar_scan_to_scan_priors_,
      lidar_scan_to_scan_velocity_priors_,
      lidar_scan_to_scan_acceleration_priors_,
      lidar_scan_to_scan_angular_velocity_priors_,
      lidar_scan_to_scan_position_priors_,
      lidar_scan_to_scan_orientation_priors_,
      lidar_scan_to_scan_dead_reckoned_,
      lidar_scan_to_scan_pose_seed_updates_,
      lidar_scan_to_scan_pose_seed_rejected_,
      lidar_scan_to_scan_velocity_clamped_,
      lidar_scan_to_scan_acceleration_clamped_,
      lidar_scan_to_scan_angular_velocity_clamped_,
      lidar_scan_to_scan_point_to_plane_corrections_,
      lidar_scan_to_scan_point_to_point_fallbacks_,
      lidar_scan_to_scan_matches_,
      lidar_scan_to_scan_rejected_,
      lidar_scan_to_scan_last_residual_m_,
      lidar_scan_to_scan_last_predicted_relative_translation_m_,
      lidar_scan_to_scan_last_correction_translation_m_,
      lidar_scan_to_scan_last_correction_rotation_rad_,
      lidar_scan_to_scan_last_raw_target_relative_translation_m_,
      lidar_scan_to_scan_last_target_relative_translation_m_,
      lidar_scan_to_scan_last_target_prediction_ratio_,
      lidar_scan_to_scan_last_target_speed_mps_,
      lidar_scan_to_scan_last_target_angular_speed_radps_,
      lidar_scan_to_scan_cumulative_target_path_m_,
      lidar_scan_to_scan_small_translation_targets_,
      lidar_scan_to_scan_prediction_translation_fallbacks_,
      lidar_scan_to_scan_translation_priors_skipped_,
      accepted_prior_count_,
      accepted_prior_position_factor_messages_,
      accepted_prior_orientation_factor_messages_,
      rejected_prior_count_,
      delayed_pointcloud_deferred_,
      delayed_pointcloud_released_,
      delayed_pointcloud_dropped_,
      delayed_pointcloud_queue_.size(),
      deferred_plane_map_update_enqueued_,
      deferred_plane_map_update_applied_,
      deferred_plane_map_update_dropped_,
      deferred_plane_map_updates_.size(),
      tum_lines_written_,
      diagnostics.rejected_solver_steps,
      diagnostics.invalid_update_rejections,
      diagnostics.position_update_rejections,
      diagnostics.rotation_update_rejections,
      diagnostics.rotation_limited_solver_steps,
      diagnostics.position_limited_solver_steps);
  }

  bool find_nearest_persistent_point(
    int64_t stamp_ns,
    const Eigen::Vector3d & point_world,
    Eigen::Vector3d & nearest_world,
    double & nearest_distance_sq) const
  {
    if (persistent_points_world_.empty() || !point_world.allFinite()) {
      return false;
    }
    const double max_distance_sq =
      persistent_point_map_nearest_distance_m_ * persistent_point_map_nearest_distance_m_;
    nearest_distance_sq = max_distance_sq;
    bool found = false;
    for (const auto & candidate : persistent_points_world_) {
      if (!persistent_point_is_match_ready(candidate, stamp_ns)) {
        continue;
      }
      const double distance_sq = (candidate.point_world - point_world).squaredNorm();
      if (distance_sq <= nearest_distance_sq) {
        nearest_distance_sq = distance_sq;
        nearest_world = candidate.point_world;
        found = true;
      }
    }
    return found;
  }

  bool persistent_point_is_match_ready(
    const PersistentPointMapEntry & candidate,
    int64_t stamp_ns) const
  {
    if (!candidate.point_world.allFinite() ||
      candidate.observations < persistent_point_map_min_observations_for_match_)
    {
      return false;
    }
    if (candidate.last_stamp_ns == stamp_ns) {
      return false;
    }
    if (persistent_point_map_min_match_age_s_ <= 0.0) {
      return true;
    }
    if (stamp_ns <= candidate.first_stamp_ns) {
      return false;
    }
    const double age_s =
      static_cast<double>(stamp_ns - candidate.first_stamp_ns) * 1.0e-9;
    return std::isfinite(age_s) && age_s >= persistent_point_map_min_match_age_s_;
  }

  std::optional<std::size_t> find_persistent_point_update_target(
    const Eigen::Vector3d & point_world) const
  {
    if (persistent_points_world_.empty() || !point_world.allFinite() ||
      persistent_point_map_merge_distance_m_ <= 0.0)
    {
      return std::nullopt;
    }
    const double max_distance_sq =
      persistent_point_map_merge_distance_m_ * persistent_point_map_merge_distance_m_;
    double nearest_distance_sq = max_distance_sq;
    std::optional<std::size_t> nearest_index;
    for (std::size_t index = 0; index < persistent_points_world_.size(); ++index) {
      const auto & candidate = persistent_points_world_[index];
      if (!candidate.point_world.allFinite()) {
        continue;
      }
      const double distance_sq = (candidate.point_world - point_world).squaredNorm();
      if (distance_sq <= nearest_distance_sq) {
        nearest_distance_sq = distance_sq;
        nearest_index = index;
      }
    }
    return nearest_index;
  }

  bool update_persistent_point_map(int64_t stamp_ns, const Eigen::Vector3d & point_world)
  {
    if (!point_world.allFinite()) {
      return false;
    }
    if (const auto target_index = find_persistent_point_update_target(point_world)) {
      auto & target = persistent_points_world_[*target_index];
      const bool new_observation_stamp = target.last_stamp_ns != stamp_ns;
      if (new_observation_stamp) {
        const int previous_observations = std::max(target.observations, 1);
        const int next_observations = previous_observations + 1;
        target.point_world =
          (target.point_world * static_cast<double>(previous_observations) + point_world) /
          static_cast<double>(next_observations);
        target.observations = next_observations;
        target.last_stamp_mean_world = point_world;
        target.last_stamp_sample_count = 1;
      } else {
        const int observations = std::max(target.observations, 1);
        const int previous_stamp_samples = std::max(target.last_stamp_sample_count, 1);
        const Eigen::Vector3d previous_stamp_mean = target.last_stamp_mean_world.allFinite() ?
          target.last_stamp_mean_world :
          point_world;
        const Eigen::Vector3d next_stamp_mean =
          (previous_stamp_mean * static_cast<double>(previous_stamp_samples) + point_world) /
          static_cast<double>(previous_stamp_samples + 1);
        target.point_world =
          (target.point_world * static_cast<double>(observations) -
          previous_stamp_mean + next_stamp_mean) /
          static_cast<double>(observations);
        target.last_stamp_mean_world = next_stamp_mean;
        target.last_stamp_sample_count = previous_stamp_samples + 1;
      }
      target.last_stamp_ns = stamp_ns;
      return true;
    }
    if (persistent_point_map_max_points_ > 0 &&
      static_cast<int>(persistent_points_world_.size()) >= persistent_point_map_max_points_)
    {
      return false;
    }
    PersistentPointMapEntry entry;
    entry.point_world = point_world;
    entry.last_stamp_mean_world = point_world;
    entry.observations = 1;
    entry.last_stamp_sample_count = 1;
    entry.first_stamp_ns = stamp_ns;
    entry.last_stamp_ns = stamp_ns;
    persistent_points_world_.push_back(entry);
    return true;
  }

  std::size_t commit_persistent_point_map_updates(
    int64_t stamp_ns,
    const std::vector<Eigen::Vector3d> & pending_point_updates)
  {
    std::size_t applied = 0U;
    for (const auto & point_world : pending_point_updates) {
      if (update_persistent_point_map(stamp_ns, point_world)) {
        ++applied;
      } else {
        ++persistent_point_map_update_skips_;
      }
    }
    return applied;
  }

  double lidar_surface_feature_scale(double abs_point_to_plane_m, double range_m) const
  {
    if (!pointcloud_use_lidar_scale_) {
      return 1.0;
    }
    if (!std::isfinite(abs_point_to_plane_m) || !std::isfinite(range_m)) {
      return 0.0;
    }
    const double safe_range = std::max(range_m, 1.0e-6);
    const double denominator = std::sqrt(std::sqrt(safe_range * safe_range));
    if (!std::isfinite(denominator) || denominator <= 1.0e-9) {
      return 0.0;
    }
    return std::clamp(
      1.0 - 0.9 * std::abs(abs_point_to_plane_m) / denominator, 0.0, 1.0);
  }

  bool lidar_feature_scale_is_accepted(double scale) const
  {
    return !pointcloud_use_lidar_scale_ ||
           (std::isfinite(scale) && scale > pointcloud_min_lidar_scale_);
  }

  int add_persistent_point_map_correspondences(
    int64_t stamp_ns,
    const std::vector<Eigen::Vector3d> & points_lidar,
    const Eigen::Matrix3d & R_w_l,
    const Eigen::Vector3d & p_w_l,
    const spline::LidarExtrinsics & extrinsics,
    bool allow_map_update)
  {
    if (!R_w_l.allFinite() || !p_w_l.allFinite()) {
      return 0;
    }
    int accepted = 0;
    int stride_counter = 0;
    std::vector<Eigen::Vector3d> pending_point_updates;
    if (allow_map_update) {
      pending_point_updates.reserve(points_lidar.size());
    }
    for (const auto & point_lidar : points_lidar) {
      if (persistent_point_map_subsample_stride_ > 1 &&
        stride_counter++ % persistent_point_map_subsample_stride_ != 0)
      {
        continue;
      }
      const double range = point_lidar.norm();
      if (!std::isfinite(range) || range < pointcloud_min_range_m_ ||
        range > pointcloud_max_range_m_)
      {
        continue;
      }
      const Eigen::Vector3d point_world = R_w_l * point_lidar + p_w_l;
      Eigen::Vector3d nearest_world = Eigen::Vector3d::Zero();
      double nearest_distance_sq = 0.0;
      if (find_nearest_persistent_point(
          stamp_ns, point_world, nearest_world, nearest_distance_sq))
      {
        const double feature_scale =
          lidar_surface_feature_scale(std::sqrt(nearest_distance_sq), range);
        if (!lidar_feature_scale_is_accepted(feature_scale)) {
          continue;
        }
        const Eigen::Vector3d nearest_map =
          extrinsics.q_world_to_map * nearest_world + extrinsics.p_world_in_map;
        estimator_->add_lidar_point_to_point_correspondence(
          stamp_ns, point_lidar, nearest_map, extrinsics,
          persistent_point_map_factor_weight_, lidar_huber_delta_m_, feature_scale);
        ++accepted;
        ++persistent_point_map_matches_;
        if (persistent_point_map_max_correspondences_ > 0 &&
          accepted >= persistent_point_map_max_correspondences_)
        {
          break;
        }
      }
      if (!allow_map_update) {
        ++persistent_point_map_update_skips_;
      } else
      {
        // Same-scan self matches collapse motion. Keep associations
        // history-based by publishing this scan's map updates only after all
        // correspondences for the scan have been built.
        pending_point_updates.push_back(point_world);
      }
    }
    persistent_point_map_updates_ +=
      commit_persistent_point_map_updates(stamp_ns, pending_point_updates);
    return accepted;
  }

  int add_gaussian_snapshot_correspondences(
    int64_t stamp_ns,
    const std::vector<Eigen::Vector3d> & points_lidar,
    const Eigen::Quaterniond & q_w_i,
    const Eigen::Vector3d & p_w_i,
    const spline::LidarExtrinsics & extrinsics)
  {
    if (!gaussian_snapshot_.complete() || points_lidar.empty() ||
      !q_w_i.coeffs().allFinite() || !p_w_i.allFinite())
    {
      ++gaussian_snapshot_lidar_factor_skips_;
      return 0;
    }

    if (gaussian_snapshot_.point_count() == 0U) {
      ++gaussian_snapshot_lidar_factor_skips_;
      return 0;
    }

    int accepted = 0;
    int stride_counter = 0;
    const double max_distance_sq =
      gaussian_snapshot_lidar_nearest_distance_m_ *
      gaussian_snapshot_lidar_nearest_distance_m_;
    for (const auto & point_lidar : points_lidar) {
      if (gaussian_snapshot_lidar_subsample_stride_ > 1 &&
        stride_counter++ % gaussian_snapshot_lidar_subsample_stride_ != 0)
      {
        continue;
      }
      const double range = point_lidar.norm();
      if (!point_lidar.allFinite() || !std::isfinite(range) ||
        range < pointcloud_min_range_m_ || range > pointcloud_max_range_m_)
      {
        continue;
      }

      const Eigen::Vector3d point_imu =
        extrinsics.q_lidar_to_imu * point_lidar + extrinsics.p_lidar_in_imu;
      const Eigen::Vector3d point_world = q_w_i * point_imu + p_w_i;
      const Eigen::Vector3d point_map =
        extrinsics.q_world_to_map * point_world + extrinsics.p_world_in_map;
      if (!point_map.allFinite()) {
        continue;
      }

      const auto nearest = gaussian_snapshot_.find_nearest(
        point_map,
        gaussian_snapshot_lidar_nearest_distance_m_,
        gaussian_snapshot_lidar_min_opacity_,
        gaussian_snapshot_map_subsample_stride_);
      if (!nearest.matched || nearest.distance_sq > max_distance_sq) {
        continue;
      }

      const double feature_scale =
        lidar_surface_feature_scale(std::sqrt(nearest.distance_sq), range);
      if (!lidar_feature_scale_is_accepted(feature_scale)) {
        continue;
      }
      estimator_->add_lidar_point_to_point_correspondence(
        stamp_ns, point_lidar, nearest.xyz, extrinsics,
        gaussian_snapshot_lidar_factor_weight_, lidar_huber_delta_m_, feature_scale);
      ++accepted;
      ++gaussian_snapshot_lidar_matches_;
      if (gaussian_snapshot_lidar_max_correspondences_ > 0 &&
        accepted >= gaussian_snapshot_lidar_max_correspondences_)
      {
        break;
      }
    }
    if (accepted == 0) {
      ++gaussian_snapshot_lidar_factor_skips_;
    }
    return accepted;
  }

  std::vector<Eigen::Vector3d> make_deskewed_points_imu(
    int64_t scan_stamp_ns,
    const std::vector<Eigen::Vector3d> & points_lidar,
    const std::vector<int64_t> & point_stamps_ns,
    const Eigen::Quaterniond & q_w_i_scan,
    const Eigen::Vector3d & p_w_i_scan,
    const spline::LidarExtrinsics & extrinsics)
  {
    std::vector<Eigen::Vector3d> points_imu;
    points_imu.reserve(points_lidar.size());
    const Eigen::Quaterniond q_scan = q_w_i_scan.normalized();
    const Eigen::Quaterniond q_scan_inv = q_scan.inverse();
    for (size_t index = 0U; index < points_lidar.size(); ++index) {
      const auto & point_lidar = points_lidar[index];
      const double range = point_lidar.norm();
      if (!point_lidar.allFinite() || !std::isfinite(range) ||
        range < pointcloud_min_range_m_ || range > pointcloud_max_range_m_)
      {
        continue;
      }
      const Eigen::Vector3d raw_point_imu =
        extrinsics.q_lidar_to_imu * point_lidar + extrinsics.p_lidar_in_imu;
      Eigen::Vector3d point_imu = raw_point_imu;
      if (enable_lidar_point_deskew_ && index < point_stamps_ns.size() &&
        point_stamps_ns[index] != scan_stamp_ns && estimator_ != nullptr)
      {
        Eigen::Quaterniond q_w_i_point;
        Eigen::Vector3d p_w_i_point;
        if (estimator_->query_pose(point_stamps_ns[index], q_w_i_point, p_w_i_point) &&
          q_w_i_point.coeffs().allFinite() && p_w_i_point.allFinite())
        {
          const Eigen::Vector3d point_w =
            q_w_i_point.normalized() * raw_point_imu + p_w_i_point;
          point_imu = q_scan_inv * (point_w - p_w_i_scan);
          const double deskew_delta_m = (point_imu - raw_point_imu).norm();
          if (point_imu.allFinite() &&
            (lidar_max_deskew_delta_m_ == 0.0 || deskew_delta_m <= lidar_max_deskew_delta_m_))
          {
            ++pointcloud_deskewed_points_;
            pointcloud_last_max_deskew_m_ = std::max(
              pointcloud_last_max_deskew_m_, deskew_delta_m);
          } else {
            point_imu = raw_point_imu;
            ++pointcloud_deskew_fallback_points_;
          }
        } else {
          ++pointcloud_deskew_fallback_points_;
        }
      }
      points_imu.push_back(point_imu);
    }
    return points_imu;
  }

  void maybe_add_lidar_scan_to_scan_prior_locked(
    int64_t stamp_ns,
    const std::vector<Eigen::Vector3d> & points_lidar,
    const std::vector<int64_t> & point_stamps_ns,
    const Eigen::Quaterniond & q_w_i,
    const Eigen::Vector3d & p_w_i,
    const spline::LidarExtrinsics & extrinsics,
    bool have_scan_pose)
  {
    if (!enable_lidar_scan_to_scan_prior_ || !have_scan_pose ||
      !q_w_i.coeffs().allFinite() || !p_w_i.allFinite() ||
      (lidar_scan_to_scan_velocity_weight_ <= 0.0 &&
      lidar_scan_to_scan_acceleration_weight_ <= 0.0 &&
      lidar_scan_to_scan_angular_velocity_weight_ <= 0.0 &&
      lidar_scan_to_scan_position_weight_ <= 0.0 &&
      lidar_scan_to_scan_orientation_weight_ <= 0.0))
    {
      return;
    }
    const std::vector<Eigen::Vector3d> points_imu =
      make_deskewed_points_imu(
      stamp_ns, points_lidar, point_stamps_ns, q_w_i, p_w_i, extrinsics);
    if (points_imu.empty()) {
      return;
    }

    TrajectoryPose current_pose;
    current_pose.stamp_ns = stamp_ns;
    current_pose.q_w_i = q_w_i.normalized();
    current_pose.p_w_i = p_w_i;
    if (!have_last_lidar_scan_to_scan_) {
      last_lidar_scan_to_scan_points_imu_ = points_imu;
      last_lidar_scan_to_scan_pose_ = current_pose;
      have_last_lidar_scan_to_scan_ = true;
      return;
    }

    const double dt_s =
      static_cast<double>(stamp_ns - last_lidar_scan_to_scan_pose_.stamp_ns) * 1.0e-9;
    if (!std::isfinite(dt_s) || dt_s <= 1.0e-6) {
      ++lidar_scan_to_scan_rejected_;
      last_lidar_scan_to_scan_points_imu_ = points_imu;
      last_lidar_scan_to_scan_pose_ = current_pose;
      have_last_lidar_scan_to_scan_velocity_ = false;
      return;
    }

    TrajectoryPose predicted_current_pose = current_pose;
    if (lidar_scan_to_scan_use_odometry_prediction_ &&
      have_last_lidar_scan_to_scan_velocity_)
    {
      predicted_current_pose.stamp_ns = stamp_ns;
      predicted_current_pose.p_w_i =
        last_lidar_scan_to_scan_pose_.p_w_i +
        last_lidar_scan_to_scan_velocity_world_ * dt_s;
      predicted_current_pose.q_w_i =
        (last_lidar_scan_to_scan_pose_.q_w_i *
        spline::quaternion_exp(last_lidar_scan_to_scan_angular_velocity_body_ * dt_s)).normalized();
      if (!predicted_current_pose.p_w_i.allFinite() ||
        !predicted_current_pose.q_w_i.coeffs().allFinite() ||
        predicted_current_pose.q_w_i.norm() <= 1.0e-9)
      {
        predicted_current_pose = current_pose;
        have_last_lidar_scan_to_scan_velocity_ = false;
      }
    }

    LidarFactor scan_matcher(lidar_pose_factor_.config());
    TrajectoryPose previous_identity;
    previous_identity.stamp_ns = last_lidar_scan_to_scan_pose_.stamp_ns;
    previous_identity.q_w_i = Eigen::Quaterniond::Identity();
    previous_identity.p_w_i = Eigen::Vector3d::Zero();
    scan_matcher.insert_keyframe(last_lidar_scan_to_scan_points_imu_, previous_identity);

    TrajectoryPose predicted_relative;
    predicted_relative.stamp_ns = stamp_ns;
    predicted_relative.q_w_i =
      (last_lidar_scan_to_scan_pose_.q_w_i.inverse() *
      predicted_current_pose.q_w_i).normalized();
    predicted_relative.p_w_i =
      last_lidar_scan_to_scan_pose_.q_w_i.inverse() *
      (predicted_current_pose.p_w_i - last_lidar_scan_to_scan_pose_.p_w_i);
    lidar_scan_to_scan_last_predicted_relative_translation_m_ =
      predicted_relative.p_w_i.norm();
    LidarPoseCorrection correction;
    bool used_point_to_plane = false;
    if (lidar_scan_to_scan_use_point_to_plane_correction_) {
      correction =
        scan_matcher.compute_point_to_plane_pose_correction(points_imu, predicted_relative);
      used_point_to_plane = correction.applied;
    }
    if (!correction.applied) {
      correction = scan_matcher.compute_pose_correction(points_imu, predicted_relative);
      if (correction.applied) {
        ++lidar_scan_to_scan_point_to_point_fallbacks_;
      }
    }
    if (!correction.applied) {
      ++lidar_scan_to_scan_rejected_;
      last_lidar_scan_to_scan_points_imu_ = points_imu;
      if (lidar_scan_to_scan_use_odometry_prediction_ &&
        lidar_scan_to_scan_dead_reckon_on_reject_ &&
        have_last_lidar_scan_to_scan_velocity_)
      {
        last_lidar_scan_to_scan_pose_ = predicted_current_pose;
        ++lidar_scan_to_scan_dead_reckoned_;
      } else {
        last_lidar_scan_to_scan_pose_ = current_pose;
        have_last_lidar_scan_to_scan_velocity_ = false;
      }
      return;
    }
    if (used_point_to_plane) {
      ++lidar_scan_to_scan_point_to_plane_corrections_;
    }

    const Eigen::Quaterniond matched_relative_q =
      (correction.delta_q * predicted_relative.q_w_i).normalized();
    const bool use_scan_matched_rotation =
      lidar_scan_to_scan_angular_velocity_weight_ > 0.0 ||
      lidar_scan_to_scan_orientation_weight_ > 0.0 ||
      (lidar_scan_to_scan_apply_pose_seed_ &&
      lidar_scan_to_scan_pose_seed_rotation_gain_ > 0.0);
    Eigen::Quaterniond target_relative_q = use_scan_matched_rotation
      ? matched_relative_q
      : predicted_relative.q_w_i;
    Eigen::Vector3d target_relative_p =
      predicted_relative.p_w_i + correction.delta_p_w;
    target_relative_p *= lidar_scan_to_scan_relative_translation_gain_;
    lidar_scan_to_scan_last_raw_target_relative_translation_m_ = target_relative_p.norm();
    const double predicted_relative_norm = predicted_relative.p_w_i.norm();
    double target_prediction_ratio = 1.0;
    if (std::isfinite(predicted_relative_norm) && predicted_relative_norm > 1.0e-12) {
      target_prediction_ratio =
        lidar_scan_to_scan_last_raw_target_relative_translation_m_ / predicted_relative_norm;
    }
    if (!std::isfinite(target_prediction_ratio)) {
      target_prediction_ratio = 0.0;
    }
    lidar_scan_to_scan_last_target_prediction_ratio_ = target_prediction_ratio;
    const bool small_by_ratio =
      lidar_scan_to_scan_min_target_prediction_ratio_ > 0.0 &&
      predicted_relative_norm > std::max(1.0e-9, lidar_scan_to_scan_min_target_translation_m_) &&
      target_prediction_ratio < lidar_scan_to_scan_min_target_prediction_ratio_;
    const bool small_by_absolute =
      lidar_scan_to_scan_min_target_translation_m_ > 0.0 &&
      predicted_relative_norm > lidar_scan_to_scan_min_target_translation_m_ &&
      lidar_scan_to_scan_last_raw_target_relative_translation_m_ <
      lidar_scan_to_scan_min_target_translation_m_;
    const bool small_translation_target =
      (small_by_ratio || small_by_absolute) && predicted_relative.p_w_i.allFinite();
    bool skip_translation_priors = false;
    if (small_translation_target) {
      ++lidar_scan_to_scan_small_translation_targets_;
      if (lidar_scan_to_scan_use_prediction_on_small_target_ ||
        lidar_scan_to_scan_skip_translation_priors_on_small_target_)
      {
        target_relative_p = predicted_relative.p_w_i;
        if (lidar_scan_to_scan_use_prediction_on_small_target_) {
          ++lidar_scan_to_scan_prediction_translation_fallbacks_;
        }
        skip_translation_priors = lidar_scan_to_scan_skip_translation_priors_on_small_target_;
      }
    }
    Eigen::Vector3d target_relative_velocity = target_relative_p / dt_s;
    Eigen::Vector3d target_angular_velocity =
      spline::quaternion_log(target_relative_q) / dt_s;
    if (lidar_scan_to_scan_yaw_only_angular_velocity_) {
      target_angular_velocity.x() = 0.0;
      target_angular_velocity.y() = 0.0;
    }
    if (target_relative_velocity.allFinite() && lidar_scan_to_scan_max_velocity_mps_ > 0.0) {
      const double speed = target_relative_velocity.norm();
      if (speed > lidar_scan_to_scan_max_velocity_mps_ && speed > 1.0e-12) {
        target_relative_velocity *= lidar_scan_to_scan_max_velocity_mps_ / speed;
        target_relative_p = target_relative_velocity * dt_s;
        ++lidar_scan_to_scan_velocity_clamped_;
      }
    }
    if (
      target_angular_velocity.allFinite() &&
      lidar_scan_to_scan_max_angular_velocity_radps_ > 0.0)
    {
      const double angular_speed = target_angular_velocity.norm();
      if (
        angular_speed > lidar_scan_to_scan_max_angular_velocity_radps_ &&
        angular_speed > 1.0e-12)
      {
        target_angular_velocity *= lidar_scan_to_scan_max_angular_velocity_radps_ / angular_speed;
        target_relative_q =
          spline::quaternion_exp(target_angular_velocity * dt_s).normalized();
        ++lidar_scan_to_scan_angular_velocity_clamped_;
      }
    }
    const Eigen::Quaterniond target_q =
      (last_lidar_scan_to_scan_pose_.q_w_i * target_relative_q).normalized();
    const Eigen::Vector3d target_p =
      last_lidar_scan_to_scan_pose_.p_w_i +
      last_lidar_scan_to_scan_pose_.q_w_i * target_relative_p;
    const Eigen::Vector3d target_velocity =
      (target_p - last_lidar_scan_to_scan_pose_.p_w_i) / dt_s;
    Eigen::Vector3d target_acceleration = Eigen::Vector3d::Zero();
    bool have_target_acceleration =
      have_last_lidar_scan_to_scan_velocity_ && target_velocity.allFinite();
    if (have_target_acceleration) {
      target_acceleration =
        (target_velocity - last_lidar_scan_to_scan_velocity_world_) / dt_s;
      have_target_acceleration = target_acceleration.allFinite();
      if (have_target_acceleration && lidar_scan_to_scan_max_acceleration_mps2_ > 0.0) {
        const double accel_norm = target_acceleration.norm();
        if (accel_norm > lidar_scan_to_scan_max_acceleration_mps2_ && accel_norm > 1.0e-12) {
          target_acceleration *= lidar_scan_to_scan_max_acceleration_mps2_ / accel_norm;
          ++lidar_scan_to_scan_acceleration_clamped_;
        }
      }
    }

    if (lidar_scan_to_scan_apply_pose_seed_) {
      const bool pose_seed_applied = estimator_->apply_pose_hint(
        stamp_ns, target_q, target_p,
        lidar_scan_to_scan_pose_seed_position_gain_,
        lidar_scan_to_scan_pose_seed_rotation_gain_);
      if (pose_seed_applied) {
        ++lidar_scan_to_scan_pose_seed_updates_;
      } else {
        ++lidar_scan_to_scan_pose_seed_rejected_;
      }
    }

    if (lidar_scan_to_scan_position_weight_ > 0.0) {
      if (skip_translation_priors) {
        ++lidar_scan_to_scan_translation_priors_skipped_;
      } else if (lidar_scan_to_scan_use_relative_pose_factor_) {
        estimator_->add_relative_position_prior(
          last_lidar_scan_to_scan_pose_.stamp_ns, stamp_ns, target_relative_p,
          lidar_scan_to_scan_position_weight_,
          lidar_scan_to_scan_position_huber_delta_m_);
      } else {
        estimator_->add_position_prior(
          stamp_ns, target_p, lidar_scan_to_scan_position_weight_,
          lidar_scan_to_scan_position_huber_delta_m_);
      }
      if (!skip_translation_priors) {
        ++lidar_scan_to_scan_position_priors_;
      }
    }
    if (lidar_scan_to_scan_orientation_weight_ > 0.0) {
      if (lidar_scan_to_scan_use_relative_pose_factor_) {
        estimator_->add_relative_orientation_prior(
          last_lidar_scan_to_scan_pose_.stamp_ns, stamp_ns, target_relative_q,
          lidar_scan_to_scan_orientation_weight_,
          lidar_scan_to_scan_orientation_huber_delta_rad_);
      } else {
        estimator_->add_orientation_prior(
          stamp_ns, target_q, lidar_scan_to_scan_orientation_weight_,
          lidar_scan_to_scan_orientation_huber_delta_rad_);
      }
      ++lidar_scan_to_scan_orientation_priors_;
    }
    if (lidar_scan_to_scan_velocity_weight_ > 0.0) {
      if (skip_translation_priors) {
        ++lidar_scan_to_scan_translation_priors_skipped_;
      } else if (target_velocity.allFinite()) {
        estimator_->add_velocity_prior(
          stamp_ns, target_velocity, lidar_scan_to_scan_velocity_weight_,
          lidar_scan_to_scan_velocity_huber_delta_mps_);
        ++lidar_scan_to_scan_velocity_priors_;
      }
    }
    if (lidar_scan_to_scan_acceleration_weight_ > 0.0) {
      if (skip_translation_priors) {
        ++lidar_scan_to_scan_translation_priors_skipped_;
      } else if (have_target_acceleration) {
        estimator_->add_acceleration_prior(
          stamp_ns, target_acceleration, lidar_scan_to_scan_acceleration_weight_,
          lidar_scan_to_scan_acceleration_huber_delta_mps2_);
        ++lidar_scan_to_scan_acceleration_priors_;
      }
    }
    if (lidar_scan_to_scan_angular_velocity_weight_ > 0.0) {
      if (target_angular_velocity.allFinite()) {
        estimator_->add_angular_velocity_prior(
          stamp_ns, target_angular_velocity, lidar_scan_to_scan_angular_velocity_weight_,
          lidar_scan_to_scan_angular_velocity_huber_delta_radps_);
        ++lidar_scan_to_scan_angular_velocity_priors_;
      }
    }
    last_lidar_scan_to_scan_velocity_world_ = target_velocity;
    last_lidar_scan_to_scan_angular_velocity_body_ = target_angular_velocity;
    have_last_lidar_scan_to_scan_velocity_ =
      last_lidar_scan_to_scan_velocity_world_.allFinite() &&
      last_lidar_scan_to_scan_angular_velocity_body_.allFinite();

    ++lidar_scan_to_scan_priors_;
    lidar_scan_to_scan_matches_ += correction.matched_points;
    lidar_scan_to_scan_last_residual_m_ = correction.mean_residual_m;
    lidar_scan_to_scan_last_correction_translation_m_ = correction.delta_p_w.norm();
    lidar_scan_to_scan_last_correction_rotation_rad_ =
      std::abs(Eigen::AngleAxisd(correction.delta_q).angle());
    lidar_scan_to_scan_last_target_relative_translation_m_ = target_relative_p.norm();
    lidar_scan_to_scan_last_target_speed_mps_ = target_velocity.norm();
    lidar_scan_to_scan_last_target_angular_speed_radps_ = target_angular_velocity.norm();
    lidar_scan_to_scan_cumulative_target_path_m_ +=
      (target_p - last_lidar_scan_to_scan_pose_.p_w_i).norm();
    last_lidar_scan_to_scan_points_imu_ = points_imu;
    last_lidar_scan_to_scan_pose_ = current_pose;
    if (lidar_scan_to_scan_store_corrected_pose_) {
      last_lidar_scan_to_scan_pose_.p_w_i = target_p;
      last_lidar_scan_to_scan_pose_.q_w_i = target_q;
    }
  }

  void maybe_add_lidar_pose_prior_locked(
    int64_t stamp_ns,
    const std::vector<Eigen::Vector3d> & points_lidar,
    const std::vector<int64_t> & point_stamps_ns,
    const Eigen::Quaterniond & q_w_i,
    const Eigen::Vector3d & p_w_i,
    const spline::LidarExtrinsics & extrinsics,
    bool have_scan_pose)
  {
    if (!enable_lidar_pose_prior_factor_ || !have_scan_pose ||
      !q_w_i.coeffs().allFinite() || !p_w_i.allFinite())
    {
      return;
    }
    const std::vector<Eigen::Vector3d> points_imu =
      make_deskewed_points_imu(
      stamp_ns, points_lidar, point_stamps_ns, q_w_i, p_w_i, extrinsics);
    if (points_imu.empty()) {
      return;
    }

    TrajectoryPose predicted_pose;
    predicted_pose.stamp_ns = stamp_ns;
    predicted_pose.q_w_i = q_w_i.normalized();
    predicted_pose.p_w_i = p_w_i;
    if (!lidar_pose_factor_has_keyframe_) {
      lidar_pose_factor_.insert_keyframe(points_imu, predicted_pose);
      lidar_pose_factor_has_keyframe_ = true;
      ++lidar_pose_factor_keyframes_;
      return;
    }

    const auto correction = lidar_pose_factor_.compute_pose_correction(points_imu, predicted_pose);
    if (!correction.applied) {
      ++lidar_pose_prior_rejected_;
      return;
    }

    const Eigen::Vector3d target_position = predicted_pose.p_w_i + correction.delta_p_w;
    const Eigen::Quaterniond target_orientation =
      (correction.delta_q * predicted_pose.q_w_i).normalized();
    if (lidar_pose_prior_position_weight_ > 0.0) {
      estimator_->add_position_prior(
        stamp_ns, target_position, lidar_pose_prior_position_weight_,
        lidar_pose_prior_position_huber_delta_m_);
    }
    if ((lidar_pose_prior_velocity_weight_ > 0.0 ||
      lidar_pose_prior_acceleration_weight_ > 0.0) &&
      have_last_lidar_pose_prior_)
    {
      const double dt_s =
        static_cast<double>(stamp_ns - last_lidar_pose_prior_stamp_ns_) * 1.0e-9;
      if (std::isfinite(dt_s) && dt_s > 1.0e-6) {
        const Eigen::Vector3d target_velocity =
          (target_position - last_lidar_pose_prior_position_) / dt_s;
        if (target_velocity.allFinite()) {
          if (lidar_pose_prior_velocity_weight_ > 0.0) {
            estimator_->add_velocity_prior(
              stamp_ns, target_velocity, lidar_pose_prior_velocity_weight_,
              lidar_pose_prior_velocity_huber_delta_mps_);
            ++lidar_pose_prior_velocity_factors_;
          }
          if (lidar_pose_prior_acceleration_weight_ > 0.0 &&
            have_last_lidar_pose_prior_velocity_)
          {
            const Eigen::Vector3d target_acceleration =
              (target_velocity - last_lidar_pose_prior_velocity_world_) / dt_s;
            if (target_acceleration.allFinite()) {
              estimator_->add_acceleration_prior(
                stamp_ns, target_acceleration, lidar_pose_prior_acceleration_weight_,
                lidar_pose_prior_acceleration_huber_delta_mps2_);
              ++lidar_pose_prior_acceleration_factors_;
            }
          }
          last_lidar_pose_prior_velocity_world_ = target_velocity;
          have_last_lidar_pose_prior_velocity_ = true;
        }
      }
    } else if (!have_last_lidar_pose_prior_) {
      have_last_lidar_pose_prior_velocity_ = false;
    }
    if (lidar_pose_prior_angular_velocity_weight_ > 0.0 && have_last_lidar_pose_prior_) {
      const double dt_s =
        static_cast<double>(stamp_ns - last_lidar_pose_prior_stamp_ns_) * 1.0e-9;
      if (std::isfinite(dt_s) && dt_s > 1.0e-6) {
        const Eigen::Quaterniond delta_q =
          (last_lidar_pose_prior_orientation_.inverse() * target_orientation).normalized();
        const Eigen::Vector3d target_angular_velocity = spline::quaternion_log(delta_q) / dt_s;
        if (target_angular_velocity.allFinite()) {
          estimator_->add_angular_velocity_prior(
            stamp_ns, target_angular_velocity, lidar_pose_prior_angular_velocity_weight_,
            lidar_pose_prior_angular_velocity_huber_delta_radps_);
          ++lidar_pose_prior_angular_velocity_factors_;
        }
      }
    }
    if (lidar_pose_prior_orientation_weight_ > 0.0) {
      estimator_->add_orientation_prior(
        stamp_ns, target_orientation, lidar_pose_prior_orientation_weight_,
        lidar_pose_prior_orientation_huber_delta_rad_);
    }
    last_lidar_pose_prior_stamp_ns_ = stamp_ns;
    last_lidar_pose_prior_position_ = target_position;
    last_lidar_pose_prior_orientation_ = target_orientation;
    have_last_lidar_pose_prior_ = true;
    ++lidar_pose_prior_factors_;
    lidar_pose_prior_matches_ += correction.matched_points;
    lidar_pose_prior_last_mean_residual_m_ = correction.mean_residual_m;

    ++lidar_pose_factor_seen_frames_;
    if (
      lidar_pose_factor_seen_frames_ %
        static_cast<std::size_t>(lidar_pose_factor_keyframe_stride_) == 0U)
    {
      TrajectoryPose corrected_pose = predicted_pose;
      corrected_pose.p_w_i = target_position;
      corrected_pose.q_w_i = target_orientation;
      lidar_pose_factor_.insert_keyframe(points_imu, corrected_pose);
      ++lidar_pose_factor_keyframes_;
    }
  }

  void seed_window_from_buffer()
  {
    if (seed_imu_buffer_.empty() || !estimator_) {
      return;
    }
    const int64_t start_stamp = seed_imu_buffer_.front().first;

    // Gravity autocalibration: assume the seed window is static, average
    // accelerometer samples to recover the gravity direction in body frame,
    // then derive a roll/pitch rotation that aligns body z with -gravity_world.
    // Yaw stays 0 (not observable from gravity alone). Mirrors the upstream
    // Coco-LIC / `tracking_node` `enable_imu_gravity_autocalibration` default.
    Eigen::Quaterniond seed_orientation = Eigen::Quaterniond::Identity();
    Eigen::Vector3d seed_position = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel_mean = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_mean = Eigen::Vector3d::Zero();
    for (const auto & sample : seed_imu_buffer_) {
      gyro_mean += sample.second.gyro;
      accel_mean += sample.second.accel;
    }
    const double inv_seed_count = 1.0 / static_cast<double>(seed_imu_buffer_.size());
    gyro_mean *= inv_seed_count;
    accel_mean *= inv_seed_count;

    // External pose prior takes precedence over gravity autocal — if a
    // ground-truth or external SLAM frontend has been publishing on
    // `external_odometry_prior_topic`, use the most recent pose as the
    // seed instead of identity. The prior is composed with the configured
    // IMU mount rotation so the resulting orientation expresses the IMU
    // body in the world frame the estimator works in.
    if (enable_external_odometry_prior_ && have_prior_pose_) {
      seed_orientation =
        (latest_prior_orientation_ * prior_to_imu_rotation_).normalized();
      seed_position = latest_prior_position_;
      RCLCPP_INFO(
        get_logger(),
        "seed from external prior + mount: p=(%.3f, %.3f, %.3f) q=(%.3f, %.3f, %.3f, %.3f)",
        seed_position.x(), seed_position.y(), seed_position.z(),
        seed_orientation.w(), seed_orientation.x(),
        seed_orientation.y(), seed_orientation.z());
    } else if (enable_imu_gravity_autocal_) {
      const double accel_mean_norm = accel_mean.norm();
      if (accel_mean_norm > 1.0e-3) {
        const Eigen::Vector3d gravity_world = estimator_->gravity_world();
        const double gravity_world_norm = gravity_world.norm();
        if (gravity_world_norm > 1.0e-3) {
          // Stationary accel reads `R_b_w^T * (-g_w)` (no motion + zero bias
          // initially). So `R_b_w^T * (-g_w) = accel_mean` → we need a
          // rotation that maps `-g_w/|g_w|` (body z when level) onto the
          // measured direction `accel_mean/|accel_mean|`.
          const Eigen::Vector3d gravity_dir = (-gravity_world / gravity_world_norm);
          const Eigen::Vector3d accel_dir = accel_mean / accel_mean_norm;
          seed_orientation = Eigen::Quaterniond::FromTwoVectors(
            gravity_dir, accel_dir).inverse().normalized();
        }
      }
      RCLCPP_INFO(
        get_logger(),
        "gravity autocal: accel_mean=(%.3f, %.3f, %.3f) -> q_w_b=(%.3f, %.3f, %.3f, %.3f)",
        accel_mean.x(), accel_mean.y(), accel_mean.z(),
        seed_orientation.w(), seed_orientation.x(),
        seed_orientation.y(), seed_orientation.z());
    }

    if (enable_startup_bias_autocal_) {
      const Eigen::Vector3d predicted_stationary_accel =
        seed_orientation.inverse() * (-estimator_->gravity_world());
      const Eigen::Vector3d startup_accel_bias = accel_mean - predicted_stationary_accel;
      const Eigen::Vector3d startup_gyro_bias = gyro_mean;
      estimator_->set_accel_bias(startup_accel_bias);
      estimator_->set_gyro_bias(startup_gyro_bias);
      RCLCPP_INFO(
        get_logger(),
        "startup bias autocal: gyro=(%.5f, %.5f, %.5f) accel=(%.5f, %.5f, %.5f)",
        startup_gyro_bias.x(), startup_gyro_bias.y(), startup_gyro_bias.z(),
        startup_accel_bias.x(), startup_accel_bias.y(), startup_accel_bias.z());
    }

    std::vector<Eigen::Quaterniond> rot_knots(
      static_cast<std::size_t>(estimator_->N + 4),
      seed_orientation);
    std::vector<Eigen::Vector3d> pos_knots(
      rot_knots.size(), seed_position);
    estimator_->initialize(start_stamp, rot_knots, pos_knots);

    for (const auto & sample : seed_imu_buffer_) {
      estimator_->add_imu_sample(sample.first, sample.second);
    }
    estimator_->step();
    last_stamp_driven_step_ns_ = start_stamp;
    seed_imu_buffer_.clear();
    RCLCPP_INFO(
      get_logger(),
      "continuous_time_node seeded window with %d IMU samples", seed_min_imu_count_);
  }

  void publish_latest_pose()
  {
    if (!estimator_) {
      return;
    }
    const int64_t newest = estimator_->newest_knot_stamp_ns();
    const int64_t query_ns = newest - 2 * knot_interval_ns_;
    if (pose_output_period_ns_ > 0) {
      if (last_published_query_ns_ == 0) {
        publish_pose_at(query_ns);
        return;
      }
      int64_t next_query_ns = last_published_query_ns_ + pose_output_period_ns_;
      while (next_query_ns <= query_ns) {
        if (!publish_pose_at(next_query_ns)) {
          break;
        }
        next_query_ns += pose_output_period_ns_;
      }
      return;
    }
    publish_pose_at(query_ns);
  }

  bool publish_pose_at(int64_t query_ns)
  {
    // Rate limit: only publish when query stamp advances. The Ceres step
    // can fire many times per timer batch when the executor catches up
    // after a busy IMU/PointCloud callback; without this guard, the same
    // pose can be emitted hundreds of times.
    if (last_published_query_ns_ != 0 && query_ns <= last_published_query_ns_) {
      return false;
    }
    Eigen::Quaterniond q;
    Eigen::Vector3d p;
    if (!estimator_->query_pose(query_ns, q, p)) {
      return false;
    }
    last_published_query_ns_ = query_ns;
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = nanoseconds_to_stamp(query_ns);
    odom.header.frame_id = world_frame_id_;
    odom.child_frame_id = body_frame_id_;
    odom.pose.pose.position.x = p.x();
    odom.pose.pose.position.y = p.y();
    odom.pose.pose.position.z = p.z();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    odom_publisher_->publish(odom);

    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = odom.header;
    pose_stamped.pose = odom.pose.pose;
    if (static_cast<int>(published_path_.poses.size()) >= max_path_history_) {
      published_path_.poses.erase(
        published_path_.poses.begin(),
        published_path_.poses.begin() +
        static_cast<int>(published_path_.poses.size()) - max_path_history_ + 1);
    }
    published_path_.header = pose_stamped.header;
    published_path_.poses.push_back(pose_stamped);
    path_publisher_->publish(published_path_);

    if (output_tum_stream_.is_open()) {
      const double stamp_s = static_cast<double>(query_ns) * 1.0e-9;
      const double position_bound = 1.0e6;
      if (p.allFinite() && q.coeffs().allFinite() &&
        std::abs(p.x()) < position_bound &&
        std::abs(p.y()) < position_bound &&
        std::abs(p.z()) < position_bound)
      {
        std::ostringstream line;
        line.setf(std::ios::fixed);
        line.precision(9);
        line << stamp_s;
        line.precision(6);
        line << ' ' << p.x() << ' ' << p.y() << ' ' << p.z();
        line << ' ' << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w();
        output_tum_stream_ << line.str() << '\n';
        output_tum_stream_.flush();
        ++tum_lines_written_;
      }
    }
    return true;
  }

  std::string raw_imu_topic_;
  std::string raw_pointcloud_topic_;
  std::string raw_image_topic_;
  std::string raw_camera_info_topic_;
  std::string external_odometry_prior_topic_;
  std::string odometry_topic_;
  std::string path_topic_;
  std::string gaussian_map_topic_;
  int gaussian_snapshot_qos_depth_{64};
  std::string body_frame_id_;
  std::string world_frame_id_;

  std::unique_ptr<spline::ContinuousTimeSlidingWindowEstimator> estimator_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
  rclcpp::Subscription<gaussian_lic_msgs::msg::GaussianArray>::SharedPtr
    gaussian_map_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr external_odometry_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::TimerBase::SharedPtr step_timer_;

  std::mutex estimator_mutex_;
  bool initialized_{false};
  std::vector<std::pair<int64_t, spline::ImuSample>> seed_imu_buffer_;
  int seed_min_imu_count_{25};
  int max_path_history_{5000};
  nav_msgs::msg::Path published_path_;

  bool last_imu_stamp_valid_{false};
  int64_t last_imu_stamp_ns_{0};
  std::size_t accepted_imu_count_{0};
  std::size_t dropped_imu_count_{0};
  std::size_t rejected_imu_count_{0};

  std::string output_tum_path_;
  std::ofstream output_tum_stream_;
  std::size_t tum_lines_written_{0};

  bool pointcloud_enable_{true};
  int pointcloud_subsample_stride_{50};
  int pointcloud_max_points_per_msg_{256};
  int pointcloud_wait_queue_max_size_{100};
  double pointcloud_min_range_m_{0.3};
  double pointcloud_max_range_m_{30.0};
  double pointcloud_factor_weight_{0.1};
  bool pointcloud_use_lidar_scale_{true};
  double pointcloud_min_lidar_scale_{0.1};
  bool enable_lidar_point_deskew_{false};
  double lidar_max_abs_point_time_offset_s_{0.25};
  double lidar_max_deskew_delta_m_{1.0};
  double lidar_huber_delta_m_{0.10};
  bool enable_lidar_pose_prior_factor_{false};
  double lidar_pose_prior_position_weight_{1.0};
  double lidar_pose_prior_velocity_weight_{0.0};
  double lidar_pose_prior_acceleration_weight_{0.0};
  double lidar_pose_prior_angular_velocity_weight_{0.0};
  double lidar_pose_prior_orientation_weight_{1.0};
  double lidar_pose_prior_position_huber_delta_m_{0.25};
  double lidar_pose_prior_velocity_huber_delta_mps_{0.25};
  double lidar_pose_prior_acceleration_huber_delta_mps2_{0.50};
  double lidar_pose_prior_angular_velocity_huber_delta_radps_{0.25};
  double lidar_pose_prior_orientation_huber_delta_rad_{0.25};
  int lidar_pose_factor_keyframe_stride_{5};
  int lidar_pose_factor_iterations_{1};
  bool enable_lidar_scan_to_scan_prior_{false};
  double lidar_scan_to_scan_position_weight_{0.0};
  double lidar_scan_to_scan_velocity_weight_{0.0};
  double lidar_scan_to_scan_acceleration_weight_{0.0};
  double lidar_scan_to_scan_angular_velocity_weight_{0.0};
  double lidar_scan_to_scan_orientation_weight_{0.0};
  double lidar_scan_to_scan_position_huber_delta_m_{0.25};
  double lidar_scan_to_scan_velocity_huber_delta_mps_{0.25};
  double lidar_scan_to_scan_acceleration_huber_delta_mps2_{0.50};
  double lidar_scan_to_scan_angular_velocity_huber_delta_radps_{0.25};
  double lidar_scan_to_scan_orientation_huber_delta_rad_{0.25};
  double lidar_scan_to_scan_max_velocity_mps_{0.0};
  double lidar_scan_to_scan_max_acceleration_mps2_{0.0};
  double lidar_scan_to_scan_max_angular_velocity_radps_{0.0};
  double lidar_scan_to_scan_relative_translation_gain_{1.0};
  double lidar_scan_to_scan_min_target_prediction_ratio_{0.0};
  double lidar_scan_to_scan_min_target_translation_m_{0.0};
  bool lidar_scan_to_scan_use_odometry_prediction_{false};
  bool lidar_scan_to_scan_use_point_to_plane_correction_{false};
  bool lidar_scan_to_scan_use_relative_pose_factor_{false};
  bool lidar_scan_to_scan_use_prediction_on_small_target_{false};
  bool lidar_scan_to_scan_skip_translation_priors_on_small_target_{false};
  bool lidar_scan_to_scan_dead_reckon_on_reject_{false};
  bool lidar_scan_to_scan_apply_pose_seed_{false};
  bool lidar_scan_to_scan_store_corrected_pose_{true};
  bool lidar_scan_to_scan_yaw_only_angular_velocity_{false};
  double lidar_scan_to_scan_pose_seed_position_gain_{1.0};
  double lidar_scan_to_scan_pose_seed_rotation_gain_{1.0};
  LidarFactor lidar_pose_factor_;
  bool lidar_pose_factor_has_keyframe_{false};
  std::size_t lidar_pose_factor_seen_frames_{0};
  std::size_t lidar_pose_factor_keyframes_{0};
  std::size_t lidar_pose_prior_factors_{0};
  std::size_t lidar_pose_prior_velocity_factors_{0};
  std::size_t lidar_pose_prior_acceleration_factors_{0};
  std::size_t lidar_pose_prior_angular_velocity_factors_{0};
  std::size_t lidar_pose_prior_matches_{0};
  std::size_t lidar_pose_prior_rejected_{0};
  double lidar_pose_prior_last_mean_residual_m_{0.0};
  bool have_last_lidar_pose_prior_{false};
  bool have_last_lidar_pose_prior_velocity_{false};
  int64_t last_lidar_pose_prior_stamp_ns_{0};
  Eigen::Vector3d last_lidar_pose_prior_position_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d last_lidar_pose_prior_velocity_world_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond last_lidar_pose_prior_orientation_{Eigen::Quaterniond::Identity()};
  bool have_last_lidar_scan_to_scan_{false};
  bool have_last_lidar_scan_to_scan_velocity_{false};
  std::vector<Eigen::Vector3d> last_lidar_scan_to_scan_points_imu_;
  TrajectoryPose last_lidar_scan_to_scan_pose_;
  Eigen::Vector3d last_lidar_scan_to_scan_velocity_world_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d last_lidar_scan_to_scan_angular_velocity_body_{Eigen::Vector3d::Zero()};
  std::size_t lidar_scan_to_scan_priors_{0};
  std::size_t lidar_scan_to_scan_position_priors_{0};
  std::size_t lidar_scan_to_scan_velocity_priors_{0};
  std::size_t lidar_scan_to_scan_acceleration_priors_{0};
  std::size_t lidar_scan_to_scan_angular_velocity_priors_{0};
  std::size_t lidar_scan_to_scan_orientation_priors_{0};
  std::size_t lidar_scan_to_scan_dead_reckoned_{0};
  std::size_t lidar_scan_to_scan_pose_seed_updates_{0};
  std::size_t lidar_scan_to_scan_pose_seed_rejected_{0};
  std::size_t lidar_scan_to_scan_velocity_clamped_{0};
  std::size_t lidar_scan_to_scan_acceleration_clamped_{0};
  std::size_t lidar_scan_to_scan_angular_velocity_clamped_{0};
  std::size_t lidar_scan_to_scan_point_to_plane_corrections_{0};
  std::size_t lidar_scan_to_scan_point_to_point_fallbacks_{0};
  std::size_t lidar_scan_to_scan_matches_{0};
  std::size_t lidar_scan_to_scan_rejected_{0};
  double lidar_scan_to_scan_last_residual_m_{0.0};
  double lidar_scan_to_scan_last_predicted_relative_translation_m_{0.0};
  double lidar_scan_to_scan_last_correction_translation_m_{0.0};
  double lidar_scan_to_scan_last_correction_rotation_rad_{0.0};
  double lidar_scan_to_scan_last_raw_target_relative_translation_m_{0.0};
  double lidar_scan_to_scan_last_target_relative_translation_m_{0.0};
  double lidar_scan_to_scan_last_target_prediction_ratio_{1.0};
  double lidar_scan_to_scan_last_target_speed_mps_{0.0};
  double lidar_scan_to_scan_last_target_angular_speed_radps_{0.0};
  double lidar_scan_to_scan_cumulative_target_path_m_{0.0};
  std::size_t lidar_scan_to_scan_small_translation_targets_{0};
  std::size_t lidar_scan_to_scan_prediction_translation_fallbacks_{0};
  std::size_t lidar_scan_to_scan_translation_priors_skipped_{0};
  bool enable_gaussian_snapshot_lidar_factor_{false};
  double gaussian_snapshot_lidar_factor_weight_{0.05};
  double gaussian_snapshot_lidar_nearest_distance_m_{0.35};
  double gaussian_snapshot_lidar_min_opacity_{0.01};
  int gaussian_snapshot_lidar_subsample_stride_{20};
  int gaussian_snapshot_map_subsample_stride_{1};
  int gaussian_snapshot_lidar_max_correspondences_{64};
  gaussian_lic_tracking::GaussianSnapshot gaussian_snapshot_;
  std::size_t gaussian_snapshot_points_{0};
  std::size_t gaussian_snapshot_chunks_received_{0};
  uint32_t gaussian_snapshot_expected_chunks_{0};
  std::size_t gaussian_snapshot_updates_{0};
  std::size_t gaussian_snapshot_lidar_matches_{0};
  std::size_t gaussian_snapshot_lidar_factor_skips_{0};
  bool enable_lidar_plane_normal_factor_{false};
  double lidar_plane_normal_factor_weight_{0.1};
  double lidar_plane_normal_huber_delta_rad_{0.10};
  std::size_t accepted_pointcloud_correspondences_{0};
  std::size_t pointcloud_messages_{0};
  std::size_t pointcloud_time_field_frames_{0};
  std::size_t pointcloud_timed_points_{0};
  std::size_t pointcloud_invalid_point_times_{0};
  std::size_t pointcloud_out_of_range_point_times_{0};
  double pointcloud_last_max_abs_point_time_offset_s_{0.0};
  std::size_t pointcloud_deskewed_points_{0};
  std::size_t pointcloud_deskew_fallback_points_{0};
  double pointcloud_last_max_deskew_m_{0.0};
  std::deque<sensor_msgs::msg::PointCloud2::SharedPtr> delayed_pointcloud_queue_;
  std::size_t delayed_pointcloud_deferred_{0};
  std::size_t delayed_pointcloud_released_{0};
  std::size_t delayed_pointcloud_dropped_{0};
  bool defer_persistent_plane_map_updates_until_solved_{false};
  int deferred_plane_map_update_max_queue_{200};
  std::deque<PendingPlaneMapUpdate> deferred_plane_map_updates_;
  std::size_t deferred_plane_map_update_enqueued_{0};
  std::size_t deferred_plane_map_update_applied_{0};
  std::size_t deferred_plane_map_update_dropped_{0};

  Eigen::Vector4d lidar_plane_{0.0, 0.0, 1.0, 0.0};
  Eigen::Vector3d lidar_to_imu_translation_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond lidar_to_imu_rotation_{Eigen::Quaterniond::Identity()};
  Eigen::Quaterniond camera_to_imu_rotation_{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d camera_to_imu_translation_{Eigen::Vector3d::Zero()};

  bool enable_visual_rotation_prior_{false};
  double visual_rotation_prior_weight_{0.1};
  double visual_rotation_prior_huber_delta_rad_{0.05};
  int visual_rotation_max_shift_px_{12};
  int visual_rotation_min_pixels_{2000};
  int visual_rotation_max_pixels_{20000};
  int visual_rotation_frame_stride_{3};
  int64_t visual_rotation_max_dt_ns_{200000000LL};
  double visual_rotation_max_rmse_{0.30};
  double visual_rotation_pixel_to_rad_scale_{1.0};
  double visual_rotation_sign_{1.0};
  VisualFactor visual_factor_{200000};
  VisualCameraIntrinsics visual_intrinsics_{};
  bool have_visual_intrinsics_{false};
  VisualFrame last_visual_frame_{};
  bool have_last_visual_frame_{false};
  VisualFrame pending_visual_se3_previous_frame_{};
  VisualFrame pending_visual_se3_current_frame_{};
  bool have_pending_visual_se3_pair_{false};
  Eigen::Quaterniond visual_orientation_target_{Eigen::Quaterniond::Identity()};
  bool have_visual_orientation_target_{false};
  std::size_t visual_rotation_image_frames_{0};
  std::size_t visual_rotation_accepted_frames_{0};
  std::size_t visual_rotation_rejected_frames_{0};
  std::size_t visual_rotation_prior_factors_{0};
  double last_visual_rotation_dx_px_{0.0};
  double last_visual_rotation_dy_px_{0.0};
  double last_visual_rotation_angle_rad_{0.0};
  double last_visual_rotation_rmse_{0.0};

  bool enable_visual_se3_prior_{false};
  double visual_se3_position_weight_{0.0};
  double visual_se3_orientation_weight_{0.0};
  double visual_se3_velocity_weight_{0.0};
  double visual_se3_huber_delta_m_{0.05};
  double visual_se3_huber_delta_rad_{0.05};
  double visual_se3_huber_delta_mps_{0.10};
  int visual_se3_max_samples_{1000};
  int visual_se3_min_samples_{32};
  double visual_se3_min_gradient_{1.0e-4};
  double visual_se3_max_abs_residual_{0.5};
  double visual_se3_huber_delta_intensity_{0.15};
  double visual_se3_min_depth_m_{0.05};
  double visual_se3_max_depth_m_{80.0};
  int visual_se3_depth_dilation_px_{2};
  int visual_se3_depth_cache_size_{8};
  int64_t visual_se3_max_dt_ns_{100000000LL};
  int visual_se3_min_hessian_rank_{4};
  double visual_se3_max_hessian_condition_{1.0e12};
  double visual_se3_min_sample_inlier_ratio_{0.20};
  int visual_se3_coverage_grid_cols_{4};
  int visual_se3_coverage_grid_rows_{4};
  int visual_se3_min_coverage_tiles_{4};
  double visual_se3_max_mean_abs_residual_{0.0};
  double visual_se3_max_translation_step_m_{0.25};
  double visual_se3_max_rotation_step_rad_{0.15};
  double visual_se3_delta_sign_{1.0};
  std::deque<SparseDepthFrame> sparse_depth_cache_;
  std::size_t visual_se3_depth_frames_{0};
  std::size_t visual_se3_depth_rejected_frames_{0};
  std::size_t visual_se3_depth_miss_count_{0};
  std::size_t visual_se3_total_batches_{0};
  std::size_t visual_se3_valid_batches_{0};
  std::size_t visual_se3_rejected_batches_{0};
  std::size_t visual_se3_degenerate_batches_{0};
  std::size_t visual_se3_step_rejected_batches_{0};
  std::size_t visual_se3_position_priors_{0};
  std::size_t visual_se3_velocity_priors_{0};
  std::size_t visual_se3_orientation_priors_{0};
  std::size_t visual_se3_sampled_depth_pixels_{0};
  std::size_t visual_se3_rejected_gradient_pixels_{0};
  std::size_t visual_se3_rejected_residual_pixels_{0};
  std::size_t last_visual_se3_samples_{0};
  std::size_t last_visual_se3_hessian_rank_{0};
  std::size_t last_visual_se3_coverage_tiles_{0};
  std::size_t last_visual_se3_coverage_total_tiles_{0};
  double last_visual_se3_hessian_condition_{0.0};
  double last_visual_se3_mean_abs_residual_{0.0};
  double last_visual_se3_translation_step_m_{0.0};
  double last_visual_se3_rotation_step_rad_{0.0};
  int64_t last_visual_se3_depth_delta_ns_{0};

  bool enable_imu_gravity_autocal_{true};
  bool enable_startup_bias_autocal_{true};
  double imu_linear_acceleration_scale_{1.0};
  bool enable_voxel_plane_extraction_{false};
  bool enable_persistent_plane_map_{true};
  bool persistent_map_update_requires_accepted_solve_{false};
  bool enable_external_odometry_prior_{false};
  bool enable_external_odometry_position_factors_{false};
  bool enable_external_odometry_orientation_factors_{false};
  double external_odometry_position_factor_weight_{1.0};
  double external_odometry_position_factor_huber_delta_m_{0.25};
  double external_odometry_orientation_factor_weight_{1.0};
  double external_odometry_orientation_factor_huber_delta_rad_{0.25};
  bool have_prior_pose_{false};
  Eigen::Vector3d latest_prior_position_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond latest_prior_orientation_{Eigen::Quaterniond::Identity()};
  Eigen::Quaterniond prior_to_imu_rotation_{Eigen::Quaterniond::Identity()};
  std::size_t accepted_prior_count_{0};
  std::size_t rejected_prior_count_{0};
  std::size_t accepted_prior_position_factor_messages_{0};
  std::size_t accepted_prior_orientation_factor_messages_{0};
  spline::LidarPlaneExtractor plane_extractor_{};
  spline::PersistentPlaneMap persistent_plane_map_{};
  std::size_t persistent_plane_map_matches_{0};
  std::size_t persistent_plane_map_updates_{0};
  std::size_t persistent_plane_map_update_skips_{0};
  std::size_t persistent_plane_normal_factors_{0};
  bool enable_persistent_point_map_{false};
  double persistent_point_map_nearest_distance_m_{0.35};
  double persistent_point_map_merge_distance_m_{0.10};
  double persistent_point_map_min_match_age_s_{0.25};
  double persistent_point_map_factor_weight_{0.05};
  int persistent_point_map_subsample_stride_{20};
  int persistent_point_map_max_points_{20000};
  int persistent_point_map_max_correspondences_{64};
  int persistent_point_map_min_observations_for_match_{3};
  std::vector<PersistentPointMapEntry> persistent_points_world_;
  std::size_t persistent_point_map_matches_{0};
  std::size_t persistent_point_map_updates_{0};
  std::size_t persistent_point_map_update_skips_{0};

  double step_period_seconds_{0.10};
  bool use_stamp_driven_steps_{false};
  int max_stamp_driven_steps_per_callback_{4};
  double pose_output_period_seconds_{0.0};
  int diagnostic_log_period_steps_{50};
  int64_t step_period_ns_{0};
  int64_t last_stamp_driven_step_ns_{0};
  int64_t pose_output_period_ns_{0};
  int64_t knot_interval_ns_{50000000};
  int64_t last_published_query_ns_{0};
  std::size_t last_logged_rejected_solver_steps_{0};
  std::size_t last_logged_rotation_limited_solver_steps_{0};
  std::size_t last_logged_position_limited_solver_steps_{0};
  std::size_t last_logged_diagnostic_step_{0};
};

}  // namespace gaussian_lic_tracking

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<gaussian_lic_tracking::ContinuousTimeNode>());
  rclcpp::shutdown();
  return 0;
}
