// SPDX-License-Identifier: GPL-3.0-or-later

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
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
#include <gaussian_lic_tracking/spline/so3_ops.hpp>
#include <gaussian_lic_tracking/time.hpp>
#include <gaussian_lic_tracking/visual_factor.hpp>
#include <gaussian_lic_msgs/msg/gaussian_array.hpp>
#include <gaussian_lic_msgs/msg/rendered_feedback.hpp>
#include <gaussian_lic_msgs/msg/tracking_status.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/SVD>

namespace
{

void accumulate_correspondence_weights(
  const std::vector<double> & weights,
  size_t & count,
  double & sum,
  double & minimum)
{
  for (const double weight : weights) {
    if (!std::isfinite(weight) || weight <= 0.0) {
      continue;
    }
    ++count;
    sum += weight;
    minimum = std::min(minimum, weight);
  }
}

double mean_or_zero(const size_t count, const double sum)
{
  return count > 0U ? sum / static_cast<double>(count) : 0.0;
}

double min_or_zero(const size_t count, const double minimum)
{
  return count > 0U ? minimum : 0.0;
}

void apply_correspondence_weight_power(std::vector<double> & weights, const double power)
{
  if (weights.empty() || !std::isfinite(power) || std::abs(power - 1.0) <= 1.0e-12) {
    return;
  }
  for (double & weight : weights) {
    if (!std::isfinite(weight) || weight <= 0.0) {
      continue;
    }
    weight = std::pow(std::clamp(weight, 1.0e-9, 1.0), power);
  }
}

double ratio_score(const double value, const double threshold)
{
  if (!std::isfinite(value) || value <= 0.0 || !std::isfinite(threshold) || threshold <= 0.0) {
    return 0.0;
  }
  return std::clamp(value / threshold, 0.0, 1.0);
}

double inverse_ratio_score(const double value, const double threshold)
{
  if (!std::isfinite(value) || value <= 0.0 || !std::isfinite(threshold) || threshold <= 0.0) {
    return 0.0;
  }
  return std::clamp(threshold / value, 0.0, 1.0);
}

gaussian_lic_tracking::VisualAlignmentMetric parse_visual_alignment_metric(
  const std::string & value)
{
  std::string lower = value;
  std::transform(lower.begin(), lower.end(), lower.begin(), [](const unsigned char ch) {
      return static_cast<char>(std::tolower(ch));
    });
  if (lower == "rmse") {
    return gaussian_lic_tracking::VisualAlignmentMetric::kRmse;
  }
  if (lower == "zncc") {
    return gaussian_lic_tracking::VisualAlignmentMetric::kZncc;
  }
  throw std::runtime_error("visual_alignment_score_mode must be 'rmse' or 'zncc'");
}

enum class VisualAlignmentFactorSource
{
  kSearch,
  kPhotometricStep,
  kSaturatedPhotometricStep,
};

enum class VisualFactorSourceIdMode
{
  kLegacy8Bit,
  kFull64Bit,
};

enum class VisualFactorReferenceStampMode
{
  kObserved,
  kRendered,
  kRenderedSourceImage,
  kRenderedSourcePose,
  kRenderedSourcePointcloud,
};

enum class SlidingWindowBiasFeedbackOwnership
{
  kOptimized,
  kPoseOnlyForVisualFeedback,
};

VisualAlignmentFactorSource parse_visual_alignment_factor_source(
  const std::string & value)
{
  std::string lower = value;
  std::transform(lower.begin(), lower.end(), lower.begin(), [](const unsigned char ch) {
      return static_cast<char>(std::tolower(ch));
    });
  if (lower == "search") {
    return VisualAlignmentFactorSource::kSearch;
  }
  if (lower == "photometric_step") {
    return VisualAlignmentFactorSource::kPhotometricStep;
  }
  if (lower == "saturated_photometric_step") {
    return VisualAlignmentFactorSource::kSaturatedPhotometricStep;
  }
  throw std::runtime_error(
    "visual_alignment_factor_source must be 'search', 'photometric_step', or "
    "'saturated_photometric_step'");
}

VisualFactorSourceIdMode parse_visual_factor_source_id_mode(const std::string & value)
{
  std::string lower = value;
  std::transform(lower.begin(), lower.end(), lower.begin(), [](const unsigned char ch) {
      return static_cast<char>(std::tolower(ch));
    });
  if (lower == "legacy_8bit") {
    return VisualFactorSourceIdMode::kLegacy8Bit;
  }
  if (lower == "full_64bit") {
    return VisualFactorSourceIdMode::kFull64Bit;
  }
  throw std::runtime_error("visual_factor_source_id_mode must be 'legacy_8bit' or 'full_64bit'");
}

VisualFactorReferenceStampMode parse_visual_factor_reference_stamp_mode(const std::string & value)
{
  std::string lower = value;
  std::transform(lower.begin(), lower.end(), lower.begin(), [](const unsigned char ch) {
      return static_cast<char>(std::tolower(ch));
    });
  if (lower == "observed") {
    return VisualFactorReferenceStampMode::kObserved;
  }
  if (lower == "rendered") {
    return VisualFactorReferenceStampMode::kRendered;
  }
  if (lower == "rendered_source_image") {
    return VisualFactorReferenceStampMode::kRenderedSourceImage;
  }
  if (lower == "rendered_source_pose") {
    return VisualFactorReferenceStampMode::kRenderedSourcePose;
  }
  if (lower == "rendered_source_pointcloud") {
    return VisualFactorReferenceStampMode::kRenderedSourcePointcloud;
  }
  throw std::runtime_error(
    "visual_factor_reference_stamp_mode must be 'observed', 'rendered', "
    "'rendered_source_image', 'rendered_source_pose', or 'rendered_source_pointcloud'");
}

SlidingWindowBiasFeedbackOwnership parse_sliding_window_bias_feedback_ownership(
  const std::string & value)
{
  std::string lower = value;
  std::transform(lower.begin(), lower.end(), lower.begin(), [](const unsigned char ch) {
      return static_cast<char>(std::tolower(ch));
    });
  if (lower == "optimized") {
    return SlidingWindowBiasFeedbackOwnership::kOptimized;
  }
  if (lower == "pose_only_for_visual_feedback") {
    return SlidingWindowBiasFeedbackOwnership::kPoseOnlyForVisualFeedback;
  }
  throw std::runtime_error(
    "sliding_window_bias_feedback_ownership must be 'optimized' or "
    "'pose_only_for_visual_feedback'");
}

const char * sliding_window_bias_feedback_ownership_name(
  const SlidingWindowBiasFeedbackOwnership ownership)
{
  switch (ownership) {
    case SlidingWindowBiasFeedbackOwnership::kOptimized:
      return "optimized";
    case SlidingWindowBiasFeedbackOwnership::kPoseOnlyForVisualFeedback:
      return "pose_only_for_visual_feedback";
  }
  return "optimized";
}

struct GaussianSnapshotPoseCorrection
{
  bool applied{false};
  size_t matched_points{0U};
  double mean_residual_m{0.0};
  Eigen::Vector3d delta_p_w{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond delta_q{Eigen::Quaterniond::Identity()};
};

}  // namespace

class TrackingNode final : public rclcpp::Node
{
  enum class StepGuardStage
  {
    kPreLio,
    kPostBa,
  };

  enum class RelativeMotionHistorySource
  {
    kPreBa,
    kPublished,
    kPublishedAfterWarmup,
  };

public:
  explicit TrackingNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("tracking_node", options)
  {
    raw_image_topic_ = declare_parameter<std::string>("raw_image_topic", "/camera/image");
    raw_camera_info_topic_ = declare_parameter<std::string>("raw_camera_info_topic", "/camera/camera_info");
    raw_depth_topic_ = declare_parameter<std::string>("raw_depth_topic", "/camera/depth");
    raw_pointcloud_topic_ = declare_parameter<std::string>("raw_pointcloud_topic", "/livox/lidar");
    raw_imu_topic_ = declare_parameter<std::string>("raw_imu_topic", "/imu");
    external_odometry_prior_topic_ = declare_parameter<std::string>(
      "external_odometry_prior_topic", "/gaussian_lic/frontend/input_odometry");
    enable_pointcloud_imu_wait_ = declare_parameter<bool>("enable_pointcloud_imu_wait", true);
    pointcloud_imu_wait_tolerance_ns_ = integer_parameter_at_least(
      "pointcloud_imu_wait_tolerance_ns",
      declare_parameter<int64_t>("pointcloud_imu_wait_tolerance_ns", 0LL), 0LL);
    pointcloud_imu_wait_queue_size_ = integer_parameter_at_least(
      "pointcloud_imu_wait_queue_size",
      declare_parameter<int>("pointcloud_imu_wait_queue_size", 4), 1);
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
    rendered_feedback_topic_ =
      declare_parameter<std::string>("rendered_feedback_topic", "/gaussian_lic/rendered_feedback");
    enable_rendered_feedback_contract_ =
      declare_parameter<bool>("enable_rendered_feedback_contract", false);
    // Deterministic offline replay: when deterministic_bag_path is set, main()
    // reads the sensor bag (and the recorded rendered-feedback bag) directly and
    // dispatches messages to the handlers IN-PROCESS, merged into one sequence
    // sorted by header stamp, instead of spinning on async DDS subscriptions.
    // This removes the cross-topic callback-interleaving + feedback-lag
    // nondeterminism that makes the live (ros2 bag play) visual evaluation
    // irreproducible. Everything is gated behind a non-empty
    // deterministic_bag_path so the production async path is byte-for-byte
    // untouched.
    deterministic_bag_path_ =
      declare_parameter<std::string>("deterministic_bag_path", "");
    deterministic_feedback_bag_path_ =
      declare_parameter<std::string>("deterministic_feedback_bag_path", "");
    output_tum_path_ = declare_parameter<std::string>("output_tum_path", "");
    if (!output_tum_path_.empty()) {
      output_tum_stream_.open(output_tum_path_, std::ios::out | std::ios::trunc);
      if (!output_tum_stream_) {
        RCLCPP_WARN(
          get_logger(),
          "could not open output_tum_path '%s' for writing",
          output_tum_path_.c_str());
      } else {
        output_tum_stream_ << "# timestamp tx ty tz qx qy qz qw" << std::endl;
        output_tum_stream_.flush();
      }
    }
    rendered_image_qos_reliability_ =
      declare_parameter<std::string>("rendered_image_qos_reliability", "reliable");
    rendered_image_qos_durability_ =
      declare_parameter<std::string>("rendered_image_qos_durability", "transient_local");
    rendered_image_qos_depth_ = integer_parameter_at_least(
      "rendered_image_qos_depth",
      declare_parameter<int>("rendered_image_qos_depth", 1),
      1);
    rendered_feedback_qos_reliability_ =
      declare_parameter<std::string>("rendered_feedback_qos_reliability", "reliable");
    rendered_feedback_qos_durability_ =
      declare_parameter<std::string>("rendered_feedback_qos_durability", "volatile");
    rendered_feedback_qos_depth_ = integer_parameter_at_least(
      "rendered_feedback_qos_depth",
      declare_parameter<int>("rendered_feedback_qos_depth", 128),
      1);
    enable_rendered_feedback_ingress_queue_ =
      declare_parameter<bool>("enable_rendered_feedback_ingress_queue", true);
    rendered_feedback_ingress_queue_size_ = integer_parameter_at_least(
      "rendered_feedback_ingress_queue_size",
      declare_parameter<int>("rendered_feedback_ingress_queue_size", 512),
      1);
    rendered_feedback_ingress_drain_max_per_cycle_ = integer_parameter_at_least(
      "rendered_feedback_ingress_drain_max_per_cycle",
      declare_parameter<int>("rendered_feedback_ingress_drain_max_per_cycle", 64),
      1);
    rendered_feedback_ingress_drain_period_ms_ = integer_parameter_at_least(
      "rendered_feedback_ingress_drain_period_ms",
      declare_parameter<int>("rendered_feedback_ingress_drain_period_ms", 5),
      1);
    gaussian_map_topic_ = declare_parameter<std::string>("gaussian_map_topic", "/gaussian_lic/gaussian_map");
    gaussian_snapshot_qos_depth_ = integer_parameter_at_least(
      "gaussian_snapshot_qos_depth",
      declare_parameter<int>("gaussian_snapshot_qos_depth", 64),
      1);
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
    enable_visual_factor_time_interpolation_ =
      declare_parameter<bool>("enable_visual_factor_time_interpolation", false);
    enable_visual_cache_reconciliation_ =
      declare_parameter<bool>("enable_visual_cache_reconciliation", false);
    visual_cache_reconciliation_monotonic_unique_ =
      declare_parameter<bool>("visual_cache_reconciliation_monotonic_unique", false);
    visual_pair_monotonic_unique_ =
      declare_parameter<bool>("visual_pair_monotonic_unique", false);
    enable_visual_watermark_pair_scheduler_ =
      declare_parameter<bool>("enable_visual_watermark_pair_scheduler", false);
    visual_watermark_pair_scheduler_max_pairs_per_pointcloud_ = integer_parameter_at_least(
      "visual_watermark_pair_scheduler_max_pairs_per_pointcloud",
      declare_parameter<int>("visual_watermark_pair_scheduler_max_pairs_per_pointcloud", 2), 1);
    enable_visual_callback_factor_ingest_ =
      declare_parameter<bool>("enable_visual_callback_factor_ingest", false);
    enable_rendered_feedback_watermark_queue_ =
      declare_parameter<bool>("enable_rendered_feedback_watermark_queue", false);
    defer_future_visual_factors_until_active_ =
      declare_parameter<bool>("defer_future_visual_factors_until_active", false);
    enable_visual_adaptive_state_retention_ =
      declare_parameter<bool>("enable_visual_adaptive_state_retention", false);
    visual_adaptive_state_retention_margin_states_ = integer_parameter_at_least(
      "visual_adaptive_state_retention_margin_states",
      declare_parameter<int>("visual_adaptive_state_retention_margin_states", 4), 0);
    visual_adaptive_state_retention_max_states_ = integer_parameter_at_least(
      "visual_adaptive_state_retention_max_states",
      declare_parameter<int>("visual_adaptive_state_retention_max_states", 64), 2);
    enable_visual_expired_factor_projection_ =
      declare_parameter<bool>("enable_visual_expired_factor_projection", false);
    enable_visual_marginalization_prior_ =
      declare_parameter<bool>("enable_visual_marginalization_prior", false);
    enable_visual_marginalization_prior_batching_ =
      declare_parameter<bool>("enable_visual_marginalization_prior_batching", false);
    enable_visual_marginalization_prior_saturation_gate_ =
      declare_parameter<bool>("enable_visual_marginalization_prior_saturation_gate", false);
    visual_marginalization_prior_saturation_gate_visual_factors_ =
      declare_parameter<bool>("visual_marginalization_prior_saturation_gate_visual_factors", true);
    visual_marginalization_prior_saturation_gate_se3_factors_ =
      declare_parameter<bool>("visual_marginalization_prior_saturation_gate_se3_factors", true);
    enable_visual_alignment_saturation_axis_mask_ =
      declare_parameter<bool>("enable_visual_alignment_saturation_axis_mask", false);
    visual_marginalization_prior_zero_bias_columns_ =
      declare_parameter<bool>("visual_marginalization_prior_zero_bias_columns", false);
    enable_visual_factor_reference_snapshot_ =
      declare_parameter<bool>("enable_visual_factor_reference_snapshot", false);
    enable_rendered_feedback_source_pose_reference_ =
      declare_parameter<bool>("enable_rendered_feedback_source_pose_reference", false);
    enable_rendered_feedback_source_motion_factor_ =
      declare_parameter<bool>("enable_rendered_feedback_source_motion_factor", false);
    rendered_feedback_source_motion_translation_weight_ = finite_nonnegative_parameter(
      "rendered_feedback_source_motion_translation_weight",
      declare_parameter<double>("rendered_feedback_source_motion_translation_weight", 0.0));
    rendered_feedback_source_motion_rotation_weight_ = finite_nonnegative_parameter(
      "rendered_feedback_source_motion_rotation_weight",
      declare_parameter<double>("rendered_feedback_source_motion_rotation_weight", 0.0));
    rendered_feedback_source_motion_huber_delta_m_ = finite_nonnegative_parameter(
      "rendered_feedback_source_motion_huber_delta_m",
      declare_parameter<double>("rendered_feedback_source_motion_huber_delta_m", 0.1));
    rendered_feedback_source_motion_rotation_huber_delta_rad_ = finite_nonnegative_parameter(
      "rendered_feedback_source_motion_rotation_huber_delta_rad",
      declare_parameter<double>("rendered_feedback_source_motion_rotation_huber_delta_rad", 0.05));
    rendered_feedback_source_motion_min_dt_s_ = finite_nonnegative_parameter(
      "rendered_feedback_source_motion_min_dt_s",
      declare_parameter<double>("rendered_feedback_source_motion_min_dt_s", 0.0));
    rendered_feedback_source_motion_max_dt_s_ = finite_nonnegative_parameter(
      "rendered_feedback_source_motion_max_dt_s",
      declare_parameter<double>("rendered_feedback_source_motion_max_dt_s", 1.0));
    if (rendered_feedback_source_motion_max_dt_s_ <
      rendered_feedback_source_motion_min_dt_s_)
    {
      throw std::runtime_error(
              "rendered_feedback_source_motion_max_dt_s must be >= "
              "rendered_feedback_source_motion_min_dt_s");
    }
    rendered_feedback_source_motion_in_from_frame_ =
      declare_parameter<bool>("rendered_feedback_source_motion_in_from_frame", false);
    enable_rendered_feedback_source_motion_marginalized_prior_ =
      declare_parameter<bool>("enable_rendered_feedback_source_motion_marginalized_prior", false);
    visual_expired_factor_projection_max_age_s_ =
      declare_parameter<double>("visual_expired_factor_projection_max_age_s", 5.0);
    if (!std::isfinite(visual_expired_factor_projection_max_age_s_)) {
      throw std::runtime_error("visual_expired_factor_projection_max_age_s must be finite");
    }
    visual_cache_reconciliation_defer_to_pointcloud_ =
      declare_parameter<bool>("visual_cache_reconciliation_defer_to_pointcloud", false);
    visual_pair_processing_defer_to_pointcloud_ =
      declare_parameter<bool>("visual_pair_processing_defer_to_pointcloud", false);
    visual_depth_max_dt_ns_ = integer_parameter_at_least(
      "visual_depth_max_dt_ns",
      declare_parameter<int64_t>("visual_depth_max_dt_ns", 0LL), 0LL);
    depth_frame_cache_size_ = integer_parameter_at_least(
      "depth_frame_cache_size", declare_parameter<int>("depth_frame_cache_size", 8), 1);
    sparse_lidar_depth_dilation_px_ = integer_parameter_at_least(
      "sparse_lidar_depth_dilation_px",
      declare_parameter<int>("sparse_lidar_depth_dilation_px", 1), 0);
    rendered_frame_cache_size_ = integer_parameter_at_least(
      "rendered_frame_cache_size", declare_parameter<int>("rendered_frame_cache_size", 8), 1);
    observed_frame_cache_size_ = integer_parameter_at_least(
      "observed_frame_cache_size", declare_parameter<int>("observed_frame_cache_size", 64), 1);
    visual_pending_factor_queue_size_ = integer_parameter_at_least(
      "visual_pending_factor_queue_size",
      declare_parameter<int>("visual_pending_factor_queue_size", 64), 1);
    enable_visual_factor_quality_weighting_ =
      declare_parameter<bool>("enable_visual_factor_quality_weighting", false);
    visual_factor_quality_min_weight_scale_ = finite_unit_interval_parameter(
      "visual_factor_quality_min_weight_scale",
      declare_parameter<double>("visual_factor_quality_min_weight_scale", 0.25));
    enable_visual_factor_quality_selection_ =
      declare_parameter<bool>("enable_visual_factor_quality_selection", false);
    visual_factor_quality_selection_max_per_reference_ = integer_parameter_at_least(
      "visual_factor_quality_selection_max_per_reference",
      declare_parameter<int>("visual_factor_quality_selection_max_per_reference", 2), 1);
    enable_visual_factor_quality_reference_cap_ =
      declare_parameter<bool>("enable_visual_factor_quality_reference_cap", true);
    visual_factor_quality_selection_start_after_s_ = finite_nonnegative_parameter(
      "visual_factor_quality_selection_start_after_s",
      declare_parameter<double>("visual_factor_quality_selection_start_after_s", 0.0));
    const auto camera_to_imu_translation = declare_parameter<std::vector<double>>(
      "camera_to_imu_translation_m", std::vector<double>{0.0, 0.0, 0.0});
    const auto camera_to_imu_rpy = declare_parameter<std::vector<double>>(
      "camera_to_imu_rpy_rad", std::vector<double>{0.0, 0.0, 0.0});
    p_i_c_ = vector3_from_parameter("camera_to_imu_translation_m", camera_to_imu_translation);
    q_i_c_ = quaternion_from_rpy_parameter("camera_to_imu_rpy_rad", camera_to_imu_rpy);
    visual_alignment_max_shift_px_ = integer_parameter_at_least(
      "visual_alignment_max_shift_px",
      declare_parameter<int>("visual_alignment_max_shift_px", 8), 0);
    visual_alignment_score_mode_ =
      declare_parameter<std::string>("visual_alignment_score_mode", "rmse");
    visual_alignment_metric_ = parse_visual_alignment_metric(visual_alignment_score_mode_);
    visual_alignment_factor_source_name_ =
      declare_parameter<std::string>("visual_alignment_factor_source", "search");
    visual_alignment_factor_source_ =
      parse_visual_alignment_factor_source(visual_alignment_factor_source_name_);
    visual_factor_source_id_mode_name_ =
      declare_parameter<std::string>("visual_factor_source_id_mode", "legacy_8bit");
    visual_factor_source_id_mode_ =
      parse_visual_factor_source_id_mode(visual_factor_source_id_mode_name_);
    visual_factor_reference_stamp_mode_name_ =
      declare_parameter<std::string>("visual_factor_reference_stamp_mode", "observed");
    visual_factor_reference_stamp_mode_ =
      parse_visual_factor_reference_stamp_mode(visual_factor_reference_stamp_mode_name_);
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
    visual_alignment_saturation_margin_px_ = finite_nonnegative_parameter(
      "visual_alignment_saturation_margin_px",
      declare_parameter<double>("visual_alignment_saturation_margin_px", 0.0));
    visual_alignment_saturated_weight_scale_ = finite_nonnegative_parameter(
      "visual_alignment_saturated_weight_scale",
      declare_parameter<double>("visual_alignment_saturated_weight_scale", 1.0));
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
    se3_photometric_min_hessian_rank_ = integer_parameter_at_least(
      "se3_photometric_min_hessian_rank",
      declare_parameter<int>("se3_photometric_min_hessian_rank", 3), 0);
    if (se3_photometric_min_hessian_rank_ > 6) {
      throw std::runtime_error("se3_photometric_min_hessian_rank must be <= 6");
    }
    se3_photometric_max_hessian_condition_ = finite_nonnegative_parameter(
      "se3_photometric_max_hessian_condition",
      declare_parameter<double>("se3_photometric_max_hessian_condition", 1.0e12));
    se3_photometric_min_sample_inlier_ratio_ = finite_unit_interval_parameter(
      "se3_photometric_min_sample_inlier_ratio",
      declare_parameter<double>("se3_photometric_min_sample_inlier_ratio", 0.25));
    se3_photometric_max_mean_abs_residual_for_factor_ = finite_nonnegative_parameter(
      "se3_photometric_max_mean_abs_residual_for_factor",
      declare_parameter<double>("se3_photometric_max_mean_abs_residual_for_factor", 0.0));
    se3_photometric_coverage_grid_cols_ = integer_parameter_at_least(
      "se3_photometric_coverage_grid_cols",
      declare_parameter<int>("se3_photometric_coverage_grid_cols", 4), 1);
    se3_photometric_coverage_grid_rows_ = integer_parameter_at_least(
      "se3_photometric_coverage_grid_rows",
      declare_parameter<int>("se3_photometric_coverage_grid_rows", 4), 1);
    se3_photometric_min_coverage_tiles_ = integer_parameter_at_least(
      "se3_photometric_min_coverage_tiles",
      declare_parameter<int>("se3_photometric_min_coverage_tiles", 4), 1);
    const int se3_photometric_total_coverage_tiles =
      se3_photometric_coverage_grid_cols_ * se3_photometric_coverage_grid_rows_;
    if (se3_photometric_min_coverage_tiles_ > se3_photometric_total_coverage_tiles) {
      throw std::runtime_error(
              "se3_photometric_min_coverage_tiles must be <= "
              "se3_photometric_coverage_grid_cols * se3_photometric_coverage_grid_rows");
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
    se3_photometric_rank_samples_by_gradient_ =
      declare_parameter<bool>("se3_photometric_rank_samples_by_gradient", false);
    se3_photometric_use_rendered_gradient_ =
      declare_parameter<bool>("se3_photometric_use_rendered_gradient", false);
    se3_photometric_huber_delta_ = finite_nonnegative_parameter(
      "se3_photometric_huber_delta",
      declare_parameter<double>("se3_photometric_huber_delta", 0.15));
    se3_photometric_max_abs_residual_ = finite_nonnegative_parameter(
      "se3_photometric_max_abs_residual",
      declare_parameter<double>("se3_photometric_max_abs_residual", 1.0));
    enable_se3_photometric_pose_correction_ =
      declare_parameter<bool>("enable_se3_photometric_pose_correction", false);
    se3_photometric_pose_correction_gain_ = finite_unit_interval_parameter(
      "se3_photometric_pose_correction_gain",
      declare_parameter<double>("se3_photometric_pose_correction_gain", 0.1));
    se3_photometric_pose_correction_max_translation_m_ = finite_nonnegative_parameter(
      "se3_photometric_pose_correction_max_translation_m",
      declare_parameter<double>("se3_photometric_pose_correction_max_translation_m", 0.02));
    se3_photometric_pose_correction_max_rotation_rad_ = finite_nonnegative_parameter(
      "se3_photometric_pose_correction_max_rotation_rad",
      declare_parameter<double>("se3_photometric_pose_correction_max_rotation_rad", 0.01));
    se3_photometric_pose_correction_max_dt_ns_ = integer_parameter_at_least(
      "se3_photometric_pose_correction_max_dt_ns",
      declare_parameter<int64_t>("se3_photometric_pose_correction_max_dt_ns", 0LL), 0LL);
    enable_lio_factor_ = declare_parameter<bool>("enable_lio_factor", true);
    enable_external_odometry_prior_ =
      declare_parameter<bool>("enable_external_odometry_prior", false);
    external_odometry_prior_max_dt_ns_ = integer_parameter_at_least(
      "external_odometry_prior_max_dt_ns",
      declare_parameter<int64_t>("external_odometry_prior_max_dt_ns", 100000000LL), 0LL);
    external_odometry_prior_cache_size_ = integer_parameter_at_least(
      "external_odometry_prior_cache_size",
      declare_parameter<int>("external_odometry_prior_cache_size", 128), 1);
    external_odometry_prior_translation_weight_ = finite_nonnegative_parameter(
      "external_odometry_prior_translation_weight",
      declare_parameter<double>("external_odometry_prior_translation_weight", 4.0));
    external_odometry_prior_rotation_weight_ = finite_nonnegative_parameter(
      "external_odometry_prior_rotation_weight",
      declare_parameter<double>("external_odometry_prior_rotation_weight", 4.0));
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
    lidar_pose_factor_iterations_ = integer_parameter_at_least(
      "lidar_pose_factor_iterations",
      declare_parameter<int>("lidar_pose_factor_iterations", 1), 1);
    lidar_window_point_factor_weight_ = finite_positive_parameter(
      "lidar_window_point_factor_weight",
      declare_parameter<double>("lidar_window_point_factor_weight", 1.0));
    lidar_window_plane_factor_weight_ = finite_positive_parameter(
      "lidar_window_plane_factor_weight",
      declare_parameter<double>("lidar_window_plane_factor_weight", 1.0));
    lidar_window_confidence_power_ = finite_positive_parameter(
      "lidar_window_confidence_power",
      declare_parameter<double>("lidar_window_confidence_power", 1.0));
    lidar_plane_min_neighbors_ = integer_parameter_at_least(
      "lidar_plane_min_neighbors",
      declare_parameter<int>("lidar_plane_min_neighbors", 5), 3);
    lidar_plane_max_condition_ = finite_positive_parameter(
      "lidar_plane_max_condition",
      declare_parameter<double>("lidar_plane_max_condition", 0.2));
    // LOAM edge/corner (point-to-line) factor — targets terminal yaw drift.
    enable_lidar_line_factor_ = declare_parameter<bool>("enable_lidar_line_factor", false);
    lidar_line_max_condition_ = finite_positive_parameter(
      "lidar_line_max_condition",
      declare_parameter<double>("lidar_line_max_condition", 0.2));
    lidar_window_line_factor_weight_ = finite_positive_parameter(
      "lidar_window_line_factor_weight",
      declare_parameter<double>("lidar_window_line_factor_weight", 1.0));
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
    if (lidar_time_mode_ != "auto" && lidar_time_mode_ != "absolute" &&
      lidar_time_mode_ != "offset" && lidar_time_mode_ != "scan_order")
    {
      throw std::runtime_error("lidar_time_mode must be auto, absolute, offset, or scan_order");
    }
    lidar_scan_order_duration_s_ = finite_positive_parameter(
      "lidar_scan_order_duration_s",
      declare_parameter<double>("lidar_scan_order_duration_s", 0.1));
    lidar_max_abs_point_time_offset_s_ = finite_positive_parameter(
      "lidar_max_abs_point_time_offset_s",
      declare_parameter<double>("lidar_max_abs_point_time_offset_s", 0.25));
    imu_history_size_ = integer_parameter_at_least(
      "imu_history_size", declare_parameter<int>("imu_history_size", 12000), 2);
    tracking_max_pose_step_m_ = finite_nonnegative_parameter(
      "tracking_max_pose_step_m",
      declare_parameter<double>("tracking_max_pose_step_m", 0.25));
    enable_pre_lio_tracking_step_guard_ =
      declare_parameter<bool>("enable_pre_lio_tracking_step_guard", true);
    enable_post_ba_tracking_step_guard_ =
      declare_parameter<bool>("enable_post_ba_tracking_step_guard", true);
    pre_lio_tracking_max_pose_step_m_ = finite_nonnegative_parameter(
      "pre_lio_tracking_max_pose_step_m",
      declare_parameter<double>("pre_lio_tracking_max_pose_step_m", 0.0));
    post_ba_tracking_max_pose_step_m_ = finite_nonnegative_parameter(
      "post_ba_tracking_max_pose_step_m",
      declare_parameter<double>("post_ba_tracking_max_pose_step_m", 0.0));
    post_ba_step_guard_confidence_max_pose_step_m_ = finite_nonnegative_parameter(
      "post_ba_step_guard_confidence_max_pose_step_m",
      declare_parameter<double>("post_ba_step_guard_confidence_max_pose_step_m", 0.0));
    post_ba_step_guard_confidence_warmup_marginalizations_ = integer_parameter_at_least(
      "post_ba_step_guard_confidence_warmup_marginalizations",
      declare_parameter<int>("post_ba_step_guard_confidence_warmup_marginalizations", 0),
      0);
    post_ba_step_guard_min_lidar_confidence_ = finite_nonnegative_parameter(
      "post_ba_step_guard_min_lidar_confidence",
      declare_parameter<double>("post_ba_step_guard_min_lidar_confidence", 0.6));
    post_ba_step_guard_min_visual_inlier_ratio_ = finite_unit_interval_parameter(
      "post_ba_step_guard_min_visual_inlier_ratio",
      declare_parameter<double>("post_ba_step_guard_min_visual_inlier_ratio", 0.85));
    post_ba_step_guard_max_visual_residual_ = finite_nonnegative_parameter(
      "post_ba_step_guard_max_visual_residual",
      declare_parameter<double>("post_ba_step_guard_max_visual_residual", 0.3));
    post_ba_step_guard_min_visual_coverage_tiles_ = integer_parameter_at_least(
      "post_ba_step_guard_min_visual_coverage_tiles",
      declare_parameter<int>("post_ba_step_guard_min_visual_coverage_tiles", 8), 1);
    post_ba_step_guard_reject_to_pre_ba_over_m_ = finite_nonnegative_parameter(
      "post_ba_step_guard_reject_to_pre_ba_over_m",
      declare_parameter<double>("post_ba_step_guard_reject_to_pre_ba_over_m", 0.0));
    post_ba_step_guard_pre_ba_agreement_max_pose_step_m_ = finite_nonnegative_parameter(
      "post_ba_step_guard_pre_ba_agreement_max_pose_step_m",
      declare_parameter<double>("post_ba_step_guard_pre_ba_agreement_max_pose_step_m", 0.0));
    post_ba_step_guard_pre_ba_agreement_late_start_marginalizations_ =
      integer_parameter_at_least(
      "post_ba_step_guard_pre_ba_agreement_late_start_marginalizations",
      declare_parameter<int>(
        "post_ba_step_guard_pre_ba_agreement_late_start_marginalizations", 0),
      0);
    post_ba_step_guard_pre_ba_agreement_late_max_pose_step_m_ =
      finite_nonnegative_parameter(
      "post_ba_step_guard_pre_ba_agreement_late_max_pose_step_m",
      declare_parameter<double>(
        "post_ba_step_guard_pre_ba_agreement_late_max_pose_step_m", 0.0));
    post_ba_step_guard_pre_ba_agreement_min_cosine_ = finite_unit_interval_parameter(
      "post_ba_step_guard_pre_ba_agreement_min_cosine",
      declare_parameter<double>("post_ba_step_guard_pre_ba_agreement_min_cosine", 0.85));
    post_ba_step_guard_pre_ba_agreement_max_delta_m_ = finite_nonnegative_parameter(
      "post_ba_step_guard_pre_ba_agreement_max_delta_m",
      declare_parameter<double>("post_ba_step_guard_pre_ba_agreement_max_delta_m", 0.05));
    post_ba_step_guard_pre_ba_agreement_margin_m_ = finite_nonnegative_parameter(
      "post_ba_step_guard_pre_ba_agreement_margin_m",
      declare_parameter<double>("post_ba_step_guard_pre_ba_agreement_margin_m", 0.0));
    post_ba_step_guard_pre_ba_blend_on_clamp_ = finite_unit_interval_parameter(
      "post_ba_step_guard_pre_ba_blend_on_clamp",
      declare_parameter<double>("post_ba_step_guard_pre_ba_blend_on_clamp", 0.0));
    tracking_step_guard_velocity_scale_ = finite_nonnegative_parameter(
      "tracking_step_guard_velocity_scale",
      declare_parameter<double>("tracking_step_guard_velocity_scale", 0.0));
    pre_lio_tracking_step_guard_velocity_scale_ = finite_nonnegative_parameter(
      "pre_lio_tracking_step_guard_velocity_scale",
      declare_parameter<double>("pre_lio_tracking_step_guard_velocity_scale", 0.0));
    post_ba_tracking_step_guard_velocity_scale_ = finite_nonnegative_parameter(
      "post_ba_tracking_step_guard_velocity_scale",
      declare_parameter<double>("post_ba_tracking_step_guard_velocity_scale", 0.0));
    tracking_step_guard_acceleration_mps2_ = finite_nonnegative_parameter(
      "tracking_step_guard_acceleration_mps2",
      declare_parameter<double>("tracking_step_guard_acceleration_mps2", 0.0));
    tracking_step_guard_max_velocity_mps_ = finite_nonnegative_parameter(
      "tracking_step_guard_max_velocity_mps",
      declare_parameter<double>("tracking_step_guard_max_velocity_mps", 0.0));
    tracking_step_guard_margin_m_ = finite_nonnegative_parameter(
      "tracking_step_guard_margin_m",
      declare_parameter<double>("tracking_step_guard_margin_m", 0.0));
    trajectory_control_interval_ns_ = integer_parameter_at_least(
      "trajectory_control_interval_ns",
      declare_parameter<int64_t>("trajectory_control_interval_ns", 50000000LL), 1LL);
    enable_sliding_window_optimizer_ = declare_parameter<bool>("enable_sliding_window_optimizer", true);
    enable_gaussian_snapshot_lidar_factor_ =
      declare_parameter<bool>("enable_gaussian_snapshot_lidar_factor", true);
    enable_gaussian_snapshot_lidar_plane_factor_ =
      declare_parameter<bool>("enable_gaussian_snapshot_lidar_plane_factor", false);
    gaussian_snapshot_lidar_min_opacity_ = finite_nonnegative_parameter(
      "gaussian_snapshot_lidar_min_opacity",
      declare_parameter<double>("gaussian_snapshot_lidar_min_opacity", 0.01));
    gaussian_snapshot_lidar_factor_weight_ = finite_positive_parameter(
      "gaussian_snapshot_lidar_factor_weight",
      declare_parameter<double>("gaussian_snapshot_lidar_factor_weight", 1.0));
    gaussian_snapshot_lidar_nearest_distance_m_ = finite_nonnegative_parameter(
      "gaussian_snapshot_lidar_nearest_distance_m",
      declare_parameter<double>("gaussian_snapshot_lidar_nearest_distance_m", 0.0));
    gaussian_snapshot_lidar_residual_preweight_ =
      declare_parameter<bool>("gaussian_snapshot_lidar_residual_preweight", true);
    enable_gaussian_snapshot_lidar_pose_correction_ =
      declare_parameter<bool>("enable_gaussian_snapshot_lidar_pose_correction", false);
    gaussian_snapshot_lidar_pose_correction_gain_ = finite_nonnegative_parameter(
      "gaussian_snapshot_lidar_pose_correction_gain",
      declare_parameter<double>("gaussian_snapshot_lidar_pose_correction_gain", 0.3));
    gaussian_snapshot_lidar_pose_correction_max_translation_m_ = finite_nonnegative_parameter(
      "gaussian_snapshot_lidar_pose_correction_max_translation_m",
      declare_parameter<double>("gaussian_snapshot_lidar_pose_correction_max_translation_m", 0.05));
    gaussian_snapshot_lidar_pose_correction_max_rotation_rad_ = finite_nonnegative_parameter(
      "gaussian_snapshot_lidar_pose_correction_max_rotation_rad",
      declare_parameter<double>("gaussian_snapshot_lidar_pose_correction_max_rotation_rad", 0.02));
    gaussian_snapshot_lidar_pose_correction_min_match_ratio_ = finite_nonnegative_parameter(
      "gaussian_snapshot_lidar_pose_correction_min_match_ratio",
      declare_parameter<double>("gaussian_snapshot_lidar_pose_correction_min_match_ratio", 0.0));
    gaussian_snapshot_lidar_pose_correction_max_mean_residual_m_ = finite_nonnegative_parameter(
      "gaussian_snapshot_lidar_pose_correction_max_mean_residual_m",
      declare_parameter<double>("gaussian_snapshot_lidar_pose_correction_max_mean_residual_m", 0.0));
    gaussian_snapshot_lidar_pose_correction_coverage_grid_cols_ = integer_parameter_at_least(
      "gaussian_snapshot_lidar_pose_correction_coverage_grid_cols",
      declare_parameter<int>("gaussian_snapshot_lidar_pose_correction_coverage_grid_cols", 1), 1);
    gaussian_snapshot_lidar_pose_correction_coverage_grid_rows_ = integer_parameter_at_least(
      "gaussian_snapshot_lidar_pose_correction_coverage_grid_rows",
      declare_parameter<int>("gaussian_snapshot_lidar_pose_correction_coverage_grid_rows", 1), 1);
    gaussian_snapshot_lidar_pose_correction_min_coverage_tiles_ = integer_parameter_at_least(
      "gaussian_snapshot_lidar_pose_correction_min_coverage_tiles",
      declare_parameter<int>("gaussian_snapshot_lidar_pose_correction_min_coverage_tiles", 0), 0);
    gaussian_snapshot_lidar_pose_correction_bidirectional_max_distance_m_ =
      finite_nonnegative_parameter(
      "gaussian_snapshot_lidar_pose_correction_bidirectional_max_distance_m",
      declare_parameter<double>(
        "gaussian_snapshot_lidar_pose_correction_bidirectional_max_distance_m", 0.0));
    gaussian_snapshot_lidar_plane_factor_weight_ = finite_positive_parameter(
      "gaussian_snapshot_lidar_plane_factor_weight",
      declare_parameter<double>("gaussian_snapshot_lidar_plane_factor_weight", 1.0));
    gaussian_snapshot_lidar_plane_min_anisotropy_ = finite_nonnegative_parameter(
      "gaussian_snapshot_lidar_plane_min_anisotropy",
      declare_parameter<double>("gaussian_snapshot_lidar_plane_min_anisotropy", 0.25));
    gaussian_snapshot_lidar_plane_min_anisotropy_ =
      std::min(gaussian_snapshot_lidar_plane_min_anisotropy_, 1.0);
    sliding_window_max_states_ = integer_parameter_at_least(
      "sliding_window_max_states",
      declare_parameter<int>("sliding_window_max_states", 12), 2);
    sliding_window_optimize_every_n_frames_ = integer_parameter_at_least(
      "sliding_window_optimize_every_n_frames",
      declare_parameter<int>("sliding_window_optimize_every_n_frames", 1), 1);
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
    sliding_window_max_feedback_translation_m_ = finite_nonnegative_parameter(
      "sliding_window_max_feedback_translation_m",
      declare_parameter<double>("sliding_window_max_feedback_translation_m", 1.0));
    sliding_window_max_feedback_rotation_rad_ = finite_nonnegative_parameter(
      "sliding_window_max_feedback_rotation_rad",
      declare_parameter<double>("sliding_window_max_feedback_rotation_rad", 0.5));
    sliding_window_max_feedback_velocity_mps_ = finite_nonnegative_parameter(
      "sliding_window_max_feedback_velocity_mps",
      declare_parameter<double>("sliding_window_max_feedback_velocity_mps", 5.0));
    sliding_window_max_feedback_velocity_norm_mps_ = finite_nonnegative_parameter(
      "sliding_window_max_feedback_velocity_norm_mps",
      declare_parameter<double>("sliding_window_max_feedback_velocity_norm_mps", 5.0));
    sliding_window_max_feedback_gyro_bias_norm_ = finite_nonnegative_parameter(
      "sliding_window_max_feedback_gyro_bias_norm",
      declare_parameter<double>("sliding_window_max_feedback_gyro_bias_norm", 0.5));
    sliding_window_max_feedback_accel_bias_norm_ = finite_nonnegative_parameter(
      "sliding_window_max_feedback_accel_bias_norm",
      declare_parameter<double>("sliding_window_max_feedback_accel_bias_norm", 2.5));
    sliding_window_max_feedback_gyro_bias_step_ = finite_nonnegative_parameter(
      "sliding_window_max_feedback_gyro_bias_step",
      declare_parameter<double>("sliding_window_max_feedback_gyro_bias_step", 0.0));
    sliding_window_max_feedback_accel_bias_step_ = finite_nonnegative_parameter(
      "sliding_window_max_feedback_accel_bias_step",
      declare_parameter<double>("sliding_window_max_feedback_accel_bias_step", 0.0));
    sliding_window_min_bias_feedback_visual_factors_ = integer_parameter_at_least(
      "sliding_window_min_bias_feedback_visual_factors",
      declare_parameter<int>("sliding_window_min_bias_feedback_visual_factors", 0), 0);
    sliding_window_bias_feedback_ownership_ =
      parse_sliding_window_bias_feedback_ownership(
      declare_parameter<std::string>("sliding_window_bias_feedback_ownership", "optimized"));
    sliding_window_sync_guarded_pose_state_ =
      declare_parameter<bool>("sliding_window_sync_guarded_pose_state", false);
    sliding_window_guarded_pose_prior_translation_weight_ = finite_nonnegative_parameter(
      "sliding_window_guarded_pose_prior_translation_weight",
      declare_parameter<double>("sliding_window_guarded_pose_prior_translation_weight", 0.0));
    sliding_window_guarded_pose_prior_rotation_weight_ = finite_nonnegative_parameter(
      "sliding_window_guarded_pose_prior_rotation_weight",
      declare_parameter<double>("sliding_window_guarded_pose_prior_rotation_weight", 0.0));
    sliding_window_max_normal_equation_condition_ = finite_positive_parameter(
      "sliding_window_max_normal_equation_condition",
      declare_parameter<double>("sliding_window_max_normal_equation_condition", 1.0e13));
    sliding_window_min_normal_equation_rank_ratio_ = finite_unit_interval_parameter(
      "sliding_window_min_normal_equation_rank_ratio",
      declare_parameter<double>("sliding_window_min_normal_equation_rank_ratio", 0.8));
    sliding_window_max_state_gap_s_ = finite_nonnegative_parameter(
      "sliding_window_max_state_gap_s",
      declare_parameter<double>("sliding_window_max_state_gap_s", 1.0));
    sliding_window_marginalization_prior_weight_ = finite_nonnegative_parameter(
      "sliding_window_marginalization_prior_weight",
      declare_parameter<double>("sliding_window_marginalization_prior_weight", 1.0));
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
    sliding_window_imu_velocity_prior_weight_ = finite_nonnegative_parameter(
      "sliding_window_imu_velocity_prior_weight",
      declare_parameter<double>("sliding_window_imu_velocity_prior_weight", 0.0));
    sliding_window_gyro_bias_prior_weight_ = finite_nonnegative_parameter(
      "sliding_window_gyro_bias_prior_weight",
      declare_parameter<double>("sliding_window_gyro_bias_prior_weight", 0.0));
    sliding_window_accel_bias_prior_weight_ = finite_nonnegative_parameter(
      "sliding_window_accel_bias_prior_weight",
      declare_parameter<double>("sliding_window_accel_bias_prior_weight", 0.0));
    sliding_window_imu_max_extrapolation_s_ = finite_nonnegative_parameter(
      "sliding_window_imu_max_extrapolation_s",
      declare_parameter<double>("sliding_window_imu_max_extrapolation_s", 0.02));
    sliding_window_bias_weight_ = finite_nonnegative_parameter(
      "sliding_window_bias_weight",
      declare_parameter<double>("sliding_window_bias_weight", 1.0));
    sliding_window_gyro_bias_weight_ = finite_nonnegative_parameter(
      "sliding_window_gyro_bias_weight",
      declare_parameter<double>("sliding_window_gyro_bias_weight", 1.0));
    sliding_window_accel_bias_weight_ = finite_nonnegative_parameter(
      "sliding_window_accel_bias_weight",
      declare_parameter<double>("sliding_window_accel_bias_weight", 1.0));
    sliding_window_bias_random_walk_reference_dt_s_ = finite_nonnegative_parameter(
      "sliding_window_bias_random_walk_reference_dt_s",
      declare_parameter<double>("sliding_window_bias_random_walk_reference_dt_s", 0.0));
    sliding_window_gyro_bias_random_walk_sigma_ = finite_nonnegative_parameter(
      "sliding_window_gyro_bias_random_walk_sigma",
      declare_parameter<double>("sliding_window_gyro_bias_random_walk_sigma", 0.0));
    sliding_window_accel_bias_random_walk_sigma_ = finite_nonnegative_parameter(
      "sliding_window_accel_bias_random_walk_sigma",
      declare_parameter<double>("sliding_window_accel_bias_random_walk_sigma", 0.0));
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
    sliding_window_smoothness_position_velocity_weight_ = finite_nonnegative_parameter(
      "sliding_window_smoothness_position_velocity_weight",
      declare_parameter<double>("sliding_window_smoothness_position_velocity_weight", 0.0));
    sliding_window_smoothness_bias_weight_ = finite_nonnegative_parameter(
      "sliding_window_smoothness_bias_weight",
      declare_parameter<double>("sliding_window_smoothness_bias_weight", 0.1));
    sliding_window_smoothness_use_motion_targets_ =
      declare_parameter<bool>("sliding_window_smoothness_use_motion_targets", false);
    sliding_window_smoothness_motion_target_min_visual_factors_ = integer_parameter_at_least(
      "sliding_window_smoothness_motion_target_min_visual_factors",
      declare_parameter<int>("sliding_window_smoothness_motion_target_min_visual_factors", 0),
      0);
    sliding_window_smoothness_motion_target_min_se3_photometric_factors_ =
      integer_parameter_at_least(
      "sliding_window_smoothness_motion_target_min_se3_photometric_factors",
      declare_parameter<int>(
        "sliding_window_smoothness_motion_target_min_se3_photometric_factors", 0),
      0);
    sliding_window_smoothness_motion_target_recent_window_ = integer_parameter_at_least(
      "sliding_window_smoothness_motion_target_recent_window",
      declare_parameter<int>("sliding_window_smoothness_motion_target_recent_window", 0),
      0);
    sliding_window_smoothness_motion_target_min_recent_visual_factors_ =
      integer_parameter_at_least(
      "sliding_window_smoothness_motion_target_min_recent_visual_factors",
      declare_parameter<int>(
        "sliding_window_smoothness_motion_target_min_recent_visual_factors", 0),
      0);
    sliding_window_smoothness_motion_target_min_recent_se3_photometric_factors_ =
      integer_parameter_at_least(
      "sliding_window_smoothness_motion_target_min_recent_se3_photometric_factors",
      declare_parameter<int>(
        "sliding_window_smoothness_motion_target_min_recent_se3_photometric_factors", 0),
      0);
    sliding_window_smoothness_motion_target_start_after_s_ =
      finite_nonnegative_parameter(
      "sliding_window_smoothness_motion_target_start_after_s",
      declare_parameter<double>("sliding_window_smoothness_motion_target_start_after_s", 0.0));
    sliding_window_smoothness_motion_target_max_rotation_rate_delta_radps_ =
      finite_nonnegative_parameter(
      "sliding_window_smoothness_motion_target_max_rotation_rate_delta_radps",
      declare_parameter<double>(
        "sliding_window_smoothness_motion_target_max_rotation_rate_delta_radps", 0.25));
    sliding_window_smoothness_motion_target_max_position_rate_delta_mps_ =
      finite_nonnegative_parameter(
      "sliding_window_smoothness_motion_target_max_position_rate_delta_mps",
      declare_parameter<double>(
        "sliding_window_smoothness_motion_target_max_position_rate_delta_mps", 0.5));
    sliding_window_smoothness_motion_target_max_velocity_acceleration_delta_mps2_ =
      finite_nonnegative_parameter(
      "sliding_window_smoothness_motion_target_max_velocity_acceleration_delta_mps2",
      declare_parameter<double>(
        "sliding_window_smoothness_motion_target_max_velocity_acceleration_delta_mps2", 2.0));
    enable_sliding_window_relative_translation_factor_ =
      declare_parameter<bool>("enable_sliding_window_relative_translation_factor", false);
    sliding_window_relative_translation_weight_ = finite_nonnegative_parameter(
      "sliding_window_relative_translation_weight",
      declare_parameter<double>("sliding_window_relative_translation_weight", 0.0));
    sliding_window_relative_translation_huber_delta_m_ = finite_nonnegative_parameter(
      "sliding_window_relative_translation_huber_delta_m",
      declare_parameter<double>("sliding_window_relative_translation_huber_delta_m", 0.1));
    sliding_window_relative_translation_in_from_frame_ =
      declare_parameter<bool>("sliding_window_relative_translation_in_from_frame", false);
    sliding_window_relative_rotation_weight_ = finite_nonnegative_parameter(
      "sliding_window_relative_rotation_weight",
      declare_parameter<double>("sliding_window_relative_rotation_weight", 0.0));
    sliding_window_relative_rotation_huber_delta_rad_ = finite_nonnegative_parameter(
      "sliding_window_relative_rotation_huber_delta_rad",
      declare_parameter<double>("sliding_window_relative_rotation_huber_delta_rad", 0.05));
    enable_sliding_window_relative_distance_factor_ =
      declare_parameter<bool>("enable_sliding_window_relative_distance_factor", false);
    sliding_window_relative_distance_weight_ = finite_nonnegative_parameter(
      "sliding_window_relative_distance_weight",
      declare_parameter<double>("sliding_window_relative_distance_weight", 0.0));
    sliding_window_relative_distance_huber_delta_m_ = finite_nonnegative_parameter(
      "sliding_window_relative_distance_huber_delta_m",
      declare_parameter<double>("sliding_window_relative_distance_huber_delta_m", 0.1));
    enable_sliding_window_multihop_relative_translation_factor_ =
      declare_parameter<bool>("enable_sliding_window_multihop_relative_translation_factor", false);
    sliding_window_multihop_relative_translation_weight_ = finite_nonnegative_parameter(
      "sliding_window_multihop_relative_translation_weight",
      declare_parameter<double>("sliding_window_multihop_relative_translation_weight", 0.0));
    sliding_window_multihop_relative_translation_huber_delta_m_ = finite_nonnegative_parameter(
      "sliding_window_multihop_relative_translation_huber_delta_m",
      declare_parameter<double>("sliding_window_multihop_relative_translation_huber_delta_m", 0.15));
    sliding_window_multihop_relative_translation_in_from_frame_ =
      declare_parameter<bool>("sliding_window_multihop_relative_translation_in_from_frame", false);
    sliding_window_multihop_relative_rotation_weight_ = finite_nonnegative_parameter(
      "sliding_window_multihop_relative_rotation_weight",
      declare_parameter<double>("sliding_window_multihop_relative_rotation_weight", 0.0));
    sliding_window_multihop_relative_rotation_huber_delta_rad_ = finite_nonnegative_parameter(
      "sliding_window_multihop_relative_rotation_huber_delta_rad",
      declare_parameter<double>("sliding_window_multihop_relative_rotation_huber_delta_rad", 0.08));
    enable_sliding_window_multihop_relative_distance_factor_ =
      declare_parameter<bool>("enable_sliding_window_multihop_relative_distance_factor", false);
    sliding_window_multihop_relative_distance_weight_ = finite_nonnegative_parameter(
      "sliding_window_multihop_relative_distance_weight",
      declare_parameter<double>("sliding_window_multihop_relative_distance_weight", 0.0));
    sliding_window_multihop_relative_distance_huber_delta_m_ = finite_nonnegative_parameter(
      "sliding_window_multihop_relative_distance_huber_delta_m",
      declare_parameter<double>("sliding_window_multihop_relative_distance_huber_delta_m", 0.15));
    sliding_window_multihop_relative_translation_min_dt_s_ = finite_nonnegative_parameter(
      "sliding_window_multihop_relative_translation_min_dt_s",
      declare_parameter<double>("sliding_window_multihop_relative_translation_min_dt_s", 0.45));
    sliding_window_multihop_relative_translation_max_dt_s_ = finite_nonnegative_parameter(
      "sliding_window_multihop_relative_translation_max_dt_s",
      declare_parameter<double>("sliding_window_multihop_relative_translation_max_dt_s", 1.05));
    sliding_window_multihop_relative_translation_max_factors_ = integer_parameter_at_least(
      "sliding_window_multihop_relative_translation_max_factors",
      declare_parameter<int>("sliding_window_multihop_relative_translation_max_factors", 1), 1);
    enable_sliding_window_delayed_published_multihop_relative_translation_factor_ =
      declare_parameter<bool>(
      "enable_sliding_window_delayed_published_multihop_relative_translation_factor", false);
    sliding_window_delayed_published_multihop_start_after_s_ =
      finite_nonnegative_parameter(
      "sliding_window_delayed_published_multihop_start_after_s",
      declare_parameter<double>("sliding_window_delayed_published_multihop_start_after_s", 0.0));
    sliding_window_delayed_published_multihop_max_factors_ = integer_parameter_at_least(
      "sliding_window_delayed_published_multihop_max_factors",
      declare_parameter<int>("sliding_window_delayed_published_multihop_max_factors", 1), 1);
    sliding_window_relative_motion_history_source_ =
      parse_relative_motion_history_source(
      declare_parameter<std::string>("sliding_window_relative_motion_history_source", "pre_ba"));
    sliding_window_relative_motion_history_published_after_s_ =
      finite_nonnegative_parameter(
      "sliding_window_relative_motion_history_published_after_s",
      declare_parameter<double>("sliding_window_relative_motion_history_published_after_s", 0.0));
    if (sliding_window_multihop_relative_translation_max_dt_s_ <
      sliding_window_multihop_relative_translation_min_dt_s_)
    {
      throw std::runtime_error(
              "sliding_window_multihop_relative_translation_max_dt_s must be >= "
              "sliding_window_multihop_relative_translation_min_dt_s");
    }
    enable_imu_gravity_autocalibration_ =
      declare_parameter<bool>("enable_imu_gravity_autocalibration", true);
    imu_gravity_autocalibration_samples_ = integer_parameter_at_least(
      "imu_gravity_autocalibration_samples",
      declare_parameter<int>("imu_gravity_autocalibration_samples", 50), 2);
    imu_gravity_magnitude_m_s2_ = finite_positive_parameter(
      "imu_gravity_magnitude_m_s2",
      declare_parameter<double>("imu_gravity_magnitude_m_s2", 9.80665));
    imu_linear_acceleration_scale_ = finite_positive_parameter(
      "imu_linear_acceleration_scale",
      declare_parameter<double>("imu_linear_acceleration_scale", 1.0));
    imu_gravity_autocalibration_min_norm_m_s2_ = finite_positive_parameter(
      "imu_gravity_autocalibration_min_norm_m_s2",
      declare_parameter<double>("imu_gravity_autocalibration_min_norm_m_s2", 6.0));
    imu_gravity_autocalibration_max_norm_m_s2_ = finite_positive_parameter(
      "imu_gravity_autocalibration_max_norm_m_s2",
      declare_parameter<double>("imu_gravity_autocalibration_max_norm_m_s2", 14.0));
    if (imu_gravity_autocalibration_max_norm_m_s2_ <= imu_gravity_autocalibration_min_norm_m_s2_) {
      throw std::runtime_error(
              "imu_gravity_autocalibration_max_norm_m_s2 must be greater than "
              "imu_gravity_autocalibration_min_norm_m_s2");
    }
    const auto gravity =
      declare_parameter<std::vector<double>>(
      "imu_gravity_w",
      std::vector<double>{0.0, 0.0, -9.80665});
    configured_imu_gravity_w_ = vector3_from_parameter("imu_gravity_w", gravity);
    if (configured_imu_gravity_w_.norm() <= 1.0e-9) {
      configured_imu_gravity_w_ = Eigen::Vector3d{0.0, 0.0, -imu_gravity_magnitude_m_s2_};
    }
    imu_propagator_.set_gravity_w(configured_imu_gravity_w_);
    imu_propagator_.set_max_history_size(static_cast<size_t>(imu_history_size_));
    trajectory_manager_.set_control_interval_ns(trajectory_control_interval_ns_);
    gaussian_lic_tracking::SlidingWindowConfig window_config;
    window_config.max_states = static_cast<size_t>(sliding_window_max_states_);
    sliding_window_effective_max_states_ = window_config.max_states;
    window_config.max_iterations = static_cast<size_t>(sliding_window_max_iterations_);
    window_config.max_rotation_step_rad = sliding_window_max_rotation_step_rad_;
    window_config.max_translation_step_m = sliding_window_max_translation_step_m_;
    window_config.max_velocity_step_mps = sliding_window_max_velocity_step_mps_;
    window_config.max_bias_step = sliding_window_max_bias_step_;
    window_config.max_normal_equation_condition = sliding_window_max_normal_equation_condition_;
    window_config.min_normal_equation_rank_ratio = sliding_window_min_normal_equation_rank_ratio_;
    window_config.max_state_gap_s = sliding_window_max_state_gap_s_;
    window_config.marginalization_prior_weight = sliding_window_marginalization_prior_weight_;
    window_config.visual_marginalization_prior_zero_bias_columns =
      visual_marginalization_prior_zero_bias_columns_;
    window_config.estimate_gravity =
      declare_parameter<bool>("enable_sliding_window_gravity_estimation", false);
    window_config.gravity_estimation_prior_weight =
      declare_parameter<double>("sliding_window_gravity_estimation_prior_weight", 1.0);
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
    lidar_config.pose_iterations = static_cast<size_t>(lidar_pose_factor_iterations_);
    lidar_config.plane_min_neighbors = static_cast<size_t>(lidar_plane_min_neighbors_);
    lidar_config.plane_max_condition = lidar_plane_max_condition_;
    lidar_config.enable_line_factor = enable_lidar_line_factor_;
    lidar_config.line_max_condition = lidar_line_max_condition_;
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
    if (enable_external_odometry_prior_ && !external_odometry_prior_topic_.empty()) {
      external_odometry_prior_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        external_odometry_prior_topic_, frontend_odometry_qos,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
          run_serialized_callback([this, msg]() {
            handle_external_odometry_prior(*msg);
          });
        });
    }

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
    if (enable_rendered_feedback_contract_) {
      rendered_feedback_sub_ =
        create_subscription<gaussian_lic_msgs::msg::RenderedFeedback>(
        rendered_feedback_topic_, make_rendered_feedback_qos(),
        [this](gaussian_lic_msgs::msg::RenderedFeedback::ConstSharedPtr msg) {
          if (enable_rendered_feedback_ingress_queue_) {
            enqueue_rendered_feedback(msg);
            return;
          }
          run_serialized_callback([this, msg]() { handle_rendered_feedback(*msg); });
        });
      // In deterministic offline replay, no wall-clock timer may fire: the
      // ingress queue is drained synchronously inside run_deterministic_replay()
      // after every dispatched message. Skip timer creation entirely so nothing
      // runs async.
      if (enable_rendered_feedback_ingress_queue_ && deterministic_bag_path_.empty()) {
        rendered_feedback_ingress_timer_ = create_wall_timer(
          std::chrono::milliseconds(rendered_feedback_ingress_drain_period_ms_),
          [this]() {
            run_serialized_callback([this]() { drain_rendered_feedback_ingress_queue(); });
          });
      }
    } else {
      rendered_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        rendered_image_topic_, make_rendered_image_qos(),
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
          run_serialized_callback([this, msg]() {
            handle_rendered_image(*msg);
          });
        });
    }
    if (enable_gaussian_snapshot_) {
      gaussian_map_sub_ = create_subscription<gaussian_lic_msgs::msg::GaussianArray>(
        gaussian_map_topic_,
        rclcpp::QoS(static_cast<size_t>(gaussian_snapshot_qos_depth_)).transient_local().reliable(),
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

  const std::string & deterministic_bag_path() const { return deterministic_bag_path_; }

  // Public on_* wrappers used by run_deterministic_replay(). They mirror the
  // live subscription lambdas exactly: each calls the existing private
  // handle_* under the same run_serialized_callback wrapper, so deterministic
  // replay exercises the identical estimator path as production.
  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    run_serialized_callback([this, msg]() { handle_imu(*msg); });
  }

  void on_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    run_serialized_callback([this, msg]() { handle_pointcloud(*msg); });
  }

  void on_image(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    run_serialized_callback([this, msg]() { handle_image(*msg); });
  }

  void on_depth(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    run_serialized_callback([this, msg]() { handle_depth(*msg); });
  }

  void on_camera_info(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    run_serialized_callback([this, msg]() { handle_camera_info(*msg); });
  }

  void on_rendered_feedback(const gaussian_lic_msgs::msg::RenderedFeedback::SharedPtr msg)
  {
    run_serialized_callback([this, msg]() { handle_rendered_feedback(*msg); });
  }

  // Legacy (contract=false) preview path: mirror the live rendered_image
  // subscription lambda (handle_rendered_image under run_serialized_callback).
  void on_rendered_image(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    run_serialized_callback([this, msg]() { handle_rendered_image(*msg); });
  }

  // In-process deterministic replay: read the sensor bag and the recorded
  // rendered-feedback bag, MERGE every message into a single sequence stably
  // sorted by header stamp, then dispatch each message synchronously to the
  // same handlers the live subscriptions use. Because the bags are fixed and
  // dispatch is single-threaded in a deterministic stamp order, the result is
  // reproducible run-to-run (unlike async `ros2 bag play`). The trajectory is
  // written via the existing output_tum_path stream from the pose-publish site.
  void run_deterministic_replay()
  {
    struct ReplayMessage
    {
      int64_t stamp_ns{0};
      std::size_t order{0};  // tie-breaker preserving merge/storage order
      std::string topic;
      std::shared_ptr<rclcpp::SerializedMessage> serialized;
    };

    rclcpp::Serialization<sensor_msgs::msg::Imu> imu_serialization;
    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> pointcloud_serialization;
    rclcpp::Serialization<sensor_msgs::msg::Image> image_serialization;
    rclcpp::Serialization<sensor_msgs::msg::CameraInfo> camera_info_serialization;
    rclcpp::Serialization<gaussian_lic_msgs::msg::RenderedFeedback> feedback_serialization;

    std::vector<ReplayMessage> messages;
    std::size_t order_counter = 0;

    // Helper: peek the header stamp from a serialized message of a known type
    // so we can merge-sort across both bags without deserializing twice for the
    // dispatch (we keep the serialized buffer and deserialize once on dispatch).
    auto load_bag = [&](const std::string & bag_path) {
      if (bag_path.empty()) {
        return;
      }
      rosbag2_cpp::Reader reader;
      reader.open(bag_path);
      while (rclcpp::ok() && reader.has_next()) {
        const auto bag_message = reader.read_next();
        auto serialized =
          std::make_shared<rclcpp::SerializedMessage>(*bag_message->serialized_data);
        const std::string & topic = bag_message->topic_name;
        int64_t stamp_ns = 0;
        bool keep = true;
        if (topic == raw_camera_info_topic_) {
          sensor_msgs::msg::CameraInfo m;
          camera_info_serialization.deserialize_message(serialized.get(), &m);
          stamp_ns = gaussian_lic_tracking::stamp_to_nanoseconds(m.header.stamp);
        } else if (topic == raw_imu_topic_) {
          sensor_msgs::msg::Imu m;
          imu_serialization.deserialize_message(serialized.get(), &m);
          stamp_ns = gaussian_lic_tracking::stamp_to_nanoseconds(m.header.stamp);
        } else if (topic == raw_pointcloud_topic_) {
          sensor_msgs::msg::PointCloud2 m;
          pointcloud_serialization.deserialize_message(serialized.get(), &m);
          stamp_ns = gaussian_lic_tracking::stamp_to_nanoseconds(m.header.stamp);
        } else if (topic == raw_image_topic_) {
          sensor_msgs::msg::Image m;
          image_serialization.deserialize_message(serialized.get(), &m);
          stamp_ns = gaussian_lic_tracking::stamp_to_nanoseconds(m.header.stamp);
        } else if (topic == raw_depth_topic_) {
          sensor_msgs::msg::Image m;
          image_serialization.deserialize_message(serialized.get(), &m);
          stamp_ns = gaussian_lic_tracking::stamp_to_nanoseconds(m.header.stamp);
        } else if (topic == rendered_feedback_topic_) {
          gaussian_lic_msgs::msg::RenderedFeedback m;
          feedback_serialization.deserialize_message(serialized.get(), &m);
          stamp_ns = gaussian_lic_tracking::stamp_to_nanoseconds(m.header.stamp);
        } else if (
          !enable_rendered_feedback_contract_ && topic == rendered_image_topic_) {
          // Legacy (contract=false) preview feedback: the recorded feedback bag
          // carries sensor_msgs/Image on rendered_image_topic_. Merge it in
          // stamp order alongside the observed /camera/image so the legacy
          // rendered<->observed pairing is deterministic.
          sensor_msgs::msg::Image m;
          image_serialization.deserialize_message(serialized.get(), &m);
          stamp_ns = gaussian_lic_tracking::stamp_to_nanoseconds(m.header.stamp);
        } else {
          keep = false;
        }
        if (!keep) {
          continue;
        }
        ReplayMessage entry;
        entry.stamp_ns = stamp_ns;
        entry.order = order_counter++;
        entry.topic = topic;
        entry.serialized = std::move(serialized);
        messages.push_back(std::move(entry));
      }
    };

    load_bag(deterministic_bag_path_);
    load_bag(deterministic_feedback_bag_path_);

    // Stable sort by header stamp; ties keep the (sensor-bag-first) load order.
    std::stable_sort(
      messages.begin(), messages.end(),
      [](const ReplayMessage & lhs, const ReplayMessage & rhs) {
        if (lhs.stamp_ns != rhs.stamp_ns) {
          return lhs.stamp_ns < rhs.stamp_ns;
        }
        return lhs.order < rhs.order;
      });

    std::size_t imu_n = 0, lidar_n = 0, image_n = 0, depth_n = 0, info_n = 0,
      feedback_n = 0, rendered_image_n = 0;
    for (const auto & entry : messages) {
      if (!rclcpp::ok()) {
        break;
      }
      const std::string & topic = entry.topic;
      if (topic == raw_camera_info_topic_) {
        auto msg = std::make_shared<sensor_msgs::msg::CameraInfo>();
        camera_info_serialization.deserialize_message(entry.serialized.get(), msg.get());
        on_camera_info(msg);
        ++info_n;
      } else if (topic == raw_imu_topic_) {
        auto msg = std::make_shared<sensor_msgs::msg::Imu>();
        imu_serialization.deserialize_message(entry.serialized.get(), msg.get());
        on_imu(msg);
        ++imu_n;
      } else if (topic == raw_pointcloud_topic_) {
        auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pointcloud_serialization.deserialize_message(entry.serialized.get(), msg.get());
        on_pointcloud(msg);
        ++lidar_n;
      } else if (topic == raw_image_topic_) {
        auto msg = std::make_shared<sensor_msgs::msg::Image>();
        image_serialization.deserialize_message(entry.serialized.get(), msg.get());
        on_image(msg);
        ++image_n;
      } else if (topic == raw_depth_topic_) {
        auto msg = std::make_shared<sensor_msgs::msg::Image>();
        image_serialization.deserialize_message(entry.serialized.get(), msg.get());
        on_depth(msg);
        ++depth_n;
      } else if (topic == rendered_feedback_topic_) {
        auto msg = std::make_shared<gaussian_lic_msgs::msg::RenderedFeedback>();
        feedback_serialization.deserialize_message(entry.serialized.get(), msg.get());
        on_rendered_feedback(msg);
        ++feedback_n;
      } else if (
        !enable_rendered_feedback_contract_ && topic == rendered_image_topic_) {
        auto msg = std::make_shared<sensor_msgs::msg::Image>();
        image_serialization.deserialize_message(entry.serialized.get(), msg.get());
        on_rendered_image(msg);
        ++rendered_image_n;
      }
      // The production async path drains the rendered-feedback ingress queue on
      // a wall-clock timer (disabled in deterministic mode). Drain it
      // synchronously after every dispatch so feedback is consumed in the same
      // deterministic stamp order.
      if (enable_rendered_feedback_contract_ && enable_rendered_feedback_ingress_queue_) {
        run_serialized_callback([this]() { drain_rendered_feedback_ingress_queue(); });
      }
    }

    if (output_tum_stream_.is_open()) {
      output_tum_stream_.flush();
    }
    RCLCPP_INFO(
      get_logger(),
      "deterministic replay done: total=%zu imu=%zu lidar=%zu image=%zu depth=%zu "
      "info=%zu feedback=%zu rendered_image=%zu",
      messages.size(), imu_n, lidar_n, image_n, depth_n, info_n, feedback_n,
      rendered_image_n);
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

  struct RenderedFeedbackMetadata
  {
    int64_t observed_stamp_ns{0};
    int64_t pose_stamp_ns{0};
    int64_t pointcloud_stamp_ns{0};
    uint64_t frame_index{0};
    uint64_t rendered_preview_index{0};
    bool has_source_pose{false};
    Eigen::Vector3d source_p_w_i{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond source_q_w_i{Eigen::Quaterniond::Identity()};
  };

  struct RenderedFeedbackSourceMotionPose
  {
    int64_t reference_stamp_ns{0};
    uint64_t frame_index{0};
    uint64_t rendered_preview_index{0};
    Eigen::Vector3d p_w_i{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond q_w_i{Eigen::Quaterniond::Identity()};
  };

  struct PendingRenderedFeedbackSourceMotionFactor
  {
    int64_t from_reference_stamp_ns{0};
    int64_t to_reference_stamp_ns{0};
    uint64_t source_id{0};
    Eigen::Vector3d delta_p_w{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond delta_q_from_to{Eigen::Quaterniond::Identity()};
  };

  struct PendingRenderedFeedbackSourceMotionBatch
  {
    PendingRenderedFeedbackSourceMotionFactor factor;
    uint64_t source_factor_count{0};
  };

  struct Se3PhotometricSampleBatch
  {
    std::vector<gaussian_lic_tracking::VisualSe3PhotometricSample> samples;
    size_t candidate_pixels{0};
    size_t sampled_depth_pixels{0};
    size_t accepted_pixels{0};
    size_t rejected_depth_pixels{0};
    size_t rejected_gradient_pixels{0};
    size_t rejected_residual_pixels{0};
    size_t coverage_tiles{0};
    size_t coverage_total_tiles{0};
    double mean_abs_residual{0.0};
  };

  struct PendingPointCloud
  {
    int64_t stamp_ns{0};
    sensor_msgs::msg::PointCloud2 message;
  };

  struct PendingImuMeasurement
  {
    int64_t stamp_ns{0};
    Eigen::Vector3d angular_velocity_rad_s{Eigen::Vector3d::Zero()};
    Eigen::Vector3d linear_acceleration_m_s2{Eigen::Vector3d::Zero()};
  };

  struct PendingVisualAlignmentFactor
  {
    int64_t stamp_ns{0};
    int64_t pair_stamp_delta_ns{0};
    uint64_t source_id{0};
    bool has_reference_pose{false};
    Eigen::Vector3d reference_p_w_i{Eigen::Vector3d::Zero()};
    std::vector<int64_t> reference_support_stamp_ns;
    std::vector<double> reference_support_weights;
    gaussian_lic_tracking::VisualAlignment alignment;
    Eigen::Vector2d component_weight_xy{Eigen::Vector2d::Ones()};
  };

  struct VisualPairKey
  {
    int64_t observed_stamp_ns{0};
    int64_t rendered_stamp_ns{0};
    bool has_rendered_feedback_source{false};
    uint64_t rendered_feedback_frame_index{0};
    uint64_t rendered_feedback_preview_index{0};
  };

  struct QueuedRenderedFeedbackPair
  {
    gaussian_lic_tracking::VisualFrame rendered;
    gaussian_lic_tracking::VisualFrame observed;
    int64_t reference_stamp_ns{0};
  };

  struct PendingSe3PhotometricFactor
  {
    int64_t stamp_ns{0};
    int64_t pair_stamp_delta_ns{0};
    uint64_t source_id{0};
    bool visual_alignment_saturated{false};
    bool has_reference_pose{false};
    Eigen::Vector3d reference_p_w_i{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond reference_q_w_i{Eigen::Quaterniond::Identity()};
    std::vector<int64_t> reference_support_stamp_ns;
    std::vector<double> reference_support_weights;
    double mean_abs_residual{0.0};
    double sample_inlier_ratio{0.0};
    double step_norm{0.0};
    double hessian_condition_number{0.0};
    size_t coverage_tiles{0};
    size_t coverage_total_tiles{0};
    gaussian_lic_tracking::VisualSe3PhotometricLinearization linearization;
  };

  struct VisualFactorReference
  {
    gaussian_lic_tracking::TrajectoryPose pose;
    std::vector<int64_t> support_stamp_ns;
    std::vector<double> support_weights;
    bool interpolated{false};
  };

  struct QosProfileParams
  {
    std::string reliability{"best_effort"};
    std::string history{"keep_last"};
    int depth{5};
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

  struct ExternalPosePrior
  {
    int64_t stamp_ns{0};
    Eigen::Vector3d p_w_i{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond q_w_i{Eigen::Quaterniond::Identity()};
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

  rclcpp::QoS make_rendered_image_qos() const
  {
    rclcpp::QoS qos{rclcpp::KeepLast(static_cast<size_t>(rendered_image_qos_depth_))};

    const std::string durability = normalized_qos_token(rendered_image_qos_durability_);
    if (durability == "transientlocal") {
      qos.transient_local();
    } else if (durability == "volatile") {
      qos.durability_volatile();
    } else {
      throw std::runtime_error(
              "rendered_image_qos_durability must be transient_local or volatile, got " +
              rendered_image_qos_durability_);
    }

    const std::string reliability = normalized_qos_token(rendered_image_qos_reliability_);
    if (reliability == "reliable") {
      qos.reliable();
    } else if (reliability == "besteffort") {
      qos.best_effort();
    } else {
      throw std::runtime_error(
              "rendered_image_qos_reliability must be reliable or best_effort, got " +
              rendered_image_qos_reliability_);
    }

    return qos;
  }

  rclcpp::QoS make_rendered_feedback_qos() const
  {
    rclcpp::QoS qos{rclcpp::KeepLast(static_cast<size_t>(rendered_feedback_qos_depth_))};

    const std::string durability = normalized_qos_token(rendered_feedback_qos_durability_);
    if (durability == "transientlocal") {
      qos.transient_local();
    } else if (durability == "volatile") {
      qos.durability_volatile();
    } else {
      throw std::runtime_error(
              "rendered_feedback_qos_durability must be transient_local or volatile, got " +
              rendered_feedback_qos_durability_);
    }

    const std::string reliability = normalized_qos_token(rendered_feedback_qos_reliability_);
    if (reliability == "reliable") {
      qos.reliable();
    } else if (reliability == "besteffort") {
      qos.best_effort();
    } else {
      throw std::runtime_error(
              "rendered_feedback_qos_reliability must be reliable or best_effort, got " +
              rendered_feedback_qos_reliability_);
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
      const Eigen::Vector3d raw_linear_acceleration{
        msg.linear_acceleration.x,
        msg.linear_acceleration.y,
        msg.linear_acceleration.z};
      const Eigen::Vector3d linear_acceleration =
        imu_linear_acceleration_scale_ * raw_linear_acceleration;
      if (!angular_velocity.allFinite() || !linear_acceleration.allFinite()) {
        ++imu_invalid_measurements_;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "dropping IMU message with non-finite angular velocity or linear acceleration");
        return;
      }
      last_imu_stamp_ns_ = stamp_ns;
      ++num_raw_imus_;
      if (maybe_buffer_imu_for_gravity_autocalibration(
          stamp_ns, angular_velocity, linear_acceleration))
      {
        process_ready_pointcloud_queue();
        return;
      }
      propagate_imu_measurement(stamp_ns, angular_velocity, linear_acceleration);
      process_ready_pointcloud_queue();
    } catch (const std::exception & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "failed to propagate IMU tracking state: %s", ex.what());
    }
  }

  void propagate_imu_measurement(
    const int64_t stamp_ns,
    const Eigen::Vector3d & angular_velocity,
    const Eigen::Vector3d & linear_acceleration)
  {
    imu_propagator_.add_measurement(stamp_ns, angular_velocity, linear_acceleration);
    if (enable_sliding_window_optimizer_) {
      sliding_window_preintegrator_.add_measurement(
        stamp_ns, angular_velocity, linear_acceleration);
      sliding_window_preintegrator_initialized_ = true;
    }
  }

  bool maybe_buffer_imu_for_gravity_autocalibration(
    const int64_t stamp_ns,
    const Eigen::Vector3d & angular_velocity,
    const Eigen::Vector3d & linear_acceleration)
  {
    if (!enable_imu_gravity_autocalibration_ || imu_gravity_autocalibrated_ ||
      imu_propagator_.initialized())
    {
      return false;
    }

    pending_imu_gravity_samples_.push_back(
      PendingImuMeasurement{stamp_ns, angular_velocity, linear_acceleration});
    imu_gravity_autocalibration_samples_collected_ =
      static_cast<uint64_t>(pending_imu_gravity_samples_.size());
    if (pending_imu_gravity_samples_.size() <
      static_cast<size_t>(imu_gravity_autocalibration_samples_))
    {
      return true;
    }

    Eigen::Vector3d mean_accel = Eigen::Vector3d::Zero();
    for (const auto & sample : pending_imu_gravity_samples_) {
      mean_accel += sample.linear_acceleration_m_s2;
    }
    mean_accel /= static_cast<double>(pending_imu_gravity_samples_.size());
    imu_gravity_autocalibration_mean_accel_ = mean_accel;

    Eigen::Quaterniond initial_q_w_i = Eigen::Quaterniond::Identity();
    const double accel_norm = mean_accel.norm();
    const double gravity_norm = configured_imu_gravity_w_.norm();
    if (accel_norm >= imu_gravity_autocalibration_min_norm_m_s2_ &&
      accel_norm <= imu_gravity_autocalibration_max_norm_m_s2_ &&
      gravity_norm > 1.0e-9)
    {
      initial_q_w_i = Eigen::Quaterniond::FromTwoVectors(
        mean_accel.normalized(),
        (-configured_imu_gravity_w_).normalized()).normalized();
      imu_gravity_autocalibration_failed_ = false;
    } else {
      imu_gravity_autocalibration_failed_ = true;
      RCLCPP_WARN(
        get_logger(),
        "IMU gravity autocalibration fell back to identity orientation: mean accel norm %.6f "
        "outside [%.6f, %.6f] or invalid gravity norm %.6f",
        accel_norm,
        imu_gravity_autocalibration_min_norm_m_s2_,
        imu_gravity_autocalibration_max_norm_m_s2_,
        gravity_norm);
    }

    gaussian_lic_tracking::ImuState initial;
    initial.stamp_ns = pending_imu_gravity_samples_.front().stamp_ns;
    initial.q_w_i = initial_q_w_i;
    imu_propagator_.set_gravity_w(configured_imu_gravity_w_);
    imu_propagator_.reset_with_measurement(
      initial,
      pending_imu_gravity_samples_.front().angular_velocity_rad_s,
      pending_imu_gravity_samples_.front().linear_acceleration_m_s2);
    if (enable_sliding_window_optimizer_) {
      sliding_window_preintegrator_ = gaussian_lic_tracking::ImuPreintegrator{};
      sliding_window_preintegrator_.add_measurement(
        initial.stamp_ns,
        pending_imu_gravity_samples_.front().angular_velocity_rad_s,
        pending_imu_gravity_samples_.front().linear_acceleration_m_s2);
      sliding_window_preintegrator_initialized_ = true;
    }
    for (size_t i = 1U; i < pending_imu_gravity_samples_.size(); ++i) {
      propagate_imu_measurement(
        pending_imu_gravity_samples_[i].stamp_ns,
        pending_imu_gravity_samples_[i].angular_velocity_rad_s,
        pending_imu_gravity_samples_[i].linear_acceleration_m_s2);
    }
    imu_gravity_autocalibrated_ = true;
    RCLCPP_INFO(
      get_logger(),
      "IMU gravity autocalibration initialized q_w_i=(%.6f, %.6f, %.6f, %.6f), "
      "gravity_w=(%.6f, %.6f, %.6f), mean_accel=(%.6f, %.6f, %.6f), samples=%zu",
      initial_q_w_i.w(),
      initial_q_w_i.x(),
      initial_q_w_i.y(),
      initial_q_w_i.z(),
      configured_imu_gravity_w_.x(),
      configured_imu_gravity_w_.y(),
      configured_imu_gravity_w_.z(),
      mean_accel.x(),
      mean_accel.y(),
      mean_accel.z(),
      pending_imu_gravity_samples_.size());
    pending_imu_gravity_samples_.clear();
    return true;
  }

  void handle_external_odometry_prior(const nav_msgs::msg::Odometry & msg)
  {
    try {
      const int64_t stamp_ns = gaussian_lic_tracking::stamp_to_nanoseconds(msg.header.stamp);
      if (!accept_stream_stamp(
          "external_odometry_prior",
          stamp_ns,
          last_external_odometry_prior_input_stamp_ns_,
          external_odometry_prior_stamp_regressions_,
          false))
      {
        return;
      }
      if (!valid_pose_msg(msg.pose.pose)) {
        ++external_odometry_prior_invalid_messages_;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "dropping external odometry prior with non-finite pose or invalid quaternion");
        return;
      }
      external_odometry_priors_.push_back(
        external_pose_prior_from_msg(stamp_ns, msg.pose.pose));
      last_external_odometry_prior_stamp_ns_ = stamp_ns;
      ++external_odometry_priors_received_;
      while (external_odometry_priors_.size() >
        static_cast<size_t>(external_odometry_prior_cache_size_))
      {
        external_odometry_priors_.pop_front();
      }
    } catch (const std::exception & ex) {
      ++external_odometry_prior_invalid_messages_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "failed to cache external odometry prior: %s", ex.what());
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
      ++camera_info_invalid_intrinsics_;
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

  static bool valid_sliding_window_state(
    const gaussian_lic_tracking::SlidingWindowState & state)
  {
    return state.p_w_i.allFinite() && state.v_w_i.allFinite() &&
           state.gyro_bias.allFinite() && state.accel_bias.allFinite() &&
           state.q_w_i.coeffs().allFinite() &&
           state.q_w_i.norm() > std::numeric_limits<double>::epsilon();
  }

  static bool valid_trajectory_pose(const gaussian_lic_tracking::TrajectoryPose & pose)
  {
    return pose.p_w_i.allFinite() && pose.v_w_i.allFinite() &&
           pose.q_w_i.coeffs().allFinite() &&
           pose.q_w_i.norm() > std::numeric_limits<double>::epsilon();
  }

  static void clamp_vector_norm(Eigen::Vector3d & value, const double max_norm)
  {
    const double norm = value.norm();
    if (max_norm > 0.0 && std::isfinite(norm) && norm > max_norm) {
      value *= max_norm / norm;
    }
  }

  static void clamp_vector_step(
    Eigen::Vector3d & value,
    const Eigen::Vector3d & reference,
    const double max_step)
  {
    Eigen::Vector3d delta = value - reference;
    const double norm = delta.norm();
    if (max_step > 0.0 && std::isfinite(norm) && norm > max_step) {
      value = reference + delta * (max_step / norm);
    }
  }

  static bool valid_pose_msg(const geometry_msgs::msg::Pose & pose)
  {
    const Eigen::Vector3d p{
      pose.position.x,
      pose.position.y,
      pose.position.z};
    const Eigen::Quaterniond q{
      pose.orientation.w,
      pose.orientation.x,
      pose.orientation.y,
      pose.orientation.z};
    return p.allFinite() && q.coeffs().allFinite() &&
           q.norm() > std::numeric_limits<double>::epsilon();
  }

  static ExternalPosePrior external_pose_prior_from_msg(
    const int64_t stamp_ns,
    const geometry_msgs::msg::Pose & pose)
  {
    ExternalPosePrior prior;
    prior.stamp_ns = stamp_ns;
    prior.p_w_i = Eigen::Vector3d{
      pose.position.x,
      pose.position.y,
      pose.position.z};
    prior.q_w_i = Eigen::Quaterniond{
      pose.orientation.w,
      pose.orientation.x,
      pose.orientation.y,
      pose.orientation.z}.normalized();
    return prior;
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

  static RelativeMotionHistorySource parse_relative_motion_history_source(const std::string & value)
  {
    if (value == "pre_ba") {
      return RelativeMotionHistorySource::kPreBa;
    }
    if (value == "published") {
      return RelativeMotionHistorySource::kPublished;
    }
    if (value == "published_after_warmup") {
      return RelativeMotionHistorySource::kPublishedAfterWarmup;
    }
    throw std::runtime_error(
            "sliding_window_relative_motion_history_source must be pre_ba, published, "
            "or published_after_warmup");
  }

  static const char * relative_motion_history_source_name(
    const RelativeMotionHistorySource source)
  {
    switch (source) {
      case RelativeMotionHistorySource::kPreBa:
        return "pre_ba";
      case RelativeMotionHistorySource::kPublished:
        return "published";
      case RelativeMotionHistorySource::kPublishedAfterWarmup:
        return "published_after_warmup";
    }
    return "pre_ba";
  }

  const gaussian_lic_tracking::TrajectoryPose & select_relative_motion_history_pose(
    const gaussian_lic_tracking::TrajectoryPose & pre_ba_pose,
    const gaussian_lic_tracking::TrajectoryPose & published_pose) const
  {
    if (sliding_window_relative_motion_history_source_ == RelativeMotionHistorySource::kPublished) {
      return published_pose;
    }
    if (sliding_window_relative_motion_history_source_ ==
      RelativeMotionHistorySource::kPublishedAfterWarmup &&
      sliding_window_start_stamp_ns_.has_value())
    {
      const double elapsed_s =
        static_cast<double>(published_pose.stamp_ns - sliding_window_start_stamp_ns_.value()) /
        static_cast<double>(gaussian_lic_tracking::kNanosecondsPerSecond);
      if (elapsed_s >= sliding_window_relative_motion_history_published_after_s_) {
        return published_pose;
      }
    }
    return pre_ba_pose;
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

  void observe_rendered_feedback_stamp(const int64_t stamp_ns)
  {
    if (last_rendered_input_stamp_ns_.has_value() &&
      stamp_ns < last_rendered_input_stamp_ns_.value())
    {
      ++rendered_stamp_regressions_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "accepting typed rendered feedback with non-monotonic image stamp because source "
        "frame/preview ids own the feedback: current=%ld previous=%ld",
        static_cast<long>(stamp_ns),
        static_cast<long>(last_rendered_input_stamp_ns_.value()));
      return;
    }
    last_rendered_input_stamp_ns_ = stamp_ns;
  }

  bool accept_rendered_feedback_source_order(const RenderedFeedbackMetadata & metadata)
  {
    last_rendered_feedback_frame_index_ = metadata.frame_index;
    last_rendered_feedback_preview_index_ = metadata.rendered_preview_index;

    const auto source_id = std::make_pair(metadata.frame_index, metadata.rendered_preview_index);
    if (!seen_rendered_feedback_source_ids_.insert(source_id).second) {
      ++rendered_feedback_duplicate_source_ids_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "dropping duplicate typed rendered feedback source id: frame=%lu preview=%lu",
        static_cast<unsigned long>(metadata.frame_index),
        static_cast<unsigned long>(metadata.rendered_preview_index));
      return false;
    }

    if (last_ordered_rendered_feedback_frame_index_.has_value()) {
      const auto previous = last_ordered_rendered_feedback_frame_index_.value();
      if (metadata.frame_index < previous) {
        ++rendered_feedback_frame_index_regressions_;
      } else {
        if (metadata.frame_index > previous + 1U) {
          ++rendered_feedback_frame_index_gap_count_;
          rendered_feedback_frame_index_missing_ += metadata.frame_index - previous - 1U;
        }
        last_ordered_rendered_feedback_frame_index_ = metadata.frame_index;
      }
    } else {
      last_ordered_rendered_feedback_frame_index_ = metadata.frame_index;
    }

    if (last_ordered_rendered_feedback_preview_index_.has_value()) {
      const auto previous = last_ordered_rendered_feedback_preview_index_.value();
      if (metadata.rendered_preview_index < previous) {
        ++rendered_feedback_preview_index_regressions_;
      } else {
        if (metadata.rendered_preview_index > previous + 1U) {
          ++rendered_feedback_preview_index_gap_count_;
          rendered_feedback_preview_index_missing_ +=
            metadata.rendered_preview_index - previous - 1U;
        }
        last_ordered_rendered_feedback_preview_index_ = metadata.rendered_preview_index;
      }
    } else {
      last_ordered_rendered_feedback_preview_index_ = metadata.rendered_preview_index;
    }

    if (metadata.frame_index < last_ordered_rendered_feedback_frame_index_.value() ||
      metadata.rendered_preview_index < last_ordered_rendered_feedback_preview_index_.value())
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "dropping regressed typed rendered feedback source id: frame=%lu preview=%lu "
        "max_frame=%lu max_preview=%lu",
        static_cast<unsigned long>(metadata.frame_index),
        static_cast<unsigned long>(metadata.rendered_preview_index),
        static_cast<unsigned long>(last_ordered_rendered_feedback_frame_index_.value()),
        static_cast<unsigned long>(last_ordered_rendered_feedback_preview_index_.value()));
      return false;
    }
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
    } else {
      ++depth_invalid_frames_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "dropping depth image with unsupported encoding, layout, or dimensions");
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
      ++image_invalid_frames_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "dropping camera image with unsupported encoding, layout, or dimensions");
      return;
    }
    last_observed_image_width_ = observed.width;
    last_observed_image_height_ = observed.height;
    cache_observed_frame(observed);
    int64_t rendered_match_delta_ns = 0;
    int64_t rendered_nearest_delta_ns = 0;
    int64_t rendered_nearest_signed_delta_ns = 0;
    bool rendered_cache_had_size_match = false;
    const gaussian_lic_tracking::VisualFrame * rendered_frame =
      select_rendered_frame_for_stamp(
      observed.stamp_ns,
      observed.width,
      observed.height,
      &rendered_match_delta_ns,
      &rendered_cache_had_size_match,
      visual_pair_requires_unique_rendered_stamp(),
      &rendered_nearest_delta_ns,
      &rendered_nearest_signed_delta_ns);
    last_visual_rendered_cache_size_ = rendered_frame_cache_.size();
    last_visual_rendered_match_delta_ns_ = rendered_frame == nullptr ? 0 : rendered_match_delta_ns;
    last_visual_rendered_nearest_delta_ns_ = rendered_nearest_delta_ns;
    last_visual_rendered_nearest_signed_delta_ns_ = rendered_nearest_signed_delta_ns;
    if (rendered_frame == nullptr) {
      if (rendered_frame_cache_.empty()) {
        ++visual_rendered_miss_count_;
      } else if (!rendered_cache_had_size_match) {
        ++visual_rendered_size_mismatch_count_;
      } else {
        ++visual_rendered_stale_count_;
      }
    }
    if (rendered_frame != nullptr && !visual_pair_processing_defer_to_pointcloud_ &&
      !enable_visual_watermark_pair_scheduler_)
    {
      process_visual_pair(*rendered_frame, observed, false);
    }
    if (!visual_cache_reconciliation_defer_to_pointcloud_ &&
      !visual_pair_processing_defer_to_pointcloud_ &&
      !enable_visual_watermark_pair_scheduler_)
    {
      reconcile_visual_frame_caches();
    }
    if (enable_visual_watermark_pair_scheduler_ && last_pointcloud_stamp_ns_ > 0) {
      process_visual_pairs_up_to_watermark(last_pointcloud_stamp_ns_, true);
    }
  }

  void handle_rendered_image(const sensor_msgs::msg::Image & msg)
  {
    handle_rendered_image_with_metadata(msg, std::nullopt);
  }

  void enqueue_rendered_feedback(
    const gaussian_lic_msgs::msg::RenderedFeedback::ConstSharedPtr & msg)
  {
    if (!msg) {
      return;
    }
    ++rendered_feedback_ingress_received_;
    std::scoped_lock lock(rendered_feedback_ingress_mutex_);
    while (rendered_feedback_ingress_queue_.size() >=
      static_cast<size_t>(rendered_feedback_ingress_queue_size_))
    {
      rendered_feedback_ingress_queue_.pop_front();
      ++rendered_feedback_ingress_drops_;
    }
    rendered_feedback_ingress_queue_.push_back(msg);
    rendered_feedback_ingress_queue_last_size_ = rendered_feedback_ingress_queue_.size();
    rendered_feedback_ingress_queue_peak_size_ = std::max(
      rendered_feedback_ingress_queue_peak_size_, rendered_feedback_ingress_queue_last_size_);
  }

  void drain_rendered_feedback_ingress_queue()
  {
    if (!enable_rendered_feedback_ingress_queue_) {
      return;
    }
    std::vector<gaussian_lic_msgs::msg::RenderedFeedback::ConstSharedPtr> batch;
    batch.reserve(static_cast<size_t>(rendered_feedback_ingress_drain_max_per_cycle_));
    {
      std::scoped_lock lock(rendered_feedback_ingress_mutex_);
      while (!rendered_feedback_ingress_queue_.empty() &&
        batch.size() < static_cast<size_t>(rendered_feedback_ingress_drain_max_per_cycle_))
      {
        batch.push_back(rendered_feedback_ingress_queue_.front());
        rendered_feedback_ingress_queue_.pop_front();
      }
      rendered_feedback_ingress_queue_last_size_ = rendered_feedback_ingress_queue_.size();
    }
    for (const auto & msg : batch) {
      ++rendered_feedback_ingress_drained_;
      handle_rendered_feedback(*msg);
    }
  }

  void handle_rendered_feedback(const gaussian_lic_msgs::msg::RenderedFeedback & msg)
  {
    ++num_rendered_feedbacks_;
    RenderedFeedbackMetadata metadata;
    try {
      const int64_t rendered_stamp_ns =
        gaussian_lic_tracking::stamp_to_nanoseconds(msg.header.stamp);
      metadata.observed_stamp_ns =
        gaussian_lic_tracking::stamp_to_nanoseconds(msg.observed_stamp);
      metadata.pose_stamp_ns =
        gaussian_lic_tracking::stamp_to_nanoseconds(msg.pose_stamp);
      metadata.pointcloud_stamp_ns =
        gaussian_lic_tracking::stamp_to_nanoseconds(msg.pointcloud_stamp);
      metadata.frame_index = msg.frame_index;
      metadata.rendered_preview_index = msg.rendered_preview_index;
      if (valid_pose_msg(msg.source_pose)) {
        metadata.has_source_pose = true;
        metadata.source_p_w_i = Eigen::Vector3d{
          msg.source_pose.position.x,
          msg.source_pose.position.y,
          msg.source_pose.position.z};
        metadata.source_q_w_i = Eigen::Quaterniond{
          msg.source_pose.orientation.w,
          msg.source_pose.orientation.x,
          msg.source_pose.orientation.y,
          msg.source_pose.orientation.z}.normalized();
      } else if (enable_rendered_feedback_source_pose_reference_) {
        ++rendered_feedback_source_pose_invalid_;
      }
      last_rendered_feedback_observed_delta_ns_ =
        metadata.observed_stamp_ns - rendered_stamp_ns;
      last_rendered_feedback_pose_delta_ns_ =
        metadata.pose_stamp_ns - rendered_stamp_ns;
      last_rendered_feedback_pointcloud_delta_ns_ =
        metadata.pointcloud_stamp_ns - rendered_stamp_ns;
      update_rendered_feedback_active_window_telemetry(
        rendered_feedback_reference_stamp_ns(metadata, rendered_stamp_ns));
      if (!accept_rendered_feedback_source_order(metadata)) {
        return;
      }
      queue_rendered_feedback_source_motion_factor(metadata, rendered_stamp_ns);
      auto image = msg.image;
      if (
        gaussian_lic_tracking::stamp_to_nanoseconds(image.header.stamp) != rendered_stamp_ns)
      {
        ++rendered_feedback_stamp_mismatches_;
        image.header.stamp = msg.header.stamp;
      }
      auto observed_image = msg.observed_image;
      if (!observed_image.data.empty() && observed_image.width > 0U && observed_image.height > 0U) {
        if (
          gaussian_lic_tracking::stamp_to_nanoseconds(observed_image.header.stamp) !=
          metadata.observed_stamp_ns)
        {
          ++rendered_feedback_stamp_mismatches_;
          observed_image.header.stamp = msg.observed_stamp;
        }
        auto observed_depth_image = msg.observed_depth_image;
        if (
          !observed_depth_image.data.empty() &&
          gaussian_lic_tracking::stamp_to_nanoseconds(observed_depth_image.header.stamp) !=
          metadata.observed_stamp_ns)
        {
          ++rendered_feedback_stamp_mismatches_;
          observed_depth_image.header.stamp = msg.observed_stamp;
        }
        handle_rendered_feedback_pair(image, observed_image, observed_depth_image, metadata);
      } else {
        handle_rendered_image_with_metadata(image, metadata);
      }
    } catch (const std::exception & ex) {
      ++rendered_invalid_frames_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "dropping rendered feedback with invalid source stamps: %s", ex.what());
    }
  }

  void handle_rendered_feedback_pair(
    const sensor_msgs::msg::Image & rendered_msg,
    const sensor_msgs::msg::Image & observed_msg,
    const sensor_msgs::msg::Image & observed_depth_msg,
    const RenderedFeedbackMetadata & metadata)
  {
    if (!enable_visual_factor_) {
      return;
    }
    const int64_t stamp_ns = gaussian_lic_tracking::stamp_to_nanoseconds(rendered_msg.header.stamp);
    observe_rendered_feedback_stamp(stamp_ns);

    gaussian_lic_tracking::VisualFrame rendered;
    gaussian_lic_tracking::VisualFrame observed;
    if (!decode_image_gray(rendered_msg, rendered)) {
      ++rendered_invalid_frames_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "dropping rendered feedback image with unsupported encoding, layout, or dimensions");
      return;
    }
    if (!decode_image_gray(observed_msg, observed)) {
      ++image_invalid_frames_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "dropping rendered feedback observed image with unsupported encoding, layout, or dimensions");
      return;
    }
    if (!observed_depth_msg.data.empty() && observed_depth_msg.width > 0U &&
      observed_depth_msg.height > 0U)
    {
      DepthFrame embedded_depth;
      if (decode_depth_image(observed_depth_msg, embedded_depth) &&
        embedded_depth.width == observed.width &&
        embedded_depth.height == observed.height &&
        embedded_depth.depth_m.size() == observed.width * observed.height)
      {
        observed.has_embedded_depth = true;
        observed.embedded_depth_m = std::move(embedded_depth.depth_m);
        ++rendered_feedback_embedded_depth_pairs_;
      } else {
        ++rendered_feedback_embedded_depth_invalid_;
        ++depth_invalid_frames_;
      }
    }

    ++num_rendered_images_;
    ++rendered_feedback_embedded_observed_pairs_;
    rendered.has_rendered_feedback_metadata = true;
    rendered.rendered_feedback_observed_stamp_ns = metadata.observed_stamp_ns;
    rendered.rendered_feedback_pose_stamp_ns = metadata.pose_stamp_ns;
    rendered.rendered_feedback_pointcloud_stamp_ns = metadata.pointcloud_stamp_ns;
    rendered.rendered_feedback_frame_index = metadata.frame_index;
    rendered.rendered_feedback_preview_index = metadata.rendered_preview_index;
    rendered.has_rendered_feedback_source_pose = metadata.has_source_pose;
    rendered.rendered_feedback_source_p_w_i = metadata.source_p_w_i;
    rendered.rendered_feedback_source_q_w_i = metadata.source_q_w_i;
    last_visual_rendered_match_delta_ns_ = 0;
    last_visual_rendered_nearest_delta_ns_ = 0;
    last_visual_rendered_nearest_signed_delta_ns_ = 0;
    last_visual_observed_match_delta_ns_ = 0;
    last_visual_observed_nearest_delta_ns_ = 0;
    last_visual_observed_nearest_signed_delta_ns_ = 0;
    if (enable_rendered_feedback_watermark_queue_) {
      queue_rendered_feedback_pair(rendered, observed);
      if (last_pointcloud_stamp_ns_ > 0) {
        process_rendered_feedback_pairs_up_to_watermark(last_pointcloud_stamp_ns_, true);
      }
      return;
    }

    cache_rendered_frame(rendered);
    cache_observed_frame(observed);
    last_visual_rendered_cache_size_ = rendered_frame_cache_.size();
    last_visual_observed_cache_size_ = observed_frame_cache_.size();

    if (!visual_pair_processing_defer_to_pointcloud_ && !enable_visual_watermark_pair_scheduler_) {
      process_visual_pair(rendered, observed, false, true);
    }
    if (!visual_cache_reconciliation_defer_to_pointcloud_ &&
      !visual_pair_processing_defer_to_pointcloud_ &&
      !enable_visual_watermark_pair_scheduler_)
    {
      reconcile_visual_frame_caches();
    }
    if (enable_visual_watermark_pair_scheduler_ && last_pointcloud_stamp_ns_ > 0) {
      process_visual_pairs_up_to_watermark(last_pointcloud_stamp_ns_, true);
    }
  }

  void queue_rendered_feedback_pair(
    const gaussian_lic_tracking::VisualFrame & rendered,
    const gaussian_lic_tracking::VisualFrame & observed)
  {
    const auto max_queue_size =
      static_cast<size_t>(std::max(1, visual_pending_factor_queue_size_));
    QueuedRenderedFeedbackPair queued;
    queued.rendered = rendered;
    queued.observed = observed;
    queued.reference_stamp_ns = visual_factor_reference_stamp_ns(observed, rendered);
    const auto queue_less =
      [](const QueuedRenderedFeedbackPair & lhs, const QueuedRenderedFeedbackPair & rhs) {
        if (lhs.reference_stamp_ns != rhs.reference_stamp_ns) {
          return lhs.reference_stamp_ns < rhs.reference_stamp_ns;
        }
        if (lhs.rendered.rendered_feedback_frame_index != rhs.rendered.rendered_feedback_frame_index) {
          return lhs.rendered.rendered_feedback_frame_index <
                 rhs.rendered.rendered_feedback_frame_index;
        }
        if (lhs.rendered.rendered_feedback_preview_index !=
          rhs.rendered.rendered_feedback_preview_index)
        {
          return lhs.rendered.rendered_feedback_preview_index <
                 rhs.rendered.rendered_feedback_preview_index;
        }
        if (lhs.rendered.stamp_ns != rhs.rendered.stamp_ns) {
          return lhs.rendered.stamp_ns < rhs.rendered.stamp_ns;
        }
        return lhs.observed.stamp_ns < rhs.observed.stamp_ns;
      };
    const auto insert_it = std::upper_bound(
      rendered_feedback_watermark_queue_.begin(),
      rendered_feedback_watermark_queue_.end(),
      queued,
      queue_less);
    if (insert_it != rendered_feedback_watermark_queue_.end()) {
      ++rendered_feedback_watermark_reordered_pairs_;
    }
    rendered_feedback_watermark_queue_.insert(insert_it, std::move(queued));
    while (rendered_feedback_watermark_queue_.size() > max_queue_size) {
      rendered_feedback_watermark_queue_.pop_front();
      ++rendered_feedback_watermark_queue_drops_;
    }
  }

  void process_rendered_feedback_pairs_up_to_watermark(
    const int64_t watermark_stamp_ns,
    const bool ingest_after_processing)
  {
    if (!enable_rendered_feedback_watermark_queue_ || !enable_visual_factor_) {
      return;
    }
    size_t processed = 0U;
    while (!rendered_feedback_watermark_queue_.empty()) {
      const auto & queued = rendered_feedback_watermark_queue_.front();
      if (queued.reference_stamp_ns > watermark_stamp_ns) {
        ++rendered_feedback_watermark_deferred_pairs_;
        break;
      }
      auto ready = std::move(rendered_feedback_watermark_queue_.front());
      rendered_feedback_watermark_queue_.pop_front();
      process_visual_pair(ready.rendered, ready.observed, false);
      ++rendered_feedback_watermark_processed_pairs_;
      ++processed;
    }
    if (ingest_after_processing && processed > 0U && last_output_tracking_pose_.has_value()) {
      ingest_pending_visual_factors_into_optimizer(last_output_tracking_pose_.value());
    }
  }

  int64_t rendered_feedback_reference_stamp_ns(
    const RenderedFeedbackMetadata & metadata,
    const int64_t rendered_stamp_ns) const
  {
    if (visual_factor_reference_stamp_mode_ == VisualFactorReferenceStampMode::kRendered) {
      return rendered_stamp_ns;
    }
    if (visual_factor_reference_stamp_mode_ == VisualFactorReferenceStampMode::kRenderedSourcePose) {
      return metadata.pose_stamp_ns;
    }
    if (
      visual_factor_reference_stamp_mode_ ==
      VisualFactorReferenceStampMode::kRenderedSourcePointcloud)
    {
      return metadata.pointcloud_stamp_ns;
    }
    return metadata.observed_stamp_ns;
  }

  void update_rendered_feedback_active_window_telemetry(const int64_t reference_stamp_ns)
  {
    last_rendered_feedback_reference_stamp_ns_ = reference_stamp_ns;
    const auto & states = sliding_window_optimizer_.states();
    if (states.empty()) {
      last_rendered_feedback_oldest_active_state_delta_ns_ = 0;
      last_rendered_feedback_newest_active_state_delta_ns_ = 0;
      return;
    }

    last_rendered_feedback_oldest_active_state_delta_ns_ =
      reference_stamp_ns - states.front().stamp_ns;
    last_rendered_feedback_newest_active_state_delta_ns_ =
      reference_stamp_ns - states.back().stamp_ns;
    if (last_rendered_feedback_oldest_active_state_delta_ns_ < 0) {
      ++rendered_feedback_before_active_window_;
    }
    if (last_rendered_feedback_newest_active_state_delta_ns_ > 0) {
      ++rendered_feedback_after_active_window_;
    }
  }

  uint64_t rendered_feedback_source_motion_source_id(
    const RenderedFeedbackSourceMotionPose & from_pose,
    const RenderedFeedbackSourceMotionPose & to_pose) const
  {
    uint64_t mixed = 0x73ab86d5d3f4834bULL;
    mixed = mix_visual_factor_source_id(mixed, from_pose.frame_index);
    mixed = mix_visual_factor_source_id(mixed, from_pose.rendered_preview_index);
    mixed = mix_visual_factor_source_id(mixed, to_pose.frame_index);
    mixed = mix_visual_factor_source_id(mixed, to_pose.rendered_preview_index);
    mixed = mix_visual_factor_source_id(
      mixed, static_cast<uint64_t>(from_pose.reference_stamp_ns));
    return mix_visual_factor_source_id(mixed, static_cast<uint64_t>(to_pose.reference_stamp_ns));
  }

  bool source_motion_pose_is_valid(const RenderedFeedbackSourceMotionPose & pose) const
  {
    return pose.reference_stamp_ns > 0 &&
           pose.p_w_i.allFinite() &&
           pose.q_w_i.coeffs().allFinite() &&
           pose.q_w_i.norm() > std::numeric_limits<double>::epsilon();
  }

  void queue_rendered_feedback_source_motion_factor(
    const RenderedFeedbackMetadata & metadata,
    const int64_t rendered_stamp_ns)
  {
    if (!enable_rendered_feedback_source_motion_factor_) {
      return;
    }
    if (rendered_feedback_source_motion_translation_weight_ <= 0.0) {
      return;
    }
    RenderedFeedbackSourceMotionPose current;
    current.reference_stamp_ns = rendered_feedback_reference_stamp_ns(metadata, rendered_stamp_ns);
    current.frame_index = metadata.frame_index;
    current.rendered_preview_index = metadata.rendered_preview_index;
    current.p_w_i = metadata.source_p_w_i;
    current.q_w_i = metadata.source_q_w_i.normalized();
    if (!metadata.has_source_pose || !source_motion_pose_is_valid(current)) {
      ++rendered_feedback_source_motion_invalid_;
      return;
    }

    if (last_rendered_feedback_source_motion_pose_.has_value()) {
      const auto previous = last_rendered_feedback_source_motion_pose_.value();
      const int64_t dt_ns = current.reference_stamp_ns - previous.reference_stamp_ns;
      if (dt_ns <= 0) {
        ++rendered_feedback_source_motion_invalid_;
      } else {
        const double dt_s =
          static_cast<double>(dt_ns) /
          static_cast<double>(gaussian_lic_tracking::kNanosecondsPerSecond);
        if (dt_s < rendered_feedback_source_motion_min_dt_s_ ||
          dt_s > rendered_feedback_source_motion_max_dt_s_)
        {
          ++rendered_feedback_source_motion_dt_skip_count_;
        } else {
          PendingRenderedFeedbackSourceMotionFactor pending;
          pending.from_reference_stamp_ns = previous.reference_stamp_ns;
          pending.to_reference_stamp_ns = current.reference_stamp_ns;
          pending.source_id = rendered_feedback_source_motion_source_id(previous, current);
          const Eigen::Vector3d delta_p_w = current.p_w_i - previous.p_w_i;
          pending.delta_p_w = rendered_feedback_source_motion_in_from_frame_
            ? previous.q_w_i.conjugate() * delta_p_w
            : delta_p_w;
          pending.delta_q_from_to =
            (previous.q_w_i.conjugate() * current.q_w_i).normalized();
          if (pending.delta_p_w.allFinite() &&
            pending.delta_q_from_to.coeffs().allFinite() &&
            pending.delta_q_from_to.norm() > std::numeric_limits<double>::epsilon())
          {
            while (pending_rendered_feedback_source_motion_factors_.size() >=
              static_cast<size_t>(visual_pending_factor_queue_size_))
            {
              pending_rendered_feedback_source_motion_factors_.pop_front();
              ++rendered_feedback_source_motion_stale_drops_;
            }
            pending_rendered_feedback_source_motion_factors_.push_back(std::move(pending));
            ++rendered_feedback_source_motion_queued_factors_;
          } else {
            ++rendered_feedback_source_motion_invalid_;
          }
        }
      }
    }
    last_rendered_feedback_source_motion_pose_ = std::move(current);
  }

  void handle_rendered_image_with_metadata(
    const sensor_msgs::msg::Image & msg,
    const std::optional<RenderedFeedbackMetadata> & metadata)
  {
    if (!enable_visual_factor_) {
      return;
    }
    const int64_t stamp_ns = gaussian_lic_tracking::stamp_to_nanoseconds(msg.header.stamp);
    if (metadata.has_value()) {
      observe_rendered_feedback_stamp(stamp_ns);
    } else {
      if (!accept_stream_stamp(
          "rendered_image", stamp_ns, last_rendered_input_stamp_ns_, rendered_stamp_regressions_, true))
      {
        return;
      }
    }
    ++num_rendered_images_;
    gaussian_lic_tracking::VisualFrame rendered;
    if (decode_image_gray(msg, rendered)) {
      if (metadata.has_value()) {
        rendered.has_rendered_feedback_metadata = true;
        rendered.rendered_feedback_observed_stamp_ns = metadata->observed_stamp_ns;
        rendered.rendered_feedback_pose_stamp_ns = metadata->pose_stamp_ns;
        rendered.rendered_feedback_pointcloud_stamp_ns = metadata->pointcloud_stamp_ns;
        rendered.rendered_feedback_frame_index = metadata->frame_index;
        rendered.rendered_feedback_preview_index = metadata->rendered_preview_index;
        rendered.has_rendered_feedback_source_pose = metadata->has_source_pose;
        rendered.rendered_feedback_source_p_w_i = metadata->source_p_w_i;
        rendered.rendered_feedback_source_q_w_i = metadata->source_q_w_i;
      }
      cache_rendered_frame(rendered);
      int64_t observed_match_delta_ns = 0;
      int64_t observed_nearest_delta_ns = 0;
      int64_t observed_nearest_signed_delta_ns = 0;
      bool observed_cache_had_size_match = false;
      const gaussian_lic_tracking::VisualFrame * observed_frame =
        select_observed_frame_for_stamp(
        rendered.stamp_ns,
        rendered.width,
        rendered.height,
        &observed_match_delta_ns,
        &observed_cache_had_size_match,
        visual_pair_requires_unique_observed_stamp(),
        &observed_nearest_delta_ns,
        &observed_nearest_signed_delta_ns);
      last_visual_observed_cache_size_ = observed_frame_cache_.size();
      last_visual_observed_match_delta_ns_ = observed_frame == nullptr ? 0 : observed_match_delta_ns;
      last_visual_observed_nearest_delta_ns_ = observed_nearest_delta_ns;
      last_visual_observed_nearest_signed_delta_ns_ = observed_nearest_signed_delta_ns;
      if (observed_frame != nullptr && !visual_pair_processing_defer_to_pointcloud_ &&
        !enable_visual_watermark_pair_scheduler_)
      {
        last_visual_rendered_cache_size_ = rendered_frame_cache_.size();
        last_visual_rendered_match_delta_ns_ = observed_match_delta_ns;
        process_visual_pair(rendered, *observed_frame, false);
      } else if (observed_frame_cache_.empty()) {
        ++visual_observed_miss_count_;
      } else if (!observed_cache_had_size_match) {
        ++visual_observed_size_mismatch_count_;
      } else {
        ++visual_observed_stale_count_;
      }
      if (!visual_cache_reconciliation_defer_to_pointcloud_ &&
        !visual_pair_processing_defer_to_pointcloud_ &&
        !enable_visual_watermark_pair_scheduler_)
      {
        reconcile_visual_frame_caches();
      }
      if (enable_visual_watermark_pair_scheduler_ && last_pointcloud_stamp_ns_ > 0) {
        process_visual_pairs_up_to_watermark(last_pointcloud_stamp_ns_, true);
      }
    } else {
      ++rendered_invalid_frames_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "dropping rendered image with unsupported encoding, layout, or dimensions");
    }
  }

  void process_visual_pair(
    const gaussian_lic_tracking::VisualFrame & rendered,
    const gaussian_lic_tracking::VisualFrame & observed,
    const bool reconciled_pair,
    const bool force_callback_ingest = false)
  {
    if (visual_pair_was_processed(
        observed,
        rendered,
        visual_pair_requires_unique_observed_stamp(),
        visual_pair_requires_unique_rendered_stamp()))
    {
      ++visual_pair_duplicate_count_;
      return;
    }
    remember_visual_pair(observed, rendered);
    last_processed_visual_observed_stamp_ns_ = observed.stamp_ns;
    last_processed_visual_rendered_stamp_ns_ = rendered.stamp_ns;
    ++visual_pair_processed_count_;
    const int64_t pair_stamp_delta_ns = stamp_delta_ns(rendered.stamp_ns, observed.stamp_ns);
    const int64_t factor_reference_stamp_ns =
      visual_factor_reference_stamp_ns(observed, rendered);
    const uint64_t factor_source_id = visual_factor_source_id(observed, rendered);

    last_visual_residual_ = visual_factor_.evaluate(rendered, observed);
    last_visual_alignment_ = visual_factor_.estimate_translation(
      rendered,
      observed,
      visual_alignment_max_shift_px_,
      visual_alignment_metric_);
    last_visual_alignment_saturated_ =
      visual_alignment_is_saturated(last_visual_alignment_);
    last_visual_alignment_effective_weight_ =
      visual_alignment_effective_weight(last_visual_alignment_);
    if (last_visual_alignment_saturated_) {
      ++visual_alignment_saturated_count_;
      if (reconciled_pair) {
        ++visual_cache_reconciled_saturated_pairs_;
      }
    }
    last_visual_photometric_linearization_ =
      visual_factor_.linearize_translation(rendered, observed);
    if (enable_se3_photometric_window_factor_) {
      const auto se3_samples = build_se3_photometric_samples(rendered, observed);
      ++visual_se3_photometric_total_batches_;
      visual_se3_photometric_total_candidate_pixels_ +=
        static_cast<uint64_t>(se3_samples.candidate_pixels);
      visual_se3_photometric_total_accepted_pixels_ +=
        static_cast<uint64_t>(se3_samples.accepted_pixels);
      last_visual_se3_photometric_candidate_pixels_ = se3_samples.candidate_pixels;
      last_visual_se3_photometric_sampled_depth_pixels_ = se3_samples.sampled_depth_pixels;
      last_visual_se3_photometric_accepted_pixels_ = se3_samples.accepted_pixels;
      last_visual_se3_photometric_rejected_depth_pixels_ = se3_samples.rejected_depth_pixels;
      last_visual_se3_photometric_rejected_gradient_pixels_ = se3_samples.rejected_gradient_pixels;
      last_visual_se3_photometric_rejected_residual_pixels_ = se3_samples.rejected_residual_pixels;
      last_visual_se3_photometric_coverage_tiles_ = se3_samples.coverage_tiles;
      last_visual_se3_photometric_coverage_total_tiles_ = se3_samples.coverage_total_tiles;
      last_visual_se3_photometric_mean_abs_residual_ = se3_samples.mean_abs_residual;
      last_visual_se3_photometric_linearization_ =
        gaussian_lic_tracking::linearize_se3_photometric_samples(camera_intrinsics_, se3_samples.samples);
      const bool has_enough_samples = last_visual_se3_photometric_linearization_.valid &&
        last_visual_se3_photometric_linearization_.sample_count >=
        static_cast<size_t>(se3_photometric_min_samples_);
      const bool hessian_is_healthy =
        se3_photometric_hessian_is_healthy(last_visual_se3_photometric_linearization_);
      const bool sample_quality_is_healthy = se3_photometric_sample_quality_is_healthy(se3_samples);
      if (has_enough_samples && hessian_is_healthy && sample_quality_is_healthy)
      {
        ++visual_se3_photometric_valid_batches_;
        last_accepted_visual_se3_photometric_sampled_depth_pixels_ =
          se3_samples.sampled_depth_pixels;
        last_accepted_visual_se3_photometric_accepted_pixels_ =
          se3_samples.accepted_pixels;
        last_accepted_visual_se3_photometric_sample_inlier_ratio_ =
          se3_photometric_sample_inlier_ratio(se3_samples);
        last_accepted_visual_se3_photometric_coverage_tiles_ =
          se3_samples.coverage_tiles;
        last_accepted_visual_se3_photometric_coverage_total_tiles_ =
          se3_samples.coverage_total_tiles;
        last_accepted_visual_se3_photometric_mean_abs_residual_ =
          se3_samples.mean_abs_residual;
        last_accepted_visual_se3_photometric_step_norm_ =
          last_visual_se3_photometric_linearization_.gauss_newton_step.norm();
        last_accepted_visual_se3_photometric_hessian_rank_ =
          last_visual_se3_photometric_linearization_.hessian_rank;
        last_accepted_visual_se3_photometric_hessian_min_singular_value_ =
          last_visual_se3_photometric_linearization_.hessian_min_singular_value;
        last_accepted_visual_se3_photometric_hessian_max_singular_value_ =
          last_visual_se3_photometric_linearization_.hessian_max_singular_value;
        last_accepted_visual_se3_photometric_hessian_condition_number_ =
          last_visual_se3_photometric_linearization_.hessian_condition_number;
        PendingSe3PhotometricFactor pending;
        pending.stamp_ns = factor_reference_stamp_ns;
        pending.pair_stamp_delta_ns = pair_stamp_delta_ns;
        pending.source_id = factor_source_id;
        pending.visual_alignment_saturated = last_visual_alignment_saturated_;
        pending.mean_abs_residual = se3_samples.mean_abs_residual;
        pending.sample_inlier_ratio = se3_photometric_sample_inlier_ratio(se3_samples);
        pending.step_norm = last_visual_se3_photometric_linearization_.gauss_newton_step.norm();
        pending.hessian_condition_number =
          last_visual_se3_photometric_linearization_.hessian_condition_number;
        pending.coverage_tiles = se3_samples.coverage_tiles;
        pending.coverage_total_tiles = se3_samples.coverage_total_tiles;
        pending.linearization = last_visual_se3_photometric_linearization_;
        attach_visual_reference(pending, rendered);
        pending_visual_se3_photometric_factors_.push_back(std::move(pending));
        trim_pending_visual_factor_queues();
      } else if (se3_samples.accepted_pixels > 0U) {
        if (has_enough_samples && hessian_is_healthy) {
          ++visual_se3_photometric_quality_rejected_batches_;
        } else if (has_enough_samples) {
          ++visual_se3_photometric_degenerate_batches_;
        } else {
          ++visual_se3_photometric_insufficient_sample_batches_;
        }
      }
    }
    auto window_alignment = visual_alignment_for_window_factor();
    if (reconciled_pair && window_alignment.has_value() &&
      visual_alignment_is_saturated(window_alignment.value()))
    {
      const auto fallback = visual_alignment_with_photometric_step(window_alignment.value());
      if (fallback.has_value() &&
        visual_alignment_photometric_step_agrees(window_alignment.value(), fallback.value()))
      {
        window_alignment = fallback;
        ++visual_cache_reconciled_alignment_photometric_fallback_pairs_;
      } else if (fallback.has_value()) {
        ++visual_cache_reconciled_alignment_photometric_disagreement_pairs_;
      }
    }
    if (window_alignment.has_value()) {
      const Eigen::Vector2d component_weight_xy =
        visual_alignment_component_weights(window_alignment.value());
      if (!visual_alignment_has_active_component(component_weight_xy)) {
        ++visual_alignment_saturation_axis_mask_skipped_factors_;
        if (reconciled_pair) {
          ++visual_cache_reconciled_alignment_skipped_pairs_;
        }
      } else if (reconciled_pair && visual_alignment_is_saturated(window_alignment.value()) &&
        !enable_visual_alignment_saturation_axis_mask_)
      {
        ++visual_cache_reconciled_alignment_skipped_pairs_;
      } else {
        PendingVisualAlignmentFactor pending;
        pending.stamp_ns = factor_reference_stamp_ns;
        pending.pair_stamp_delta_ns = pair_stamp_delta_ns;
        pending.source_id = factor_source_id;
        pending.alignment = window_alignment.value();
        pending.component_weight_xy = component_weight_xy;
        record_visual_alignment_saturation_axis_mask(component_weight_xy);
        attach_visual_reference(pending, rendered);
        pending_visual_alignment_factors_.push_back(std::move(pending));
        trim_pending_visual_factor_queues();
      }
    }
    if (
      (enable_visual_callback_factor_ingest_ || force_callback_ingest) &&
      last_output_tracking_pose_.has_value())
    {
      ingest_pending_visual_factors_into_optimizer(last_output_tracking_pose_.value());
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

  static double finite_positive_or(const double value, const double fallback)
  {
    return std::isfinite(value) && value > 0.0 ? value : fallback;
  }

  double visual_alignment_candidate_score(const PendingVisualAlignmentFactor & pending) const
  {
    double score = finite_positive_or(pending.alignment.rmse, 1.0e6);
    const double pair_dt_s =
      static_cast<double>(std::max<int64_t>(pending.pair_stamp_delta_ns, 0LL)) /
      static_cast<double>(gaussian_lic_tracking::kNanosecondsPerSecond);
    score += 0.05 * pair_dt_s;
    score += 0.01 * std::hypot(pending.alignment.subpixel_dx, pending.alignment.subpixel_dy);
    if (visual_alignment_is_saturated(pending.alignment)) {
      score += 1.0;
    }
    return score;
  }

  static double se3_candidate_coverage_ratio(const PendingSe3PhotometricFactor & pending)
  {
    return pending.coverage_total_tiles > 0U
      ? static_cast<double>(pending.coverage_tiles) / static_cast<double>(pending.coverage_total_tiles)
      : 0.0;
  }

  double se3_photometric_candidate_score(const PendingSe3PhotometricFactor & pending) const
  {
    double score = finite_positive_or(pending.mean_abs_residual, 1.0e6);
    const double pair_dt_s =
      static_cast<double>(std::max<int64_t>(pending.pair_stamp_delta_ns, 0LL)) /
      static_cast<double>(gaussian_lic_tracking::kNanosecondsPerSecond);
    score += 0.05 * pair_dt_s;
    score += 0.01 * finite_positive_or(pending.step_norm, 0.0);
    if (std::isfinite(pending.hessian_condition_number) &&
      pending.hessian_condition_number > 1.0)
    {
      score += 0.001 * std::log10(pending.hessian_condition_number);
    } else {
      score += 1.0;
    }
    score += 0.1 * (1.0 - std::clamp(pending.sample_inlier_ratio, 0.0, 1.0));
    score += 0.1 * (1.0 - std::clamp(se3_candidate_coverage_ratio(pending), 0.0, 1.0));
    return score;
  }

  double visual_factor_quality_weight_floor() const
  {
    return std::max(visual_factor_quality_min_weight_scale_, 1.0e-9);
  }

  double bounded_visual_factor_quality_weight(const double weight) const
  {
    if (!std::isfinite(weight) || weight <= 0.0) {
      return visual_factor_quality_weight_floor();
    }
    return std::clamp(weight, visual_factor_quality_weight_floor(), 1.0);
  }

  double visual_pair_dt_quality_weight(const int64_t pair_stamp_delta_ns) const
  {
    if (!enable_visual_factor_quality_weighting_ || visual_factor_max_dt_ns_ <= 0LL) {
      return 1.0;
    }
    const double pair_dt_ns = static_cast<double>(std::max<int64_t>(pair_stamp_delta_ns, 0LL));
    const double max_dt_ns = static_cast<double>(visual_factor_max_dt_ns_);
    return bounded_visual_factor_quality_weight(1.0 - (pair_dt_ns / max_dt_ns));
  }

  std::optional<VisualFactorReference> snapshot_visual_factor_reference(
    const int64_t factor_stamp_ns) const
  {
    if (!enable_visual_factor_reference_snapshot_) {
      return std::nullopt;
    }
    if (!enable_sliding_window_optimizer_ || !last_output_tracking_pose_.has_value()) {
      return std::nullopt;
    }
    return select_visual_factor_reference(factor_stamp_ns, last_output_tracking_pose_.value());
  }

  void attach_visual_reference_snapshot(PendingVisualAlignmentFactor & pending) const
  {
    const auto reference = snapshot_visual_factor_reference(pending.stamp_ns);
    if (!reference.has_value()) {
      return;
    }
    pending.has_reference_pose = true;
    pending.reference_p_w_i = reference->pose.p_w_i;
    pending.reference_support_stamp_ns = reference->support_stamp_ns;
    pending.reference_support_weights = reference->support_weights;
  }

  void attach_visual_reference_snapshot(PendingSe3PhotometricFactor & pending) const
  {
    const auto reference = snapshot_visual_factor_reference(pending.stamp_ns);
    if (!reference.has_value()) {
      return;
    }
    pending.has_reference_pose = true;
    pending.reference_p_w_i = reference->pose.p_w_i;
    pending.reference_q_w_i = reference->pose.q_w_i.normalized();
    pending.reference_support_stamp_ns = reference->support_stamp_ns;
    pending.reference_support_weights = reference->support_weights;
  }

  bool attach_rendered_feedback_source_pose_reference(
    PendingVisualAlignmentFactor & pending,
    const gaussian_lic_tracking::VisualFrame & rendered)
  {
    if (!enable_rendered_feedback_source_pose_reference_ ||
      !rendered.has_rendered_feedback_source_pose ||
      !rendered.rendered_feedback_source_p_w_i.allFinite())
    {
      return false;
    }
    pending.has_reference_pose = true;
    pending.reference_p_w_i = rendered.rendered_feedback_source_p_w_i;
    pending.reference_support_stamp_ns.clear();
    pending.reference_support_weights.clear();
    ++rendered_feedback_source_pose_reference_factors_;
    return true;
  }

  bool attach_rendered_feedback_source_pose_reference(
    PendingSe3PhotometricFactor & pending,
    const gaussian_lic_tracking::VisualFrame & rendered)
  {
    if (!enable_rendered_feedback_source_pose_reference_ ||
      !rendered.has_rendered_feedback_source_pose ||
      !rendered.rendered_feedback_source_p_w_i.allFinite() ||
      !rendered.rendered_feedback_source_q_w_i.coeffs().allFinite() ||
      rendered.rendered_feedback_source_q_w_i.norm() <= std::numeric_limits<double>::epsilon())
    {
      return false;
    }
    pending.has_reference_pose = true;
    pending.reference_p_w_i = rendered.rendered_feedback_source_p_w_i;
    pending.reference_q_w_i = rendered.rendered_feedback_source_q_w_i.normalized();
    pending.reference_support_stamp_ns.clear();
    pending.reference_support_weights.clear();
    ++rendered_feedback_source_pose_reference_factors_;
    return true;
  }

  void attach_visual_reference(
    PendingVisualAlignmentFactor & pending,
    const gaussian_lic_tracking::VisualFrame & rendered)
  {
    if (attach_rendered_feedback_source_pose_reference(pending, rendered)) {
      return;
    }
    attach_visual_reference_snapshot(pending);
  }

  void attach_visual_reference(
    PendingSe3PhotometricFactor & pending,
    const gaussian_lic_tracking::VisualFrame & rendered)
  {
    if (attach_rendered_feedback_source_pose_reference(pending, rendered)) {
      return;
    }
    attach_visual_reference_snapshot(pending);
  }

  double visual_alignment_quality_weight_scale(const PendingVisualAlignmentFactor & pending) const
  {
    if (!enable_visual_factor_quality_weighting_) {
      return 1.0;
    }
    double weight = visual_pair_dt_quality_weight(pending.pair_stamp_delta_ns);
    if (visual_alignment_is_saturated(pending.alignment)) {
      weight = std::min(weight, visual_factor_quality_weight_floor());
    }
    return bounded_visual_factor_quality_weight(weight);
  }

  double se3_photometric_quality_weight_scale(const PendingSe3PhotometricFactor & pending) const
  {
    if (!enable_visual_factor_quality_weighting_) {
      return 1.0;
    }
    double weight = visual_pair_dt_quality_weight(pending.pair_stamp_delta_ns);
    weight *= std::clamp(pending.sample_inlier_ratio, visual_factor_quality_weight_floor(), 1.0);
    if (pending.coverage_total_tiles > 0U) {
      weight *= std::clamp(
        se3_candidate_coverage_ratio(pending), visual_factor_quality_weight_floor(), 1.0);
    }
    return bounded_visual_factor_quality_weight(weight);
  }

  bool visual_expired_factor_projection_age_allowed(
    const int64_t factor_stamp_ns,
    const int64_t reference_stamp_ns) const
  {
    if (!enable_visual_expired_factor_projection_) {
      return false;
    }
    if (factor_stamp_ns > reference_stamp_ns) {
      return false;
    }
    if (visual_expired_factor_projection_max_age_s_ <= 0.0) {
      return true;
    }
    const double age_s =
      static_cast<double>(reference_stamp_ns - factor_stamp_ns) /
      static_cast<double>(gaussian_lic_tracking::kNanosecondsPerSecond);
    return age_s <= visual_expired_factor_projection_max_age_s_;
  }

  std::optional<gaussian_lic_tracking::SlidingWindowVisualAlignmentFactor>
  make_marginalized_visual_factor_from_pending(
    const PendingVisualAlignmentFactor & pending) const
  {
    gaussian_lic_tracking::SlidingWindowVisualAlignmentFactor factor;
    factor.stamp_ns = pending.stamp_ns;
    factor.source_id = pending.source_id;
    factor.has_reference_pose = pending.has_reference_pose;
    factor.support_stamp_ns = pending.reference_support_stamp_ns;
    factor.support_weights = pending.reference_support_weights;
    if (pending.has_reference_pose) {
      factor.reference_p_w_i = pending.reference_p_w_i;
    }
    factor.measured_shift_px = Eigen::Vector2d{
      pending.alignment.subpixel_dx,
      pending.alignment.subpixel_dy};
    factor.component_weight_xy = pending.component_weight_xy;
    factor.meters_per_pixel = visual_alignment_meters_per_pixel_;
    factor.weight =
      visual_alignment_effective_weight(pending.alignment) *
      visual_alignment_quality_weight_scale(pending);
    factor.huber_delta_m = visual_alignment_huber_delta_m_;
    return factor;
  }

  std::optional<gaussian_lic_tracking::SlidingWindowSe3PhotometricFactor>
  make_marginalized_se3_factor_from_pending(
    const PendingSe3PhotometricFactor & pending) const
  {
    gaussian_lic_tracking::SlidingWindowSe3PhotometricFactor factor;
    factor.stamp_ns = pending.stamp_ns;
    factor.source_id = pending.source_id;
    factor.has_reference_pose = pending.has_reference_pose;
    factor.support_stamp_ns = pending.reference_support_stamp_ns;
    factor.support_weights = pending.reference_support_weights;
    if (pending.has_reference_pose) {
      factor.reference_p_w_i = pending.reference_p_w_i;
      factor.reference_q_w_i = pending.reference_q_w_i.normalized();
    }
    factor.target_delta = gaussian_lic_tracking::transform_camera_delta_to_body(
      q_i_c_,
      p_i_c_,
      pending.linearization.gauss_newton_step);
    const Eigen::Matrix<double, 6, 6> body_hessian =
      gaussian_lic_tracking::transform_camera_information_to_body(
      q_i_c_,
      p_i_c_,
      pending.linearization.hessian);
    factor.sqrt_information =
      gaussian_lic_tracking::sqrt_information_from_hessian(body_hessian);
    factor.weight =
      se3_photometric_window_weight_ * se3_photometric_quality_weight_scale(pending);
    factor.huber_delta = se3_photometric_factor_huber_delta_;
    if (factor.sqrt_information.norm() <= std::numeric_limits<double>::epsilon()) {
      return std::nullopt;
    }
    return factor;
  }

  bool add_marginalized_visual_prior_from_pending(
    const PendingVisualAlignmentFactor & pending,
    const bool callback_ingest,
    const bool count_skip_on_failure = true)
  {
    if (!enable_visual_marginalization_prior_) {
      return false;
    }
    if (visual_marginalization_prior_saturation_gate_rejects(pending)) {
      record_visual_marginalization_prior_saturation_rejection();
      return false;
    }
    const auto factor = make_marginalized_visual_factor_from_pending(pending);
    if (!factor.has_value()) {
      if (count_skip_on_failure) {
        ++visual_marginalization_prior_skipped_factors_;
      }
      return false;
    }
    try {
      if (sliding_window_optimizer_.add_marginalized_visual_alignment_prior(factor.value())) {
        ++sliding_window_total_visual_factors_;
        if (callback_ingest) {
          ++visual_callback_ingested_visual_factors_;
        }
        ++visual_alignment_marginalization_priors_;
        return true;
      }
    } catch (const std::exception & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "marginalized visual alignment prior skipped: %s", ex.what());
    }
    if (count_skip_on_failure) {
      ++visual_marginalization_prior_skipped_factors_;
    }
    return false;
  }

  bool add_marginalized_se3_prior_from_pending(
    const PendingSe3PhotometricFactor & pending,
    const bool callback_ingest,
    const bool count_skip_on_failure = true)
  {
    if (!enable_visual_marginalization_prior_) {
      return false;
    }
    if (visual_marginalization_prior_saturation_gate_rejects(pending)) {
      record_se3_marginalization_prior_saturation_rejection();
      return false;
    }
    const auto factor = make_marginalized_se3_factor_from_pending(pending);
    if (!factor.has_value()) {
      if (count_skip_on_failure) {
        ++visual_marginalization_prior_skipped_factors_;
      }
      return false;
    }
    try {
      if (sliding_window_optimizer_.add_marginalized_se3_photometric_prior(factor.value())) {
        ++sliding_window_total_se3_photometric_factors_;
        if (callback_ingest) {
          ++visual_callback_ingested_se3_photometric_factors_;
        }
        ++visual_se3_photometric_marginalization_priors_;
        return true;
      }
    } catch (const std::exception & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "marginalized SE3 photometric prior skipped: %s", ex.what());
    }
    if (count_skip_on_failure) {
      ++visual_marginalization_prior_skipped_factors_;
    }
    return false;
  }

  bool visual_pending_factor_needs_marginalized_prior(
    const int64_t stamp_ns,
    const gaussian_lic_tracking::TrajectoryPose & tracking_pose) const
  {
    if (!enable_visual_marginalization_prior_) {
      return false;
    }
    if (visual_factor_should_defer_future_reference(stamp_ns, tracking_pose)) {
      return false;
    }
    if (visual_factor_stamp_is_before_active_window(stamp_ns)) {
      return true;
    }
    if (select_visual_factor_reference(stamp_ns, tracking_pose).has_value()) {
      return false;
    }
    return visual_factor_stamp_is_expired(stamp_ns, tracking_pose.stamp_ns);
  }

  bool visual_marginalization_prior_saturation_gate_rejects(
    const PendingVisualAlignmentFactor & pending) const
  {
    return enable_visual_marginalization_prior_saturation_gate_ &&
      visual_marginalization_prior_saturation_gate_visual_factors_ &&
      visual_alignment_is_saturated(pending.alignment);
  }

  bool visual_marginalization_prior_saturation_gate_rejects(
    const PendingSe3PhotometricFactor & pending) const
  {
    return enable_visual_marginalization_prior_saturation_gate_ &&
      visual_marginalization_prior_saturation_gate_se3_factors_ &&
      pending.visual_alignment_saturated;
  }

  void record_visual_marginalization_prior_saturation_rejection()
  {
    ++visual_marginalization_prior_saturation_rejected_factors_;
    ++visual_marginalization_prior_saturation_rejected_visual_factors_;
  }

  void record_se3_marginalization_prior_saturation_rejection()
  {
    ++visual_marginalization_prior_saturation_rejected_factors_;
    ++visual_marginalization_prior_saturation_rejected_se3_factors_;
  }

  bool add_batched_marginalized_visual_priors_from_pending(
    const gaussian_lic_tracking::TrajectoryPose & tracking_pose,
    const bool callback_ingest)
  {
    if (!enable_visual_marginalization_prior_ ||
      !enable_visual_marginalization_prior_batching_ ||
      !enable_sliding_window_optimizer_)
    {
      return false;
    }
    std::vector<gaussian_lic_tracking::SlidingWindowVisualAlignmentFactor> visual_factors;
    std::vector<gaussian_lic_tracking::SlidingWindowSe3PhotometricFactor> se3_factors;
    visual_factors.reserve(pending_visual_alignment_factors_.size());
    se3_factors.reserve(pending_visual_se3_photometric_factors_.size());
    bool rejected_by_saturation_gate = false;
    for (const auto & pending : pending_visual_alignment_factors_) {
      if (!visual_pending_factor_needs_marginalized_prior(pending.stamp_ns, tracking_pose)) {
        continue;
      }
      if (visual_marginalization_prior_saturation_gate_rejects(pending)) {
        record_visual_marginalization_prior_saturation_rejection();
        rejected_by_saturation_gate = true;
        continue;
      }
      const auto factor = make_marginalized_visual_factor_from_pending(pending);
      if (!factor.has_value()) {
        return false;
      }
      visual_factors.push_back(factor.value());
    }
    for (const auto & pending : pending_visual_se3_photometric_factors_) {
      if (!visual_pending_factor_needs_marginalized_prior(pending.stamp_ns, tracking_pose)) {
        continue;
      }
      if (visual_marginalization_prior_saturation_gate_rejects(pending)) {
        record_se3_marginalization_prior_saturation_rejection();
        rejected_by_saturation_gate = true;
        continue;
      }
      const auto factor = make_marginalized_se3_factor_from_pending(pending);
      if (!factor.has_value()) {
        return false;
      }
      se3_factors.push_back(factor.value());
    }
    if (visual_factors.empty() && se3_factors.empty()) {
      if (rejected_by_saturation_gate) {
        pending_visual_alignment_factors_.erase(
          std::remove_if(
            pending_visual_alignment_factors_.begin(),
            pending_visual_alignment_factors_.end(),
            [this, &tracking_pose](const PendingVisualAlignmentFactor & pending) {
              return visual_pending_factor_needs_marginalized_prior(pending.stamp_ns, tracking_pose) &&
                visual_marginalization_prior_saturation_gate_rejects(pending);
            }),
          pending_visual_alignment_factors_.end());
        pending_visual_se3_photometric_factors_.erase(
          std::remove_if(
            pending_visual_se3_photometric_factors_.begin(),
            pending_visual_se3_photometric_factors_.end(),
            [this, &tracking_pose](const PendingSe3PhotometricFactor & pending) {
              return visual_pending_factor_needs_marginalized_prior(pending.stamp_ns, tracking_pose) &&
                visual_marginalization_prior_saturation_gate_rejects(pending);
            }),
          pending_visual_se3_photometric_factors_.end());
      }
      return false;
    }
    const auto result =
      sliding_window_optimizer_.add_batched_marginalized_visual_priors(visual_factors, se3_factors);
    if (!result.added || result.skipped_factor_count != 0U ||
      result.visual_alignment_prior_count != visual_factors.size() ||
      result.se3_photometric_prior_count != se3_factors.size())
    {
      ++visual_batched_marginalization_prior_skipped_batches_;
      visual_batched_marginalization_prior_skipped_factors_ +=
        std::max<size_t>(
        result.skipped_factor_count,
        visual_factors.size() + se3_factors.size() -
        result.visual_alignment_prior_count - result.se3_photometric_prior_count);
      return false;
    }
    ++visual_batched_marginalization_prior_batches_;
    visual_batched_marginalization_prior_visual_factors_ +=
      result.visual_alignment_prior_count;
    visual_batched_marginalization_prior_se3_factors_ +=
      result.se3_photometric_prior_count;
    sliding_window_total_visual_factors_ += result.visual_alignment_prior_count;
    sliding_window_total_se3_photometric_factors_ += result.se3_photometric_prior_count;
    visual_alignment_marginalization_priors_ += result.visual_alignment_prior_count;
    visual_se3_photometric_marginalization_priors_ +=
      result.se3_photometric_prior_count;
    if (callback_ingest) {
      visual_callback_ingested_visual_factors_ += result.visual_alignment_prior_count;
      visual_callback_ingested_se3_photometric_factors_ += result.se3_photometric_prior_count;
    }
    auto erase_batched_visual = [this, &tracking_pose](const PendingVisualAlignmentFactor & pending) {
        return visual_pending_factor_needs_marginalized_prior(pending.stamp_ns, tracking_pose);
      };
    auto erase_batched_se3 = [this, &tracking_pose](const PendingSe3PhotometricFactor & pending) {
        return visual_pending_factor_needs_marginalized_prior(pending.stamp_ns, tracking_pose);
      };
    pending_visual_alignment_factors_.erase(
      std::remove_if(
        pending_visual_alignment_factors_.begin(),
        pending_visual_alignment_factors_.end(),
        erase_batched_visual),
      pending_visual_alignment_factors_.end());
    pending_visual_se3_photometric_factors_.erase(
      std::remove_if(
        pending_visual_se3_photometric_factors_.begin(),
        pending_visual_se3_photometric_factors_.end(),
        erase_batched_se3),
      pending_visual_se3_photometric_factors_.end());
    return true;
  }

  gaussian_lic_tracking::TrajectoryPose expired_visual_projection_reference_pose(
    const gaussian_lic_tracking::TrajectoryPose & current_pose) const
  {
    const auto & states = sliding_window_optimizer_.states();
    if (states.empty()) {
      return current_pose;
    }

    // Late rendered-feedback factors belong to source time. Once the source state
    // has left the active window, keep the diagnostic projection at the oldest
    // retained state instead of anchoring it to the newest tracking pose.
    const auto & oldest = states.front();
    gaussian_lic_tracking::TrajectoryPose reference_pose;
    reference_pose.stamp_ns = oldest.stamp_ns;
    reference_pose.p_w_i = oldest.p_w_i;
    reference_pose.q_w_i = oldest.q_w_i.normalized();
    reference_pose.v_w_i = oldest.v_w_i;
    return reference_pose;
  }

  std::optional<gaussian_lic_tracking::SlidingWindowVisualAlignmentFactor>
  make_projected_expired_visual_factor(
    const PendingVisualAlignmentFactor & pending,
    const gaussian_lic_tracking::TrajectoryPose & reference_pose) const
  {
    if (!visual_expired_factor_projection_age_allowed(pending.stamp_ns, reference_pose.stamp_ns) ||
      !reference_pose.p_w_i.allFinite())
    {
      return std::nullopt;
    }
    gaussian_lic_tracking::SlidingWindowVisualAlignmentFactor factor;
    factor.stamp_ns = reference_pose.stamp_ns;
    factor.source_id = pending.source_id;
    factor.support_stamp_ns = {reference_pose.stamp_ns};
    factor.support_weights = {1.0};
    factor.reference_p_w_i =
      pending.has_reference_pose ? pending.reference_p_w_i : reference_pose.p_w_i;
    factor.measured_shift_px = Eigen::Vector2d{
      pending.alignment.subpixel_dx,
      pending.alignment.subpixel_dy};
    factor.component_weight_xy = pending.component_weight_xy;
    factor.meters_per_pixel = visual_alignment_meters_per_pixel_;
    factor.weight =
      visual_alignment_effective_weight(pending.alignment) *
      visual_alignment_quality_weight_scale(pending);
    factor.huber_delta_m = visual_alignment_huber_delta_m_;
    return factor;
  }

  std::optional<gaussian_lic_tracking::SlidingWindowSe3PhotometricFactor>
  make_projected_expired_se3_factor(
    const PendingSe3PhotometricFactor & pending,
    const gaussian_lic_tracking::TrajectoryPose & reference_pose) const
  {
    if (!visual_expired_factor_projection_age_allowed(pending.stamp_ns, reference_pose.stamp_ns) ||
      !reference_pose.p_w_i.allFinite() ||
      !reference_pose.q_w_i.coeffs().allFinite() ||
      reference_pose.q_w_i.norm() <= std::numeric_limits<double>::epsilon())
    {
      return std::nullopt;
    }
    gaussian_lic_tracking::SlidingWindowSe3PhotometricFactor factor;
    factor.stamp_ns = reference_pose.stamp_ns;
    factor.source_id = pending.source_id;
    factor.support_stamp_ns = {reference_pose.stamp_ns};
    factor.support_weights = {1.0};
    factor.reference_p_w_i =
      pending.has_reference_pose ? pending.reference_p_w_i : reference_pose.p_w_i;
    factor.reference_q_w_i =
      pending.has_reference_pose ? pending.reference_q_w_i.normalized() :
      reference_pose.q_w_i.normalized();
    factor.target_delta = gaussian_lic_tracking::transform_camera_delta_to_body(
      q_i_c_,
      p_i_c_,
      pending.linearization.gauss_newton_step);
    const Eigen::Matrix<double, 6, 6> body_hessian =
      gaussian_lic_tracking::transform_camera_information_to_body(
      q_i_c_,
      p_i_c_,
      pending.linearization.hessian);
    factor.sqrt_information =
      gaussian_lic_tracking::sqrt_information_from_hessian(body_hessian);
    factor.weight =
      se3_photometric_window_weight_ * se3_photometric_quality_weight_scale(pending);
    factor.huber_delta = se3_photometric_factor_huber_delta_;
    if (factor.sqrt_information.norm() <= std::numeric_limits<double>::epsilon()) {
      return std::nullopt;
    }
    return factor;
  }

  bool visual_factor_quality_selection_is_active(const int64_t stamp_ns) const
  {
    if (!enable_visual_factor_quality_selection_) {
      return false;
    }
    if (visual_factor_quality_selection_start_after_s_ <= 0.0) {
      return true;
    }
    if (!sliding_window_start_stamp_ns_.has_value()) {
      return false;
    }
    const int64_t start_after_ns = static_cast<int64_t>(
      visual_factor_quality_selection_start_after_s_ *
      static_cast<double>(gaussian_lic_tracking::kNanosecondsPerSecond));
    return stamp_ns >= sliding_window_start_stamp_ns_.value() + start_after_ns;
  }

  void trim_pending_visual_factor_queues()
  {
    const auto max_queue_size = static_cast<size_t>(visual_pending_factor_queue_size_);
    while (pending_visual_alignment_factors_.size() > max_queue_size) {
      if (visual_factor_quality_selection_is_active(pending_visual_alignment_factors_.back().stamp_ns)) {
        const auto worst = std::max_element(
          pending_visual_alignment_factors_.begin(),
          pending_visual_alignment_factors_.end(),
          [this](
            const PendingVisualAlignmentFactor & lhs,
            const PendingVisualAlignmentFactor & rhs) {
            return visual_alignment_candidate_score(lhs) < visual_alignment_candidate_score(rhs);
          });
        pending_visual_alignment_factors_.erase(worst);
      } else {
        pending_visual_alignment_factors_.pop_front();
      }
      ++visual_alignment_pending_stale_drops_;
      ++visual_alignment_pending_queue_trim_drops_;
    }
    while (pending_visual_se3_photometric_factors_.size() > max_queue_size) {
      if (visual_factor_quality_selection_is_active(
          pending_visual_se3_photometric_factors_.back().stamp_ns))
      {
        const auto worst = std::max_element(
          pending_visual_se3_photometric_factors_.begin(),
          pending_visual_se3_photometric_factors_.end(),
          [this](
            const PendingSe3PhotometricFactor & lhs,
            const PendingSe3PhotometricFactor & rhs) {
            return se3_photometric_candidate_score(lhs) < se3_photometric_candidate_score(rhs);
          });
        pending_visual_se3_photometric_factors_.erase(worst);
      } else {
        pending_visual_se3_photometric_factors_.pop_front();
      }
      ++visual_se3_photometric_pending_stale_drops_;
      ++visual_se3_photometric_pending_queue_trim_drops_;
    }
  }

  void ingest_pending_visual_factors_into_optimizer(
    const gaussian_lic_tracking::TrajectoryPose & tracking_pose)
  {
    if (!enable_sliding_window_optimizer_) {
      return;
    }
    add_batched_marginalized_visual_priors_from_pending(tracking_pose, true);
    if (enable_visual_alignment_window_factor_) {
      std::deque<PendingVisualAlignmentFactor> retained_pending;
      for (const auto & pending : pending_visual_alignment_factors_) {
        if (visual_factor_should_defer_future_reference(pending.stamp_ns, tracking_pose)) {
          retained_pending.push_back(pending);
          ++visual_alignment_pending_future_deferrals_;
          continue;
        }
        if (visual_pending_factor_needs_marginalized_prior(pending.stamp_ns, tracking_pose) &&
          visual_marginalization_prior_saturation_gate_rejects(pending))
        {
          record_visual_marginalization_prior_saturation_rejection();
          continue;
        }
        if (visual_factor_stamp_is_before_active_window(pending.stamp_ns) &&
          add_marginalized_visual_prior_from_pending(pending, true, false))
        {
          continue;
        }
        const auto visual_reference = select_visual_factor_reference(pending.stamp_ns, tracking_pose);
        if (!visual_reference.has_value()) {
          if (visual_factor_stamp_is_expired(pending.stamp_ns, tracking_pose.stamp_ns)) {
            if (add_marginalized_visual_prior_from_pending(pending, true)) {
              continue;
            }
            const auto projection_reference = expired_visual_projection_reference_pose(tracking_pose);
            const auto projected =
              make_projected_expired_visual_factor(pending, projection_reference);
            if (projected.has_value()) {
              try {
                sliding_window_optimizer_.add_visual_alignment_factor(projected.value());
                ++sliding_window_total_visual_factors_;
                ++visual_callback_ingested_visual_factors_;
                ++visual_alignment_expired_projected_factors_;
              } catch (const std::exception & ex) {
                ++sliding_window_visual_factor_skip_count_;
                ++visual_expired_projection_skipped_factors_;
                RCLCPP_WARN_THROTTLE(
                  get_logger(), *get_clock(), 2000,
                  "projected expired visual factor skipped: %s", ex.what());
              }
            } else {
              ++visual_alignment_pending_stale_drops_;
              ++visual_alignment_pending_expired_drops_;
              ++visual_expired_projection_skipped_factors_;
            }
          } else {
            retained_pending.push_back(pending);
          }
          continue;
        }
        gaussian_lic_tracking::SlidingWindowVisualAlignmentFactor visual_factor;
        visual_factor.stamp_ns = visual_reference->pose.stamp_ns;
        visual_factor.source_id = pending.source_id;
        visual_factor.support_stamp_ns = visual_reference->support_stamp_ns;
        visual_factor.support_weights = visual_reference->support_weights;
        visual_factor.reference_p_w_i =
          pending.has_reference_pose ? pending.reference_p_w_i : visual_reference->pose.p_w_i;
        visual_factor.measured_shift_px = Eigen::Vector2d{
          pending.alignment.subpixel_dx,
          pending.alignment.subpixel_dy};
        visual_factor.component_weight_xy = pending.component_weight_xy;
        visual_factor.meters_per_pixel = visual_alignment_meters_per_pixel_;
        visual_factor.weight =
          visual_alignment_effective_weight(pending.alignment) *
          visual_alignment_quality_weight_scale(pending);
        visual_factor.huber_delta_m = visual_alignment_huber_delta_m_;
        try {
          sliding_window_optimizer_.add_visual_alignment_factor(visual_factor);
          ++sliding_window_total_visual_factors_;
          ++visual_callback_ingested_visual_factors_;
          if (visual_reference->interpolated) {
            ++visual_alignment_interpolated_factor_count_;
          }
        } catch (const std::exception & ex) {
          ++sliding_window_visual_factor_skip_count_;
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "callback-ingested visual factor skipped: %s", ex.what());
        }
      }
      pending_visual_alignment_factors_ = std::move(retained_pending);
    }
    if (enable_se3_photometric_window_factor_) {
      std::deque<PendingSe3PhotometricFactor> retained_pending;
      for (const auto & pending : pending_visual_se3_photometric_factors_) {
        if (visual_factor_should_defer_future_reference(pending.stamp_ns, tracking_pose)) {
          retained_pending.push_back(pending);
          ++visual_se3_photometric_pending_future_deferrals_;
          continue;
        }
        if (visual_pending_factor_needs_marginalized_prior(pending.stamp_ns, tracking_pose) &&
          visual_marginalization_prior_saturation_gate_rejects(pending))
        {
          record_se3_marginalization_prior_saturation_rejection();
          continue;
        }
        if (visual_factor_stamp_is_before_active_window(pending.stamp_ns) &&
          add_marginalized_se3_prior_from_pending(pending, true, false))
        {
          continue;
        }
        const auto visual_reference = select_visual_factor_reference(pending.stamp_ns, tracking_pose);
        if (!visual_reference.has_value()) {
          if (visual_factor_stamp_is_expired(pending.stamp_ns, tracking_pose.stamp_ns)) {
            if (add_marginalized_se3_prior_from_pending(pending, true)) {
              continue;
            }
            const auto projection_reference = expired_visual_projection_reference_pose(tracking_pose);
            const auto projected =
              make_projected_expired_se3_factor(pending, projection_reference);
            if (projected.has_value()) {
              try {
                sliding_window_optimizer_.add_se3_photometric_factor(projected.value());
                ++sliding_window_total_se3_photometric_factors_;
                ++visual_callback_ingested_se3_photometric_factors_;
                ++visual_se3_photometric_expired_projected_factors_;
              } catch (const std::exception & ex) {
                ++sliding_window_se3_photometric_factor_skip_count_;
                ++visual_expired_projection_skipped_factors_;
                RCLCPP_WARN_THROTTLE(
                  get_logger(), *get_clock(), 2000,
                  "projected expired SE3 photometric factor skipped: %s", ex.what());
              }
            } else {
              ++visual_se3_photometric_pending_stale_drops_;
              ++visual_se3_photometric_pending_expired_drops_;
              ++visual_expired_projection_skipped_factors_;
            }
          } else {
            retained_pending.push_back(pending);
          }
          continue;
        }
        gaussian_lic_tracking::SlidingWindowSe3PhotometricFactor factor;
        factor.stamp_ns = visual_reference->pose.stamp_ns;
        factor.source_id = pending.source_id;
        factor.support_stamp_ns = visual_reference->support_stamp_ns;
        factor.support_weights = visual_reference->support_weights;
        factor.reference_p_w_i =
          pending.has_reference_pose ? pending.reference_p_w_i : visual_reference->pose.p_w_i;
        factor.reference_q_w_i =
          pending.has_reference_pose ? pending.reference_q_w_i.normalized() :
          visual_reference->pose.q_w_i;
        factor.target_delta = gaussian_lic_tracking::transform_camera_delta_to_body(
          q_i_c_,
          p_i_c_,
          pending.linearization.gauss_newton_step);
        const Eigen::Matrix<double, 6, 6> body_hessian =
          gaussian_lic_tracking::transform_camera_information_to_body(
          q_i_c_,
          p_i_c_,
          pending.linearization.hessian);
        factor.sqrt_information =
          gaussian_lic_tracking::sqrt_information_from_hessian(body_hessian);
        factor.weight =
          se3_photometric_window_weight_ * se3_photometric_quality_weight_scale(pending);
        factor.huber_delta = se3_photometric_factor_huber_delta_;
        if (factor.sqrt_information.norm() <= std::numeric_limits<double>::epsilon()) {
          ++sliding_window_se3_photometric_factor_skip_count_;
          continue;
        }
        try {
          sliding_window_optimizer_.add_se3_photometric_factor(factor);
          ++sliding_window_total_se3_photometric_factors_;
          ++visual_callback_ingested_se3_photometric_factors_;
          if (visual_reference->interpolated) {
            ++visual_se3_photometric_interpolated_factor_count_;
          }
        } catch (const std::exception & ex) {
          ++sliding_window_se3_photometric_factor_skip_count_;
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "callback-ingested SE3 photometric factor skipped: %s", ex.what());
        }
      }
      pending_visual_se3_photometric_factors_ = std::move(retained_pending);
    }
  }

  std::optional<int64_t> select_rendered_feedback_source_motion_state_stamp(
    const int64_t reference_stamp_ns,
    const gaussian_lic_tracking::TrajectoryPose & tracking_pose) const
  {
    const int64_t max_delta_ns = max_visual_factor_reference_delta_ns();
    std::optional<int64_t> best_stamp_ns;
    int64_t best_delta_ns = std::numeric_limits<int64_t>::max();
    const auto consider = [&](const int64_t candidate_stamp_ns) {
        const int64_t delta_ns = stamp_delta_ns(candidate_stamp_ns, reference_stamp_ns);
        if (delta_ns <= max_delta_ns && delta_ns < best_delta_ns) {
          best_delta_ns = delta_ns;
          best_stamp_ns = candidate_stamp_ns;
        }
      };
    for (const auto & state : sliding_window_optimizer_.states()) {
      consider(state.stamp_ns);
    }
    consider(tracking_pose.stamp_ns);
    return best_stamp_ns;
  }

  gaussian_lic_tracking::SlidingWindowRelativeTranslationFactor
  make_rendered_feedback_source_motion_factor(
    const PendingRenderedFeedbackSourceMotionFactor & pending,
    const int64_t from_stamp_ns,
    const int64_t to_stamp_ns) const
  {
    gaussian_lic_tracking::SlidingWindowRelativeTranslationFactor factor;
    factor.from_stamp_ns = from_stamp_ns;
    factor.to_stamp_ns = to_stamp_ns;
    factor.source_id = pending.source_id;
    factor.delta_p_w = pending.delta_p_w;
    factor.delta_q_from_to = pending.delta_q_from_to.normalized();
    factor.translation_in_from_frame = rendered_feedback_source_motion_in_from_frame_;
    factor.weight = rendered_feedback_source_motion_translation_weight_;
    factor.huber_delta_m = rendered_feedback_source_motion_huber_delta_m_;
    factor.rotation_weight = rendered_feedback_source_motion_rotation_weight_;
    factor.rotation_huber_delta_rad =
      rendered_feedback_source_motion_rotation_huber_delta_rad_;
    return factor;
  }

  bool append_pending_rendered_feedback_source_motion_to_batch(
    PendingRenderedFeedbackSourceMotionBatch & batch,
    const PendingRenderedFeedbackSourceMotionFactor & pending) const
  {
    if (batch.source_factor_count == 0U) {
      batch.factor = pending;
      batch.source_factor_count = 1U;
      return true;
    }
    if (rendered_feedback_source_motion_in_from_frame_ ||
      batch.factor.to_reference_stamp_ns != pending.from_reference_stamp_ns ||
      batch.factor.to_reference_stamp_ns >= pending.to_reference_stamp_ns)
    {
      return false;
    }
    batch.factor.to_reference_stamp_ns = pending.to_reference_stamp_ns;
    batch.factor.source_id = mix_visual_factor_source_id(batch.factor.source_id, pending.source_id);
    batch.factor.delta_p_w += pending.delta_p_w;
    batch.factor.delta_q_from_to =
      (batch.factor.delta_q_from_to * pending.delta_q_from_to).normalized();
    ++batch.source_factor_count;
    return batch.factor.delta_p_w.allFinite() &&
           batch.factor.delta_q_from_to.coeffs().allFinite() &&
           batch.factor.delta_q_from_to.norm() > std::numeric_limits<double>::epsilon();
  }

  void flush_rendered_feedback_source_motion_marginalized_batch(
    PendingRenderedFeedbackSourceMotionBatch & batch)
  {
    if (batch.source_factor_count == 0U) {
      return;
    }
    const auto late_factor =
      make_rendered_feedback_source_motion_factor(
      batch.factor,
      batch.factor.from_reference_stamp_ns,
      batch.factor.to_reference_stamp_ns);
    if (sliding_window_optimizer_.add_marginalized_relative_translation_prior(late_factor)) {
      ++rendered_feedback_source_motion_marginalized_priors_;
      rendered_feedback_source_motion_marginalized_source_factors_ +=
        batch.source_factor_count;
    } else {
      ++rendered_feedback_source_motion_marginalized_prior_skips_;
      rendered_feedback_source_motion_stale_drops_ += batch.source_factor_count;
    }
    batch = PendingRenderedFeedbackSourceMotionBatch{};
  }

  void append_rendered_feedback_source_motion_factors(
    const gaussian_lic_tracking::TrajectoryPose & tracking_pose,
    std::vector<gaussian_lic_tracking::SlidingWindowRelativeTranslationFactor> & factors)
  {
    if (!enable_rendered_feedback_source_motion_factor_ ||
      rendered_feedback_source_motion_translation_weight_ <= 0.0 ||
      pending_rendered_feedback_source_motion_factors_.empty())
    {
      return;
    }

    std::deque<PendingRenderedFeedbackSourceMotionFactor> retained_pending;
    PendingRenderedFeedbackSourceMotionBatch late_batch;
    for (const auto & pending : pending_rendered_feedback_source_motion_factors_) {
      if (pending.to_reference_stamp_ns > tracking_pose.stamp_ns) {
        flush_rendered_feedback_source_motion_marginalized_batch(late_batch);
        retained_pending.push_back(pending);
        ++rendered_feedback_source_motion_future_deferrals_;
        continue;
      }

      const auto from_stamp_ns =
        select_rendered_feedback_source_motion_state_stamp(
        pending.from_reference_stamp_ns, tracking_pose);
      const auto to_stamp_ns =
        select_rendered_feedback_source_motion_state_stamp(
        pending.to_reference_stamp_ns, tracking_pose);
      if (!from_stamp_ns.has_value() || !to_stamp_ns.has_value() ||
        from_stamp_ns.value() >= to_stamp_ns.value())
      {
        const bool stale_or_before_active =
          visual_factor_stamp_is_expired(pending.to_reference_stamp_ns, tracking_pose.stamp_ns) ||
          visual_factor_stamp_is_before_active_window(pending.from_reference_stamp_ns);
        if (enable_rendered_feedback_source_motion_marginalized_prior_ && stale_or_before_active) {
          if (!append_pending_rendered_feedback_source_motion_to_batch(late_batch, pending)) {
            flush_rendered_feedback_source_motion_marginalized_batch(late_batch);
            if (append_pending_rendered_feedback_source_motion_to_batch(late_batch, pending)) {
              continue;
            }
            ++rendered_feedback_source_motion_marginalized_prior_skips_;
            ++rendered_feedback_source_motion_stale_drops_;
            continue;
          }
          continue;
        }
        flush_rendered_feedback_source_motion_marginalized_batch(late_batch);
        if (stale_or_before_active) {
          ++rendered_feedback_source_motion_stale_drops_;
        } else {
          retained_pending.push_back(pending);
        }
        continue;
      }

      flush_rendered_feedback_source_motion_marginalized_batch(late_batch);
      factors.push_back(
        make_rendered_feedback_source_motion_factor(
          pending, from_stamp_ns.value(), to_stamp_ns.value()));
      ++rendered_feedback_source_motion_factors_;
    }
    flush_rendered_feedback_source_motion_marginalized_batch(late_batch);
    pending_rendered_feedback_source_motion_factors_ = std::move(retained_pending);
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
    const int64_t stamp_ns = gaussian_lic_tracking::stamp_to_nanoseconds(msg.header.stamp);
    process_ready_pointcloud_queue();
    if (!pending_pointclouds_waiting_for_imu_.empty() || should_wait_for_imu(stamp_ns)) {
      enqueue_pointcloud_for_imu(msg, stamp_ns);
      return;
    }
    process_pointcloud(msg);
  }

  bool should_wait_for_imu(const int64_t pointcloud_stamp_ns) const
  {
    if (!enable_pointcloud_imu_wait_ || !enable_sliding_window_optimizer_) {
      return false;
    }
    if (enable_imu_gravity_autocalibration_ && !imu_gravity_autocalibrated_ &&
      !imu_propagator_.initialized())
    {
      return true;
    }
    if (!imu_propagator_.initialized() || last_imu_stamp_ns_ == 0)
    {
      return false;
    }
    return last_imu_stamp_ns_ + pointcloud_imu_wait_tolerance_ns_ < pointcloud_stamp_ns;
  }

  void enqueue_pointcloud_for_imu(
    const sensor_msgs::msg::PointCloud2 & msg,
    const int64_t stamp_ns)
  {
    if (!pending_pointclouds_waiting_for_imu_.empty() &&
      stamp_ns <= pending_pointclouds_waiting_for_imu_.back().stamp_ns)
    {
      ++pointcloud_imu_wait_dropped_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "dropping queued point cloud with non-monotonic stamp while waiting for IMU");
      return;
    }
    while (pending_pointclouds_waiting_for_imu_.size() >=
      static_cast<size_t>(pointcloud_imu_wait_queue_size_))
    {
      pending_pointclouds_waiting_for_imu_.pop_front();
      ++pointcloud_imu_wait_dropped_;
    }
    PendingPointCloud pending;
    pending.stamp_ns = stamp_ns;
    pending.message = msg;
    pending_pointclouds_waiting_for_imu_.push_back(std::move(pending));
    ++pointcloud_imu_wait_deferred_;
  }

  void process_ready_pointcloud_queue()
  {
    while (!pending_pointclouds_waiting_for_imu_.empty()) {
      const auto & pending = pending_pointclouds_waiting_for_imu_.front();
      if (should_wait_for_imu(pending.stamp_ns)) {
        return;
      }
      const auto oldest_imu_stamp_ns = imu_propagator_.oldest_history_stamp_ns();
      if (oldest_imu_stamp_ns.has_value() && pending.stamp_ns < oldest_imu_stamp_ns.value()) {
        const auto stale_stamp_ns = pending.stamp_ns;
        pending_pointclouds_waiting_for_imu_.pop_front();
        ++pointcloud_imu_wait_stale_dropped_;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "dropping startup point cloud at %" PRId64
          " because IMU initialization/rebase history starts at %" PRId64,
          stale_stamp_ns,
          oldest_imu_stamp_ns.value());
        continue;
      }
      const auto message = pending.message;
      pending_pointclouds_waiting_for_imu_.pop_front();
      ++pointcloud_imu_wait_released_;
      process_pointcloud(message);
    }
  }

  void process_pointcloud(const sensor_msgs::msg::PointCloud2 & msg)
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
    drain_rendered_feedback_ingress_queue();
    tracking_pose.q_w_i = Eigen::Quaterniond::Identity();
    if (imu_propagator_.initialized()) {
      gaussian_lic_tracking::ImuState state;
      if (!imu_propagator_.query_state(tracking_pose.stamp_ns, state)) {
        const auto & latest_state = imu_propagator_.state();
        const auto oldest_imu_stamp_ns = imu_propagator_.oldest_history_stamp_ns();
        if (oldest_imu_stamp_ns.has_value() && tracking_pose.stamp_ns < oldest_imu_stamp_ns.value()) {
          ++pointcloud_imu_wait_stale_dropped_;
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "dropping startup point cloud at %" PRId64
            " because IMU initialization/rebase history starts at %" PRId64,
            tracking_pose.stamp_ns,
            oldest_imu_stamp_ns.value());
          return;
        }
        if (latest_state.stamp_ns > tracking_pose.stamp_ns) {
          ++lidar_invalid_frames_;
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "dropping point cloud at %" PRId64
            " because the matching IMU state has fallen out of history; latest IMU is %" PRId64,
            tracking_pose.stamp_ns,
            latest_state.stamp_ns);
          return;
        }
        state = latest_state;
      }
      tracking_pose.p_w_i = state.p_w_i;
      tracking_pose.q_w_i = state.q_w_i;
      tracking_pose.v_w_i = state.v_w_i;
    }
    apply_se3_photometric_pose_correction(tracking_pose);
    std::vector<gaussian_lic_tracking::SlidingWindowRelativeTranslationFactor>
      relative_translation_factors;
    std::vector<gaussian_lic_tracking::SlidingWindowRelativeDistanceFactor>
      relative_distance_factors;
    if (enable_sliding_window_relative_translation_factor_ &&
      sliding_window_relative_translation_weight_ > 0.0 &&
      last_output_tracking_pose_.has_value() &&
      tracking_pose.stamp_ns > last_output_tracking_pose_->stamp_ns)
    {
      const auto & previous_pose = last_output_tracking_pose_.value();
      const Eigen::Vector3d raw_delta_p_w = tracking_pose.p_w_i - previous_pose.p_w_i;
      if (raw_delta_p_w.allFinite()) {
        gaussian_lic_tracking::SlidingWindowRelativeTranslationFactor factor;
        factor.from_stamp_ns = previous_pose.stamp_ns;
        factor.to_stamp_ns = tracking_pose.stamp_ns;
        factor.delta_p_w = sliding_window_relative_translation_in_from_frame_
          ? previous_pose.q_w_i.conjugate() * raw_delta_p_w
          : raw_delta_p_w;
        factor.delta_q_from_to =
          (previous_pose.q_w_i.conjugate() * tracking_pose.q_w_i).normalized();
        factor.translation_in_from_frame = sliding_window_relative_translation_in_from_frame_;
        factor.weight = sliding_window_relative_translation_weight_;
        factor.huber_delta_m = sliding_window_relative_translation_huber_delta_m_;
        factor.rotation_weight = sliding_window_relative_rotation_weight_;
        factor.rotation_huber_delta_rad = sliding_window_relative_rotation_huber_delta_rad_;
        relative_translation_factors.push_back(factor);
      }
    }
    if (enable_sliding_window_relative_distance_factor_ &&
      sliding_window_relative_distance_weight_ > 0.0 &&
      last_output_tracking_pose_.has_value() &&
      tracking_pose.stamp_ns > last_output_tracking_pose_->stamp_ns)
    {
      const auto & previous_pose = last_output_tracking_pose_.value();
      const Eigen::Vector3d raw_delta_p_w = tracking_pose.p_w_i - previous_pose.p_w_i;
      if (raw_delta_p_w.allFinite()) {
        gaussian_lic_tracking::SlidingWindowRelativeDistanceFactor factor;
        factor.from_stamp_ns = previous_pose.stamp_ns;
        factor.to_stamp_ns = tracking_pose.stamp_ns;
        factor.distance_m = raw_delta_p_w.norm();
        factor.weight = sliding_window_relative_distance_weight_;
        factor.huber_delta_m = sliding_window_relative_distance_huber_delta_m_;
        relative_distance_factors.push_back(factor);
      }
    }
    if (enable_pre_lio_tracking_step_guard_) {
      apply_tracking_step_guard(tracking_pose, true, StepGuardStage::kPreLio);
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
    cache_sparse_lidar_depth_frame(tracking_pose.stamp_ns, lidar_points);

    std::vector<gaussian_lic_tracking::SlidingWindowPointToPointFactor> window_point_factors;
    std::vector<gaussian_lic_tracking::SlidingWindowPointToPlaneFactor> window_plane_factors;
    std::vector<gaussian_lic_tracking::SlidingWindowPointToLineFactor> window_line_factors;
    size_t window_point_correspondences = 0U;
    size_t window_plane_correspondences = 0U;
    size_t window_point_weight_count = 0U;
    size_t window_plane_weight_count = 0U;
    double window_point_weight_sum = 0.0;
    double window_plane_weight_sum = 0.0;
    double window_point_weight_min = std::numeric_limits<double>::infinity();
    double window_plane_weight_min = std::numeric_limits<double>::infinity();
    if (enable_lio_factor_) {
      const auto correction = lidar_factor_.compute_pose_correction(lidar_points, tracking_pose);
      if (correction.applied) {
        tracking_pose.p_w_i += correction.delta_p_w;
        tracking_pose.q_w_i = (correction.delta_q * tracking_pose.q_w_i).normalized();
      }
      if (enable_gaussian_snapshot_lidar_pose_correction_ && gaussian_snapshot_.complete()) {
        const double gaussian_snapshot_nearest_distance_m =
          gaussian_snapshot_lidar_nearest_distance_m_ > 0.0
          ? gaussian_snapshot_lidar_nearest_distance_m_
          : lidar_nearest_distance_m_;
        const auto gaussian_pose_correction =
          compute_gaussian_snapshot_pose_correction(
          lidar_points,
          tracking_pose,
          gaussian_snapshot_nearest_distance_m);
        if (gaussian_pose_correction.applied) {
          tracking_pose.p_w_i += gaussian_pose_correction.delta_p_w;
          tracking_pose.q_w_i = (gaussian_pose_correction.delta_q * tracking_pose.q_w_i).normalized();
          RCLCPP_DEBUG_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Gaussian snapshot pose correction matched %zu points, residual %.4fm, delta %.4fm",
            gaussian_pose_correction.matched_points,
            gaussian_pose_correction.mean_residual_m,
            gaussian_pose_correction.delta_p_w.norm());
        }
      }
      if (enable_sliding_window_optimizer_) {
        auto lidar_window_factor = lidar_factor_.build_point_to_point_factor(lidar_points, tracking_pose);
        if (!lidar_window_factor.frame_points_i.empty()) {
          lidar_window_factor.weight *= lidar_window_point_factor_weight_;
          apply_correspondence_weight_power(
            lidar_window_factor.point_weights, lidar_window_confidence_power_);
          window_point_correspondences += lidar_window_factor.frame_points_i.size();
          accumulate_correspondence_weights(
            lidar_window_factor.point_weights,
            window_point_weight_count,
            window_point_weight_sum,
            window_point_weight_min);
          window_point_factors.push_back(std::move(lidar_window_factor));
        }
        if (enable_lidar_plane_factor_) {
          auto lidar_plane_factor = lidar_factor_.build_point_to_plane_factor(lidar_points, tracking_pose);
          if (!lidar_plane_factor.frame_points_i.empty()) {
            lidar_plane_factor.weight *= lidar_window_plane_factor_weight_;
            apply_correspondence_weight_power(
              lidar_plane_factor.point_weights, lidar_window_confidence_power_);
            window_plane_correspondences += lidar_plane_factor.frame_points_i.size();
            accumulate_correspondence_weights(
              lidar_plane_factor.point_weights,
              window_plane_weight_count,
              window_plane_weight_sum,
              window_plane_weight_min);
            window_plane_factors.push_back(std::move(lidar_plane_factor));
          }
        }
        if (enable_lidar_line_factor_) {
          auto lidar_line_factor = lidar_factor_.build_point_to_line_factor(lidar_points, tracking_pose);
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "[line-factor] enabled=%d line_corrs=%zu",
            static_cast<int>(enable_lidar_line_factor_),
            lidar_line_factor.frame_points_i.size());
          if (!lidar_line_factor.frame_points_i.empty()) {
            lidar_line_factor.weight *= lidar_window_line_factor_weight_;
            apply_correspondence_weight_power(
              lidar_line_factor.point_weights, lidar_window_confidence_power_);
            window_line_factors.push_back(std::move(lidar_line_factor));
          }
        }
        if (enable_gaussian_snapshot_lidar_factor_ && gaussian_snapshot_.complete()) {
          const double gaussian_snapshot_nearest_distance_m =
            gaussian_snapshot_lidar_nearest_distance_m_ > 0.0
            ? gaussian_snapshot_lidar_nearest_distance_m_
            : lidar_nearest_distance_m_;
          auto gaussian_window_factor = gaussian_snapshot_.build_point_to_point_factor(
            lidar_points,
            tracking_pose,
            static_cast<size_t>(lidar_min_points_),
            static_cast<size_t>(lidar_max_frame_points_),
            gaussian_snapshot_nearest_distance_m,
            gaussian_snapshot_lidar_min_opacity_,
            gaussian_snapshot_lidar_residual_preweight_);
          if (!gaussian_window_factor.frame_points_i.empty()) {
            gaussian_window_factor.weight *= gaussian_snapshot_lidar_factor_weight_;
            apply_correspondence_weight_power(
              gaussian_window_factor.point_weights, lidar_window_confidence_power_);
            window_point_correspondences += gaussian_window_factor.frame_points_i.size();
            accumulate_correspondence_weights(
              gaussian_window_factor.point_weights,
              window_point_weight_count,
              window_point_weight_sum,
              window_point_weight_min);
            window_point_factors.push_back(std::move(gaussian_window_factor));
          }
        }
        if (enable_gaussian_snapshot_lidar_plane_factor_ && gaussian_snapshot_.complete()) {
          const double gaussian_snapshot_nearest_distance_m =
            gaussian_snapshot_lidar_nearest_distance_m_ > 0.0
            ? gaussian_snapshot_lidar_nearest_distance_m_
            : lidar_nearest_distance_m_;
          auto gaussian_plane_factor = gaussian_snapshot_.build_point_to_plane_factor(
            lidar_points,
            tracking_pose,
            static_cast<size_t>(lidar_min_points_),
            static_cast<size_t>(lidar_max_frame_points_),
            gaussian_snapshot_nearest_distance_m,
            gaussian_snapshot_lidar_min_opacity_,
            gaussian_snapshot_lidar_plane_min_anisotropy_,
            gaussian_snapshot_lidar_residual_preweight_);
          if (!gaussian_plane_factor.frame_points_i.empty()) {
            gaussian_plane_factor.weight *= gaussian_snapshot_lidar_plane_factor_weight_;
            apply_correspondence_weight_power(
              gaussian_plane_factor.point_weights, lidar_window_confidence_power_);
            window_plane_correspondences += gaussian_plane_factor.frame_points_i.size();
            accumulate_correspondence_weights(
              gaussian_plane_factor.point_weights,
              window_plane_weight_count,
              window_plane_weight_sum,
              window_plane_weight_min);
            window_plane_factors.push_back(std::move(gaussian_plane_factor));
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
      last_window_point_confidence_mean_ =
        mean_or_zero(window_point_weight_count, window_point_weight_sum);
      last_window_point_confidence_min_ =
        min_or_zero(window_point_weight_count, window_point_weight_min);
      last_window_plane_confidence_mean_ =
        mean_or_zero(window_plane_weight_count, window_plane_weight_sum);
      last_window_plane_confidence_min_ =
        min_or_zero(window_plane_weight_count, window_plane_weight_min);
      const bool lidar_has_tracking_support =
        !has_lidar_keyframe_ || correction.applied ||
        window_point_correspondences + window_plane_correspondences >=
        static_cast<size_t>(lidar_min_points_);
      if (lidar_has_tracking_support && should_insert_lidar_keyframe(tracking_pose, lidar_points.size())) {
        lidar_factor_.insert_keyframe(lidar_points, tracking_pose);
        ++num_lidar_keyframes_;
        last_lidar_keyframe_pose_ = tracking_pose;
        has_lidar_keyframe_ = true;
      }
    }

    std::vector<gaussian_lic_tracking::SlidingWindowVisualAlignmentFactor> visual_window_factors;
    std::vector<gaussian_lic_tracking::SlidingWindowSe3PhotometricFactor> se3_photometric_factors;
    std::vector<double> visual_window_factor_scores;
    std::vector<double> se3_photometric_factor_scores;
    const bool quality_selection_active =
      visual_factor_quality_selection_is_active(tracking_pose.stamp_ns);
    if (enable_visual_watermark_pair_scheduler_) {
      process_visual_pairs_up_to_watermark(tracking_pose.stamp_ns, false);
    }
    if (enable_rendered_feedback_watermark_queue_) {
      process_rendered_feedback_pairs_up_to_watermark(tracking_pose.stamp_ns, false);
    }
    if (!enable_visual_watermark_pair_scheduler_ &&
      (visual_cache_reconciliation_defer_to_pointcloud_ ||
      visual_pair_processing_defer_to_pointcloud_)
    ) {
      reconcile_visual_frame_caches(tracking_pose.stamp_ns);
    }
    add_batched_marginalized_visual_priors_from_pending(tracking_pose, false);
    const auto add_visual_window_factor =
      [this, quality_selection_active, &visual_window_factors, &visual_window_factor_scores](
        gaussian_lic_tracking::SlidingWindowVisualAlignmentFactor factor,
        const double score) {
        if (!quality_selection_active || !enable_visual_factor_quality_reference_cap_) {
          visual_window_factors.push_back(std::move(factor));
          return;
        }
        size_t same_reference_count = 0U;
        std::optional<size_t> worst_same_reference_index;
        double worst_same_reference_score = -std::numeric_limits<double>::infinity();
        for (size_t index = 0; index < visual_window_factors.size(); ++index) {
          if (visual_window_factors[index].stamp_ns == factor.stamp_ns) {
            ++same_reference_count;
            if (visual_window_factor_scores[index] > worst_same_reference_score) {
              worst_same_reference_score = visual_window_factor_scores[index];
              worst_same_reference_index = index;
            }
          }
        }
        if (same_reference_count >=
          static_cast<size_t>(visual_factor_quality_selection_max_per_reference_))
        {
          if (!worst_same_reference_index.has_value() || score >= worst_same_reference_score) {
            return;
          }
          visual_window_factors[worst_same_reference_index.value()] = std::move(factor);
          visual_window_factor_scores[worst_same_reference_index.value()] = score;
          return;
        }
        visual_window_factors.push_back(std::move(factor));
        visual_window_factor_scores.push_back(score);
      };
    const auto add_se3_photometric_factor =
      [this, quality_selection_active, &se3_photometric_factors, &se3_photometric_factor_scores](
        gaussian_lic_tracking::SlidingWindowSe3PhotometricFactor factor,
        const double score) {
        if (!quality_selection_active || !enable_visual_factor_quality_reference_cap_) {
          se3_photometric_factors.push_back(std::move(factor));
          return;
        }
        size_t same_reference_count = 0U;
        std::optional<size_t> worst_same_reference_index;
        double worst_same_reference_score = -std::numeric_limits<double>::infinity();
        for (size_t index = 0; index < se3_photometric_factors.size(); ++index) {
          if (se3_photometric_factors[index].stamp_ns == factor.stamp_ns) {
            ++same_reference_count;
            if (se3_photometric_factor_scores[index] > worst_same_reference_score) {
              worst_same_reference_score = se3_photometric_factor_scores[index];
              worst_same_reference_index = index;
            }
          }
        }
        if (same_reference_count >=
          static_cast<size_t>(visual_factor_quality_selection_max_per_reference_))
        {
          if (!worst_same_reference_index.has_value() || score >= worst_same_reference_score) {
            return;
          }
          se3_photometric_factors[worst_same_reference_index.value()] = std::move(factor);
          se3_photometric_factor_scores[worst_same_reference_index.value()] = score;
          return;
        }
        se3_photometric_factors.push_back(std::move(factor));
        se3_photometric_factor_scores.push_back(score);
      };
    if (enable_sliding_window_optimizer_ && enable_visual_alignment_window_factor_) {
      std::deque<PendingVisualAlignmentFactor> retained_pending;
      for (const auto & pending : pending_visual_alignment_factors_) {
        if (visual_factor_should_defer_future_reference(pending.stamp_ns, tracking_pose)) {
          retained_pending.push_back(pending);
          ++visual_alignment_pending_future_deferrals_;
          continue;
        }
        if (visual_factor_stamp_is_before_active_window(pending.stamp_ns) &&
          add_marginalized_visual_prior_from_pending(pending, false, false))
        {
          continue;
        }
        const auto visual_reference = select_visual_factor_reference(pending.stamp_ns, tracking_pose);
        if (!visual_reference.has_value()) {
          if (visual_factor_stamp_is_expired(pending.stamp_ns, tracking_pose.stamp_ns)) {
            if (add_marginalized_visual_prior_from_pending(pending, false)) {
              continue;
            }
            const auto projection_reference = expired_visual_projection_reference_pose(tracking_pose);
            const auto projected =
              make_projected_expired_visual_factor(pending, projection_reference);
            if (projected.has_value()) {
              add_visual_window_factor(projected.value(), visual_alignment_candidate_score(pending));
              ++visual_alignment_expired_projected_factors_;
            } else {
              ++visual_alignment_pending_stale_drops_;
              ++visual_alignment_pending_expired_drops_;
              ++visual_expired_projection_skipped_factors_;
            }
          } else {
            retained_pending.push_back(pending);
          }
          continue;
        }
        gaussian_lic_tracking::SlidingWindowVisualAlignmentFactor visual_factor;
        visual_factor.stamp_ns = visual_reference->pose.stamp_ns;
        visual_factor.source_id = pending.source_id;
        visual_factor.support_stamp_ns = visual_reference->support_stamp_ns;
        visual_factor.support_weights = visual_reference->support_weights;
        visual_factor.reference_p_w_i =
          pending.has_reference_pose ? pending.reference_p_w_i : visual_reference->pose.p_w_i;
        visual_factor.measured_shift_px = Eigen::Vector2d{
          pending.alignment.subpixel_dx,
          pending.alignment.subpixel_dy};
        visual_factor.component_weight_xy = pending.component_weight_xy;
        visual_factor.meters_per_pixel = visual_alignment_meters_per_pixel_;
        visual_factor.weight =
          visual_alignment_effective_weight(pending.alignment) *
          visual_alignment_quality_weight_scale(pending);
        visual_factor.huber_delta_m = visual_alignment_huber_delta_m_;
        if (visual_reference->interpolated) {
          ++visual_alignment_interpolated_factor_count_;
        }
        add_visual_window_factor(visual_factor, visual_alignment_candidate_score(pending));
      }
      pending_visual_alignment_factors_ = std::move(retained_pending);
    }
    if (enable_sliding_window_optimizer_ && enable_se3_photometric_window_factor_) {
      std::deque<PendingSe3PhotometricFactor> retained_pending;
      for (const auto & pending : pending_visual_se3_photometric_factors_) {
        if (visual_factor_should_defer_future_reference(pending.stamp_ns, tracking_pose)) {
          retained_pending.push_back(pending);
          ++visual_se3_photometric_pending_future_deferrals_;
          continue;
        }
        if (visual_factor_stamp_is_before_active_window(pending.stamp_ns) &&
          add_marginalized_se3_prior_from_pending(pending, false, false))
        {
          continue;
        }
        const auto visual_reference = select_visual_factor_reference(pending.stamp_ns, tracking_pose);
        if (!visual_reference.has_value()) {
          if (visual_factor_stamp_is_expired(pending.stamp_ns, tracking_pose.stamp_ns)) {
            if (add_marginalized_se3_prior_from_pending(pending, false)) {
              continue;
            }
            const auto projection_reference = expired_visual_projection_reference_pose(tracking_pose);
            const auto projected =
              make_projected_expired_se3_factor(pending, projection_reference);
            if (projected.has_value()) {
              add_se3_photometric_factor(projected.value(), se3_photometric_candidate_score(pending));
              ++visual_se3_photometric_expired_projected_factors_;
            } else {
              ++visual_se3_photometric_pending_stale_drops_;
              ++visual_se3_photometric_pending_expired_drops_;
              ++visual_expired_projection_skipped_factors_;
            }
          } else {
            retained_pending.push_back(pending);
          }
          continue;
        }
        gaussian_lic_tracking::SlidingWindowSe3PhotometricFactor factor;
        factor.stamp_ns = visual_reference->pose.stamp_ns;
        factor.source_id = pending.source_id;
        factor.support_stamp_ns = visual_reference->support_stamp_ns;
        factor.support_weights = visual_reference->support_weights;
        factor.reference_p_w_i =
          pending.has_reference_pose ? pending.reference_p_w_i : visual_reference->pose.p_w_i;
        factor.reference_q_w_i =
          pending.has_reference_pose ? pending.reference_q_w_i.normalized() :
          visual_reference->pose.q_w_i;
        factor.target_delta = gaussian_lic_tracking::transform_camera_delta_to_body(
          q_i_c_,
          p_i_c_,
          pending.linearization.gauss_newton_step);
        const Eigen::Matrix<double, 6, 6> body_hessian =
          gaussian_lic_tracking::transform_camera_information_to_body(
          q_i_c_,
          p_i_c_,
          pending.linearization.hessian);
        factor.sqrt_information =
          gaussian_lic_tracking::sqrt_information_from_hessian(body_hessian);
        factor.weight =
          se3_photometric_window_weight_ * se3_photometric_quality_weight_scale(pending);
        factor.huber_delta = se3_photometric_factor_huber_delta_;
        if (factor.sqrt_information.norm() > std::numeric_limits<double>::epsilon()) {
          if (visual_reference->interpolated) {
            ++visual_se3_photometric_interpolated_factor_count_;
          }
          add_se3_photometric_factor(factor, se3_photometric_candidate_score(pending));
        } else {
          ++sliding_window_se3_photometric_factor_skip_count_;
        }
      }
      pending_visual_se3_photometric_factors_ = std::move(retained_pending);
    }
    append_multihop_relative_translation_factors(tracking_pose, relative_translation_factors);
    append_multihop_relative_distance_factors(tracking_pose, relative_distance_factors);
    append_rendered_feedback_source_motion_factors(tracking_pose, relative_translation_factors);

    const auto pre_ba_tracking_pose = tracking_pose;
    if (enable_sliding_window_optimizer_) {
      tracking_pose = update_sliding_window(
        tracking_pose,
        window_point_factors,
        window_plane_factors,
        window_line_factors,
        visual_window_factors,
        se3_photometric_factors,
        relative_translation_factors,
        relative_distance_factors);
    }
    bool post_ba_guard_adjusted_pose = false;
    if (enable_post_ba_tracking_step_guard_) {
      post_ba_guard_adjusted_pose = apply_tracking_step_guard(
        tracking_pose, true, StepGuardStage::kPostBa, &pre_ba_tracking_pose);
    }
    if (post_ba_guard_adjusted_pose && sliding_window_sync_guarded_pose_state_) {
      sync_guarded_pose_to_sliding_window_state(tracking_pose);
    }
    if (post_ba_guard_adjusted_pose) {
      add_guarded_pose_prior_to_sliding_window(tracking_pose);
    }
    add_delayed_published_multihop_relative_translation_factors(tracking_pose);
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

    // Deterministic-replay trajectory output. Gated by the output stream being
    // open (only opened when output_tum_path is set), so the production async
    // path is unaffected. TUM format: timestamp tx ty tz qx qy qz qw.
    if (output_tum_stream_.is_open()) {
      const double stamp_s =
        static_cast<double>(pose.header.stamp.sec) +
        static_cast<double>(pose.header.stamp.nanosec) * 1.0e-9;
      output_tum_stream_ << std::fixed << std::setprecision(9) << stamp_s << ' '
                         << std::setprecision(9) << tracking_pose.p_w_i.x() << ' '
                         << tracking_pose.p_w_i.y() << ' '
                         << tracking_pose.p_w_i.z() << ' '
                         << tracking_pose.q_w_i.x() << ' '
                         << tracking_pose.q_w_i.y() << ' '
                         << tracking_pose.q_w_i.z() << ' '
                         << tracking_pose.q_w_i.w() << '\n';
    }

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
    last_output_tracking_pose_ = tracking_pose;
    publish_tracking_status(msg.header.stamp);
    cache_relative_motion_pose(
      relative_motion_pose_history_,
      select_relative_motion_history_pose(pre_ba_tracking_pose, tracking_pose));
    cache_relative_motion_pose(published_relative_motion_pose_history_, tracking_pose);
  }

  void append_multihop_relative_translation_factors(
    const gaussian_lic_tracking::TrajectoryPose & pre_ba_pose,
    std::vector<gaussian_lic_tracking::SlidingWindowRelativeTranslationFactor> & factors) const
  {
    if (!enable_sliding_window_multihop_relative_translation_factor_ ||
      sliding_window_multihop_relative_translation_weight_ <= 0.0 ||
      relative_motion_pose_history_.empty())
    {
      return;
    }
    const int64_t min_dt_ns = static_cast<int64_t>(
      sliding_window_multihop_relative_translation_min_dt_s_ *
      static_cast<double>(gaussian_lic_tracking::kNanosecondsPerSecond));
    const int64_t max_dt_ns = static_cast<int64_t>(
      sliding_window_multihop_relative_translation_max_dt_s_ *
      static_cast<double>(gaussian_lic_tracking::kNanosecondsPerSecond));
    size_t added_factors = 0U;
    for (const auto & history_pose : relative_motion_pose_history_) {
      if (history_pose.stamp_ns >= pre_ba_pose.stamp_ns) {
        continue;
      }
      const int64_t dt_ns = pre_ba_pose.stamp_ns - history_pose.stamp_ns;
      if (dt_ns < min_dt_ns || dt_ns > max_dt_ns) {
        continue;
      }
      const auto factor = make_multihop_relative_translation_factor(history_pose, pre_ba_pose);
      if (!factor.has_value()) {
        continue;
      }
      factors.push_back(factor.value());
      ++added_factors;
      if (added_factors >=
        static_cast<size_t>(sliding_window_multihop_relative_translation_max_factors_))
      {
        return;
      }
    }
  }

  std::optional<gaussian_lic_tracking::SlidingWindowRelativeTranslationFactor>
  make_multihop_relative_translation_factor(
    const gaussian_lic_tracking::TrajectoryPose & from_pose,
    const gaussian_lic_tracking::TrajectoryPose & to_pose,
    const uint64_t source_id = 0U) const
  {
    if (from_pose.stamp_ns >= to_pose.stamp_ns ||
      !from_pose.p_w_i.allFinite() || !to_pose.p_w_i.allFinite() ||
      !from_pose.q_w_i.coeffs().allFinite() || !to_pose.q_w_i.coeffs().allFinite() ||
      from_pose.q_w_i.norm() <= std::numeric_limits<double>::epsilon() ||
      to_pose.q_w_i.norm() <= std::numeric_limits<double>::epsilon())
    {
      return std::nullopt;
    }
    const Eigen::Vector3d delta_p_w = to_pose.p_w_i - from_pose.p_w_i;
    if (!delta_p_w.allFinite()) {
      return std::nullopt;
    }
    gaussian_lic_tracking::SlidingWindowRelativeTranslationFactor factor;
    factor.from_stamp_ns = from_pose.stamp_ns;
    factor.to_stamp_ns = to_pose.stamp_ns;
    factor.source_id = source_id;
    factor.delta_p_w = sliding_window_multihop_relative_translation_in_from_frame_
      ? from_pose.q_w_i.conjugate() * delta_p_w
      : delta_p_w;
    factor.delta_q_from_to =
      (from_pose.q_w_i.conjugate() * to_pose.q_w_i).normalized();
    factor.translation_in_from_frame =
      sliding_window_multihop_relative_translation_in_from_frame_;
    factor.weight = sliding_window_multihop_relative_translation_weight_;
    factor.huber_delta_m = sliding_window_multihop_relative_translation_huber_delta_m_;
    factor.rotation_weight = sliding_window_multihop_relative_rotation_weight_;
    factor.rotation_huber_delta_rad =
      sliding_window_multihop_relative_rotation_huber_delta_rad_;
    return factor;
  }

  void append_multihop_relative_distance_factors(
    const gaussian_lic_tracking::TrajectoryPose & pre_ba_pose,
    std::vector<gaussian_lic_tracking::SlidingWindowRelativeDistanceFactor> & factors) const
  {
    if (!enable_sliding_window_multihop_relative_distance_factor_ ||
      sliding_window_multihop_relative_distance_weight_ <= 0.0 ||
      relative_motion_pose_history_.empty())
    {
      return;
    }
    const int64_t min_dt_ns = static_cast<int64_t>(
      sliding_window_multihop_relative_translation_min_dt_s_ *
      static_cast<double>(gaussian_lic_tracking::kNanosecondsPerSecond));
    const int64_t max_dt_ns = static_cast<int64_t>(
      sliding_window_multihop_relative_translation_max_dt_s_ *
      static_cast<double>(gaussian_lic_tracking::kNanosecondsPerSecond));
    size_t added_factors = 0U;
    for (const auto & history_pose : relative_motion_pose_history_) {
      if (history_pose.stamp_ns >= pre_ba_pose.stamp_ns) {
        continue;
      }
      const int64_t dt_ns = pre_ba_pose.stamp_ns - history_pose.stamp_ns;
      if (dt_ns < min_dt_ns || dt_ns > max_dt_ns) {
        continue;
      }
      const auto factor = make_multihop_relative_distance_factor(history_pose, pre_ba_pose);
      if (!factor.has_value()) {
        continue;
      }
      factors.push_back(factor.value());
      ++added_factors;
      if (added_factors >=
        static_cast<size_t>(sliding_window_multihop_relative_translation_max_factors_))
      {
        return;
      }
    }
  }

  std::optional<gaussian_lic_tracking::SlidingWindowRelativeDistanceFactor>
  make_multihop_relative_distance_factor(
    const gaussian_lic_tracking::TrajectoryPose & from_pose,
    const gaussian_lic_tracking::TrajectoryPose & to_pose,
    const uint64_t source_id = 0U) const
  {
    if (from_pose.stamp_ns >= to_pose.stamp_ns ||
      !from_pose.p_w_i.allFinite() || !to_pose.p_w_i.allFinite())
    {
      return std::nullopt;
    }
    const Eigen::Vector3d delta_p_w = to_pose.p_w_i - from_pose.p_w_i;
    if (!delta_p_w.allFinite()) {
      return std::nullopt;
    }
    gaussian_lic_tracking::SlidingWindowRelativeDistanceFactor factor;
    factor.from_stamp_ns = from_pose.stamp_ns;
    factor.to_stamp_ns = to_pose.stamp_ns;
    factor.source_id = source_id;
    factor.distance_m = delta_p_w.norm();
    factor.weight = sliding_window_multihop_relative_distance_weight_;
    factor.huber_delta_m = sliding_window_multihop_relative_distance_huber_delta_m_;
    return factor;
  }

  void add_delayed_published_multihop_relative_translation_factors(
    const gaussian_lic_tracking::TrajectoryPose & published_pose)
  {
    if (!enable_sliding_window_delayed_published_multihop_relative_translation_factor_ ||
      !enable_sliding_window_multihop_relative_translation_factor_ ||
      sliding_window_multihop_relative_translation_weight_ <= 0.0 ||
      published_relative_motion_pose_history_.empty() ||
      !sliding_window_start_stamp_ns_.has_value())
    {
      return;
    }
    const double elapsed_s =
      static_cast<double>(published_pose.stamp_ns - sliding_window_start_stamp_ns_.value()) /
      static_cast<double>(gaussian_lic_tracking::kNanosecondsPerSecond);
    if (elapsed_s < sliding_window_delayed_published_multihop_start_after_s_) {
      return;
    }
    const int64_t min_dt_ns = static_cast<int64_t>(
      sliding_window_multihop_relative_translation_min_dt_s_ *
      static_cast<double>(gaussian_lic_tracking::kNanosecondsPerSecond));
    const int64_t max_dt_ns = static_cast<int64_t>(
      sliding_window_multihop_relative_translation_max_dt_s_ *
      static_cast<double>(gaussian_lic_tracking::kNanosecondsPerSecond));
    size_t added_factors = 0U;
    for (const auto & history_pose : published_relative_motion_pose_history_) {
      if (history_pose.stamp_ns >= published_pose.stamp_ns) {
        continue;
      }
      const int64_t dt_ns = published_pose.stamp_ns - history_pose.stamp_ns;
      if (dt_ns < min_dt_ns || dt_ns > max_dt_ns) {
        continue;
      }
      const auto factor = make_multihop_relative_translation_factor(
        history_pose,
        published_pose,
        1U);
      if (!factor.has_value()) {
        continue;
      }
      try {
        sliding_window_optimizer_.add_relative_translation_factor(factor.value());
        ++sliding_window_delayed_published_multihop_relative_factor_count_;
        ++added_factors;
      } catch (const std::exception & ex) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "delayed published multi-hop relative factor skipped: %s", ex.what());
      }
      if (added_factors >=
        static_cast<size_t>(sliding_window_delayed_published_multihop_max_factors_))
      {
        return;
      }
    }
  }

  GaussianSnapshotPoseCorrection compute_gaussian_snapshot_pose_correction(
    const std::vector<Eigen::Vector3d> & frame_points_i,
    const gaussian_lic_tracking::TrajectoryPose & predicted_pose,
    const double nearest_distance_m) const
  {
    GaussianSnapshotPoseCorrection correction;
    if (!gaussian_snapshot_.complete() || frame_points_i.size() < static_cast<size_t>(lidar_min_points_) ||
      gaussian_snapshot_.point_count() < static_cast<size_t>(lidar_min_points_) ||
      nearest_distance_m <= 0.0 || !predicted_pose.p_w_i.allFinite() ||
      !predicted_pose.q_w_i.coeffs().allFinite() ||
      predicted_pose.q_w_i.norm() <= std::numeric_limits<double>::epsilon())
    {
      return correction;
    }

    const size_t max_frame_points = static_cast<size_t>(lidar_max_frame_points_);
    const size_t stride = max_frame_points > 0U && frame_points_i.size() > max_frame_points
      ? static_cast<size_t>(
        std::ceil(static_cast<double>(frame_points_i.size()) / static_cast<double>(max_frame_points)))
      : 1U;
    const double max_distance_sq = nearest_distance_m * nearest_distance_m;
    const double robust_kernel_m = 0.5 * nearest_distance_m;
    const Eigen::Quaterniond q_w_i = predicted_pose.q_w_i.normalized();

    std::vector<Eigen::Vector3d> source_w;
    std::vector<Eigen::Vector3d> target_w;
    std::vector<double> match_weights;
    std::vector<double> residual_norms;
    source_w.reserve(std::min(frame_points_i.size(), max_frame_points > 0U ? max_frame_points : frame_points_i.size()));
    target_w.reserve(source_w.capacity());
    match_weights.reserve(source_w.capacity());
    residual_norms.reserve(source_w.capacity());
    double residual_weight_sum = 0.0;
    double residual_norm_sum = 0.0;
    size_t finite_sample_count = 0U;
    for (size_t point_index = 0U; point_index < frame_points_i.size(); point_index += stride) {
      const auto & point_i = frame_points_i[point_index];
      if (!point_i.allFinite()) {
        continue;
      }
      ++finite_sample_count;
      const Eigen::Vector3d point_w = q_w_i * point_i + predicted_pose.p_w_i;
      const auto nearest =
        gaussian_snapshot_.find_nearest(
        point_w,
        nearest_distance_m,
        gaussian_snapshot_lidar_min_opacity_,
        1);
      if (!nearest.matched || nearest.distance_sq > max_distance_sq) {
        continue;
      }
      const double residual_norm = std::sqrt(nearest.distance_sq);
      const double match_weight =
        std::min(1.0, robust_kernel_m / std::max(residual_norm, 1.0e-12));
      if (!std::isfinite(match_weight) || match_weight <= 0.0) {
        continue;
      }
      source_w.push_back(point_w);
      target_w.push_back(nearest.xyz);
      match_weights.push_back(match_weight);
      residual_norms.push_back(residual_norm);
      residual_weight_sum += match_weight;
      residual_norm_sum += match_weight * residual_norm;
    }
    if (gaussian_snapshot_lidar_pose_correction_bidirectional_max_distance_m_ > 0.0 &&
      !source_w.empty())
    {
      const double bidirectional_max_distance_sq =
        gaussian_snapshot_lidar_pose_correction_bidirectional_max_distance_m_ *
        gaussian_snapshot_lidar_pose_correction_bidirectional_max_distance_m_;
      std::vector<Eigen::Vector3d> filtered_source_w;
      std::vector<Eigen::Vector3d> filtered_target_w;
      std::vector<double> filtered_match_weights;
      std::vector<double> filtered_residual_norms;
      filtered_source_w.reserve(source_w.size());
      filtered_target_w.reserve(target_w.size());
      filtered_match_weights.reserve(match_weights.size());
      filtered_residual_norms.reserve(residual_norms.size());
      double filtered_weight_sum = 0.0;
      double filtered_residual_sum = 0.0;
      for (size_t target_index = 0U; target_index < target_w.size(); ++target_index) {
        size_t nearest_source_index = 0U;
        double nearest_source_distance_sq = std::numeric_limits<double>::infinity();
        for (size_t source_index = 0U; source_index < source_w.size(); ++source_index) {
          const double distance_sq = (target_w[target_index] - source_w[source_index]).squaredNorm();
          if (distance_sq < nearest_source_distance_sq) {
            nearest_source_distance_sq = distance_sq;
            nearest_source_index = source_index;
          }
        }
        if (nearest_source_index != target_index ||
          nearest_source_distance_sq > bidirectional_max_distance_sq)
        {
          continue;
        }
        filtered_source_w.push_back(source_w[target_index]);
        filtered_target_w.push_back(target_w[target_index]);
        filtered_match_weights.push_back(match_weights[target_index]);
        filtered_residual_norms.push_back(residual_norms[target_index]);
        filtered_weight_sum += match_weights[target_index];
        filtered_residual_sum += match_weights[target_index] * residual_norms[target_index];
      }
      source_w = std::move(filtered_source_w);
      target_w = std::move(filtered_target_w);
      match_weights = std::move(filtered_match_weights);
      residual_norms = std::move(filtered_residual_norms);
      residual_weight_sum = filtered_weight_sum;
      residual_norm_sum = filtered_residual_sum;
    }
    if (source_w.size() < static_cast<size_t>(lidar_min_points_) ||
      residual_weight_sum <= std::numeric_limits<double>::epsilon())
    {
      return correction;
    }
    const double match_ratio = finite_sample_count > 0U
      ? static_cast<double>(source_w.size()) / static_cast<double>(finite_sample_count)
      : 0.0;
    if (match_ratio < gaussian_snapshot_lidar_pose_correction_min_match_ratio_) {
      return correction;
    }
    const double mean_residual_m = residual_norm_sum / residual_weight_sum;
    if (gaussian_snapshot_lidar_pose_correction_max_mean_residual_m_ > 0.0 &&
      mean_residual_m > gaussian_snapshot_lidar_pose_correction_max_mean_residual_m_)
    {
      return correction;
    }
    if (gaussian_snapshot_lidar_pose_correction_min_coverage_tiles_ > 0) {
      Eigen::Vector2d min_xy(
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity());
      Eigen::Vector2d max_xy(
        -std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity());
      for (const auto & point_w : source_w) {
        min_xy.x() = std::min(min_xy.x(), point_w.x());
        min_xy.y() = std::min(min_xy.y(), point_w.y());
        max_xy.x() = std::max(max_xy.x(), point_w.x());
        max_xy.y() = std::max(max_xy.y(), point_w.y());
      }
      const double range_x = max_xy.x() - min_xy.x();
      const double range_y = max_xy.y() - min_xy.y();
      if (range_x <= 1.0e-6 || range_y <= 1.0e-6) {
        return correction;
      }
      const int cols = gaussian_snapshot_lidar_pose_correction_coverage_grid_cols_;
      const int rows = gaussian_snapshot_lidar_pose_correction_coverage_grid_rows_;
      std::vector<bool> occupied(static_cast<size_t>(cols * rows), false);
      for (const auto & point_w : source_w) {
        const int col = std::clamp(
          static_cast<int>(std::floor((point_w.x() - min_xy.x()) / range_x * cols)),
          0,
          cols - 1);
        const int row = std::clamp(
          static_cast<int>(std::floor((point_w.y() - min_xy.y()) / range_y * rows)),
          0,
          rows - 1);
        occupied[static_cast<size_t>(row * cols + col)] = true;
      }
      const int occupied_tiles = static_cast<int>(
        std::count(occupied.begin(), occupied.end(), true));
      if (occupied_tiles < gaussian_snapshot_lidar_pose_correction_min_coverage_tiles_) {
        return correction;
      }
    }

    Eigen::Vector3d source_centroid = Eigen::Vector3d::Zero();
    Eigen::Vector3d target_centroid = Eigen::Vector3d::Zero();
    for (size_t index = 0U; index < source_w.size(); ++index) {
      source_centroid += match_weights[index] * source_w[index];
      target_centroid += match_weights[index] * target_w[index];
    }
    source_centroid /= residual_weight_sum;
    target_centroid /= residual_weight_sum;

    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (size_t index = 0U; index < source_w.size(); ++index) {
      covariance += match_weights[index] *
        (target_w[index] - target_centroid) * (source_w[index] - source_centroid).transpose();
    }
    const Eigen::JacobiSVD<Eigen::Matrix3d> svd(
      covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
    if (svd.matrixU().cols() != 3 || svd.matrixV().cols() != 3) {
      return correction;
    }
    Eigen::Matrix3d u = svd.matrixU();
    const Eigen::Matrix3d v = svd.matrixV();
    Eigen::Matrix3d rotation = u * v.transpose();
    if (rotation.determinant() < 0.0) {
      u.col(2) *= -1.0;
      rotation = u * v.transpose();
    }
    if (!rotation.allFinite()) {
      return correction;
    }

    Eigen::Quaterniond step_q(rotation);
    step_q.normalize();
    Eigen::AngleAxisd step_aa(step_q);
    if (step_aa.angle() > gaussian_snapshot_lidar_pose_correction_max_rotation_rad_) {
      step_q = Eigen::Quaterniond(
        Eigen::AngleAxisd(
        gaussian_snapshot_lidar_pose_correction_max_rotation_rad_,
        step_aa.axis())).normalized();
      rotation = step_q.toRotationMatrix();
    }
    const Eigen::Vector3d correction_translation = target_centroid - rotation * source_centroid;
    Eigen::Vector3d step_p =
      (rotation * predicted_pose.p_w_i + correction_translation) - predicted_pose.p_w_i;
    step_p *= gaussian_snapshot_lidar_pose_correction_gain_;
    const double step_p_norm = step_p.norm();
    if (step_p_norm > gaussian_snapshot_lidar_pose_correction_max_translation_m_) {
      step_p *= gaussian_snapshot_lidar_pose_correction_max_translation_m_ / step_p_norm;
    }
    if (gaussian_snapshot_lidar_pose_correction_gain_ < 1.0) {
      Eigen::AngleAxisd gained_aa(step_q);
      if (gained_aa.angle() > 1.0e-12) {
        step_q = Eigen::Quaterniond(
          Eigen::AngleAxisd(
          gaussian_snapshot_lidar_pose_correction_gain_ * gained_aa.angle(),
          gained_aa.axis())).normalized();
      }
    }
    if (!step_p.allFinite() || !step_q.coeffs().allFinite()) {
      return correction;
    }

    correction.applied = true;
    correction.matched_points = source_w.size();
    correction.mean_residual_m = mean_residual_m;
    correction.delta_p_w = step_p;
    correction.delta_q = step_q;
    return correction;
  }

  void cache_relative_motion_pose(
    std::deque<gaussian_lic_tracking::TrajectoryPose> & history,
    const gaussian_lic_tracking::TrajectoryPose & pose)
  {
    if (!pose.p_w_i.allFinite() || !pose.q_w_i.coeffs().allFinite() ||
      pose.q_w_i.norm() <= std::numeric_limits<double>::epsilon())
    {
      return;
    }
    if (!history.empty() && pose.stamp_ns <= history.back().stamp_ns)
    {
      return;
    }
    history.push_back(pose);
    const int64_t max_history_ns = static_cast<int64_t>(
      std::max(
        sliding_window_multihop_relative_translation_max_dt_s_,
        sliding_window_max_state_gap_s_) *
      static_cast<double>(gaussian_lic_tracking::kNanosecondsPerSecond));
    while (!history.empty() &&
      history.front().stamp_ns + max_history_ns < pose.stamp_ns)
    {
      history.pop_front();
    }
    const size_t max_history_size = std::max<size_t>(
      static_cast<size_t>(sliding_window_max_states_) + 2U, 4U);
    while (history.size() > max_history_size) {
      history.pop_front();
    }
  }

  std::optional<gaussian_lic_tracking::TrajectoryPose> find_relative_motion_pose(
    const int64_t stamp_ns) const
  {
    const auto it = std::find_if(
      relative_motion_pose_history_.begin(), relative_motion_pose_history_.end(),
      [stamp_ns](const gaussian_lic_tracking::TrajectoryPose & pose) {
        return pose.stamp_ns == stamp_ns;
      });
    if (it == relative_motion_pose_history_.end()) {
      return std::nullopt;
    }
    return *it;
  }

  static bool clamp_motion_target_norm(Eigen::Vector3d & vector, const double max_norm)
  {
    if (!std::isfinite(max_norm) || max_norm <= 0.0) {
      return false;
    }
    const double norm = vector.norm();
    if (!std::isfinite(norm) || norm <= max_norm) {
      return false;
    }
    vector *= max_norm / norm;
    return true;
  }

  void record_relative_motion_target_norms(
    const Eigen::Vector3d & rotation_rate_delta,
    const Eigen::Vector3d & position_rate_delta,
    const Eigen::Vector3d & velocity_acceleration_delta)
  {
    last_sliding_window_smoothness_motion_target_rotation_rate_delta_norm_ =
      rotation_rate_delta.norm();
    last_sliding_window_smoothness_motion_target_position_rate_delta_norm_ =
      position_rate_delta.norm();
    last_sliding_window_smoothness_motion_target_velocity_acceleration_delta_norm_ =
      velocity_acceleration_delta.norm();
    sliding_window_smoothness_motion_target_max_rotation_rate_delta_norm_ =
      std::max(
      sliding_window_smoothness_motion_target_max_rotation_rate_delta_norm_,
      last_sliding_window_smoothness_motion_target_rotation_rate_delta_norm_);
    sliding_window_smoothness_motion_target_max_position_rate_delta_norm_ =
      std::max(
      sliding_window_smoothness_motion_target_max_position_rate_delta_norm_,
      last_sliding_window_smoothness_motion_target_position_rate_delta_norm_);
    sliding_window_smoothness_motion_target_max_velocity_acceleration_delta_norm_ =
      std::max(
      sliding_window_smoothness_motion_target_max_velocity_acceleration_delta_norm_,
      last_sliding_window_smoothness_motion_target_velocity_acceleration_delta_norm_);
  }

  void update_motion_target_support_history(
    const size_t visual_factor_count,
    const size_t se3_photometric_factor_count)
  {
    if (sliding_window_smoothness_motion_target_recent_window_ <= 0) {
      return;
    }
    motion_target_support_history_.emplace_back(visual_factor_count, se3_photometric_factor_count);
    const size_t max_size = static_cast<size_t>(sliding_window_smoothness_motion_target_recent_window_);
    while (motion_target_support_history_.size() > max_size) {
      motion_target_support_history_.pop_front();
    }
    last_sliding_window_smoothness_motion_target_recent_visual_factors_ = 0;
    last_sliding_window_smoothness_motion_target_recent_se3_photometric_factors_ = 0;
    for (const auto & counts : motion_target_support_history_) {
      last_sliding_window_smoothness_motion_target_recent_visual_factors_ += counts.first;
      last_sliding_window_smoothness_motion_target_recent_se3_photometric_factors_ += counts.second;
    }
  }

  bool motion_target_recent_support_is_healthy() const
  {
    if (sliding_window_smoothness_motion_target_recent_window_ <= 0) {
      return true;
    }
    const size_t required_window =
      static_cast<size_t>(sliding_window_smoothness_motion_target_recent_window_);
    if (motion_target_support_history_.size() < required_window) {
      return false;
    }
    if (last_sliding_window_smoothness_motion_target_recent_visual_factors_ <
      static_cast<uint64_t>(sliding_window_smoothness_motion_target_min_recent_visual_factors_))
    {
      return false;
    }
    if (last_sliding_window_smoothness_motion_target_recent_se3_photometric_factors_ <
      static_cast<uint64_t>(
        sliding_window_smoothness_motion_target_min_recent_se3_photometric_factors_))
    {
      return false;
    }
    return true;
  }

  void apply_relative_motion_smoothness_targets(
    gaussian_lic_tracking::SlidingWindowTrajectorySmoothnessFactor & factor,
    const gaussian_lic_tracking::TrajectoryPose & next_pose,
    const size_t visual_factor_count,
    const size_t se3_photometric_factor_count)
  {
    if (!sliding_window_smoothness_use_motion_targets_) {
      return;
    }
    if (sliding_window_smoothness_motion_target_start_after_s_ > 0.0 &&
      sliding_window_start_stamp_ns_.has_value())
    {
      const double elapsed_s =
        static_cast<double>(next_pose.stamp_ns - sliding_window_start_stamp_ns_.value()) /
        static_cast<double>(gaussian_lic_tracking::kNanosecondsPerSecond);
      if (elapsed_s < sliding_window_smoothness_motion_target_start_after_s_) {
        ++sliding_window_smoothness_motion_target_warmup_skip_count_;
        return;
      }
    }
    if (visual_factor_count <
      static_cast<size_t>(sliding_window_smoothness_motion_target_min_visual_factors_) ||
      se3_photometric_factor_count <
      static_cast<size_t>(
        sliding_window_smoothness_motion_target_min_se3_photometric_factors_))
    {
      ++sliding_window_smoothness_motion_target_support_skip_count_;
      return;
    }
    if (!motion_target_recent_support_is_healthy()) {
      ++sliding_window_smoothness_motion_target_recent_support_skip_count_;
      return;
    }
    const auto previous_pose = find_relative_motion_pose(factor.previous_stamp_ns);
    const auto current_pose = find_relative_motion_pose(factor.current_stamp_ns);
    if (!previous_pose.has_value() || !current_pose.has_value() ||
      next_pose.stamp_ns != factor.next_stamp_ns)
    {
      ++sliding_window_smoothness_motion_target_history_miss_count_;
      return;
    }
    const double previous_dt_s =
      static_cast<double>(current_pose->stamp_ns - previous_pose->stamp_ns) /
      static_cast<double>(gaussian_lic_tracking::kNanosecondsPerSecond);
    const double next_dt_s =
      static_cast<double>(next_pose.stamp_ns - current_pose->stamp_ns) /
      static_cast<double>(gaussian_lic_tracking::kNanosecondsPerSecond);
    if (previous_dt_s <= 0.0 || next_dt_s <= 0.0) {
      ++sliding_window_smoothness_motion_target_invalid_count_;
      return;
    }
    const auto rotation_rate = [](const Eigen::Quaterniond & from,
        const Eigen::Quaterniond & to,
        const double dt_s) {
        return gaussian_lic_tracking::spline::quaternion_log(
          from.normalized().inverse() * to.normalized()) / dt_s;
      };
    factor.target_rotation_rate_delta =
      rotation_rate(current_pose->q_w_i, next_pose.q_w_i, next_dt_s) -
      rotation_rate(previous_pose->q_w_i, current_pose->q_w_i, previous_dt_s);
    factor.target_position_rate_delta =
      (next_pose.p_w_i - current_pose->p_w_i) / next_dt_s -
      (current_pose->p_w_i - previous_pose->p_w_i) / previous_dt_s;
    factor.target_velocity_acceleration_delta =
      (next_pose.v_w_i - current_pose->v_w_i) / next_dt_s -
      (current_pose->v_w_i - previous_pose->v_w_i) / previous_dt_s;
    if (!factor.target_rotation_rate_delta.allFinite() ||
      !factor.target_position_rate_delta.allFinite() ||
      !factor.target_velocity_acceleration_delta.allFinite())
    {
      factor.target_rotation_rate_delta.setZero();
      factor.target_position_rate_delta.setZero();
      factor.target_velocity_acceleration_delta.setZero();
      ++sliding_window_smoothness_motion_target_invalid_count_;
      return;
    }
    const bool clamped =
      clamp_motion_target_norm(
      factor.target_rotation_rate_delta,
      sliding_window_smoothness_motion_target_max_rotation_rate_delta_radps_) |
      clamp_motion_target_norm(
      factor.target_position_rate_delta,
      sliding_window_smoothness_motion_target_max_position_rate_delta_mps_) |
      clamp_motion_target_norm(
      factor.target_velocity_acceleration_delta,
      sliding_window_smoothness_motion_target_max_velocity_acceleration_delta_mps2_);
    if (clamped) {
      ++sliding_window_smoothness_motion_target_clamp_count_;
    }
    ++sliding_window_smoothness_motion_target_applied_count_;
    record_relative_motion_target_norms(
      factor.target_rotation_rate_delta,
      factor.target_position_rate_delta,
      factor.target_velocity_acceleration_delta);
  }

  gaussian_lic_tracking::TrajectoryPose update_sliding_window(
    const gaussian_lic_tracking::TrajectoryPose & input_pose,
    const std::vector<gaussian_lic_tracking::SlidingWindowPointToPointFactor> & point_factors,
    const std::vector<gaussian_lic_tracking::SlidingWindowPointToPlaneFactor> & plane_factors,
    const std::vector<gaussian_lic_tracking::SlidingWindowPointToLineFactor> & line_factors,
    const std::vector<gaussian_lic_tracking::SlidingWindowVisualAlignmentFactor> & visual_factors,
    const std::vector<gaussian_lic_tracking::SlidingWindowSe3PhotometricFactor> & se3_photometric_factors,
    const std::vector<gaussian_lic_tracking::SlidingWindowRelativeTranslationFactor> &
      relative_translation_factors,
    const std::vector<gaussian_lic_tracking::SlidingWindowRelativeDistanceFactor> &
      relative_distance_factors)
  {
    gaussian_lic_tracking::TrajectoryPose output_pose = input_pose;
    gaussian_lic_tracking::ImuState imu_state;
    if (!imu_propagator_.query_state(input_pose.stamp_ns, imu_state)) {
      if (!imu_propagator_.initialized()) {
        return output_pose;
      }
      imu_state = imu_propagator_.state();
    }
    update_sliding_window_effective_config();

    gaussian_lic_tracking::SlidingWindowState state;
    state.stamp_ns = input_pose.stamp_ns;
    state.p_w_i = input_pose.p_w_i;
    state.q_w_i = input_pose.q_w_i;
    state.v_w_i = imu_state.v_w_i;
    state.gyro_bias = sliding_window_bias_.gyro;
    state.accel_bias = sliding_window_bias_.accel;
    state.fixed = !has_sliding_window_state_;
    if (!sliding_window_start_stamp_ns_.has_value()) {
      sliding_window_start_stamp_ns_ = input_pose.stamp_ns;
    }
    sliding_window_optimizer_.add_or_update_state(state);

    gaussian_lic_tracking::SlidingWindowPosePrior prior;
    prior.stamp_ns = input_pose.stamp_ns;
    prior.p_w_i = input_pose.p_w_i;
    prior.q_w_i = input_pose.q_w_i;
    prior.translation_weight = sliding_window_pose_translation_weight_;
    prior.rotation_weight = sliding_window_pose_rotation_weight_;
    sliding_window_optimizer_.add_pose_prior(prior);
    bool window_factor_added = false;
    bool external_feedback_factor_added = false;
    size_t visual_feedback_factors_added = 0;
    if (sliding_window_imu_velocity_prior_weight_ > 0.0 ||
      sliding_window_gyro_bias_prior_weight_ > 0.0 ||
      sliding_window_accel_bias_prior_weight_ > 0.0)
    {
      gaussian_lic_tracking::SlidingWindowStatePrior insertion_prior;
      insertion_prior.stamp_ns = input_pose.stamp_ns;
      insertion_prior.p_w_i = input_pose.p_w_i;
      insertion_prior.q_w_i = input_pose.q_w_i;
      insertion_prior.v_w_i = imu_state.v_w_i;
      insertion_prior.gyro_bias = sliding_window_bias_.gyro;
      insertion_prior.accel_bias = sliding_window_bias_.accel;
      insertion_prior.rotation_weight = 0.0;
      insertion_prior.velocity_weight = sliding_window_imu_velocity_prior_weight_;
      insertion_prior.position_weight = 0.0;
      insertion_prior.gyro_bias_weight = sliding_window_gyro_bias_prior_weight_;
      insertion_prior.accel_bias_weight = sliding_window_accel_bias_prior_weight_;
      sliding_window_optimizer_.add_state_prior(insertion_prior);
      window_factor_added = true;
    }
    if (enable_external_odometry_prior_) {
      const auto external_prior = select_external_odometry_prior(input_pose.stamp_ns);
      if (external_prior.has_value()) {
        gaussian_lic_tracking::SlidingWindowPosePrior odometry_prior;
        odometry_prior.stamp_ns = input_pose.stamp_ns;
        odometry_prior.p_w_i = external_prior->p_w_i;
        odometry_prior.q_w_i = external_prior->q_w_i;
        odometry_prior.translation_weight = external_odometry_prior_translation_weight_;
        odometry_prior.rotation_weight = external_odometry_prior_rotation_weight_;
        try {
          sliding_window_optimizer_.add_pose_prior(odometry_prior);
          ++external_odometry_prior_matches_;
          window_factor_added = true;
          external_feedback_factor_added = true;
        } catch (const std::exception & ex) {
          ++external_odometry_prior_invalid_messages_;
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "external odometry pose prior skipped: %s", ex.what());
        }
      } else {
        ++external_odometry_prior_misses_;
      }
    }
    for (const auto & point_factor : point_factors) {
      try {
        sliding_window_optimizer_.add_point_to_point_factor(point_factor);
        window_factor_added = true;
        external_feedback_factor_added = true;
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
        external_feedback_factor_added = true;
      } catch (const std::exception & ex) {
        ++sliding_window_plane_factor_skip_count_;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "sliding window plane factor skipped: %s", ex.what());
      }
    }
    for (const auto & line_factor : line_factors) {
      try {
        sliding_window_optimizer_.add_point_to_line_factor(line_factor);
        window_factor_added = true;
        external_feedback_factor_added = true;
      } catch (const std::exception & ex) {
        ++sliding_window_plane_factor_skip_count_;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "sliding window line factor skipped: %s", ex.what());
      }
    }
    for (const auto & visual_factor : visual_factors) {
      try {
        sliding_window_optimizer_.add_visual_alignment_factor(visual_factor);
        ++sliding_window_total_visual_factors_;
        ++visual_feedback_factors_added;
        window_factor_added = true;
        external_feedback_factor_added = true;
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
        ++sliding_window_total_se3_photometric_factors_;
        ++visual_feedback_factors_added;
        window_factor_added = true;
        external_feedback_factor_added = true;
      } catch (const std::exception & ex) {
        ++sliding_window_se3_photometric_factor_skip_count_;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "sliding window SE3 photometric factor skipped: %s", ex.what());
      }
    }
    for (const auto & relative_factor : relative_translation_factors) {
      try {
        sliding_window_optimizer_.add_relative_translation_factor(relative_factor);
        window_factor_added = true;
      } catch (const std::exception & ex) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "sliding window relative translation factor skipped: %s", ex.what());
      }
    }
    for (const auto & relative_factor : relative_distance_factors) {
      try {
        sliding_window_optimizer_.add_relative_distance_factor(relative_factor);
        window_factor_added = true;
      } catch (const std::exception & ex) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "sliding window relative distance factor skipped: %s", ex.what());
      }
    }
    update_motion_target_support_history(visual_factors.size(), se3_photometric_factors.size());
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
      factor.position_velocity_consistency_weight =
        sliding_window_smoothness_position_velocity_weight_;
      factor.gyro_bias_rate_weight = sliding_window_smoothness_bias_weight_;
      factor.accel_bias_rate_weight = sliding_window_smoothness_bias_weight_;
      apply_relative_motion_smoothness_targets(
        factor, input_pose, visual_factors.size(), se3_photometric_factors.size());
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
      gaussian_lic_tracking::ImuPreintegrator preintegration;
      if (!prepare_sliding_window_imu_preintegration(
          last_sliding_window_stamp_ns_, input_pose.stamp_ns, preintegration))
      {
        ++sliding_window_imu_factor_skip_count_;
      } else {
        gaussian_lic_tracking::SlidingWindowImuFactor factor;
        factor.from_stamp_ns = last_sliding_window_stamp_ns_;
        factor.to_stamp_ns = input_pose.stamp_ns;
        factor.preintegration = preintegration;
        factor.gravity_w = imu_propagator_.gravity_w();
        factor.weight = sliding_window_imu_weight_;
        factor.rotation_weight = sliding_window_imu_rotation_weight_;
        factor.velocity_weight = sliding_window_imu_velocity_weight_;
        factor.position_weight = sliding_window_imu_position_weight_;
        factor.bias_weight = sliding_window_bias_weight_;
        factor.gyro_bias_weight = sliding_window_gyro_bias_weight_;
        factor.accel_bias_weight = sliding_window_accel_bias_weight_;
        factor.bias_random_walk_reference_dt_s = sliding_window_bias_random_walk_reference_dt_s_;
        factor.gyro_bias_random_walk_sigma = sliding_window_gyro_bias_random_walk_sigma_;
        factor.accel_bias_random_walk_sigma = sliding_window_accel_bias_random_walk_sigma_;
        try {
          sliding_window_optimizer_.add_imu_factor(factor);
          ++sliding_window_total_imu_factors_;
          sliding_window_total_imu_preintegration_samples_ +=
            static_cast<uint64_t>(preintegration.samples().size());
          sliding_window_total_imu_preintegration_dt_s_ += preintegration.delta_t_s();
          window_factor_added = true;
        } catch (const std::exception & ex) {
          ++sliding_window_imu_factor_skip_count_;
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "sliding window IMU factor skipped: %s", ex.what());
        }
      }
    }

    ++sliding_window_frame_count_;
    const bool optimize_this_frame =
      sliding_window_optimize_every_n_frames_ <= 1 ||
      sliding_window_frame_count_ % static_cast<uint64_t>(sliding_window_optimize_every_n_frames_) == 0U;
    if (has_sliding_window_state_ && window_factor_added && optimize_this_frame) {
      try {
        const auto optimization_start = std::chrono::steady_clock::now();
        const auto summary = sliding_window_optimizer_.optimize();
        const auto optimization_end = std::chrono::steady_clock::now();
        last_sliding_window_optimization_duration_ms_ =
          std::chrono::duration<double, std::milli>(optimization_end - optimization_start).count();
        last_sliding_window_summary_ = summary;
        has_last_sliding_window_summary_ = true;
        gaussian_lic_tracking::SlidingWindowState optimized;
        bool sync_window_controls = false;
        if (external_feedback_factor_added &&
          sliding_window_optimizer_.get_state(input_pose.stamp_ns, optimized))
        {
          if (!valid_sliding_window_state(optimized)) {
            ++sliding_window_invalid_optimized_states_;
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 2000,
              "sliding window optimized state rejected before odometry/IMU feedback");
          } else {
            auto applied = optimized;
            const Eigen::Vector3d raw_translation_delta = optimized.p_w_i - input_pose.p_w_i;
            const double raw_translation_delta_m = raw_translation_delta.norm();
            if (sliding_window_max_feedback_translation_m_ > 0.0 &&
              raw_translation_delta_m > sliding_window_max_feedback_translation_m_)
            {
              applied.p_w_i = input_pose.p_w_i +
                raw_translation_delta * (sliding_window_max_feedback_translation_m_ / raw_translation_delta_m);
            }
            Eigen::Quaterniond feedback_delta_q =
              (input_pose.q_w_i.normalized().inverse() * optimized.q_w_i.normalized()).normalized();
            if (feedback_delta_q.w() < 0.0) {
              feedback_delta_q.coeffs() *= -1.0;
            }
            const double raw_rotation_delta_rad =
              2.0 * std::atan2(feedback_delta_q.vec().norm(), feedback_delta_q.w());
            if (sliding_window_max_feedback_rotation_rad_ > 0.0 &&
              raw_rotation_delta_rad > sliding_window_max_feedback_rotation_rad_)
            {
              const double ratio = sliding_window_max_feedback_rotation_rad_ / raw_rotation_delta_rad;
              const Eigen::Quaterniond limited_delta =
                Eigen::Quaterniond::Identity().slerp(ratio, feedback_delta_q).normalized();
              applied.q_w_i = (input_pose.q_w_i.normalized() * limited_delta).normalized();
            }
            const Eigen::Vector3d raw_velocity_delta = optimized.v_w_i - imu_state.v_w_i;
            const double raw_velocity_delta_mps = raw_velocity_delta.norm();
            if (sliding_window_max_feedback_velocity_mps_ > 0.0 &&
              raw_velocity_delta_mps > sliding_window_max_feedback_velocity_mps_)
            {
              applied.v_w_i = imu_state.v_w_i +
                raw_velocity_delta * (sliding_window_max_feedback_velocity_mps_ / raw_velocity_delta_mps);
            }
            clamp_vector_norm(applied.v_w_i, sliding_window_max_feedback_velocity_norm_mps_);
            const bool visual_feedback_bias_pose_only =
              sliding_window_bias_feedback_ownership_ ==
              SlidingWindowBiasFeedbackOwnership::kPoseOnlyForVisualFeedback &&
              visual_feedback_factors_added > 0U;
            const bool hold_bias_feedback =
              sliding_window_min_bias_feedback_visual_factors_ > 0 &&
              visual_feedback_factors_added <
              static_cast<size_t>(sliding_window_min_bias_feedback_visual_factors_);
            const bool preserve_bias_feedback =
              hold_bias_feedback || visual_feedback_bias_pose_only;
            if (preserve_bias_feedback) {
              applied.gyro_bias = sliding_window_bias_.gyro;
              applied.accel_bias = sliding_window_bias_.accel;
              ++sliding_window_bias_feedback_hold_count_;
              if (visual_feedback_bias_pose_only) {
                ++sliding_window_bias_feedback_ownership_hold_count_;
              }
            } else {
              clamp_vector_step(
                applied.gyro_bias, sliding_window_bias_.gyro,
                sliding_window_max_feedback_gyro_bias_step_);
              clamp_vector_step(
                applied.accel_bias, sliding_window_bias_.accel,
                sliding_window_max_feedback_accel_bias_step_);
              clamp_vector_norm(applied.gyro_bias, sliding_window_max_feedback_gyro_bias_norm_);
              clamp_vector_norm(applied.accel_bias, sliding_window_max_feedback_accel_bias_norm_);
            }
            if (!valid_sliding_window_state(applied)) {
              ++sliding_window_invalid_optimized_states_;
              RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "sliding window limited feedback rejected because it became invalid");
            } else {
              const Eigen::Quaterniond applied_delta_q =
                (input_pose.q_w_i.normalized().inverse() * applied.q_w_i.normalized()).normalized();
              const double feedback_translation_delta_m =
                (applied.p_w_i - input_pose.p_w_i).norm();
              const double feedback_rotation_delta_rad =
                2.0 * std::atan2(applied_delta_q.vec().norm(), std::abs(applied_delta_q.w()));
              const double feedback_velocity_delta_mps =
                (applied.v_w_i - imu_state.v_w_i).norm();
              sync_window_controls = true;
              last_sliding_window_feedback_translation_delta_m_ = feedback_translation_delta_m;
              last_sliding_window_feedback_rotation_delta_rad_ = feedback_rotation_delta_rad;
              last_sliding_window_feedback_velocity_delta_mps_ = feedback_velocity_delta_mps;
              last_sliding_window_feedback_stamp_ns_ = applied.stamp_ns;
              ++sliding_window_feedback_update_count_;
              output_pose.p_w_i = applied.p_w_i;
              output_pose.q_w_i = applied.q_w_i;
              output_pose.v_w_i = applied.v_w_i;
              sliding_window_bias_.gyro = applied.gyro_bias;
              sliding_window_bias_.accel = applied.accel_bias;
              gaussian_lic_tracking::ImuState corrected_state;
              corrected_state.stamp_ns = applied.stamp_ns;
              corrected_state.p_w_i = applied.p_w_i;
              corrected_state.q_w_i = applied.q_w_i;
              corrected_state.v_w_i = applied.v_w_i;
              corrected_state.gyro_bias = applied.gyro_bias;
              corrected_state.accel_bias = applied.accel_bias;
              bool imu_reanchored = false;
              if (!imu_propagator_.initialized()) {
                imu_propagator_.reset(corrected_state);
                imu_reanchored = true;
              } else if (imu_propagator_.rebase_from_state(corrected_state)) {
                imu_reanchored = true;
              } else if (imu_propagator_.state().stamp_ns <= applied.stamp_ns) {
                imu_propagator_.reset(corrected_state);
                imu_reanchored = true;
              } else {
                RCLCPP_WARN_THROTTLE(
                  get_logger(), *get_clock(), 2000,
                  "sliding window feedback could not rebase delayed IMU history at %" PRId64,
                  applied.stamp_ns);
              }
              if (imu_reanchored) {
                ++num_sliding_window_imu_reanchors_;
              }
            }
          }
        }
        if (sync_window_controls) {
          sync_optimized_trajectory_controls();
        }
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
    } else if (has_sliding_window_state_ && window_factor_added) {
      ++sliding_window_optimization_skip_count_;
    }

    if (has_sliding_window_state_) {
      previous_sliding_window_stamp_ns_ = last_sliding_window_stamp_ns_;
    }
    has_sliding_window_state_ = true;
    last_sliding_window_stamp_ns_ = input_pose.stamp_ns;
    reset_sliding_window_preintegrator_from_history(input_pose.stamp_ns);
    return output_pose;
  }

  void reset_sliding_window_preintegrator_from_history(const int64_t start_stamp_ns)
  {
    sliding_window_preintegrator_.reset(start_stamp_ns, sliding_window_bias_);
    sliding_window_preintegrator_initialized_ = true;
    for (const auto & measurement : imu_propagator_.measurements_after(start_stamp_ns)) {
      try {
        sliding_window_preintegrator_.add_measurement(
          measurement.stamp_ns,
          measurement.angular_velocity_rad_s,
          measurement.linear_acceleration_m_s2);
      } catch (const std::exception & ex) {
        ++sliding_window_imu_time_gap_skip_count_;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "sliding window IMU history replay stopped at %" PRId64 ": %s",
          measurement.stamp_ns,
          ex.what());
        return;
      }
    }
  }

  std::optional<ExternalPosePrior> select_external_odometry_prior(const int64_t stamp_ns) const
  {
    if (!enable_external_odometry_prior_ || external_odometry_priors_.empty()) {
      return std::nullopt;
    }
    const ExternalPosePrior * best_prior = nullptr;
    int64_t best_abs_dt_ns = std::numeric_limits<int64_t>::max();
    for (const auto & prior : external_odometry_priors_) {
      const int64_t dt_ns = prior.stamp_ns - stamp_ns;
      const int64_t abs_dt_ns = dt_ns < 0 ? -dt_ns : dt_ns;
      if (abs_dt_ns < best_abs_dt_ns) {
        best_abs_dt_ns = abs_dt_ns;
        best_prior = &prior;
      }
    }
    if (best_prior == nullptr || best_abs_dt_ns > external_odometry_prior_max_dt_ns_) {
      return std::nullopt;
    }
    return *best_prior;
  }

  bool prepare_sliding_window_imu_preintegration(
    const int64_t from_stamp_ns,
    const int64_t to_stamp_ns,
    gaussian_lic_tracking::ImuPreintegrator & preintegration)
  {
    preintegration = sliding_window_preintegrator_;
    double extrapolated_dt_s = 0.0;
    if (preintegration.start_stamp_ns() != from_stamp_ns) {
      ++sliding_window_imu_time_gap_skip_count_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "sliding window IMU factor skipped: preintegration span [%" PRId64 ", %" PRId64
        "] does not fit factor span [%" PRId64 ", %" PRId64 "]",
        preintegration.start_stamp_ns(),
        preintegration.end_stamp_ns(),
        from_stamp_ns,
        to_stamp_ns);
      return false;
    }
    if (preintegration.end_stamp_ns() > to_stamp_ns) {
      try {
        preintegration = preintegration.truncated(to_stamp_ns);
      } catch (const std::exception & ex) {
        ++sliding_window_imu_time_gap_skip_count_;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "sliding window IMU factor skipped while truncating preintegration: %s", ex.what());
        return false;
      }
    }
    if (preintegration.end_stamp_ns() < to_stamp_ns) {
      const int64_t gap_ns = to_stamp_ns - preintegration.end_stamp_ns();
      const double gap_s = static_cast<double>(gap_ns) /
        static_cast<double>(gaussian_lic_tracking::kNanosecondsPerSecond);
      if (gap_s > sliding_window_imu_max_extrapolation_s_ || preintegration.samples().empty()) {
        ++sliding_window_imu_time_gap_skip_count_;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "sliding window IMU factor skipped: preintegration end gap %.6fs exceeds %.6fs",
          gap_s,
          sliding_window_imu_max_extrapolation_s_);
        return false;
      }
      const auto last_sample = preintegration.samples().back();
      try {
        preintegration.add_measurement(
          to_stamp_ns,
          last_sample.angular_velocity_rad_s,
          last_sample.linear_acceleration_m_s2);
        extrapolated_dt_s = gap_s;
      } catch (const std::exception & ex) {
        ++sliding_window_imu_time_gap_skip_count_;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "sliding window IMU factor skipped while extending preintegration: %s", ex.what());
        return false;
      }
    }
    last_sliding_window_imu_preintegration_extrapolated_dt_s_ = extrapolated_dt_s;
    update_last_sliding_window_imu_preintegration_status(preintegration);
    return preintegration.start_stamp_ns() == from_stamp_ns &&
      preintegration.end_stamp_ns() == to_stamp_ns &&
      preintegration.delta_t_s() > 0.0;
  }

  void update_last_sliding_window_imu_preintegration_status(
    const gaussian_lic_tracking::ImuPreintegrator & preintegration)
  {
    last_sliding_window_imu_preintegration_samples_ =
      static_cast<uint64_t>(preintegration.sample_count());
    last_sliding_window_imu_preintegration_dt_s_ = preintegration.delta_t_s();
    last_sliding_window_imu_preintegration_start_stamp_ns_ = preintegration.start_stamp_ns();
    last_sliding_window_imu_preintegration_end_stamp_ns_ = preintegration.end_stamp_ns();
  }

  void update_sliding_window_effective_config()
  {
    size_t effective_max_states = static_cast<size_t>(sliding_window_max_states_);
    visual_render_backlog_frames_ = 0U;
    if (enable_visual_adaptive_state_retention_ && enable_visual_factor_) {
      if (num_raw_images_ > num_rendered_images_) {
        visual_render_backlog_frames_ = num_raw_images_ - num_rendered_images_;
      }
      const size_t requested_max_states = effective_max_states +
        static_cast<size_t>(visual_render_backlog_frames_) +
        static_cast<size_t>(visual_adaptive_state_retention_margin_states_);
      const size_t capped_max_states = std::min(
        requested_max_states,
        static_cast<size_t>(visual_adaptive_state_retention_max_states_));
      effective_max_states = std::max(effective_max_states, capped_max_states);
    }
    sliding_window_effective_max_states_ = effective_max_states;
    auto config = sliding_window_optimizer_.config();
    if (config.max_states != effective_max_states) {
      config.max_states = effective_max_states;
      sliding_window_optimizer_.set_config(config);
    }
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
      ++trajectory_control_pose_skip_count_;
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

  void sync_guarded_pose_to_sliding_window_state(
    const gaussian_lic_tracking::TrajectoryPose & pose)
  {
    gaussian_lic_tracking::SlidingWindowState guarded_state;
    if (!sliding_window_optimizer_.get_state(pose.stamp_ns, guarded_state)) {
      return;
    }
    guarded_state.p_w_i = pose.p_w_i;
    guarded_state.q_w_i = pose.q_w_i.normalized();
    guarded_state.v_w_i = pose.v_w_i;
    guarded_state.gyro_bias = sliding_window_bias_.gyro;
    guarded_state.accel_bias = sliding_window_bias_.accel;
    try {
      sliding_window_optimizer_.add_or_update_state(guarded_state);
      ++sliding_window_guarded_state_sync_count_;
    } catch (const std::exception & ex) {
      ++sliding_window_invalid_optimized_states_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "guarded sliding-window state sync skipped: %s", ex.what());
    }
  }

  void add_guarded_pose_prior_to_sliding_window(
    const gaussian_lic_tracking::TrajectoryPose & pose)
  {
    if (sliding_window_guarded_pose_prior_translation_weight_ <= 0.0 &&
      sliding_window_guarded_pose_prior_rotation_weight_ <= 0.0)
    {
      return;
    }
    if (!valid_trajectory_pose(pose)) {
      return;
    }
    gaussian_lic_tracking::SlidingWindowPosePrior prior;
    prior.stamp_ns = pose.stamp_ns;
    prior.p_w_i = pose.p_w_i;
    prior.q_w_i = pose.q_w_i.normalized();
    prior.translation_weight = sliding_window_guarded_pose_prior_translation_weight_;
    prior.rotation_weight = sliding_window_guarded_pose_prior_rotation_weight_;
    try {
      sliding_window_optimizer_.add_pose_prior(prior);
      ++sliding_window_guarded_pose_prior_count_;
    } catch (const std::exception & ex) {
      ++sliding_window_invalid_optimized_states_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "guarded sliding-window pose prior skipped: %s", ex.what());
    }
  }

  std::optional<VisualFactorReference> select_visual_factor_reference(
    const int64_t factor_stamp_ns,
    const gaussian_lic_tracking::TrajectoryPose & current_pose) const
  {
    if (enable_visual_factor_time_interpolation_) {
      const auto & states = sliding_window_optimizer_.states();
      const gaussian_lic_tracking::SlidingWindowState * before = nullptr;
      const gaussian_lic_tracking::SlidingWindowState * after = nullptr;
      for (const auto & state : states) {
        if (state.stamp_ns <= factor_stamp_ns) {
          before = &state;
        }
        if (state.stamp_ns >= factor_stamp_ns) {
          after = &state;
          break;
        }
      }

      const int64_t max_delta_ns = max_visual_factor_reference_delta_ns();
      if (before != nullptr && after != nullptr &&
        stamp_delta_ns(before->stamp_ns, factor_stamp_ns) <= max_delta_ns &&
        stamp_delta_ns(after->stamp_ns, factor_stamp_ns) <= max_delta_ns)
      {
        VisualFactorReference reference;
        reference.pose.stamp_ns = factor_stamp_ns;
        if (before->stamp_ns == after->stamp_ns) {
          reference.pose.p_w_i = before->p_w_i;
          reference.pose.q_w_i = before->q_w_i.normalized();
          reference.pose.v_w_i = before->v_w_i;
          reference.support_stamp_ns = {before->stamp_ns};
          reference.support_weights = {1.0};
          return reference;
        }

        const double span_ns = static_cast<double>(after->stamp_ns - before->stamp_ns);
        if (std::isfinite(span_ns) && span_ns > 0.0) {
          const double alpha =
            std::clamp(static_cast<double>(factor_stamp_ns - before->stamp_ns) / span_ns, 0.0, 1.0);
          reference.pose.p_w_i = (1.0 - alpha) * before->p_w_i + alpha * after->p_w_i;
          reference.pose.q_w_i = before->q_w_i.normalized().slerp(alpha, after->q_w_i.normalized()).normalized();
          reference.pose.v_w_i = (1.0 - alpha) * before->v_w_i + alpha * after->v_w_i;
          if (alpha <= 1.0e-9) {
            reference.support_stamp_ns = {before->stamp_ns};
            reference.support_weights = {1.0};
          } else if (alpha >= 1.0 - 1.0e-9) {
            reference.support_stamp_ns = {after->stamp_ns};
            reference.support_weights = {1.0};
          } else {
            reference.support_stamp_ns = {before->stamp_ns, after->stamp_ns};
            reference.support_weights = {1.0 - alpha, alpha};
            reference.interpolated = true;
          }
          return reference;
        }
      }

      if (before != nullptr && after == nullptr &&
        current_pose.stamp_ns >= factor_stamp_ns &&
        current_pose.stamp_ns > before->stamp_ns &&
        stamp_delta_ns(before->stamp_ns, factor_stamp_ns) <= max_delta_ns &&
        stamp_delta_ns(current_pose.stamp_ns, factor_stamp_ns) <= max_delta_ns)
      {
        VisualFactorReference reference;
        reference.pose.stamp_ns = factor_stamp_ns;
        const double span_ns = static_cast<double>(current_pose.stamp_ns - before->stamp_ns);
        if (std::isfinite(span_ns) && span_ns > 0.0) {
          const double alpha =
            std::clamp(static_cast<double>(factor_stamp_ns - before->stamp_ns) / span_ns, 0.0, 1.0);
          reference.pose.p_w_i = (1.0 - alpha) * before->p_w_i + alpha * current_pose.p_w_i;
          reference.pose.q_w_i =
            before->q_w_i.normalized().slerp(alpha, current_pose.q_w_i.normalized()).normalized();
          reference.pose.v_w_i = (1.0 - alpha) * before->v_w_i + alpha * current_pose.v_w_i;
          if (alpha <= 1.0e-9) {
            reference.support_stamp_ns = {before->stamp_ns};
            reference.support_weights = {1.0};
          } else if (alpha >= 1.0 - 1.0e-9) {
            reference.support_stamp_ns = {current_pose.stamp_ns};
            reference.support_weights = {1.0};
          } else {
            reference.support_stamp_ns = {before->stamp_ns, current_pose.stamp_ns};
            reference.support_weights = {1.0 - alpha, alpha};
            reference.interpolated = true;
          }
          return reference;
        }
      }
    }

    if (stamp_delta_is_within(factor_stamp_ns, current_pose.stamp_ns, visual_factor_max_dt_ns_)) {
      VisualFactorReference reference;
      reference.pose = current_pose;
      return reference;
    }

    const auto & states = sliding_window_optimizer_.states();
    const gaussian_lic_tracking::SlidingWindowState * best_state = nullptr;
    int64_t best_delta_ns = std::numeric_limits<int64_t>::max();
    for (const auto & state : states) {
      const int64_t delta_ns = stamp_delta_ns(state.stamp_ns, factor_stamp_ns);
      if (delta_ns < best_delta_ns) {
        best_delta_ns = delta_ns;
        best_state = &state;
      }
    }
    if (best_state == nullptr || best_delta_ns > max_visual_factor_reference_delta_ns()) {
      return std::nullopt;
    }

    VisualFactorReference reference;
    reference.pose.stamp_ns = best_state->stamp_ns;
    reference.pose.p_w_i = best_state->p_w_i;
    reference.pose.q_w_i = best_state->q_w_i;
    reference.pose.v_w_i = best_state->v_w_i;
    return reference;
  }

  int64_t max_visual_factor_reference_delta_ns() const
  {
    return std::max<int64_t>(
      visual_factor_max_dt_ns_,
      static_cast<int64_t>(
        std::max(0.0, sliding_window_max_state_gap_s_) *
        static_cast<double>(gaussian_lic_tracking::kNanosecondsPerSecond)));
  }

  int64_t max_visual_depth_delta_ns() const
  {
    return visual_depth_max_dt_ns_ > 0LL ? visual_depth_max_dt_ns_ : visual_factor_max_dt_ns_;
  }

  bool visual_factor_stamp_is_expired(
    const int64_t factor_stamp_ns,
    const int64_t current_stamp_ns) const
  {
    return factor_stamp_ns + max_visual_factor_reference_delta_ns() < current_stamp_ns;
  }

  bool visual_factor_stamp_is_before_active_window(const int64_t factor_stamp_ns) const
  {
    const auto & states = sliding_window_optimizer_.states();
    return !states.empty() && factor_stamp_ns < states.front().stamp_ns;
  }

  bool visual_factor_should_defer_future_reference(
    const int64_t factor_stamp_ns,
    const gaussian_lic_tracking::TrajectoryPose & current_pose) const
  {
    return defer_future_visual_factors_until_active_ &&
           factor_stamp_ns > current_pose.stamp_ns;
  }

  bool se3_photometric_hessian_is_healthy(
    const gaussian_lic_tracking::VisualSe3PhotometricLinearization & linearization) const
  {
    if (!linearization.valid) {
      return false;
    }
    if (linearization.hessian_rank < static_cast<size_t>(se3_photometric_min_hessian_rank_)) {
      return false;
    }
    if (se3_photometric_max_hessian_condition_ > 0.0 &&
      (!std::isfinite(linearization.hessian_condition_number) ||
      linearization.hessian_condition_number <= 0.0 ||
      linearization.hessian_condition_number > se3_photometric_max_hessian_condition_))
    {
      return false;
    }
    return true;
  }

  bool visual_alignment_is_saturated(
    const gaussian_lic_tracking::VisualAlignment & alignment) const
  {
    if (!alignment.valid || visual_alignment_max_shift_px_ <= 0) {
      return false;
    }
    const double limit =
      static_cast<double>(visual_alignment_max_shift_px_) + 0.5 -
      visual_alignment_saturation_margin_px_;
    return std::abs(alignment.subpixel_dx) >= limit ||
           std::abs(alignment.subpixel_dy) >= limit;
  }

  Eigen::Vector2d visual_alignment_component_weights(
    const gaussian_lic_tracking::VisualAlignment & alignment) const
  {
    Eigen::Vector2d weights = Eigen::Vector2d::Ones();
    if (!enable_visual_alignment_saturation_axis_mask_ || !alignment.valid ||
      visual_alignment_max_shift_px_ <= 0)
    {
      return weights;
    }
    const double limit =
      static_cast<double>(visual_alignment_max_shift_px_) + 0.5 -
      visual_alignment_saturation_margin_px_;
    if (std::abs(alignment.subpixel_dx) >= limit) {
      weights.x() = 0.0;
    }
    if (std::abs(alignment.subpixel_dy) >= limit) {
      weights.y() = 0.0;
    }
    return weights;
  }

  size_t visual_alignment_masked_axis_count(const Eigen::Vector2d & weights) const
  {
    size_t count = 0U;
    if (weights.x() <= 0.0) {
      ++count;
    }
    if (weights.y() <= 0.0) {
      ++count;
    }
    return count;
  }

  bool visual_alignment_has_active_component(const Eigen::Vector2d & weights) const
  {
    return weights.allFinite() && (weights.x() > 0.0 || weights.y() > 0.0);
  }

  void record_visual_alignment_saturation_axis_mask(const Eigen::Vector2d & weights)
  {
    if (!enable_visual_alignment_saturation_axis_mask_) {
      return;
    }
    const size_t masked_axes = visual_alignment_masked_axis_count(weights);
    if (masked_axes == 0U) {
      return;
    }
    ++visual_alignment_saturation_axis_masked_factors_;
    visual_alignment_saturation_axis_masked_axes_ += static_cast<uint64_t>(masked_axes);
  }

  double visual_alignment_effective_weight(
    const gaussian_lic_tracking::VisualAlignment & alignment) const
  {
    double weight = visual_alignment_window_weight_;
    if (visual_alignment_is_saturated(alignment)) {
      weight *= visual_alignment_saturated_weight_scale_;
    }
    return std::max(weight, 1.0e-9);
  }

  std::optional<gaussian_lic_tracking::VisualAlignment> visual_alignment_for_window_factor() const
  {
    if (!last_visual_alignment_.valid) {
      return std::nullopt;
    }
    if (visual_alignment_factor_source_ == VisualAlignmentFactorSource::kSearch ||
      (visual_alignment_factor_source_ == VisualAlignmentFactorSource::kSaturatedPhotometricStep &&
      !last_visual_alignment_saturated_))
    {
      return last_visual_alignment_;
    }
    return visual_alignment_with_photometric_step(last_visual_alignment_).value_or(last_visual_alignment_);
  }

  std::optional<gaussian_lic_tracking::VisualAlignment> visual_alignment_with_photometric_step(
    const gaussian_lic_tracking::VisualAlignment & base_alignment) const
  {
    if (!last_visual_photometric_linearization_.valid ||
      !last_visual_photometric_linearization_.gauss_newton_step.allFinite())
    {
      return std::nullopt;
    }
    gaussian_lic_tracking::VisualAlignment alignment = base_alignment;
    alignment.subpixel_dx = last_visual_photometric_linearization_.gauss_newton_step.x();
    alignment.subpixel_dy = last_visual_photometric_linearization_.gauss_newton_step.y();
    alignment.dx = static_cast<int>(std::lround(alignment.subpixel_dx));
    alignment.dy = static_cast<int>(std::lround(alignment.subpixel_dy));
    return alignment;
  }

  static bool visual_alignment_photometric_step_agrees(
    const gaussian_lic_tracking::VisualAlignment & search_alignment,
    const gaussian_lic_tracking::VisualAlignment & photometric_alignment)
  {
    const Eigen::Vector2d search{
      search_alignment.subpixel_dx,
      search_alignment.subpixel_dy};
    const Eigen::Vector2d photometric{
      photometric_alignment.subpixel_dx,
      photometric_alignment.subpixel_dy};
    if (!search.allFinite() || !photometric.allFinite()) {
      return false;
    }
    if (search.squaredNorm() <= 1.0e-12 || photometric.squaredNorm() <= 1.0e-12) {
      return false;
    }
    return search.dot(photometric) > 0.0;
  }

  static double se3_photometric_sample_inlier_ratio(const Se3PhotometricSampleBatch & batch)
  {
    return batch.sampled_depth_pixels > 0U
      ? static_cast<double>(batch.accepted_pixels) / static_cast<double>(batch.sampled_depth_pixels)
      : 0.0;
  }

  bool se3_photometric_sample_quality_is_healthy(
    const Se3PhotometricSampleBatch & batch) const
  {
    const double ratio = se3_photometric_sample_inlier_ratio(batch);
    if (ratio < se3_photometric_min_sample_inlier_ratio_) {
      return false;
    }
    if (se3_photometric_max_mean_abs_residual_for_factor_ > 0.0 &&
      (!std::isfinite(batch.mean_abs_residual) ||
      batch.mean_abs_residual > se3_photometric_max_mean_abs_residual_for_factor_))
    {
      return false;
    }
    if (batch.coverage_tiles < static_cast<size_t>(se3_photometric_min_coverage_tiles_)) {
      return false;
    }
    return true;
  }

  std::optional<size_t> select_se3_photometric_pose_correction_index(
    const int64_t stamp_ns) const
  {
    if (!enable_se3_photometric_pose_correction_ ||
      se3_photometric_pose_correction_gain_ <= 0.0 ||
      pending_visual_se3_photometric_factors_.empty())
    {
      return std::nullopt;
    }

    const int64_t max_dt_ns = se3_photometric_pose_correction_max_dt_ns_ > 0LL
      ? se3_photometric_pose_correction_max_dt_ns_
      : visual_factor_max_dt_ns_;
    std::optional<size_t> best_index;
    int64_t best_delta_ns = std::numeric_limits<int64_t>::max();
    for (size_t index = 0; index < pending_visual_se3_photometric_factors_.size(); ++index) {
      const auto & pending = pending_visual_se3_photometric_factors_[index];
      if (last_applied_se3_photometric_pose_correction_stamp_ns_.has_value() &&
        pending.stamp_ns == last_applied_se3_photometric_pose_correction_stamp_ns_.value() &&
        pending.source_id == last_applied_se3_photometric_pose_correction_source_id_)
      {
        continue;
      }
      const int64_t delta_ns = stamp_delta_ns(pending.stamp_ns, stamp_ns);
      if (delta_ns <= max_dt_ns && delta_ns < best_delta_ns) {
        best_index = index;
        best_delta_ns = delta_ns;
      }
    }
    return best_index;
  }

  bool apply_se3_photometric_pose_correction(
    gaussian_lic_tracking::TrajectoryPose & tracking_pose)
  {
    const auto pending_index = select_se3_photometric_pose_correction_index(tracking_pose.stamp_ns);
    if (!pending_index.has_value()) {
      return false;
    }
    const auto pending = pending_visual_se3_photometric_factors_[pending_index.value()];

    Eigen::Matrix<double, 6, 1> body_delta =
      gaussian_lic_tracking::transform_camera_delta_to_body(
      q_i_c_, p_i_c_, pending.linearization.gauss_newton_step);
    if (!body_delta.allFinite()) {
      return false;
    }
    body_delta *= se3_photometric_pose_correction_gain_;

    Eigen::Vector3d rotation_delta = body_delta.template segment<3>(0);
    Eigen::Vector3d translation_delta = body_delta.template segment<3>(3);
    const double raw_translation_m = translation_delta.norm();
    if (se3_photometric_pose_correction_max_translation_m_ > 0.0 &&
      raw_translation_m > se3_photometric_pose_correction_max_translation_m_)
    {
      translation_delta *= se3_photometric_pose_correction_max_translation_m_ / raw_translation_m;
    }
    const double raw_rotation_rad = rotation_delta.norm();
    if (se3_photometric_pose_correction_max_rotation_rad_ > 0.0 &&
      raw_rotation_rad > se3_photometric_pose_correction_max_rotation_rad_)
    {
      rotation_delta *= se3_photometric_pose_correction_max_rotation_rad_ / raw_rotation_rad;
    }

    Eigen::Quaterniond rotation_correction = Eigen::Quaterniond::Identity();
    const double rotation_rad = rotation_delta.norm();
    if (rotation_rad > 1.0e-12) {
      rotation_correction =
        Eigen::Quaterniond(Eigen::AngleAxisd(rotation_rad, rotation_delta / rotation_rad)).normalized();
    }
    const double translation_m = translation_delta.norm();
    if (translation_m <= 0.0 && rotation_rad <= 0.0) {
      return false;
    }

    tracking_pose.p_w_i += translation_delta;
    tracking_pose.q_w_i = (rotation_correction * tracking_pose.q_w_i).normalized();
    pending_visual_se3_photometric_factors_.erase(
      pending_visual_se3_photometric_factors_.begin() +
      static_cast<std::deque<PendingSe3PhotometricFactor>::difference_type>(pending_index.value()));
    ++se3_photometric_pose_correction_count_;
    last_se3_photometric_pose_correction_stamp_delta_ns_ =
      stamp_delta_ns(pending.stamp_ns, tracking_pose.stamp_ns);
    last_se3_photometric_pose_correction_translation_m_ = translation_m;
    last_se3_photometric_pose_correction_rotation_rad_ = rotation_rad;
    last_applied_se3_photometric_pose_correction_stamp_ns_ = pending.stamp_ns;
    last_applied_se3_photometric_pose_correction_source_id_ = pending.source_id;
    return true;
  }

  static uint64_t mix_visual_factor_source_id(uint64_t mixed, const uint64_t value)
  {
    mixed ^= value + 0x9e3779b97f4a7c15ULL +
      (mixed << 6U) + (mixed >> 2U);
    mixed ^= mixed >> 30U;
    mixed *= 0xbf58476d1ce4e5b9ULL;
    mixed ^= mixed >> 27U;
    mixed *= 0x94d049bb133111ebULL;
    mixed ^= mixed >> 31U;
    return mixed == 0U ? 1U : mixed;
  }

  uint64_t visual_factor_source_id(
    const int64_t observed_stamp_ns,
    const int64_t rendered_stamp_ns) const
  {
    const uint64_t mixed = mix_visual_factor_source_id(
      static_cast<uint64_t>(observed_stamp_ns),
      static_cast<uint64_t>(rendered_stamp_ns));
    if (visual_factor_source_id_mode_ == VisualFactorSourceIdMode::kLegacy8Bit) {
      return 1U + (mixed % 254U);
    }
    return mixed;
  }

  uint64_t visual_factor_source_id(
    const gaussian_lic_tracking::VisualFrame & observed,
    const gaussian_lic_tracking::VisualFrame & rendered) const
  {
    if (rendered.has_rendered_feedback_metadata) {
      uint64_t mixed = 0xd6e8feb86659fd93ULL;
      mixed = mix_visual_factor_source_id(mixed, rendered.rendered_feedback_frame_index);
      mixed = mix_visual_factor_source_id(mixed, rendered.rendered_feedback_preview_index);
      mixed = mix_visual_factor_source_id(
        mixed, static_cast<uint64_t>(rendered.rendered_feedback_observed_stamp_ns));
      return mixed;
    }
    return visual_factor_source_id(observed.stamp_ns, rendered.stamp_ns);
  }

  int64_t visual_factor_reference_stamp_ns(
    const gaussian_lic_tracking::VisualFrame & observed,
    const gaussian_lic_tracking::VisualFrame & rendered) const
  {
    if (visual_factor_reference_stamp_mode_ == VisualFactorReferenceStampMode::kRendered) {
      return rendered.stamp_ns;
    }
    if (
      visual_factor_reference_stamp_mode_ == VisualFactorReferenceStampMode::kRenderedSourceImage &&
      rendered.has_rendered_feedback_metadata)
    {
      return rendered.rendered_feedback_observed_stamp_ns;
    }
    if (
      visual_factor_reference_stamp_mode_ == VisualFactorReferenceStampMode::kRenderedSourcePose &&
      rendered.has_rendered_feedback_metadata)
    {
      return rendered.rendered_feedback_pose_stamp_ns;
    }
    if (
      visual_factor_reference_stamp_mode_ ==
      VisualFactorReferenceStampMode::kRenderedSourcePointcloud &&
      rendered.has_rendered_feedback_metadata)
    {
      return rendered.rendered_feedback_pointcloud_stamp_ns;
    }
    return observed.stamp_ns;
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

  void cache_sparse_lidar_depth_frame(
    const int64_t stamp_ns,
    const std::vector<Eigen::Vector3d> & points_i)
  {
    if (!enable_visual_factor_ || !enable_se3_photometric_window_factor_ ||
      !has_camera_intrinsics_ || last_observed_image_width_ == 0U ||
      last_observed_image_height_ == 0U || points_i.empty())
    {
      return;
    }

    DepthFrame frame;
    frame.stamp_ns = stamp_ns;
    frame.width = last_observed_image_width_;
    frame.height = last_observed_image_height_;
    frame.depth_m.assign(
      frame.width * frame.height,
      std::numeric_limits<float>::quiet_NaN());

    const Eigen::Quaterniond q_c_i = q_i_c_.normalized().inverse();
    size_t projected_count = 0U;
    for (const auto & point_i : points_i) {
      if (!point_i.allFinite()) {
        continue;
      }
      const Eigen::Vector3d point_c = q_c_i * (point_i - p_i_c_);
      const double z = point_c.z();
      if (!std::isfinite(z) || z <= se3_photometric_min_depth_m_ ||
        z > se3_photometric_max_depth_m_)
      {
        continue;
      }
      const double u_f = camera_intrinsics_.fx * point_c.x() / z + camera_intrinsics_.cx;
      const double v_f = camera_intrinsics_.fy * point_c.y() / z + camera_intrinsics_.cy;
      if (!std::isfinite(u_f) || !std::isfinite(v_f)) {
        continue;
      }
      const auto u = static_cast<int64_t>(std::llround(u_f));
      const auto v = static_cast<int64_t>(std::llround(v_f));
      if (u < 0 || v < 0 ||
        u >= static_cast<int64_t>(frame.width) ||
        v >= static_cast<int64_t>(frame.height))
      {
        continue;
      }
      const float depth = static_cast<float>(z);
      const int64_t dilation_px = static_cast<int64_t>(sparse_lidar_depth_dilation_px_);
      for (int64_t dy = -dilation_px; dy <= dilation_px; ++dy) {
        for (int64_t dx = -dilation_px; dx <= dilation_px; ++dx) {
          if (dx * dx + dy * dy > dilation_px * dilation_px) {
            continue;
          }
          const int64_t dilated_u = u + dx;
          const int64_t dilated_v = v + dy;
          if (dilated_u < 0 || dilated_v < 0 ||
            dilated_u >= static_cast<int64_t>(frame.width) ||
            dilated_v >= static_cast<int64_t>(frame.height))
          {
            continue;
          }
          const size_t index =
            static_cast<size_t>(dilated_v) * frame.width + static_cast<size_t>(dilated_u);
          if (!std::isfinite(frame.depth_m[index]) || depth < frame.depth_m[index]) {
            if (!std::isfinite(frame.depth_m[index])) {
              ++projected_count;
            }
            frame.depth_m[index] = depth;
          }
        }
      }
    }

    if (projected_count >= static_cast<size_t>(se3_photometric_min_samples_)) {
      cache_depth_frame(std::move(frame));
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

  void cache_observed_frame(gaussian_lic_tracking::VisualFrame frame)
  {
    const auto insert_it = std::lower_bound(
      observed_frame_cache_.begin(), observed_frame_cache_.end(), frame.stamp_ns,
      [](const gaussian_lic_tracking::VisualFrame & cached, const int64_t stamp_ns) {
        return cached.stamp_ns < stamp_ns;
      });
    if (insert_it != observed_frame_cache_.end() && insert_it->stamp_ns == frame.stamp_ns) {
      *insert_it = std::move(frame);
    } else {
      observed_frame_cache_.insert(insert_it, std::move(frame));
    }

    const auto max_cache_size = static_cast<size_t>(observed_frame_cache_size_);
    while (observed_frame_cache_.size() > max_cache_size) {
      observed_frame_cache_.pop_front();
    }
  }

  static bool visual_frame_has_rendered_feedback_source(
    const gaussian_lic_tracking::VisualFrame & rendered)
  {
    return rendered.has_rendered_feedback_metadata;
  }

  static bool visual_pair_sources_match(
    const VisualPairKey & key,
    const gaussian_lic_tracking::VisualFrame & rendered)
  {
    return key.has_rendered_feedback_source &&
           visual_frame_has_rendered_feedback_source(rendered) &&
           key.rendered_feedback_frame_index == rendered.rendered_feedback_frame_index &&
           key.rendered_feedback_preview_index == rendered.rendered_feedback_preview_index;
  }

  static VisualPairKey make_visual_pair_key(
    const gaussian_lic_tracking::VisualFrame & observed,
    const gaussian_lic_tracking::VisualFrame & rendered)
  {
    VisualPairKey key;
    key.observed_stamp_ns = observed.stamp_ns;
    key.rendered_stamp_ns = rendered.stamp_ns;
    key.has_rendered_feedback_source = visual_frame_has_rendered_feedback_source(rendered);
    if (key.has_rendered_feedback_source) {
      key.rendered_feedback_frame_index = rendered.rendered_feedback_frame_index;
      key.rendered_feedback_preview_index = rendered.rendered_feedback_preview_index;
    }
    return key;
  }

  bool visual_pair_was_processed(
    const int64_t observed_stamp_ns,
    const int64_t rendered_stamp_ns,
    const bool collapse_observed_stamp,
    const bool collapse_rendered_stamp) const
  {
    return std::any_of(
      processed_visual_pairs_.begin(), processed_visual_pairs_.end(),
      [observed_stamp_ns, rendered_stamp_ns, collapse_observed_stamp, collapse_rendered_stamp](
        const VisualPairKey & key)
      {
        const bool observed_matches = key.observed_stamp_ns == observed_stamp_ns;
        const bool rendered_matches = key.rendered_stamp_ns == rendered_stamp_ns;
        if (collapse_observed_stamp && observed_matches) {
          return true;
        }
        if (collapse_rendered_stamp && rendered_matches) {
          return true;
        }
        return observed_matches && rendered_matches;
      });
  }

  bool visual_pair_was_processed(
    const gaussian_lic_tracking::VisualFrame & observed,
    const gaussian_lic_tracking::VisualFrame & rendered,
    const bool collapse_observed_stamp,
    const bool collapse_rendered_stamp) const
  {
    return std::any_of(
      processed_visual_pairs_.begin(), processed_visual_pairs_.end(),
      [this, &observed, &rendered, collapse_observed_stamp, collapse_rendered_stamp](
        const VisualPairKey & key)
      {
        const bool observed_matches = key.observed_stamp_ns == observed.stamp_ns;
        const bool rendered_matches = key.rendered_stamp_ns == rendered.stamp_ns;
        if (collapse_observed_stamp && observed_matches) {
          return true;
        }
        if (collapse_rendered_stamp && rendered_matches) {
          return true;
        }
        if (visual_pair_sources_match(key, rendered)) {
          return true;
        }
        if (key.has_rendered_feedback_source || visual_frame_has_rendered_feedback_source(rendered)) {
          return false;
        }
        return observed_matches && rendered_matches;
      });
  }

  bool rendered_frame_was_processed(
    const gaussian_lic_tracking::VisualFrame & rendered) const
  {
    return std::any_of(
      processed_visual_pairs_.begin(), processed_visual_pairs_.end(),
      [&rendered](const VisualPairKey & key) {
        if (visual_pair_sources_match(key, rendered)) {
          return true;
        }
        if (key.has_rendered_feedback_source || visual_frame_has_rendered_feedback_source(rendered)) {
          return false;
        }
        return key.rendered_stamp_ns == rendered.stamp_ns;
      });
  }

  bool visual_pair_requires_unique_observed_stamp() const
  {
    return enable_visual_cache_reconciliation_ || visual_pair_monotonic_unique_;
  }

  bool visual_pair_requires_unique_rendered_stamp() const
  {
    return visual_cache_reconciliation_monotonic_unique_ || visual_pair_monotonic_unique_;
  }

  void remember_visual_pair(
    const gaussian_lic_tracking::VisualFrame & observed,
    const gaussian_lic_tracking::VisualFrame & rendered)
  {
    processed_visual_pairs_.push_back(make_visual_pair_key(observed, rendered));
    const auto cache_bound = static_cast<size_t>(
      std::max(256, 4 * (rendered_frame_cache_size_ + observed_frame_cache_size_)));
    while (processed_visual_pairs_.size() > cache_bound) {
      processed_visual_pairs_.pop_front();
    }
  }

  void reconcile_visual_frame_caches(std::optional<int64_t> reference_stamp_ns = std::nullopt)
  {
    if (!enable_visual_cache_reconciliation_ || !enable_visual_factor_ ||
      rendered_frame_cache_.empty() || observed_frame_cache_.empty())
    {
      return;
    }
    if (!reference_stamp_ns.has_value() && last_output_tracking_pose_.has_value()) {
      reference_stamp_ns = last_output_tracking_pose_->stamp_ns;
    }
    constexpr size_t kMaxReconciledPairsPerCallback = 1U;
    size_t reconciled = 0U;
    for (const auto & observed : observed_frame_cache_) {
      if (reference_stamp_ns.has_value() &&
        visual_factor_stamp_is_expired(observed.stamp_ns, reference_stamp_ns.value()))
      {
        continue;
      }
      int64_t rendered_match_delta_ns = 0;
      int64_t rendered_nearest_delta_ns = 0;
      int64_t rendered_nearest_signed_delta_ns = 0;
      bool rendered_cache_had_size_match = false;
      const auto * rendered = select_rendered_frame_for_stamp(
        observed.stamp_ns,
        observed.width,
        observed.height,
        &rendered_match_delta_ns,
        &rendered_cache_had_size_match,
        visual_pair_requires_unique_rendered_stamp(),
        &rendered_nearest_delta_ns,
        &rendered_nearest_signed_delta_ns);
      last_visual_rendered_nearest_delta_ns_ = rendered_nearest_delta_ns;
      last_visual_rendered_nearest_signed_delta_ns_ = rendered_nearest_signed_delta_ns;
      if (rendered == nullptr) {
        continue;
      }
      if (visual_pair_was_processed(
          observed, *rendered, true, visual_pair_requires_unique_rendered_stamp()))
      {
        continue;
      }
      last_visual_rendered_cache_size_ = rendered_frame_cache_.size();
      last_visual_rendered_match_delta_ns_ = rendered_match_delta_ns;
      process_visual_pair(*rendered, observed, true);
      ++visual_cache_reconciled_pairs_;
      ++reconciled;
      if (reconciled >= kMaxReconciledPairsPerCallback) {
        return;
      }
    }
  }

  void process_visual_pairs_up_to_watermark(
    const int64_t watermark_stamp_ns,
    const bool ingest_after_processing)
  {
    if (!enable_visual_watermark_pair_scheduler_ || !enable_visual_factor_ ||
      rendered_frame_cache_.empty() || observed_frame_cache_.empty())
    {
      return;
    }

    size_t processed_this_call = 0U;
    const size_t max_pairs =
      static_cast<size_t>(visual_watermark_pair_scheduler_max_pairs_per_pointcloud_);
    while (processed_this_call < max_pairs) {
      const gaussian_lic_tracking::VisualFrame * selected_observed = nullptr;
      const gaussian_lic_tracking::VisualFrame * selected_rendered = nullptr;
      int64_t selected_rendered_match_delta_ns = 0;
      int64_t selected_rendered_nearest_delta_ns = 0;
      int64_t selected_rendered_nearest_signed_delta_ns = 0;

      for (const auto & observed : observed_frame_cache_) {
        if (observed.stamp_ns > watermark_stamp_ns) {
          ++visual_watermark_pair_scheduler_deferred_pairs_;
          break;
        }
        if (visual_factor_stamp_is_expired(observed.stamp_ns, watermark_stamp_ns)) {
          continue;
        }
        if (visual_pair_was_processed(
            observed.stamp_ns, std::numeric_limits<int64_t>::min(), true, false))
        {
          continue;
        }
        bool rendered_cache_had_size_match = false;
        const auto * rendered = select_rendered_frame_for_stamp(
          observed.stamp_ns,
          observed.width,
          observed.height,
          &selected_rendered_match_delta_ns,
          &rendered_cache_had_size_match,
          true,
          &selected_rendered_nearest_delta_ns,
          &selected_rendered_nearest_signed_delta_ns,
          watermark_stamp_ns);
        last_visual_rendered_nearest_delta_ns_ = selected_rendered_nearest_delta_ns;
        last_visual_rendered_nearest_signed_delta_ns_ =
          selected_rendered_nearest_signed_delta_ns;
        if (rendered == nullptr) {
          ++visual_watermark_pair_scheduler_deferred_pairs_;
          continue;
        }
        if (visual_pair_was_processed(observed, *rendered, true, true)) {
          continue;
        }
        selected_observed = &observed;
        selected_rendered = rendered;
        break;
      }

      if (selected_observed == nullptr || selected_rendered == nullptr) {
        break;
      }
      last_visual_rendered_cache_size_ = rendered_frame_cache_.size();
      last_visual_rendered_match_delta_ns_ = selected_rendered_match_delta_ns;
      process_visual_pair(*selected_rendered, *selected_observed, true);
      ++visual_cache_reconciled_pairs_;
      ++visual_watermark_pair_scheduler_processed_pairs_;
      ++processed_this_call;
    }
    if (
      ingest_after_processing && processed_this_call > 0U &&
      last_output_tracking_pose_.has_value())
    {
      ingest_pending_visual_factors_into_optimizer(last_output_tracking_pose_.value());
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
      if (delta_ns <= std::max<int64_t>(max_visual_depth_delta_ns(), 0LL) &&
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

  const DepthFrame * select_depth_frame_for_visual_pair(
    const gaussian_lic_tracking::VisualFrame & rendered,
    const gaussian_lic_tracking::VisualFrame & observed,
    int64_t * selected_delta_ns = nullptr,
    bool * cache_had_size_match = nullptr)
  {
    bool had_size_match = false;
    int64_t observed_delta_ns = 0;
    bool observed_had_size_match = false;
    const DepthFrame * observed_depth =
      select_depth_frame_for_stamp(
      observed.stamp_ns,
      observed.width,
      observed.height,
      &observed_delta_ns,
      &observed_had_size_match);
    had_size_match = had_size_match || observed_had_size_match;
    if (observed_depth != nullptr) {
      ++visual_depth_observed_stamp_matches_;
      if (selected_delta_ns != nullptr) {
        *selected_delta_ns = observed_delta_ns;
      }
      if (cache_had_size_match != nullptr) {
        *cache_had_size_match = had_size_match;
      }
      return observed_depth;
    }

    if (
      rendered.has_rendered_feedback_metadata &&
      rendered.rendered_feedback_pointcloud_stamp_ns != 0)
    {
      ++visual_depth_source_pointcloud_fallback_queries_;
      int64_t source_delta_ns = 0;
      bool source_had_size_match = false;
      const DepthFrame * source_depth =
        select_depth_frame_for_stamp(
        rendered.rendered_feedback_pointcloud_stamp_ns,
        observed.width,
        observed.height,
        &source_delta_ns,
        &source_had_size_match);
      had_size_match = had_size_match || source_had_size_match;
      if (source_depth != nullptr) {
        ++visual_depth_source_pointcloud_fallback_matches_;
        if (selected_delta_ns != nullptr) {
          *selected_delta_ns = source_delta_ns;
        }
        if (cache_had_size_match != nullptr) {
          *cache_had_size_match = had_size_match;
        }
        return source_depth;
      }
      ++visual_depth_source_pointcloud_fallback_misses_;
    }
    if (selected_delta_ns != nullptr) {
      *selected_delta_ns = 0;
    }
    if (cache_had_size_match != nullptr) {
      *cache_had_size_match = had_size_match;
    }
    return nullptr;
  }

  const gaussian_lic_tracking::VisualFrame * select_rendered_frame_for_stamp(
    const int64_t image_stamp_ns,
    const size_t width,
    const size_t height,
    int64_t * selected_delta_ns = nullptr,
    bool * cache_had_size_match = nullptr,
    bool require_unprocessed_rendered = false,
    int64_t * nearest_delta_ns = nullptr,
    int64_t * nearest_signed_delta_ns = nullptr,
    std::optional<int64_t> max_rendered_stamp_ns = std::nullopt) const
  {
    const gaussian_lic_tracking::VisualFrame * best = nullptr;
    int64_t best_delta_ns = std::numeric_limits<int64_t>::max();
    int64_t nearest_delta = std::numeric_limits<int64_t>::max();
    int64_t nearest_signed_delta = 0;
    bool had_size_match = false;
    for (const auto & frame : rendered_frame_cache_) {
      if (max_rendered_stamp_ns.has_value() && frame.stamp_ns > max_rendered_stamp_ns.value()) {
        continue;
      }
      if (frame.width != width || frame.height != height) {
        continue;
      }
      had_size_match = true;
      if (require_unprocessed_rendered && rendered_frame_was_processed(frame))
      {
        continue;
      }
      const int64_t signed_delta_ns = frame.stamp_ns - image_stamp_ns;
      const int64_t delta_ns = stamp_delta_ns(frame.stamp_ns, image_stamp_ns);
      if (delta_ns < nearest_delta) {
        nearest_delta = delta_ns;
        nearest_signed_delta = signed_delta_ns;
      }
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
    if (nearest_delta_ns != nullptr) {
      *nearest_delta_ns = nearest_delta == std::numeric_limits<int64_t>::max() ? 0 : nearest_delta;
    }
    if (nearest_signed_delta_ns != nullptr) {
      *nearest_signed_delta_ns =
        nearest_delta == std::numeric_limits<int64_t>::max() ? 0 : nearest_signed_delta;
    }
    return best;
  }

  const gaussian_lic_tracking::VisualFrame * select_observed_frame_for_stamp(
    const int64_t rendered_stamp_ns,
    const size_t width,
    const size_t height,
    int64_t * selected_delta_ns = nullptr,
    bool * cache_had_size_match = nullptr,
    bool require_unprocessed_observed = false,
    int64_t * nearest_delta_ns = nullptr,
    int64_t * nearest_signed_delta_ns = nullptr) const
  {
    const gaussian_lic_tracking::VisualFrame * best = nullptr;
    int64_t best_delta_ns = std::numeric_limits<int64_t>::max();
    int64_t nearest_delta = std::numeric_limits<int64_t>::max();
    int64_t nearest_signed_delta = 0;
    bool had_size_match = false;
    for (const auto & frame : observed_frame_cache_) {
      if (frame.width != width || frame.height != height) {
        continue;
      }
      had_size_match = true;
      if (require_unprocessed_observed &&
        visual_pair_was_processed(frame.stamp_ns, std::numeric_limits<int64_t>::min(), true, false))
      {
        continue;
      }
      const int64_t signed_delta_ns = frame.stamp_ns - rendered_stamp_ns;
      const int64_t delta_ns = stamp_delta_ns(frame.stamp_ns, rendered_stamp_ns);
      if (delta_ns < nearest_delta) {
        nearest_delta = delta_ns;
        nearest_signed_delta = signed_delta_ns;
      }
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
    if (nearest_delta_ns != nullptr) {
      *nearest_delta_ns = nearest_delta == std::numeric_limits<int64_t>::max() ? 0 : nearest_delta;
    }
    if (nearest_signed_delta_ns != nullptr) {
      *nearest_signed_delta_ns =
        nearest_delta == std::numeric_limits<int64_t>::max() ? 0 : nearest_signed_delta;
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
    DepthFrame embedded_depth;
    const DepthFrame * depth_frame = nullptr;
    const size_t pixel_count = observed.width * observed.height;
    if (observed.has_embedded_depth && observed.embedded_depth_m.size() == pixel_count) {
      embedded_depth.stamp_ns = observed.stamp_ns;
      embedded_depth.width = observed.width;
      embedded_depth.height = observed.height;
      embedded_depth.depth_m = observed.embedded_depth_m;
      depth_frame = &embedded_depth;
      ++visual_depth_embedded_observed_matches_;
    } else {
      depth_frame = select_depth_frame_for_visual_pair(
        rendered,
        observed,
        &depth_match_delta_ns,
        &depth_cache_had_size_match);
    }
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
    if (rendered.gray.size() != pixel_count || observed.gray.size() != pixel_count ||
      depth_frame->depth_m.size() != pixel_count)
    {
      return batch;
    }
    const size_t max_samples = static_cast<size_t>(se3_photometric_max_samples_);
    const size_t coverage_grid_cols = static_cast<size_t>(se3_photometric_coverage_grid_cols_);
    const size_t coverage_grid_rows = static_cast<size_t>(se3_photometric_coverage_grid_rows_);
    batch.coverage_total_tiles = coverage_grid_cols * coverage_grid_rows;
    std::vector<bool> occupied_coverage_tiles(batch.coverage_total_tiles, false);
    std::vector<std::vector<size_t>> valid_depth_tiles(batch.coverage_total_tiles);
    const auto coverage_tile_for_pixel = [coverage_grid_cols, coverage_grid_rows, &observed](
        const size_t x, const size_t y) {
        const size_t tile_x = std::min(coverage_grid_cols - 1U, (x * coverage_grid_cols) / observed.width);
        const size_t tile_y = std::min(coverage_grid_rows - 1U, (y * coverage_grid_rows) / observed.height);
        return tile_y * coverage_grid_cols + tile_x;
      };
    size_t interior_pixels = 0U;
    size_t valid_depth_pixels = 0U;
    for (size_t index = 0; index < pixel_count; ++index) {
      const size_t x = index % observed.width;
      const size_t y = index / observed.width;
      if (x == 0U || y == 0U || x + 1U >= observed.width || y + 1U >= observed.height) {
        continue;
      }
      ++interior_pixels;
      const float depth = depth_frame->depth_m[index];
      if (!std::isfinite(depth) ||
        static_cast<double>(depth) < se3_photometric_min_depth_m_ ||
        static_cast<double>(depth) > se3_photometric_max_depth_m_)
      {
        continue;
      }
      valid_depth_tiles[coverage_tile_for_pixel(x, y)].push_back(index);
      ++valid_depth_pixels;
    }
    batch.candidate_pixels = interior_pixels;
    batch.rejected_depth_pixels = interior_pixels - valid_depth_pixels;
    if (valid_depth_pixels == 0U) {
      return batch;
    }
    auto observed_at = [&observed](const size_t px, const size_t py) {
        return static_cast<double>(observed.gray[py * observed.width + px]);
      };
    auto rendered_at = [&rendered](const size_t px, const size_t py) {
        return static_cast<double>(rendered.gray[py * rendered.width + px]);
      };
    auto gradient_at = [&](const size_t px, const size_t py) {
        return se3_photometric_use_rendered_gradient_ ? rendered_at(px, py) : observed_at(px, py);
      };
    auto gradient_score = [&](const size_t index) {
        const size_t x = index % observed.width;
        const size_t y = index / observed.width;
        const Eigen::Vector2d gradient{
          0.5 * (gradient_at(x + 1U, y) - gradient_at(x - 1U, y)),
          0.5 * (gradient_at(x, y + 1U) - gradient_at(x, y - 1U))};
        const double norm = gradient.norm();
        return std::isfinite(norm) ? norm : 0.0;
      };
    std::vector<size_t> tile_quotas(valid_depth_tiles.size(), 0U);
    size_t active_tiles = 0U;
    for (const auto & tile_indices : valid_depth_tiles) {
      if (!tile_indices.empty()) {
        ++active_tiles;
      }
    }
    size_t remaining_samples = std::min(valid_depth_pixels, max_samples);
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
    valid_depth_indices.reserve(std::min(valid_depth_pixels, max_samples));
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
      if (se3_photometric_rank_samples_by_gradient_) {
        std::vector<size_t> ranked_indices = tile_indices;
        const auto quota_end =
          ranked_indices.begin() + static_cast<std::vector<size_t>::difference_type>(quota);
        std::partial_sort(
          ranked_indices.begin(), quota_end, ranked_indices.end(),
          [&](const size_t lhs, const size_t rhs) {
            return gradient_score(lhs) > gradient_score(rhs);
          });
        valid_depth_indices.insert(valid_depth_indices.end(), ranked_indices.begin(), quota_end);
        continue;
      }
      for (size_t i = 0; i < quota; ++i) {
        valid_depth_indices.push_back(tile_indices[(i * tile_indices.size()) / quota]);
      }
    }
    batch.samples.reserve(std::min(pixel_count, max_samples));
    double abs_residual_sum = 0.0;
    for (const size_t index : valid_depth_indices) {
      const size_t x = index % observed.width;
      const size_t y = index / observed.width;
      const float depth = depth_frame->depth_m[index];
      gaussian_lic_tracking::VisualSe3PhotometricSample sample;
      ++batch.sampled_depth_pixels;
      const double z = static_cast<double>(depth);
      sample.point_camera = Eigen::Vector3d{
        (static_cast<double>(x) - camera_intrinsics_.cx) * z / camera_intrinsics_.fx,
        (static_cast<double>(y) - camera_intrinsics_.cy) * z / camera_intrinsics_.fy,
        z};
      sample.image_gradient = Eigen::Vector2d{
        0.5 * (gradient_at(x + 1U, y) - gradient_at(x - 1U, y)),
        0.5 * (gradient_at(x, y + 1U) - gradient_at(x, y - 1U))};
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
      occupied_coverage_tiles[coverage_tile_for_pixel(x, y)] = true;
      batch.samples.push_back(sample);
      ++batch.accepted_pixels;
      abs_residual_sum += abs_residual;
    }
    batch.coverage_tiles = static_cast<size_t>(
      std::count(occupied_coverage_tiles.begin(), occupied_coverage_tiles.end(), true));
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
    if (lidar_time_mode_ == "scan_order") {
      return nullptr;
    }
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
      ++lidar_invalid_frames_;
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
      ++lidar_invalid_frames_;
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
      ++lidar_invalid_frames_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "PointCloud2 point_step is too small for x/y/z fields");
      return points;
    }

    const size_t count = static_cast<size_t>(msg.width) * static_cast<size_t>(msg.height);
    points.reserve(count);
    const int64_t cloud_stamp_ns = gaussian_lic_tracking::stamp_to_nanoseconds(msg.header.stamp);
    size_t invalid_points = 0U;
    size_t invalid_point_times = 0U;
    size_t out_of_range_point_times = 0U;
    double max_abs_point_time_offset_s = 0.0;
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
        ++invalid_points;
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
              const double abs_offset_s =
                static_cast<double>(stamp_delta_ns(stamp_ns.value(), cloud_stamp_ns)) /
                static_cast<double>(gaussian_lic_tracking::kNanosecondsPerSecond);
              max_abs_point_time_offset_s = std::max(max_abs_point_time_offset_s, abs_offset_s);
              if (abs_offset_s <= lidar_max_abs_point_time_offset_s_) {
                point.stamp_ns = stamp_ns.value();
                point.has_stamp = true;
              } else {
                ++out_of_range_point_times;
              }
            } else {
              ++invalid_point_times;
            }
          } else {
            ++invalid_point_times;
          }
        } else if (lidar_time_mode_ == "scan_order") {
          const double normalized_index = count > 1U
            ? static_cast<double>(index) / static_cast<double>(count - 1U)
            : 0.5;
          const double offset_s = (normalized_index - 0.5) * lidar_scan_order_duration_s_;
          const int64_t offset_ns = static_cast<int64_t>(
            std::llround(offset_s * static_cast<double>(gaussian_lic_tracking::kNanosecondsPerSecond)));
          const int64_t stamp_ns = cloud_stamp_ns + offset_ns;
          const double abs_offset_s = std::abs(offset_s);
          max_abs_point_time_offset_s = std::max(max_abs_point_time_offset_s, abs_offset_s);
          if (abs_offset_s <= lidar_max_abs_point_time_offset_s_) {
            point.stamp_ns = stamp_ns;
            point.has_stamp = true;
          } else {
            ++out_of_range_point_times;
          }
        }
        points.push_back(point);
      } else {
        ++invalid_points;
      }
    }
    lidar_invalid_points_ += invalid_points;
    lidar_invalid_point_times_ += invalid_point_times;
    lidar_out_of_range_point_times_ += out_of_range_point_times;
    last_lidar_max_abs_point_time_offset_s_ = max_abs_point_time_offset_s;
    if (invalid_points > 0U || invalid_point_times > 0U || out_of_range_point_times > 0U) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "PointCloud2 decode skipped invalid_points=%zu invalid_point_times=%zu out_of_range_point_times=%zu max_abs_time_offset_s=%.6f limit_s=%.6f",
        invalid_points,
        invalid_point_times,
        out_of_range_point_times,
        max_abs_point_time_offset_s,
        lidar_max_abs_point_time_offset_s_);
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
        ++trajectory_deskew_hits_;
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

  bool apply_tracking_step_guard(
    gaussian_lic_tracking::TrajectoryPose & pose,
    const bool rebase_imu,
    const StepGuardStage stage,
    const gaussian_lic_tracking::TrajectoryPose * fallback_pose = nullptr)
  {
    if (!last_output_tracking_pose_.has_value() || tracking_max_pose_step_m_ <= 0.0) {
      return false;
    }
    const auto & previous = last_output_tracking_pose_.value();
    if (pose.stamp_ns <= previous.stamp_ns) {
      return false;
    }
    Eigen::Vector3d delta = pose.p_w_i - previous.p_w_i;
    const double step_m = delta.norm();
    const double dt_s = static_cast<double>(pose.stamp_ns - previous.stamp_ns) /
      static_cast<double>(gaussian_lic_tracking::kNanosecondsPerSecond);
    double allowed_step_m = tracking_max_pose_step_m_;
    if (stage == StepGuardStage::kPreLio && pre_lio_tracking_max_pose_step_m_ > 0.0) {
      allowed_step_m = pre_lio_tracking_max_pose_step_m_;
    } else if (stage == StepGuardStage::kPostBa && post_ba_tracking_max_pose_step_m_ > 0.0) {
      allowed_step_m = post_ba_tracking_max_pose_step_m_;
    }
    const double stage_base_step_m = allowed_step_m;
    double confidence_score = 0.0;
    const bool confidence_warmup_ready =
      post_ba_step_guard_confidence_warmup_marginalizations_ <= 0 ||
      (has_last_sliding_window_summary_ &&
      last_sliding_window_summary_.schur_marginalization_count >=
      static_cast<size_t>(post_ba_step_guard_confidence_warmup_marginalizations_));
    if (stage == StepGuardStage::kPostBa &&
      confidence_warmup_ready &&
      post_ba_step_guard_confidence_max_pose_step_m_ > allowed_step_m)
    {
      confidence_score = post_ba_step_guard_confidence_score();
      allowed_step_m += confidence_score *
        (post_ba_step_guard_confidence_max_pose_step_m_ - allowed_step_m);
    }
    double previous_speed_mps = previous.v_w_i.norm();
    if (!std::isfinite(previous_speed_mps)) {
      previous_speed_mps = 0.0;
    }
    double reference_speed_mps = std::max(previous_speed_mps, pose.v_w_i.norm());
    if (!std::isfinite(reference_speed_mps)) {
      reference_speed_mps = 0.0;
    }
    if (dt_s > 1.0e-9) {
      double velocity_scale = tracking_step_guard_velocity_scale_;
      if (stage == StepGuardStage::kPreLio) {
        velocity_scale = std::max(velocity_scale, pre_lio_tracking_step_guard_velocity_scale_);
      } else if (stage == StepGuardStage::kPostBa) {
        velocity_scale = std::max(velocity_scale, post_ba_tracking_step_guard_velocity_scale_);
      }
      if (velocity_scale > 0.0) {
        allowed_step_m = std::max(
          allowed_step_m,
          velocity_scale * reference_speed_mps * dt_s +
          tracking_step_guard_margin_m_);
      }
      if (tracking_step_guard_acceleration_mps2_ > 0.0) {
        allowed_step_m = std::max(
          allowed_step_m,
          (previous_speed_mps + tracking_step_guard_acceleration_mps2_ * dt_s) * dt_s +
          tracking_step_guard_margin_m_);
      }
      if (tracking_step_guard_max_velocity_mps_ > 0.0) {
        const double hard_velocity_step_m =
          tracking_step_guard_max_velocity_mps_ * dt_s + tracking_step_guard_margin_m_;
        allowed_step_m = std::min(
          allowed_step_m,
          std::max(stage_base_step_m, hard_velocity_step_m));
      }
    }
    const bool late_pre_ba_agreement_active =
      stage == StepGuardStage::kPostBa && post_ba_pre_ba_agreement_late_is_ready();
    const double pre_ba_agreement_max_pose_step_m =
      post_ba_pre_ba_agreement_max_pose_step_m();
    last_tracking_step_guard_pre_ba_agreement_limit_m_ =
      stage == StepGuardStage::kPostBa ? pre_ba_agreement_max_pose_step_m : 0.0;
    last_tracking_step_guard_pre_ba_agreement_late_active_ =
      late_pre_ba_agreement_active;
    if (stage == StepGuardStage::kPostBa &&
      pre_ba_agreement_max_pose_step_m > allowed_step_m &&
      fallback_pose != nullptr &&
      fallback_pose->stamp_ns == pose.stamp_ns &&
      valid_trajectory_pose(*fallback_pose))
    {
      const Eigen::Vector3d pre_ba_delta = fallback_pose->p_w_i - previous.p_w_i;
      const double pre_ba_step_m = pre_ba_delta.norm();
      const double ba_pre_delta_m = (pose.p_w_i - fallback_pose->p_w_i).norm();
      if (std::isfinite(pre_ba_step_m) && std::isfinite(ba_pre_delta_m) &&
        pre_ba_step_m > 1.0e-9 && step_m > 1.0e-9 &&
        ba_pre_delta_m <= post_ba_step_guard_pre_ba_agreement_max_delta_m_)
      {
        const double agreement_cosine = delta.dot(pre_ba_delta) / (step_m * pre_ba_step_m);
        if (std::isfinite(agreement_cosine) &&
          agreement_cosine >= post_ba_step_guard_pre_ba_agreement_min_cosine_)
        {
          const double previous_allowed_step_m = allowed_step_m;
          const double agreed_limit = std::min(
            pre_ba_agreement_max_pose_step_m,
            pre_ba_step_m + post_ba_step_guard_pre_ba_agreement_margin_m_);
          allowed_step_m = std::max(allowed_step_m, agreed_limit);
          if (allowed_step_m > previous_allowed_step_m) {
            ++tracking_step_guard_pre_ba_agreement_release_count_;
            if (late_pre_ba_agreement_active) {
              ++tracking_step_guard_late_pre_ba_agreement_release_count_;
            }
          }
        }
      }
    }
    last_tracking_step_guard_raw_step_m_ = step_m;
    last_tracking_step_guard_allowed_step_m_ = allowed_step_m;
    last_tracking_step_guard_dt_s_ = dt_s;
    last_tracking_step_guard_reference_speed_mps_ = reference_speed_mps;
    last_tracking_step_guard_confidence_score_ = confidence_score;
    if (!std::isfinite(step_m) || !std::isfinite(allowed_step_m) || step_m <= allowed_step_m) {
      return false;
    }
    if (stage == StepGuardStage::kPostBa &&
      post_ba_step_guard_reject_to_pre_ba_over_m_ > 0.0 &&
      step_m > post_ba_step_guard_reject_to_pre_ba_over_m_ &&
      fallback_pose != nullptr &&
      fallback_pose->stamp_ns == pose.stamp_ns &&
      valid_trajectory_pose(*fallback_pose))
    {
      pose = *fallback_pose;
      ++tracking_step_guard_post_ba_rejection_count_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "tracking post-BA step guard rejected pose step %.3fm over %.3fm at %" PRId64,
        step_m,
        post_ba_step_guard_reject_to_pre_ba_over_m_,
        pose.stamp_ns);
      if (rebase_imu && imu_propagator_.initialized()) {
        gaussian_lic_tracking::ImuState corrected_state;
        corrected_state.stamp_ns = pose.stamp_ns;
        corrected_state.p_w_i = pose.p_w_i;
        corrected_state.q_w_i = pose.q_w_i;
        corrected_state.v_w_i = pose.v_w_i;
        corrected_state.gyro_bias = sliding_window_bias_.gyro;
        corrected_state.accel_bias = sliding_window_bias_.accel;
        if (imu_propagator_.rebase_from_state(corrected_state)) {
          ++num_sliding_window_imu_reanchors_;
        }
      }
      return true;
    }
    if (stage == StepGuardStage::kPostBa &&
      post_ba_step_guard_pre_ba_blend_on_clamp_ > 0.0 &&
      fallback_pose != nullptr &&
      fallback_pose->stamp_ns == pose.stamp_ns &&
      valid_trajectory_pose(*fallback_pose))
    {
      const Eigen::Vector3d pre_ba_delta = fallback_pose->p_w_i - previous.p_w_i;
      const double pre_ba_step_m = pre_ba_delta.norm();
      if (std::isfinite(pre_ba_step_m) && pre_ba_step_m > 1.0e-9) {
        const Eigen::Vector3d blended_delta =
          (1.0 - post_ba_step_guard_pre_ba_blend_on_clamp_) * delta +
          post_ba_step_guard_pre_ba_blend_on_clamp_ * pre_ba_delta;
        const double blended_step_m = blended_delta.norm();
        if (std::isfinite(blended_step_m) && blended_step_m > 1.0e-9) {
          delta = blended_delta * (step_m / blended_step_m);
        }
      }
    }
    delta *= allowed_step_m / step_m;
    pose.p_w_i = previous.p_w_i + delta;
    if (dt_s > 1.0e-9) {
      pose.v_w_i = delta / dt_s;
      clamp_vector_norm(pose.v_w_i, sliding_window_max_feedback_velocity_norm_mps_);
    } else {
      pose.v_w_i = previous.v_w_i;
    }
    ++tracking_step_guard_clamp_count_;
    if (stage == StepGuardStage::kPreLio) {
      ++tracking_step_guard_pre_lio_clamp_count_;
    } else {
      ++tracking_step_guard_post_ba_clamp_count_;
    }
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "tracking %s step guard clamped pose step %.3fm to %.3fm at %" PRId64,
      stage == StepGuardStage::kPreLio ? "pre-LIO" : "post-BA",
      step_m,
      allowed_step_m,
      pose.stamp_ns);
    if (rebase_imu && imu_propagator_.initialized()) {
      gaussian_lic_tracking::ImuState corrected_state;
      corrected_state.stamp_ns = pose.stamp_ns;
      corrected_state.p_w_i = pose.p_w_i;
      corrected_state.q_w_i = pose.q_w_i;
      corrected_state.v_w_i = pose.v_w_i;
      corrected_state.gyro_bias = sliding_window_bias_.gyro;
      corrected_state.accel_bias = sliding_window_bias_.accel;
      if (imu_propagator_.rebase_from_state(corrected_state)) {
        ++num_sliding_window_imu_reanchors_;
      }
    }
    return true;
  }

  double post_ba_step_guard_confidence_score() const
  {
    double score = 1.0;
    bool has_signal = false;
    const size_t total_correspondences =
      last_window_point_correspondences_ + last_window_plane_correspondences_;
    if (total_correspondences >= static_cast<size_t>(lidar_min_points_)) {
      has_signal = true;
      const double lidar_confidence =
        std::max(last_window_point_confidence_mean_, last_window_plane_confidence_mean_);
      score = std::min(score, ratio_score(lidar_confidence, post_ba_step_guard_min_lidar_confidence_));
    }
    if (enable_visual_factor_ && visual_se3_photometric_valid_batches_ > 0U) {
      has_signal = true;
      score = std::min(
        score,
        ratio_score(
          last_accepted_visual_se3_photometric_sample_inlier_ratio_,
          post_ba_step_guard_min_visual_inlier_ratio_));
      score = std::min(
        score,
        ratio_score(
          static_cast<double>(last_accepted_visual_se3_photometric_coverage_tiles_),
          static_cast<double>(post_ba_step_guard_min_visual_coverage_tiles_)));
      if (post_ba_step_guard_max_visual_residual_ > 0.0) {
        score = std::min(
          score,
          inverse_ratio_score(
            last_accepted_visual_se3_photometric_mean_abs_residual_,
            post_ba_step_guard_max_visual_residual_));
      }
    }
    return has_signal ? std::clamp(score, 0.0, 1.0) : 0.0;
  }

  bool post_ba_pre_ba_agreement_late_is_ready() const
  {
    return post_ba_step_guard_pre_ba_agreement_late_start_marginalizations_ > 0 &&
           post_ba_step_guard_pre_ba_agreement_late_max_pose_step_m_ >
           post_ba_step_guard_pre_ba_agreement_max_pose_step_m_ &&
           has_last_sliding_window_summary_ &&
           last_sliding_window_summary_.schur_marginalization_count >=
           static_cast<size_t>(
             post_ba_step_guard_pre_ba_agreement_late_start_marginalizations_);
  }

  double post_ba_pre_ba_agreement_max_pose_step_m() const
  {
    if (post_ba_pre_ba_agreement_late_is_ready()) {
      return post_ba_step_guard_pre_ba_agreement_late_max_pose_step_m_;
    }
    return post_ba_step_guard_pre_ba_agreement_max_pose_step_m_;
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
    status.external_odometry_prior_stamp_regressions =
      external_odometry_prior_stamp_regressions_;
    status.imu_invalid_measurements = imu_invalid_measurements_;
    status.external_odometry_prior_invalid_messages =
      external_odometry_prior_invalid_messages_;
    status.camera_info_invalid_intrinsics = camera_info_invalid_intrinsics_;
    status.image_invalid_frames = image_invalid_frames_;
    status.depth_invalid_frames = depth_invalid_frames_;
    status.rendered_invalid_frames = rendered_invalid_frames_;
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
    status.num_rendered_images = num_rendered_images_;
    status.rendered_feedback_contract_enabled = enable_rendered_feedback_contract_;
    status.num_rendered_feedbacks = num_rendered_feedbacks_;
    status.rendered_feedback_ingress_queue_enabled =
      enable_rendered_feedback_ingress_queue_;
    status.rendered_feedback_ingress_received =
      rendered_feedback_ingress_received_;
    status.rendered_feedback_ingress_drained =
      rendered_feedback_ingress_drained_;
    status.rendered_feedback_ingress_drops =
      rendered_feedback_ingress_drops_;
    status.rendered_feedback_ingress_queue_size =
      static_cast<uint64_t>(rendered_feedback_ingress_queue_last_size_);
    status.rendered_feedback_ingress_queue_peak_size =
      static_cast<uint64_t>(rendered_feedback_ingress_queue_peak_size_);
    status.rendered_feedback_embedded_observed_pairs =
      rendered_feedback_embedded_observed_pairs_;
    status.rendered_feedback_embedded_depth_pairs =
      rendered_feedback_embedded_depth_pairs_;
    status.rendered_feedback_embedded_depth_invalid =
      rendered_feedback_embedded_depth_invalid_;
    status.rendered_feedback_source_pose_reference_enabled =
      enable_rendered_feedback_source_pose_reference_;
    status.rendered_feedback_source_pose_reference_factors =
      rendered_feedback_source_pose_reference_factors_;
    status.rendered_feedback_source_pose_invalid =
      rendered_feedback_source_pose_invalid_;
    status.rendered_feedback_source_motion_factor_enabled =
      enable_rendered_feedback_source_motion_factor_;
    status.rendered_feedback_source_motion_marginalized_prior_enabled =
      enable_rendered_feedback_source_motion_marginalized_prior_;
    status.rendered_feedback_source_motion_queued_factors =
      rendered_feedback_source_motion_queued_factors_;
    status.rendered_feedback_source_motion_factors =
      rendered_feedback_source_motion_factors_;
    status.rendered_feedback_source_motion_pending_factors =
      static_cast<uint64_t>(pending_rendered_feedback_source_motion_factors_.size());
    status.rendered_feedback_source_motion_invalid =
      rendered_feedback_source_motion_invalid_;
    status.rendered_feedback_source_motion_dt_skip_count =
      rendered_feedback_source_motion_dt_skip_count_;
    status.rendered_feedback_source_motion_stale_drops =
      rendered_feedback_source_motion_stale_drops_;
    status.rendered_feedback_source_motion_future_deferrals =
      rendered_feedback_source_motion_future_deferrals_;
    status.rendered_feedback_source_motion_marginalized_priors =
      rendered_feedback_source_motion_marginalized_priors_;
    status.rendered_feedback_source_motion_marginalized_source_factors =
      rendered_feedback_source_motion_marginalized_source_factors_;
    status.rendered_feedback_source_motion_marginalized_prior_skips =
      rendered_feedback_source_motion_marginalized_prior_skips_;
    status.last_rendered_feedback_frame_index = last_rendered_feedback_frame_index_;
    status.last_rendered_feedback_preview_index = last_rendered_feedback_preview_index_;
    status.rendered_feedback_frame_index_regressions =
      rendered_feedback_frame_index_regressions_;
    status.rendered_feedback_preview_index_regressions =
      rendered_feedback_preview_index_regressions_;
    status.rendered_feedback_frame_index_gap_count =
      rendered_feedback_frame_index_gap_count_;
    status.rendered_feedback_preview_index_gap_count =
      rendered_feedback_preview_index_gap_count_;
    status.rendered_feedback_frame_index_missing =
      rendered_feedback_frame_index_missing_;
    status.rendered_feedback_preview_index_missing =
      rendered_feedback_preview_index_missing_;
    status.rendered_feedback_duplicate_source_ids =
      rendered_feedback_duplicate_source_ids_;
    status.last_rendered_feedback_observed_delta_ns =
      last_rendered_feedback_observed_delta_ns_;
    status.last_rendered_feedback_pose_delta_ns =
      last_rendered_feedback_pose_delta_ns_;
    status.last_rendered_feedback_pointcloud_delta_ns =
      last_rendered_feedback_pointcloud_delta_ns_;
    status.last_rendered_feedback_reference_stamp_ns =
      last_rendered_feedback_reference_stamp_ns_;
    status.last_rendered_feedback_oldest_active_state_delta_ns =
      last_rendered_feedback_oldest_active_state_delta_ns_;
    status.last_rendered_feedback_newest_active_state_delta_ns =
      last_rendered_feedback_newest_active_state_delta_ns_;
    status.rendered_feedback_before_active_window =
      rendered_feedback_before_active_window_;
    status.rendered_feedback_after_active_window =
      rendered_feedback_after_active_window_;
    status.num_raw_pointclouds = num_raw_pointclouds_;
    status.num_raw_imus = num_raw_imus_;
    status.num_published_poses = num_published_poses_;
    status.pointcloud_imu_wait_queue_size =
      static_cast<uint64_t>(pending_pointclouds_waiting_for_imu_.size());
    status.pointcloud_imu_wait_deferred = pointcloud_imu_wait_deferred_;
    status.pointcloud_imu_wait_released = pointcloud_imu_wait_released_;
    status.pointcloud_imu_wait_dropped = pointcloud_imu_wait_dropped_;
    status.pointcloud_imu_wait_stale_dropped = pointcloud_imu_wait_stale_dropped_;
    status.external_odometry_priors_received = external_odometry_priors_received_;
    status.external_odometry_prior_matches = external_odometry_prior_matches_;
    status.external_odometry_prior_misses = external_odometry_prior_misses_;
    status.last_external_odometry_prior_stamp_ns = last_external_odometry_prior_stamp_ns_;

    status.num_lidar_keyframes = num_lidar_keyframes_;
    status.lidar_map_points = static_cast<uint64_t>(lidar_factor_.map_size());
    status.lidar_spatial_index_voxels =
      static_cast<uint64_t>(lidar_factor_.spatial_index_voxels());
    status.lidar_spatial_index_voxel_size_m = lidar_factor_.spatial_index_voxel_size_m();
    status.last_lidar_points = static_cast<uint64_t>(last_lidar_points_);
    status.lidar_invalid_frames = lidar_invalid_frames_;
    status.lidar_invalid_points = lidar_invalid_points_;
    status.lidar_invalid_point_times = lidar_invalid_point_times_;
    status.lidar_out_of_range_point_times = lidar_out_of_range_point_times_;
    status.last_lidar_max_abs_point_time_offset_s = last_lidar_max_abs_point_time_offset_s_;
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
    status.last_window_point_confidence_mean = last_window_point_confidence_mean_;
    status.last_window_point_confidence_min = last_window_point_confidence_min_;
    status.last_window_plane_confidence_mean = last_window_plane_confidence_mean_;
    status.last_window_plane_confidence_min = last_window_plane_confidence_min_;

    const auto & summary = last_sliding_window_summary_;
    status.sliding_window_enabled = enable_sliding_window_optimizer_;
    status.sliding_window_relative_motion_history_source =
      relative_motion_history_source_name(sliding_window_relative_motion_history_source_);
    status.sliding_window_relative_motion_history_published_after_s =
      sliding_window_relative_motion_history_published_after_s_;
    status.sliding_window_delayed_published_multihop_relative_factors =
      sliding_window_delayed_published_multihop_relative_factor_count_;
    status.sliding_window_delayed_published_multihop_max_factors =
      static_cast<uint32_t>(sliding_window_delayed_published_multihop_max_factors_);
    status.sliding_window_states = has_last_sliding_window_summary_
      ? static_cast<uint64_t>(summary.state_count)
      : static_cast<uint64_t>(sliding_window_optimizer_.states().size());
    status.sliding_window_effective_max_states =
      static_cast<uint32_t>(sliding_window_effective_max_states_);
    status.sliding_window_imu_factors = static_cast<uint64_t>(summary.imu_factor_count);
    status.sliding_window_total_imu_factors = sliding_window_total_imu_factors_;
    status.sliding_window_total_imu_preintegration_samples =
      sliding_window_total_imu_preintegration_samples_;
    status.sliding_window_total_imu_preintegration_dt_s =
      sliding_window_total_imu_preintegration_dt_s_;
    status.sliding_window_total_visual_factors = sliding_window_total_visual_factors_;
    status.sliding_window_total_se3_photometric_factors =
      sliding_window_total_se3_photometric_factors_;
    status.sliding_window_pose_priors = static_cast<uint64_t>(summary.pose_prior_count);
    status.sliding_window_dense_priors = static_cast<uint64_t>(summary.dense_prior_count);
    status.sliding_window_point_factors = static_cast<uint64_t>(summary.point_factor_count);
    status.sliding_window_plane_factors = static_cast<uint64_t>(summary.plane_factor_count);
    status.sliding_window_visual_factors = static_cast<uint64_t>(summary.visual_factor_count);
    status.sliding_window_se3_photometric_factors =
      static_cast<uint64_t>(summary.se3_photometric_factor_count);
    status.sliding_window_relative_translation_factors =
      static_cast<uint64_t>(summary.relative_translation_factor_count);
    status.sliding_window_relative_distance_factors =
      static_cast<uint64_t>(summary.relative_distance_factor_count);
    status.sliding_window_smoothness_factors =
      static_cast<uint64_t>(summary.smoothness_factor_count);
    status.sliding_window_imu_factor_replacement_count =
      static_cast<uint64_t>(summary.imu_factor_replacement_count);
    status.sliding_window_point_factor_replacement_count =
      static_cast<uint64_t>(summary.point_factor_replacement_count);
    status.sliding_window_plane_factor_replacement_count =
      static_cast<uint64_t>(summary.plane_factor_replacement_count);
    status.sliding_window_visual_factor_replacement_count =
      static_cast<uint64_t>(summary.visual_factor_replacement_count);
    status.sliding_window_se3_photometric_factor_replacement_count =
      static_cast<uint64_t>(summary.se3_photometric_factor_replacement_count);
    status.sliding_window_relative_translation_factor_replacement_count =
      static_cast<uint64_t>(summary.relative_translation_factor_replacement_count);
    status.sliding_window_relative_distance_factor_replacement_count =
      static_cast<uint64_t>(summary.relative_distance_factor_replacement_count);
    status.sliding_window_smoothness_factor_replacement_count =
      static_cast<uint64_t>(summary.smoothness_factor_replacement_count);
    status.sliding_window_orphan_factors = static_cast<uint64_t>(summary.orphan_factor_count);
    status.sliding_window_point_factor_skip_count = sliding_window_point_factor_skip_count_;
    status.sliding_window_plane_factor_skip_count = sliding_window_plane_factor_skip_count_;
    status.sliding_window_visual_factor_skip_count = sliding_window_visual_factor_skip_count_;
    status.sliding_window_se3_photometric_factor_skip_count =
      sliding_window_se3_photometric_factor_skip_count_;
    status.sliding_window_smoothness_factor_skip_count =
      sliding_window_smoothness_factor_skip_count_;
    status.sliding_window_smoothness_motion_target_applied_count =
      sliding_window_smoothness_motion_target_applied_count_;
    status.sliding_window_smoothness_motion_target_support_skip_count =
      sliding_window_smoothness_motion_target_support_skip_count_;
    status.sliding_window_smoothness_motion_target_recent_support_skip_count =
      sliding_window_smoothness_motion_target_recent_support_skip_count_;
    status.sliding_window_smoothness_motion_target_warmup_skip_count =
      sliding_window_smoothness_motion_target_warmup_skip_count_;
    status.sliding_window_smoothness_motion_target_history_miss_count =
      sliding_window_smoothness_motion_target_history_miss_count_;
    status.sliding_window_smoothness_motion_target_invalid_count =
      sliding_window_smoothness_motion_target_invalid_count_;
    status.sliding_window_smoothness_motion_target_clamp_count =
      sliding_window_smoothness_motion_target_clamp_count_;
    status.sliding_window_imu_factor_skip_count = sliding_window_imu_factor_skip_count_;
    status.sliding_window_imu_time_gap_skip_count = sliding_window_imu_time_gap_skip_count_;
    status.sliding_window_last_imu_preintegration_samples =
      last_sliding_window_imu_preintegration_samples_;
    status.sliding_window_last_imu_preintegration_dt_s =
      last_sliding_window_imu_preintegration_dt_s_;
    status.sliding_window_last_imu_preintegration_extrapolated_dt_s =
      last_sliding_window_imu_preintegration_extrapolated_dt_s_;
    status.sliding_window_last_imu_preintegration_start_stamp_ns =
      last_sliding_window_imu_preintegration_start_stamp_ns_;
    status.sliding_window_last_imu_preintegration_end_stamp_ns =
      last_sliding_window_imu_preintegration_end_stamp_ns_;
    status.sliding_window_optimization_skip_count = sliding_window_optimization_skip_count_;
    status.sliding_window_invalid_optimized_states = sliding_window_invalid_optimized_states_;
    status.sliding_window_last_optimization_duration_ms =
      last_sliding_window_optimization_duration_ms_;
    status.sliding_window_feedback_updates = sliding_window_feedback_update_count_;
    status.sliding_window_guarded_state_syncs = sliding_window_guarded_state_sync_count_;
    status.sliding_window_guarded_pose_priors = sliding_window_guarded_pose_prior_count_;
    status.sliding_window_last_feedback_stamp_ns = last_sliding_window_feedback_stamp_ns_;
    status.sliding_window_last_feedback_translation_delta_m =
      last_sliding_window_feedback_translation_delta_m_;
    status.sliding_window_last_feedback_rotation_delta_rad =
      last_sliding_window_feedback_rotation_delta_rad_;
    status.sliding_window_last_feedback_velocity_delta_mps =
      last_sliding_window_feedback_velocity_delta_mps_;
    status.sliding_window_max_feedback_translation_m =
      sliding_window_max_feedback_translation_m_;
    status.sliding_window_max_feedback_rotation_rad =
      sliding_window_max_feedback_rotation_rad_;
    status.sliding_window_max_feedback_velocity_mps =
      sliding_window_max_feedback_velocity_mps_;
    status.sliding_window_bias_feedback_holds = sliding_window_bias_feedback_hold_count_;
    status.sliding_window_min_bias_feedback_visual_factors =
      static_cast<uint64_t>(sliding_window_min_bias_feedback_visual_factors_);
    status.sliding_window_bias_feedback_ownership =
      sliding_window_bias_feedback_ownership_name(sliding_window_bias_feedback_ownership_);
    status.sliding_window_bias_feedback_ownership_holds =
      sliding_window_bias_feedback_ownership_hold_count_;
    status.sliding_window_marginalized_states = static_cast<uint64_t>(summary.marginalized_state_count);
    status.sliding_window_schur_marginalizations =
      static_cast<uint64_t>(summary.schur_marginalization_count);
    status.sliding_window_fallback_marginalization_priors =
      static_cast<uint64_t>(summary.fallback_marginalization_prior_count);
    status.sliding_window_marginalized_backsubstitutions =
      static_cast<uint64_t>(summary.marginalized_backsubstitution_count);
    status.sliding_window_marginalized_backsubstitution_chain_updates =
      static_cast<uint64_t>(summary.marginalized_backsubstitution_chain_update_count);
    status.sliding_window_marginalized_backsubstitution_interpolations =
      static_cast<uint64_t>(summary.marginalized_backsubstitution_interpolation_count);
    status.sliding_window_visual_marginalization_priors =
      static_cast<uint64_t>(summary.visual_marginalization_prior_count);
    status.sliding_window_se3_photometric_marginalization_priors =
      static_cast<uint64_t>(summary.se3_photometric_marginalization_prior_count);
    status.sliding_window_dense_prior_rows = static_cast<uint64_t>(summary.dense_prior_rows);
    status.sliding_window_dense_prior_cols = static_cast<uint64_t>(summary.dense_prior_cols);
    status.sliding_window_dense_prior_rank = static_cast<uint64_t>(summary.dense_prior_rank);
    status.sliding_window_normal_equation_rows =
      static_cast<uint64_t>(summary.normal_equation_rows);
    status.sliding_window_normal_equation_cols =
      static_cast<uint64_t>(summary.normal_equation_cols);
    status.sliding_window_normal_equation_rank =
      static_cast<uint64_t>(summary.normal_equation_rank);
    status.sliding_window_numeric_jacobian_blocks =
      static_cast<uint64_t>(summary.numeric_jacobian_block_count);
    status.sliding_window_numeric_jacobian_columns =
      static_cast<uint64_t>(summary.numeric_jacobian_column_count);
    status.sliding_window_normal_equation_rank_ratio =
      summary.normal_equation_cols > 0U ?
      static_cast<double>(summary.normal_equation_rank) /
      static_cast<double>(summary.normal_equation_cols) :
      0.0;
    status.sliding_window_min_normal_equation_rank_ratio =
      sliding_window_min_normal_equation_rank_ratio_;
    status.sliding_window_max_normal_equation_condition =
      sliding_window_max_normal_equation_condition_;
    status.sliding_window_iterations = static_cast<uint64_t>(summary.iterations);
    status.sliding_window_accepted_steps = static_cast<uint64_t>(summary.accepted_steps);
    status.sliding_window_rejected_steps = static_cast<uint64_t>(summary.rejected_steps);
    status.sliding_window_limited_steps = static_cast<uint64_t>(summary.limited_steps);
    status.sliding_window_invalid_candidate_steps =
      static_cast<uint64_t>(summary.invalid_candidate_steps);
    status.sliding_window_linearization_failure_count =
      static_cast<uint64_t>(summary.linearization_failure_count);
    status.sliding_window_linear_solve_failure_count =
      static_cast<uint64_t>(summary.linear_solve_failure_count);
    status.sliding_window_initial_cost = summary.initial_cost;
    status.sliding_window_final_cost = summary.final_cost;
    status.sliding_window_imu_cost = summary.imu_cost;
    status.sliding_window_pose_prior_cost = summary.pose_prior_cost;
    status.sliding_window_state_prior_cost = summary.state_prior_cost;
    status.sliding_window_dense_prior_cost = summary.dense_prior_cost;
    status.sliding_window_point_factor_cost = summary.point_factor_cost;
    status.sliding_window_plane_factor_cost = summary.plane_factor_cost;
    status.sliding_window_visual_factor_cost = summary.visual_factor_cost;
    status.sliding_window_se3_photometric_factor_cost =
      summary.se3_photometric_factor_cost;
    status.sliding_window_relative_translation_factor_cost =
      summary.relative_translation_factor_cost;
    status.sliding_window_relative_distance_factor_cost =
      summary.relative_distance_factor_cost;
    status.sliding_window_smoothness_factor_cost = summary.smoothness_factor_cost;
    status.sliding_window_smoothness_motion_target_last_rotation_rate_delta_norm =
      last_sliding_window_smoothness_motion_target_rotation_rate_delta_norm_;
    status.sliding_window_smoothness_motion_target_last_position_rate_delta_norm =
      last_sliding_window_smoothness_motion_target_position_rate_delta_norm_;
    status.sliding_window_smoothness_motion_target_last_velocity_acceleration_delta_norm =
      last_sliding_window_smoothness_motion_target_velocity_acceleration_delta_norm_;
    status.sliding_window_smoothness_motion_target_max_rotation_rate_delta_norm =
      sliding_window_smoothness_motion_target_max_rotation_rate_delta_norm_;
    status.sliding_window_smoothness_motion_target_max_position_rate_delta_norm =
      sliding_window_smoothness_motion_target_max_position_rate_delta_norm_;
    status.sliding_window_smoothness_motion_target_max_velocity_acceleration_delta_norm =
      sliding_window_smoothness_motion_target_max_velocity_acceleration_delta_norm_;
    status.sliding_window_smoothness_motion_target_recent_visual_factors =
      last_sliding_window_smoothness_motion_target_recent_visual_factors_;
    status.sliding_window_smoothness_motion_target_recent_se3_photometric_factors =
      last_sliding_window_smoothness_motion_target_recent_se3_photometric_factors_;
    status.sliding_window_last_step_norm = summary.last_step_norm;
    status.sliding_window_last_step_scale = summary.last_step_scale;
    status.sliding_window_last_damping = summary.last_damping;
    status.sliding_window_dense_prior_min_singular_value = summary.dense_prior_min_singular_value;
    status.sliding_window_dense_prior_max_singular_value = summary.dense_prior_max_singular_value;
    status.sliding_window_dense_prior_gyro_bias_min_singular_value =
      summary.dense_prior_gyro_bias_min_singular_value;
    status.sliding_window_dense_prior_gyro_bias_max_singular_value =
      summary.dense_prior_gyro_bias_max_singular_value;
    status.sliding_window_dense_prior_accel_bias_min_singular_value =
      summary.dense_prior_accel_bias_min_singular_value;
    status.sliding_window_dense_prior_accel_bias_max_singular_value =
      summary.dense_prior_accel_bias_max_singular_value;
    status.sliding_window_min_state_dt_s = summary.min_state_dt_s;
    status.sliding_window_max_state_dt_s = summary.max_state_dt_s;
    status.sliding_window_normal_equation_min_singular_value =
      summary.normal_equation_min_singular_value;
    status.sliding_window_normal_equation_max_singular_value =
      summary.normal_equation_max_singular_value;
    status.sliding_window_normal_equation_condition_number =
      summary.normal_equation_condition_number;
    status.sliding_window_gyro_bias_norm = summary.gyro_bias_norm;
    status.sliding_window_accel_bias_norm = summary.accel_bias_norm;
    status.sliding_window_gyro_bias_x = sliding_window_bias_.gyro.x();
    status.sliding_window_gyro_bias_y = sliding_window_bias_.gyro.y();
    status.sliding_window_gyro_bias_z = sliding_window_bias_.gyro.z();
    status.sliding_window_accel_bias_x = sliding_window_bias_.accel.x();
    status.sliding_window_accel_bias_y = sliding_window_bias_.accel.y();
    status.sliding_window_accel_bias_z = sliding_window_bias_.accel.z();
    status.sliding_window_gyro_bias_random_walk_sqrt_info_mean =
      summary.gyro_bias_random_walk_sqrt_info_mean;
    status.sliding_window_accel_bias_random_walk_sqrt_info_mean =
      summary.accel_bias_random_walk_sqrt_info_mean;
    status.sliding_window_gyro_bias_random_walk_sqrt_info_max =
      summary.gyro_bias_random_walk_sqrt_info_max;
    status.sliding_window_accel_bias_random_walk_sqrt_info_max =
      summary.accel_bias_random_walk_sqrt_info_max;
    status.sliding_window_gyro_bias_observability = summary.gyro_bias_observability;
    status.sliding_window_accel_bias_observability = summary.accel_bias_observability;
    status.sliding_window_converged = summary.converged;
    status.sliding_window_normal_equation_degenerate = summary.normal_equation_degenerate;
    status.sliding_window_state_gap_degenerate = summary.state_gap_degenerate;
    status.sliding_window_imu_reanchors = num_sliding_window_imu_reanchors_;
    status.trajectory_control_poses = static_cast<uint64_t>(trajectory_manager_.size());
    status.trajectory_deskew_queries = trajectory_deskew_queries_;
    status.trajectory_deskew_hits = trajectory_deskew_hits_;
    status.trajectory_control_pose_skip_count = trajectory_control_pose_skip_count_;
    status.tracking_step_guard_clamps = tracking_step_guard_clamp_count_;
    status.tracking_step_guard_pre_lio_clamps = tracking_step_guard_pre_lio_clamp_count_;
    status.tracking_step_guard_post_ba_clamps = tracking_step_guard_post_ba_clamp_count_;
    status.tracking_step_guard_post_ba_rejections =
      tracking_step_guard_post_ba_rejection_count_;
    status.tracking_step_guard_pre_ba_agreement_releases =
      tracking_step_guard_pre_ba_agreement_release_count_;
    status.tracking_step_guard_late_pre_ba_agreement_releases =
      tracking_step_guard_late_pre_ba_agreement_release_count_;
    status.tracking_step_guard_last_raw_step_m = last_tracking_step_guard_raw_step_m_;
    status.tracking_step_guard_last_allowed_step_m = last_tracking_step_guard_allowed_step_m_;
    status.tracking_step_guard_last_dt_s = last_tracking_step_guard_dt_s_;
    status.tracking_step_guard_last_reference_speed_mps =
      last_tracking_step_guard_reference_speed_mps_;
    status.tracking_step_guard_last_confidence_score =
      last_tracking_step_guard_confidence_score_;
    status.tracking_step_guard_last_pre_ba_agreement_limit_m =
      last_tracking_step_guard_pre_ba_agreement_limit_m_;
    status.tracking_step_guard_last_pre_ba_agreement_late_active =
      last_tracking_step_guard_pre_ba_agreement_late_active_;

    status.gaussian_snapshot_points = static_cast<uint64_t>(gaussian_snapshot_.point_count());
    status.gaussian_snapshot_expected_total = last_gaussian_total_count_;
    status.gaussian_snapshot_chunks_received = gaussian_snapshot_chunks_received_;
    status.gaussian_snapshot_expected_chunks = last_gaussian_chunk_count_;
    status.gaussian_snapshot_complete = gaussian_snapshot_.complete();

    status.visual_factor_enabled = enable_visual_factor_;
    status.visual_factor_reference_stamp_mode = visual_factor_reference_stamp_mode_name_;
    status.visual_factor_time_interpolation_enabled = enable_visual_factor_time_interpolation_;
    status.visual_cache_reconciliation_enabled = enable_visual_cache_reconciliation_;
    status.visual_pair_monotonic_unique_enabled = visual_pair_monotonic_unique_;
    status.visual_watermark_pair_scheduler_enabled = enable_visual_watermark_pair_scheduler_;
    status.visual_watermark_pair_scheduler_processed_pairs =
      visual_watermark_pair_scheduler_processed_pairs_;
    status.visual_watermark_pair_scheduler_deferred_pairs =
      visual_watermark_pair_scheduler_deferred_pairs_;
    status.visual_callback_factor_ingest_enabled = enable_visual_callback_factor_ingest_;
    status.rendered_feedback_watermark_queue_enabled =
      enable_rendered_feedback_watermark_queue_;
    status.rendered_feedback_watermark_queue_size =
      static_cast<uint64_t>(rendered_feedback_watermark_queue_.size());
    status.rendered_feedback_watermark_processed_pairs =
      rendered_feedback_watermark_processed_pairs_;
    status.rendered_feedback_watermark_deferred_pairs =
      rendered_feedback_watermark_deferred_pairs_;
    status.rendered_feedback_watermark_queue_drops =
      rendered_feedback_watermark_queue_drops_;
    status.rendered_feedback_watermark_reordered_pairs =
      rendered_feedback_watermark_reordered_pairs_;
    status.defer_future_visual_factors_until_active_enabled =
      defer_future_visual_factors_until_active_;
    status.visual_adaptive_state_retention_enabled =
      enable_visual_adaptive_state_retention_;
    status.visual_render_backlog_frames = visual_render_backlog_frames_;
    status.visual_expired_factor_projection_enabled =
      enable_visual_expired_factor_projection_;
    status.visual_marginalization_prior_enabled =
      enable_visual_marginalization_prior_;
    status.visual_marginalization_prior_batching_enabled =
      enable_visual_marginalization_prior_batching_;
    status.visual_marginalization_prior_saturation_gate_enabled =
      enable_visual_marginalization_prior_saturation_gate_;
    status.visual_marginalization_prior_saturation_gate_visual_factors =
      visual_marginalization_prior_saturation_gate_visual_factors_;
    status.visual_marginalization_prior_saturation_gate_se3_factors =
      visual_marginalization_prior_saturation_gate_se3_factors_;
    status.visual_alignment_saturation_axis_mask_enabled =
      enable_visual_alignment_saturation_axis_mask_;
    status.visual_marginalization_prior_zero_bias_columns =
      visual_marginalization_prior_zero_bias_columns_;
    status.visual_alignment_expired_projected_factors =
      visual_alignment_expired_projected_factors_;
    status.visual_se3_photometric_expired_projected_factors =
      visual_se3_photometric_expired_projected_factors_;
    status.visual_expired_projection_skipped_factors =
      visual_expired_projection_skipped_factors_;
    status.visual_alignment_marginalization_priors =
      visual_alignment_marginalization_priors_;
    status.visual_se3_photometric_marginalization_priors =
      visual_se3_photometric_marginalization_priors_;
    status.visual_marginalization_prior_skipped_factors =
      visual_marginalization_prior_skipped_factors_;
    status.visual_marginalization_prior_saturation_rejected_factors =
      visual_marginalization_prior_saturation_rejected_factors_;
    status.visual_marginalization_prior_saturation_rejected_visual_factors =
      visual_marginalization_prior_saturation_rejected_visual_factors_;
    status.visual_marginalization_prior_saturation_rejected_se3_factors =
      visual_marginalization_prior_saturation_rejected_se3_factors_;
    status.visual_alignment_saturation_axis_masked_factors =
      visual_alignment_saturation_axis_masked_factors_;
    status.visual_alignment_saturation_axis_masked_axes =
      visual_alignment_saturation_axis_masked_axes_;
    status.visual_alignment_saturation_axis_mask_skipped_factors =
      visual_alignment_saturation_axis_mask_skipped_factors_;
    status.visual_batched_marginalization_prior_batches =
      visual_batched_marginalization_prior_batches_;
    status.visual_batched_marginalization_prior_visual_factors =
      visual_batched_marginalization_prior_visual_factors_;
    status.visual_batched_marginalization_prior_se3_factors =
      visual_batched_marginalization_prior_se3_factors_;
    status.visual_batched_marginalization_prior_skipped_batches =
      visual_batched_marginalization_prior_skipped_batches_;
    status.visual_batched_marginalization_prior_skipped_factors =
      visual_batched_marginalization_prior_skipped_factors_;
    status.visual_alignment_interpolated_factors = visual_alignment_interpolated_factor_count_;
    status.visual_se3_photometric_interpolated_factors =
      visual_se3_photometric_interpolated_factor_count_;
    status.visual_cache_reconciled_pairs = visual_cache_reconciled_pairs_;
    status.visual_cache_reconciled_saturated_pairs =
      visual_cache_reconciled_saturated_pairs_;
    status.visual_cache_reconciled_alignment_skipped_pairs =
      visual_cache_reconciled_alignment_skipped_pairs_;
    status.visual_cache_reconciled_alignment_photometric_fallback_pairs =
      visual_cache_reconciled_alignment_photometric_fallback_pairs_;
    status.visual_cache_reconciled_alignment_photometric_disagreement_pairs =
      visual_cache_reconciled_alignment_photometric_disagreement_pairs_;
    status.visual_rendered_cache_size = static_cast<uint64_t>(last_visual_rendered_cache_size_);
    status.visual_rendered_match_delta_ns = last_visual_rendered_match_delta_ns_;
    status.visual_rendered_nearest_delta_ns = last_visual_rendered_nearest_delta_ns_;
    status.visual_rendered_nearest_signed_delta_ns =
      last_visual_rendered_nearest_signed_delta_ns_;
    status.visual_rendered_miss_count = visual_rendered_miss_count_;
    status.visual_rendered_stale_count = visual_rendered_stale_count_;
    status.visual_rendered_size_mismatch_count = visual_rendered_size_mismatch_count_;
    status.visual_observed_cache_size = static_cast<uint64_t>(last_visual_observed_cache_size_);
    status.visual_observed_match_delta_ns = last_visual_observed_match_delta_ns_;
    status.visual_observed_nearest_delta_ns = last_visual_observed_nearest_delta_ns_;
    status.visual_observed_nearest_signed_delta_ns =
      last_visual_observed_nearest_signed_delta_ns_;
    status.visual_observed_miss_count = visual_observed_miss_count_;
    status.visual_observed_stale_count = visual_observed_stale_count_;
    status.visual_observed_size_mismatch_count = visual_observed_size_mismatch_count_;
    status.visual_depth_cache_size = static_cast<uint64_t>(last_visual_depth_cache_size_);
    status.visual_depth_dilation_px = static_cast<uint32_t>(sparse_lidar_depth_dilation_px_);
    status.visual_depth_match_delta_ns = last_visual_depth_match_delta_ns_;
    status.visual_depth_miss_count = visual_depth_miss_count_;
    status.visual_depth_stale_count = visual_depth_stale_count_;
    status.visual_depth_size_mismatch_count = visual_depth_size_mismatch_count_;
    status.visual_depth_embedded_observed_matches =
      visual_depth_embedded_observed_matches_;
    status.visual_depth_observed_stamp_matches =
      visual_depth_observed_stamp_matches_;
    status.visual_depth_source_pointcloud_fallback_queries =
      visual_depth_source_pointcloud_fallback_queries_;
    status.visual_depth_source_pointcloud_fallback_matches =
      visual_depth_source_pointcloud_fallback_matches_;
    status.visual_depth_source_pointcloud_fallback_misses =
      visual_depth_source_pointcloud_fallback_misses_;
    status.visual_alignment_pending_queue_size =
      static_cast<uint64_t>(pending_visual_alignment_factors_.size());
    status.visual_se3_photometric_pending_queue_size =
      static_cast<uint64_t>(pending_visual_se3_photometric_factors_.size());
    status.visual_alignment_pending_stale_drops = visual_alignment_pending_stale_drops_;
    status.visual_se3_photometric_pending_stale_drops = visual_se3_photometric_pending_stale_drops_;
    status.visual_alignment_pending_queue_trim_drops =
      visual_alignment_pending_queue_trim_drops_;
    status.visual_se3_photometric_pending_queue_trim_drops =
      visual_se3_photometric_pending_queue_trim_drops_;
    status.visual_alignment_pending_expired_drops = visual_alignment_pending_expired_drops_;
    status.visual_se3_photometric_pending_expired_drops =
      visual_se3_photometric_pending_expired_drops_;
    status.visual_alignment_pending_future_deferrals =
      visual_alignment_pending_future_deferrals_;
    status.visual_se3_photometric_pending_future_deferrals =
      visual_se3_photometric_pending_future_deferrals_;
    status.visual_pair_processed_count = visual_pair_processed_count_;
    status.visual_pair_duplicate_count = visual_pair_duplicate_count_;
    status.visual_alignment_valid = last_visual_alignment_.valid;
    status.visual_alignment_saturated = last_visual_alignment_saturated_;
    status.visual_alignment_saturated_count = visual_alignment_saturated_count_;
    status.visual_alignment_effective_weight = last_visual_alignment_effective_weight_;
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
    status.visual_se3_photometric_pose_corrections = se3_photometric_pose_correction_count_;
    status.visual_se3_photometric_pose_correction_stamp_delta_ns =
      last_se3_photometric_pose_correction_stamp_delta_ns_;
    status.visual_se3_photometric_pose_correction_translation_m =
      last_se3_photometric_pose_correction_translation_m_;
    status.visual_se3_photometric_pose_correction_rotation_rad =
      last_se3_photometric_pose_correction_rotation_rad_;
    const bool se3_sample_quality_valid =
      last_visual_se3_photometric_sampled_depth_pixels_ > 0U &&
      (static_cast<double>(last_visual_se3_photometric_accepted_pixels_) /
      static_cast<double>(last_visual_se3_photometric_sampled_depth_pixels_)) >=
      se3_photometric_min_sample_inlier_ratio_ &&
      (se3_photometric_max_mean_abs_residual_for_factor_ <= 0.0 ||
      last_visual_se3_photometric_mean_abs_residual_ <=
      se3_photometric_max_mean_abs_residual_for_factor_) &&
      last_visual_se3_photometric_coverage_tiles_ >=
      static_cast<size_t>(se3_photometric_min_coverage_tiles_);
    const bool se3_photometric_valid =
      last_visual_se3_photometric_linearization_.valid &&
      last_visual_se3_photometric_linearization_.sample_count >=
      static_cast<size_t>(se3_photometric_min_samples_) &&
      se3_photometric_hessian_is_healthy(last_visual_se3_photometric_linearization_) &&
      se3_sample_quality_valid;
    status.visual_se3_photometric_valid = se3_photometric_valid;
    status.visual_se3_photometric_total_batches = visual_se3_photometric_total_batches_;
    status.visual_se3_photometric_valid_batches = visual_se3_photometric_valid_batches_;
    status.visual_se3_photometric_insufficient_sample_batches =
      visual_se3_photometric_insufficient_sample_batches_;
    status.visual_se3_photometric_degenerate_batches =
      visual_se3_photometric_degenerate_batches_;
    status.visual_se3_photometric_quality_rejected_batches =
      visual_se3_photometric_quality_rejected_batches_;
    status.visual_se3_photometric_total_candidates =
      visual_se3_photometric_total_candidate_pixels_;
    status.visual_se3_photometric_total_samples =
      visual_se3_photometric_total_accepted_pixels_;
    status.visual_se3_photometric_candidates =
      static_cast<uint64_t>(last_visual_se3_photometric_candidate_pixels_);
    status.visual_se3_photometric_sampled_depth =
      static_cast<uint64_t>(last_visual_se3_photometric_sampled_depth_pixels_);
    status.visual_se3_photometric_samples =
      static_cast<uint64_t>(last_visual_se3_photometric_accepted_pixels_);
    status.visual_se3_photometric_rejected_depth =
      static_cast<uint64_t>(last_visual_se3_photometric_rejected_depth_pixels_);
    status.visual_se3_photometric_rejected_gradient =
      static_cast<uint64_t>(last_visual_se3_photometric_rejected_gradient_pixels_);
    status.visual_se3_photometric_rejected_residual =
      static_cast<uint64_t>(last_visual_se3_photometric_rejected_residual_pixels_);
    status.visual_se3_photometric_coverage_tiles =
      static_cast<uint64_t>(last_visual_se3_photometric_coverage_tiles_);
    status.visual_se3_photometric_coverage_total_tiles =
      static_cast<uint64_t>(last_visual_se3_photometric_coverage_total_tiles_);
    status.visual_se3_photometric_inlier_ratio = last_visual_se3_photometric_candidate_pixels_ > 0U
      ? static_cast<double>(last_visual_se3_photometric_accepted_pixels_) /
      static_cast<double>(last_visual_se3_photometric_candidate_pixels_)
      : 0.0;
    status.visual_se3_photometric_sample_inlier_ratio =
      last_visual_se3_photometric_sampled_depth_pixels_ > 0U
      ? static_cast<double>(last_visual_se3_photometric_accepted_pixels_) /
      static_cast<double>(last_visual_se3_photometric_sampled_depth_pixels_)
      : 0.0;
    status.visual_se3_photometric_mean_abs_residual =
      last_visual_se3_photometric_mean_abs_residual_;
    status.visual_se3_photometric_cost = se3_photometric_valid
      ? last_visual_se3_photometric_linearization_.cost
      : 0.0;
    status.visual_se3_photometric_step_norm = se3_photometric_valid
      ? last_visual_se3_photometric_linearization_.gauss_newton_step.norm()
      : 0.0;
    status.visual_se3_photometric_hessian_rank =
      static_cast<uint64_t>(last_visual_se3_photometric_linearization_.hessian_rank);
    status.visual_se3_photometric_hessian_min_singular_value =
      last_visual_se3_photometric_linearization_.hessian_min_singular_value;
    status.visual_se3_photometric_hessian_max_singular_value =
      last_visual_se3_photometric_linearization_.hessian_max_singular_value;
    status.visual_se3_photometric_hessian_condition_number =
      last_visual_se3_photometric_linearization_.hessian_condition_number;
    status.visual_se3_photometric_last_accepted_hessian_rank =
      static_cast<uint64_t>(last_accepted_visual_se3_photometric_hessian_rank_);
    status.visual_se3_photometric_last_accepted_hessian_min_singular_value =
      last_accepted_visual_se3_photometric_hessian_min_singular_value_;
    status.visual_se3_photometric_last_accepted_hessian_max_singular_value =
      last_accepted_visual_se3_photometric_hessian_max_singular_value_;
    status.visual_se3_photometric_last_accepted_hessian_condition_number =
      last_accepted_visual_se3_photometric_hessian_condition_number_;
    status.visual_se3_photometric_last_accepted_sampled_depth =
      static_cast<uint64_t>(last_accepted_visual_se3_photometric_sampled_depth_pixels_);
    status.visual_se3_photometric_last_accepted_samples =
      static_cast<uint64_t>(last_accepted_visual_se3_photometric_accepted_pixels_);
    status.visual_se3_photometric_last_accepted_sample_inlier_ratio =
      last_accepted_visual_se3_photometric_sample_inlier_ratio_;
    status.visual_se3_photometric_last_accepted_coverage_tiles =
      static_cast<uint64_t>(last_accepted_visual_se3_photometric_coverage_tiles_);
    status.visual_se3_photometric_last_accepted_coverage_total_tiles =
      static_cast<uint64_t>(last_accepted_visual_se3_photometric_coverage_total_tiles_);
    status.visual_se3_photometric_last_accepted_mean_abs_residual =
      last_accepted_visual_se3_photometric_mean_abs_residual_;
    status.visual_se3_photometric_last_accepted_step_norm =
      last_accepted_visual_se3_photometric_step_norm_;
    tracking_status_pub_->publish(status);
  }

  std::string raw_image_topic_;
  std::string raw_camera_info_topic_;
  std::string raw_depth_topic_;
  std::string raw_pointcloud_topic_;
  std::string raw_imu_topic_;
  std::string external_odometry_prior_topic_;
  bool enable_pointcloud_imu_wait_{true};
  int64_t pointcloud_imu_wait_tolerance_ns_{0};
  int pointcloud_imu_wait_queue_size_{4};
  std::string image_topic_;
  std::string camera_info_topic_;
  std::string depth_topic_;
  std::string pointcloud_topic_;
  std::string pose_topic_;
  std::string odometry_topic_;
  std::string path_topic_;
  std::string tracking_status_topic_;
  std::string rendered_image_topic_;
  std::string rendered_feedback_topic_;
  std::string deterministic_bag_path_;
  std::string deterministic_feedback_bag_path_;
  std::string output_tum_path_;
  std::ofstream output_tum_stream_;
  std::string gaussian_map_topic_;
  int gaussian_snapshot_qos_depth_{64};
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
  std::string rendered_image_qos_reliability_{"reliable"};
  std::string rendered_image_qos_durability_{"transient_local"};
  int rendered_image_qos_depth_{1};
  std::string rendered_feedback_qos_reliability_{"reliable"};
  std::string rendered_feedback_qos_durability_{"volatile"};
  int rendered_feedback_qos_depth_{128};
  bool enable_rendered_feedback_contract_{false};
  bool enable_rendered_feedback_ingress_queue_{true};
  int rendered_feedback_ingress_queue_size_{512};
  int rendered_feedback_ingress_drain_max_per_cycle_{64};
  int rendered_feedback_ingress_drain_period_ms_{5};
  bool serialize_callbacks_{true};
  bool enable_visual_factor_{true};
  bool enable_gaussian_snapshot_{true};
  bool enable_external_odometry_prior_{false};
  int visual_max_pixels_{200000};
  int64_t visual_factor_max_dt_ns_{150000000LL};
  bool enable_visual_factor_time_interpolation_{false};
  bool enable_visual_cache_reconciliation_{false};
  bool visual_cache_reconciliation_monotonic_unique_{false};
  bool visual_pair_monotonic_unique_{false};
  bool enable_visual_watermark_pair_scheduler_{false};
  int visual_watermark_pair_scheduler_max_pairs_per_pointcloud_{2};
  bool enable_visual_callback_factor_ingest_{false};
  bool enable_rendered_feedback_watermark_queue_{false};
  bool defer_future_visual_factors_until_active_{false};
  bool enable_visual_adaptive_state_retention_{false};
  int visual_adaptive_state_retention_margin_states_{4};
  int visual_adaptive_state_retention_max_states_{64};
  bool enable_visual_expired_factor_projection_{false};
  bool enable_visual_marginalization_prior_{false};
  bool enable_visual_marginalization_prior_batching_{false};
  bool enable_visual_marginalization_prior_saturation_gate_{false};
  bool visual_marginalization_prior_saturation_gate_visual_factors_{true};
  bool visual_marginalization_prior_saturation_gate_se3_factors_{true};
  bool enable_visual_alignment_saturation_axis_mask_{false};
  bool visual_marginalization_prior_zero_bias_columns_{false};
  bool enable_visual_factor_reference_snapshot_{false};
  bool enable_rendered_feedback_source_pose_reference_{false};
  double visual_expired_factor_projection_max_age_s_{5.0};
  bool visual_cache_reconciliation_defer_to_pointcloud_{false};
  bool visual_pair_processing_defer_to_pointcloud_{false};
  int64_t visual_depth_max_dt_ns_{0LL};
  int depth_frame_cache_size_{8};
  int sparse_lidar_depth_dilation_px_{1};
  int rendered_frame_cache_size_{8};
  int observed_frame_cache_size_{64};
  int visual_pending_factor_queue_size_{64};
  bool enable_visual_factor_quality_weighting_{false};
  double visual_factor_quality_min_weight_scale_{0.25};
  bool enable_visual_factor_quality_selection_{false};
  bool enable_visual_factor_quality_reference_cap_{true};
  int visual_factor_quality_selection_max_per_reference_{2};
  double visual_factor_quality_selection_start_after_s_{0.0};
  Eigen::Vector3d p_i_c_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond q_i_c_{Eigen::Quaterniond::Identity()};
  int visual_alignment_max_shift_px_{8};
  std::string visual_alignment_score_mode_{"rmse"};
  gaussian_lic_tracking::VisualAlignmentMetric visual_alignment_metric_{
    gaussian_lic_tracking::VisualAlignmentMetric::kRmse};
  std::string visual_alignment_factor_source_name_{"search"};
  VisualAlignmentFactorSource visual_alignment_factor_source_{VisualAlignmentFactorSource::kSearch};
  std::string visual_factor_source_id_mode_name_{"legacy_8bit"};
  VisualFactorSourceIdMode visual_factor_source_id_mode_{VisualFactorSourceIdMode::kLegacy8Bit};
  std::string visual_factor_reference_stamp_mode_name_{"observed"};
  VisualFactorReferenceStampMode visual_factor_reference_stamp_mode_{
    VisualFactorReferenceStampMode::kObserved};
  bool enable_rendered_feedback_source_motion_factor_{false};
  bool enable_rendered_feedback_source_motion_marginalized_prior_{false};
  double rendered_feedback_source_motion_translation_weight_{0.0};
  double rendered_feedback_source_motion_rotation_weight_{0.0};
  double rendered_feedback_source_motion_huber_delta_m_{0.1};
  double rendered_feedback_source_motion_rotation_huber_delta_rad_{0.05};
  double rendered_feedback_source_motion_min_dt_s_{0.0};
  double rendered_feedback_source_motion_max_dt_s_{1.0};
  bool rendered_feedback_source_motion_in_from_frame_{false};
  bool enable_visual_alignment_window_factor_{true};
  double visual_alignment_meters_per_pixel_{0.01};
  double visual_alignment_window_weight_{1.0};
  double visual_alignment_huber_delta_m_{0.05};
  double visual_alignment_saturation_margin_px_{0.0};
  double visual_alignment_saturated_weight_scale_{1.0};
  bool enable_se3_photometric_window_factor_{true};
  double se3_photometric_window_weight_{1.0};
  double se3_photometric_factor_huber_delta_{1.0};
  int se3_photometric_max_samples_{2000};
  int se3_photometric_min_samples_{16};
  int se3_photometric_min_hessian_rank_{3};
  double se3_photometric_min_depth_m_{0.05};
  double se3_photometric_max_depth_m_{200.0};
  double se3_photometric_min_gradient_{1.0e-4};
  bool se3_photometric_rank_samples_by_gradient_{false};
  bool se3_photometric_use_rendered_gradient_{false};
  double se3_photometric_huber_delta_{0.15};
  double se3_photometric_max_abs_residual_{1.0};
  bool enable_se3_photometric_pose_correction_{false};
  double se3_photometric_pose_correction_gain_{0.1};
  double se3_photometric_pose_correction_max_translation_m_{0.02};
  double se3_photometric_pose_correction_max_rotation_rad_{0.01};
  int64_t se3_photometric_pose_correction_max_dt_ns_{0};
  double se3_photometric_max_hessian_condition_{1.0e12};
  double se3_photometric_min_sample_inlier_ratio_{0.25};
  double se3_photometric_max_mean_abs_residual_for_factor_{0.0};
  int se3_photometric_coverage_grid_cols_{4};
  int se3_photometric_coverage_grid_rows_{4};
  int se3_photometric_min_coverage_tiles_{4};
  bool enable_lio_factor_{true};
  bool enable_lidar_plane_factor_{true};
  bool enable_lidar_deskew_{true};
  bool enable_sliding_window_optimizer_{true};
  bool enable_gaussian_snapshot_lidar_factor_{true};
  bool enable_gaussian_snapshot_lidar_plane_factor_{false};
  std::string lidar_time_field_{"auto"};
  std::string lidar_time_unit_{"auto"};
  std::string lidar_time_mode_{"auto"};
  double lidar_scan_order_duration_s_{0.1};
  double lidar_max_abs_point_time_offset_s_{0.25};
  int imu_history_size_{12000};
  double tracking_max_pose_step_m_{0.25};
  bool enable_pre_lio_tracking_step_guard_{true};
  bool enable_post_ba_tracking_step_guard_{true};
  double pre_lio_tracking_max_pose_step_m_{0.0};
  double post_ba_tracking_max_pose_step_m_{0.0};
  double post_ba_step_guard_confidence_max_pose_step_m_{0.0};
  int post_ba_step_guard_confidence_warmup_marginalizations_{0};
  double post_ba_step_guard_min_lidar_confidence_{0.6};
  double post_ba_step_guard_min_visual_inlier_ratio_{0.85};
  double post_ba_step_guard_max_visual_residual_{0.3};
  int post_ba_step_guard_min_visual_coverage_tiles_{8};
  double post_ba_step_guard_reject_to_pre_ba_over_m_{0.0};
  double post_ba_step_guard_pre_ba_agreement_max_pose_step_m_{0.0};
  int post_ba_step_guard_pre_ba_agreement_late_start_marginalizations_{0};
  double post_ba_step_guard_pre_ba_agreement_late_max_pose_step_m_{0.0};
  double post_ba_step_guard_pre_ba_agreement_min_cosine_{0.85};
  double post_ba_step_guard_pre_ba_agreement_max_delta_m_{0.05};
  double post_ba_step_guard_pre_ba_agreement_margin_m_{0.0};
  double post_ba_step_guard_pre_ba_blend_on_clamp_{0.0};
  double tracking_step_guard_velocity_scale_{0.0};
  double pre_lio_tracking_step_guard_velocity_scale_{0.0};
  double post_ba_tracking_step_guard_velocity_scale_{0.0};
  double tracking_step_guard_acceleration_mps2_{0.0};
  double tracking_step_guard_max_velocity_mps_{0.0};
  double tracking_step_guard_margin_m_{0.0};
  bool enable_imu_gravity_autocalibration_{true};
  int imu_gravity_autocalibration_samples_{50};
  double imu_gravity_magnitude_m_s2_{9.80665};
  double imu_linear_acceleration_scale_{1.0};
  double imu_gravity_autocalibration_min_norm_m_s2_{6.0};
  double imu_gravity_autocalibration_max_norm_m_s2_{14.0};
  Eigen::Vector3d configured_imu_gravity_w_{0.0, 0.0, -9.80665};
  int64_t trajectory_control_interval_ns_{50000000LL};
  int64_t external_odometry_prior_max_dt_ns_{100000000LL};
  int external_odometry_prior_cache_size_{128};
  int sliding_window_max_states_{12};
  size_t sliding_window_effective_max_states_{12U};
  int sliding_window_optimize_every_n_frames_{1};
  int sliding_window_max_iterations_{3};
  double sliding_window_max_rotation_step_rad_{0.5};
  double sliding_window_max_translation_step_m_{1.0};
  double sliding_window_max_velocity_step_mps_{5.0};
  double sliding_window_max_bias_step_{1.0};
  double sliding_window_max_feedback_translation_m_{1.0};
  double sliding_window_max_feedback_rotation_rad_{0.5};
  double sliding_window_max_feedback_velocity_mps_{5.0};
  double sliding_window_max_feedback_velocity_norm_mps_{5.0};
  double sliding_window_max_feedback_gyro_bias_norm_{0.5};
  double sliding_window_max_feedback_accel_bias_norm_{2.5};
  double sliding_window_max_feedback_gyro_bias_step_{0.0};
  double sliding_window_max_feedback_accel_bias_step_{0.0};
  int sliding_window_min_bias_feedback_visual_factors_{0};
  SlidingWindowBiasFeedbackOwnership sliding_window_bias_feedback_ownership_{
    SlidingWindowBiasFeedbackOwnership::kOptimized};
  bool sliding_window_sync_guarded_pose_state_{false};
  double sliding_window_guarded_pose_prior_translation_weight_{0.0};
  double sliding_window_guarded_pose_prior_rotation_weight_{0.0};
  double sliding_window_max_normal_equation_condition_{1.0e13};
  double sliding_window_min_normal_equation_rank_ratio_{0.8};
  double sliding_window_max_state_gap_s_{1.0};
  double sliding_window_marginalization_prior_weight_{1.0};
  double gaussian_snapshot_lidar_factor_weight_{1.0};
  double gaussian_snapshot_lidar_nearest_distance_m_{0.0};
  bool gaussian_snapshot_lidar_residual_preweight_{true};
  bool enable_gaussian_snapshot_lidar_pose_correction_{false};
  double gaussian_snapshot_lidar_pose_correction_gain_{0.3};
  double gaussian_snapshot_lidar_pose_correction_max_translation_m_{0.05};
  double gaussian_snapshot_lidar_pose_correction_max_rotation_rad_{0.02};
  double gaussian_snapshot_lidar_pose_correction_min_match_ratio_{0.0};
  double gaussian_snapshot_lidar_pose_correction_max_mean_residual_m_{0.0};
  int gaussian_snapshot_lidar_pose_correction_coverage_grid_cols_{1};
  int gaussian_snapshot_lidar_pose_correction_coverage_grid_rows_{1};
  int gaussian_snapshot_lidar_pose_correction_min_coverage_tiles_{0};
  double gaussian_snapshot_lidar_pose_correction_bidirectional_max_distance_m_{0.0};
  double gaussian_snapshot_lidar_plane_factor_weight_{1.0};
  double gaussian_snapshot_lidar_min_opacity_{0.01};
  double gaussian_snapshot_lidar_plane_min_anisotropy_{0.25};
  double sliding_window_imu_weight_{1.0};
  double sliding_window_imu_rotation_weight_{1.0};
  double sliding_window_imu_velocity_weight_{1.0};
  double sliding_window_imu_position_weight_{1.0};
  double sliding_window_imu_velocity_prior_weight_{0.0};
  double sliding_window_gyro_bias_prior_weight_{0.0};
  double sliding_window_accel_bias_prior_weight_{0.0};
  double sliding_window_imu_max_extrapolation_s_{0.02};
  double sliding_window_bias_weight_{1.0};
  double sliding_window_gyro_bias_weight_{1.0};
  double sliding_window_accel_bias_weight_{1.0};
  double sliding_window_bias_random_walk_reference_dt_s_{0.0};
  double sliding_window_gyro_bias_random_walk_sigma_{0.0};
  double sliding_window_accel_bias_random_walk_sigma_{0.0};
  double sliding_window_pose_translation_weight_{2.0};
  double sliding_window_pose_rotation_weight_{2.0};
  double external_odometry_prior_translation_weight_{4.0};
  double external_odometry_prior_rotation_weight_{4.0};
  bool enable_sliding_window_smoothness_factor_{true};
  double sliding_window_smoothness_rotation_weight_{0.1};
  double sliding_window_smoothness_position_weight_{0.1};
  double sliding_window_smoothness_velocity_weight_{0.1};
  double sliding_window_smoothness_position_velocity_weight_{0.0};
  double sliding_window_smoothness_bias_weight_{0.1};
  bool sliding_window_smoothness_use_motion_targets_{false};
  int sliding_window_smoothness_motion_target_min_visual_factors_{0};
  int sliding_window_smoothness_motion_target_min_se3_photometric_factors_{0};
  int sliding_window_smoothness_motion_target_recent_window_{0};
  int sliding_window_smoothness_motion_target_min_recent_visual_factors_{0};
  int sliding_window_smoothness_motion_target_min_recent_se3_photometric_factors_{0};
  double sliding_window_smoothness_motion_target_start_after_s_{0.0};
  double sliding_window_smoothness_motion_target_max_rotation_rate_delta_radps_{0.25};
  double sliding_window_smoothness_motion_target_max_position_rate_delta_mps_{0.5};
  double sliding_window_smoothness_motion_target_max_velocity_acceleration_delta_mps2_{2.0};
  bool enable_sliding_window_relative_translation_factor_{false};
  double sliding_window_relative_translation_weight_{0.0};
  double sliding_window_relative_translation_huber_delta_m_{0.1};
  bool sliding_window_relative_translation_in_from_frame_{false};
  double sliding_window_relative_rotation_weight_{0.0};
  double sliding_window_relative_rotation_huber_delta_rad_{0.05};
  bool enable_sliding_window_relative_distance_factor_{false};
  double sliding_window_relative_distance_weight_{0.0};
  double sliding_window_relative_distance_huber_delta_m_{0.1};
  bool enable_sliding_window_multihop_relative_translation_factor_{false};
  double sliding_window_multihop_relative_translation_weight_{0.0};
  double sliding_window_multihop_relative_translation_huber_delta_m_{0.15};
  bool sliding_window_multihop_relative_translation_in_from_frame_{false};
  double sliding_window_multihop_relative_rotation_weight_{0.0};
  double sliding_window_multihop_relative_rotation_huber_delta_rad_{0.08};
  bool enable_sliding_window_multihop_relative_distance_factor_{false};
  double sliding_window_multihop_relative_distance_weight_{0.0};
  double sliding_window_multihop_relative_distance_huber_delta_m_{0.15};
  double sliding_window_multihop_relative_translation_min_dt_s_{0.45};
  double sliding_window_multihop_relative_translation_max_dt_s_{1.05};
  int sliding_window_multihop_relative_translation_max_factors_{1};
  bool enable_sliding_window_delayed_published_multihop_relative_translation_factor_{false};
  double sliding_window_delayed_published_multihop_start_after_s_{0.0};
  int sliding_window_delayed_published_multihop_max_factors_{1};
  RelativeMotionHistorySource sliding_window_relative_motion_history_source_{
    RelativeMotionHistorySource::kPreBa};
  double sliding_window_relative_motion_history_published_after_s_{0.0};
  int lidar_min_points_{32};
  int lidar_max_frame_points_{2000};
  int lidar_max_map_points_{20000};
  double lidar_nearest_distance_m_{0.35};
  double lidar_correction_gain_{0.7};
  double lidar_max_correction_m_{0.25};
  double lidar_max_rotation_rad_{0.08};
  double lidar_robust_kernel_m_{0.15};
  int lidar_pose_factor_iterations_{1};
  double lidar_window_point_factor_weight_{1.0};
  double lidar_window_plane_factor_weight_{1.0};
  double lidar_window_confidence_power_{1.0};
  int lidar_plane_min_neighbors_{5};
  double lidar_plane_max_condition_{0.2};
  bool enable_lidar_line_factor_{false};
  double lidar_line_max_condition_{0.2};
  double lidar_window_line_factor_weight_{1.0};
  double lidar_keyframe_translation_m_{0.25};
  Eigen::Vector3d p_i_l_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond q_i_l_{Eigen::Quaterniond::Identity()};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr external_odometry_prior_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rendered_image_sub_;
  rclcpp::Subscription<gaussian_lic_msgs::msg::RenderedFeedback>::SharedPtr rendered_feedback_sub_;
  rclcpp::Subscription<gaussian_lic_msgs::msg::GaussianArray>::SharedPtr gaussian_map_sub_;
  rclcpp::TimerBase::SharedPtr rendered_feedback_ingress_timer_;
  std::mutex callback_mutex_;
  std::mutex rendered_feedback_ingress_mutex_;
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
  std::deque<ExternalPosePrior> external_odometry_priors_;
  std::deque<PendingPointCloud> pending_pointclouds_waiting_for_imu_;
  std::vector<PendingImuMeasurement> pending_imu_gravity_samples_;
  gaussian_lic_tracking::ImuPreintegrator sliding_window_preintegrator_;
  gaussian_lic_tracking::ImuBias sliding_window_bias_;
  bool sliding_window_preintegrator_initialized_{false};
  bool has_sliding_window_state_{false};
  std::optional<int64_t> sliding_window_start_stamp_ns_;
  int64_t last_sliding_window_stamp_ns_{0};
  std::optional<int64_t> previous_sliding_window_stamp_ns_;
  std::optional<int64_t> last_trajectory_control_stamp_ns_;
  uint64_t num_sliding_window_imu_reanchors_{0};
  uint64_t sliding_window_total_imu_factors_{0};
  uint64_t sliding_window_total_imu_preintegration_samples_{0};
  double sliding_window_total_imu_preintegration_dt_s_{0.0};
  uint64_t sliding_window_total_visual_factors_{0};
  uint64_t sliding_window_total_se3_photometric_factors_{0};
  uint64_t sliding_window_point_factor_skip_count_{0};
  uint64_t sliding_window_plane_factor_skip_count_{0};
  uint64_t sliding_window_visual_factor_skip_count_{0};
  uint64_t sliding_window_se3_photometric_factor_skip_count_{0};
  uint64_t sliding_window_smoothness_factor_skip_count_{0};
  uint64_t sliding_window_smoothness_motion_target_applied_count_{0};
  uint64_t sliding_window_smoothness_motion_target_support_skip_count_{0};
  uint64_t sliding_window_smoothness_motion_target_recent_support_skip_count_{0};
  uint64_t sliding_window_smoothness_motion_target_warmup_skip_count_{0};
  uint64_t sliding_window_smoothness_motion_target_history_miss_count_{0};
  uint64_t sliding_window_smoothness_motion_target_invalid_count_{0};
  uint64_t sliding_window_smoothness_motion_target_clamp_count_{0};
  uint64_t sliding_window_delayed_published_multihop_relative_factor_count_{0};
  uint64_t sliding_window_imu_factor_skip_count_{0};
  uint64_t sliding_window_imu_time_gap_skip_count_{0};
  uint64_t sliding_window_feedback_update_count_{0};
  uint64_t sliding_window_guarded_state_sync_count_{0};
  uint64_t sliding_window_guarded_pose_prior_count_{0};
  uint64_t sliding_window_bias_feedback_hold_count_{0};
  uint64_t sliding_window_bias_feedback_ownership_hold_count_{0};
  int64_t last_sliding_window_feedback_stamp_ns_{0};
  double last_sliding_window_feedback_translation_delta_m_{0.0};
  double last_sliding_window_feedback_rotation_delta_rad_{0.0};
  double last_sliding_window_feedback_velocity_delta_mps_{0.0};
  uint64_t last_sliding_window_imu_preintegration_samples_{0};
  double last_sliding_window_imu_preintegration_dt_s_{0.0};
  double last_sliding_window_imu_preintegration_extrapolated_dt_s_{0.0};
  int64_t last_sliding_window_imu_preintegration_start_stamp_ns_{0};
  int64_t last_sliding_window_imu_preintegration_end_stamp_ns_{0};
  uint64_t sliding_window_optimization_skip_count_{0};
  uint64_t sliding_window_frame_count_{0};
  uint64_t sliding_window_invalid_optimized_states_{0};
  uint64_t trajectory_deskew_queries_{0};
  uint64_t trajectory_deskew_hits_{0};
  uint64_t trajectory_control_pose_skip_count_{0};
  uint64_t tracking_step_guard_clamp_count_{0};
  uint64_t tracking_step_guard_pre_lio_clamp_count_{0};
  double last_sliding_window_smoothness_motion_target_rotation_rate_delta_norm_{0.0};
  double last_sliding_window_smoothness_motion_target_position_rate_delta_norm_{0.0};
  double last_sliding_window_smoothness_motion_target_velocity_acceleration_delta_norm_{0.0};
  double sliding_window_smoothness_motion_target_max_rotation_rate_delta_norm_{0.0};
  double sliding_window_smoothness_motion_target_max_position_rate_delta_norm_{0.0};
  double sliding_window_smoothness_motion_target_max_velocity_acceleration_delta_norm_{0.0};
  uint64_t last_sliding_window_smoothness_motion_target_recent_visual_factors_{0};
  uint64_t last_sliding_window_smoothness_motion_target_recent_se3_photometric_factors_{0};
  std::deque<std::pair<size_t, size_t>> motion_target_support_history_;
  uint64_t tracking_step_guard_post_ba_clamp_count_{0};
  uint64_t tracking_step_guard_post_ba_rejection_count_{0};
  uint64_t tracking_step_guard_pre_ba_agreement_release_count_{0};
  uint64_t tracking_step_guard_late_pre_ba_agreement_release_count_{0};
  double last_tracking_step_guard_raw_step_m_{0.0};
  double last_tracking_step_guard_allowed_step_m_{0.0};
  double last_tracking_step_guard_dt_s_{0.0};
  double last_tracking_step_guard_reference_speed_mps_{0.0};
  double last_tracking_step_guard_confidence_score_{0.0};
  double last_tracking_step_guard_pre_ba_agreement_limit_m_{0.0};
  bool last_tracking_step_guard_pre_ba_agreement_late_active_{false};
  gaussian_lic_tracking::LidarFactor lidar_factor_;
  gaussian_lic_tracking::TrajectoryPose last_lidar_keyframe_pose_;
  std::optional<gaussian_lic_tracking::TrajectoryPose> last_output_tracking_pose_;
  std::deque<gaussian_lic_tracking::TrajectoryPose> relative_motion_pose_history_;
  std::deque<gaussian_lic_tracking::TrajectoryPose> published_relative_motion_pose_history_;
  bool has_lidar_keyframe_{false};
  gaussian_lic_tracking::GaussianSnapshot gaussian_snapshot_;
  gaussian_lic_tracking::VisualFactor visual_factor_;
  std::deque<gaussian_lic_tracking::VisualFrame> rendered_frame_cache_;
  std::deque<VisualPairKey> processed_visual_pairs_;
  size_t last_visual_rendered_cache_size_{0};
  int64_t last_visual_rendered_match_delta_ns_{0};
  int64_t last_visual_rendered_nearest_delta_ns_{0};
  int64_t last_visual_rendered_nearest_signed_delta_ns_{0};
  uint64_t visual_rendered_miss_count_{0};
  uint64_t visual_rendered_stale_count_{0};
  uint64_t visual_rendered_size_mismatch_count_{0};
  size_t last_visual_observed_cache_size_{0};
  int64_t last_visual_observed_match_delta_ns_{0};
  int64_t last_visual_observed_nearest_delta_ns_{0};
  int64_t last_visual_observed_nearest_signed_delta_ns_{0};
  uint64_t visual_observed_miss_count_{0};
  uint64_t visual_observed_stale_count_{0};
  uint64_t visual_observed_size_mismatch_count_{0};
  gaussian_lic_tracking::VisualResidual last_visual_residual_;
  gaussian_lic_tracking::VisualAlignment last_visual_alignment_;
  gaussian_lic_tracking::VisualPhotometricLinearization last_visual_photometric_linearization_;
  bool last_visual_alignment_saturated_{false};
  double last_visual_alignment_effective_weight_{0.0};
  gaussian_lic_tracking::VisualSe3PhotometricLinearization last_visual_se3_photometric_linearization_;
  std::deque<QueuedRenderedFeedbackPair> rendered_feedback_watermark_queue_;
  std::deque<gaussian_lic_msgs::msg::RenderedFeedback::ConstSharedPtr>
    rendered_feedback_ingress_queue_;
  std::deque<PendingVisualAlignmentFactor> pending_visual_alignment_factors_;
  std::deque<PendingSe3PhotometricFactor> pending_visual_se3_photometric_factors_;
  std::deque<PendingRenderedFeedbackSourceMotionFactor>
  pending_rendered_feedback_source_motion_factors_;
  std::optional<RenderedFeedbackSourceMotionPose> last_rendered_feedback_source_motion_pose_;
  size_t last_visual_se3_photometric_candidate_pixels_{0};
  size_t last_visual_se3_photometric_sampled_depth_pixels_{0};
  size_t last_visual_se3_photometric_accepted_pixels_{0};
  size_t last_visual_se3_photometric_rejected_depth_pixels_{0};
  size_t last_visual_se3_photometric_rejected_gradient_pixels_{0};
  size_t last_visual_se3_photometric_rejected_residual_pixels_{0};
  size_t last_visual_se3_photometric_coverage_tiles_{0};
  size_t last_visual_se3_photometric_coverage_total_tiles_{0};
  double last_visual_se3_photometric_mean_abs_residual_{0.0};
  size_t last_accepted_visual_se3_photometric_hessian_rank_{0};
  double last_accepted_visual_se3_photometric_hessian_min_singular_value_{0.0};
  double last_accepted_visual_se3_photometric_hessian_max_singular_value_{0.0};
  double last_accepted_visual_se3_photometric_hessian_condition_number_{0.0};
  size_t last_accepted_visual_se3_photometric_sampled_depth_pixels_{0};
  size_t last_accepted_visual_se3_photometric_accepted_pixels_{0};
  double last_accepted_visual_se3_photometric_sample_inlier_ratio_{0.0};
  size_t last_accepted_visual_se3_photometric_coverage_tiles_{0};
  size_t last_accepted_visual_se3_photometric_coverage_total_tiles_{0};
  double last_accepted_visual_se3_photometric_mean_abs_residual_{0.0};
  double last_accepted_visual_se3_photometric_step_norm_{0.0};
  uint64_t se3_photometric_pose_correction_count_{0};
  int64_t last_se3_photometric_pose_correction_stamp_delta_ns_{0};
  double last_se3_photometric_pose_correction_translation_m_{0.0};
  double last_se3_photometric_pose_correction_rotation_rad_{0.0};
  std::optional<int64_t> last_applied_se3_photometric_pose_correction_stamp_ns_;
  uint64_t last_applied_se3_photometric_pose_correction_source_id_{0};
  gaussian_lic_tracking::VisualCameraIntrinsics camera_intrinsics_;
  std::deque<DepthFrame> depth_frame_cache_;
  std::deque<gaussian_lic_tracking::VisualFrame> observed_frame_cache_;
  std::optional<int64_t> last_processed_visual_observed_stamp_ns_;
  std::optional<int64_t> last_processed_visual_rendered_stamp_ns_;
  uint64_t rendered_feedback_watermark_processed_pairs_{0};
  uint64_t rendered_feedback_watermark_deferred_pairs_{0};
  uint64_t rendered_feedback_watermark_queue_drops_{0};
  uint64_t rendered_feedback_watermark_reordered_pairs_{0};
  uint64_t rendered_feedback_source_pose_reference_factors_{0};
  uint64_t rendered_feedback_source_pose_invalid_{0};
  uint64_t rendered_feedback_source_motion_queued_factors_{0};
  uint64_t rendered_feedback_source_motion_factors_{0};
  uint64_t rendered_feedback_source_motion_invalid_{0};
  uint64_t rendered_feedback_source_motion_dt_skip_count_{0};
  uint64_t rendered_feedback_source_motion_stale_drops_{0};
  uint64_t rendered_feedback_source_motion_future_deferrals_{0};
  uint64_t rendered_feedback_source_motion_marginalized_priors_{0};
  uint64_t rendered_feedback_source_motion_marginalized_source_factors_{0};
  uint64_t rendered_feedback_source_motion_marginalized_prior_skips_{0};
  size_t last_visual_depth_cache_size_{0};
  int64_t last_visual_depth_match_delta_ns_{0};
  uint64_t visual_depth_miss_count_{0};
  uint64_t visual_depth_stale_count_{0};
  uint64_t visual_depth_size_mismatch_count_{0};
  uint64_t visual_depth_embedded_observed_matches_{0};
  uint64_t visual_depth_observed_stamp_matches_{0};
  uint64_t visual_depth_source_pointcloud_fallback_queries_{0};
  uint64_t visual_depth_source_pointcloud_fallback_matches_{0};
  uint64_t visual_depth_source_pointcloud_fallback_misses_{0};
  bool has_camera_intrinsics_{false};
  size_t last_observed_image_width_{0};
  size_t last_observed_image_height_{0};
  uint64_t visual_alignment_pending_stale_drops_{0};
  uint64_t visual_se3_photometric_pending_stale_drops_{0};
  uint64_t visual_alignment_pending_queue_trim_drops_{0};
  uint64_t visual_se3_photometric_pending_queue_trim_drops_{0};
  uint64_t visual_alignment_pending_expired_drops_{0};
  uint64_t visual_se3_photometric_pending_expired_drops_{0};
  uint64_t visual_alignment_pending_future_deferrals_{0};
  uint64_t visual_se3_photometric_pending_future_deferrals_{0};
  uint64_t visual_callback_ingested_visual_factors_{0};
  uint64_t visual_callback_ingested_se3_photometric_factors_{0};
  uint64_t visual_render_backlog_frames_{0};
  uint64_t visual_alignment_expired_projected_factors_{0};
  uint64_t visual_se3_photometric_expired_projected_factors_{0};
  uint64_t visual_expired_projection_skipped_factors_{0};
  uint64_t visual_alignment_marginalization_priors_{0};
  uint64_t visual_se3_photometric_marginalization_priors_{0};
  uint64_t visual_marginalization_prior_skipped_factors_{0};
  uint64_t visual_marginalization_prior_saturation_rejected_factors_{0};
  uint64_t visual_marginalization_prior_saturation_rejected_visual_factors_{0};
  uint64_t visual_marginalization_prior_saturation_rejected_se3_factors_{0};
  uint64_t visual_alignment_saturation_axis_masked_factors_{0};
  uint64_t visual_alignment_saturation_axis_masked_axes_{0};
  uint64_t visual_alignment_saturation_axis_mask_skipped_factors_{0};
  uint64_t visual_batched_marginalization_prior_batches_{0};
  uint64_t visual_batched_marginalization_prior_visual_factors_{0};
  uint64_t visual_batched_marginalization_prior_se3_factors_{0};
  uint64_t visual_batched_marginalization_prior_skipped_batches_{0};
  uint64_t visual_batched_marginalization_prior_skipped_factors_{0};
  uint64_t visual_alignment_interpolated_factor_count_{0};
  uint64_t visual_se3_photometric_interpolated_factor_count_{0};
  uint64_t visual_cache_reconciled_pairs_{0};
  uint64_t visual_cache_reconciled_saturated_pairs_{0};
  uint64_t visual_cache_reconciled_alignment_skipped_pairs_{0};
  uint64_t visual_cache_reconciled_alignment_photometric_fallback_pairs_{0};
  uint64_t visual_cache_reconciled_alignment_photometric_disagreement_pairs_{0};
  uint64_t visual_pair_processed_count_{0};
  uint64_t visual_pair_duplicate_count_{0};
  uint64_t visual_watermark_pair_scheduler_processed_pairs_{0};
  uint64_t visual_watermark_pair_scheduler_deferred_pairs_{0};
  uint64_t visual_alignment_saturated_count_{0};
  uint64_t visual_se3_photometric_total_batches_{0};
  uint64_t visual_se3_photometric_valid_batches_{0};
  uint64_t visual_se3_photometric_insufficient_sample_batches_{0};
  uint64_t visual_se3_photometric_degenerate_batches_{0};
  uint64_t visual_se3_photometric_quality_rejected_batches_{0};
  uint64_t visual_se3_photometric_total_candidate_pixels_{0};
  uint64_t visual_se3_photometric_total_accepted_pixels_{0};
  gaussian_lic_tracking::SlidingWindowSummary last_sliding_window_summary_;
  bool has_last_sliding_window_summary_{false};
  double last_sliding_window_optimization_duration_ms_{0.0};
  uint64_t num_raw_images_{0};
  uint64_t num_rendered_images_{0};
  uint64_t num_rendered_feedbacks_{0};
  uint64_t rendered_feedback_ingress_received_{0};
  uint64_t rendered_feedback_ingress_drained_{0};
  uint64_t rendered_feedback_ingress_drops_{0};
  size_t rendered_feedback_ingress_queue_last_size_{0};
  size_t rendered_feedback_ingress_queue_peak_size_{0};
  uint64_t rendered_feedback_embedded_observed_pairs_{0};
  uint64_t rendered_feedback_embedded_depth_pairs_{0};
  uint64_t rendered_feedback_embedded_depth_invalid_{0};
  uint64_t rendered_feedback_stamp_mismatches_{0};
  uint64_t last_rendered_feedback_frame_index_{0};
  uint64_t last_rendered_feedback_preview_index_{0};
  uint64_t rendered_feedback_frame_index_regressions_{0};
  uint64_t rendered_feedback_preview_index_regressions_{0};
  uint64_t rendered_feedback_frame_index_gap_count_{0};
  uint64_t rendered_feedback_preview_index_gap_count_{0};
  uint64_t rendered_feedback_frame_index_missing_{0};
  uint64_t rendered_feedback_preview_index_missing_{0};
  uint64_t rendered_feedback_duplicate_source_ids_{0};
  std::set<std::pair<uint64_t, uint64_t>> seen_rendered_feedback_source_ids_;
  std::optional<uint64_t> last_ordered_rendered_feedback_frame_index_;
  std::optional<uint64_t> last_ordered_rendered_feedback_preview_index_;
  int64_t last_rendered_feedback_observed_delta_ns_{0};
  int64_t last_rendered_feedback_pose_delta_ns_{0};
  int64_t last_rendered_feedback_pointcloud_delta_ns_{0};
  int64_t last_rendered_feedback_reference_stamp_ns_{0};
  int64_t last_rendered_feedback_oldest_active_state_delta_ns_{0};
  int64_t last_rendered_feedback_newest_active_state_delta_ns_{0};
  uint64_t rendered_feedback_before_active_window_{0};
  uint64_t rendered_feedback_after_active_window_{0};
  uint64_t num_raw_pointclouds_{0};
  uint64_t num_raw_imus_{0};
  uint64_t num_published_poses_{0};
  uint64_t pointcloud_imu_wait_deferred_{0};
  uint64_t pointcloud_imu_wait_released_{0};
  uint64_t pointcloud_imu_wait_dropped_{0};
  uint64_t pointcloud_imu_wait_stale_dropped_{0};
  uint64_t imu_gravity_autocalibration_samples_collected_{0};
  bool imu_gravity_autocalibrated_{false};
  bool imu_gravity_autocalibration_failed_{false};
  Eigen::Vector3d imu_gravity_autocalibration_mean_accel_{Eigen::Vector3d::Zero()};
  uint64_t external_odometry_priors_received_{0};
  uint64_t external_odometry_prior_matches_{0};
  uint64_t external_odometry_prior_misses_{0};
  uint64_t lidar_invalid_points_{0};
  uint64_t lidar_invalid_point_times_{0};
  uint64_t lidar_out_of_range_point_times_{0};
  double last_lidar_max_abs_point_time_offset_s_{0.0};
  int64_t last_image_stamp_ns_{0};
  int64_t last_pointcloud_stamp_ns_{0};
  int64_t last_imu_stamp_ns_{0};
  int64_t last_external_odometry_prior_stamp_ns_{0};
  std::optional<int64_t> last_image_input_stamp_ns_;
  std::optional<int64_t> last_depth_input_stamp_ns_;
  std::optional<int64_t> last_rendered_input_stamp_ns_;
  std::optional<int64_t> last_pointcloud_input_stamp_ns_;
  std::optional<int64_t> last_imu_input_stamp_ns_;
  std::optional<int64_t> last_external_odometry_prior_input_stamp_ns_;
  uint64_t image_stamp_regressions_{0};
  uint64_t depth_stamp_regressions_{0};
  uint64_t rendered_stamp_regressions_{0};
  uint64_t pointcloud_stamp_regressions_{0};
  uint64_t imu_stamp_regressions_{0};
  uint64_t external_odometry_prior_stamp_regressions_{0};
  uint64_t imu_invalid_measurements_{0};
  uint64_t external_odometry_prior_invalid_messages_{0};
  uint64_t camera_info_invalid_intrinsics_{0};
  uint64_t image_invalid_frames_{0};
  uint64_t depth_invalid_frames_{0};
  uint64_t rendered_invalid_frames_{0};
  uint64_t num_lidar_keyframes_{0};
  uint64_t lidar_invalid_frames_{0};
  size_t last_lidar_points_{0};
  size_t last_lidar_matches_{0};
  size_t last_window_point_correspondences_{0};
  size_t last_window_plane_correspondences_{0};
  uint64_t total_window_point_correspondences_{0};
  uint64_t total_window_plane_correspondences_{0};
  double last_lidar_mean_residual_m_{0.0};
  double last_window_point_confidence_mean_{0.0};
  double last_window_point_confidence_min_{0.0};
  double last_window_plane_confidence_mean_{0.0};
  double last_window_plane_confidence_min_{0.0};
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
  auto node = std::make_shared<TrackingNode>();
  if (!node->deterministic_bag_path().empty()) {
    node->run_deterministic_replay();
  } else {
    rclcpp::spin(node);
  }
  rclcpp::shutdown();
  return 0;
}
