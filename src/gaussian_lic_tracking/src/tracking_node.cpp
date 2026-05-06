// SPDX-License-Identifier: GPL-3.0-or-later

#include <algorithm>
#include <chrono>
#include <cinttypes>
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

}  // namespace

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
    se3_photometric_huber_delta_ = finite_nonnegative_parameter(
      "se3_photometric_huber_delta",
      declare_parameter<double>("se3_photometric_huber_delta", 0.15));
    se3_photometric_max_abs_residual_ = finite_nonnegative_parameter(
      "se3_photometric_max_abs_residual",
      declare_parameter<double>("se3_photometric_max_abs_residual", 1.0));
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
    sliding_window_max_feedback_translation_m_ = finite_nonnegative_parameter(
      "sliding_window_max_feedback_translation_m",
      declare_parameter<double>("sliding_window_max_feedback_translation_m", 1.0));
    sliding_window_max_feedback_rotation_rad_ = finite_nonnegative_parameter(
      "sliding_window_max_feedback_rotation_rad",
      declare_parameter<double>("sliding_window_max_feedback_rotation_rad", 0.5));
    sliding_window_max_feedback_velocity_mps_ = finite_nonnegative_parameter(
      "sliding_window_max_feedback_velocity_mps",
      declare_parameter<double>("sliding_window_max_feedback_velocity_mps", 5.0));
    sliding_window_max_normal_equation_condition_ = finite_positive_parameter(
      "sliding_window_max_normal_equation_condition",
      declare_parameter<double>("sliding_window_max_normal_equation_condition", 1.0e13));
    sliding_window_min_normal_equation_rank_ratio_ = finite_unit_interval_parameter(
      "sliding_window_min_normal_equation_rank_ratio",
      declare_parameter<double>("sliding_window_min_normal_equation_rank_ratio", 0.8));
    sliding_window_max_state_gap_s_ = finite_nonnegative_parameter(
      "sliding_window_max_state_gap_s",
      declare_parameter<double>("sliding_window_max_state_gap_s", 1.0));
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
    sliding_window_imu_max_extrapolation_s_ = finite_nonnegative_parameter(
      "sliding_window_imu_max_extrapolation_s",
      declare_parameter<double>("sliding_window_imu_max_extrapolation_s", 0.02));
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
    window_config.max_state_gap_s = sliding_window_max_state_gap_s_;
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

  struct PendingVisualAlignmentFactor
  {
    int64_t stamp_ns{0};
    uint8_t source_id{0};
    gaussian_lic_tracking::VisualAlignment alignment;
  };

  struct PendingSe3PhotometricFactor
  {
    int64_t stamp_ns{0};
    uint8_t source_id{0};
    gaussian_lic_tracking::VisualSe3PhotometricLinearization linearization;
  };

  struct QosProfileParams
  {
    std::string reliability{"best_effort"};
    std::string history{"keep_last"};
    int depth{5};
  };

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
      process_ready_pointcloud_queue();
    } catch (const std::exception & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "failed to propagate IMU tracking state: %s", ex.what());
    }
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
      process_visual_pair(*rendered_frame, observed);
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
      cache_rendered_frame(rendered);
      int64_t observed_match_delta_ns = 0;
      const gaussian_lic_tracking::VisualFrame * observed_frame =
        select_observed_frame_for_stamp(
        rendered.stamp_ns,
        rendered.width,
        rendered.height,
        &observed_match_delta_ns,
        nullptr);
      if (observed_frame != nullptr) {
        last_visual_rendered_cache_size_ = rendered_frame_cache_.size();
        last_visual_rendered_match_delta_ns_ = observed_match_delta_ns;
        process_visual_pair(rendered, *observed_frame);
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
    const gaussian_lic_tracking::VisualFrame & observed)
  {
    if (last_processed_visual_observed_stamp_ns_.has_value() &&
      last_processed_visual_rendered_stamp_ns_.has_value() &&
      last_processed_visual_observed_stamp_ns_.value() == observed.stamp_ns &&
      last_processed_visual_rendered_stamp_ns_.value() == rendered.stamp_ns)
    {
      return;
    }
    last_processed_visual_observed_stamp_ns_ = observed.stamp_ns;
    last_processed_visual_rendered_stamp_ns_ = rendered.stamp_ns;

    last_visual_residual_ = visual_factor_.evaluate(rendered, observed);
    last_visual_alignment_ = visual_factor_.estimate_translation(
      rendered,
      observed,
      visual_alignment_max_shift_px_);
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
        pending.stamp_ns = observed.stamp_ns;
        pending.source_id = visual_factor_source_id(observed.stamp_ns, rendered.stamp_ns);
        pending.linearization = last_visual_se3_photometric_linearization_;
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
    if (last_visual_alignment_.valid) {
      PendingVisualAlignmentFactor pending;
      pending.stamp_ns = observed.stamp_ns;
      pending.source_id = visual_factor_source_id(observed.stamp_ns, rendered.stamp_ns);
      pending.alignment = last_visual_alignment_;
      pending_visual_alignment_factors_.push_back(std::move(pending));
      trim_pending_visual_factor_queues();
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

  void trim_pending_visual_factor_queues()
  {
    const auto max_queue_size = static_cast<size_t>(visual_pending_factor_queue_size_);
    while (pending_visual_alignment_factors_.size() > max_queue_size) {
      pending_visual_alignment_factors_.pop_front();
      ++visual_alignment_pending_stale_drops_;
    }
    while (pending_visual_se3_photometric_factors_.size() > max_queue_size) {
      pending_visual_se3_photometric_factors_.pop_front();
      ++visual_se3_photometric_pending_stale_drops_;
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
    if (!enable_pointcloud_imu_wait_ || !enable_sliding_window_optimizer_ ||
      !imu_propagator_.initialized() || last_imu_stamp_ns_ == 0)
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
    cache_sparse_lidar_depth_frame(tracking_pose.stamp_ns, lidar_points);

    std::vector<gaussian_lic_tracking::SlidingWindowPointToPointFactor> window_point_factors;
    std::vector<gaussian_lic_tracking::SlidingWindowPointToPlaneFactor> window_plane_factors;
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
      if (enable_sliding_window_optimizer_) {
        auto lidar_window_factor = lidar_factor_.build_point_to_point_factor(lidar_points, tracking_pose);
        if (!lidar_window_factor.frame_points_i.empty()) {
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
            window_plane_correspondences += lidar_plane_factor.frame_points_i.size();
            accumulate_correspondence_weights(
              lidar_plane_factor.point_weights,
              window_plane_weight_count,
              window_plane_weight_sum,
              window_plane_weight_min);
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
            accumulate_correspondence_weights(
              gaussian_window_factor.point_weights,
              window_point_weight_count,
              window_point_weight_sum,
              window_point_weight_min);
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
      last_window_point_confidence_mean_ =
        mean_or_zero(window_point_weight_count, window_point_weight_sum);
      last_window_point_confidence_min_ =
        min_or_zero(window_point_weight_count, window_point_weight_min);
      last_window_plane_confidence_mean_ =
        mean_or_zero(window_plane_weight_count, window_plane_weight_sum);
      last_window_plane_confidence_min_ =
        min_or_zero(window_plane_weight_count, window_plane_weight_min);
      if (should_insert_lidar_keyframe(tracking_pose, lidar_points.size())) {
        lidar_factor_.insert_keyframe(lidar_points, tracking_pose);
        ++num_lidar_keyframes_;
        last_lidar_keyframe_pose_ = tracking_pose;
        has_lidar_keyframe_ = true;
      }
    }

    std::vector<gaussian_lic_tracking::SlidingWindowVisualAlignmentFactor> visual_window_factors;
    std::vector<gaussian_lic_tracking::SlidingWindowSe3PhotometricFactor> se3_photometric_factors;
    if (enable_sliding_window_optimizer_ && enable_visual_alignment_window_factor_) {
      std::deque<PendingVisualAlignmentFactor> retained_pending;
      for (const auto & pending : pending_visual_alignment_factors_) {
        const auto visual_reference = select_visual_factor_reference(pending.stamp_ns, tracking_pose);
        if (!visual_reference.has_value()) {
          if (visual_factor_stamp_is_expired(pending.stamp_ns, tracking_pose.stamp_ns)) {
            ++visual_alignment_pending_stale_drops_;
          } else {
            retained_pending.push_back(pending);
          }
          continue;
        }
        gaussian_lic_tracking::SlidingWindowVisualAlignmentFactor visual_factor;
        visual_factor.stamp_ns = visual_reference->stamp_ns;
        visual_factor.source_id = pending.source_id;
        visual_factor.reference_p_w_i = visual_reference->p_w_i;
        visual_factor.measured_shift_px = Eigen::Vector2d{
          pending.alignment.subpixel_dx,
          pending.alignment.subpixel_dy};
        visual_factor.meters_per_pixel = visual_alignment_meters_per_pixel_;
        visual_factor.weight = visual_alignment_window_weight_;
        visual_factor.huber_delta_m = visual_alignment_huber_delta_m_;
        visual_window_factors.push_back(visual_factor);
      }
      pending_visual_alignment_factors_ = std::move(retained_pending);
    }
    if (enable_sliding_window_optimizer_ && enable_se3_photometric_window_factor_) {
      std::deque<PendingSe3PhotometricFactor> retained_pending;
      for (const auto & pending : pending_visual_se3_photometric_factors_) {
        const auto visual_reference = select_visual_factor_reference(pending.stamp_ns, tracking_pose);
        if (!visual_reference.has_value()) {
          if (visual_factor_stamp_is_expired(pending.stamp_ns, tracking_pose.stamp_ns)) {
            ++visual_se3_photometric_pending_stale_drops_;
          } else {
            retained_pending.push_back(pending);
          }
          continue;
        }
        gaussian_lic_tracking::SlidingWindowSe3PhotometricFactor factor;
        factor.stamp_ns = visual_reference->stamp_ns;
        factor.source_id = pending.source_id;
        factor.reference_p_w_i = visual_reference->p_w_i;
        factor.reference_q_w_i = visual_reference->q_w_i;
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
        factor.weight = se3_photometric_window_weight_;
        factor.huber_delta = se3_photometric_factor_huber_delta_;
        if (factor.sqrt_information.norm() > std::numeric_limits<double>::epsilon()) {
          se3_photometric_factors.push_back(factor);
        } else {
          ++sliding_window_se3_photometric_factor_skip_count_;
        }
      }
      pending_visual_se3_photometric_factors_ = std::move(retained_pending);
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
        ++sliding_window_total_visual_factors_;
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
        ++sliding_window_total_se3_photometric_factors_;
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

    if (has_sliding_window_state_ && window_factor_added) {
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
        if (sliding_window_optimizer_.get_state(input_pose.stamp_ns, optimized)) {
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
              if (!imu_propagator_.initialized() ||
                imu_propagator_.state().stamp_ns <= applied.stamp_ns)
              {
                gaussian_lic_tracking::ImuState corrected_state;
                corrected_state.stamp_ns = applied.stamp_ns;
                corrected_state.p_w_i = applied.p_w_i;
                corrected_state.q_w_i = applied.q_w_i;
                corrected_state.v_w_i = applied.v_w_i;
                corrected_state.gyro_bias = applied.gyro_bias;
                corrected_state.accel_bias = applied.accel_bias;
                imu_propagator_.reset(corrected_state);
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

  std::optional<gaussian_lic_tracking::TrajectoryPose> select_visual_factor_reference(
    const int64_t factor_stamp_ns,
    const gaussian_lic_tracking::TrajectoryPose & current_pose) const
  {
    if (stamp_delta_is_within(factor_stamp_ns, current_pose.stamp_ns, visual_factor_max_dt_ns_)) {
      return current_pose;
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

    gaussian_lic_tracking::TrajectoryPose reference;
    reference.stamp_ns = best_state->stamp_ns;
    reference.p_w_i = best_state->p_w_i;
    reference.q_w_i = best_state->q_w_i;
    reference.v_w_i = best_state->v_w_i;
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

  static uint8_t visual_factor_source_id(
    const int64_t observed_stamp_ns,
    const int64_t rendered_stamp_ns)
  {
    uint64_t mixed = static_cast<uint64_t>(observed_stamp_ns);
    mixed ^= static_cast<uint64_t>(rendered_stamp_ns) + 0x9e3779b97f4a7c15ULL +
      (mixed << 6U) + (mixed >> 2U);
    mixed ^= mixed >> 30U;
    mixed *= 0xbf58476d1ce4e5b9ULL;
    mixed ^= mixed >> 27U;
    mixed *= 0x94d049bb133111ebULL;
    mixed ^= mixed >> 31U;
    return static_cast<uint8_t>(1U + (mixed % 254U));
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

  const gaussian_lic_tracking::VisualFrame * select_observed_frame_for_stamp(
    const int64_t rendered_stamp_ns,
    const size_t width,
    const size_t height,
    int64_t * selected_delta_ns = nullptr,
    bool * cache_had_size_match = nullptr) const
  {
    const gaussian_lic_tracking::VisualFrame * best = nullptr;
    int64_t best_delta_ns = std::numeric_limits<int64_t>::max();
    bool had_size_match = false;
    for (const auto & frame : observed_frame_cache_) {
      if (frame.width != width || frame.height != height) {
        continue;
      }
      had_size_match = true;
      const int64_t delta_ns = stamp_delta_ns(frame.stamp_ns, rendered_stamp_ns);
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
      auto at = [&observed](const size_t px, const size_t py) {
          return static_cast<double>(observed.gray[py * observed.width + px]);
        };
      gaussian_lic_tracking::VisualSe3PhotometricSample sample;
      ++batch.sampled_depth_pixels;
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
    status.num_raw_pointclouds = num_raw_pointclouds_;
    status.num_raw_imus = num_raw_imus_;
    status.num_published_poses = num_published_poses_;
    status.pointcloud_imu_wait_queue_size =
      static_cast<uint64_t>(pending_pointclouds_waiting_for_imu_.size());
    status.pointcloud_imu_wait_deferred = pointcloud_imu_wait_deferred_;
    status.pointcloud_imu_wait_released = pointcloud_imu_wait_released_;
    status.pointcloud_imu_wait_dropped = pointcloud_imu_wait_dropped_;
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
    status.sliding_window_states = has_last_sliding_window_summary_
      ? static_cast<uint64_t>(summary.state_count)
      : static_cast<uint64_t>(sliding_window_optimizer_.states().size());
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
    status.sliding_window_marginalized_states = static_cast<uint64_t>(summary.marginalized_state_count);
    status.sliding_window_schur_marginalizations =
      static_cast<uint64_t>(summary.schur_marginalization_count);
    status.sliding_window_fallback_marginalization_priors =
      static_cast<uint64_t>(summary.fallback_marginalization_prior_count);
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
    status.sliding_window_smoothness_factor_cost = summary.smoothness_factor_cost;
    status.sliding_window_last_step_norm = summary.last_step_norm;
    status.sliding_window_last_step_scale = summary.last_step_scale;
    status.sliding_window_last_damping = summary.last_damping;
    status.sliding_window_dense_prior_min_singular_value = summary.dense_prior_min_singular_value;
    status.sliding_window_dense_prior_max_singular_value = summary.dense_prior_max_singular_value;
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
    status.visual_depth_dilation_px = static_cast<uint32_t>(sparse_lidar_depth_dilation_px_);
    status.visual_depth_match_delta_ns = last_visual_depth_match_delta_ns_;
    status.visual_depth_miss_count = visual_depth_miss_count_;
    status.visual_depth_stale_count = visual_depth_stale_count_;
    status.visual_depth_size_mismatch_count = visual_depth_size_mismatch_count_;
    status.visual_alignment_pending_queue_size =
      static_cast<uint64_t>(pending_visual_alignment_factors_.size());
    status.visual_se3_photometric_pending_queue_size =
      static_cast<uint64_t>(pending_visual_se3_photometric_factors_.size());
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
  bool enable_external_odometry_prior_{false};
  int visual_max_pixels_{200000};
  int64_t visual_factor_max_dt_ns_{150000000LL};
  int64_t visual_depth_max_dt_ns_{0LL};
  int depth_frame_cache_size_{8};
  int sparse_lidar_depth_dilation_px_{1};
  int rendered_frame_cache_size_{8};
  int observed_frame_cache_size_{64};
  int visual_pending_factor_queue_size_{64};
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
  int se3_photometric_min_hessian_rank_{3};
  double se3_photometric_min_depth_m_{0.05};
  double se3_photometric_max_depth_m_{200.0};
  double se3_photometric_min_gradient_{1.0e-4};
  double se3_photometric_huber_delta_{0.15};
  double se3_photometric_max_abs_residual_{1.0};
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
  std::string lidar_time_field_{"auto"};
  std::string lidar_time_unit_{"auto"};
  std::string lidar_time_mode_{"auto"};
  double lidar_scan_order_duration_s_{0.1};
  double lidar_max_abs_point_time_offset_s_{0.25};
  int imu_history_size_{4000};
  int64_t trajectory_control_interval_ns_{50000000LL};
  int64_t external_odometry_prior_max_dt_ns_{100000000LL};
  int external_odometry_prior_cache_size_{128};
  int sliding_window_max_states_{12};
  int sliding_window_max_iterations_{3};
  double sliding_window_max_rotation_step_rad_{0.5};
  double sliding_window_max_translation_step_m_{1.0};
  double sliding_window_max_velocity_step_mps_{5.0};
  double sliding_window_max_bias_step_{1.0};
  double sliding_window_max_feedback_translation_m_{1.0};
  double sliding_window_max_feedback_rotation_rad_{0.5};
  double sliding_window_max_feedback_velocity_mps_{5.0};
  double sliding_window_max_normal_equation_condition_{1.0e13};
  double sliding_window_min_normal_equation_rank_ratio_{0.8};
  double sliding_window_max_state_gap_s_{1.0};
  double gaussian_snapshot_lidar_min_opacity_{0.01};
  double sliding_window_imu_weight_{1.0};
  double sliding_window_imu_rotation_weight_{1.0};
  double sliding_window_imu_velocity_weight_{1.0};
  double sliding_window_imu_position_weight_{1.0};
  double sliding_window_imu_max_extrapolation_s_{0.02};
  double sliding_window_bias_weight_{1.0};
  double sliding_window_pose_translation_weight_{2.0};
  double sliding_window_pose_rotation_weight_{2.0};
  double external_odometry_prior_translation_weight_{4.0};
  double external_odometry_prior_rotation_weight_{4.0};
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
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr external_odometry_prior_sub_;
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
  std::deque<ExternalPosePrior> external_odometry_priors_;
  std::deque<PendingPointCloud> pending_pointclouds_waiting_for_imu_;
  gaussian_lic_tracking::ImuPreintegrator sliding_window_preintegrator_;
  gaussian_lic_tracking::ImuBias sliding_window_bias_;
  bool sliding_window_preintegrator_initialized_{false};
  bool has_sliding_window_state_{false};
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
  uint64_t sliding_window_imu_factor_skip_count_{0};
  uint64_t sliding_window_imu_time_gap_skip_count_{0};
  uint64_t sliding_window_feedback_update_count_{0};
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
  uint64_t sliding_window_invalid_optimized_states_{0};
  uint64_t trajectory_deskew_queries_{0};
  uint64_t trajectory_deskew_hits_{0};
  uint64_t trajectory_control_pose_skip_count_{0};
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
  std::deque<PendingVisualAlignmentFactor> pending_visual_alignment_factors_;
  std::deque<PendingSe3PhotometricFactor> pending_visual_se3_photometric_factors_;
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
  gaussian_lic_tracking::VisualCameraIntrinsics camera_intrinsics_;
  std::deque<DepthFrame> depth_frame_cache_;
  std::deque<gaussian_lic_tracking::VisualFrame> observed_frame_cache_;
  std::optional<int64_t> last_processed_visual_observed_stamp_ns_;
  std::optional<int64_t> last_processed_visual_rendered_stamp_ns_;
  size_t last_visual_depth_cache_size_{0};
  int64_t last_visual_depth_match_delta_ns_{0};
  uint64_t visual_depth_miss_count_{0};
  uint64_t visual_depth_stale_count_{0};
  uint64_t visual_depth_size_mismatch_count_{0};
  bool has_camera_intrinsics_{false};
  size_t last_observed_image_width_{0};
  size_t last_observed_image_height_{0};
  uint64_t visual_alignment_pending_stale_drops_{0};
  uint64_t visual_se3_photometric_pending_stale_drops_{0};
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
  uint64_t num_raw_pointclouds_{0};
  uint64_t num_raw_imus_{0};
  uint64_t num_published_poses_{0};
  uint64_t pointcloud_imu_wait_deferred_{0};
  uint64_t pointcloud_imu_wait_released_{0};
  uint64_t pointcloud_imu_wait_dropped_{0};
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
  rclcpp::spin(std::make_shared<TrackingNode>());
  rclcpp::shutdown();
  return 0;
}
