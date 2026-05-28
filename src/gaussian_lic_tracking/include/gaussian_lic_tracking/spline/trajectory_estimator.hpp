// SPDX-License-Identifier: GPL-3.0-or-later
//
// ROS2-native port of Coco-LIC's TrajectoryEstimator harness
// (external/Coco-LIC/src/odom/trajectory_estimator.cpp). Owns the cumulative
// B-spline control knots, IMU biases, and gravity, and drives Ceres over the
// ROS2-native continuous-time factors ported in this directory.
//
// The harness is intentionally minimal: it covers the IMU + LiDAR factors
// required to validate the residual surface on real bags. Photometric and
// marginalization-on-knots can be added without changing this interface.

#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gaussian_lic_tracking/spline/continuous_time_imu_factor.hpp>
#include <gaussian_lic_tracking/spline/continuous_time_lidar_factor.hpp>

namespace gaussian_lic_tracking
{
namespace spline
{

struct TrajectoryEstimatorOptions
{
  int max_num_iterations{30};
  double function_tolerance{1.0e-8};
  double parameter_tolerance{1.0e-8};
  double gradient_tolerance{1.0e-10};
  // Optional Ceres trust-region controls. Leave <= 0 to use Ceres defaults.
  double initial_trust_region_radius{0.0};
  double max_trust_region_radius{0.0};
  bool minimizer_progress_to_stdout{false};
  bool hold_gyro_bias_constant{false};
  bool hold_accel_bias_constant{false};
  bool hold_gravity_constant{true};
  // Coco-LIC fixes all control points up to index 3 in each local solve. This
  // anchors the gauge freedom while newly-added knots absorb the current scan.
  // Leave negative to optimize every knot, which is useful for batch probes.
  int fixed_control_point_index{-1};
};

struct TrajectoryEstimatorSummary
{
  bool success{false};
  double initial_cost{0.0};
  double final_cost{0.0};
  double initial_imu_cost{0.0};
  double final_imu_cost{0.0};
  double initial_lidar_cost{0.0};
  double final_lidar_cost{0.0};
  double initial_position_prior_cost{0.0};
  double final_position_prior_cost{0.0};
  double initial_velocity_prior_cost{0.0};
  double final_velocity_prior_cost{0.0};
  double initial_orientation_prior_cost{0.0};
  double final_orientation_prior_cost{0.0};
  double initial_bias_prior_cost{0.0};
  double final_bias_prior_cost{0.0};
  double initial_smoothness_cost{0.0};
  double final_smoothness_cost{0.0};
  int iterations{0};
  std::string brief_report;
};

class TrajectoryEstimator
{
public:
  static constexpr int N = kPositionSplineOrder;

  TrajectoryEstimator(double dt_s);
  ~TrajectoryEstimator();
  TrajectoryEstimator(const TrajectoryEstimator &) = delete;
  TrajectoryEstimator & operator=(const TrajectoryEstimator &) = delete;
  TrajectoryEstimator(TrajectoryEstimator &&) noexcept;
  TrajectoryEstimator & operator=(TrajectoryEstimator &&) noexcept;

  // Replace the spline knot vectors. Both arrays must have the same length and
  // contain at least 4 entries. Rotation knots are normalized on entry.
  void set_knots(
    const std::vector<Eigen::Quaterniond> & rotation_knots,
    const std::vector<Eigen::Vector3d> & position_knots);

  void set_gyro_bias(const Eigen::Vector3d & gyro_bias);
  void set_accel_bias(const Eigen::Vector3d & accel_bias);
  void set_gravity_world(const Eigen::Vector3d & gravity_world);

  std::vector<Eigen::Quaterniond> rotation_knots() const;
  std::vector<Eigen::Vector3d> position_knots() const;
  Eigen::Vector3d gyro_bias() const { return gyro_bias_; }
  Eigen::Vector3d accel_bias() const { return accel_bias_; }
  Eigen::Vector3d gravity_world() const { return gravity_world_; }

  std::size_t knot_count() const { return rotation_knots_.size(); }
  double dt_s() const { return dt_s_; }

  // Locate the active segment for a time `t_s` relative to the first knot.
  // Returns false when the stamp falls outside the optimizable interior
  // (segment indices [1 .. knot_count - 3]).
  bool find_segment(double t_s, int & segment_index, double & u) const;

  // Add a continuous-time IMU factor. Returns false if the stamp is outside
  // the optimizable interior.
  bool add_imu_factor(
    double t_s,
    const ImuSample & sample,
    const Eigen::Matrix<double, 6, 1> & info_diag);

  // Add a continuous-time LiDAR factor (plane or edge). Returns false if the
  // stamp is outside the optimizable interior.
  bool add_lidar_factor(
    double t_s,
    const LidarPointCorrespondence & correspondence,
    const LidarExtrinsics & extrinsics,
    double weight = 1.0,
    double huber_delta_m = 0.0);

  // Add a continuous-time 3D point-to-point LiDAR factor. This is used by the
  // persistent/Gaussian map frontend when a LiDAR-frame point is associated to
  // a map-frame Gaussian/map point. Unlike three independent axis-aligned plane
  // residuals, this keeps the SE(3) residual coupled under one robust loss.
  bool add_lidar_point_to_point_factor(
    double t_s,
    const Eigen::Vector3d & point_lidar,
    const Eigen::Vector3d & target_point_map,
    const LidarExtrinsics & extrinsics,
    double weight = 1.0,
    double huber_delta_m = 0.0,
    double scale = 1.0);

  // Add a plane-normal alignment factor. This complements point-to-plane
  // distance factors by directly observing rotation when a LiDAR-frame plane
  // has been associated to a persistent world-frame plane.
  bool add_lidar_plane_normal_factor(
    double t_s,
    const Eigen::Vector3d & normal_lidar,
    const Eigen::Vector3d & normal_world,
    const LidarExtrinsics & extrinsics,
    double weight = 1.0,
    double huber_delta_rad = 0.0);

  // Add a position-only prior on the continuous-time body trajectory.
  // Returns false if the stamp is outside the optimizable interior.
  bool add_position_prior_factor(
    double t_s,
    const Eigen::Vector3d & position_world,
    double weight = 1.0,
    double huber_delta_m = 0.0);

  // Add a velocity prior on the continuous-time body trajectory. This is a
  // motion-increment constraint and does not anchor the absolute position.
  bool add_velocity_prior_factor(
    double t_s,
    const Eigen::Vector3d & velocity_world,
    double weight = 1.0,
    double huber_delta_mps = 0.0);

  // Add an angular-velocity prior in body coordinates. This constrains the
  // local SO(3) spline derivative without anchoring absolute yaw/roll/pitch.
  bool add_angular_velocity_prior_factor(
    double t_s,
    const Eigen::Vector3d & angular_velocity_body,
    double weight = 1.0,
    double huber_delta_radps = 0.0);

  // Add a relative translation prior between two continuous-time body poses.
  // The target is expressed in the body frame at t0:
  //   target = q(t0)^-1 * (p(t1) - p(t0)).
  // This preserves scan-to-scan motion information without anchoring the
  // global position of either pose.
  bool add_relative_position_prior_factor(
    double t0_s,
    double t1_s,
    const Eigen::Vector3d & target_p_body0,
    double weight = 1.0,
    double huber_delta_m = 0.0);

  // Add a relative SO(3) prior between two continuous-time body poses:
  //   target = q(t0)^-1 * q(t1).
  bool add_relative_orientation_prior_factor(
    double t0_s,
    double t1_s,
    const Eigen::Quaterniond & target_q_body0_body1,
    double weight = 1.0,
    double huber_delta_rad = 0.0);

  // Add an orientation-only prior on the continuous-time body trajectory.
  // The residual is the SO(3) log-map between target and spline orientation.
  bool add_orientation_prior_factor(
    double t_s,
    const Eigen::Quaterniond & q_world_body,
    double weight = 1.0,
    double huber_delta_rad = 0.0);

  bool add_gyro_bias_prior_factor(
    const Eigen::Vector3d & gyro_bias,
    double weight = 1.0,
    double huber_delta_radps = 0.0);

  bool add_accel_bias_prior_factor(
    const Eigen::Vector3d & accel_bias,
    double weight = 1.0,
    double huber_delta_mps2 = 0.0);

  bool add_position_smoothness_factor(
    std::size_t first_knot_index,
    double weight = 1.0,
    double huber_delta_m = 0.0);

  bool add_rotation_smoothness_factor(
    std::size_t first_knot_index,
    double weight = 1.0,
    double huber_delta_rad = 0.0);

  // Direct priors on control knots. These are the retained-window counterpart
  // of Coco-LIC's marginalization prior: the sliding-window driver can keep a
  // prefix of already-estimated knots softly anchored while newer knots absorb
  // the current scan.
  bool add_knot_position_prior_factor(
    std::size_t knot_index,
    const Eigen::Vector3d & position_world,
    double weight = 1.0,
    double huber_delta_m = 0.0);

  bool add_knot_orientation_prior_factor(
    std::size_t knot_index,
    const Eigen::Quaterniond & q_world_body,
    double weight = 1.0,
    double huber_delta_rad = 0.0);

  // Adds a dense linearized prior over position control knots:
  //   r = J * (p_current - p_reference) + r0
  // where columns are stacked in the same order as `knot_indices`.
  bool add_dense_position_prior_factor(
    const std::vector<std::size_t> & knot_indices,
    const std::vector<Eigen::Vector3d> & reference_positions,
    const Eigen::MatrixXd & jacobian,
    const Eigen::VectorXd & residual);

  std::size_t imu_factor_count() const { return imu_factor_count_; }
  std::size_t lidar_factor_count() const { return lidar_factor_count_; }
  std::size_t lidar_point_factor_count() const { return lidar_point_factor_count_; }
  std::size_t lidar_normal_factor_count() const { return lidar_normal_factor_count_; }
  std::size_t position_prior_factor_count() const { return position_prior_factor_count_; }
  std::size_t velocity_prior_factor_count() const { return velocity_prior_factor_count_; }
  std::size_t angular_velocity_prior_factor_count() const
  {
    return angular_velocity_prior_factor_count_;
  }
  std::size_t orientation_prior_factor_count() const { return orientation_prior_factor_count_; }
  std::size_t gyro_bias_prior_factor_count() const { return gyro_bias_prior_factor_count_; }
  std::size_t accel_bias_prior_factor_count() const { return accel_bias_prior_factor_count_; }
  std::size_t position_smoothness_factor_count() const { return position_smoothness_factor_count_; }
  std::size_t rotation_smoothness_factor_count() const { return rotation_smoothness_factor_count_; }
  std::size_t dense_position_prior_factor_count() const
  {
    return dense_position_prior_factor_count_;
  }

  TrajectoryEstimatorSummary solve(const TrajectoryEstimatorOptions & options);

private:
  void rebuild_problem();
  void sync_state_to_storage();
  void sync_state_from_storage();

  double dt_s_{0.01};
  std::vector<Eigen::Quaterniond> rotation_knots_;
  std::vector<Eigen::Vector3d> position_knots_;
  Eigen::Vector3d gyro_bias_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accel_bias_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gravity_world_{Eigen::Vector3d::Zero()};

  // Pimpl so callers do not need Ceres headers transitively.
  struct Impl;
  std::unique_ptr<Impl> impl_;

  std::size_t imu_factor_count_{0};
  std::size_t lidar_factor_count_{0};
  std::size_t lidar_point_factor_count_{0};
  std::size_t lidar_normal_factor_count_{0};
  std::size_t position_prior_factor_count_{0};
  std::size_t velocity_prior_factor_count_{0};
  std::size_t angular_velocity_prior_factor_count_{0};
  std::size_t orientation_prior_factor_count_{0};
  std::size_t gyro_bias_prior_factor_count_{0};
  std::size_t accel_bias_prior_factor_count_{0};
  std::size_t position_smoothness_factor_count_{0};
  std::size_t rotation_smoothness_factor_count_{0};
  std::size_t dense_position_prior_factor_count_{0};
};

}  // namespace spline
}  // namespace gaussian_lic_tracking
