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
  bool minimizer_progress_to_stdout{false};
  bool hold_gyro_bias_constant{false};
  bool hold_accel_bias_constant{false};
  bool hold_gravity_constant{true};
};

struct TrajectoryEstimatorSummary
{
  bool success{false};
  double initial_cost{0.0};
  double final_cost{0.0};
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
    double weight = 1.0);

  std::size_t imu_factor_count() const { return imu_factor_count_; }
  std::size_t lidar_factor_count() const { return lidar_factor_count_; }

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
};

}  // namespace spline
}  // namespace gaussian_lic_tracking
