// SPDX-License-Identifier: GPL-3.0-or-later
//
// ROS2-native port of Coco-LIC's IMUFactor / IMUFactorNURBS continuous-time
// residual (external/Coco-LIC/src/odom/factor/analytic_diff/trajectory_value_factor.h).
//
// The residual is a 6-dimensional vector:
//   r[0:3] = omega_spline(t) - (omega_meas - b_g)
//   r[3:6] = R_w_b(t)^T * (a_w(t) - g_w) - (a_meas - b_a)
// where the spline state is obtained by evaluating the cumulative SO(3) +
// Euclidean uniform cubic B-spline at the IMU stamp `t`.
//
// Sliding-window solvers should treat the spline rotation/position knots as
// state variables (each rotation knot perturbed in tangent space, each
// position knot in R^3). Gravity, gyro bias, and accelerometer bias are also
// optimized.

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gaussian_lic_tracking/spline/ceres_spline_helper.hpp>
#include <gaussian_lic_tracking/spline/so3_ops.hpp>
#include <gaussian_lic_tracking/spline/split_spline_view.hpp>

namespace gaussian_lic_tracking
{
namespace spline
{

struct ImuSample
{
  Eigen::Vector3d gyro{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accel{Eigen::Vector3d::Zero()};
};

struct ImuFactorState
{
  std::array<Eigen::Quaterniond, kPositionSplineOrder> rotation_knots;
  std::array<Eigen::Vector3d, kPositionSplineOrder> position_knots;
  Eigen::Vector3d gyro_bias{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accel_bias{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gravity_world{Eigen::Vector3d::Zero()};
};

class ContinuousTimeImuFactor
{
public:
  static constexpr int N = kPositionSplineOrder;
  static constexpr int kResidualDim = 6;

  ContinuousTimeImuFactor(
    double normalized_time,
    double inv_dt_s,
    const ImuSample & measurement,
    const Eigen::Matrix<double, kResidualDim, 1> & info_diag)
  : u_(normalized_time),
    inv_dt_s_(inv_dt_s),
    measurement_(measurement),
    info_diag_(info_diag)
  {
  }

  Eigen::Matrix<double, kResidualDim, 1> residual(const ImuFactorState & state) const
  {
    const SplitSplineView<N> view(state.rotation_knots, state.position_knots, u_, inv_dt_s_);
    const auto evaluated = view.evaluate(true);

    Eigen::Matrix<double, kResidualDim, 1> r;
    r.head<3>() = evaluated.omega_b - (measurement_.gyro - state.gyro_bias);
    const Eigen::Vector3d predicted_accel =
      evaluated.q_w_b.inverse() * (evaluated.a_w_b - state.gravity_world);
    r.tail<3>() = predicted_accel - (measurement_.accel - state.accel_bias);
    return info_diag_.asDiagonal() * r;
  }

  Eigen::Vector3d predicted_gyro(const ImuFactorState & state) const
  {
    const SplitSplineView<N> view(state.rotation_knots, state.position_knots, u_, inv_dt_s_);
    return view.angular_velocity_body() + state.gyro_bias;
  }

  Eigen::Vector3d predicted_accel(const ImuFactorState & state) const
  {
    const SplitSplineView<N> view(state.rotation_knots, state.position_knots, u_, inv_dt_s_);
    const Eigen::Quaterniond q = view.rotation();
    const Eigen::Vector3d a_w = view.acceleration_world();
    return q.inverse() * (a_w - state.gravity_world) + state.accel_bias;
  }

  double normalized_time() const { return u_; }
  double inv_dt_s() const { return inv_dt_s_; }
  const ImuSample & measurement() const { return measurement_; }
  const Eigen::Matrix<double, kResidualDim, 1> & info_diag() const { return info_diag_; }

private:
  double u_{0.0};
  double inv_dt_s_{1.0};
  ImuSample measurement_{};
  Eigen::Matrix<double, kResidualDim, 1> info_diag_{
    Eigen::Matrix<double, kResidualDim, 1>::Ones()};
};

// Numeric Jacobian of the IMU residual w.r.t. one rotation control knot
// (perturbation in tangent space, so the Jacobian has shape 6x3).
inline Eigen::Matrix<double, 6, 3> numeric_jacobian_rotation_knot(
  const ContinuousTimeImuFactor & factor,
  const ImuFactorState & state,
  int knot_index,
  double epsilon = 1.0e-6)
{
  Eigen::Matrix<double, 6, 3> jacobian;
  for (int axis = 0; axis < 3; ++axis) {
    Eigen::Vector3d delta = Eigen::Vector3d::Zero();
    delta[axis] = epsilon;
    ImuFactorState plus = state;
    ImuFactorState minus = state;
    plus.rotation_knots[knot_index] =
      (state.rotation_knots[knot_index] * quaternion_exp(delta)).normalized();
    minus.rotation_knots[knot_index] =
      (state.rotation_knots[knot_index] * quaternion_exp(-delta)).normalized();
    const auto r_plus = factor.residual(plus);
    const auto r_minus = factor.residual(minus);
    jacobian.col(axis) = (r_plus - r_minus) / (2.0 * epsilon);
  }
  return jacobian;
}

inline Eigen::Matrix<double, 6, 3> numeric_jacobian_position_knot(
  const ContinuousTimeImuFactor & factor,
  const ImuFactorState & state,
  int knot_index,
  double epsilon = 1.0e-6)
{
  Eigen::Matrix<double, 6, 3> jacobian;
  for (int axis = 0; axis < 3; ++axis) {
    ImuFactorState plus = state;
    ImuFactorState minus = state;
    plus.position_knots[knot_index][axis] += epsilon;
    minus.position_knots[knot_index][axis] -= epsilon;
    const auto r_plus = factor.residual(plus);
    const auto r_minus = factor.residual(minus);
    jacobian.col(axis) = (r_plus - r_minus) / (2.0 * epsilon);
  }
  return jacobian;
}

inline Eigen::Matrix<double, 6, 3> numeric_jacobian_gyro_bias(
  const ContinuousTimeImuFactor & factor,
  const ImuFactorState & state,
  double epsilon = 1.0e-6)
{
  Eigen::Matrix<double, 6, 3> jacobian;
  for (int axis = 0; axis < 3; ++axis) {
    ImuFactorState plus = state;
    ImuFactorState minus = state;
    plus.gyro_bias[axis] += epsilon;
    minus.gyro_bias[axis] -= epsilon;
    jacobian.col(axis) = (factor.residual(plus) - factor.residual(minus)) / (2.0 * epsilon);
  }
  return jacobian;
}

inline Eigen::Matrix<double, 6, 3> numeric_jacobian_accel_bias(
  const ContinuousTimeImuFactor & factor,
  const ImuFactorState & state,
  double epsilon = 1.0e-6)
{
  Eigen::Matrix<double, 6, 3> jacobian;
  for (int axis = 0; axis < 3; ++axis) {
    ImuFactorState plus = state;
    ImuFactorState minus = state;
    plus.accel_bias[axis] += epsilon;
    minus.accel_bias[axis] -= epsilon;
    jacobian.col(axis) = (factor.residual(plus) - factor.residual(minus)) / (2.0 * epsilon);
  }
  return jacobian;
}

inline Eigen::Matrix<double, 6, 3> numeric_jacobian_gravity(
  const ContinuousTimeImuFactor & factor,
  const ImuFactorState & state,
  double epsilon = 1.0e-6)
{
  Eigen::Matrix<double, 6, 3> jacobian;
  for (int axis = 0; axis < 3; ++axis) {
    ImuFactorState plus = state;
    ImuFactorState minus = state;
    plus.gravity_world[axis] += epsilon;
    minus.gravity_world[axis] -= epsilon;
    jacobian.col(axis) = (factor.residual(plus) - factor.residual(minus)) / (2.0 * epsilon);
  }
  return jacobian;
}

}  // namespace spline
}  // namespace gaussian_lic_tracking
