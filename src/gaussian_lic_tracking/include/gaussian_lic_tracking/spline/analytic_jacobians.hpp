// SPDX-License-Identifier: GPL-3.0-or-later
//
// Closed-form analytic Jacobians for the continuous-time IMU and LiDAR
// residuals ported from Coco-LIC. The forms here cover the cases where the
// chain rule collapses to a constant matrix:
//
//   * IMU position-knot Jacobian for the accelerometer residual (the position
//     spline appears only through its second time derivative, so each knot
//     contributes a fixed coefficient times the body-frame rotation).
//   * IMU bias Jacobian for both gyro and accel residuals — identity columns
//     scaled by the corresponding information weights.
//   * IMU gravity Jacobian — the accel residual is the only one that touches
//     gravity, with a `-R^T` block.
//   * LiDAR plane position-knot Jacobian — linear in knot position because
//     the plane residual is `n^T p_world + d`.
//
// The harder rotation-knot Jacobian (which requires Lie-group chain rule
// through the cumulative B-spline) remains numeric for now. Callers that
// want full analytic coverage should keep using `NumericDiffCostFunction`
// for rotation knots and combine the analytic helpers in this file for the
// other parameter blocks.

#pragma once

#include <array>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gaussian_lic_tracking/spline/ceres_spline_helper.hpp>
#include <gaussian_lic_tracking/spline/so3_ops.hpp>
#include <gaussian_lic_tracking/spline/split_spline_view.hpp>

namespace gaussian_lic_tracking
{
namespace spline
{

// Returns the Jacobian of the IMU 6-d residual with respect to position knot
// `knot_index` (∈ [0, kPositionSplineOrder)). Layout is:
//   [gyro residual rows]      → 3×3 zero
//   [accel residual rows]     → 3×3 of `coeff * inv_dt_s^2 * R_b_w^T`
// scaled by the 6-vector info diagonal.
inline Eigen::Matrix<double, 6, 3> analytic_imu_position_knot_jacobian(
  double u,
  double inv_dt_s,
  const Eigen::Quaterniond & rotation_world_to_body,
  const Eigen::Matrix<double, 6, 1> & info_diag,
  int knot_index)
{
  using Helper = CubicSplineHelper;
  typename Helper::VecN raw;
  Helper::base_coefficients_with_time<2>(raw, u);
  const typename Helper::VecN coeff =
    inv_dt_s * inv_dt_s * Helper::blending_matrix() * raw;

  Eigen::Matrix<double, 6, 3> jacobian = Eigen::Matrix<double, 6, 3>::Zero();
  if (knot_index < 0 || knot_index >= Helper::N) {
    return jacobian;
  }
  const Eigen::Matrix3d r_b_w = rotation_world_to_body.inverse().toRotationMatrix();
  jacobian.block<3, 3>(3, 0) = coeff[knot_index] * r_b_w;
  return (info_diag.asDiagonal() * jacobian).eval();
}

// Returns the Jacobian of the IMU 6-d residual with respect to the gyro bias
// (a 3-vector). Closed form: `[I; 0]` scaled by the info diagonal because
// `r_gyro = ω_spline - (ω_meas - b_g)` → `∂r_gyro / ∂b_g = +I`.
inline Eigen::Matrix<double, 6, 3> analytic_imu_gyro_bias_jacobian(
  const Eigen::Matrix<double, 6, 1> & info_diag)
{
  Eigen::Matrix<double, 6, 3> jacobian = Eigen::Matrix<double, 6, 3>::Zero();
  jacobian.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  return (info_diag.asDiagonal() * jacobian).eval();
}

// Returns the Jacobian of the IMU 6-d residual with respect to the accel
// bias (a 3-vector). Closed form: `[0; I]` scaled by the info diagonal.
inline Eigen::Matrix<double, 6, 3> analytic_imu_accel_bias_jacobian(
  const Eigen::Matrix<double, 6, 1> & info_diag)
{
  Eigen::Matrix<double, 6, 3> jacobian = Eigen::Matrix<double, 6, 3>::Zero();
  jacobian.block<3, 3>(3, 0) = Eigen::Matrix3d::Identity();
  return (info_diag.asDiagonal() * jacobian).eval();
}

// Returns the Jacobian of the IMU residual with respect to gravity (R^3).
// Only the accel rows depend on gravity, with sign `-R_w_b^T`.
inline Eigen::Matrix<double, 6, 3> analytic_imu_gravity_jacobian(
  const Eigen::Quaterniond & rotation_world_to_body,
  const Eigen::Matrix<double, 6, 1> & info_diag)
{
  Eigen::Matrix<double, 6, 3> jacobian = Eigen::Matrix<double, 6, 3>::Zero();
  jacobian.block<3, 3>(3, 0) = -rotation_world_to_body.inverse().toRotationMatrix();
  return (info_diag.asDiagonal() * jacobian).eval();
}

// Returns the Jacobian of the LiDAR plane residual with respect to position
// knot `knot_index`. Closed form: the residual is
//   r = weight * (n^T (R_world_to_map * p_world(u) + p_world_in_map) + d)
// and p_world(u) = sum_i coeff[i] * p_knot[i], hence
//   ∂r / ∂p_knot[i] = weight * coeff[i] * n^T * R_world_to_map.
inline Eigen::Matrix<double, 1, 3> analytic_lidar_plane_position_knot_jacobian(
  double u,
  double /*inv_dt_s*/,
  double weight,
  const Eigen::Vector3d & plane_normal_in_map,
  const Eigen::Quaterniond & rotation_world_to_map,
  int knot_index)
{
  using Helper = CubicSplineHelper;
  typename Helper::VecN raw;
  Helper::base_coefficients_with_time<0>(raw, u);
  const typename Helper::VecN coeff = Helper::blending_matrix() * raw;
  Eigen::Matrix<double, 1, 3> jacobian = Eigen::Matrix<double, 1, 3>::Zero();
  if (knot_index < 0 || knot_index >= Helper::N) {
    return jacobian;
  }
  jacobian =
    (weight * coeff[knot_index]) *
    plane_normal_in_map.transpose() *
    rotation_world_to_map.toRotationMatrix();
  return jacobian;
}

}  // namespace spline
}  // namespace gaussian_lic_tracking
