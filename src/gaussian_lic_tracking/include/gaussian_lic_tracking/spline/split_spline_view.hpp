// SPDX-License-Identifier: GPL-3.0-or-later
//
// ROS2-native port of Coco-LIC's SplitSplineView
// (external/Coco-LIC/src/odom/factor/analytic_diff/split_spline_view.h).
//
// SplitSplineView combines a cumulative SO(3) B-spline for orientation and a
// standard Euclidean B-spline for world-frame position into the SE(3)
// trajectory used by the continuous-time IMU/LiDAR/camera factors.
//
// The upstream class exposes value/derivative queries plus analytic Jacobians
// w.r.t. control knots; this file ports the value/derivative queries first.
// Knot Jacobians live in the factor files that consume them (so they can
// share the same Ceres-style raw-pointer interface).

#pragma once

#include <cstddef>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gaussian_lic_tracking/spline/ceres_spline_helper.hpp>

namespace gaussian_lic_tracking
{
namespace spline
{

struct WorldStateAtT
{
  Eigen::Quaterniond q_w_b{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d p_w_b{Eigen::Vector3d::Zero()};
  Eigen::Vector3d v_w_b{Eigen::Vector3d::Zero()};
  Eigen::Vector3d a_w_b{Eigen::Vector3d::Zero()};
  Eigen::Vector3d omega_b{Eigen::Vector3d::Zero()};
  Eigen::Vector3d alpha_b{Eigen::Vector3d::Zero()};
};

template <int N_ = kPositionSplineOrder>
class SplitSplineView
{
public:
  static constexpr int N = N_;
  using Helper = CeresSplineHelper<N_>;

  SplitSplineView(
    const std::array<Eigen::Quaterniond, N_> & rotation_knots,
    const std::array<Eigen::Vector3d, N_> & position_knots,
    double normalized_time,
    double inv_dt_s)
  : rotation_knots_(rotation_knots),
    position_knots_(position_knots),
    u_(normalized_time),
    inv_dt_s_(inv_dt_s)
  {
  }

  WorldStateAtT evaluate(bool with_acceleration = true) const
  {
    WorldStateAtT state;
    Helper::evaluate_lie_so3(
      rotation_knots_, u_, inv_dt_s_,
      &state.q_w_b,
      &state.omega_b,
      with_acceleration ? &state.alpha_b : nullptr);
    state.p_w_b = Helper::template evaluate_rd<3, 0>(position_knots_, u_, inv_dt_s_);
    state.v_w_b = Helper::template evaluate_rd<3, 1>(position_knots_, u_, inv_dt_s_);
    if (with_acceleration) {
      state.a_w_b = Helper::template evaluate_rd<3, 2>(position_knots_, u_, inv_dt_s_);
    }
    return state;
  }

  Eigen::Quaterniond rotation() const
  {
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    Helper::evaluate_lie_so3(rotation_knots_, u_, inv_dt_s_, &q, nullptr, nullptr);
    return q;
  }

  Eigen::Vector3d angular_velocity_body() const
  {
    Eigen::Vector3d omega = Eigen::Vector3d::Zero();
    Helper::evaluate_lie_so3(rotation_knots_, u_, inv_dt_s_, nullptr, &omega, nullptr);
    return omega;
  }

  Eigen::Vector3d position_world() const
  {
    return Helper::template evaluate_rd<3, 0>(position_knots_, u_, inv_dt_s_);
  }

  Eigen::Vector3d velocity_world() const
  {
    return Helper::template evaluate_rd<3, 1>(position_knots_, u_, inv_dt_s_);
  }

  Eigen::Vector3d acceleration_world() const
  {
    return Helper::template evaluate_rd<3, 2>(position_knots_, u_, inv_dt_s_);
  }

  // The IMU NURBS factor needs the predicted gyroscope and accelerometer
  // readings expressed in the body frame given gravity in world frame.
  Eigen::Vector3d predicted_gyroscope(const Eigen::Vector3d & gyro_bias = Eigen::Vector3d::Zero()) const
  {
    return angular_velocity_body() + gyro_bias;
  }

  Eigen::Vector3d predicted_accelerometer(
    const Eigen::Vector3d & gravity_world,
    const Eigen::Vector3d & accel_bias = Eigen::Vector3d::Zero()) const
  {
    const Eigen::Quaterniond q = rotation();
    const Eigen::Vector3d a_w = acceleration_world();
    return q.inverse() * (a_w - gravity_world) + accel_bias;
  }

private:
  const std::array<Eigen::Quaterniond, N_> & rotation_knots_;
  const std::array<Eigen::Vector3d, N_> & position_knots_;
  const double u_{0.0};
  const double inv_dt_s_{1.0};
};

}  // namespace spline
}  // namespace gaussian_lic_tracking
