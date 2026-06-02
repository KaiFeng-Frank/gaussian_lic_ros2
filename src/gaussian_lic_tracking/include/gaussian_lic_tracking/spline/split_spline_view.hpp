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

  // Analytic Jacobian of the SO(3) value R(u) w.r.t. the rotation control
  // knots. `d_val_d_knot[i]` is the 3x3 block mapping a right perturbation of
  // knot i (knot_i -> knot_i * exp(eps_i)) to the resulting right perturbation
  // of R(u) (R(u) -> R(u) * exp(delta)): delta ~= sum_i d_val_d_knot[i]*eps_i.
  // Ported verbatim from Coco-LIC So3SplineView::EvaluateRpNURBS
  // (external/Coco-LIC/src/odom/factor/analytic_diff/so3_spline_view.h:244).
  struct So3KnotJacobian
  {
    std::array<Eigen::Matrix3d, N> d_val_d_knot;
  };

  Eigen::Quaterniond rotation_with_knot_jacobian(So3KnotJacobian & jac) const
  {
    typename Helper::VecN raw;
    Helper::template base_coefficients_with_time<0>(raw, u_);
    const typename Helper::VecN coeff = Helper::cumulative_blending_matrix() * raw;

    for (int i = 0; i < N; ++i) {
      jac.d_val_d_knot[i].setZero();
    }

    // Backward accumulation: A_accum_inv = exp(-k.d_{DEG-1}) ... exp(-k.d_0);
    // A_post_inv[i] caches the partial product matrix (A_post_inv[DEG] = I).
    std::array<Eigen::Matrix3d, N> A_post_inv;
    std::array<Eigen::Matrix3d, N> Jr_inv_delta;
    std::array<Eigen::Matrix3d, N> Jr_kdelta;
    A_post_inv[N - 1] = Eigen::Matrix3d::Identity();
    Eigen::Quaterniond A_accum_inv = Eigen::Quaterniond::Identity();
    for (int i = N - 2; i >= 0; --i) {
      const Eigen::Quaterniond r01 =
        (rotation_knots_[i].normalized().inverse() *
        rotation_knots_[i + 1].normalized()).normalized();
      const Eigen::Vector3d delta = quaternion_log(r01);
      const Eigen::Vector3d kdelta = coeff[i + 1] * delta;
      A_accum_inv = (A_accum_inv * quaternion_exp(Eigen::Vector3d(-kdelta))).normalized();
      Jr_inv_delta[i] = right_jacobian_inv_so3(delta);
      Jr_kdelta[i] = right_jacobian_so3(kdelta);
      A_post_inv[i] = A_accum_inv.toRotationMatrix();
    }
    const Eigen::Quaterniond res =
      (rotation_knots_[0].normalized() * A_accum_inv.inverse()).normalized();

    jac.d_val_d_knot[0] = A_post_inv[0];
    for (int i = 0; i < N - 1; ++i) {
      const Eigen::Matrix3d J_helper = coeff[i + 1] * A_post_inv[i + 1] * Jr_kdelta[i];
      jac.d_val_d_knot[i] -= J_helper * Jr_inv_delta[i].transpose();
      jac.d_val_d_knot[i + 1] = J_helper * Jr_inv_delta[i];
    }
    return res;
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

  // Analytic Jacobian of the world position p(u) w.r.t. the position knots.
  // The Euclidean B-spline is linear in its knots: p(u) = sum_i coeff[i]*knot_i,
  // so d_p/d_knot_i = coeff[i] * I_3. `d_val_d_knot[i]` stores the scalar
  // coeff[i] (mirrors Coco-LIC RdSplineView::JacobianStruct, which is diagonal).
  struct R3KnotJacobian
  {
    std::array<double, N> d_val_d_knot;
  };

  Eigen::Vector3d position_with_knot_jacobian(R3KnotJacobian & jac) const
  {
    typename Helper::VecN raw;
    Helper::template base_coefficients_with_time<0>(raw, u_);
    const typename Helper::VecN coeff = Helper::blending_matrix() * raw;
    Eigen::Vector3d p = Eigen::Vector3d::Zero();
    for (int i = 0; i < N; ++i) {
      jac.d_val_d_knot[i] = coeff[i];
      p += coeff[i] * position_knots_[i];
    }
    return p;
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
