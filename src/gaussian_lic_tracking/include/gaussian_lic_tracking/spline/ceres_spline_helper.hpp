// SPDX-License-Identifier: GPL-3.0-or-later
//
// ROS2-native port of Coco-LIC's CeresSplineHelper
// (external/Coco-LIC/src/spline/ceres_spline_helper.h).
//
// The upstream helper provides analytic evaluation of:
//   * Lie-group cumulative B-splines (used for SO(3) orientation) and
//   * Euclidean uniform B-splines (used for world position, biases, gravity).
//
// This port stays header-only and depends only on Eigen plus the inline SO(3)
// helpers in `so3_ops.hpp`. Sophus is not introduced: SO(3) tangent math is
// expressed directly through quaternion log/exp, mirroring the conventions
// already used by `imu_preintegrator.cpp` and `sliding_window_optimizer.cpp`.
//
// Conventions:
//   * Position knots: world-frame Eigen::Vector3d (DIM=3). The Euclidean
//     evaluator returns world-frame position / velocity / acceleration when
//     called with DERIV=0/1/2.
//   * Orientation knots: Eigen::Quaterniond (Sophus::SO3 internal storage is
//     also a unit quaternion in upstream).
//   * `vel` / `accel` outputs from `evaluate_lie_so3` are body-frame angular
//     velocity and angular acceleration, matching upstream comments.

#pragma once

#include <array>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gaussian_lic_tracking/spline/so3_ops.hpp>
#include <gaussian_lic_tracking/spline/spline_common.hpp>

namespace gaussian_lic_tracking
{
namespace spline
{

template <int N_>
struct CeresSplineHelper
{
  static constexpr int N = N_;
  static constexpr int DEG = N_ - 1;

  using MatN = Eigen::Matrix<double, N_, N_>;
  using VecN = Eigen::Matrix<double, N_, 1>;

  static const MatN & blending_matrix()
  {
    static const MatN m = compute_blending_matrix<N_, double, false>();
    return m;
  }

  static const MatN & cumulative_blending_matrix()
  {
    static const MatN m = compute_blending_matrix<N_, double, true>();
    return m;
  }

  static const MatN & base_coefficients()
  {
    static const MatN m = compute_base_coefficients<N_, double>();
    return m;
  }

  template <int Derivative>
  static void base_coefficients_with_time(VecN & result, double u)
  {
    result.setZero();
    if constexpr (Derivative < N_) {
      result[Derivative] = base_coefficients()(Derivative, Derivative);
      double power = u;
      for (int j = Derivative + 1; j < N_; ++j) {
        result[j] = base_coefficients()(Derivative, j) * power;
        power *= u;
      }
    }
  }

  template <int Derivative, typename T>
  static void base_coefficients_with_time_t(Eigen::Matrix<T, N_, 1> & result, T u)
  {
    result.setZero();
    if constexpr (Derivative < N_) {
      result[Derivative] = T(base_coefficients()(Derivative, Derivative));
      T power = u;
      for (int j = Derivative + 1; j < N_; ++j) {
        result[j] = T(base_coefficients()(Derivative, j)) * power;
        power *= u;
      }
    }
  }

  // Evaluate a uniform Euclidean B-spline at normalized time u in [0, 1).
  // `knots` is an array of N pointers to the first scalar of an Eigen vector
  // of dimension DIM. `inv_dt` is 1/Δt where Δt is the spacing between knots
  // in seconds; pass inv_dt = 1 for derivative=0 in unit knot-time.
  template <int DIM, int DERIV>
  static Eigen::Matrix<double, DIM, 1> evaluate_rd(
    const std::array<Eigen::Matrix<double, DIM, 1>, N_> & knots,
    double u,
    double inv_dt)
  {
    VecN raw;
    base_coefficients_with_time<DERIV>(raw, u);
    const VecN coeff =
      std::pow(inv_dt, DERIV) * blending_matrix() * raw;

    Eigen::Matrix<double, DIM, 1> out;
    out.setZero();
    for (int i = 0; i < N_; ++i) {
      out += coeff[i] * knots[i];
    }
    return out;
  }

  template <int DIM, int DERIV, typename T>
  static Eigen::Matrix<T, DIM, 1> evaluate_rd_t(
    const std::array<Eigen::Matrix<T, DIM, 1>, N_> & knots,
    T u,
    T inv_dt)
  {
    using std::pow;
    Eigen::Matrix<T, N_, 1> raw;
    base_coefficients_with_time_t<DERIV, T>(raw, u);
    const Eigen::Matrix<T, N_, 1> coeff =
      pow(inv_dt, T(static_cast<double>(DERIV))) *
      blending_matrix().template cast<T>() * raw;

    Eigen::Matrix<T, DIM, 1> out = Eigen::Matrix<T, DIM, 1>::Zero();
    for (int i = 0; i < N_; ++i) {
      out += coeff[i] * knots[i];
    }
    return out;
  }

  // Evaluate the cumulative SO(3) B-spline at normalized time u in [0, 1).
  // Outputs:
  //   * rotation_out  – SO(3) orientation R(u) (optional)
  //   * angular_vel_out – body-frame angular velocity ω (optional)
  //   * angular_acc_out – body-frame angular acceleration α (optional)
  // Conventions follow Coco-LIC's evaluate_lie<Sophus::SO3, double>.
  static void evaluate_lie_so3(
    const std::array<Eigen::Quaterniond, N_> & knots,
    double u,
    double inv_dt,
    Eigen::Quaterniond * rotation_out = nullptr,
    Eigen::Vector3d * angular_vel_out = nullptr,
    Eigen::Vector3d * angular_acc_out = nullptr)
  {
    VecN raw;
    base_coefficients_with_time<0>(raw, u);
    VecN coeff = cumulative_blending_matrix() * raw;

    VecN dcoeff = VecN::Zero();
    VecN ddcoeff = VecN::Zero();

    const bool need_vel = angular_vel_out != nullptr || angular_acc_out != nullptr;
    const bool need_acc = angular_acc_out != nullptr;

    if (need_vel) {
      base_coefficients_with_time<1>(raw, u);
      dcoeff = inv_dt * cumulative_blending_matrix() * raw;
    }
    if (need_acc) {
      base_coefficients_with_time<2>(raw, u);
      ddcoeff = inv_dt * inv_dt * cumulative_blending_matrix() * raw;
    }

    Eigen::Quaterniond rotation = knots[0];
    Eigen::Vector3d rot_vel = Eigen::Vector3d::Zero();
    Eigen::Vector3d rot_accel = Eigen::Vector3d::Zero();

    for (int i = 0; i < DEG; ++i) {
      const Eigen::Quaterniond r01 = (knots[i].normalized().inverse() * knots[i + 1].normalized()).normalized();
      const Eigen::Vector3d delta = quaternion_log(r01);

      const Eigen::Quaterniond exp_kdelta = quaternion_exp(delta * coeff[i + 1]);
      rotation = (rotation * exp_kdelta).normalized();

      if (need_vel) {
        const Eigen::Matrix3d A = adjoint(exp_kdelta.inverse());
        rot_vel = A * rot_vel;
        const Eigen::Vector3d rot_vel_current = delta * dcoeff[i + 1];
        rot_vel += rot_vel_current;

        if (need_acc) {
          rot_accel = A * rot_accel;
          rot_accel += ddcoeff[i + 1] * delta + lie_bracket(rot_vel, rot_vel_current);
        }
      }
    }

    if (rotation_out) {
      *rotation_out = rotation;
    }
    if (angular_vel_out) {
      *angular_vel_out = rot_vel;
    }
    if (angular_acc_out) {
      *angular_acc_out = rot_accel;
    }
  }

  template <typename T>
  static void evaluate_lie_so3_t(
    const std::array<Eigen::Quaternion<T>, N_> & knots,
    T u,
    T inv_dt,
    Eigen::Quaternion<T> * rotation_out = nullptr,
    Eigen::Matrix<T, 3, 1> * angular_vel_out = nullptr,
    Eigen::Matrix<T, 3, 1> * angular_acc_out = nullptr)
  {
    Eigen::Matrix<T, N_, 1> raw;
    base_coefficients_with_time_t<0, T>(raw, u);
    Eigen::Matrix<T, N_, 1> coeff =
      cumulative_blending_matrix().template cast<T>() * raw;

    Eigen::Matrix<T, N_, 1> dcoeff = Eigen::Matrix<T, N_, 1>::Zero();
    Eigen::Matrix<T, N_, 1> ddcoeff = Eigen::Matrix<T, N_, 1>::Zero();

    const bool need_vel = angular_vel_out != nullptr || angular_acc_out != nullptr;
    const bool need_acc = angular_acc_out != nullptr;

    if (need_vel) {
      base_coefficients_with_time_t<1, T>(raw, u);
      dcoeff = inv_dt * cumulative_blending_matrix().template cast<T>() * raw;
    }
    if (need_acc) {
      base_coefficients_with_time_t<2, T>(raw, u);
      ddcoeff = inv_dt * inv_dt * cumulative_blending_matrix().template cast<T>() * raw;
    }

    Eigen::Quaternion<T> rotation = knots[0];
    Eigen::Matrix<T, 3, 1> rot_vel = Eigen::Matrix<T, 3, 1>::Zero();
    Eigen::Matrix<T, 3, 1> rot_accel = Eigen::Matrix<T, 3, 1>::Zero();

    for (int i = 0; i < DEG; ++i) {
      const Eigen::Quaternion<T> r01 =
        (knots[i].normalized().inverse() * knots[i + 1].normalized()).normalized();
      const Eigen::Matrix<T, 3, 1> delta = quaternion_log_t<T>(r01);

      const Eigen::Quaternion<T> exp_kdelta = quaternion_exp_t<T>(delta * coeff[i + 1]);
      rotation = (rotation * exp_kdelta).normalized();

      if (need_vel) {
        const Eigen::Matrix<T, 3, 3> A = adjoint_t<T>(exp_kdelta.inverse());
        rot_vel = A * rot_vel;
        const Eigen::Matrix<T, 3, 1> rot_vel_current = delta * dcoeff[i + 1];
        rot_vel += rot_vel_current;

        if (need_acc) {
          rot_accel = A * rot_accel;
          rot_accel +=
            ddcoeff[i + 1] * delta + lie_bracket_t(rot_vel, rot_vel_current);
        }
      }
    }

    if (rotation_out) {
      *rotation_out = rotation;
    }
    if (angular_vel_out) {
      *angular_vel_out = rot_vel;
    }
    if (angular_acc_out) {
      *angular_acc_out = rot_accel;
    }
  }
};

using CubicSplineHelper = CeresSplineHelper<kPositionSplineOrder>;

}  // namespace spline
}  // namespace gaussian_lic_tracking
