// SPDX-License-Identifier: GPL-3.0-or-later
//
// Asserts the closed-form analytic Jacobians from `analytic_jacobians.hpp`
// match finite-difference Jacobians on the continuous-time IMU and LiDAR
// residuals at multiple knot configurations.

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gaussian_lic_tracking/spline/analytic_jacobians.hpp>
#include <gaussian_lic_tracking/spline/continuous_time_imu_factor.hpp>
#include <gaussian_lic_tracking/spline/continuous_time_lidar_factor.hpp>

using gaussian_lic_tracking::spline::analytic_imu_accel_bias_jacobian;
using gaussian_lic_tracking::spline::analytic_imu_gravity_jacobian;
using gaussian_lic_tracking::spline::analytic_imu_gyro_bias_jacobian;
using gaussian_lic_tracking::spline::analytic_imu_position_knot_jacobian;
using gaussian_lic_tracking::spline::analytic_lidar_plane_position_knot_jacobian;
using gaussian_lic_tracking::spline::ContinuousTimeImuFactor;
using gaussian_lic_tracking::spline::ContinuousTimeLidarFactor;
using gaussian_lic_tracking::spline::ImuFactorState;
using gaussian_lic_tracking::spline::ImuSample;
using gaussian_lic_tracking::spline::LidarExtrinsics;
using gaussian_lic_tracking::spline::LidarFactorState;
using gaussian_lic_tracking::spline::LidarFeatureGeometry;
using gaussian_lic_tracking::spline::LidarPointCorrespondence;
using gaussian_lic_tracking::spline::numeric_jacobian_accel_bias;
using gaussian_lic_tracking::spline::numeric_jacobian_gravity;
using gaussian_lic_tracking::spline::numeric_jacobian_gyro_bias;
using gaussian_lic_tracking::spline::numeric_jacobian_position_knot;
using gaussian_lic_tracking::spline::SplitSplineView;

namespace
{

constexpr int N = ContinuousTimeImuFactor::N;

ImuFactorState build_state(double dt, const Eigen::Vector3d & gravity)
{
  ImuFactorState state;
  for (int i = 0; i < N; ++i) {
    const double t = i * dt;
    state.rotation_knots[i] =
      Eigen::Quaterniond(Eigen::AngleAxisd(0.4 * t, Eigen::Vector3d::UnitZ())).normalized();
    state.position_knots[i] = Eigen::Vector3d(0.5 * t * t, 0.0, 0.1 * t);
  }
  state.gyro_bias = Eigen::Vector3d(0.001, -0.002, 0.003);
  state.accel_bias = Eigen::Vector3d(0.05, -0.04, 0.03);
  state.gravity_world = gravity;
  return state;
}

bool nearly_equal(
  const Eigen::Matrix<double, 6, 3> & a,
  const Eigen::Matrix<double, 6, 3> & b,
  double tol)
{
  return (a - b).cwiseAbs().maxCoeff() <= tol;
}

void check_imu_bias_jacobians()
{
  Eigen::Matrix<double, 6, 1> info_diag;
  info_diag << 100.0, 100.0, 100.0, 10.0, 10.0, 10.0;
  const auto state = build_state(0.05, Eigen::Vector3d(0.0, 0.0, -9.81));
  ImuSample sample;
  sample.gyro = Eigen::Vector3d(0.0, 0.0, 0.1);
  sample.accel = Eigen::Vector3d(0.0, 0.0, 9.5);
  ContinuousTimeImuFactor factor(0.5, 1.0 / 0.05, sample, info_diag);

  const auto a_gyro = analytic_imu_gyro_bias_jacobian(info_diag);
  const auto n_gyro = numeric_jacobian_gyro_bias(factor, state);
  if (!nearly_equal(a_gyro, n_gyro, 1.0e-7)) {
    std::fprintf(stderr,
      "gyro bias jacobian analytic vs numeric mismatch (%.6g)\n",
      (a_gyro - n_gyro).cwiseAbs().maxCoeff());
    std::exit(1);
  }
  const auto a_acc = analytic_imu_accel_bias_jacobian(info_diag);
  const auto n_acc = numeric_jacobian_accel_bias(factor, state);
  if (!nearly_equal(a_acc, n_acc, 1.0e-7)) {
    std::fprintf(stderr,
      "accel bias jacobian analytic vs numeric mismatch (%.6g)\n",
      (a_acc - n_acc).cwiseAbs().maxCoeff());
    std::exit(1);
  }
}

void check_imu_gravity_jacobian()
{
  Eigen::Matrix<double, 6, 1> info_diag;
  info_diag << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
  const auto state = build_state(0.05, Eigen::Vector3d(0.0, 0.0, -9.81));
  ImuSample sample;
  sample.gyro = Eigen::Vector3d::Zero();
  sample.accel = Eigen::Vector3d::Zero();
  for (double u : {0.1, 0.5, 0.9}) {
    ContinuousTimeImuFactor factor(u, 1.0 / 0.05, sample, info_diag);
    SplitSplineView<N> view(state.rotation_knots, state.position_knots, u, 1.0 / 0.05);
    const auto a = analytic_imu_gravity_jacobian(view.rotation(), info_diag);
    const auto n = numeric_jacobian_gravity(factor, state);
    if (!nearly_equal(a, n, 1.0e-6)) {
      std::fprintf(stderr,
        "gravity jacobian mismatch at u=%.2f (%.6g)\n",
        u, (a - n).cwiseAbs().maxCoeff());
      std::exit(1);
    }
  }
}

void check_imu_position_knot_jacobian()
{
  Eigen::Matrix<double, 6, 1> info_diag;
  info_diag << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
  const auto state = build_state(0.05, Eigen::Vector3d(0.0, 0.0, -9.81));
  ImuSample sample;
  sample.gyro = Eigen::Vector3d::Zero();
  sample.accel = Eigen::Vector3d::Zero();
  for (double u : {0.2, 0.5, 0.8}) {
    ContinuousTimeImuFactor factor(u, 1.0 / 0.05, sample, info_diag);
    SplitSplineView<N> view(state.rotation_knots, state.position_knots, u, 1.0 / 0.05);
    const auto rotation = view.rotation();
    for (int knot = 0; knot < N; ++knot) {
      const auto analytic =
        analytic_imu_position_knot_jacobian(u, 1.0 / 0.05, rotation, info_diag, knot);
      const auto numeric =
        numeric_jacobian_position_knot(factor, state, knot, 1.0e-6);
      if (!nearly_equal(analytic, numeric, 1.0e-5)) {
        std::fprintf(stderr,
          "position knot jacobian mismatch knot=%d u=%.2f (%.6g)\n",
          knot, u, (analytic - numeric).cwiseAbs().maxCoeff());
        std::exit(1);
      }
    }
  }
}

void check_lidar_plane_position_knot_jacobian()
{
  // Plane residual is purely linear in position knots, so analytic and
  // finite-difference Jacobians should agree to machine epsilon.
  LidarFactorState state;
  for (int i = 0; i < N; ++i) {
    const double t = i * 0.05;
    state.rotation_knots[i] =
      Eigen::Quaterniond(Eigen::AngleAxisd(0.1 * t, Eigen::Vector3d::UnitZ())).normalized();
    state.position_knots[i] = Eigen::Vector3d(t, 0.5 * t, 0.2);
  }
  LidarExtrinsics extrinsics;
  // Add a non-identity world→map rotation to exercise the jacobian's R term.
  extrinsics.q_world_to_map =
    Eigen::Quaterniond(Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitY()));
  LidarPointCorrespondence pc;
  pc.geometry = LidarFeatureGeometry::kPlane;
  pc.plane << 0.0, 0.0, 1.0, -0.1;
  pc.point_lidar = Eigen::Vector3d(0.4, 0.2, 0.0);
  const double weight = 1.7;

  for (double u : {0.25, 0.5, 0.75}) {
    ContinuousTimeLidarFactor factor(u, 1.0 / 0.05, pc, extrinsics, weight);
    const double r0 = factor.residual(state);
    for (int knot = 0; knot < N; ++knot) {
      const auto analytic = analytic_lidar_plane_position_knot_jacobian(
        u, 1.0 / 0.05, weight, pc.plane.head<3>(), extrinsics.q_world_to_map, knot);

      Eigen::Matrix<double, 1, 3> numeric;
      const double eps = 1.0e-6;
      for (int axis = 0; axis < 3; ++axis) {
        LidarFactorState plus = state;
        LidarFactorState minus = state;
        plus.position_knots[knot][axis] += eps;
        minus.position_knots[knot][axis] -= eps;
        numeric[axis] = (factor.residual(plus) - factor.residual(minus)) / (2.0 * eps);
      }
      if ((analytic - numeric).cwiseAbs().maxCoeff() > 1.0e-6) {
        std::fprintf(stderr,
          "lidar plane position knot jacobian mismatch knot=%d u=%.2f (analytic=%s numeric=%s) r0=%.4f\n",
          knot, u, "(see code)", "(see code)", r0);
        std::exit(1);
      }
    }
  }
}

}  // namespace

int main()
{
  try {
    check_imu_bias_jacobians();
    check_imu_gravity_jacobian();
    check_imu_position_knot_jacobian();
    check_lidar_plane_position_knot_jacobian();
  } catch (const std::exception & exception) {
    std::fprintf(stderr,
      "analytic_jacobians_probe exception: %s\n", exception.what());
    return 1;
  }
  std::printf("analytic_jacobians_probe ok\n");
  return 0;
}
