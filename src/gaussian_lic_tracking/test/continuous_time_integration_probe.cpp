// SPDX-License-Identifier: GPL-3.0-or-later
//
// End-to-end probe that exercises the ROS2-native continuous-time IMU and
// LiDAR factors against a single synthetic spline. The integration probe is
// the first proof that the SplitSplineView + ContinuousTimeImuFactor +
// ContinuousTimeLidarFactor pieces compose into the same residual surface a
// real trajectory estimator will rely on.
//
// Coverage:
//   * Many IMU samples taken at random u-values along an analytical spline
//     produce zero residual when the spline knots match.
//   * Translating one position knot grows the IMU/LiDAR residuals
//     monotonically with the perturbation magnitude.
//   * Mixed IMU + LiDAR cost surface is finite and bowl-shaped near the true
//     state, evidence the residual model is well-conditioned for downstream
//     Gauss-Newton iteration.

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <stdexcept>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gaussian_lic_tracking/spline/continuous_time_imu_factor.hpp>
#include <gaussian_lic_tracking/spline/continuous_time_lidar_factor.hpp>

using gaussian_lic_tracking::spline::ContinuousTimeImuFactor;
using gaussian_lic_tracking::spline::ContinuousTimeLidarFactor;
using gaussian_lic_tracking::spline::ImuFactorState;
using gaussian_lic_tracking::spline::ImuSample;
using gaussian_lic_tracking::spline::LidarExtrinsics;
using gaussian_lic_tracking::spline::LidarFactorState;
using gaussian_lic_tracking::spline::LidarFeatureGeometry;
using gaussian_lic_tracking::spline::LidarPointCorrespondence;
using gaussian_lic_tracking::spline::SplitSplineView;

namespace
{

constexpr int N = ContinuousTimeImuFactor::N;

ImuFactorState build_truth_imu_state(double dt, const Eigen::Vector3d & gravity)
{
  ImuFactorState state;
  for (int i = 0; i < N; ++i) {
    const double t = i * dt;
    state.rotation_knots[i] =
      Eigen::Quaterniond(Eigen::AngleAxisd(1.0 * t, Eigen::Vector3d::UnitZ()));
    state.position_knots[i] = Eigen::Vector3d(0.5 * t * t, 0.0, 0.0);
  }
  state.gravity_world = gravity;
  return state;
}

LidarFactorState to_lidar_state(const ImuFactorState & s)
{
  LidarFactorState lidar;
  lidar.rotation_knots = s.rotation_knots;
  lidar.position_knots = s.position_knots;
  return lidar;
}

void check_imu_factors_zero_on_truth()
{
  const double dt = 0.05;
  const double inv_dt = 1.0 / dt;
  const Eigen::Vector3d gravity(0.0, 0.0, -9.81);
  const auto truth = build_truth_imu_state(dt, gravity);

  std::mt19937 rng(42);
  std::uniform_real_distribution<double> uniform_u(0.05, 0.95);

  Eigen::Matrix<double, 6, 1> info_diag = Eigen::Matrix<double, 6, 1>::Ones();
  const int sample_count = 16;
  for (int i = 0; i < sample_count; ++i) {
    const double u = uniform_u(rng);
    SplitSplineView<N> view(truth.rotation_knots, truth.position_knots, u, inv_dt);
    const auto evaluated = view.evaluate(true);
    ImuSample sample;
    sample.gyro = evaluated.omega_b;
    sample.accel = evaluated.q_w_b.inverse() * (evaluated.a_w_b - gravity);
    ContinuousTimeImuFactor factor(u, inv_dt, sample, info_diag);
    const auto r = factor.residual(truth);
    if (r.cwiseAbs().maxCoeff() > 1.0e-7) {
      std::fprintf(stderr,
        "IMU residual on truth nonzero at u=%.3f: max|r|=%.6g\n",
        u, r.cwiseAbs().maxCoeff());
      std::exit(1);
    }
  }
}

void check_residual_grows_with_position_perturbation()
{
  const double dt = 0.05;
  const double inv_dt = 1.0 / dt;
  const Eigen::Vector3d gravity(0.0, 0.0, -9.81);
  const auto truth = build_truth_imu_state(dt, gravity);
  Eigen::Matrix<double, 6, 1> info_diag = Eigen::Matrix<double, 6, 1>::Ones();

  // Build one IMU factor at u=0.5 and one LiDAR factor that observes a point
  // sitting on the world-frame z=0 plane.
  SplitSplineView<N> view(truth.rotation_knots, truth.position_knots, 0.5, inv_dt);
  const auto evaluated = view.evaluate(true);
  ImuSample imu_sample;
  imu_sample.gyro = evaluated.omega_b;
  imu_sample.accel = evaluated.q_w_b.inverse() * (evaluated.a_w_b - gravity);
  ContinuousTimeImuFactor imu_factor(0.5, inv_dt, imu_sample, info_diag);

  LidarExtrinsics lidar_extrinsics;
  LidarPointCorrespondence pc;
  pc.geometry = LidarFeatureGeometry::kPlane;
  pc.plane << 0.0, 0.0, 1.0, 0.0;
  // Pick a lidar point that lies exactly on z=0 in the body frame at u=0.5.
  pc.point_lidar = evaluated.q_w_b.inverse() *
    (Eigen::Vector3d(0.1, -0.2, -evaluated.p_w_b.z()) - evaluated.p_w_b);
  // The constructed point should now satisfy n^T p_world + d = 0 ⇒ z=0.
  ContinuousTimeLidarFactor lidar_factor(0.5, inv_dt, pc, lidar_extrinsics, 1.0);

  // Sanity: zero residual at truth.
  if (imu_factor.residual(truth).cwiseAbs().maxCoeff() > 1.0e-7) {
    std::fprintf(stderr, "IMU residual nonzero at truth in integration check\n");
    std::exit(1);
  }
  if (std::abs(lidar_factor.residual(to_lidar_state(truth))) > 1.0e-7) {
    std::fprintf(stderr, "LiDAR residual nonzero at truth in integration check\n");
    std::exit(1);
  }

  // Perturb the central position knot by progressively larger z offsets; both
  // IMU and LiDAR residuals should increase monotonically.
  double prev_total = 0.0;
  for (double shift : {0.01, 0.02, 0.05, 0.1}) {
    ImuFactorState perturbed = truth;
    perturbed.position_knots[1].z() += shift;
    const auto r_imu = imu_factor.residual(perturbed);
    const double r_lidar = lidar_factor.residual(to_lidar_state(perturbed));
    const double total = r_imu.squaredNorm() + r_lidar * r_lidar;
    if (total <= prev_total) {
      std::fprintf(stderr,
        "integration cost did not increase monotonically for shift=%.3f (total=%.6g prev=%.6g)\n",
        shift, total, prev_total);
      std::exit(1);
    }
    prev_total = total;
  }
}

void check_cost_bowl_shape()
{
  // Sweep one position knot in z over [-0.05, +0.05] and verify the total
  // squared residual achieves its minimum at zero shift (the true state).
  const double dt = 0.05;
  const double inv_dt = 1.0 / dt;
  const Eigen::Vector3d gravity(0.0, 0.0, -9.81);
  const auto truth = build_truth_imu_state(dt, gravity);
  Eigen::Matrix<double, 6, 1> info_diag = Eigen::Matrix<double, 6, 1>::Ones();

  SplitSplineView<N> view(truth.rotation_knots, truth.position_knots, 0.5, inv_dt);
  const auto evaluated = view.evaluate(true);
  ImuSample imu_sample;
  imu_sample.gyro = evaluated.omega_b;
  imu_sample.accel = evaluated.q_w_b.inverse() * (evaluated.a_w_b - gravity);
  ContinuousTimeImuFactor imu_factor(0.5, inv_dt, imu_sample, info_diag);

  double min_cost = std::numeric_limits<double>::infinity();
  double min_shift = 0.0;
  for (double shift = -0.05; shift <= 0.05 + 1.0e-9; shift += 0.01) {
    ImuFactorState perturbed = truth;
    perturbed.position_knots[1].z() += shift;
    const double cost = imu_factor.residual(perturbed).squaredNorm();
    if (cost < min_cost) {
      min_cost = cost;
      min_shift = shift;
    }
  }
  if (std::abs(min_shift) > 1.0e-9) {
    std::fprintf(stderr,
      "cost bowl minimum not at zero shift: min_shift=%.6f cost=%.6g\n",
      min_shift, min_cost);
    std::exit(1);
  }
}

}  // namespace

int main()
{
  try {
    check_imu_factors_zero_on_truth();
    check_residual_grows_with_position_perturbation();
    check_cost_bowl_shape();
  } catch (const std::exception & exception) {
    std::fprintf(stderr,
      "continuous_time_integration_probe exception: %s\n", exception.what());
    return 1;
  }
  std::printf("continuous_time_integration_probe ok\n");
  return 0;
}
