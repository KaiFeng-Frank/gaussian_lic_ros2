// SPDX-License-Identifier: GPL-3.0-or-later
//
// Verifies the ROS2 TrajectoryEstimator (the Coco-LIC TrajectoryEstimator
// port) drives Ceres over the new continuous-time factors and recovers a
// known-true spline trajectory from a perturbed initial guess.

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <stdexcept>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gaussian_lic_tracking/spline/ceres_spline_helper.hpp>
#include <gaussian_lic_tracking/spline/continuous_time_imu_factor.hpp>
#include <gaussian_lic_tracking/spline/continuous_time_lidar_factor.hpp>
#include <gaussian_lic_tracking/spline/split_spline_view.hpp>
#include <gaussian_lic_tracking/spline/trajectory_estimator.hpp>

using gaussian_lic_tracking::spline::CubicSplineHelper;
using gaussian_lic_tracking::spline::ImuSample;
using gaussian_lic_tracking::spline::LidarExtrinsics;
using gaussian_lic_tracking::spline::LidarFeatureGeometry;
using gaussian_lic_tracking::spline::LidarPointCorrespondence;
using gaussian_lic_tracking::spline::SplitSplineView;
using gaussian_lic_tracking::spline::TrajectoryEstimator;
using gaussian_lic_tracking::spline::TrajectoryEstimatorOptions;

namespace
{

struct TruthSpline
{
  std::vector<Eigen::Quaterniond> rotation_knots;
  std::vector<Eigen::Vector3d> position_knots;
  double dt_s{0.05};
};

TruthSpline build_truth(double dt_s, int knot_count)
{
  TruthSpline t;
  t.dt_s = dt_s;
  t.rotation_knots.reserve(knot_count);
  t.position_knots.reserve(knot_count);
  for (int i = 0; i < knot_count; ++i) {
    const double time = i * dt_s;
    t.rotation_knots.emplace_back(
      Eigen::AngleAxisd(0.5 * time, Eigen::Vector3d::UnitZ()));
    t.position_knots.emplace_back(0.5 * time * time, 0.0, 0.0);
  }
  return t;
}

ImuSample synthesize_imu(const TruthSpline & truth, double t_s, const Eigen::Vector3d & gravity)
{
  const double inv_dt = 1.0 / truth.dt_s;
  const int idx = static_cast<int>(std::floor(t_s / truth.dt_s));
  const double u = (t_s - idx * truth.dt_s) / truth.dt_s;
  std::array<Eigen::Quaterniond, 4> rot_knots;
  std::array<Eigen::Vector3d, 4> pos_knots;
  for (int i = 0; i < 4; ++i) {
    rot_knots[i] = truth.rotation_knots[idx - 1 + i];
    pos_knots[i] = truth.position_knots[idx - 1 + i];
  }
  SplitSplineView<4> view(rot_knots, pos_knots, u, inv_dt);
  const auto state = view.evaluate(true);
  ImuSample sample;
  sample.gyro = state.omega_b;
  sample.accel = state.q_w_b.inverse() * (state.a_w_b - gravity);
  return sample;
}

double max_position_drift(
  const TrajectoryEstimator & estimator,
  const TruthSpline & truth)
{
  const auto pos = estimator.position_knots();
  double worst = 0.0;
  for (std::size_t i = 0; i < pos.size(); ++i) {
    worst = std::max(worst, (pos[i] - truth.position_knots[i]).cwiseAbs().maxCoeff());
  }
  return worst;
}

double max_rotation_drift(
  const TrajectoryEstimator & estimator,
  const TruthSpline & truth)
{
  const auto rot = estimator.rotation_knots();
  double worst = 0.0;
  for (std::size_t i = 0; i < rot.size(); ++i) {
    const Eigen::Quaterniond delta = rot[i].inverse() * truth.rotation_knots[i];
    Eigen::Quaterniond d = delta;
    if (d.w() < 0.0) {
      d.coeffs() *= -1.0;
    }
    worst = std::max(worst, 2.0 * std::acos(std::min(1.0, std::max(-1.0, d.w()))));
  }
  return worst;
}

void check_zero_residual_when_seeded_with_truth()
{
  const double dt = 0.05;
  const auto truth = build_truth(dt, 8);
  TrajectoryEstimator estimator(dt);
  estimator.set_knots(truth.rotation_knots, truth.position_knots);
  estimator.set_gravity_world(Eigen::Vector3d(0.0, 0.0, -9.81));

  Eigen::Matrix<double, 6, 1> info_diag = Eigen::Matrix<double, 6, 1>::Ones();
  for (double t = 0.1; t < 0.30; t += 0.02) {
    const auto sample = synthesize_imu(truth, t, estimator.gravity_world());
    if (!estimator.add_imu_factor(t, sample, info_diag)) {
      std::fprintf(stderr, "IMU factor at t=%.3f rejected\n", t);
      std::exit(1);
    }
  }

  TrajectoryEstimatorOptions options;
  options.max_num_iterations = 5;
  const auto summary = estimator.solve(options);

  if (summary.final_cost > 1.0e-12) {
    std::fprintf(stderr,
      "expected zero cost when seeded with truth, got final=%.6g (%s)\n",
      summary.final_cost, summary.brief_report.c_str());
    std::exit(1);
  }
  if (max_position_drift(estimator, truth) > 1.0e-9) {
    std::fprintf(stderr, "position drift on truth seed: %.6g\n",
      max_position_drift(estimator, truth));
    std::exit(1);
  }
}

void check_converges_from_position_perturbation()
{
  // Constant-offset position perturbations leave a_w invariant, so we
  // perturb a single interior knot. The resulting acceleration error must be
  // pulled back to zero by the IMU residual.
  const double dt = 0.05;
  const auto truth = build_truth(dt, 8);
  TrajectoryEstimator estimator(dt);

  auto perturbed_positions = truth.position_knots;
  perturbed_positions[3].z() += 0.05;
  estimator.set_knots(truth.rotation_knots, perturbed_positions);
  estimator.set_gravity_world(Eigen::Vector3d(0.0, 0.0, -9.81));

  Eigen::Matrix<double, 6, 1> info_diag;
  info_diag << 1.0, 1.0, 1.0, 100.0, 100.0, 100.0;
  for (double t = 0.06; t < 0.34; t += 0.005) {
    const auto sample = synthesize_imu(truth, t, estimator.gravity_world());
    estimator.add_imu_factor(t, sample, info_diag);
  }

  TrajectoryEstimatorOptions options;
  options.max_num_iterations = 80;
  options.function_tolerance = 1.0e-12;
  options.parameter_tolerance = 1.0e-12;
  options.gradient_tolerance = 1.0e-14;
  options.hold_gyro_bias_constant = true;
  options.hold_accel_bias_constant = true;
  options.hold_gravity_constant = true;
  const auto summary = estimator.solve(options);

  // The IMU residual is invariant to a constant + linear position offset
  // (only second derivatives appear in the accel residual), so the solver is
  // not expected to recover the *exact* truth for an arbitrary position
  // perturbation without auxiliary priors. The probe asserts:
  //   (1) cost is driven below epsilon — the residual surface is reached;
  //   (2) drift on the perturbed knot is reduced substantially.
  if (summary.final_cost > 1.0e-6) {
    std::fprintf(stderr,
      "solver did not drive IMU cost to zero: initial=%.6g final=%.6g (%s)\n",
      summary.initial_cost, summary.final_cost, summary.brief_report.c_str());
    std::exit(1);
  }
  if (summary.final_cost > summary.initial_cost) {
    std::fprintf(stderr,
      "solver increased cost: initial=%.6g final=%.6g\n",
      summary.initial_cost, summary.final_cost);
    std::exit(1);
  }
  const double final_z = estimator.position_knots()[3].z();
  const double initial_drift = 0.05;
  const double final_drift = std::abs(final_z - truth.position_knots[3].z());
  if (final_drift > 0.6 * initial_drift) {
    std::fprintf(stderr,
      "perturbed central knot was not pulled toward truth: initial_drift=%.4f final_drift=%.4f\n",
      initial_drift, final_drift);
    std::exit(1);
  }
}

void check_lidar_plane_factor_pulls_position()
{
  // Truth pose at the spline center is at z=0; perturb position knots by
  // +0.10 m in z and add LiDAR plane factors observing a flat z=0 ground.
  // After the solve the central position knots should be pulled back toward
  // zero.
  const double dt = 0.05;
  const auto truth = build_truth(dt, 8);
  TrajectoryEstimator estimator(dt);

  auto perturbed_positions = truth.position_knots;
  for (auto & p : perturbed_positions) {
    p.z() += 0.10;
  }
  estimator.set_knots(truth.rotation_knots, perturbed_positions);

  LidarExtrinsics extrinsics;
  LidarPointCorrespondence pc;
  pc.geometry = LidarFeatureGeometry::kPlane;
  pc.plane << 0.0, 0.0, 1.0, 0.0;
  for (double t = 0.06; t < 0.34; t += 0.005) {
    // 12 observations of ground points at different (x, y) — equivalent to a
    // multi-beam LiDAR scan returning ground.
    for (double offset_x = -0.5; offset_x <= 0.5; offset_x += 0.2) {
      pc.point_lidar = Eigen::Vector3d(offset_x, 0.0, -truth.position_knots[3].z());
      estimator.add_lidar_factor(t, pc, extrinsics, 1.0);
    }
  }

  TrajectoryEstimatorOptions options;
  options.max_num_iterations = 50;
  options.hold_gyro_bias_constant = true;
  options.hold_accel_bias_constant = true;
  options.hold_gravity_constant = true;
  const auto summary = estimator.solve(options);

  // Central knot z should be much closer to 0 than the starting 0.10 m.
  const double final_z = estimator.position_knots()[3].z();
  if (std::abs(final_z) > 0.03) {
    std::fprintf(stderr,
      "lidar plane factor failed to pull central z to ground: final=%.6f (%s)\n",
      final_z, summary.brief_report.c_str());
    std::exit(1);
  }
  if (summary.final_cost > summary.initial_cost) {
    std::fprintf(stderr,
      "lidar solve increased cost: initial=%.6g final=%.6g\n",
      summary.initial_cost, summary.final_cost);
    std::exit(1);
  }
}

}  // namespace

int main()
{
  try {
    check_zero_residual_when_seeded_with_truth();
    check_converges_from_position_perturbation();
    check_lidar_plane_factor_pulls_position();
  } catch (const std::exception & exception) {
    std::fprintf(stderr, "trajectory_estimator_probe exception: %s\n", exception.what());
    return 1;
  }
  std::printf("trajectory_estimator_probe ok\n");
  return 0;
}
