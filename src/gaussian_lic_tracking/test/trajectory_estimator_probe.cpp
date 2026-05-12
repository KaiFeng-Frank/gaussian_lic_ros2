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

Eigen::Vector3d truth_position_at(const TruthSpline & truth, double t_s)
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
  return view.position_world();
}

Eigen::Vector3d velocity_at(
  const std::vector<Eigen::Quaterniond> & rotation_knots,
  const std::vector<Eigen::Vector3d> & position_knots,
  double dt_s,
  double t_s)
{
  const double inv_dt = 1.0 / dt_s;
  const int idx = static_cast<int>(std::floor(t_s / dt_s));
  const double u = (t_s - idx * dt_s) / dt_s;
  std::array<Eigen::Quaterniond, 4> rot_knots;
  std::array<Eigen::Vector3d, 4> pos_knots;
  for (int i = 0; i < 4; ++i) {
    rot_knots[i] = rotation_knots[idx - 1 + i];
    pos_knots[i] = position_knots[idx - 1 + i];
  }
  SplitSplineView<4> view(rot_knots, pos_knots, u, inv_dt);
  return view.velocity_world();
}

Eigen::Vector3d truth_velocity_at(const TruthSpline & truth, double t_s)
{
  return velocity_at(truth.rotation_knots, truth.position_knots, truth.dt_s, t_s);
}

Eigen::Quaterniond truth_orientation_at(const TruthSpline & truth, double t_s)
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
  return view.rotation();
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

void check_lidar_point_to_point_factor_pulls_map_alignment()
{
  // Persistent Gaussian/map anchors need a true 3D residual, not three
  // independent axis planes. A global translation offset is unobservable to IMU
  // acceleration, so point-to-point factors must pull it back.
  const double dt = 0.05;
  const auto truth = build_truth(dt, 8);
  TrajectoryEstimator estimator(dt);

  auto perturbed_positions = truth.position_knots;
  const Eigen::Vector3d offset(0.20, -0.12, 0.08);
  for (auto & p : perturbed_positions) {
    p += offset;
  }
  estimator.set_knots(truth.rotation_knots, perturbed_positions);

  LidarExtrinsics extrinsics;
  const std::vector<Eigen::Vector3d> points_lidar{
    Eigen::Vector3d(0.3, -0.2, 1.0),
    Eigen::Vector3d(-0.4, 0.1, 0.8),
    Eigen::Vector3d(0.2, 0.3, 1.2)};
  for (double t = 0.06; t < 0.34; t += 0.005) {
    const Eigen::Quaterniond q_truth = truth_orientation_at(truth, t);
    const Eigen::Vector3d p_truth = truth_position_at(truth, t);
    for (const auto & point_lidar : points_lidar) {
      const Eigen::Vector3d target_map = q_truth * point_lidar + p_truth;
      estimator.add_lidar_point_to_point_factor(
        t, point_lidar, target_map, extrinsics, 10.0, 0.0, 1.0);
    }
  }

  TrajectoryEstimatorOptions options;
  options.max_num_iterations = 80;
  options.function_tolerance = 1.0e-12;
  options.parameter_tolerance = 1.0e-12;
  options.hold_gyro_bias_constant = true;
  options.hold_accel_bias_constant = true;
  options.hold_gravity_constant = true;
  const auto summary = estimator.solve(options);
  if (estimator.lidar_point_factor_count() == 0) {
    std::fprintf(stderr, "LiDAR point-to-point factors were not added\n");
    std::exit(1);
  }
  if (summary.final_lidar_cost > 1.0e-8) {
    std::fprintf(stderr,
      "point-to-point residual did not converge: initial=%.6g final=%.6g (%s)\n",
      summary.initial_lidar_cost, summary.final_lidar_cost,
      summary.brief_report.c_str());
    std::exit(1);
  }
  const double drift = (estimator.position_knots()[3] - truth.position_knots[3]).norm();
  if (drift > 1.0e-4) {
    std::fprintf(stderr,
      "point-to-point factor failed to recover map alignment: drift=%.9f (%s)\n",
      drift, summary.brief_report.c_str());
    std::exit(1);
  }
}

void check_position_prior_factor_pulls_position_without_synthetic_lidar()
{
  // A global position offset is invisible to IMU acceleration residuals.
  // Position-only priors must therefore pull the spline back without relying
  // on synthetic LiDAR geometry.
  const double dt = 0.05;
  const auto truth = build_truth(dt, 8);
  TrajectoryEstimator estimator(dt);

  auto perturbed_positions = truth.position_knots;
  const Eigen::Vector3d offset(0.35, -0.20, 0.12);
  for (auto & p : perturbed_positions) {
    p += offset;
  }
  estimator.set_knots(truth.rotation_knots, perturbed_positions);

  for (double t = 0.06; t < 0.34; t += 0.005) {
    estimator.add_position_prior_factor(t, truth_position_at(truth, t), 10.0, 0.0);
  }

  TrajectoryEstimatorOptions options;
  options.max_num_iterations = 50;
  options.hold_gyro_bias_constant = true;
  options.hold_accel_bias_constant = true;
  options.hold_gravity_constant = true;
  const auto summary = estimator.solve(options);
  if (estimator.position_prior_factor_count() == 0) {
    std::fprintf(stderr, "position prior factors were not added\n");
    std::exit(1);
  }
  if (summary.final_cost > summary.initial_cost) {
    std::fprintf(stderr,
      "position prior solve increased cost: initial=%.6g final=%.6g\n",
      summary.initial_cost, summary.final_cost);
    std::exit(1);
  }
  const double drift = (estimator.position_knots()[3] - truth.position_knots[3]).norm();
  if (drift > 0.02) {
    std::fprintf(stderr,
      "position prior failed to pull central knot: drift=%.6f (%s)\n",
      drift, summary.brief_report.c_str());
    std::exit(1);
  }
}

void check_velocity_prior_factor_pulls_motion_scale_without_position_anchor()
{
  const double dt = 0.05;
  const auto truth = build_truth(dt, 8);
  TrajectoryEstimator estimator(dt);

  auto perturbed_positions = truth.position_knots;
  for (auto & p : perturbed_positions) {
    p.x() *= 0.5;
  }
  estimator.set_knots(truth.rotation_knots, perturbed_positions);

  for (double t = 0.06; t < 0.34; t += 0.005) {
    estimator.add_velocity_prior_factor(t, truth_velocity_at(truth, t), 10.0, 0.0);
  }

  TrajectoryEstimatorOptions options;
  options.max_num_iterations = 50;
  options.hold_gyro_bias_constant = true;
  options.hold_accel_bias_constant = true;
  options.hold_gravity_constant = true;
  const auto summary = estimator.solve(options);
  if (estimator.velocity_prior_factor_count() == 0) {
    std::fprintf(stderr, "velocity prior factors were not added\n");
    std::exit(1);
  }
  if (summary.final_velocity_prior_cost > summary.initial_velocity_prior_cost) {
    std::fprintf(stderr,
      "velocity prior solve increased cost: initial=%.6g final=%.6g\n",
      summary.initial_velocity_prior_cost, summary.final_velocity_prior_cost);
    std::exit(1);
  }
  const double velocity_error =
    (velocity_at(truth.rotation_knots, estimator.position_knots(), dt, 0.22) -
    truth_velocity_at(truth, 0.22)).norm();
  if (velocity_error > 0.02) {
    std::fprintf(stderr,
      "velocity prior failed to recover local motion scale: error=%.6f (%s)\n",
      velocity_error, summary.brief_report.c_str());
    std::exit(1);
  }
}

void check_orientation_prior_factor_pulls_rotation_without_imu()
{
  const double dt = 0.05;
  const auto truth = build_truth(dt, 8);
  TrajectoryEstimator estimator(dt);

  auto perturbed_rotations = truth.rotation_knots;
  const Eigen::Quaterniond offset(Eigen::AngleAxisd(0.20, Eigen::Vector3d::UnitY()));
  for (auto & q : perturbed_rotations) {
    q = (q * offset).normalized();
  }
  estimator.set_knots(perturbed_rotations, truth.position_knots);

  for (double t = 0.06; t < 0.34; t += 0.005) {
    estimator.add_orientation_prior_factor(t, truth_orientation_at(truth, t), 10.0, 0.0);
  }

  TrajectoryEstimatorOptions options;
  options.max_num_iterations = 50;
  options.hold_gyro_bias_constant = true;
  options.hold_accel_bias_constant = true;
  options.hold_gravity_constant = true;
  const auto summary = estimator.solve(options);
  if (estimator.orientation_prior_factor_count() == 0) {
    std::fprintf(stderr, "orientation prior factors were not added\n");
    std::exit(1);
  }
  if (summary.final_cost > summary.initial_cost) {
    std::fprintf(stderr,
      "orientation prior solve increased cost: initial=%.6g final=%.6g\n",
      summary.initial_cost, summary.final_cost);
    std::exit(1);
  }
  const double drift = estimator.rotation_knots()[3].angularDistance(truth.rotation_knots[3]);
  if (drift > 0.02) {
    std::fprintf(stderr,
      "orientation prior failed to pull central knot: drift=%.6f (%s)\n",
      drift, summary.brief_report.c_str());
    std::exit(1);
  }
}

void check_bias_prior_factors_regularize_imu_bias()
{
  const double dt = 0.05;
  const auto truth = build_truth(dt, 8);
  TrajectoryEstimator estimator(dt);
  estimator.set_knots(truth.rotation_knots, truth.position_knots);
  estimator.set_gyro_bias(Eigen::Vector3d(0.04, -0.02, 0.03));
  estimator.set_accel_bias(Eigen::Vector3d(0.2, -0.1, 0.05));

  if (!estimator.add_gyro_bias_prior_factor(Eigen::Vector3d::Zero(), 10.0, 0.0) ||
    !estimator.add_accel_bias_prior_factor(Eigen::Vector3d::Zero(), 10.0, 0.0))
  {
    std::fprintf(stderr, "bias prior factors were not added\n");
    std::exit(1);
  }

  TrajectoryEstimatorOptions options;
  options.max_num_iterations = 20;
  options.hold_gravity_constant = true;
  const auto summary = estimator.solve(options);
  if (estimator.gyro_bias_prior_factor_count() != 1 ||
    estimator.accel_bias_prior_factor_count() != 1)
  {
    std::fprintf(stderr, "bias prior counters are wrong\n");
    std::exit(1);
  }
  if (summary.final_bias_prior_cost > 1.0e-12 ||
    estimator.gyro_bias().norm() > 1.0e-8 ||
    estimator.accel_bias().norm() > 1.0e-8)
  {
    std::fprintf(stderr,
      "bias prior failed to regularize biases: final_cost=%.6g gyro=%.6g accel=%.6g (%s)\n",
      summary.final_bias_prior_cost,
      estimator.gyro_bias().norm(),
      estimator.accel_bias().norm(),
      summary.brief_report.c_str());
    std::exit(1);
  }
}

void check_rotation_smoothness_regularizes_orientation_shape()
{
  const double dt = 0.05;
  const auto truth = build_truth(dt, 8);
  TrajectoryEstimator estimator(dt);

  auto perturbed_rotations = truth.rotation_knots;
  perturbed_rotations[3] =
    (perturbed_rotations[3] *
    Eigen::Quaterniond(Eigen::AngleAxisd(0.18, Eigen::Vector3d::UnitX()))).normalized();
  estimator.set_knots(perturbed_rotations, truth.position_knots);

  for (std::size_t i = 0; i + 2 < perturbed_rotations.size(); ++i) {
    estimator.add_rotation_smoothness_factor(i, 10.0, 0.0);
  }

  TrajectoryEstimatorOptions options;
  options.max_num_iterations = 60;
  options.hold_gyro_bias_constant = true;
  options.hold_accel_bias_constant = true;
  options.hold_gravity_constant = true;
  const auto summary = estimator.solve(options);
  if (estimator.rotation_smoothness_factor_count() == 0) {
    std::fprintf(stderr, "rotation smoothness factors were not added\n");
    std::exit(1);
  }
  if (!(summary.initial_smoothness_cost > 1.0e-8 &&
    summary.final_smoothness_cost < 0.10 * summary.initial_smoothness_cost))
  {
    std::fprintf(stderr,
      "rotation smoothness did not reduce shape cost: initial=%.6g final=%.6g (%s)\n",
      summary.initial_smoothness_cost, summary.final_smoothness_cost,
      summary.brief_report.c_str());
    std::exit(1);
  }
}

void check_lidar_huber_loss_suppresses_plane_outlier()
{
  const double dt = 0.05;
  const auto truth = build_truth(dt, 8);
  LidarExtrinsics extrinsics;

  auto run_case = [&](double huber_delta_m) {
      TrajectoryEstimator estimator(dt);
      auto perturbed_positions = truth.position_knots;
      for (auto & p : perturbed_positions) {
        p.z() += 0.10;
      }
      estimator.set_knots(truth.rotation_knots, perturbed_positions);

      LidarPointCorrespondence inlier;
      inlier.geometry = LidarFeatureGeometry::kPlane;
      inlier.plane << 0.0, 0.0, 1.0, 0.0;
      LidarPointCorrespondence outlier = inlier;
      outlier.plane << 0.0, 0.0, 1.0, -1.0;
      for (double t = 0.08; t < 0.32; t += 0.01) {
        for (double offset_x = -0.4; offset_x <= 0.4; offset_x += 0.2) {
          inlier.point_lidar = Eigen::Vector3d(offset_x, 0.0, -truth.position_knots[3].z());
          outlier.point_lidar = inlier.point_lidar;
          estimator.add_lidar_factor(t, inlier, extrinsics, 1.0, huber_delta_m);
          estimator.add_lidar_factor(t, outlier, extrinsics, 1.0, huber_delta_m);
        }
      }

      TrajectoryEstimatorOptions options;
      options.max_num_iterations = 60;
      options.hold_gyro_bias_constant = true;
      options.hold_accel_bias_constant = true;
      options.hold_gravity_constant = true;
      estimator.solve(options);
      return estimator.position_knots()[3].z();
    };

  const double unrobust_z = run_case(0.0);
  const double robust_z = run_case(0.05);
  if (!(std::abs(robust_z) < std::abs(unrobust_z) &&
    std::abs(robust_z) < 0.20))
  {
    std::fprintf(stderr,
      "Huber loss failed to suppress LiDAR plane outlier: unrobust_z=%.6f robust_z=%.6f\n",
      unrobust_z, robust_z);
    std::exit(1);
  }
}

void check_fixed_control_point_prefix_stays_constant()
{
  const double dt = 0.05;
  const auto truth = build_truth(dt, 8);
  TrajectoryEstimator estimator(dt);
  estimator.set_knots(truth.rotation_knots, truth.position_knots);
  estimator.add_position_prior_factor(0.08, Eigen::Vector3d(1.0, -0.5, 0.25), 10.0, 0.0);

  TrajectoryEstimatorOptions options;
  options.max_num_iterations = 40;
  options.hold_gyro_bias_constant = true;
  options.hold_accel_bias_constant = true;
  options.hold_gravity_constant = true;
  options.fixed_control_point_index = 3;
  estimator.solve(options);

  const auto positions = estimator.position_knots();
  const auto rotations = estimator.rotation_knots();
  for (int i = 0; i <= options.fixed_control_point_index; ++i) {
    if ((positions[static_cast<std::size_t>(i)] - truth.position_knots[static_cast<std::size_t>(i)]).norm() >
      1.0e-12 ||
      rotations[static_cast<std::size_t>(i)].angularDistance(
        truth.rotation_knots[static_cast<std::size_t>(i)]) > 1.0e-12)
    {
      std::fprintf(stderr, "fixed control point %d moved during solve\n", i);
      std::exit(1);
    }
  }
}

void check_direct_knot_priors_anchor_control_point()
{
  const double dt = 0.05;
  const auto truth = build_truth(dt, 8);
  TrajectoryEstimator estimator(dt);

  auto perturbed_positions = truth.position_knots;
  auto perturbed_rotations = truth.rotation_knots;
  perturbed_positions[2] += Eigen::Vector3d(0.30, -0.20, 0.10);
  perturbed_rotations[2] =
    (perturbed_rotations[2] *
    Eigen::Quaterniond(Eigen::AngleAxisd(0.25, Eigen::Vector3d::UnitY()))).normalized();
  estimator.set_knots(perturbed_rotations, perturbed_positions);

  if (!estimator.add_knot_position_prior_factor(2, truth.position_knots[2], 25.0, 0.0) ||
    !estimator.add_knot_orientation_prior_factor(2, truth.rotation_knots[2], 25.0, 0.0))
  {
    std::fprintf(stderr, "direct knot prior factors were not added\n");
    std::exit(1);
  }

  TrajectoryEstimatorOptions options;
  options.max_num_iterations = 50;
  options.hold_gyro_bias_constant = true;
  options.hold_accel_bias_constant = true;
  options.hold_gravity_constant = true;
  const auto summary = estimator.solve(options);
  const auto positions = estimator.position_knots();
  const auto rotations = estimator.rotation_knots();
  if ((positions[2] - truth.position_knots[2]).norm() > 1.0e-8 ||
    rotations[2].angularDistance(truth.rotation_knots[2]) > 1.0e-8 ||
    summary.final_position_prior_cost > 1.0e-10 ||
    summary.final_orientation_prior_cost > 1.0e-10)
  {
    std::fprintf(stderr,
      "direct knot prior failed: p_err=%.9g r_err=%.9g pos_cost=%.9g rot_cost=%.9g (%s)\n",
      (positions[2] - truth.position_knots[2]).norm(),
      rotations[2].angularDistance(truth.rotation_knots[2]),
      summary.final_position_prior_cost,
      summary.final_orientation_prior_cost,
      summary.brief_report.c_str());
    std::exit(1);
  }
}

}  // namespace

int main()
{
  try {
    check_zero_residual_when_seeded_with_truth();
    check_converges_from_position_perturbation();
    check_position_prior_factor_pulls_position_without_synthetic_lidar();
    check_velocity_prior_factor_pulls_motion_scale_without_position_anchor();
    check_orientation_prior_factor_pulls_rotation_without_imu();
    check_bias_prior_factors_regularize_imu_bias();
    check_rotation_smoothness_regularizes_orientation_shape();
    check_lidar_plane_factor_pulls_position();
    check_lidar_point_to_point_factor_pulls_map_alignment();
    check_lidar_huber_loss_suppresses_plane_outlier();
    check_fixed_control_point_prefix_stays_constant();
    check_direct_knot_priors_anchor_control_point();
  } catch (const std::exception & exception) {
    std::fprintf(stderr, "trajectory_estimator_probe exception: %s\n", exception.what());
    return 1;
  }
  std::printf("trajectory_estimator_probe ok\n");
  return 0;
}
