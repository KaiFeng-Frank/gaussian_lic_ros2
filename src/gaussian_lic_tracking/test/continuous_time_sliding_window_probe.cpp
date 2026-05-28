// SPDX-License-Identifier: GPL-3.0-or-later
//
// Verifies `ContinuousTimeSlidingWindowEstimator` consumes streaming IMU /
// LiDAR samples and recovers a synthetic spline trajectory across many
// knot intervals — proving the sliding-window driver is functional beyond
// the eight-knot batch covered by `trajectory_estimator_probe`.

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gaussian_lic_tracking/spline/continuous_time_sliding_window.hpp>
#include <gaussian_lic_tracking/spline/split_spline_view.hpp>

using gaussian_lic_tracking::spline::ContinuousTimeSlidingWindowEstimator;
using gaussian_lic_tracking::spline::ContinuousTimeSlidingWindowOptions;
using gaussian_lic_tracking::spline::ImuSample;
using gaussian_lic_tracking::spline::LidarExtrinsics;
using gaussian_lic_tracking::spline::LidarFeatureGeometry;
using gaussian_lic_tracking::spline::LidarPointCorrespondence;
using gaussian_lic_tracking::spline::SplitSplineView;

namespace
{

constexpr int N = ContinuousTimeSlidingWindowEstimator::N;

void check_orientation_marginalization_is_default_off()
{
  const ContinuousTimeSlidingWindowOptions options;
  if (options.enable_spline_orientation_marginalization_prior) {
    std::fprintf(stderr, "SO(3) spline marginalization prior must be explicit opt-in\n");
    std::exit(1);
  }
}

struct TruthTrajectory
{
  std::vector<Eigen::Quaterniond> rotation_knots;
  std::vector<Eigen::Vector3d> position_knots;
  double dt_s{0.05};
  int64_t dt_ns{50000000};
};

TruthTrajectory build_truth(double dt_s, int knot_count)
{
  TruthTrajectory truth;
  truth.dt_s = dt_s;
  truth.dt_ns = static_cast<int64_t>(std::llround(dt_s * 1.0e9));
  truth.rotation_knots.reserve(knot_count);
  truth.position_knots.reserve(knot_count);
  for (int i = 0; i < knot_count; ++i) {
    const double t = i * dt_s;
    truth.rotation_knots.emplace_back(
      Eigen::AngleAxisd(0.3 * t, Eigen::Vector3d::UnitZ()));
    truth.position_knots.emplace_back(
      0.5 * t * t,
      0.2 * std::sin(t),
      0.1 * t);
  }
  return truth;
}

ImuSample synthesize_imu(
  const TruthTrajectory & truth,
  int64_t stamp_ns,
  const Eigen::Vector3d & gravity)
{
  const int idx = static_cast<int>(stamp_ns / truth.dt_ns);
  const double u =
    static_cast<double>(stamp_ns - idx * truth.dt_ns) / static_cast<double>(truth.dt_ns);
  std::array<Eigen::Quaterniond, N> rot;
  std::array<Eigen::Vector3d, N> pos;
  for (int i = 0; i < N; ++i) {
    rot[i] = truth.rotation_knots[idx - 1 + i];
    pos[i] = truth.position_knots[idx - 1 + i];
  }
  SplitSplineView<N> view(rot, pos, u, 1.0 / truth.dt_s);
  const auto state = view.evaluate(true);
  ImuSample sample;
  sample.gyro = state.omega_b;
  sample.accel = state.q_w_b.inverse() * (state.a_w_b - gravity);
  return sample;
}

void truth_pose_at(
  const TruthTrajectory & truth,
  int64_t stamp_ns,
  Eigen::Quaterniond & q_w_b,
  Eigen::Vector3d & p_w_b)
{
  const int idx = static_cast<int>(stamp_ns / truth.dt_ns);
  const double u =
    static_cast<double>(stamp_ns - idx * truth.dt_ns) / static_cast<double>(truth.dt_ns);
  std::array<Eigen::Quaterniond, N> rot;
  std::array<Eigen::Vector3d, N> pos;
  for (int i = 0; i < N; ++i) {
    rot[i] = truth.rotation_knots[idx - 1 + i];
    pos[i] = truth.position_knots[idx - 1 + i];
  }
  SplitSplineView<N> view(rot, pos, u, 1.0 / truth.dt_s);
  q_w_b = view.rotation();
  p_w_b = view.position_world();
}

double max_window_position_drift(
  const ContinuousTimeSlidingWindowEstimator & estimator,
  const TruthTrajectory & truth)
{
  double worst = 0.0;
  const int64_t newest = estimator.newest_knot_stamp_ns();
  const int64_t step = static_cast<int64_t>(std::llround(truth.dt_s * 0.5 * 1.0e9));
  for (int64_t t = estimator.oldest_active_knot_stamp_ns() + truth.dt_ns;
    t < newest - truth.dt_ns; t += step)
  {
    Eigen::Quaterniond q;
    Eigen::Vector3d p;
    if (!estimator.query_pose(t, q, p)) {
      continue;
    }
    const int idx = static_cast<int>(t / truth.dt_ns);
    const double u =
      static_cast<double>(t - idx * truth.dt_ns) / static_cast<double>(truth.dt_ns);
    std::array<Eigen::Quaterniond, N> rot;
    std::array<Eigen::Vector3d, N> pos;
    for (int i = 0; i < N; ++i) {
      rot[i] = truth.rotation_knots[idx - 1 + i];
      pos[i] = truth.position_knots[idx - 1 + i];
    }
    SplitSplineView<N> view(rot, pos, u, 1.0 / truth.dt_s);
    const Eigen::Vector3d truth_p = view.position_world();
    worst = std::max(worst, (p - truth_p).cwiseAbs().maxCoeff());
  }
  return worst;
}

void check_single_step_solves_within_seeded_window()
{
  // No window extension, no marginalization: initialize with the full
  // truth-aligned knot set, perturb one interior knot, run a single step
  // with all IMU samples that fall inside the window, and assert the active
  // window recovers truth.
  const double dt_s = 0.05;
  const Eigen::Vector3d gravity(0.0, 0.0, -9.81);
  const auto truth = build_truth(dt_s, 12);

  ContinuousTimeSlidingWindowOptions options;
  options.dt_s = dt_s;
  options.window_knot_count = 12;
  options.marginalize_oldest_count = 0;
  options.gravity_world = gravity;
  options.hold_gravity_constant = true;
  options.hold_accel_bias_constant = true;
  options.hold_gyro_bias_constant = true;
  options.max_iterations_per_step = 80;
  options.max_position_update_m = 0.0;
  options.max_rotation_update_rad = 0.0;

  ContinuousTimeSlidingWindowEstimator estimator(options);
  auto initial_rot = truth.rotation_knots;
  auto initial_pos = truth.position_knots;
  initial_pos[5].z() += 0.03;
  estimator.initialize(0, initial_rot, initial_pos);

  const int64_t imu_period_ns = static_cast<int64_t>(std::llround(dt_s * 1.0e9 / 10.0));
  const int64_t last_interior_ns =
    static_cast<int64_t>(truth.rotation_knots.size() - 3) * truth.dt_ns;
  for (int64_t t = truth.dt_ns; t < last_interior_ns; t += imu_period_ns) {
    estimator.add_imu_sample(t, synthesize_imu(truth, t, gravity));
  }
  if (!estimator.step()) {
    std::fprintf(stderr, "single-step solve refused to run\n");
    std::exit(1);
  }
  const auto & diag = estimator.diagnostics();
  if (diag.last_step_final_cost > 1.0e-6) {
    std::fprintf(stderr,
      "single-step final cost too large: initial=%.6g final=%.6g\n",
      diag.last_step_initial_cost, diag.last_step_final_cost);
    std::exit(1);
  }
  const double drift = max_window_position_drift(estimator, truth);
  if (drift > 0.02) {
    std::fprintf(stderr,
      "single-step window drift too large: %.4f m\n", drift);
    std::exit(1);
  }
}

void check_solver_step_rejection_keeps_window_finite()
{
  const double dt_s = 0.05;
  const Eigen::Vector3d gravity(0.0, 0.0, -9.81);
  const auto truth = build_truth(dt_s, 12);

  ContinuousTimeSlidingWindowOptions options;
  options.dt_s = dt_s;
  options.window_knot_count = 12;
  options.marginalize_oldest_count = 0;
  options.gravity_world = gravity;
  options.hold_gravity_constant = true;
  options.hold_accel_bias_constant = true;
  options.hold_gyro_bias_constant = true;
  options.max_iterations_per_step = 80;
  options.max_position_update_m = 1.0e-6;
  options.max_rotation_update_rad = 1.0e-6;

  ContinuousTimeSlidingWindowEstimator estimator(options);
  auto initial_rot = truth.rotation_knots;
  auto initial_pos = truth.position_knots;
  initial_pos[5].z() += 0.03;
  estimator.initialize(0, initial_rot, initial_pos);

  const int64_t imu_period_ns = static_cast<int64_t>(std::llround(dt_s * 1.0e9 / 10.0));
  const int64_t last_interior_ns =
    static_cast<int64_t>(truth.rotation_knots.size() - 3) * truth.dt_ns;
  for (int64_t t = truth.dt_ns; t < last_interior_ns; t += imu_period_ns) {
    estimator.add_imu_sample(t, synthesize_imu(truth, t, gravity));
  }
  if (!estimator.step()) {
    std::fprintf(stderr, "solver rejection path should still advance the window\n");
    std::exit(1);
  }
  const auto & diag = estimator.diagnostics();
  if (diag.rejected_solver_steps != 1) {
    std::fprintf(stderr,
      "expected one rejected solver step, got %zu\n",
      diag.rejected_solver_steps);
    std::exit(1);
  }
  if (!(diag.last_rejected_position_update_m > options.max_position_update_m)) {
    std::fprintf(stderr,
      "rejected update magnitude was not recorded: %.9f\n",
      diag.last_rejected_position_update_m);
    std::exit(1);
  }
  Eigen::Quaterniond q;
  Eigen::Vector3d p;
  if (!estimator.query_pose(5 * truth.dt_ns, q, p) ||
    !p.allFinite() || !q.coeffs().allFinite())
  {
    std::fprintf(stderr, "rejected solve left a non-finite query pose\n");
    std::exit(1);
  }
}

void check_limited_position_update_clamps_without_rejecting()
{
  const double dt_s = 0.05;
  const Eigen::Vector3d gravity(0.0, 0.0, -9.81);
  const auto truth = build_truth(dt_s, 12);

  ContinuousTimeSlidingWindowOptions options;
  options.dt_s = dt_s;
  options.window_knot_count = 12;
  options.marginalize_oldest_count = 0;
  options.gravity_world = gravity;
  options.hold_gravity_constant = true;
  options.hold_accel_bias_constant = true;
  options.hold_gyro_bias_constant = true;
  options.max_iterations_per_step = 80;
  options.max_position_update_m = 0.01;
  options.max_rotation_update_rad = 0.0;
  options.apply_limited_position_update = true;

  ContinuousTimeSlidingWindowEstimator estimator(options);
  auto initial_rot = truth.rotation_knots;
  auto initial_pos = truth.position_knots;
  initial_pos[5].z() += 0.03;
  estimator.initialize(0, initial_rot, initial_pos);

  Eigen::Quaterniond q_before;
  Eigen::Vector3d p_before;
  if (!estimator.query_pose(5 * truth.dt_ns, q_before, p_before)) {
    std::fprintf(stderr, "limited-position test could not query initial pose\n");
    std::exit(1);
  }

  const int64_t imu_period_ns = static_cast<int64_t>(std::llround(dt_s * 1.0e9 / 10.0));
  const int64_t last_interior_ns =
    static_cast<int64_t>(truth.rotation_knots.size() - 3) * truth.dt_ns;
  for (int64_t t = truth.dt_ns; t < last_interior_ns; t += imu_period_ns) {
    estimator.add_imu_sample(t, synthesize_imu(truth, t, gravity));
  }
  if (!estimator.step()) {
    std::fprintf(stderr, "limited-position solve refused to run\n");
    std::exit(1);
  }

  const auto & diag = estimator.diagnostics();
  if (diag.rejected_solver_steps != 0 || diag.position_update_rejections != 0) {
    std::fprintf(stderr,
      "limited-position update was rejected: rejected=%zu position_rejections=%zu\n",
      diag.rejected_solver_steps, diag.position_update_rejections);
    std::exit(1);
  }
  if (diag.position_limited_solver_steps != 1 || !diag.last_step_position_limited) {
    std::fprintf(stderr,
      "limited-position counters not set: steps=%zu last=%d\n",
      diag.position_limited_solver_steps,
      static_cast<int>(diag.last_step_position_limited));
    std::exit(1);
  }
  if (diag.accepted_solver_steps != 1 || !diag.last_step_update_accepted) {
    std::fprintf(stderr,
      "limited-position update was not marked applied: accepted=%zu last=%d\n",
      diag.accepted_solver_steps,
      static_cast<int>(diag.last_step_update_accepted));
    std::exit(1);
  }
  if (!(diag.last_position_limited_position_update_m > options.max_position_update_m)) {
    std::fprintf(stderr,
      "limited-position magnitude was not recorded: %.9f\n",
      diag.last_position_limited_position_update_m);
    std::exit(1);
  }

  Eigen::Quaterniond q_after;
  Eigen::Vector3d p_after;
  if (!estimator.query_pose(5 * truth.dt_ns, q_after, p_after)) {
    std::fprintf(stderr, "limited-position test could not query final pose\n");
    std::exit(1);
  }
  const double applied_query_step = (p_after - p_before).norm();
  if (applied_query_step <= 1.0e-6 || applied_query_step > 0.012) {
    std::fprintf(stderr,
      "limited-position query step outside trust region: %.9f\n",
      applied_query_step);
    std::exit(1);
  }
}

void check_limited_update_carries_auxiliary_state()
{
  const double dt_s = 0.05;
  const Eigen::Vector3d gravity(0.0, 0.0, -9.81);
  const auto truth = build_truth(dt_s, 12);

  ContinuousTimeSlidingWindowOptions options;
  options.dt_s = dt_s;
  options.window_knot_count = 12;
  options.marginalize_oldest_count = 0;
  options.gravity_world = gravity;
  options.initial_gyro_bias = Eigen::Vector3d(0.05, -0.03, 0.02);
  options.initial_accel_bias = Eigen::Vector3d(0.20, -0.10, 0.05);
  options.hold_gravity_constant = true;
  options.hold_accel_bias_constant = false;
  options.hold_gyro_bias_constant = false;
  options.max_iterations_per_step = 120;
  options.max_position_update_m = 0.01;
  options.max_rotation_update_rad = 0.0;
  options.apply_limited_position_update = true;

  ContinuousTimeSlidingWindowEstimator estimator(options);
  auto initial_rot = truth.rotation_knots;
  auto initial_pos = truth.position_knots;
  initial_pos[5].z() += 0.03;
  estimator.initialize(0, initial_rot, initial_pos);

  const Eigen::Vector3d gyro_before = estimator.gyro_bias();
  const Eigen::Vector3d accel_before = estimator.accel_bias();
  const int64_t imu_period_ns = static_cast<int64_t>(std::llround(dt_s * 1.0e9 / 10.0));
  const int64_t last_interior_ns =
    static_cast<int64_t>(truth.rotation_knots.size() - 3) * truth.dt_ns;
  for (int64_t t = truth.dt_ns; t < last_interior_ns; t += imu_period_ns) {
    estimator.add_imu_sample(t, synthesize_imu(truth, t, gravity));
  }
  if (!estimator.step()) {
    std::fprintf(stderr, "limited auxiliary-state solve refused to run\n");
    std::exit(1);
  }

  const auto & diag = estimator.diagnostics();
  if (diag.position_limited_solver_steps != 1 || diag.accepted_solver_steps != 1) {
    std::fprintf(stderr,
      "limited auxiliary-state update did not take limited branch: limited=%zu accepted=%zu\n",
      diag.position_limited_solver_steps,
      diag.accepted_solver_steps);
    std::exit(1);
  }
  const double gyro_change = (estimator.gyro_bias() - gyro_before).norm();
  const double accel_change = (estimator.accel_bias() - accel_before).norm();
  if (gyro_change <= 1.0e-6 && accel_change <= 1.0e-6) {
    std::fprintf(stderr,
      "limited auxiliary-state update left IMU biases frozen: gyro=%.9g accel=%.9g\n",
      gyro_change,
      accel_change);
    std::exit(1);
  }
}

void check_point_to_point_map_factor_pulls_streaming_window()
{
  const double dt_s = 0.05;
  const Eigen::Vector3d gravity(0.0, 0.0, -9.81);
  const auto truth = build_truth(dt_s, 12);

  ContinuousTimeSlidingWindowOptions options;
  options.dt_s = dt_s;
  options.window_knot_count = 12;
  options.marginalize_oldest_count = 0;
  options.gravity_world = gravity;
  options.hold_gravity_constant = true;
  options.hold_accel_bias_constant = true;
  options.hold_gyro_bias_constant = true;
  options.max_iterations_per_step = 80;
  options.max_position_update_m = 0.0;
  options.max_rotation_update_rad = 0.0;

  ContinuousTimeSlidingWindowEstimator estimator(options);
  auto initial_rot = truth.rotation_knots;
  auto initial_pos = truth.position_knots;
  const Eigen::Vector3d offset(0.16, -0.10, 0.06);
  for (auto & p : initial_pos) {
    p += offset;
  }
  estimator.initialize(0, initial_rot, initial_pos);

  LidarExtrinsics extrinsics;
  const std::vector<Eigen::Vector3d> points_lidar{
    Eigen::Vector3d(0.2, -0.2, 1.0),
    Eigen::Vector3d(-0.3, 0.1, 0.7),
    Eigen::Vector3d(0.1, 0.3, 1.1)};
  const int64_t sample_period_ns = static_cast<int64_t>(std::llround(dt_s * 1.0e9 / 5.0));
  const int64_t last_interior_ns =
    static_cast<int64_t>(truth.rotation_knots.size() - 3) * truth.dt_ns;
  for (int64_t t = truth.dt_ns; t < last_interior_ns; t += sample_period_ns) {
    Eigen::Quaterniond q_truth;
    Eigen::Vector3d p_truth;
    truth_pose_at(truth, t, q_truth, p_truth);
    for (const auto & point_lidar : points_lidar) {
      estimator.add_lidar_point_to_point_correspondence(
        t, point_lidar, q_truth * point_lidar + p_truth, extrinsics, 10.0, 0.0, 1.0);
    }
  }

  if (!estimator.step()) {
    std::fprintf(stderr, "point-to-point map solve refused to run\n");
    std::exit(1);
  }
  const auto & diag = estimator.diagnostics();
  if (diag.total_lidar_point_factors == 0 || diag.last_step_lidar_point_factors == 0) {
    std::fprintf(stderr,
      "point-to-point map factors were not consumed: total=%zu last=%zu\n",
      diag.total_lidar_point_factors,
      diag.last_step_lidar_point_factors);
    std::exit(1);
  }
  const double drift = max_window_position_drift(estimator, truth);
  if (drift > 0.015) {
    std::fprintf(stderr,
      "point-to-point map factors failed to pull streaming window: drift=%.6f\n",
      drift);
    std::exit(1);
  }
}

void check_relative_pose_priors_pull_streaming_window_motion()
{
  const double dt_s = 0.05;
  const Eigen::Vector3d gravity(0.0, 0.0, -9.81);
  const auto truth = build_truth(dt_s, 12);

  ContinuousTimeSlidingWindowOptions options;
  options.dt_s = dt_s;
  options.window_knot_count = 12;
  options.marginalize_oldest_count = 0;
  options.gravity_world = gravity;
  options.hold_gravity_constant = true;
  options.hold_accel_bias_constant = true;
  options.hold_gyro_bias_constant = true;
  options.max_iterations_per_step = 80;
  options.max_position_update_m = 0.0;
  options.max_rotation_update_rad = 0.0;

  ContinuousTimeSlidingWindowEstimator estimator(options);
  auto initial_rot = truth.rotation_knots;
  auto initial_pos = truth.position_knots;
  initial_pos[4] += Eigen::Vector3d(0.12, -0.03, 0.02);
  initial_rot[4] =
    (Eigen::Quaterniond(Eigen::AngleAxisd(0.10, Eigen::Vector3d::UnitZ())) *
    initial_rot[4]).normalized();
  estimator.initialize(0, initial_rot, initial_pos);

  const int64_t t0 = 3 * truth.dt_ns;
  const int64_t t1 = 5 * truth.dt_ns;
  Eigen::Quaterniond q0_truth;
  Eigen::Quaterniond q1_truth;
  Eigen::Vector3d p0_truth;
  Eigen::Vector3d p1_truth;
  truth_pose_at(truth, t0, q0_truth, p0_truth);
  truth_pose_at(truth, t1, q1_truth, p1_truth);
  const Eigen::Vector3d target_p_body0 =
    q0_truth.inverse() * (p1_truth - p0_truth);
  const Eigen::Quaterniond target_q_body0_body1 =
    (q0_truth.inverse() * q1_truth).normalized();

  estimator.add_relative_position_prior(t0, t1, target_p_body0, 20.0, 0.0);
  estimator.add_relative_orientation_prior(t0, t1, target_q_body0_body1, 20.0, 0.0);
  if (!estimator.step()) {
    std::fprintf(stderr, "relative pose prior solve refused to run\n");
    std::exit(1);
  }
  const auto & diag = estimator.diagnostics();
  if (diag.last_step_position_prior_factors == 0 || diag.last_step_orientation_prior_factors == 0) {
    std::fprintf(stderr,
      "relative pose priors were not consumed: pos=%zu rot=%zu\n",
      diag.last_step_position_prior_factors,
      diag.last_step_orientation_prior_factors);
    std::exit(1);
  }
  Eigen::Quaterniond q0;
  Eigen::Quaterniond q1;
  Eigen::Vector3d p0;
  Eigen::Vector3d p1;
  if (!estimator.query_pose(t0, q0, p0) || !estimator.query_pose(t1, q1, p1)) {
    std::fprintf(stderr, "relative pose prior result was not queryable\n");
    std::exit(1);
  }
  const Eigen::Vector3d relative_p = q0.inverse() * (p1 - p0);
  const Eigen::Quaterniond relative_q = (q0.inverse() * q1).normalized();
  const double position_error = (relative_p - target_p_body0).norm();
  const double rotation_error = relative_q.angularDistance(target_q_body0_body1);
  if (position_error > 1.0e-8 || rotation_error > 1.0e-8) {
    std::fprintf(stderr,
      "relative pose priors failed: p_err=%.9g r_err=%.9g\n",
      position_error,
      rotation_error);
    std::exit(1);
  }
}

void check_sliding_window_recovers_streamed_trajectory()
{
  // Streaming mode: the seed only spans the first window; the estimator
  // grows the window forward as IMU samples arrive. Drift bound is looser
  // because newly-extrapolated knots take time to settle.
  const double dt_s = 0.05;
  const Eigen::Vector3d gravity(0.0, 0.0, -9.81);
  const auto truth = build_truth(dt_s, 24);

  ContinuousTimeSlidingWindowOptions options;
  options.dt_s = dt_s;
  options.window_knot_count = 8;
  options.marginalize_oldest_count = 0;
  options.gravity_world = gravity;
  options.hold_gravity_constant = true;
  options.hold_accel_bias_constant = true;
  options.hold_gyro_bias_constant = true;
  options.max_iterations_per_step = 60;
  options.max_position_update_m = 0.0;
  options.max_rotation_update_rad = 0.0;

  ContinuousTimeSlidingWindowEstimator estimator(options);
  std::vector<Eigen::Quaterniond> initial_rot(
    truth.rotation_knots.begin(),
    truth.rotation_knots.begin() + 8);
  std::vector<Eigen::Vector3d> initial_pos(
    truth.position_knots.begin(),
    truth.position_knots.begin() + 8);
  estimator.initialize(0, initial_rot, initial_pos);

  const int64_t imu_period_ns = static_cast<int64_t>(std::llround(dt_s * 1.0e9 / 10.0));
  const int64_t end_stamp_ns =
    static_cast<int64_t>(truth.rotation_knots.size() - 3) * truth.dt_ns;
  // Step every IMU sample to give the optimizer maximum opportunities to
  // settle freshly extrapolated knots.
  for (int64_t t = truth.dt_ns; t < end_stamp_ns; t += imu_period_ns) {
    estimator.add_imu_sample(t, synthesize_imu(truth, t, gravity));
    estimator.step();
  }
  // Final flush.
  estimator.step();

  const auto & diag = estimator.diagnostics();
  if (diag.steps_run < 4) {
    std::fprintf(stderr,
      "expected >=4 solver steps, got %zu\n", diag.steps_run);
    std::exit(1);
  }
  if (diag.total_imu_factors < 100) {
    std::fprintf(stderr,
      "expected >=100 cumulative IMU factors, got %zu\n",
      diag.total_imu_factors);
    std::exit(1);
  }
  const double drift = max_window_position_drift(estimator, truth);
  if (drift > 0.08) {
    std::fprintf(stderr,
      "sliding window streaming drift too large: %.4f m (steps=%zu imu_factors=%zu)\n",
      drift, diag.steps_run, diag.total_imu_factors);
    std::exit(1);
  }
}

void check_marginalization_keeps_window_bounded()
{
  const double dt_s = 0.05;
  const Eigen::Vector3d gravity(0.0, 0.0, -9.81);
  const auto truth = build_truth(dt_s, 30);

  ContinuousTimeSlidingWindowOptions options;
  options.dt_s = dt_s;
  options.window_knot_count = 8;
  options.marginalize_oldest_count = 1;
  options.gravity_world = gravity;
  options.hold_gravity_constant = true;
  options.hold_accel_bias_constant = true;
  options.hold_gyro_bias_constant = true;
  options.max_iterations_per_step = 30;
  options.max_position_update_m = 0.0;
  options.max_rotation_update_rad = 0.0;
  options.retained_knot_prior_count = 4;
  options.retained_knot_position_prior_weight = 5.0;
  options.retained_knot_orientation_prior_weight = 5.0;
  options.rotation_smoothness_weight = 2.0;
  options.enable_spline_orientation_marginalization_prior = true;

  ContinuousTimeSlidingWindowEstimator estimator(options);
  std::vector<Eigen::Quaterniond> initial_rot(
    truth.rotation_knots.begin(),
    truth.rotation_knots.begin() + 8);
  std::vector<Eigen::Vector3d> initial_pos(
    truth.position_knots.begin(),
    truth.position_knots.begin() + 8);
  estimator.initialize(0, initial_rot, initial_pos);

  const int64_t imu_period_ns = static_cast<int64_t>(std::llround(dt_s * 1.0e9 / 10.0));
  const int64_t end_stamp_ns =
    static_cast<int64_t>(truth.rotation_knots.size() - 3) * truth.dt_ns;
  int64_t next_step_stamp = truth.dt_ns * 4;

  std::size_t max_seen_window = 0;
  for (int64_t t = truth.dt_ns; t < end_stamp_ns; t += imu_period_ns) {
    estimator.add_imu_sample(t, synthesize_imu(truth, t, gravity));
    if (t >= next_step_stamp) {
      Eigen::Quaterniond q;
      Eigen::Vector3d p;
      truth_pose_at(truth, t, q, p);
      estimator.add_position_prior(t, p, 5.0, 0.0);
      estimator.step();
      max_seen_window = std::max(max_seen_window, estimator.active_knot_count());
      next_step_stamp += truth.dt_ns * 4;
    }
  }
  estimator.step();

  if (estimator.active_knot_count() > 12u) {
    std::fprintf(stderr,
      "marginalization failed to bound window: %zu knots active\n",
      estimator.active_knot_count());
    std::exit(1);
  }
  if (estimator.diagnostics().total_marginalized_knots == 0) {
    std::fprintf(stderr, "no knots ever marginalized despite enabled policy\n");
    std::exit(1);
  }
  if (estimator.diagnostics().total_retained_knot_position_prior_factors == 0 ||
    estimator.diagnostics().total_retained_knot_orientation_prior_factors == 0)
  {
    std::fprintf(stderr,
      "retained knot priors were not injected: pos=%zu rot=%zu\n",
      estimator.diagnostics().total_retained_knot_position_prior_factors,
      estimator.diagnostics().total_retained_knot_orientation_prior_factors);
    std::exit(1);
  }
  if (estimator.diagnostics().total_spline_marginalization_priors == 0 ||
    estimator.diagnostics().total_spline_marginalization_prior_rows == 0)
  {
    std::fprintf(stderr,
      "dense spline marginalization priors were not generated: priors=%zu rows=%zu\n",
      estimator.diagnostics().total_spline_marginalization_priors,
      estimator.diagnostics().total_spline_marginalization_prior_rows);
    std::exit(1);
  }
  if (
    estimator.diagnostics().total_spline_orientation_marginalization_priors == 0 ||
    estimator.diagnostics().total_spline_orientation_marginalization_prior_rows == 0)
  {
    std::fprintf(stderr,
      "dense spline orientation marginalization priors were not generated: priors=%zu rows=%zu\n",
      estimator.diagnostics().total_spline_orientation_marginalization_priors,
      estimator.diagnostics().total_spline_orientation_marginalization_prior_rows);
    std::exit(1);
  }
}

}  // namespace

int main()
{
  try {
    check_orientation_marginalization_is_default_off();
    check_single_step_solves_within_seeded_window();
    check_solver_step_rejection_keeps_window_finite();
    check_limited_position_update_clamps_without_rejecting();
    check_limited_update_carries_auxiliary_state();
    check_point_to_point_map_factor_pulls_streaming_window();
    check_relative_pose_priors_pull_streaming_window_motion();
    check_sliding_window_recovers_streamed_trajectory();
    check_marginalization_keeps_window_bounded();
  } catch (const std::exception & exception) {
    std::fprintf(stderr,
      "continuous_time_sliding_window_probe exception: %s\n", exception.what());
    return 1;
  }
  std::printf("continuous_time_sliding_window_probe ok\n");
  return 0;
}
