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
}

}  // namespace

int main()
{
  try {
    check_single_step_solves_within_seeded_window();
    check_solver_step_rejection_keeps_window_finite();
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
