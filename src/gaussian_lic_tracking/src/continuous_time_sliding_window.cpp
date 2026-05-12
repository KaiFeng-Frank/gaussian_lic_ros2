// SPDX-License-Identifier: GPL-3.0-or-later

#include <gaussian_lic_tracking/spline/continuous_time_sliding_window.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace gaussian_lic_tracking
{
namespace spline
{

namespace
{

struct BufferedImu
{
  int64_t stamp_ns{0};
  ImuSample sample;
};

struct BufferedLidar
{
  int64_t stamp_ns{0};
  LidarPointCorrespondence correspondence;
  LidarExtrinsics extrinsics;
  double weight{1.0};
};

bool quaternion_is_valid(const Eigen::Quaterniond & q)
{
  return q.coeffs().allFinite() && std::isfinite(q.norm()) && q.norm() > 1.0e-9;
}

}  // namespace

struct ContinuousTimeSlidingWindowEstimator::Impl
{
  ContinuousTimeSlidingWindowOptions options;
  int64_t dt_ns{50000000};

  std::deque<int64_t> knot_stamps;
  std::deque<Eigen::Quaterniond> rotation_knots;
  std::deque<Eigen::Vector3d> position_knots;

  Eigen::Vector3d gyro_bias{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accel_bias{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gravity_world{Eigen::Vector3d::Zero()};

  // Active factors persist across steps — they are removed only when the
  // window has marginalized past their stamp.
  std::deque<BufferedImu> active_imu;
  std::deque<BufferedLidar> active_lidar;

  // Brand-new samples waiting for their first solve.
  std::deque<BufferedImu> pending_imu;
  std::deque<BufferedLidar> pending_lidar;

  ContinuousTimeSlidingWindowDiagnostics diagnostics;
};

ContinuousTimeSlidingWindowEstimator::ContinuousTimeSlidingWindowEstimator(
  const ContinuousTimeSlidingWindowOptions & options)
: impl_(std::make_unique<Impl>())
{
  if (!(options.dt_s > 0.0)) {
    throw std::runtime_error("sliding window dt_s must be positive");
  }
  if (options.window_knot_count < N + 2) {
    throw std::runtime_error("window must hold at least N+2 knots to expose interior samples");
  }
  if (options.marginalize_oldest_count < 0 ||
    options.marginalize_oldest_count >= options.window_knot_count - N)
  {
    throw std::runtime_error("marginalize_oldest_count must leave at least N kept knots");
  }
  impl_->options = options;
  impl_->dt_ns = static_cast<int64_t>(std::llround(options.dt_s * 1.0e9));
  impl_->gyro_bias = options.initial_gyro_bias;
  impl_->accel_bias = options.initial_accel_bias;
  impl_->gravity_world = options.gravity_world;
}

ContinuousTimeSlidingWindowEstimator::~ContinuousTimeSlidingWindowEstimator() = default;

void ContinuousTimeSlidingWindowEstimator::initialize(
  int64_t start_stamp_ns,
  const std::vector<Eigen::Quaterniond> & rotation_knots,
  const std::vector<Eigen::Vector3d> & position_knots)
{
  if (rotation_knots.size() != position_knots.size()) {
    throw std::runtime_error("initialize: rotation and position knot count mismatch");
  }
  if (rotation_knots.size() < static_cast<std::size_t>(N)) {
    throw std::runtime_error("initialize: need at least N=4 control knots");
  }
  impl_->knot_stamps.clear();
  impl_->rotation_knots.clear();
  impl_->position_knots.clear();
  for (std::size_t i = 0; i < rotation_knots.size(); ++i) {
    impl_->knot_stamps.push_back(start_stamp_ns + static_cast<int64_t>(i) * impl_->dt_ns);
    impl_->rotation_knots.push_back(rotation_knots[i].normalized());
    impl_->position_knots.push_back(position_knots[i]);
  }
}

void ContinuousTimeSlidingWindowEstimator::add_imu_sample(
  int64_t stamp_ns,
  const ImuSample & sample)
{
  impl_->pending_imu.push_back({stamp_ns, sample});
}

void ContinuousTimeSlidingWindowEstimator::add_lidar_correspondence(
  int64_t stamp_ns,
  const LidarPointCorrespondence & correspondence,
  const LidarExtrinsics & extrinsics,
  double weight)
{
  impl_->pending_lidar.push_back({stamp_ns, correspondence, extrinsics, weight});
}

bool ContinuousTimeSlidingWindowEstimator::step()
{
  if (impl_->knot_stamps.size() < static_cast<std::size_t>(N)) {
    return false;
  }

  // Promote any pending samples that fit inside the current optimizable
  // interior. Samples beyond the current interior remain pending so the
  // window can grow toward them.
  auto interior_end_ns = [this]() {
    return impl_->knot_stamps[impl_->knot_stamps.size() - 3];
  };

  // Extend window forward while there are pending samples past the current
  // interior end.
  while (!impl_->pending_imu.empty() &&
    impl_->pending_imu.back().stamp_ns >= interior_end_ns())
  {
    const int64_t next_stamp = impl_->knot_stamps.back() + impl_->dt_ns;
    const std::size_t n = impl_->rotation_knots.size();
    const Eigen::Quaterniond q_new =
      (impl_->rotation_knots[n - 1] *
      (impl_->rotation_knots[n - 2].inverse() * impl_->rotation_knots[n - 1])).normalized();
    const Eigen::Vector3d p_new =
      impl_->position_knots[n - 1] +
      (impl_->position_knots[n - 1] - impl_->position_knots[n - 2]);
    impl_->knot_stamps.push_back(next_stamp);
    impl_->rotation_knots.push_back(q_new);
    impl_->position_knots.push_back(p_new);
    if (next_stamp > impl_->pending_imu.back().stamp_ns + impl_->dt_ns) {
      break;
    }
  }

  // Move pending samples whose stamps now fall inside the window into the
  // active list (which persists across solves).
  std::deque<BufferedImu> imu_still_pending;
  for (const auto & p : impl_->pending_imu) {
    if (p.stamp_ns < interior_end_ns()) {
      impl_->active_imu.push_back(p);
    } else {
      imu_still_pending.push_back(p);
    }
  }
  impl_->pending_imu = imu_still_pending;

  std::deque<BufferedLidar> lidar_still_pending;
  for (const auto & p : impl_->pending_lidar) {
    if (p.stamp_ns < interior_end_ns()) {
      impl_->active_lidar.push_back(p);
    } else {
      lidar_still_pending.push_back(p);
    }
  }
  impl_->pending_lidar = lidar_still_pending;

  // Marginalize oldest knots once the window exceeds the configured size.
  while (
    static_cast<int>(impl_->knot_stamps.size()) >
    impl_->options.window_knot_count &&
    impl_->options.marginalize_oldest_count > 0)
  {
    impl_->knot_stamps.pop_front();
    impl_->rotation_knots.pop_front();
    impl_->position_knots.pop_front();
    ++impl_->diagnostics.total_marginalized_knots;
  }

  const int64_t window_start = impl_->knot_stamps.front();
  // Drop active factors whose stamps now fall before the optimizable
  // interior (they have been marginalized out of the window).
  const int64_t interior_start_ns = impl_->knot_stamps[1];
  while (!impl_->active_imu.empty() && impl_->active_imu.front().stamp_ns < interior_start_ns) {
    impl_->active_imu.pop_front();
  }
  while (!impl_->active_lidar.empty() && impl_->active_lidar.front().stamp_ns < interior_start_ns) {
    impl_->active_lidar.pop_front();
  }

  if (impl_->active_imu.empty() && impl_->active_lidar.empty()) {
    return false;
  }

  const double estimator_dt_s = impl_->options.dt_s;
  TrajectoryEstimator estimator(estimator_dt_s);
  estimator.set_knots(
    std::vector<Eigen::Quaterniond>(impl_->rotation_knots.begin(), impl_->rotation_knots.end()),
    std::vector<Eigen::Vector3d>(impl_->position_knots.begin(), impl_->position_knots.end()));
  estimator.set_gyro_bias(impl_->gyro_bias);
  estimator.set_accel_bias(impl_->accel_bias);
  estimator.set_gravity_world(impl_->gravity_world);

  Eigen::Matrix<double, 6, 1> info_diag;
  info_diag <<
    impl_->options.imu_info_gyro, impl_->options.imu_info_gyro, impl_->options.imu_info_gyro,
    impl_->options.imu_info_accel, impl_->options.imu_info_accel, impl_->options.imu_info_accel;
  for (const auto & active : impl_->active_imu) {
    const double t_s = (active.stamp_ns - window_start) * 1.0e-9;
    if (estimator.add_imu_factor(t_s, active.sample, info_diag)) {
      ++impl_->diagnostics.total_imu_factors;
    }
  }
  for (const auto & active : impl_->active_lidar) {
    const double t_s = (active.stamp_ns - window_start) * 1.0e-9;
    if (estimator.add_lidar_factor(
        t_s, active.correspondence, active.extrinsics, active.weight))
    {
      ++impl_->diagnostics.total_lidar_factors;
    }
  }

  if (estimator.imu_factor_count() == 0 && estimator.lidar_factor_count() == 0) {
    return false;
  }

  TrajectoryEstimatorOptions solve_options;
  solve_options.max_num_iterations = impl_->options.max_iterations_per_step;
  solve_options.hold_gyro_bias_constant = impl_->options.hold_gyro_bias_constant;
  solve_options.hold_accel_bias_constant = impl_->options.hold_accel_bias_constant;
  solve_options.hold_gravity_constant = impl_->options.hold_gravity_constant;
  const auto summary = estimator.solve(solve_options);
  impl_->diagnostics.last_step_initial_cost = summary.initial_cost;
  impl_->diagnostics.last_step_final_cost = summary.final_cost;
  ++impl_->diagnostics.steps_run;

  // Pull optimized knots back only when the solve produced a physically
  // plausible online update. Ceres can otherwise turn a bad geometric
  // correspondence set into hundreds of kilometers of odometry in one step.
  const auto rotation_out = estimator.rotation_knots();
  const auto position_out = estimator.position_knots();
  bool reject_update = false;
  double max_position_update = 0.0;
  double max_rotation_update = 0.0;
  if (rotation_out.size() != impl_->rotation_knots.size() ||
    position_out.size() != impl_->position_knots.size())
  {
    reject_update = true;
  } else {
    for (std::size_t i = 0; i < rotation_out.size(); ++i) {
      if (!quaternion_is_valid(rotation_out[i]) || !position_out[i].allFinite()) {
        reject_update = true;
        break;
      }
      max_position_update = std::max(
        max_position_update,
        (position_out[i] - impl_->position_knots[i]).norm());
      max_rotation_update = std::max(
        max_rotation_update,
        rotation_out[i].angularDistance(impl_->rotation_knots[i]));
    }
  }
  if (!estimator.gyro_bias().allFinite() || !estimator.accel_bias().allFinite() ||
    !estimator.gravity_world().allFinite())
  {
    reject_update = true;
  }
  if (impl_->options.max_position_update_m > 0.0 &&
    max_position_update > impl_->options.max_position_update_m)
  {
    reject_update = true;
  }
  if (impl_->options.max_rotation_update_rad > 0.0 &&
    max_rotation_update > impl_->options.max_rotation_update_rad)
  {
    reject_update = true;
  }
  if (reject_update) {
    ++impl_->diagnostics.rejected_solver_steps;
    impl_->diagnostics.last_rejected_position_update_m = max_position_update;
    impl_->diagnostics.last_rejected_rotation_update_rad = max_rotation_update;
    return true;
  }

  for (std::size_t i = 0; i < rotation_out.size(); ++i) {
    impl_->rotation_knots[i] = rotation_out[i];
    impl_->position_knots[i] = position_out[i];
  }
  impl_->gyro_bias = estimator.gyro_bias();
  impl_->accel_bias = estimator.accel_bias();
  impl_->gravity_world = estimator.gravity_world();
  return true;
}

bool ContinuousTimeSlidingWindowEstimator::query_pose(
  int64_t stamp_ns,
  Eigen::Quaterniond & q_w_b,
  Eigen::Vector3d & p_w_b) const
{
  if (impl_->knot_stamps.size() < static_cast<std::size_t>(N)) {
    return false;
  }
  const int64_t window_start = impl_->knot_stamps.front();
  if (stamp_ns < impl_->knot_stamps[1] ||
    stamp_ns > impl_->knot_stamps[impl_->knot_stamps.size() - 3])
  {
    return false;
  }
  const int idx =
    static_cast<int>((stamp_ns - window_start) / impl_->dt_ns);
  if (idx < 1 || idx + 2 >= static_cast<int>(impl_->knot_stamps.size())) {
    return false;
  }
  const double u =
    static_cast<double>(stamp_ns - impl_->knot_stamps[idx]) /
    static_cast<double>(impl_->dt_ns);

  std::array<Eigen::Quaterniond, N> rot_knots;
  std::array<Eigen::Vector3d, N> pos_knots;
  for (int i = 0; i < N; ++i) {
    rot_knots[i] = impl_->rotation_knots[idx - 1 + i];
    pos_knots[i] = impl_->position_knots[idx - 1 + i];
  }
  SplitSplineView<N> view(rot_knots, pos_knots, u, 1.0 / impl_->options.dt_s);
  q_w_b = view.rotation();
  p_w_b = view.position_world();
  return true;
}

int64_t ContinuousTimeSlidingWindowEstimator::newest_knot_stamp_ns() const
{
  return impl_->knot_stamps.empty() ? 0 : impl_->knot_stamps.back();
}

int64_t ContinuousTimeSlidingWindowEstimator::oldest_active_knot_stamp_ns() const
{
  return impl_->knot_stamps.empty() ? 0 : impl_->knot_stamps.front();
}

std::size_t ContinuousTimeSlidingWindowEstimator::active_knot_count() const
{
  return impl_->knot_stamps.size();
}

std::size_t ContinuousTimeSlidingWindowEstimator::buffered_imu_count() const
{
  return impl_->pending_imu.size() + impl_->active_imu.size();
}

std::size_t ContinuousTimeSlidingWindowEstimator::buffered_lidar_count() const
{
  return impl_->pending_lidar.size() + impl_->active_lidar.size();
}

const Eigen::Vector3d & ContinuousTimeSlidingWindowEstimator::gyro_bias() const
{
  return impl_->gyro_bias;
}

const Eigen::Vector3d & ContinuousTimeSlidingWindowEstimator::accel_bias() const
{
  return impl_->accel_bias;
}

const Eigen::Vector3d & ContinuousTimeSlidingWindowEstimator::gravity_world() const
{
  return impl_->gravity_world;
}

const ContinuousTimeSlidingWindowDiagnostics &
ContinuousTimeSlidingWindowEstimator::diagnostics() const
{
  return impl_->diagnostics;
}

}  // namespace spline
}  // namespace gaussian_lic_tracking
