// SPDX-License-Identifier: GPL-3.0-or-later

#include <gaussian_lic_tracking/spline/continuous_time_sliding_window.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <gaussian_lic_tracking/spline/so3_ops.hpp>

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
  double huber_delta_m{0.0};
};

struct BufferedLidarPointToPoint
{
  int64_t stamp_ns{0};
  Eigen::Vector3d point_lidar{Eigen::Vector3d::Zero()};
  Eigen::Vector3d target_point_map{Eigen::Vector3d::Zero()};
  LidarExtrinsics extrinsics;
  double weight{1.0};
  double huber_delta_m{0.0};
  double scale{1.0};
};

struct BufferedLidarNormal
{
  int64_t stamp_ns{0};
  Eigen::Vector3d normal_lidar{Eigen::Vector3d::UnitZ()};
  Eigen::Vector3d normal_world{Eigen::Vector3d::UnitZ()};
  LidarExtrinsics extrinsics;
  double weight{1.0};
  double huber_delta_rad{0.0};
};

struct BufferedPositionPrior
{
  int64_t stamp_ns{0};
  Eigen::Vector3d position_world{Eigen::Vector3d::Zero()};
  double weight{1.0};
  double huber_delta_m{0.0};
};

struct BufferedVelocityPrior
{
  int64_t stamp_ns{0};
  Eigen::Vector3d velocity_world{Eigen::Vector3d::Zero()};
  double weight{1.0};
  double huber_delta_mps{0.0};
};

struct BufferedAngularVelocityPrior
{
  int64_t stamp_ns{0};
  Eigen::Vector3d angular_velocity_body{Eigen::Vector3d::Zero()};
  double weight{1.0};
  double huber_delta_radps{0.0};
};

struct BufferedOrientationPrior
{
  int64_t stamp_ns{0};
  Eigen::Quaterniond q_world_body{Eigen::Quaterniond::Identity()};
  double weight{1.0};
  double huber_delta_rad{0.0};
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
  std::deque<BufferedLidarPointToPoint> active_lidar_points;
  std::deque<BufferedLidarNormal> active_lidar_normals;
  std::deque<BufferedPositionPrior> active_position_priors;
  std::deque<BufferedVelocityPrior> active_velocity_priors;
  std::deque<BufferedAngularVelocityPrior> active_angular_velocity_priors;
  std::deque<BufferedOrientationPrior> active_orientation_priors;

  // Brand-new samples waiting for their first solve.
  std::deque<BufferedImu> pending_imu;
  std::deque<BufferedLidar> pending_lidar;
  std::deque<BufferedLidarPointToPoint> pending_lidar_points;
  std::deque<BufferedLidarNormal> pending_lidar_normals;
  std::deque<BufferedPositionPrior> pending_position_priors;
  std::deque<BufferedVelocityPrior> pending_velocity_priors;
  std::deque<BufferedAngularVelocityPrior> pending_angular_velocity_priors;
  std::deque<BufferedOrientationPrior> pending_orientation_priors;

  ContinuousTimeSlidingWindowDiagnostics diagnostics;
};

ContinuousTimeSlidingWindowEstimator::ContinuousTimeSlidingWindowEstimator(
  const ContinuousTimeSlidingWindowOptions & options)
: impl_(std::make_unique<Impl>())
{
  if (!(options.dt_s > 0.0)) {
    throw std::runtime_error("sliding window dt_s must be positive");
  }
  if (options.update_gate_edge_knot_margin < 0) {
    throw std::runtime_error("update_gate_edge_knot_margin must be non-negative");
  }
  if (options.fixed_control_point_index < -1) {
    throw std::runtime_error("fixed_control_point_index must be >= -1");
  }
  if (!std::isfinite(options.rotation_smoothness_weight) ||
    options.rotation_smoothness_weight < 0.0 ||
    !std::isfinite(options.rotation_smoothness_huber_delta_rad) ||
    options.rotation_smoothness_huber_delta_rad < 0.0)
  {
    throw std::runtime_error("rotation smoothness parameters must be finite and non-negative");
  }
  if (options.retained_knot_prior_count < 0 ||
    !std::isfinite(options.retained_knot_position_prior_weight) ||
    options.retained_knot_position_prior_weight < 0.0 ||
    !std::isfinite(options.retained_knot_position_prior_huber_delta_m) ||
    options.retained_knot_position_prior_huber_delta_m < 0.0 ||
    !std::isfinite(options.retained_knot_orientation_prior_weight) ||
    options.retained_knot_orientation_prior_weight < 0.0 ||
    !std::isfinite(options.retained_knot_orientation_prior_huber_delta_rad) ||
    options.retained_knot_orientation_prior_huber_delta_rad < 0.0)
  {
    throw std::runtime_error("retained knot prior parameters must be finite and non-negative");
  }
  if (!std::isfinite(options.gyro_bias_prior_weight) ||
    options.gyro_bias_prior_weight < 0.0 ||
    !std::isfinite(options.gyro_bias_prior_huber_delta_radps) ||
    options.gyro_bias_prior_huber_delta_radps < 0.0 ||
    !std::isfinite(options.accel_bias_prior_weight) ||
    options.accel_bias_prior_weight < 0.0 ||
    !std::isfinite(options.accel_bias_prior_huber_delta_mps2) ||
    options.accel_bias_prior_huber_delta_mps2 < 0.0)
  {
    throw std::runtime_error("bias prior parameters must be finite and non-negative");
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
  double weight,
  double huber_delta_m)
{
  const double effective_huber =
    huber_delta_m >= 0.0 ? huber_delta_m : impl_->options.lidar_huber_delta_m;
  impl_->pending_lidar.push_back(
    {stamp_ns, correspondence, extrinsics, weight, effective_huber});
}

void ContinuousTimeSlidingWindowEstimator::add_lidar_point_to_point_correspondence(
  int64_t stamp_ns,
  const Eigen::Vector3d & point_lidar,
  const Eigen::Vector3d & target_point_map,
  const LidarExtrinsics & extrinsics,
  double weight,
  double huber_delta_m,
  double scale)
{
  const double effective_huber =
    huber_delta_m >= 0.0 ? huber_delta_m : impl_->options.lidar_huber_delta_m;
  impl_->pending_lidar_points.push_back(
    {stamp_ns, point_lidar, target_point_map, extrinsics, weight, effective_huber, scale});
}

void ContinuousTimeSlidingWindowEstimator::add_lidar_plane_normal_correspondence(
  int64_t stamp_ns,
  const Eigen::Vector3d & normal_lidar,
  const Eigen::Vector3d & normal_world,
  const LidarExtrinsics & extrinsics,
  double weight,
  double huber_delta_rad)
{
  const double effective_huber =
    huber_delta_rad >= 0.0 ? huber_delta_rad : impl_->options.lidar_huber_delta_m;
  impl_->pending_lidar_normals.push_back(
    {stamp_ns, normal_lidar, normal_world, extrinsics, weight, effective_huber});
}

void ContinuousTimeSlidingWindowEstimator::add_position_prior(
  int64_t stamp_ns,
  const Eigen::Vector3d & position_world,
  double weight,
  double huber_delta_m)
{
  const double effective_huber =
    huber_delta_m >= 0.0 ? huber_delta_m : impl_->options.lidar_huber_delta_m;
  impl_->pending_position_priors.push_back(
    {stamp_ns, position_world, weight, effective_huber});
}

void ContinuousTimeSlidingWindowEstimator::add_velocity_prior(
  int64_t stamp_ns,
  const Eigen::Vector3d & velocity_world,
  double weight,
  double huber_delta_mps)
{
  const double effective_huber =
    huber_delta_mps >= 0.0 ? huber_delta_mps : impl_->options.lidar_huber_delta_m;
  impl_->pending_velocity_priors.push_back(
    {stamp_ns, velocity_world, weight, effective_huber});
}

void ContinuousTimeSlidingWindowEstimator::add_angular_velocity_prior(
  int64_t stamp_ns,
  const Eigen::Vector3d & angular_velocity_body,
  double weight,
  double huber_delta_radps)
{
  const double effective_huber =
    huber_delta_radps >= 0.0 ? huber_delta_radps : impl_->options.lidar_huber_delta_m;
  impl_->pending_angular_velocity_priors.push_back(
    {stamp_ns, angular_velocity_body, weight, effective_huber});
}

void ContinuousTimeSlidingWindowEstimator::add_orientation_prior(
  int64_t stamp_ns,
  const Eigen::Quaterniond & q_world_body,
  double weight,
  double huber_delta_rad)
{
  const double effective_huber =
    huber_delta_rad >= 0.0 ? huber_delta_rad : impl_->options.lidar_huber_delta_m;
  impl_->pending_orientation_priors.push_back(
    {stamp_ns, q_world_body, weight, effective_huber});
}

bool ContinuousTimeSlidingWindowEstimator::apply_pose_hint(
  int64_t stamp_ns,
  const Eigen::Quaterniond & q_world_body,
  const Eigen::Vector3d & position_world,
  double position_gain,
  double rotation_gain)
{
  if (!position_world.allFinite() || !quaternion_is_valid(q_world_body) ||
    !std::isfinite(position_gain) || !std::isfinite(rotation_gain))
  {
    return false;
  }
  Eigen::Quaterniond q_current;
  Eigen::Vector3d p_current;
  if (!query_pose(stamp_ns, q_current, p_current)) {
    return false;
  }
  const double clamped_position_gain = std::clamp(position_gain, 0.0, 1.0);
  const double clamped_rotation_gain = std::clamp(rotation_gain, 0.0, 1.0);
  const Eigen::Vector3d position_delta =
    clamped_position_gain * (position_world - p_current);
  const Eigen::Quaterniond rotation_delta =
    (q_world_body.normalized() * q_current.inverse()).normalized();
  const Eigen::Quaterniond scaled_rotation_delta =
    quaternion_exp(quaternion_log(rotation_delta) * clamped_rotation_gain);
  if (!position_delta.allFinite() || !quaternion_is_valid(scaled_rotation_delta)) {
    return false;
  }
  const int64_t window_start = impl_->knot_stamps.front();
  const double span_ns = static_cast<double>(stamp_ns - window_start);
  for (std::size_t i = 0; i < impl_->position_knots.size(); ++i) {
    double temporal_weight = 1.0;
    if (span_ns > 1.0) {
      temporal_weight =
        std::clamp(static_cast<double>(impl_->knot_stamps[i] - window_start) / span_ns, 0.0, 1.0);
    }
    impl_->position_knots[i] += temporal_weight * position_delta;
    const Eigen::Quaterniond knot_rotation_delta =
      quaternion_exp(quaternion_log(scaled_rotation_delta) * temporal_weight);
    impl_->rotation_knots[i] = (knot_rotation_delta * impl_->rotation_knots[i]).normalized();
  }
  return true;
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
    const double extrapolation_damping =
      std::isfinite(impl_->options.position_extrapolation_damping) ?
      std::clamp(impl_->options.position_extrapolation_damping, 0.0, 1.0) : 1.0;
    const Eigen::Vector3d p_new =
      impl_->position_knots[n - 1] +
      extrapolation_damping *
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

  std::deque<BufferedLidarPointToPoint> lidar_point_still_pending;
  for (const auto & p : impl_->pending_lidar_points) {
    if (p.stamp_ns < interior_end_ns()) {
      impl_->active_lidar_points.push_back(p);
    } else {
      lidar_point_still_pending.push_back(p);
    }
  }
  impl_->pending_lidar_points = lidar_point_still_pending;

  std::deque<BufferedLidarNormal> lidar_normal_still_pending;
  for (const auto & p : impl_->pending_lidar_normals) {
    if (p.stamp_ns < interior_end_ns()) {
      impl_->active_lidar_normals.push_back(p);
    } else {
      lidar_normal_still_pending.push_back(p);
    }
  }
  impl_->pending_lidar_normals = lidar_normal_still_pending;

  std::deque<BufferedPositionPrior> position_prior_still_pending;
  for (const auto & p : impl_->pending_position_priors) {
    if (p.stamp_ns < interior_end_ns()) {
      impl_->active_position_priors.push_back(p);
    } else {
      position_prior_still_pending.push_back(p);
    }
  }
  impl_->pending_position_priors = position_prior_still_pending;

  std::deque<BufferedVelocityPrior> velocity_prior_still_pending;
  for (const auto & p : impl_->pending_velocity_priors) {
    if (p.stamp_ns < interior_end_ns()) {
      impl_->active_velocity_priors.push_back(p);
    } else {
      velocity_prior_still_pending.push_back(p);
    }
  }
  impl_->pending_velocity_priors = velocity_prior_still_pending;

  std::deque<BufferedAngularVelocityPrior> angular_velocity_prior_still_pending;
  for (const auto & p : impl_->pending_angular_velocity_priors) {
    if (p.stamp_ns < interior_end_ns()) {
      impl_->active_angular_velocity_priors.push_back(p);
    } else {
      angular_velocity_prior_still_pending.push_back(p);
    }
  }
  impl_->pending_angular_velocity_priors = angular_velocity_prior_still_pending;

  std::deque<BufferedOrientationPrior> orientation_prior_still_pending;
  for (const auto & p : impl_->pending_orientation_priors) {
    if (p.stamp_ns < interior_end_ns()) {
      impl_->active_orientation_priors.push_back(p);
    } else {
      orientation_prior_still_pending.push_back(p);
    }
  }
  impl_->pending_orientation_priors = orientation_prior_still_pending;

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
  while (
    !impl_->active_lidar_points.empty() &&
    impl_->active_lidar_points.front().stamp_ns < interior_start_ns)
  {
    impl_->active_lidar_points.pop_front();
  }
  while (
    !impl_->active_lidar_normals.empty() &&
    impl_->active_lidar_normals.front().stamp_ns < interior_start_ns)
  {
    impl_->active_lidar_normals.pop_front();
  }
  while (
    !impl_->active_position_priors.empty() &&
    impl_->active_position_priors.front().stamp_ns < interior_start_ns)
  {
    impl_->active_position_priors.pop_front();
  }
  while (
    !impl_->active_velocity_priors.empty() &&
    impl_->active_velocity_priors.front().stamp_ns < interior_start_ns)
  {
    impl_->active_velocity_priors.pop_front();
  }
  while (
    !impl_->active_angular_velocity_priors.empty() &&
    impl_->active_angular_velocity_priors.front().stamp_ns < interior_start_ns)
  {
    impl_->active_angular_velocity_priors.pop_front();
  }
  while (
    !impl_->active_orientation_priors.empty() &&
    impl_->active_orientation_priors.front().stamp_ns < interior_start_ns)
  {
    impl_->active_orientation_priors.pop_front();
  }

  if (impl_->active_imu.empty() && impl_->active_lidar.empty() &&
    impl_->active_lidar_points.empty() &&
    impl_->active_lidar_normals.empty() &&
    impl_->active_position_priors.empty() && impl_->active_velocity_priors.empty() &&
    impl_->active_angular_velocity_priors.empty() &&
    impl_->active_orientation_priors.empty())
  {
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

  std::size_t step_retained_position_priors = 0U;
  std::size_t step_retained_orientation_priors = 0U;
  if (impl_->options.retained_knot_prior_count > 0 &&
    (impl_->options.retained_knot_position_prior_weight > 0.0 ||
    impl_->options.retained_knot_orientation_prior_weight > 0.0))
  {
    const std::size_t retained_count = std::min<std::size_t>(
      static_cast<std::size_t>(impl_->options.retained_knot_prior_count),
      estimator.knot_count());
    for (std::size_t i = 0; i < retained_count; ++i) {
      if (impl_->options.retained_knot_position_prior_weight > 0.0 &&
        estimator.add_knot_position_prior_factor(
          i, impl_->position_knots[i],
          impl_->options.retained_knot_position_prior_weight,
          impl_->options.retained_knot_position_prior_huber_delta_m))
      {
        ++step_retained_position_priors;
      }
      if (impl_->options.retained_knot_orientation_prior_weight > 0.0 &&
        estimator.add_knot_orientation_prior_factor(
          i, impl_->rotation_knots[i],
          impl_->options.retained_knot_orientation_prior_weight,
          impl_->options.retained_knot_orientation_prior_huber_delta_rad))
      {
        ++step_retained_orientation_priors;
      }
    }
  }
  impl_->diagnostics.total_retained_knot_position_prior_factors +=
    step_retained_position_priors;
  impl_->diagnostics.total_retained_knot_orientation_prior_factors +=
    step_retained_orientation_priors;

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
        t_s, active.correspondence, active.extrinsics, active.weight,
        active.huber_delta_m))
    {
      ++impl_->diagnostics.total_lidar_factors;
    }
  }
  for (const auto & active : impl_->active_lidar_points) {
    const double t_s = (active.stamp_ns - window_start) * 1.0e-9;
    if (estimator.add_lidar_point_to_point_factor(
        t_s, active.point_lidar, active.target_point_map, active.extrinsics,
        active.weight, active.huber_delta_m, active.scale))
    {
      ++impl_->diagnostics.total_lidar_point_factors;
    }
  }
  for (const auto & active : impl_->active_lidar_normals) {
    const double t_s = (active.stamp_ns - window_start) * 1.0e-9;
    if (estimator.add_lidar_plane_normal_factor(
        t_s, active.normal_lidar, active.normal_world, active.extrinsics,
        active.weight, active.huber_delta_rad))
    {
      ++impl_->diagnostics.total_lidar_normal_factors;
    }
  }
  for (const auto & active : impl_->active_position_priors) {
    const double t_s = (active.stamp_ns - window_start) * 1.0e-9;
    if (estimator.add_position_prior_factor(
        t_s, active.position_world, active.weight, active.huber_delta_m))
    {
      ++impl_->diagnostics.total_position_prior_factors;
    }
  }
  for (const auto & active : impl_->active_velocity_priors) {
    const double t_s = (active.stamp_ns - window_start) * 1.0e-9;
    if (estimator.add_velocity_prior_factor(
        t_s, active.velocity_world, active.weight, active.huber_delta_mps))
    {
      ++impl_->diagnostics.total_velocity_prior_factors;
    }
  }
  for (const auto & active : impl_->active_angular_velocity_priors) {
    const double t_s = (active.stamp_ns - window_start) * 1.0e-9;
    if (estimator.add_angular_velocity_prior_factor(
        t_s, active.angular_velocity_body, active.weight, active.huber_delta_radps))
    {
      ++impl_->diagnostics.total_angular_velocity_prior_factors;
    }
  }
  for (const auto & active : impl_->active_orientation_priors) {
    const double t_s = (active.stamp_ns - window_start) * 1.0e-9;
    if (estimator.add_orientation_prior_factor(
        t_s, active.q_world_body, active.weight, active.huber_delta_rad))
    {
      ++impl_->diagnostics.total_orientation_prior_factors;
    }
  }
  if (impl_->options.position_smoothness_weight > 0.0) {
    for (std::size_t i = 0; i + 2 < estimator.knot_count(); ++i) {
      if (estimator.add_position_smoothness_factor(
          i, impl_->options.position_smoothness_weight,
          impl_->options.position_smoothness_huber_delta_m))
      {
        ++impl_->diagnostics.total_position_smoothness_factors;
      }
    }
  }
  if (impl_->options.rotation_smoothness_weight > 0.0) {
    for (std::size_t i = 0; i + 2 < estimator.knot_count(); ++i) {
      if (estimator.add_rotation_smoothness_factor(
          i, impl_->options.rotation_smoothness_weight,
          impl_->options.rotation_smoothness_huber_delta_rad))
      {
        ++impl_->diagnostics.total_rotation_smoothness_factors;
      }
    }
  }
  if (impl_->options.gyro_bias_prior_weight > 0.0 &&
    estimator.add_gyro_bias_prior_factor(
      impl_->gyro_bias, impl_->options.gyro_bias_prior_weight,
      impl_->options.gyro_bias_prior_huber_delta_radps))
  {
    ++impl_->diagnostics.total_gyro_bias_prior_factors;
  }
  if (impl_->options.accel_bias_prior_weight > 0.0 &&
    estimator.add_accel_bias_prior_factor(
      impl_->accel_bias, impl_->options.accel_bias_prior_weight,
      impl_->options.accel_bias_prior_huber_delta_mps2))
  {
    ++impl_->diagnostics.total_accel_bias_prior_factors;
  }

  if (estimator.imu_factor_count() == 0 && estimator.lidar_factor_count() == 0 &&
    estimator.lidar_point_factor_count() == 0 &&
    estimator.lidar_normal_factor_count() == 0 &&
    estimator.position_prior_factor_count() == 0 &&
    estimator.velocity_prior_factor_count() == 0 &&
    estimator.angular_velocity_prior_factor_count() == 0 &&
    estimator.orientation_prior_factor_count() == 0 &&
    estimator.gyro_bias_prior_factor_count() == 0 &&
    estimator.accel_bias_prior_factor_count() == 0 &&
    estimator.position_smoothness_factor_count() == 0 &&
    estimator.rotation_smoothness_factor_count() == 0)
  {
    return false;
  }
  impl_->diagnostics.last_step_imu_factors = estimator.imu_factor_count();
  impl_->diagnostics.last_step_lidar_factors = estimator.lidar_factor_count();
  impl_->diagnostics.last_step_lidar_point_factors = estimator.lidar_point_factor_count();
  impl_->diagnostics.last_step_lidar_normal_factors =
    estimator.lidar_normal_factor_count();
  impl_->diagnostics.last_step_position_prior_factors =
    estimator.position_prior_factor_count();
  impl_->diagnostics.last_step_velocity_prior_factors =
    estimator.velocity_prior_factor_count();
  impl_->diagnostics.last_step_angular_velocity_prior_factors =
    estimator.angular_velocity_prior_factor_count();
  impl_->diagnostics.last_step_orientation_prior_factors =
    estimator.orientation_prior_factor_count();
  impl_->diagnostics.last_step_gyro_bias_prior_factors =
    estimator.gyro_bias_prior_factor_count();
  impl_->diagnostics.last_step_accel_bias_prior_factors =
    estimator.accel_bias_prior_factor_count();
  impl_->diagnostics.last_step_position_smoothness_factors =
    estimator.position_smoothness_factor_count();
  impl_->diagnostics.last_step_rotation_smoothness_factors =
    estimator.rotation_smoothness_factor_count();
  impl_->diagnostics.last_step_retained_knot_position_prior_factors =
    step_retained_position_priors;
  impl_->diagnostics.last_step_retained_knot_orientation_prior_factors =
    step_retained_orientation_priors;

  TrajectoryEstimatorOptions solve_options;
  solve_options.max_num_iterations = impl_->options.max_iterations_per_step;
  solve_options.initial_trust_region_radius =
    impl_->options.ceres_initial_trust_region_radius;
  solve_options.max_trust_region_radius =
    impl_->options.ceres_max_trust_region_radius;
  solve_options.hold_gyro_bias_constant = impl_->options.hold_gyro_bias_constant;
  solve_options.hold_accel_bias_constant = impl_->options.hold_accel_bias_constant;
  solve_options.hold_gravity_constant = impl_->options.hold_gravity_constant;
  solve_options.fixed_control_point_index = impl_->options.fixed_control_point_index;
  const auto summary = estimator.solve(solve_options);
  impl_->diagnostics.last_step_initial_cost = summary.initial_cost;
  impl_->diagnostics.last_step_final_cost = summary.final_cost;
  impl_->diagnostics.last_step_initial_imu_cost = summary.initial_imu_cost;
  impl_->diagnostics.last_step_final_imu_cost = summary.final_imu_cost;
  impl_->diagnostics.last_step_initial_lidar_cost = summary.initial_lidar_cost;
  impl_->diagnostics.last_step_final_lidar_cost = summary.final_lidar_cost;
  impl_->diagnostics.last_step_initial_position_prior_cost =
    summary.initial_position_prior_cost;
  impl_->diagnostics.last_step_final_position_prior_cost =
    summary.final_position_prior_cost;
  impl_->diagnostics.last_step_initial_velocity_prior_cost =
    summary.initial_velocity_prior_cost;
  impl_->diagnostics.last_step_final_velocity_prior_cost =
    summary.final_velocity_prior_cost;
  impl_->diagnostics.last_step_initial_orientation_prior_cost =
    summary.initial_orientation_prior_cost;
  impl_->diagnostics.last_step_final_orientation_prior_cost =
    summary.final_orientation_prior_cost;
  impl_->diagnostics.last_step_initial_bias_prior_cost =
    summary.initial_bias_prior_cost;
  impl_->diagnostics.last_step_final_bias_prior_cost =
    summary.final_bias_prior_cost;
  impl_->diagnostics.last_step_initial_smoothness_cost = summary.initial_smoothness_cost;
  impl_->diagnostics.last_step_final_smoothness_cost = summary.final_smoothness_cost;
  impl_->diagnostics.last_step_update_accepted = false;
  impl_->diagnostics.last_step_update_rejected = false;
  impl_->diagnostics.last_step_rotation_limited = false;
  impl_->diagnostics.last_step_position_limited = false;
  ++impl_->diagnostics.steps_run;

  // Pull optimized knots back only when the solve produced a physically
  // plausible online update. Ceres can otherwise turn a bad geometric
  // correspondence set into hundreds of kilometers of odometry in one step.
  const auto rotation_out = estimator.rotation_knots();
  const auto position_out = estimator.position_knots();
  bool invalid_update = false;
  double max_position_update = 0.0;
  double max_rotation_update = 0.0;
  std::size_t gated_update_count = 0U;
  if (rotation_out.size() != impl_->rotation_knots.size() ||
    position_out.size() != impl_->position_knots.size())
  {
    invalid_update = true;
  } else {
    for (std::size_t i = 0; i < rotation_out.size(); ++i) {
      if (!quaternion_is_valid(rotation_out[i]) || !position_out[i].allFinite()) {
        invalid_update = true;
        break;
      }
      const std::size_t edge_margin =
        static_cast<std::size_t>(impl_->options.update_gate_edge_knot_margin);
      const bool include_in_update_gate =
        edge_margin == 0U || (i >= edge_margin && i + edge_margin < rotation_out.size());
      if (include_in_update_gate) {
        ++gated_update_count;
        max_position_update = std::max(
          max_position_update,
          (position_out[i] - impl_->position_knots[i]).norm());
        max_rotation_update = std::max(
          max_rotation_update,
          rotation_out[i].angularDistance(impl_->rotation_knots[i]));
      }
    }
    if (!invalid_update && gated_update_count == 0U) {
      invalid_update = true;
    }
  }
  if (!estimator.gyro_bias().allFinite() || !estimator.accel_bias().allFinite() ||
    !estimator.gravity_world().allFinite())
  {
    invalid_update = true;
  }
  const bool position_update_too_large =
    impl_->options.max_position_update_m > 0.0 &&
    max_position_update > impl_->options.max_position_update_m;
  const bool rotation_update_too_large =
    impl_->options.max_rotation_update_rad > 0.0 &&
    max_rotation_update > impl_->options.max_rotation_update_rad;
  impl_->diagnostics.last_step_max_position_update_m = max_position_update;
  impl_->diagnostics.last_step_max_rotation_update_rad = max_rotation_update;
  const auto mark_applied_update = [&]() {
    ++impl_->diagnostics.accepted_solver_steps;
    impl_->diagnostics.last_step_update_accepted = true;
  };
  const auto apply_auxiliary_state_update = [&](double scale) {
    const double update_scale = std::clamp(std::isfinite(scale) ? scale : 0.0, 0.0, 1.0);
    impl_->gyro_bias += update_scale * (estimator.gyro_bias() - impl_->gyro_bias);
    impl_->accel_bias += update_scale * (estimator.accel_bias() - impl_->accel_bias);
    impl_->gravity_world += update_scale * (estimator.gravity_world() - impl_->gravity_world);
  };
  if (invalid_update) {
    ++impl_->diagnostics.rejected_solver_steps;
    impl_->diagnostics.last_step_update_rejected = true;
    ++impl_->diagnostics.invalid_update_rejections;
    impl_->diagnostics.last_rejected_position_update_m = max_position_update;
    impl_->diagnostics.last_rejected_rotation_update_rad = max_rotation_update;
    return true;
  }
  if (position_update_too_large) {
    const bool can_limit_position =
      impl_->options.apply_limited_position_update &&
      impl_->options.max_position_update_m > 0.0 &&
      max_position_update > 1.0e-12;
    const bool can_limit_rotation =
      rotation_update_too_large &&
      impl_->options.apply_limited_rotation_update &&
      impl_->options.max_rotation_update_rad > 0.0 &&
      max_rotation_update > 1.0e-12;
    const bool can_apply_without_rotation =
      rotation_update_too_large && impl_->options.apply_position_update_on_rotation_reject;
    if (can_limit_position &&
      (!rotation_update_too_large || can_limit_rotation || can_apply_without_rotation))
    {
      const double position_scale =
        std::clamp(impl_->options.max_position_update_m / max_position_update, 0.0, 1.0);
      const double rotation_scale = can_limit_rotation ?
        std::clamp(impl_->options.max_rotation_update_rad / max_rotation_update, 0.0, 1.0) :
        1.0;
      for (std::size_t i = 0; i < rotation_out.size(); ++i) {
        const Eigen::Vector3d position_delta = position_out[i] - impl_->position_knots[i];
        impl_->position_knots[i] += position_scale * position_delta;
        if (!rotation_update_too_large) {
          impl_->rotation_knots[i] = rotation_out[i];
        } else if (can_limit_rotation) {
          const Eigen::Quaterniond delta =
            impl_->rotation_knots[i].inverse() * rotation_out[i];
          impl_->rotation_knots[i] =
            (impl_->rotation_knots[i] *
            quaternion_exp(quaternion_log(delta) * rotation_scale)).normalized();
        }
      }
      ++impl_->diagnostics.position_limited_solver_steps;
      impl_->diagnostics.last_step_position_limited = true;
      impl_->diagnostics.last_position_limited_position_update_m = max_position_update;
      impl_->diagnostics.last_position_limited_rotation_update_rad = max_rotation_update;
      if (can_limit_rotation) {
        ++impl_->diagnostics.rotation_limited_solver_steps;
        impl_->diagnostics.last_step_rotation_limited = true;
        impl_->diagnostics.last_rotation_limited_position_update_m = max_position_update;
        impl_->diagnostics.last_rotation_limited_rotation_update_rad = max_rotation_update;
      }
      const double auxiliary_scale = (rotation_update_too_large && !can_limit_rotation) ?
        0.0 : std::min(position_scale, rotation_scale);
      apply_auxiliary_state_update(auxiliary_scale);
      mark_applied_update();
      return true;
    }
    ++impl_->diagnostics.rejected_solver_steps;
    impl_->diagnostics.last_step_update_rejected = true;
    ++impl_->diagnostics.position_update_rejections;
    impl_->diagnostics.last_rejected_position_update_m = max_position_update;
    impl_->diagnostics.last_rejected_rotation_update_rad = max_rotation_update;
    return true;
  }
  if (rotation_update_too_large) {
    if (impl_->options.apply_limited_rotation_update &&
      impl_->options.max_rotation_update_rad > 0.0 &&
      max_rotation_update > 1.0e-12)
    {
      const double rotation_scale =
        std::clamp(impl_->options.max_rotation_update_rad / max_rotation_update, 0.0, 1.0);
      for (std::size_t i = 0; i < rotation_out.size(); ++i) {
        const Eigen::Quaterniond delta =
          impl_->rotation_knots[i].inverse() * rotation_out[i];
        impl_->rotation_knots[i] =
          (impl_->rotation_knots[i] *
          quaternion_exp(quaternion_log(delta) * rotation_scale)).normalized();
        const Eigen::Vector3d position_delta = position_out[i] - impl_->position_knots[i];
        impl_->position_knots[i] += impl_->options.scale_position_with_limited_rotation ?
          rotation_scale * position_delta : position_delta;
      }
      ++impl_->diagnostics.rotation_limited_solver_steps;
      impl_->diagnostics.last_step_rotation_limited = true;
      impl_->diagnostics.last_rotation_limited_position_update_m = max_position_update;
      impl_->diagnostics.last_rotation_limited_rotation_update_rad = max_rotation_update;
      apply_auxiliary_state_update(rotation_scale);
      mark_applied_update();
      return true;
    }
    if (!impl_->options.apply_position_update_on_rotation_reject) {
      ++impl_->diagnostics.rejected_solver_steps;
      impl_->diagnostics.last_step_update_rejected = true;
      ++impl_->diagnostics.rotation_update_rejections;
      impl_->diagnostics.last_rejected_position_update_m = max_position_update;
      impl_->diagnostics.last_rejected_rotation_update_rad = max_rotation_update;
      return true;
    }
    for (std::size_t i = 0; i < position_out.size(); ++i) {
      impl_->position_knots[i] = position_out[i];
    }
    ++impl_->diagnostics.rotation_limited_solver_steps;
    impl_->diagnostics.last_step_rotation_limited = true;
    impl_->diagnostics.last_rotation_limited_position_update_m = max_position_update;
    impl_->diagnostics.last_rotation_limited_rotation_update_rad = max_rotation_update;
    mark_applied_update();
    return true;
  }

  for (std::size_t i = 0; i < rotation_out.size(); ++i) {
    impl_->rotation_knots[i] = rotation_out[i];
    impl_->position_knots[i] = position_out[i];
  }
  apply_auxiliary_state_update(1.0);
  mark_applied_update();
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
  return impl_->pending_lidar.size() + impl_->active_lidar.size() +
         impl_->pending_lidar_normals.size() + impl_->active_lidar_normals.size();
}

const Eigen::Vector3d & ContinuousTimeSlidingWindowEstimator::gyro_bias() const
{
  return impl_->gyro_bias;
}

const Eigen::Vector3d & ContinuousTimeSlidingWindowEstimator::accel_bias() const
{
  return impl_->accel_bias;
}

void ContinuousTimeSlidingWindowEstimator::set_gyro_bias(const Eigen::Vector3d & gyro_bias)
{
  if (gyro_bias.allFinite()) {
    impl_->gyro_bias = gyro_bias;
  }
}

void ContinuousTimeSlidingWindowEstimator::set_accel_bias(const Eigen::Vector3d & accel_bias)
{
  if (accel_bias.allFinite()) {
    impl_->accel_bias = accel_bias;
  }
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
