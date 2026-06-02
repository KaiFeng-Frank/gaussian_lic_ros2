// SPDX-License-Identifier: GPL-3.0-or-later

#include <gaussian_lic_tracking/spline/continuous_time_sliding_window.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <stdexcept>

#include <gaussian_lic_tracking/spline/ceres_spline_helper.hpp>
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

// Increment 2 Part B3: Coco-LIC adaptive control-point density.
// Ported VERBATIM from external/Coco-LIC/src/odom/odometry_manager.h:60-107
// (GetKnotDensity): gears 1/2/3/4 keyed on IMU motion, taking the max of the
// gyro-based and (gravity-removed) accel-based gear. Returns the number of
// control points to add over a window-extend segment (1 == single push ==
// uniform / byte-identical).
inline int get_knot_density(double gyro_norm, double acce_norm, double gravity_norm)
{
  acce_norm = std::abs(acce_norm - gravity_norm);
  auto gear = [](double v) { return v < 0.5 ? 1 : (v < 1.0 ? 2 : (v < 5.0 ? 3 : 4)); };
  return std::max(gear(gyro_norm), gear(acce_norm));
}

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

struct BufferedPhotometricObservation
{
  int64_t stamp_ns{0};
  PhotometricObservation observation;
  double huber_delta{0.0};
};

struct BufferedVelocityPrior
{
  int64_t stamp_ns{0};
  Eigen::Vector3d velocity_world{Eigen::Vector3d::Zero()};
  double weight{1.0};
  double huber_delta_mps{0.0};
};

struct BufferedAccelerationPrior
{
  int64_t stamp_ns{0};
  Eigen::Vector3d acceleration_world{Eigen::Vector3d::Zero()};
  double weight{1.0};
  double huber_delta_mps2{0.0};
};

struct BufferedAngularVelocityPrior
{
  int64_t stamp_ns{0};
  Eigen::Vector3d angular_velocity_body{Eigen::Vector3d::Zero()};
  double weight{1.0};
  double huber_delta_radps{0.0};
};

struct BufferedRelativePositionPrior
{
  int64_t start_stamp_ns{0};
  int64_t end_stamp_ns{0};
  Eigen::Vector3d target_p_body0{Eigen::Vector3d::Zero()};
  double weight{1.0};
  double huber_delta_m{0.0};
};

struct BufferedRelativeOrientationPrior
{
  int64_t start_stamp_ns{0};
  int64_t end_stamp_ns{0};
  Eigen::Quaterniond target_q_body0_body1{Eigen::Quaterniond::Identity()};
  double weight{1.0};
  double huber_delta_rad{0.0};
};

struct BufferedOrientationPrior
{
  int64_t stamp_ns{0};
  Eigen::Quaterniond q_world_body{Eigen::Quaterniond::Identity()};
  double weight{1.0};
  double huber_delta_rad{0.0};
};

struct BufferedDensePositionPrior
{
  std::vector<int64_t> knot_stamps;
  std::vector<Eigen::Vector3d> reference_positions;
  Eigen::MatrixXd jacobian;
  Eigen::VectorXd residual;
};

struct BufferedDenseOrientationPrior
{
  std::vector<int64_t> knot_stamps;
  std::vector<Eigen::Quaterniond> reference_rotations;
  Eigen::MatrixXd jacobian;
  Eigen::VectorXd residual;
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

  // DIAGNOSTIC seed reference (ascending by stamp). Empty => disabled.
  std::vector<int64_t> reference_stamps_ns;
  std::vector<Eigen::Quaterniond> reference_rot;
  std::vector<Eigen::Vector3d> reference_pos;
  bool interpolate_reference(int64_t stamp_ns, Eigen::Quaterniond & q, Eigen::Vector3d & p) const
  {
    if (reference_stamps_ns.empty()) {return false;}
    if (stamp_ns <= reference_stamps_ns.front()) {q = reference_rot.front(); p = reference_pos.front(); return true;}
    if (stamp_ns >= reference_stamps_ns.back()) {q = reference_rot.back(); p = reference_pos.back(); return true;}
    const auto it = std::upper_bound(reference_stamps_ns.begin(), reference_stamps_ns.end(), stamp_ns);
    const std::size_t hi = static_cast<std::size_t>(it - reference_stamps_ns.begin());
    const std::size_t lo = hi - 1;
    const int64_t s0 = reference_stamps_ns[lo], s1 = reference_stamps_ns[hi];
    const double alpha = (s1 > s0) ? static_cast<double>(stamp_ns - s0) / static_cast<double>(s1 - s0) : 0.0;
    p = reference_pos[lo] + alpha * (reference_pos[hi] - reference_pos[lo]);
    q = reference_rot[lo].slerp(alpha, reference_rot[hi]).normalized();
    return true;
  }

  Eigen::Vector3d gyro_bias{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accel_bias{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gravity_world{Eigen::Vector3d::Zero()};
  // Increment 3B: open-loop IMU-integrated velocity used to seed newly-appended
  // position knots, decoupling the metric seed from the (cold-start
  // under-scaled) optimized position history. Reset to zero at initialize().
  Eigen::Vector3d seed_velocity{Eigen::Vector3d::Zero()};

  // Active factors persist across steps — they are removed only when the
  // window has marginalized past their stamp.
  std::deque<BufferedImu> active_imu;
  std::deque<BufferedLidar> active_lidar;
  std::deque<BufferedLidarPointToPoint> active_lidar_points;
  std::deque<BufferedLidarNormal> active_lidar_normals;
  std::deque<BufferedPhotometricObservation> active_photometric;
  std::deque<BufferedPositionPrior> active_position_priors;
  std::deque<BufferedVelocityPrior> active_velocity_priors;
  std::deque<BufferedAccelerationPrior> active_acceleration_priors;
  std::deque<BufferedAngularVelocityPrior> active_angular_velocity_priors;
  std::deque<BufferedRelativePositionPrior> active_relative_position_priors;
  std::deque<BufferedRelativeOrientationPrior> active_relative_orientation_priors;
  std::deque<BufferedOrientationPrior> active_orientation_priors;
  std::deque<BufferedDensePositionPrior> active_dense_position_priors;
  std::deque<BufferedDenseOrientationPrior> active_dense_orientation_priors;

  // Brand-new samples waiting for their first solve.
  std::deque<BufferedImu> pending_imu;
  std::deque<BufferedLidar> pending_lidar;
  std::deque<BufferedLidarPointToPoint> pending_lidar_points;
  std::deque<BufferedLidarNormal> pending_lidar_normals;
  std::deque<BufferedPhotometricObservation> pending_photometric;
  std::deque<BufferedPositionPrior> pending_position_priors;
  std::deque<BufferedVelocityPrior> pending_velocity_priors;
  std::deque<BufferedAccelerationPrior> pending_acceleration_priors;
  std::deque<BufferedAngularVelocityPrior> pending_angular_velocity_priors;
  std::deque<BufferedRelativePositionPrior> pending_relative_position_priors;
  std::deque<BufferedRelativeOrientationPrior> pending_relative_orientation_priors;
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
  if (!std::isfinite(options.bias_random_walk_reference_dt_s) ||
    options.bias_random_walk_reference_dt_s <= 0.0 ||
    !std::isfinite(options.gyro_bias_random_walk_sigma_radps_per_sqrt_s) ||
    options.gyro_bias_random_walk_sigma_radps_per_sqrt_s < 0.0 ||
    !std::isfinite(options.accel_bias_random_walk_sigma_mps2_per_sqrt_s) ||
    options.accel_bias_random_walk_sigma_mps2_per_sqrt_s < 0.0)
  {
    throw std::runtime_error("bias random-walk parameters must be finite and non-negative");
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
  impl_->seed_velocity.setZero();  // Increment 3B: reset open-loop seed velocity
  for (std::size_t i = 0; i < rotation_knots.size(); ++i) {
    const int64_t knot_stamp = start_stamp_ns + static_cast<int64_t>(i) * impl_->dt_ns;
    impl_->knot_stamps.push_back(knot_stamp);
    Eigen::Quaterniond q_seed; Eigen::Vector3d p_seed;
    if (impl_->interpolate_reference(knot_stamp, q_seed, p_seed)) {
      impl_->rotation_knots.push_back(q_seed.normalized());
      impl_->position_knots.push_back(p_seed);
    } else {
      impl_->rotation_knots.push_back(rotation_knots[i].normalized());
      impl_->position_knots.push_back(position_knots[i]);
    }
  }
}

void ContinuousTimeSlidingWindowEstimator::set_reference_trajectory(
  const std::vector<int64_t> & stamps_ns,
  const std::vector<Eigen::Quaterniond> & q_world_body,
  const std::vector<Eigen::Vector3d> & p_world)
{
  impl_->reference_stamps_ns.clear();
  impl_->reference_rot.clear();
  impl_->reference_pos.clear();
  const std::size_t n = std::min({stamps_ns.size(), q_world_body.size(), p_world.size()});
  for (std::size_t i = 0; i < n; ++i) {
    impl_->reference_stamps_ns.push_back(stamps_ns[i]);
    impl_->reference_rot.push_back(q_world_body[i].normalized());
    impl_->reference_pos.push_back(p_world[i]);
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

void ContinuousTimeSlidingWindowEstimator::add_photometric_factor(
  int64_t stamp_ns,
  const PhotometricObservation & observation,
  double huber_delta)
{
  const double effective_huber =
    huber_delta >= 0.0 ? huber_delta : impl_->options.lidar_huber_delta_m;
  impl_->pending_photometric.push_back({stamp_ns, observation, effective_huber});
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

void ContinuousTimeSlidingWindowEstimator::add_acceleration_prior(
  int64_t stamp_ns,
  const Eigen::Vector3d & acceleration_world,
  double weight,
  double huber_delta_mps2)
{
  const double effective_huber =
    huber_delta_mps2 >= 0.0 ? huber_delta_mps2 : impl_->options.lidar_huber_delta_m;
  impl_->pending_acceleration_priors.push_back(
    {stamp_ns, acceleration_world, weight, effective_huber});
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

void ContinuousTimeSlidingWindowEstimator::add_relative_position_prior(
  int64_t start_stamp_ns,
  int64_t end_stamp_ns,
  const Eigen::Vector3d & target_p_body0,
  double weight,
  double huber_delta_m)
{
  const double effective_huber =
    huber_delta_m >= 0.0 ? huber_delta_m : impl_->options.lidar_huber_delta_m;
  impl_->pending_relative_position_priors.push_back(
    {start_stamp_ns, end_stamp_ns, target_p_body0, weight, effective_huber});
}

void ContinuousTimeSlidingWindowEstimator::add_relative_orientation_prior(
  int64_t start_stamp_ns,
  int64_t end_stamp_ns,
  const Eigen::Quaterniond & target_q_body0_body1,
  double weight,
  double huber_delta_rad)
{
  const double effective_huber =
    huber_delta_rad >= 0.0 ? huber_delta_rad : impl_->options.lidar_huber_delta_m;
  impl_->pending_relative_orientation_priors.push_back(
    {start_stamp_ns, end_stamp_ns, target_q_body0_body1.normalized(), weight, effective_huber});
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
  impl_->diagnostics.last_step_spline_marginalization_prior_factors = 0U;
  impl_->diagnostics.last_step_spline_marginalization_prior_rows = 0U;
  impl_->diagnostics.last_step_spline_orientation_marginalization_prior_factors = 0U;
  impl_->diagnostics.last_step_spline_orientation_marginalization_prior_rows = 0U;

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
  std::size_t num_new_knots = 0;  // Increment 3C: knots appended this step
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
    // Increment 3: IMU-propagation metric seed (faithful to upstream
    // InitTrajWithPropagation, trajectory_manager.cpp:274-316). Augment the
    // constant-velocity extrapolation (which supplies v*dt from the optimized
    // history) with the IMU double-integration term 0.5 * a_world * dt^2 so the
    // appended knot gets the full kinematic update p + v*dt + 0.5*a*dt^2 and a
    // METRIC initial guess instead of zero displacement at cold start. World
    // acceleration a_world = R * (accel_meas - accel_bias) + gravity_world
    // matches the IMU residual convention (trajectory_estimator.cpp:128-130);
    // accel_bias / gravity_world are the live post-solve estimates
    // (updated at :2004-2005). Gated + cold-start-only (the warm-start
    // interpolate_reference override at the knot push below wins when active),
    // so the flag-off path is byte-identical and the validated paths cannot
    // regress.
    Eigen::Vector3d p_seed = p_new;
    if (impl_->options.enable_imu_propagation_seed ||
      impl_->options.enable_imu_velocity_seed)
    {
      Eigen::Vector3d accel_sum = Eigen::Vector3d::Zero();
      int accel_cnt = 0;
      const int64_t seg_start_ns = impl_->knot_stamps.back();
      for (const auto & bi : impl_->pending_imu) {
        if (bi.stamp_ns >= seg_start_ns && bi.stamp_ns < next_stamp) {
          accel_sum += bi.sample.accel;
          ++accel_cnt;
        }
      }
      if (accel_cnt > 0) {
        const Eigen::Vector3d accel_mean =
          accel_sum / static_cast<double>(accel_cnt);
        const Eigen::Vector3d a_world =
          impl_->rotation_knots[n - 1] * (accel_mean - impl_->accel_bias) +
          impl_->gravity_world;
        const double dt_s = static_cast<double>(impl_->dt_ns) * 1.0e-9;
        if (a_world.allFinite() && std::isfinite(dt_s)) {
          if (impl_->options.enable_imu_velocity_seed) {
            // 3B: full IMU kinematics off the open-loop integrated velocity,
            // decoupled from the under-scaled history. p[n-1] is the latest
            // optimized (corrected) position; v carries metric velocity.
            p_seed = impl_->position_knots[n - 1] +
              impl_->seed_velocity * dt_s + 0.5 * a_world * dt_s * dt_s;
            impl_->seed_velocity += a_world * dt_s;
          } else {
            // 3: augment the constant-velocity extrapolation with 0.5*a*dt^2.
            p_seed = p_new + 0.5 * a_world * dt_s * dt_s;
          }
        }
      }
    }
    // Increment 2 Part B3: adaptive Coco-LIC knot density. When the flag is on
    // AND adaptive density is enabled, compute the number of control points to
    // add over this window-extend segment from the IMU motion in the segment
    // [base_stamp, next_stamp) (mean |gyro|, mean |accel|), then subdivide the
    // segment into cp_add_num equal steps (intermediate knots at
    // base_stamp+step*k for k<cp_add_num, final knot EXACTLY at next_stamp).
    // Mirrors odometry_manager.cpp:540-554. When the flag is off OR cp_add_num
    // resolves to 1, this is a single push at next_stamp == byte-identical to
    // the original code. DETERMINISTIC (no RNG) -> CT_seed warm-start
    // reproducible.
    int cp_add_num = 1;
    if (impl_->options.enable_non_uniform_knots &&
      impl_->options.enable_adaptive_knot_density)
    {
      Eigen::Vector3d ar = Eigen::Vector3d::Zero();
      Eigen::Vector3d aa = Eigen::Vector3d::Zero();
      int cnt = 0;
      const int64_t s0 = impl_->knot_stamps.back();
      for (const auto & bi : impl_->pending_imu) {
        if (bi.stamp_ns >= s0 && bi.stamp_ns < next_stamp) {
          ar += bi.sample.gyro;
          aa += bi.sample.accel;
          ++cnt;
        }
      }
      if (cnt > 0) {
        ar /= static_cast<double>(cnt);
        aa /= static_cast<double>(cnt);
        cp_add_num = std::max(
          1, get_knot_density(ar.norm(), aa.norm(), impl_->gravity_world.norm()));
      }
    }
    const int64_t base_stamp = impl_->knot_stamps.back();
    const int64_t step = (next_stamp - base_stamp) / cp_add_num;
    for (int k = 1; k <= cp_add_num; ++k) {
      const int64_t t = (k < cp_add_num) ? (base_stamp + step * k) : next_stamp;
      Eigen::Quaterniond qk = q_new;
      Eigen::Vector3d pk = p_seed;
      if (impl_->interpolate_reference(t, qk, pk)) {
        qk = qk.normalized();
      }
      impl_->knot_stamps.push_back(t);
      impl_->rotation_knots.push_back(qk);
      impl_->position_knots.push_back(pk);
      ++num_new_knots;  // Increment 3C
    }
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

  std::deque<BufferedPhotometricObservation> photometric_still_pending;
  for (const auto & p : impl_->pending_photometric) {
    if (p.stamp_ns < interior_end_ns()) {
      impl_->active_photometric.push_back(p);
    } else {
      photometric_still_pending.push_back(p);
    }
  }
  impl_->pending_photometric = photometric_still_pending;

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

  std::deque<BufferedAccelerationPrior> acceleration_prior_still_pending;
  for (const auto & p : impl_->pending_acceleration_priors) {
    if (p.stamp_ns < interior_end_ns()) {
      impl_->active_acceleration_priors.push_back(p);
    } else {
      acceleration_prior_still_pending.push_back(p);
    }
  }
  impl_->pending_acceleration_priors = acceleration_prior_still_pending;

  std::deque<BufferedAngularVelocityPrior> angular_velocity_prior_still_pending;
  for (const auto & p : impl_->pending_angular_velocity_priors) {
    if (p.stamp_ns < interior_end_ns()) {
      impl_->active_angular_velocity_priors.push_back(p);
    } else {
      angular_velocity_prior_still_pending.push_back(p);
    }
  }
  impl_->pending_angular_velocity_priors = angular_velocity_prior_still_pending;

  std::deque<BufferedRelativePositionPrior> relative_position_prior_still_pending;
  for (const auto & p : impl_->pending_relative_position_priors) {
    if (p.end_stamp_ns < interior_end_ns()) {
      impl_->active_relative_position_priors.push_back(p);
    } else {
      relative_position_prior_still_pending.push_back(p);
    }
  }
  impl_->pending_relative_position_priors = relative_position_prior_still_pending;

  std::deque<BufferedRelativeOrientationPrior> relative_orientation_prior_still_pending;
  for (const auto & p : impl_->pending_relative_orientation_priors) {
    if (p.end_stamp_ns < interior_end_ns()) {
      impl_->active_relative_orientation_priors.push_back(p);
    } else {
      relative_orientation_prior_still_pending.push_back(p);
    }
  }
  impl_->pending_relative_orientation_priors = relative_orientation_prior_still_pending;

  std::deque<BufferedOrientationPrior> orientation_prior_still_pending;
  for (const auto & p : impl_->pending_orientation_priors) {
    if (p.stamp_ns < interior_end_ns()) {
      impl_->active_orientation_priors.push_back(p);
    } else {
      orientation_prior_still_pending.push_back(p);
    }
  }
  impl_->pending_orientation_priors = orientation_prior_still_pending;

  auto knot_index_for_stamp = [this](const int64_t stamp_ns) -> std::optional<std::size_t> {
      const auto it = std::find(impl_->knot_stamps.begin(), impl_->knot_stamps.end(), stamp_ns);
      if (it == impl_->knot_stamps.end()) {
        return std::nullopt;
      }
      return static_cast<std::size_t>(std::distance(impl_->knot_stamps.begin(), it));
    };
  auto dense_prior_references_any =
    [](const BufferedDensePositionPrior & prior, const std::vector<int64_t> & stamp_ns) {
      return std::any_of(
        prior.knot_stamps.begin(), prior.knot_stamps.end(),
        [&stamp_ns](const int64_t prior_stamp_ns) {
          return std::find(stamp_ns.begin(), stamp_ns.end(), prior_stamp_ns) != stamp_ns.end();
        });
    };
  auto dense_orientation_prior_references_any =
    [](const BufferedDenseOrientationPrior & prior, const std::vector<int64_t> & stamp_ns) {
      return std::any_of(
        prior.knot_stamps.begin(), prior.knot_stamps.end(),
        [&stamp_ns](const int64_t prior_stamp_ns) {
          return std::find(stamp_ns.begin(), stamp_ns.end(), prior_stamp_ns) != stamp_ns.end();
            });
    };
  auto robust_sqrt_scale =
    [](const Eigen::Vector3d & weighted_residual, double huber_delta, double weight) {
      if (!std::isfinite(huber_delta) || huber_delta <= 0.0 ||
        !std::isfinite(weight) || weight <= 0.0)
      {
        return 1.0;
      }
      const double norm = weighted_residual.norm();
      const double effective_delta = huber_delta * weight;
      if (!std::isfinite(norm) || norm <= effective_delta || effective_delta <= 0.0) {
        return 1.0;
      }
      return std::sqrt(effective_delta / norm);
    };
  auto spline_position_support =
    [this](
      int64_t stamp_ns,
      int derivative,
      Eigen::Matrix<double, N, 1> & coefficients,
      std::array<int64_t, N> & stamps,
      std::array<Eigen::Vector3d, N> & positions) {
      if (impl_->knot_stamps.size() < static_cast<std::size_t>(N) ||
        !(impl_->options.dt_s > 0.0))
      {
        return false;
      }
      const double t_s = static_cast<double>(stamp_ns - impl_->knot_stamps.front()) * 1.0e-9;
      if (!std::isfinite(t_s) || t_s < 0.0) {
        return false;
      }
      // Increment 2 Part B2(a): non-uniform segment lookup. When the flag is on,
      // upper_bound on the (possibly non-uniform) knot-time vector + local u from
      // the actual knot spacing; on uniform knots this is exactly floor(t_s/dt_s)
      // and u==scaled-segment_index, so the off-flag path is byte-identical.
      int segment_index;
      double u;
      if (impl_->options.enable_non_uniform_knots) {
        const auto it = std::upper_bound(
          impl_->knot_stamps.begin(), impl_->knot_stamps.end(), stamp_ns);
        segment_index =
          static_cast<int>(std::distance(impl_->knot_stamps.begin(), it)) - 1;
        if (segment_index < 1 ||
          segment_index + 2 >= static_cast<int>(impl_->knot_stamps.size()))
        {
          return false;
        }
        const double tlo = static_cast<double>(
          impl_->knot_stamps[static_cast<std::size_t>(segment_index)] -
          impl_->knot_stamps.front()) * 1.0e-9;
        const double thi = static_cast<double>(
          impl_->knot_stamps[static_cast<std::size_t>(segment_index + 1)] -
          impl_->knot_stamps.front()) * 1.0e-9;
        if (!(thi > tlo)) {
          return false;
        }
        u = (t_s - tlo) / (thi - tlo);
      } else {
        const double scaled = t_s / impl_->options.dt_s;
        segment_index = static_cast<int>(std::floor(scaled));
        if (segment_index < 1 ||
          segment_index + 2 >= static_cast<int>(impl_->knot_stamps.size()))
        {
          return false;
        }
        u = scaled - static_cast<double>(segment_index);
      }
      if (u < 0.0) {
        u = 0.0;
      }
      if (u >= 1.0) {
        u = 1.0 - 1.0e-12;
      }

      typename CeresSplineHelper<N>::VecN raw;
      if (derivative == 0) {
        CeresSplineHelper<N>::template base_coefficients_with_time<0>(raw, u);
      } else if (derivative == 1) {
        CeresSplineHelper<N>::template base_coefficients_with_time<1>(raw, u);
      } else if (derivative == 2) {
        CeresSplineHelper<N>::template base_coefficients_with_time<2>(raw, u);
      } else {
        return false;
      }
      // Increment 1: per-segment non-uniform blending matrix built from the
      // ACTUAL knot times of the 6-knot window [segment_index-2 ..
      // segment_index+3] (matches Coco-LIC InitBlendMat knts[1..6]). With
      // uniform knots delta_t == dt_s and Mnu == blending_matrix(), so this
      // reproduces the uniform branch exactly.
      // The non-uniform 6-knot window needs segment_index>=2 AND
      // segment_index+3<size, which is STRICTER than the uniform 4-knot window
      // (segment_index>=1, segment_index+2<size) already passed at the guard
      // above. For boundary segments where the full 6-knot window is NOT
      // available we MUST NOT drop the segment (that would silently discard the
      // first valid segment's IMU/lidar/photometric constraints); instead fall
      // back to the uniform 4-knot blending matrix — correct here because at the
      // boundary, and for increment 1's still-uniform knot placement, the
      // uniform matrix is exactly the right operator.
      const bool nonuniform_window_ok =
        impl_->options.enable_non_uniform_knots &&
        segment_index >= 2 &&
        static_cast<std::size_t>(segment_index + 3) < impl_->knot_stamps.size();
      if (nonuniform_window_ok) {
        std::array<double, 6> knot_times_s;
        for (int k = 0; k < 6; ++k) {
          knot_times_s[static_cast<std::size_t>(k)] =
            static_cast<double>(
              impl_->knot_stamps[static_cast<std::size_t>(segment_index - 2 + k)] -
              impl_->knot_stamps.front()) *
            1.0e-9;
        }
        const Eigen::Matrix4d mnu =
          compute_blending_matrix_nonuniform_cubic(knot_times_s, false);
        // Local segment width (seconds): se3_spline.h:478 uses
        // (knts[su.first+1] - knts[su.first]) * NS_TO_S, NOT dt_s.
        const double delta_t =
          static_cast<double>(
            impl_->knot_stamps[static_cast<std::size_t>(segment_index + 1)] -
            impl_->knot_stamps[static_cast<std::size_t>(segment_index)]) *
          1.0e-9;
        coefficients = std::pow(1.0 / delta_t, derivative) * mnu * raw;
      } else {
        // Uniform path: flag off, OR flag on at a boundary segment where the
        // full 6-knot non-uniform window is unavailable (fall back, do NOT drop).
        coefficients =
          std::pow(1.0 / impl_->options.dt_s, derivative) *
          CeresSplineHelper<N>::blending_matrix() * raw;
      }

      const std::size_t base = static_cast<std::size_t>(segment_index - 1);
      for (int i = 0; i < N; ++i) {
        stamps[i] = impl_->knot_stamps[base + static_cast<std::size_t>(i)];
        positions[i] = impl_->position_knots[base + static_cast<std::size_t>(i)];
      }
      return coefficients.allFinite();
    };
  // TODO(increment-2): thread the cumulative non-uniform blending matrix into
  // the rotation path once GetKnotDensity makes the rotation knots non-uniform.
  // In increment 1 rotation is value-only (inv_dt == 1.0 in
  // evaluate_rotation_support) and the rotation knots are still uniformly
  // spaced, so the cumulative non-uniform matrix equals
  // cumulative_blending_matrix() to ~2.22e-16; leaving this path UNCHANGED is
  // provably equivalent and keeps the diff minimal.
  auto spline_rotation_support =
    [this](
      int64_t stamp_ns,
      std::array<int64_t, N> & stamps,
      std::array<Eigen::Quaterniond, N> & rotations,
      double & u) {
      if (impl_->knot_stamps.size() < static_cast<std::size_t>(N) ||
        !(impl_->options.dt_s > 0.0))
      {
        return false;
      }
      const double t_s = static_cast<double>(stamp_ns - impl_->knot_stamps.front()) * 1.0e-9;
      if (!std::isfinite(t_s) || t_s < 0.0) {
        return false;
      }
      // Increment 2 Part B2(b): non-uniform segment lookup (same gated
      // upper_bound index/u as the position support). The rotation value path
      // below stays value-only (evaluate_rotation_support, inv_dt==1.0); this
      // support feeds ONLY the orientation marginalization prior, not the solve
      // or the published .tum. On uniform knots upper_bound==floor so the
      // off-flag path is byte-identical.
      int segment_index;
      if (impl_->options.enable_non_uniform_knots) {
        const auto it = std::upper_bound(
          impl_->knot_stamps.begin(), impl_->knot_stamps.end(), stamp_ns);
        segment_index =
          static_cast<int>(std::distance(impl_->knot_stamps.begin(), it)) - 1;
        if (segment_index < 1 ||
          segment_index + 2 >= static_cast<int>(impl_->knot_stamps.size()))
        {
          return false;
        }
        const double tlo = static_cast<double>(
          impl_->knot_stamps[static_cast<std::size_t>(segment_index)] -
          impl_->knot_stamps.front()) * 1.0e-9;
        const double thi = static_cast<double>(
          impl_->knot_stamps[static_cast<std::size_t>(segment_index + 1)] -
          impl_->knot_stamps.front()) * 1.0e-9;
        if (!(thi > tlo)) {
          return false;
        }
        u = (t_s - tlo) / (thi - tlo);
      } else {
        const double scaled = t_s / impl_->options.dt_s;
        segment_index = static_cast<int>(std::floor(scaled));
        if (segment_index < 1 ||
          segment_index + 2 >= static_cast<int>(impl_->knot_stamps.size()))
        {
          return false;
        }
        u = scaled - static_cast<double>(segment_index);
      }
      if (u < 0.0) {
        u = 0.0;
      }
      if (u >= 1.0) {
        u = 1.0 - 1.0e-12;
      }

      const std::size_t base = static_cast<std::size_t>(segment_index - 1);
      for (int i = 0; i < N; ++i) {
        stamps[i] = impl_->knot_stamps[base + static_cast<std::size_t>(i)];
        rotations[i] = impl_->rotation_knots[base + static_cast<std::size_t>(i)].normalized();
      }
      return true;
    };
  auto evaluate_rotation_support =
    [](const std::array<Eigen::Quaterniond, N> & rotations, double u) {
      Eigen::Quaterniond q_w_b;
      CeresSplineHelper<N>::evaluate_lie_so3(rotations, u, 1.0, &q_w_b, nullptr, nullptr);
      return q_w_b.normalized();
    };
  auto rotation_residual_jacobians =
    [&evaluate_rotation_support, &robust_sqrt_scale](
      const std::array<Eigen::Quaterniond, N> & rotations,
      double u,
      const Eigen::Quaterniond & target,
      double weight,
      double huber_delta,
      Eigen::Vector3d & weighted_residual,
      std::array<Eigen::Matrix3d, N> & jacobians) {
      if (
        target.norm() <= 1.0e-9 ||
        !target.coeffs().allFinite() ||
        !std::isfinite(weight) || weight <= 0.0 ||
        !std::isfinite(huber_delta) || huber_delta < 0.0)
      {
        return false;
      }
      const Eigen::Quaterniond normalized_target = target.normalized();
      const auto residual_at = [&](const std::array<Eigen::Quaterniond, N> & knots) {
          return weight * quaternion_log(
            (normalized_target.conjugate() * evaluate_rotation_support(knots, u)).normalized());
        };

      weighted_residual = residual_at(rotations);
      const double sqrt_scale = robust_sqrt_scale(weighted_residual, huber_delta, weight);
      weighted_residual *= sqrt_scale;
      if (!weighted_residual.allFinite()) {
        return false;
      }

      constexpr double epsilon = 1.0e-6;
      for (int knot = 0; knot < N; ++knot) {
        for (int axis = 0; axis < 3; ++axis) {
          Eigen::Vector3d delta = Eigen::Vector3d::Zero();
          delta[axis] = epsilon;
          auto plus = rotations;
          auto minus = rotations;
          plus[knot] = (plus[knot] * quaternion_exp(delta)).normalized();
          minus[knot] = (minus[knot] * quaternion_exp(-delta)).normalized();
          const Eigen::Vector3d diff =
            (residual_at(plus) - residual_at(minus)) / (2.0 * epsilon);
          jacobians[static_cast<std::size_t>(knot)].col(axis) = sqrt_scale * diff;
        }
      }
      return true;
    };
  auto add_position_like_marginalization_block =
    [&](
      SplineMarginalizationInfo & marginalization,
      int64_t stamp_ns,
      const Eigen::Vector3d & target,
      double weight,
      double huber_delta,
      int derivative,
      const std::vector<int64_t> & marginalized_stamp_ns) {
      if (!target.allFinite() || !std::isfinite(weight) || weight <= 0.0 ||
        !std::isfinite(huber_delta) || huber_delta < 0.0)
      {
        return;
      }

      Eigen::Matrix<double, N, 1> coefficients;
      std::array<int64_t, N> stamps{};
      std::array<Eigen::Vector3d, N> positions{};
      if (!spline_position_support(stamp_ns, derivative, coefficients, stamps, positions)) {
        return;
      }

      const bool touches_marginalized = std::any_of(
        stamps.begin(), stamps.end(),
        [&marginalized_stamp_ns](const int64_t support_stamp_ns) {
          return std::find(
            marginalized_stamp_ns.begin(), marginalized_stamp_ns.end(), support_stamp_ns) !=
                 marginalized_stamp_ns.end();
        });
      if (!touches_marginalized) {
        return;
      }

      Eigen::Vector3d predicted = Eigen::Vector3d::Zero();
      for (int i = 0; i < N; ++i) {
        predicted.noalias() += coefficients[i] * positions[i];
      }
      Eigen::Vector3d weighted_residual = weight * (predicted - target);
      const double sqrt_scale = robust_sqrt_scale(weighted_residual, huber_delta, weight);
      weighted_residual *= sqrt_scale;
      if (!weighted_residual.allFinite()) {
        return;
      }

      LinearizedResidualBlock block;
      block.parameter_block_ids.assign(stamps.begin(), stamps.end());
      block.parameter_block_sizes.assign(static_cast<std::size_t>(N), 3);
      block.jacobians.reserve(static_cast<std::size_t>(N));
      for (int i = 0; i < N; ++i) {
        block.jacobians.push_back(
          sqrt_scale * weight * coefficients[i] * Eigen::Matrix3d::Identity());
      }
      block.residual = weighted_residual;
      marginalization.add_residual_block(block);
    };
  auto build_position_marginalization_prior =
    [this, &knot_index_for_stamp, &dense_prior_references_any,
    &add_position_like_marginalization_block](
      const std::vector<int64_t> & marginalized_stamp_ns) -> std::optional<BufferedDensePositionPrior>
    {
      if (marginalized_stamp_ns.empty()) {
        return std::nullopt;
      }

      SplineMarginalizationInfo marginalization;
      for (const auto stamp_ns : marginalized_stamp_ns) {
        marginalization.mark_block_to_marginalize(stamp_ns);
      }

      for (const auto & prior : impl_->active_position_priors) {
        add_position_like_marginalization_block(
          marginalization, prior.stamp_ns, prior.position_world, prior.weight,
          prior.huber_delta_m, 0, marginalized_stamp_ns);
      }
      for (const auto & prior : impl_->active_velocity_priors) {
        add_position_like_marginalization_block(
          marginalization, prior.stamp_ns, prior.velocity_world, prior.weight,
          prior.huber_delta_mps, 1, marginalized_stamp_ns);
      }
      for (const auto & prior : impl_->active_acceleration_priors) {
        add_position_like_marginalization_block(
          marginalization, prior.stamp_ns, prior.acceleration_world, prior.weight,
          prior.huber_delta_mps2, 2, marginalized_stamp_ns);
      }

      const double smoothness_weight =
        std::isfinite(impl_->options.position_smoothness_weight) ?
        impl_->options.position_smoothness_weight : 0.0;
      if (smoothness_weight > 0.0) {
        for (std::size_t i = 0; i + 2U < impl_->knot_stamps.size(); ++i) {
          const std::array<int64_t, 3U> stamps{
            impl_->knot_stamps[i],
            impl_->knot_stamps[i + 1U],
            impl_->knot_stamps[i + 2U]};
          const bool touches_marginalized = std::any_of(
            stamps.begin(), stamps.end(),
            [&marginalized_stamp_ns](const int64_t stamp_ns) {
              return std::find(
                marginalized_stamp_ns.begin(), marginalized_stamp_ns.end(), stamp_ns) !=
                     marginalized_stamp_ns.end();
            });
          if (!touches_marginalized) {
            continue;
          }

          LinearizedResidualBlock block;
          block.parameter_block_ids = {stamps[0], stamps[1], stamps[2]};
          block.parameter_block_sizes = {3, 3, 3};
          block.jacobians = {
            smoothness_weight * Eigen::Matrix3d::Identity(),
            -2.0 * smoothness_weight * Eigen::Matrix3d::Identity(),
            smoothness_weight * Eigen::Matrix3d::Identity()};
          block.residual =
            smoothness_weight *
            (impl_->position_knots[i + 2U] - 2.0 * impl_->position_knots[i + 1U] +
            impl_->position_knots[i]);
          if (block.residual.allFinite()) {
            marginalization.add_residual_block(block);
          }
        }
      }

      for (const auto & prior : impl_->active_dense_position_priors) {
        if (!dense_prior_references_any(prior, marginalized_stamp_ns)) {
          continue;
        }
        if (prior.knot_stamps.empty() ||
          prior.knot_stamps.size() != prior.reference_positions.size() ||
          prior.jacobian.cols() != static_cast<Eigen::Index>(3 * prior.knot_stamps.size()) ||
          prior.jacobian.rows() != prior.residual.size())
        {
          continue;
        }

        Eigen::VectorXd delta(prior.jacobian.cols());
        bool complete = true;
        for (std::size_t i = 0; i < prior.knot_stamps.size(); ++i) {
          const auto knot_index = knot_index_for_stamp(prior.knot_stamps[i]);
          if (!knot_index.has_value()) {
            complete = false;
            break;
          }
          delta.template segment<3>(static_cast<Eigen::Index>(3 * i)) =
            impl_->position_knots[knot_index.value()] - prior.reference_positions[i];
        }
        if (!complete || !delta.allFinite()) {
          continue;
        }

        LinearizedResidualBlock block;
        block.parameter_block_ids = prior.knot_stamps;
        block.parameter_block_sizes.assign(prior.knot_stamps.size(), 3);
        block.jacobians.reserve(prior.knot_stamps.size());
        for (std::size_t i = 0; i < prior.knot_stamps.size(); ++i) {
          block.jacobians.push_back(
            prior.jacobian.block(
              0, static_cast<Eigen::Index>(3 * i), prior.jacobian.rows(), 3));
        }
        block.residual = prior.jacobian * delta + prior.residual;
        if (block.residual.allFinite()) {
          marginalization.add_residual_block(block);
        }
      }

      MarginalizationResult result;
      if (!marginalization.marginalize(result) ||
        result.kept_block_ids.empty() ||
        result.jacobian.rows() != result.residual.size() ||
        result.jacobian.cols() != result.keep_rows ||
        !result.jacobian.allFinite() ||
        !result.residual.allFinite())
      {
        return std::nullopt;
      }

      BufferedDensePositionPrior prior;
      prior.knot_stamps = result.kept_block_ids;
      prior.reference_positions.reserve(result.kept_block_ids.size());
      for (std::size_t i = 0; i < result.kept_block_ids.size(); ++i) {
        if (result.kept_block_sizes[i] != 3) {
          return std::nullopt;
        }
        const auto knot_index = knot_index_for_stamp(result.kept_block_ids[i]);
        if (!knot_index.has_value()) {
          return std::nullopt;
        }
        prior.reference_positions.push_back(impl_->position_knots[knot_index.value()]);
      }
      prior.jacobian = std::move(result.jacobian);
      prior.residual = std::move(result.residual);
      return prior;
    };
  auto add_orientation_marginalization_block =
    [&](
      SplineMarginalizationInfo & marginalization,
      int64_t stamp_ns,
      const Eigen::Quaterniond & target,
      double weight,
      double huber_delta,
      const std::vector<int64_t> & marginalized_stamp_ns) {
      std::array<int64_t, N> stamps{};
      std::array<Eigen::Quaterniond, N> rotations{};
      double u = 0.0;
      if (!spline_rotation_support(stamp_ns, stamps, rotations, u)) {
        return;
      }
      const bool touches_marginalized = std::any_of(
        stamps.begin(), stamps.end(),
        [&marginalized_stamp_ns](const int64_t support_stamp_ns) {
          return std::find(
            marginalized_stamp_ns.begin(), marginalized_stamp_ns.end(), support_stamp_ns) !=
                 marginalized_stamp_ns.end();
        });
      if (!touches_marginalized) {
        return;
      }

      Eigen::Vector3d weighted_residual = Eigen::Vector3d::Zero();
      std::array<Eigen::Matrix3d, N> jacobians{};
      if (!rotation_residual_jacobians(
          rotations, u, target, weight, huber_delta, weighted_residual, jacobians))
      {
        return;
      }

      LinearizedResidualBlock block;
      block.parameter_block_ids.assign(stamps.begin(), stamps.end());
      block.parameter_block_sizes.assign(static_cast<std::size_t>(N), 3);
      block.jacobians.reserve(static_cast<std::size_t>(N));
      for (const auto & jacobian : jacobians) {
        block.jacobians.push_back(jacobian);
      }
      block.residual = weighted_residual;
      marginalization.add_residual_block(block);
    };
  auto build_orientation_marginalization_prior =
    [this, &knot_index_for_stamp, &dense_orientation_prior_references_any,
    &add_orientation_marginalization_block](
      const std::vector<int64_t> & marginalized_stamp_ns) -> std::optional<BufferedDenseOrientationPrior>
    {
      if (marginalized_stamp_ns.empty()) {
        return std::nullopt;
      }

      SplineMarginalizationInfo marginalization;
      for (const auto stamp_ns : marginalized_stamp_ns) {
        marginalization.mark_block_to_marginalize(stamp_ns);
      }

      for (const auto & prior : impl_->active_orientation_priors) {
        add_orientation_marginalization_block(
          marginalization, prior.stamp_ns, prior.q_world_body, prior.weight,
          prior.huber_delta_rad, marginalized_stamp_ns);
      }

      const double smoothness_weight =
        std::isfinite(impl_->options.rotation_smoothness_weight) ?
        impl_->options.rotation_smoothness_weight : 0.0;
      if (smoothness_weight > 0.0) {
        for (std::size_t i = 0; i + 2U < impl_->knot_stamps.size(); ++i) {
          const std::array<int64_t, 3U> stamps{
            impl_->knot_stamps[i],
            impl_->knot_stamps[i + 1U],
            impl_->knot_stamps[i + 2U]};
          const bool touches_marginalized = std::any_of(
            stamps.begin(), stamps.end(),
            [&marginalized_stamp_ns](const int64_t stamp_ns) {
              return std::find(
                marginalized_stamp_ns.begin(), marginalized_stamp_ns.end(), stamp_ns) !=
                     marginalized_stamp_ns.end();
            });
          if (!touches_marginalized) {
            continue;
          }

          const std::array<Eigen::Quaterniond, 3U> rotations{
            impl_->rotation_knots[i].normalized(),
            impl_->rotation_knots[i + 1U].normalized(),
            impl_->rotation_knots[i + 2U].normalized()};
          const Eigen::Quaterniond delta01 =
            (rotations[0].conjugate() * rotations[1]).normalized();
          const Eigen::Quaterniond delta12 =
            (rotations[1].conjugate() * rotations[2]).normalized();
          const Eigen::Quaterniond second_delta =
            (delta01.conjugate() * delta12).normalized();
          Eigen::Vector3d weighted_residual =
            smoothness_weight * quaternion_log(second_delta);
          if (!weighted_residual.allFinite()) {
            continue;
          }

          constexpr double epsilon = 1.0e-6;
          std::array<Eigen::Matrix3d, 3U> jacobians{};
          auto smoothness_residual_at =
            [smoothness_weight](const std::array<Eigen::Quaterniond, 3U> & knots) {
              const Eigen::Quaterniond d01 =
                (knots[0].conjugate() * knots[1]).normalized();
              const Eigen::Quaterniond d12 =
                (knots[1].conjugate() * knots[2]).normalized();
              return smoothness_weight * quaternion_log((d01.conjugate() * d12).normalized());
            };
          for (int knot = 0; knot < 3; ++knot) {
            for (int axis = 0; axis < 3; ++axis) {
              Eigen::Vector3d delta = Eigen::Vector3d::Zero();
              delta[axis] = epsilon;
              auto plus = rotations;
              auto minus = rotations;
              plus[static_cast<std::size_t>(knot)] =
                (plus[static_cast<std::size_t>(knot)] * quaternion_exp(delta)).normalized();
              minus[static_cast<std::size_t>(knot)] =
                (minus[static_cast<std::size_t>(knot)] * quaternion_exp(-delta)).normalized();
              jacobians[static_cast<std::size_t>(knot)].col(axis) =
                (smoothness_residual_at(plus) - smoothness_residual_at(minus)) /
                (2.0 * epsilon);
            }
          }

          LinearizedResidualBlock block;
          block.parameter_block_ids = {stamps[0], stamps[1], stamps[2]};
          block.parameter_block_sizes = {3, 3, 3};
          block.jacobians = {jacobians[0], jacobians[1], jacobians[2]};
          block.residual = weighted_residual;
          marginalization.add_residual_block(block);
        }
      }

      for (const auto & prior : impl_->active_dense_orientation_priors) {
        if (!dense_orientation_prior_references_any(prior, marginalized_stamp_ns)) {
          continue;
        }
        if (prior.knot_stamps.empty() ||
          prior.knot_stamps.size() != prior.reference_rotations.size() ||
          prior.jacobian.cols() != static_cast<Eigen::Index>(3 * prior.knot_stamps.size()) ||
          prior.jacobian.rows() != prior.residual.size())
        {
          continue;
        }

        Eigen::VectorXd delta(prior.jacobian.cols());
        bool complete = true;
        for (std::size_t i = 0; i < prior.knot_stamps.size(); ++i) {
          const auto knot_index = knot_index_for_stamp(prior.knot_stamps[i]);
          if (!knot_index.has_value()) {
            complete = false;
            break;
          }
          delta.template segment<3>(static_cast<Eigen::Index>(3 * i)) =
            quaternion_log(
              (prior.reference_rotations[i].normalized().conjugate() *
              impl_->rotation_knots[knot_index.value()].normalized()).normalized());
        }
        if (!complete || !delta.allFinite()) {
          continue;
        }

        LinearizedResidualBlock block;
        block.parameter_block_ids = prior.knot_stamps;
        block.parameter_block_sizes.assign(prior.knot_stamps.size(), 3);
        block.jacobians.reserve(prior.knot_stamps.size());
        for (std::size_t i = 0; i < prior.knot_stamps.size(); ++i) {
          block.jacobians.push_back(
            prior.jacobian.block(
              0, static_cast<Eigen::Index>(3 * i), prior.jacobian.rows(), 3));
        }
        block.residual = prior.jacobian * delta + prior.residual;
        if (block.residual.allFinite()) {
          marginalization.add_residual_block(block);
        }
      }

      MarginalizationResult result;
      if (!marginalization.marginalize(result) ||
        result.kept_block_ids.empty() ||
        result.jacobian.rows() != result.residual.size() ||
        result.jacobian.cols() != result.keep_rows ||
        !result.jacobian.allFinite() ||
        !result.residual.allFinite())
      {
        return std::nullopt;
      }

      BufferedDenseOrientationPrior prior;
      prior.knot_stamps = result.kept_block_ids;
      prior.reference_rotations.reserve(result.kept_block_ids.size());
      for (std::size_t i = 0; i < result.kept_block_ids.size(); ++i) {
        if (result.kept_block_sizes[i] != 3) {
          return std::nullopt;
        }
        const auto knot_index = knot_index_for_stamp(result.kept_block_ids[i]);
        if (!knot_index.has_value()) {
          return std::nullopt;
        }
        prior.reference_rotations.push_back(impl_->rotation_knots[knot_index.value()].normalized());
      }
      prior.jacobian = std::move(result.jacobian);
      prior.residual = std::move(result.residual);
      return prior;
    };

  // Marginalize oldest knots once the window exceeds the configured size.
  while (
    static_cast<int>(impl_->knot_stamps.size()) >
    impl_->options.window_knot_count &&
    impl_->options.marginalize_oldest_count > 0)
  {
    const std::vector<int64_t> marginalized_stamp_ns{impl_->knot_stamps.front()};
    const auto dense_prior = build_position_marginalization_prior(marginalized_stamp_ns);
    const auto dense_orientation_prior =
      impl_->options.enable_spline_orientation_marginalization_prior ?
      build_orientation_marginalization_prior(marginalized_stamp_ns) :
      std::optional<BufferedDenseOrientationPrior>{};
    impl_->active_dense_position_priors.erase(
      std::remove_if(
        impl_->active_dense_position_priors.begin(),
        impl_->active_dense_position_priors.end(),
        [&dense_prior_references_any, &marginalized_stamp_ns](
          const BufferedDensePositionPrior & prior) {
          return dense_prior_references_any(prior, marginalized_stamp_ns);
        }),
      impl_->active_dense_position_priors.end());
    if (dense_prior.has_value()) {
      impl_->active_dense_position_priors.push_back(dense_prior.value());
      ++impl_->diagnostics.total_spline_marginalization_priors;
      impl_->diagnostics.total_spline_marginalization_prior_rows +=
        static_cast<std::size_t>(dense_prior->residual.size());
    }
    impl_->active_dense_orientation_priors.erase(
      std::remove_if(
        impl_->active_dense_orientation_priors.begin(),
        impl_->active_dense_orientation_priors.end(),
        [&dense_orientation_prior_references_any, &marginalized_stamp_ns](
          const BufferedDenseOrientationPrior & prior) {
          return dense_orientation_prior_references_any(prior, marginalized_stamp_ns);
        }),
      impl_->active_dense_orientation_priors.end());
    if (dense_orientation_prior.has_value()) {
      impl_->active_dense_orientation_priors.push_back(dense_orientation_prior.value());
      ++impl_->diagnostics.total_spline_orientation_marginalization_priors;
      impl_->diagnostics.total_spline_orientation_marginalization_prior_rows +=
        static_cast<std::size_t>(dense_orientation_prior->residual.size());
    }
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
    !impl_->active_photometric.empty() &&
    impl_->active_photometric.front().stamp_ns < interior_start_ns)
  {
    impl_->active_photometric.pop_front();
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
    !impl_->active_acceleration_priors.empty() &&
    impl_->active_acceleration_priors.front().stamp_ns < interior_start_ns)
  {
    impl_->active_acceleration_priors.pop_front();
  }
  while (
    !impl_->active_angular_velocity_priors.empty() &&
    impl_->active_angular_velocity_priors.front().stamp_ns < interior_start_ns)
  {
    impl_->active_angular_velocity_priors.pop_front();
  }
  while (
    !impl_->active_relative_position_priors.empty() &&
    impl_->active_relative_position_priors.front().start_stamp_ns < interior_start_ns)
  {
    impl_->active_relative_position_priors.pop_front();
  }
  while (
    !impl_->active_relative_orientation_priors.empty() &&
    impl_->active_relative_orientation_priors.front().start_stamp_ns < interior_start_ns)
  {
    impl_->active_relative_orientation_priors.pop_front();
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
    impl_->active_photometric.empty() &&
    impl_->active_position_priors.empty() && impl_->active_velocity_priors.empty() &&
    impl_->active_acceleration_priors.empty() &&
    impl_->active_angular_velocity_priors.empty() &&
    impl_->active_relative_position_priors.empty() &&
    impl_->active_relative_orientation_priors.empty() &&
    impl_->active_orientation_priors.empty() &&
    impl_->active_dense_position_priors.empty() &&
    impl_->active_dense_orientation_priors.empty())
  {
    return false;
  }

  const double estimator_dt_s = impl_->options.dt_s;

  // Increment 3C (Option B): faithful upstream InitTrajWithPropagation
  // (trajectory_manager.cpp:274-316). Before the full LIC solve, run an
  // IMU-ONLY Ceres pre-solve that fixes the history knots and locks
  // bias+gravity, letting the IMU factors pull the newly-appended knots to a
  // METRIC position. Bounded by the fixed history (unlike open-loop 3B which
  // diverged). Cold-start-only (reference_stamps_ns empty) + gated; the new
  // knots' refined positions become the initial guess for the main solve below.
  if (impl_->options.enable_imu_presolve_seed &&
    num_new_knots > 0 &&
    impl_->reference_stamps_ns.empty() &&
    !impl_->active_imu.empty())
  {
    const std::size_t kc = impl_->rotation_knots.size();
    if (kc > static_cast<std::size_t>(N)) {
      // Upstream-faithful: fix only the first N (=4) knots (SetFixedIndex(3))
      // and free the whole active window, so the IMU factors distribute over
      // many DOF anchored at the fixed head — avoids the single-free-knot
      // 2nd-derivative ill-conditioning that flings a lone knot to infinity.
      const int fixed_idx = N - 1;
      TrajectoryEstimator presolve(estimator_dt_s);
      presolve.set_knots(
        std::vector<Eigen::Quaterniond>(
          impl_->rotation_knots.begin(), impl_->rotation_knots.end()),
        std::vector<Eigen::Vector3d>(
          impl_->position_knots.begin(), impl_->position_knots.end()));
      if (impl_->options.enable_non_uniform_knots) {
        std::vector<double> ks;
        ks.reserve(impl_->knot_stamps.size());
        for (const auto s : impl_->knot_stamps) {
          ks.push_back(static_cast<double>(s - window_start) * 1.0e-9);
        }
        presolve.set_knot_stamps_s(ks);
        presolve.set_non_uniform(true);
      }
      presolve.set_gyro_bias(impl_->gyro_bias);
      presolve.set_accel_bias(impl_->accel_bias);
      presolve.set_gravity_world(impl_->gravity_world);
      Eigen::Matrix<double, 6, 1> pre_info;
      pre_info <<
        impl_->options.imu_info_gyro, impl_->options.imu_info_gyro,
        impl_->options.imu_info_gyro,
        impl_->options.imu_info_accel, impl_->options.imu_info_accel,
        impl_->options.imu_info_accel;
      std::size_t pre_imu = 0;
      for (const auto & active : impl_->active_imu) {
        const double t_s = (active.stamp_ns - window_start) * 1.0e-9;
        if (presolve.add_imu_factor(t_s, active.sample, pre_info)) {++pre_imu;}
      }
      if (pre_imu > 0) {
        TrajectoryEstimatorOptions popt;
        popt.max_num_iterations = 50;  // upstream InitTrajWithPropagation: 50
        popt.hold_gyro_bias_constant = true;
        popt.hold_accel_bias_constant = true;
        popt.hold_gravity_constant = true;
        popt.fixed_control_point_index = fixed_idx;
        presolve.solve(popt);
        const auto pre_pos = presolve.position_knots();
        const auto pre_rot = presolve.rotation_knots();
        if (pre_pos.size() == impl_->position_knots.size() &&
          pre_rot.size() == impl_->rotation_knots.size())
        {
          bool all_finite = true;
          for (std::size_t i = 0; i < pre_pos.size(); ++i) {
            if (!pre_pos[i].allFinite() || !pre_rot[i].coeffs().allFinite()) {
              all_finite = false;
              break;
            }
          }
          if (all_finite) {
            for (std::size_t i = 0; i < pre_pos.size(); ++i) {
              impl_->position_knots[i] = pre_pos[i];
              impl_->rotation_knots[i] = pre_rot[i].normalized();
            }
          }
        }
      }
    }
  }

  TrajectoryEstimator estimator(estimator_dt_s);
  estimator.set_knots(
    std::vector<Eigen::Quaterniond>(impl_->rotation_knots.begin(), impl_->rotation_knots.end()),
    std::vector<Eigen::Vector3d>(impl_->position_knots.begin(), impl_->position_knots.end()));
  if (impl_->options.enable_non_uniform_knots) {
    // Window-relative knot times in seconds, SAME origin as the factor stamps
    // (t_s = (stamp_ns - window_start) * 1e-9, window_start = knot_stamps.front()
    // at this function's :1356). The estimator's find_segment / per-segment
    // blending-matrix builder consume these. On uniform knot spacing this is
    // exactly {0, dt_s, 2*dt_s, ...} so the on-flag path stays uniform-equivalent.
    std::vector<double> ks;
    ks.reserve(impl_->knot_stamps.size());
    for (const auto s : impl_->knot_stamps) {
      ks.push_back(static_cast<double>(s - window_start) * 1.0e-9);
    }
    estimator.set_knot_stamps_s(ks);
    estimator.set_non_uniform(true);
  }
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

  std::size_t step_spline_marginalization_prior_factors = 0U;
  std::size_t step_spline_marginalization_prior_rows = 0U;
  std::size_t step_spline_orientation_marginalization_prior_factors = 0U;
  std::size_t step_spline_orientation_marginalization_prior_rows = 0U;
  for (const auto & prior : impl_->active_dense_position_priors) {
    if (prior.knot_stamps.empty() ||
      prior.knot_stamps.size() != prior.reference_positions.size() ||
      prior.jacobian.cols() != static_cast<Eigen::Index>(3 * prior.knot_stamps.size()) ||
      prior.jacobian.rows() != prior.residual.size() ||
      !prior.jacobian.allFinite() ||
      !prior.residual.allFinite())
    {
      continue;
    }

    std::vector<std::size_t> knot_indices;
    knot_indices.reserve(prior.knot_stamps.size());
    bool complete = true;
    for (const auto stamp_ns : prior.knot_stamps) {
      const auto knot_index = knot_index_for_stamp(stamp_ns);
      if (!knot_index.has_value()) {
        complete = false;
        break;
      }
      knot_indices.push_back(knot_index.value());
    }
    if (!complete) {
      continue;
    }

    if (estimator.add_dense_position_prior_factor(
        knot_indices, prior.reference_positions, prior.jacobian, prior.residual))
    {
      ++step_spline_marginalization_prior_factors;
      step_spline_marginalization_prior_rows +=
        static_cast<std::size_t>(prior.residual.size());
    }
  }
  for (const auto & prior : impl_->active_dense_orientation_priors) {
    if (!impl_->options.enable_spline_orientation_marginalization_prior) {
      break;
    }
    if (prior.knot_stamps.empty() ||
      prior.knot_stamps.size() != prior.reference_rotations.size() ||
      prior.jacobian.cols() != static_cast<Eigen::Index>(3 * prior.knot_stamps.size()) ||
      prior.jacobian.rows() != prior.residual.size() ||
      !prior.jacobian.allFinite() ||
      !prior.residual.allFinite())
    {
      continue;
    }

    std::vector<std::size_t> knot_indices;
    knot_indices.reserve(prior.knot_stamps.size());
    bool complete = true;
    for (const auto stamp_ns : prior.knot_stamps) {
      const auto knot_index = knot_index_for_stamp(stamp_ns);
      if (!knot_index.has_value()) {
        complete = false;
        break;
      }
      knot_indices.push_back(knot_index.value());
    }
    if (!complete) {
      continue;
    }

    if (estimator.add_dense_orientation_prior_factor(
        knot_indices, prior.reference_rotations, prior.jacobian, prior.residual))
    {
      ++step_spline_orientation_marginalization_prior_factors;
      step_spline_orientation_marginalization_prior_rows +=
        static_cast<std::size_t>(prior.residual.size());
    }
  }

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
  for (const auto & active : impl_->active_photometric) {
    const double t_s = (active.stamp_ns - window_start) * 1.0e-9;
    if (estimator.add_photometric_factor(t_s, active.observation, active.huber_delta)) {
      ++impl_->diagnostics.total_photometric_factors;
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
  for (const auto & active : impl_->active_acceleration_priors) {
    const double t_s = (active.stamp_ns - window_start) * 1.0e-9;
    if (estimator.add_acceleration_prior_factor(
        t_s, active.acceleration_world, active.weight, active.huber_delta_mps2))
    {
      ++impl_->diagnostics.total_acceleration_prior_factors;
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
  for (const auto & active : impl_->active_relative_position_priors) {
    const double t0_s = (active.start_stamp_ns - window_start) * 1.0e-9;
    const double t1_s = (active.end_stamp_ns - window_start) * 1.0e-9;
    if (estimator.add_relative_position_prior_factor(
        t0_s, t1_s, active.target_p_body0, active.weight, active.huber_delta_m))
    {
      ++impl_->diagnostics.total_position_prior_factors;
    }
  }
  for (const auto & active : impl_->active_relative_orientation_priors) {
    const double t0_s = (active.start_stamp_ns - window_start) * 1.0e-9;
    const double t1_s = (active.end_stamp_ns - window_start) * 1.0e-9;
    if (estimator.add_relative_orientation_prior_factor(
        t0_s, t1_s, active.target_q_body0_body1, active.weight,
        active.huber_delta_rad))
    {
      ++impl_->diagnostics.total_orientation_prior_factors;
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
  auto effective_bias_prior_weight = [this](double manual_weight, double sigma) {
      if (std::isfinite(manual_weight) && manual_weight > 0.0) {
        return manual_weight;
      }
      if (!std::isfinite(sigma) || sigma <= 0.0) {
        return 0.0;
      }
      const double reference_dt_s = std::isfinite(impl_->options.bias_random_walk_reference_dt_s) ?
        std::max(impl_->options.bias_random_walk_reference_dt_s, 1.0e-6) : 1.0;
      return 1.0 / (sigma * std::sqrt(reference_dt_s));
    };
  const double effective_gyro_bias_prior_weight = effective_bias_prior_weight(
    impl_->options.gyro_bias_prior_weight,
    impl_->options.gyro_bias_random_walk_sigma_radps_per_sqrt_s);
  const double effective_accel_bias_prior_weight = effective_bias_prior_weight(
    impl_->options.accel_bias_prior_weight,
    impl_->options.accel_bias_random_walk_sigma_mps2_per_sqrt_s);
  impl_->diagnostics.last_step_effective_gyro_bias_prior_weight =
    effective_gyro_bias_prior_weight;
  impl_->diagnostics.last_step_effective_accel_bias_prior_weight =
    effective_accel_bias_prior_weight;
  if (effective_gyro_bias_prior_weight > 0.0 &&
    estimator.add_gyro_bias_prior_factor(
      impl_->gyro_bias, effective_gyro_bias_prior_weight,
      impl_->options.gyro_bias_prior_huber_delta_radps))
  {
    ++impl_->diagnostics.total_gyro_bias_prior_factors;
  }
  if (effective_accel_bias_prior_weight > 0.0 &&
    estimator.add_accel_bias_prior_factor(
      impl_->accel_bias, effective_accel_bias_prior_weight,
      impl_->options.accel_bias_prior_huber_delta_mps2))
  {
    ++impl_->diagnostics.total_accel_bias_prior_factors;
  }

  // Time-varying bias: couple consecutive per-knot bias states with a
  // random-walk factor (same effective weight as the per-step bias prior).
  estimator.add_bias_random_walk_factors(
    effective_gyro_bias_prior_weight, effective_accel_bias_prior_weight);

  if (estimator.imu_factor_count() == 0 && estimator.lidar_factor_count() == 0 &&
    estimator.lidar_point_factor_count() == 0 &&
    estimator.lidar_normal_factor_count() == 0 &&
    estimator.photometric_factor_count() == 0 &&
    estimator.position_prior_factor_count() == 0 &&
    estimator.velocity_prior_factor_count() == 0 &&
    estimator.acceleration_prior_factor_count() == 0 &&
    estimator.angular_velocity_prior_factor_count() == 0 &&
    estimator.orientation_prior_factor_count() == 0 &&
    estimator.gyro_bias_prior_factor_count() == 0 &&
    estimator.accel_bias_prior_factor_count() == 0 &&
    estimator.position_smoothness_factor_count() == 0 &&
    estimator.rotation_smoothness_factor_count() == 0 &&
    estimator.dense_position_prior_factor_count() == 0 &&
    estimator.dense_orientation_prior_factor_count() == 0)
  {
    return false;
  }
  impl_->diagnostics.last_step_imu_factors = estimator.imu_factor_count();
  impl_->diagnostics.last_step_lidar_factors = estimator.lidar_factor_count();
  impl_->diagnostics.last_step_lidar_point_factors = estimator.lidar_point_factor_count();
  impl_->diagnostics.last_step_lidar_normal_factors =
    estimator.lidar_normal_factor_count();
  impl_->diagnostics.last_step_photometric_factors =
    estimator.photometric_factor_count();
  impl_->diagnostics.last_step_position_prior_factors =
    estimator.position_prior_factor_count();
  impl_->diagnostics.last_step_velocity_prior_factors =
    estimator.velocity_prior_factor_count();
  impl_->diagnostics.last_step_acceleration_prior_factors =
    estimator.acceleration_prior_factor_count();
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
  impl_->diagnostics.last_step_spline_marginalization_prior_factors =
    step_spline_marginalization_prior_factors;
  impl_->diagnostics.last_step_spline_marginalization_prior_rows =
    step_spline_marginalization_prior_rows;
  impl_->diagnostics.last_step_spline_orientation_marginalization_prior_factors =
    step_spline_orientation_marginalization_prior_factors;
  impl_->diagnostics.last_step_spline_orientation_marginalization_prior_rows =
    step_spline_orientation_marginalization_prior_rows;

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
  impl_->diagnostics.last_step_initial_acceleration_prior_cost =
    summary.initial_acceleration_prior_cost;
  impl_->diagnostics.last_step_final_acceleration_prior_cost =
    summary.final_acceleration_prior_cost;
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
  // Increment 2 Part B2(c): non-uniform pose query for the PUBLISHED .tum
  // output (the accuracy gate). When the flag is on, locate the segment by
  // upper_bound on the (non-uniform) knot times, build the per-segment
  // non-cumulative (position) + cumulative (rotation) blending matrices from the
  // 6-knot window, and evaluate via the Part A nu_t overloads. Falls through to
  // the verbatim uniform SplitSplineView path for the flag-off case and for
  // boundary segments where the full 6-knot window is unavailable (do NOT drop
  // the sample). On uniform knots this is exactly equivalent to the uniform
  // path (compute_blending_matrix_nonuniform_cubic == static matrices).
  if (impl_->options.enable_non_uniform_knots) {
    const auto it = std::upper_bound(
      impl_->knot_stamps.begin(), impl_->knot_stamps.end(), stamp_ns);
    const int nidx =
      static_cast<int>(std::distance(impl_->knot_stamps.begin(), it)) - 1;
    if (nidx >= 2 && nidx + 3 < static_cast<int>(impl_->knot_stamps.size())) {
      const double tlo = static_cast<double>(
        impl_->knot_stamps[static_cast<std::size_t>(nidx)] - window_start) * 1.0e-9;
      const double thi = static_cast<double>(
        impl_->knot_stamps[static_cast<std::size_t>(nidx + 1)] - window_start) * 1.0e-9;
      const double un = (thi > tlo) ?
        std::clamp(
          static_cast<double>(
            stamp_ns - impl_->knot_stamps[static_cast<std::size_t>(nidx)]) * 1.0e-9 /
          (thi - tlo),
          0.0, 1.0 - 1.0e-12) :
        0.0;
      std::array<double, 6> kt;
      for (int k = 0; k < 6; ++k) {
        kt[static_cast<std::size_t>(k)] = static_cast<double>(
          impl_->knot_stamps[static_cast<std::size_t>(nidx - 2 + k)] - window_start) *
          1.0e-9;
      }
      const Eigen::Matrix4d mc = compute_blending_matrix_nonuniform_cubic(kt, true);
      const Eigen::Matrix4d mn = compute_blending_matrix_nonuniform_cubic(kt, false);
      const double dtn = static_cast<double>(
        impl_->knot_stamps[static_cast<std::size_t>(nidx + 1)] -
        impl_->knot_stamps[static_cast<std::size_t>(nidx)]) * 1.0e-9;
      std::array<Eigen::Quaterniond, N> rk;
      std::array<Eigen::Vector3d, N> pk;
      for (int i = 0; i < N; ++i) {
        rk[static_cast<std::size_t>(i)] =
          impl_->rotation_knots[static_cast<std::size_t>(nidx - 1 + i)];
        pk[static_cast<std::size_t>(i)] =
          impl_->position_knots[static_cast<std::size_t>(nidx - 1 + i)];
      }
      CeresSplineHelper<N>::template evaluate_lie_so3_nu_t<double>(
        rk, un, mc, dtn, &q_w_b, nullptr, nullptr);
      p_w_b = CeresSplineHelper<N>::template evaluate_rd_nu_t<3, 0, double>(
        pk, un, mn, dtn);
      return q_w_b.coeffs().allFinite() && p_w_b.allFinite();
    }
    // else: boundary segment, fall through to the uniform SplitSplineView path.
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
