// SPDX-License-Identifier: GPL-3.0-or-later

#include <gaussian_lic_tracking/spline/trajectory_estimator.hpp>

#include <ceres/ceres.h>
#include <ceres/manifold.h>

#include <array>
#include <cmath>
#include <cstring>
#include <stdexcept>

namespace gaussian_lic_tracking
{
namespace spline
{

namespace
{

constexpr int N = TrajectoryEstimator::N;

// NumericDiffCostFunction expects variadic operator() — one `const double*`
// per parameter block plus a final `double*` for residuals. The functors
// below mirror the Coco-LIC IMU/LiDAR knot layout: 4 rotation knots + 4
// position knots (+ biases + gravity for IMU).

Eigen::Quaterniond quaternion_from_coeffs(const double * q)
{
  return Eigen::Quaterniond(q[3], q[0], q[1], q[2]).normalized();
}

struct ImuCeresFunctor
{
  double u_normalized{0.0};
  double inv_dt_s{1.0};
  ImuSample measurement{};
  Eigen::Matrix<double, 6, 1> info_diag{Eigen::Matrix<double, 6, 1>::Ones()};

  bool operator()(
    const double * r0, const double * r1, const double * r2, const double * r3,
    const double * p0, const double * p1, const double * p2, const double * p3,
    const double * bg, const double * ba, const double * g,
    double * residuals) const
  {
    ImuFactorState state;
    state.rotation_knots[0] = quaternion_from_coeffs(r0);
    state.rotation_knots[1] = quaternion_from_coeffs(r1);
    state.rotation_knots[2] = quaternion_from_coeffs(r2);
    state.rotation_knots[3] = quaternion_from_coeffs(r3);
    state.position_knots[0] = Eigen::Map<const Eigen::Vector3d>(p0);
    state.position_knots[1] = Eigen::Map<const Eigen::Vector3d>(p1);
    state.position_knots[2] = Eigen::Map<const Eigen::Vector3d>(p2);
    state.position_knots[3] = Eigen::Map<const Eigen::Vector3d>(p3);
    state.gyro_bias = Eigen::Map<const Eigen::Vector3d>(bg);
    state.accel_bias = Eigen::Map<const Eigen::Vector3d>(ba);
    state.gravity_world = Eigen::Map<const Eigen::Vector3d>(g);

    ContinuousTimeImuFactor factor(u_normalized, inv_dt_s, measurement, info_diag);
    const Eigen::Matrix<double, 6, 1> r = factor.residual(state);
    Eigen::Map<Eigen::Matrix<double, 6, 1>> r_map(residuals);
    r_map = r;
    return true;
  }
};

struct LidarCeresFunctor
{
  double u_normalized{0.0};
  double inv_dt_s{1.0};
  LidarPointCorrespondence correspondence{};
  LidarExtrinsics extrinsics{};
  double weight{1.0};

  bool operator()(
    const double * r0, const double * r1, const double * r2, const double * r3,
    const double * p0, const double * p1, const double * p2, const double * p3,
    double * residuals) const
  {
    LidarFactorState state;
    state.rotation_knots[0] = quaternion_from_coeffs(r0);
    state.rotation_knots[1] = quaternion_from_coeffs(r1);
    state.rotation_knots[2] = quaternion_from_coeffs(r2);
    state.rotation_knots[3] = quaternion_from_coeffs(r3);
    state.position_knots[0] = Eigen::Map<const Eigen::Vector3d>(p0);
    state.position_knots[1] = Eigen::Map<const Eigen::Vector3d>(p1);
    state.position_knots[2] = Eigen::Map<const Eigen::Vector3d>(p2);
    state.position_knots[3] = Eigen::Map<const Eigen::Vector3d>(p3);

    ContinuousTimeLidarFactor factor(
      u_normalized, inv_dt_s, correspondence, extrinsics, weight);
    residuals[0] = factor.residual(state);
    return true;
  }
};

}  // namespace

struct TrajectoryEstimator::Impl
{
  // Use deque so element addresses are stable even after rebuilds; Ceres
  // stores raw double pointers into these vectors for the lifetime of the
  // problem.
  std::vector<Eigen::Vector4d> rotation_storage;   // x, y, z, w per knot
  std::vector<Eigen::Vector3d> position_storage;
  Eigen::Vector3d gyro_bias_storage{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accel_bias_storage{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gravity_storage{Eigen::Vector3d::Zero()};

  std::unique_ptr<ceres::Problem> problem;
};

TrajectoryEstimator::TrajectoryEstimator(double dt_s)
: dt_s_(dt_s),
  impl_(std::make_unique<Impl>())
{
  if (!(dt_s_ > 0.0)) {
    throw std::runtime_error("TrajectoryEstimator dt_s must be positive");
  }
}

TrajectoryEstimator::~TrajectoryEstimator() = default;
TrajectoryEstimator::TrajectoryEstimator(TrajectoryEstimator &&) noexcept = default;
TrajectoryEstimator & TrajectoryEstimator::operator=(TrajectoryEstimator &&) noexcept = default;

void TrajectoryEstimator::set_knots(
  const std::vector<Eigen::Quaterniond> & rotation_knots,
  const std::vector<Eigen::Vector3d> & position_knots)
{
  if (rotation_knots.size() != position_knots.size()) {
    throw std::runtime_error("rotation and position knot counts must match");
  }
  if (rotation_knots.size() < static_cast<std::size_t>(N)) {
    throw std::runtime_error("trajectory estimator needs at least N=4 control knots");
  }
  rotation_knots_.clear();
  rotation_knots_.reserve(rotation_knots.size());
  for (const auto & q : rotation_knots) {
    rotation_knots_.push_back(q.normalized());
  }
  position_knots_ = position_knots;
}

void TrajectoryEstimator::set_gyro_bias(const Eigen::Vector3d & gyro_bias)
{
  gyro_bias_ = gyro_bias;
}

void TrajectoryEstimator::set_accel_bias(const Eigen::Vector3d & accel_bias)
{
  accel_bias_ = accel_bias;
}

void TrajectoryEstimator::set_gravity_world(const Eigen::Vector3d & gravity_world)
{
  gravity_world_ = gravity_world;
}

std::vector<Eigen::Quaterniond> TrajectoryEstimator::rotation_knots() const
{
  return rotation_knots_;
}

std::vector<Eigen::Vector3d> TrajectoryEstimator::position_knots() const
{
  return position_knots_;
}

bool TrajectoryEstimator::find_segment(double t_s, int & segment_index, double & u) const
{
  if (rotation_knots_.size() < static_cast<std::size_t>(N)) {
    return false;
  }
  if (t_s < 0.0) {
    return false;
  }
  const double scaled = t_s / dt_s_;
  const int idx = static_cast<int>(std::floor(scaled));
  if (idx < 1 || idx + 2 >= static_cast<int>(rotation_knots_.size())) {
    return false;
  }
  segment_index = idx;
  u = scaled - static_cast<double>(idx);
  if (u < 0.0) {
    u = 0.0;
  }
  if (u >= 1.0) {
    u = 1.0 - 1.0e-12;
  }
  return true;
}

bool TrajectoryEstimator::add_imu_factor(
  double t_s,
  const ImuSample & sample,
  const Eigen::Matrix<double, 6, 1> & info_diag)
{
  int segment_index = 0;
  double u = 0.0;
  if (!find_segment(t_s, segment_index, u)) {
    return false;
  }

  if (impl_->rotation_storage.empty()) {
    rebuild_problem();
  }

  ImuCeresFunctor functor;
  functor.u_normalized = u;
  functor.inv_dt_s = 1.0 / dt_s_;
  functor.measurement = sample;
  functor.info_diag = info_diag;

  // NumericDiffCostFunction template:
  //   <Functor, NumericDiffMethod, ResidualDim, ParamBlockSizes...>
  auto * cost = new ceres::NumericDiffCostFunction<
    ImuCeresFunctor, ceres::CENTRAL, 6,
    4, 4, 4, 4,
    3, 3, 3, 3,
    3, 3, 3>(new ImuCeresFunctor(functor));

  const int base_rot = segment_index - 1;
  const int base_pos = segment_index - 1;
  std::vector<double *> parameter_blocks;
  parameter_blocks.reserve(2 * N + 3);
  for (int i = 0; i < N; ++i) {
    parameter_blocks.push_back(impl_->rotation_storage[base_rot + i].data());
  }
  for (int i = 0; i < N; ++i) {
    parameter_blocks.push_back(impl_->position_storage[base_pos + i].data());
  }
  parameter_blocks.push_back(impl_->gyro_bias_storage.data());
  parameter_blocks.push_back(impl_->accel_bias_storage.data());
  parameter_blocks.push_back(impl_->gravity_storage.data());

  impl_->problem->AddResidualBlock(cost, nullptr, parameter_blocks);
  ++imu_factor_count_;
  return true;
}

bool TrajectoryEstimator::add_lidar_factor(
  double t_s,
  const LidarPointCorrespondence & correspondence,
  const LidarExtrinsics & extrinsics,
  double weight)
{
  int segment_index = 0;
  double u = 0.0;
  if (!find_segment(t_s, segment_index, u)) {
    return false;
  }

  if (impl_->rotation_storage.empty()) {
    rebuild_problem();
  }

  LidarCeresFunctor functor;
  functor.u_normalized = u;
  functor.inv_dt_s = 1.0 / dt_s_;
  functor.correspondence = correspondence;
  functor.extrinsics = extrinsics;
  functor.weight = weight;

  auto * cost = new ceres::NumericDiffCostFunction<
    LidarCeresFunctor, ceres::CENTRAL, 1,
    4, 4, 4, 4,
    3, 3, 3, 3>(new LidarCeresFunctor(functor));

  const int base = segment_index - 1;
  std::vector<double *> parameter_blocks;
  parameter_blocks.reserve(2 * N);
  for (int i = 0; i < N; ++i) {
    parameter_blocks.push_back(impl_->rotation_storage[base + i].data());
  }
  for (int i = 0; i < N; ++i) {
    parameter_blocks.push_back(impl_->position_storage[base + i].data());
  }

  impl_->problem->AddResidualBlock(cost, nullptr, parameter_blocks);
  ++lidar_factor_count_;
  return true;
}

void TrajectoryEstimator::rebuild_problem()
{
  sync_state_to_storage();

  // Ceres Problem takes ownership of cost / manifold / loss objects by
  // default. Reset the problem to drop any previously-owned manifolds
  // before re-attaching.
  impl_->problem = std::make_unique<ceres::Problem>();

  for (auto & rot : impl_->rotation_storage) {
    impl_->problem->AddParameterBlock(rot.data(), 4, new ceres::EigenQuaternionManifold());
  }
  for (auto & pos : impl_->position_storage) {
    impl_->problem->AddParameterBlock(pos.data(), 3);
  }
  impl_->problem->AddParameterBlock(impl_->gyro_bias_storage.data(), 3);
  impl_->problem->AddParameterBlock(impl_->accel_bias_storage.data(), 3);
  impl_->problem->AddParameterBlock(impl_->gravity_storage.data(), 3);
}

void TrajectoryEstimator::sync_state_to_storage()
{
  impl_->rotation_storage.clear();
  impl_->position_storage.clear();
  impl_->rotation_storage.reserve(rotation_knots_.size());
  impl_->position_storage.reserve(position_knots_.size());
  for (const auto & q : rotation_knots_) {
    impl_->rotation_storage.push_back(q.coeffs());  // (x, y, z, w)
  }
  for (const auto & p : position_knots_) {
    impl_->position_storage.push_back(p);
  }
  impl_->gyro_bias_storage = gyro_bias_;
  impl_->accel_bias_storage = accel_bias_;
  impl_->gravity_storage = gravity_world_;
}

void TrajectoryEstimator::sync_state_from_storage()
{
  rotation_knots_.clear();
  position_knots_.clear();
  rotation_knots_.reserve(impl_->rotation_storage.size());
  position_knots_.reserve(impl_->position_storage.size());
  for (const auto & q : impl_->rotation_storage) {
    rotation_knots_.push_back(Eigen::Quaterniond(q[3], q[0], q[1], q[2]).normalized());
  }
  for (const auto & p : impl_->position_storage) {
    position_knots_.push_back(p);
  }
  gyro_bias_ = impl_->gyro_bias_storage;
  accel_bias_ = impl_->accel_bias_storage;
  gravity_world_ = impl_->gravity_storage;
}

TrajectoryEstimatorSummary TrajectoryEstimator::solve(
  const TrajectoryEstimatorOptions & options)
{
  TrajectoryEstimatorSummary summary;
  if (impl_->rotation_storage.empty()) {
    rebuild_problem();
  }

  if (options.hold_gyro_bias_constant) {
    impl_->problem->SetParameterBlockConstant(impl_->gyro_bias_storage.data());
  }
  if (options.hold_accel_bias_constant) {
    impl_->problem->SetParameterBlockConstant(impl_->accel_bias_storage.data());
  }
  if (options.hold_gravity_constant) {
    impl_->problem->SetParameterBlockConstant(impl_->gravity_storage.data());
  }

  ceres::Solver::Options solver_options;
  solver_options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  solver_options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  solver_options.max_num_iterations = options.max_num_iterations;
  solver_options.function_tolerance = options.function_tolerance;
  solver_options.parameter_tolerance = options.parameter_tolerance;
  solver_options.gradient_tolerance = options.gradient_tolerance;
  solver_options.minimizer_progress_to_stdout = options.minimizer_progress_to_stdout;

  ceres::Solver::Summary ceres_summary;
  ceres::Solve(solver_options, impl_->problem.get(), &ceres_summary);

  sync_state_from_storage();

  summary.initial_cost = ceres_summary.initial_cost;
  summary.final_cost = ceres_summary.final_cost;
  summary.iterations = static_cast<int>(ceres_summary.iterations.size());
  summary.brief_report = ceres_summary.BriefReport();
  summary.success =
    ceres_summary.termination_type == ceres::CONVERGENCE ||
    ceres_summary.termination_type == ceres::USER_SUCCESS;
  return summary;
}

}  // namespace spline
}  // namespace gaussian_lic_tracking
