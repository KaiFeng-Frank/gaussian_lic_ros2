// SPDX-License-Identifier: GPL-3.0-or-later

#include <gaussian_lic_tracking/spline/trajectory_estimator.hpp>

#include <ceres/ceres.h>
#include <ceres/manifold.h>

#include <gaussian_lic_tracking/spline/so3_ops.hpp>

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

// AutoDiff-compatible functors. The operator() is templated on Scalar so
// Ceres can substitute `ceres::Jet<double, N>` and get exact analytic
// Jacobians via dual numbers. The templated math lives in the cumulative
// B-spline `_t` variants under `gaussian_lic_tracking/spline/`.
//
// Parameter block layout for IMU functor (4 rot + 4 pos + bg + ba + g):
//   r0..r3 — rotation knot quaternions (x, y, z, w each)
//   p0..p3 — position knots (x, y, z each)
//   bg     — gyro bias (3)
//   ba     — accel bias (3)
//   g      — gravity world (3)
// LiDAR functor layout: 4 rot + 4 pos.

template <typename T>
Eigen::Quaternion<T> quaternion_from_coeffs_t(const T * q)
{
  return Eigen::Quaternion<T>(q[3], q[0], q[1], q[2]).normalized();
}

struct ImuAutoDiffFunctor
{
  double u_normalized{0.0};
  double inv_dt_s{1.0};
  ImuSample measurement{};
  Eigen::Matrix<double, 6, 1> info_diag{Eigen::Matrix<double, 6, 1>::Ones()};

  template <typename T>
  bool operator()(
    const T * r0, const T * r1, const T * r2, const T * r3,
    const T * p0, const T * p1, const T * p2, const T * p3,
    const T * bg, const T * ba, const T * g,
    T * residuals) const
  {
    std::array<Eigen::Quaternion<T>, 4> rot_knots = {
      quaternion_from_coeffs_t<T>(r0),
      quaternion_from_coeffs_t<T>(r1),
      quaternion_from_coeffs_t<T>(r2),
      quaternion_from_coeffs_t<T>(r3)};
    std::array<Eigen::Matrix<T, 3, 1>, 4> pos_knots = {
      Eigen::Map<const Eigen::Matrix<T, 3, 1>>(p0),
      Eigen::Map<const Eigen::Matrix<T, 3, 1>>(p1),
      Eigen::Map<const Eigen::Matrix<T, 3, 1>>(p2),
      Eigen::Map<const Eigen::Matrix<T, 3, 1>>(p3)};
    const Eigen::Map<const Eigen::Matrix<T, 3, 1>> gyro_bias(bg);
    const Eigen::Map<const Eigen::Matrix<T, 3, 1>> accel_bias(ba);
    const Eigen::Map<const Eigen::Matrix<T, 3, 1>> gravity_world(g);

    Eigen::Quaternion<T> q_w_b;
    Eigen::Matrix<T, 3, 1> omega_b;
    Eigen::Matrix<T, 3, 1> alpha_b;
    CeresSplineHelper<4>::template evaluate_lie_so3_t<T>(
      rot_knots, T(u_normalized), T(inv_dt_s),
      &q_w_b, &omega_b, &alpha_b);
    const Eigen::Matrix<T, 3, 1> a_w_b =
      CeresSplineHelper<4>::template evaluate_rd_t<3, 2, T>(
      pos_knots, T(u_normalized), T(inv_dt_s));

    const Eigen::Matrix<T, 3, 1> meas_gyro_t = measurement.gyro.cast<T>();
    const Eigen::Matrix<T, 3, 1> meas_accel_t = measurement.accel.cast<T>();

    Eigen::Matrix<T, 6, 1> r;
    r.template head<3>() = omega_b - (meas_gyro_t - gyro_bias);
    const Eigen::Matrix<T, 3, 1> predicted_accel =
      q_w_b.inverse() * (a_w_b - gravity_world);
    r.template tail<3>() = predicted_accel - (meas_accel_t - accel_bias);

    const Eigen::Matrix<T, 6, 1> info_diag_t = info_diag.cast<T>();
    Eigen::Map<Eigen::Matrix<T, 6, 1>> r_map(residuals);
    r_map = info_diag_t.asDiagonal() * r;
    return true;
  }
};

struct LidarAutoDiffFunctor
{
  double u_normalized{0.0};
  double inv_dt_s{1.0};
  LidarPointCorrespondence correspondence{};
  LidarExtrinsics extrinsics{};
  double weight{1.0};

  template <typename T>
  bool operator()(
    const T * r0, const T * r1, const T * r2, const T * r3,
    const T * p0, const T * p1, const T * p2, const T * p3,
    T * residuals) const
  {
    std::array<Eigen::Quaternion<T>, 4> rot_knots = {
      quaternion_from_coeffs_t<T>(r0),
      quaternion_from_coeffs_t<T>(r1),
      quaternion_from_coeffs_t<T>(r2),
      quaternion_from_coeffs_t<T>(r3)};
    std::array<Eigen::Matrix<T, 3, 1>, 4> pos_knots = {
      Eigen::Map<const Eigen::Matrix<T, 3, 1>>(p0),
      Eigen::Map<const Eigen::Matrix<T, 3, 1>>(p1),
      Eigen::Map<const Eigen::Matrix<T, 3, 1>>(p2),
      Eigen::Map<const Eigen::Matrix<T, 3, 1>>(p3)};

    Eigen::Quaternion<T> q_w_b;
    CeresSplineHelper<4>::template evaluate_lie_so3_t<T>(
      rot_knots, T(u_normalized), T(inv_dt_s),
      &q_w_b, nullptr, nullptr);
    const Eigen::Matrix<T, 3, 1> p_w_b =
      CeresSplineHelper<4>::template evaluate_rd_t<3, 0, T>(
      pos_knots, T(u_normalized), T(inv_dt_s));

    const Eigen::Quaternion<T> q_lidar_to_imu_t =
      extrinsics.q_lidar_to_imu.cast<T>();
    const Eigen::Matrix<T, 3, 1> p_lidar_in_imu_t =
      extrinsics.p_lidar_in_imu.cast<T>();
    const Eigen::Quaternion<T> q_world_to_map_t =
      extrinsics.q_world_to_map.cast<T>();
    const Eigen::Matrix<T, 3, 1> p_world_in_map_t =
      extrinsics.p_world_in_map.cast<T>();

    const Eigen::Matrix<T, 3, 1> point_lidar_t =
      correspondence.point_lidar.cast<T>();
    const Eigen::Matrix<T, 3, 1> p_imu =
      q_lidar_to_imu_t * point_lidar_t + p_lidar_in_imu_t;
    const Eigen::Matrix<T, 3, 1> p_world = q_w_b * p_imu + p_w_b;
    const Eigen::Matrix<T, 3, 1> p_map = q_world_to_map_t * p_world + p_world_in_map_t;

    if (correspondence.geometry == LidarFeatureGeometry::kPlane) {
      const Eigen::Matrix<T, 3, 1> plane_normal_t =
        correspondence.plane.template head<3>().cast<T>();
      const T plane_offset_t = T(correspondence.plane[3]);
      residuals[0] = T(weight) * (plane_normal_t.dot(p_map) + plane_offset_t);
    } else {
      const Eigen::Matrix<T, 3, 1> edge_point_t = correspondence.edge_point.cast<T>();
      const Eigen::Matrix<T, 3, 1> edge_normal_t = correspondence.edge_normal.cast<T>();
      const Eigen::Matrix<T, 3, 1> diff = p_map - edge_point_t;
      const Eigen::Matrix<T, 3, 1> cross_vec = diff.cross(edge_normal_t);
      residuals[0] = T(weight) * cross_vec.norm();
    }
    return true;
  }
};

struct LidarPlaneNormalAutoDiffFunctor
{
  double u_normalized{0.0};
  double inv_dt_s{1.0};
  Eigen::Vector3d normal_lidar{Eigen::Vector3d::UnitZ()};
  Eigen::Vector3d normal_world{Eigen::Vector3d::UnitZ()};
  LidarExtrinsics extrinsics{};
  double weight{1.0};

  template <typename T>
  bool operator()(
    const T * r0, const T * r1, const T * r2, const T * r3,
    T * residuals) const
  {
    std::array<Eigen::Quaternion<T>, 4> rot_knots = {
      quaternion_from_coeffs_t<T>(r0),
      quaternion_from_coeffs_t<T>(r1),
      quaternion_from_coeffs_t<T>(r2),
      quaternion_from_coeffs_t<T>(r3)};

    Eigen::Quaternion<T> q_w_b;
    CeresSplineHelper<4>::template evaluate_lie_so3_t<T>(
      rot_knots, T(u_normalized), T(inv_dt_s),
      &q_w_b, nullptr, nullptr);

    const Eigen::Quaternion<T> q_lidar_to_imu_t =
      extrinsics.q_lidar_to_imu.cast<T>();
    const Eigen::Matrix<T, 3, 1> n_l = normal_lidar.cast<T>().normalized();
    const Eigen::Matrix<T, 3, 1> n_w = normal_world.cast<T>().normalized();
    const Eigen::Matrix<T, 3, 1> predicted =
      (q_w_b * q_lidar_to_imu_t * n_l).normalized();
    Eigen::Map<Eigen::Matrix<T, 3, 1>> r(residuals);
    r = T(weight) * predicted.cross(n_w);
    return true;
  }
};

struct PositionPriorAutoDiffFunctor
{
  double u_normalized{0.0};
  double inv_dt_s{1.0};
  Eigen::Vector3d position_world{Eigen::Vector3d::Zero()};
  double weight{1.0};

  template <typename T>
  bool operator()(
    const T * p0, const T * p1, const T * p2, const T * p3,
    T * residuals) const
  {
    std::array<Eigen::Matrix<T, 3, 1>, 4> pos_knots = {
      Eigen::Map<const Eigen::Matrix<T, 3, 1>>(p0),
      Eigen::Map<const Eigen::Matrix<T, 3, 1>>(p1),
      Eigen::Map<const Eigen::Matrix<T, 3, 1>>(p2),
      Eigen::Map<const Eigen::Matrix<T, 3, 1>>(p3)};

    const Eigen::Matrix<T, 3, 1> p_w_b =
      CeresSplineHelper<4>::template evaluate_rd_t<3, 0, T>(
      pos_knots, T(u_normalized), T(inv_dt_s));
    const Eigen::Matrix<T, 3, 1> target = position_world.cast<T>();
    Eigen::Map<Eigen::Matrix<T, 3, 1>> r(residuals);
    r = T(weight) * (p_w_b - target);
    return true;
  }
};

struct VelocityPriorAutoDiffFunctor
{
  double u_normalized{0.0};
  double inv_dt_s{1.0};
  Eigen::Vector3d velocity_world{Eigen::Vector3d::Zero()};
  double weight{1.0};

  template <typename T>
  bool operator()(
    const T * p0, const T * p1, const T * p2, const T * p3,
    T * residuals) const
  {
    std::array<Eigen::Matrix<T, 3, 1>, 4> pos_knots = {
      Eigen::Map<const Eigen::Matrix<T, 3, 1>>(p0),
      Eigen::Map<const Eigen::Matrix<T, 3, 1>>(p1),
      Eigen::Map<const Eigen::Matrix<T, 3, 1>>(p2),
      Eigen::Map<const Eigen::Matrix<T, 3, 1>>(p3)};

    const Eigen::Matrix<T, 3, 1> v_w_b =
      CeresSplineHelper<4>::template evaluate_rd_t<3, 1, T>(
      pos_knots, T(u_normalized), T(inv_dt_s));
    const Eigen::Matrix<T, 3, 1> target = velocity_world.cast<T>();
    Eigen::Map<Eigen::Matrix<T, 3, 1>> r(residuals);
    r = T(weight) * (v_w_b - target);
    return true;
  }
};

struct OrientationPriorAutoDiffFunctor
{
  double u_normalized{0.0};
  double inv_dt_s{1.0};
  Eigen::Quaterniond q_world_body{Eigen::Quaterniond::Identity()};
  double weight{1.0};

  template <typename T>
  bool operator()(
    const T * r0, const T * r1, const T * r2, const T * r3,
    T * residuals) const
  {
    std::array<Eigen::Quaternion<T>, 4> rot_knots = {
      quaternion_from_coeffs_t<T>(r0),
      quaternion_from_coeffs_t<T>(r1),
      quaternion_from_coeffs_t<T>(r2),
      quaternion_from_coeffs_t<T>(r3)};

    Eigen::Quaternion<T> q_w_b;
    CeresSplineHelper<4>::template evaluate_lie_so3_t<T>(
      rot_knots, T(u_normalized), T(inv_dt_s),
      &q_w_b, nullptr, nullptr);
    const Eigen::Quaternion<T> q_target = q_world_body.cast<T>().normalized();
    const Eigen::Quaternion<T> error = q_target.conjugate() * q_w_b.normalized();
    Eigen::Map<Eigen::Matrix<T, 3, 1>> r(residuals);
    r = T(weight) * quaternion_log_t<T>(error);
    return true;
  }
};

struct AngularVelocityPriorAutoDiffFunctor
{
  double u_normalized{0.0};
  double inv_dt_s{1.0};
  Eigen::Vector3d angular_velocity_body{Eigen::Vector3d::Zero()};
  double weight{1.0};

  template <typename T>
  bool operator()(
    const T * r0, const T * r1, const T * r2, const T * r3,
    T * residuals) const
  {
    std::array<Eigen::Quaternion<T>, 4> rot_knots = {
      quaternion_from_coeffs_t<T>(r0),
      quaternion_from_coeffs_t<T>(r1),
      quaternion_from_coeffs_t<T>(r2),
      quaternion_from_coeffs_t<T>(r3)};

    Eigen::Matrix<T, 3, 1> omega_b;
    CeresSplineHelper<4>::template evaluate_lie_so3_t<T>(
      rot_knots, T(u_normalized), T(inv_dt_s),
      nullptr, &omega_b, nullptr);
    const Eigen::Matrix<T, 3, 1> target = angular_velocity_body.cast<T>();
    Eigen::Map<Eigen::Matrix<T, 3, 1>> r(residuals);
    r = T(weight) * (omega_b - target);
    return true;
  }
};

struct PositionSmoothnessAutoDiffFunctor
{
  double weight{1.0};

  template <typename T>
  bool operator()(const T * p0, const T * p1, const T * p2, T * residuals) const
  {
    const Eigen::Map<const Eigen::Matrix<T, 3, 1>> x0(p0);
    const Eigen::Map<const Eigen::Matrix<T, 3, 1>> x1(p1);
    const Eigen::Map<const Eigen::Matrix<T, 3, 1>> x2(p2);
    Eigen::Map<Eigen::Matrix<T, 3, 1>> r(residuals);
    r = T(weight) * (x2 - T(2.0) * x1 + x0);
    return true;
  }
};

struct RotationSmoothnessAutoDiffFunctor
{
  double weight{1.0};

  template <typename T>
  bool operator()(const T * r0, const T * r1, const T * r2, T * residuals) const
  {
    const Eigen::Quaternion<T> q0 = quaternion_from_coeffs_t<T>(r0);
    const Eigen::Quaternion<T> q1 = quaternion_from_coeffs_t<T>(r1);
    const Eigen::Quaternion<T> q2 = quaternion_from_coeffs_t<T>(r2);
    const Eigen::Quaternion<T> delta01 = (q0.conjugate() * q1).normalized();
    const Eigen::Quaternion<T> delta12 = (q1.conjugate() * q2).normalized();
    const Eigen::Quaternion<T> second_delta = (delta01.conjugate() * delta12).normalized();
    Eigen::Map<Eigen::Matrix<T, 3, 1>> r(residuals);
    r = T(weight) * quaternion_log_t<T>(second_delta);
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
  std::vector<ceres::ResidualBlockId> imu_residual_blocks;
  std::vector<ceres::ResidualBlockId> lidar_residual_blocks;
  std::vector<ceres::ResidualBlockId> position_prior_residual_blocks;
  std::vector<ceres::ResidualBlockId> velocity_prior_residual_blocks;
  std::vector<ceres::ResidualBlockId> orientation_prior_residual_blocks;
  std::vector<ceres::ResidualBlockId> smoothness_residual_blocks;
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

  ImuAutoDiffFunctor functor;
  functor.u_normalized = u;
  functor.inv_dt_s = 1.0 / dt_s_;
  functor.measurement = sample;
  functor.info_diag = info_diag;

  // AutoDiffCostFunction template:
  //   <Functor, ResidualDim, ParamBlockSizes...>
  auto * cost = new ceres::AutoDiffCostFunction<
    ImuAutoDiffFunctor, 6,
    4, 4, 4, 4,
    3, 3, 3, 3,
    3, 3, 3>(new ImuAutoDiffFunctor(functor));

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

  const auto block = impl_->problem->AddResidualBlock(cost, nullptr, parameter_blocks);
  impl_->imu_residual_blocks.push_back(block);
  ++imu_factor_count_;
  return true;
}

bool TrajectoryEstimator::add_lidar_factor(
  double t_s,
  const LidarPointCorrespondence & correspondence,
  const LidarExtrinsics & extrinsics,
  double weight,
  double huber_delta_m)
{
  if (!std::isfinite(weight) || weight <= 0.0 ||
    !std::isfinite(huber_delta_m) || huber_delta_m < 0.0)
  {
    return false;
  }
  int segment_index = 0;
  double u = 0.0;
  if (!find_segment(t_s, segment_index, u)) {
    return false;
  }

  if (impl_->rotation_storage.empty()) {
    rebuild_problem();
  }

  LidarAutoDiffFunctor functor;
  functor.u_normalized = u;
  functor.inv_dt_s = 1.0 / dt_s_;
  functor.correspondence = correspondence;
  functor.extrinsics = extrinsics;
  functor.weight = weight;

  auto * cost = new ceres::AutoDiffCostFunction<
    LidarAutoDiffFunctor, 1,
    4, 4, 4, 4,
    3, 3, 3, 3>(new LidarAutoDiffFunctor(functor));

  const int base = segment_index - 1;
  std::vector<double *> parameter_blocks;
  parameter_blocks.reserve(2 * N);
  for (int i = 0; i < N; ++i) {
    parameter_blocks.push_back(impl_->rotation_storage[base + i].data());
  }
  for (int i = 0; i < N; ++i) {
    parameter_blocks.push_back(impl_->position_storage[base + i].data());
  }

  ceres::LossFunction * loss = nullptr;
  if (huber_delta_m > 0.0) {
    loss = new ceres::HuberLoss(huber_delta_m);
  }
  const auto block = impl_->problem->AddResidualBlock(cost, loss, parameter_blocks);
  impl_->lidar_residual_blocks.push_back(block);
  ++lidar_factor_count_;
  return true;
}

bool TrajectoryEstimator::add_lidar_plane_normal_factor(
  double t_s,
  const Eigen::Vector3d & normal_lidar,
  const Eigen::Vector3d & normal_world,
  const LidarExtrinsics & extrinsics,
  double weight,
  double huber_delta_rad)
{
  if (!normal_lidar.allFinite() || normal_lidar.norm() <= 1.0e-9 ||
    !normal_world.allFinite() || normal_world.norm() <= 1.0e-9 ||
    !std::isfinite(weight) || weight <= 0.0 ||
    !std::isfinite(huber_delta_rad) || huber_delta_rad < 0.0)
  {
    return false;
  }
  int segment_index = 0;
  double u = 0.0;
  if (!find_segment(t_s, segment_index, u)) {
    return false;
  }

  if (impl_->rotation_storage.empty()) {
    rebuild_problem();
  }

  LidarPlaneNormalAutoDiffFunctor functor;
  functor.u_normalized = u;
  functor.inv_dt_s = 1.0 / dt_s_;
  functor.normal_lidar = normal_lidar.normalized();
  functor.normal_world = normal_world.normalized();
  functor.extrinsics = extrinsics;
  functor.weight = weight;

  auto * cost = new ceres::AutoDiffCostFunction<
    LidarPlaneNormalAutoDiffFunctor, 3,
    4, 4, 4, 4>(new LidarPlaneNormalAutoDiffFunctor(functor));

  const int base = segment_index - 1;
  std::vector<double *> parameter_blocks;
  parameter_blocks.reserve(N);
  for (int i = 0; i < N; ++i) {
    parameter_blocks.push_back(impl_->rotation_storage[base + i].data());
  }

  ceres::LossFunction * loss = nullptr;
  if (huber_delta_rad > 0.0) {
    loss = new ceres::HuberLoss(huber_delta_rad);
  }
  const auto block = impl_->problem->AddResidualBlock(cost, loss, parameter_blocks);
  impl_->lidar_residual_blocks.push_back(block);
  ++lidar_normal_factor_count_;
  return true;
}

bool TrajectoryEstimator::add_position_prior_factor(
  double t_s,
  const Eigen::Vector3d & position_world,
  double weight,
  double huber_delta_m)
{
  if (!position_world.allFinite() ||
    !std::isfinite(weight) || weight <= 0.0 ||
    !std::isfinite(huber_delta_m) || huber_delta_m < 0.0)
  {
    return false;
  }
  int segment_index = 0;
  double u = 0.0;
  if (!find_segment(t_s, segment_index, u)) {
    return false;
  }

  if (impl_->rotation_storage.empty()) {
    rebuild_problem();
  }

  PositionPriorAutoDiffFunctor functor;
  functor.u_normalized = u;
  functor.inv_dt_s = 1.0 / dt_s_;
  functor.position_world = position_world;
  functor.weight = weight;

  auto * cost = new ceres::AutoDiffCostFunction<
    PositionPriorAutoDiffFunctor, 3,
    3, 3, 3, 3>(new PositionPriorAutoDiffFunctor(functor));

  const int base = segment_index - 1;
  std::vector<double *> parameter_blocks;
  parameter_blocks.reserve(N);
  for (int i = 0; i < N; ++i) {
    parameter_blocks.push_back(impl_->position_storage[base + i].data());
  }

  ceres::LossFunction * loss = nullptr;
  if (huber_delta_m > 0.0) {
    loss = new ceres::HuberLoss(huber_delta_m);
  }
  const auto block = impl_->problem->AddResidualBlock(cost, loss, parameter_blocks);
  impl_->position_prior_residual_blocks.push_back(block);
  ++position_prior_factor_count_;
  return true;
}

bool TrajectoryEstimator::add_velocity_prior_factor(
  double t_s,
  const Eigen::Vector3d & velocity_world,
  double weight,
  double huber_delta_mps)
{
  if (!velocity_world.allFinite() ||
    !std::isfinite(weight) || weight <= 0.0 ||
    !std::isfinite(huber_delta_mps) || huber_delta_mps < 0.0)
  {
    return false;
  }
  int segment_index = 0;
  double u = 0.0;
  if (!find_segment(t_s, segment_index, u)) {
    return false;
  }

  if (impl_->rotation_storage.empty()) {
    rebuild_problem();
  }

  VelocityPriorAutoDiffFunctor functor;
  functor.u_normalized = u;
  functor.inv_dt_s = 1.0 / dt_s_;
  functor.velocity_world = velocity_world;
  functor.weight = weight;

  auto * cost = new ceres::AutoDiffCostFunction<
    VelocityPriorAutoDiffFunctor, 3,
    3, 3, 3, 3>(new VelocityPriorAutoDiffFunctor(functor));

  const int base = segment_index - 1;
  std::vector<double *> parameter_blocks;
  parameter_blocks.reserve(N);
  for (int i = 0; i < N; ++i) {
    parameter_blocks.push_back(impl_->position_storage[base + i].data());
  }

  ceres::LossFunction * loss = nullptr;
  if (huber_delta_mps > 0.0) {
    loss = new ceres::HuberLoss(huber_delta_mps);
  }
  const auto block = impl_->problem->AddResidualBlock(cost, loss, parameter_blocks);
  impl_->velocity_prior_residual_blocks.push_back(block);
  ++velocity_prior_factor_count_;
  return true;
}

bool TrajectoryEstimator::add_angular_velocity_prior_factor(
  double t_s,
  const Eigen::Vector3d & angular_velocity_body,
  double weight,
  double huber_delta_radps)
{
  if (!angular_velocity_body.allFinite() ||
    !std::isfinite(weight) || weight <= 0.0 ||
    !std::isfinite(huber_delta_radps) || huber_delta_radps < 0.0)
  {
    return false;
  }
  int segment_index = 0;
  double u = 0.0;
  if (!find_segment(t_s, segment_index, u)) {
    return false;
  }

  if (impl_->rotation_storage.empty()) {
    rebuild_problem();
  }

  AngularVelocityPriorAutoDiffFunctor functor;
  functor.u_normalized = u;
  functor.inv_dt_s = 1.0 / dt_s_;
  functor.angular_velocity_body = angular_velocity_body;
  functor.weight = weight;

  auto * cost = new ceres::AutoDiffCostFunction<
    AngularVelocityPriorAutoDiffFunctor, 3,
    4, 4, 4, 4>(new AngularVelocityPriorAutoDiffFunctor(functor));

  const int base = segment_index - 1;
  std::vector<double *> parameter_blocks;
  parameter_blocks.reserve(N);
  for (int i = 0; i < N; ++i) {
    parameter_blocks.push_back(impl_->rotation_storage[base + i].data());
  }

  ceres::LossFunction * loss = nullptr;
  if (huber_delta_radps > 0.0) {
    loss = new ceres::HuberLoss(huber_delta_radps);
  }
  const auto block = impl_->problem->AddResidualBlock(cost, loss, parameter_blocks);
  impl_->orientation_prior_residual_blocks.push_back(block);
  ++angular_velocity_prior_factor_count_;
  return true;
}

bool TrajectoryEstimator::add_orientation_prior_factor(
  double t_s,
  const Eigen::Quaterniond & q_world_body,
  double weight,
  double huber_delta_rad)
{
  if (!q_world_body.coeffs().allFinite() || q_world_body.norm() <= 1.0e-9 ||
    !std::isfinite(weight) || weight <= 0.0 ||
    !std::isfinite(huber_delta_rad) || huber_delta_rad < 0.0)
  {
    return false;
  }
  int segment_index = 0;
  double u = 0.0;
  if (!find_segment(t_s, segment_index, u)) {
    return false;
  }

  if (impl_->rotation_storage.empty()) {
    rebuild_problem();
  }

  OrientationPriorAutoDiffFunctor functor;
  functor.u_normalized = u;
  functor.inv_dt_s = 1.0 / dt_s_;
  functor.q_world_body = q_world_body.normalized();
  functor.weight = weight;

  auto * cost = new ceres::AutoDiffCostFunction<
    OrientationPriorAutoDiffFunctor, 3,
    4, 4, 4, 4>(new OrientationPriorAutoDiffFunctor(functor));

  const int base = segment_index - 1;
  std::vector<double *> parameter_blocks;
  parameter_blocks.reserve(N);
  for (int i = 0; i < N; ++i) {
    parameter_blocks.push_back(impl_->rotation_storage[base + i].data());
  }

  ceres::LossFunction * loss = nullptr;
  if (huber_delta_rad > 0.0) {
    loss = new ceres::HuberLoss(huber_delta_rad);
  }
  const auto block = impl_->problem->AddResidualBlock(cost, loss, parameter_blocks);
  impl_->orientation_prior_residual_blocks.push_back(block);
  ++orientation_prior_factor_count_;
  return true;
}

bool TrajectoryEstimator::add_position_smoothness_factor(
  std::size_t first_knot_index,
  double weight,
  double huber_delta_m)
{
  if (first_knot_index + 2 >= position_knots_.size() ||
    !std::isfinite(weight) || weight <= 0.0 ||
    !std::isfinite(huber_delta_m) || huber_delta_m < 0.0)
  {
    return false;
  }
  if (impl_->rotation_storage.empty()) {
    rebuild_problem();
  }

  PositionSmoothnessAutoDiffFunctor functor;
  functor.weight = weight;
  auto * cost = new ceres::AutoDiffCostFunction<
    PositionSmoothnessAutoDiffFunctor, 3,
    3, 3, 3>(new PositionSmoothnessAutoDiffFunctor(functor));

  ceres::LossFunction * loss = nullptr;
  if (huber_delta_m > 0.0) {
    loss = new ceres::HuberLoss(huber_delta_m);
  }
  const auto block = impl_->problem->AddResidualBlock(
    cost, loss,
    impl_->position_storage[first_knot_index].data(),
    impl_->position_storage[first_knot_index + 1].data(),
    impl_->position_storage[first_knot_index + 2].data());
  impl_->smoothness_residual_blocks.push_back(block);
  ++position_smoothness_factor_count_;
  return true;
}

bool TrajectoryEstimator::add_rotation_smoothness_factor(
  std::size_t first_knot_index,
  double weight,
  double huber_delta_rad)
{
  if (first_knot_index + 2 >= rotation_knots_.size() ||
    !std::isfinite(weight) || weight <= 0.0 ||
    !std::isfinite(huber_delta_rad) || huber_delta_rad < 0.0)
  {
    return false;
  }
  if (impl_->rotation_storage.empty()) {
    rebuild_problem();
  }

  RotationSmoothnessAutoDiffFunctor functor;
  functor.weight = weight;
  auto * cost = new ceres::AutoDiffCostFunction<
    RotationSmoothnessAutoDiffFunctor, 3,
    4, 4, 4>(new RotationSmoothnessAutoDiffFunctor(functor));

  ceres::LossFunction * loss = nullptr;
  if (huber_delta_rad > 0.0) {
    loss = new ceres::HuberLoss(huber_delta_rad);
  }
  const auto block = impl_->problem->AddResidualBlock(
    cost, loss,
    impl_->rotation_storage[first_knot_index].data(),
    impl_->rotation_storage[first_knot_index + 1].data(),
    impl_->rotation_storage[first_knot_index + 2].data());
  impl_->smoothness_residual_blocks.push_back(block);
  ++rotation_smoothness_factor_count_;
  return true;
}

void TrajectoryEstimator::rebuild_problem()
{
  sync_state_to_storage();

  // Ceres Problem takes ownership of cost / manifold / loss objects by
  // default. Reset the problem to drop any previously-owned manifolds
  // before re-attaching.
  impl_->problem = std::make_unique<ceres::Problem>();
  impl_->imu_residual_blocks.clear();
  impl_->lidar_residual_blocks.clear();
  impl_->position_prior_residual_blocks.clear();
  impl_->velocity_prior_residual_blocks.clear();
  impl_->orientation_prior_residual_blocks.clear();
  impl_->smoothness_residual_blocks.clear();

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

  auto evaluate_cost = [this](const std::vector<ceres::ResidualBlockId> & blocks) {
      if (blocks.empty()) {
        return 0.0;
      }
      ceres::Problem::EvaluateOptions eval_options;
      eval_options.residual_blocks = blocks;
      double cost = 0.0;
      if (!impl_->problem->Evaluate(eval_options, &cost, nullptr, nullptr, nullptr) ||
        !std::isfinite(cost))
      {
        return 0.0;
      }
      return cost;
    };

  summary.initial_imu_cost = evaluate_cost(impl_->imu_residual_blocks);
  summary.initial_lidar_cost = evaluate_cost(impl_->lidar_residual_blocks);
  summary.initial_position_prior_cost =
    evaluate_cost(impl_->position_prior_residual_blocks);
  summary.initial_velocity_prior_cost =
    evaluate_cost(impl_->velocity_prior_residual_blocks);
  summary.initial_orientation_prior_cost =
    evaluate_cost(impl_->orientation_prior_residual_blocks);
  summary.initial_smoothness_cost = evaluate_cost(impl_->smoothness_residual_blocks);

  ceres::Solver::Options solver_options;
  solver_options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  solver_options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  solver_options.max_num_iterations = options.max_num_iterations;
  solver_options.function_tolerance = options.function_tolerance;
  solver_options.parameter_tolerance = options.parameter_tolerance;
  solver_options.gradient_tolerance = options.gradient_tolerance;
  solver_options.minimizer_progress_to_stdout = options.minimizer_progress_to_stdout;
  if (std::isfinite(options.initial_trust_region_radius) &&
    options.initial_trust_region_radius > 0.0)
  {
    solver_options.initial_trust_region_radius = options.initial_trust_region_radius;
  }
  if (std::isfinite(options.max_trust_region_radius) &&
    options.max_trust_region_radius > 0.0)
  {
    solver_options.max_trust_region_radius = options.max_trust_region_radius;
  }

  ceres::Solver::Summary ceres_summary;
  ceres::Solve(solver_options, impl_->problem.get(), &ceres_summary);

  sync_state_from_storage();

  summary.initial_cost = ceres_summary.initial_cost;
  summary.final_cost = ceres_summary.final_cost;
  summary.final_imu_cost = evaluate_cost(impl_->imu_residual_blocks);
  summary.final_lidar_cost = evaluate_cost(impl_->lidar_residual_blocks);
  summary.final_position_prior_cost =
    evaluate_cost(impl_->position_prior_residual_blocks);
  summary.final_velocity_prior_cost =
    evaluate_cost(impl_->velocity_prior_residual_blocks);
  summary.final_orientation_prior_cost =
    evaluate_cost(impl_->orientation_prior_residual_blocks);
  summary.final_smoothness_cost = evaluate_cost(impl_->smoothness_residual_blocks);
  summary.iterations = static_cast<int>(ceres_summary.iterations.size());
  summary.brief_report = ceres_summary.BriefReport();
  summary.success =
    ceres_summary.termination_type == ceres::CONVERGENCE ||
    ceres_summary.termination_type == ceres::USER_SUCCESS;
  return summary;
}

}  // namespace spline
}  // namespace gaussian_lic_tracking
