// SPDX-License-Identifier: GPL-3.0-or-later
//
// Verifies the continuous-time LiDAR factor reproduces the geometric distance
// for a known plane and that its analytic body-pose Jacobian matches finite
// differences.

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gaussian_lic_tracking/spline/continuous_time_lidar_factor.hpp>

using gaussian_lic_tracking::spline::ContinuousTimeLidarFactor;
using gaussian_lic_tracking::spline::LidarExtrinsics;
using gaussian_lic_tracking::spline::LidarFactorState;
using gaussian_lic_tracking::spline::LidarFeatureGeometry;
using gaussian_lic_tracking::spline::LidarPointCorrespondence;
using gaussian_lic_tracking::spline::numeric_residual_with_body_pose_offset;

namespace
{

constexpr int N = ContinuousTimeLidarFactor::N;

LidarFactorState identity_state()
{
  LidarFactorState state;
  for (int i = 0; i < N; ++i) {
    state.rotation_knots[i] = Eigen::Quaterniond::Identity();
    state.position_knots[i] = Eigen::Vector3d::Zero();
  }
  return state;
}

void check_plane_residual_zero_for_consistent_point()
{
  // World identity pose; point on z=0 plane should produce zero residual.
  LidarFactorState state = identity_state();
  LidarExtrinsics extrinsics;  // identity
  LidarPointCorrespondence pc;
  pc.point_lidar = Eigen::Vector3d(2.0, -1.0, 0.0);
  pc.geometry = LidarFeatureGeometry::kPlane;
  pc.plane << 0.0, 0.0, 1.0, 0.0;

  ContinuousTimeLidarFactor factor(0.5, 1.0, pc, extrinsics, 1.0);
  const double r = factor.residual(state);
  if (std::abs(r) > 1.0e-9) {
    std::fprintf(stderr, "plane residual nonzero for consistent point: %.6g\n", r);
    std::exit(1);
  }
}

void check_plane_residual_distance_offset()
{
  // Translate every position knot in z so the point sits 0.25 m above z=0.
  LidarFactorState state = identity_state();
  for (auto & p : state.position_knots) {
    p.z() = 0.25;
  }
  LidarExtrinsics extrinsics;
  LidarPointCorrespondence pc;
  pc.point_lidar = Eigen::Vector3d(2.0, -1.0, 0.0);
  pc.geometry = LidarFeatureGeometry::kPlane;
  pc.plane << 0.0, 0.0, 1.0, 0.0;
  ContinuousTimeLidarFactor factor(0.5, 1.0, pc, extrinsics, 2.0);
  const double r = factor.residual(state);
  if (std::abs(r - 0.5) > 1.0e-6) {
    std::fprintf(stderr,
      "plane residual offset mismatch: expected 0.5 got %.6g\n", r);
    std::exit(1);
  }
}

void check_body_pose_jacobian_finite_difference_plane()
{
  // Use a perturbed initial state so the analytic body-pose Jacobian is not
  // trivially zero, then check it against finite differences.
  LidarFactorState state = identity_state();
  for (int i = 0; i < N; ++i) {
    state.position_knots[i] = Eigen::Vector3d(0.0, 0.0, 0.3 + 0.05 * i);
    state.rotation_knots[i] =
      Eigen::Quaterniond(Eigen::AngleAxisd(0.1 * i, Eigen::Vector3d::UnitZ()));
  }
  LidarExtrinsics extrinsics;
  LidarPointCorrespondence pc;
  pc.point_lidar = Eigen::Vector3d(2.0, -1.0, 0.0);
  pc.geometry = LidarFeatureGeometry::kPlane;
  pc.plane << 0.0, 0.0, 1.0, -0.2;

  ContinuousTimeLidarFactor factor(0.5, 1.0 / 0.05, pc, extrinsics, 1.0);
  const auto jacobian = factor.body_pose_jacobian(state);
  const double r0 = factor.residual(state);

  const double eps = 1.0e-5;
  for (int axis = 0; axis < 6; ++axis) {
    Eigen::Matrix<double, 6, 1> delta = Eigen::Matrix<double, 6, 1>::Zero();
    delta[axis] = eps;
    const double r_plus = numeric_residual_with_body_pose_offset(factor, state, delta);
    delta[axis] = -eps;
    const double r_minus = numeric_residual_with_body_pose_offset(factor, state, delta);
    const double fd = (r_plus - r_minus) / (2.0 * eps);
    if (std::abs(fd - jacobian[axis]) > 1.0e-3) {
      std::fprintf(stderr,
        "plane body-pose jacobian axis %d mismatch: analytic %.6f fd %.6f (r0=%.6f)\n",
        axis, jacobian[axis], fd, r0);
      std::exit(1);
    }
  }
}

void check_edge_residual_finite()
{
  // Edge feature with the point exactly on the line should produce zero;
  // off-line should produce a positive distance equal to perpendicular gap.
  LidarFactorState state = identity_state();
  LidarExtrinsics extrinsics;
  LidarPointCorrespondence pc;
  pc.point_lidar = Eigen::Vector3d(1.0, 0.25, 0.0);
  pc.geometry = LidarFeatureGeometry::kEdge;
  pc.edge_point = Eigen::Vector3d(1.0, 0.0, 0.0);
  pc.edge_normal = Eigen::Vector3d(1.0, 0.0, 0.0).normalized();
  ContinuousTimeLidarFactor factor(0.5, 1.0, pc, extrinsics, 1.0);
  const double r = factor.residual(state);
  // Distance from (1, 0.25, 0) to line through (1, 0, 0) along x is 0.25 m.
  if (std::abs(r - 0.25) > 1.0e-6) {
    std::fprintf(stderr, "edge residual mismatch: expected 0.25 got %.6f\n", r);
    std::exit(1);
  }
}

}  // namespace

int main()
{
  try {
    check_plane_residual_zero_for_consistent_point();
    check_plane_residual_distance_offset();
    check_body_pose_jacobian_finite_difference_plane();
    check_edge_residual_finite();
  } catch (const std::exception & exception) {
    std::fprintf(stderr, "continuous_time_lidar_factor_probe exception: %s\n", exception.what());
    return 1;
  }
  std::printf("continuous_time_lidar_factor_probe ok\n");
  return 0;
}
