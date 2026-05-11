// SPDX-License-Identifier: GPL-3.0-or-later
//
// ROS2-native port of Coco-LIC's LoamFeatureFactorNURBS continuous-time LiDAR
// residual (external/Coco-LIC/src/odom/factor/analytic_diff/lidar_feature_factor.h).
//
// The factor constrains the SE(3) spline so that a per-point LiDAR
// observation (already deskewed to its acquisition stamp) lies on the
// associated geometric primitive expressed in the map frame:
//
//   p_I = S_LtoI * p_L + p_LinI                  (LiDAR -> IMU body frame)
//   p_G = R_ItoG(t) * p_I + p_IinG(t)           (spline query)
//   p_M = S_GtoM * p_G + p_GinM                 (world G -> map M)
//   plane: r = n^T p_M + d
//   edge : r = |(p_M - p_geo) cross n_geo|
//
// Residual is scaled by `weight` (information factor).

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gaussian_lic_tracking/spline/ceres_spline_helper.hpp>
#include <gaussian_lic_tracking/spline/so3_ops.hpp>
#include <gaussian_lic_tracking/spline/split_spline_view.hpp>

namespace gaussian_lic_tracking
{
namespace spline
{

enum class LidarFeatureGeometry
{
  kPlane,
  kEdge
};

struct LidarPointCorrespondence
{
  Eigen::Vector3d point_lidar{Eigen::Vector3d::Zero()};
  LidarFeatureGeometry geometry{LidarFeatureGeometry::kPlane};
  // Plane: (nx, ny, nz, d) such that residual = n · p_M + d.
  Eigen::Vector4d plane{Eigen::Vector4d::Zero()};
  // Edge: any point on the edge line + its tangent normal.
  Eigen::Vector3d edge_point{Eigen::Vector3d::Zero()};
  Eigen::Vector3d edge_normal{Eigen::Vector3d::Zero()};
};

struct LidarExtrinsics
{
  Eigen::Quaterniond q_lidar_to_imu{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d p_lidar_in_imu{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond q_world_to_map{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d p_world_in_map{Eigen::Vector3d::Zero()};
};

struct LidarFactorState
{
  std::array<Eigen::Quaterniond, kPositionSplineOrder> rotation_knots;
  std::array<Eigen::Vector3d, kPositionSplineOrder> position_knots;
};

class ContinuousTimeLidarFactor
{
public:
  static constexpr int N = kPositionSplineOrder;
  static constexpr int kResidualDim = 1;

  ContinuousTimeLidarFactor(
    double normalized_time,
    double inv_dt_s,
    const LidarPointCorrespondence & correspondence,
    const LidarExtrinsics & extrinsics,
    double weight = 1.0)
  : u_(normalized_time),
    inv_dt_s_(inv_dt_s),
    correspondence_(correspondence),
    extrinsics_(extrinsics),
    weight_(weight)
  {
  }

  double residual(const LidarFactorState & state) const
  {
    const SplitSplineView<N> view(state.rotation_knots, state.position_knots, u_, inv_dt_s_);
    const Eigen::Quaterniond q_imu_to_world = view.rotation();
    const Eigen::Vector3d p_imu_in_world = view.position_world();

    const Eigen::Vector3d p_imu =
      extrinsics_.q_lidar_to_imu * correspondence_.point_lidar + extrinsics_.p_lidar_in_imu;
    const Eigen::Vector3d p_world = q_imu_to_world * p_imu + p_imu_in_world;
    const Eigen::Vector3d p_map =
      extrinsics_.q_world_to_map * p_world + extrinsics_.p_world_in_map;

    double r = 0.0;
    if (correspondence_.geometry == LidarFeatureGeometry::kPlane) {
      r = correspondence_.plane.head<3>().dot(p_map) + correspondence_.plane[3];
    } else {
      const Eigen::Vector3d dist_vec =
        (p_map - correspondence_.edge_point).cross(correspondence_.edge_normal);
      r = dist_vec.norm();
    }
    return weight_ * r;
  }

  // Linearization: derivative w.r.t. perturbation of body pose at the spline
  // query stamp (rotation tangent + world position). Returns a 1x6 row used
  // by sliding-window optimizers that interpolate the spline pose down to a
  // discrete state.
  Eigen::Matrix<double, 1, 6> body_pose_jacobian(const LidarFactorState & state) const
  {
    const SplitSplineView<N> view(state.rotation_knots, state.position_knots, u_, inv_dt_s_);
    const Eigen::Matrix3d r_imu_to_world = view.rotation().toRotationMatrix();
    const Eigen::Matrix3d r_world_to_map = extrinsics_.q_world_to_map.toRotationMatrix();

    const Eigen::Vector3d p_imu =
      extrinsics_.q_lidar_to_imu * correspondence_.point_lidar + extrinsics_.p_lidar_in_imu;
    const Eigen::Vector3d p_world = r_imu_to_world * p_imu + view.position_world();
    const Eigen::Vector3d p_map = r_world_to_map * p_world + extrinsics_.p_world_in_map;

    Eigen::Vector3d dr_dp_map;
    if (correspondence_.geometry == LidarFeatureGeometry::kPlane) {
      dr_dp_map = correspondence_.plane.head<3>();
    } else {
      const Eigen::Vector3d dist_vec =
        (p_map - correspondence_.edge_point).cross(correspondence_.edge_normal);
      const double norm = dist_vec.norm();
      if (norm < 1.0e-12) {
        return Eigen::Matrix<double, 1, 6>::Zero();
      }
      dr_dp_map = (-dist_vec.transpose() / norm * skew(correspondence_.edge_normal)).transpose();
    }

    Eigen::Matrix<double, 1, 6> jacobian;
    jacobian.head<3>() =
      (dr_dp_map.transpose() * r_world_to_map * (-r_imu_to_world * skew(p_imu))) * weight_;
    jacobian.tail<3>() =
      (dr_dp_map.transpose() * r_world_to_map) * weight_;
    return jacobian;
  }

  double normalized_time() const { return u_; }
  double inv_dt_s() const { return inv_dt_s_; }
  double weight() const { return weight_; }
  const LidarPointCorrespondence & correspondence() const { return correspondence_; }
  const LidarExtrinsics & extrinsics() const { return extrinsics_; }

private:
  double u_{0.0};
  double inv_dt_s_{1.0};
  LidarPointCorrespondence correspondence_{};
  LidarExtrinsics extrinsics_{};
  double weight_{1.0};
};

inline double numeric_residual_with_body_pose_offset(
  const ContinuousTimeLidarFactor & factor,
  const LidarFactorState & state,
  const Eigen::Matrix<double, 6, 1> & delta)
{
  LidarFactorState perturbed = state;
  const Eigen::Vector3d rot_delta = delta.head<3>();
  // Apply rotation perturbation in body tangent at the *evaluation stamp*: we
  // approximate by perturbing every rotation knot by the same right-tangent
  // delta. For verifying body-pose jacobians this matches small-angle behavior
  // because the cumulative-spline value commutes with a common right factor
  // applied to all knots.
  for (int i = 0; i < ContinuousTimeLidarFactor::N; ++i) {
    perturbed.rotation_knots[i] =
      (state.rotation_knots[i] * quaternion_exp(rot_delta)).normalized();
    perturbed.position_knots[i] = state.position_knots[i] + delta.tail<3>();
  }
  return factor.residual(perturbed);
}

}  // namespace spline
}  // namespace gaussian_lic_tracking
