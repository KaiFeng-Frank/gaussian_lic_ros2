// SPDX-License-Identifier: GPL-3.0-or-later
//
// Minimal SO(3) primitives shared by the cumulative B-spline view and the
// continuous-time analytic factors. Mirrors the subset of Sophus used by
// Coco-LIC's CeresSplineHelper::evaluate_lie without adding a Sophus runtime
// dependency.

#pragma once

#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace gaussian_lic_tracking
{
namespace spline
{

inline Eigen::Matrix3d skew(const Eigen::Vector3d & v)
{
  Eigen::Matrix3d m;
  m <<
    0.0, -v.z(), v.y(),
    v.z(), 0.0, -v.x(),
    -v.y(), v.x(), 0.0;
  return m;
}

inline Eigen::Vector3d quaternion_log(Eigen::Quaterniond quaternion)
{
  quaternion.normalize();
  if (quaternion.w() < 0.0) {
    quaternion.coeffs() *= -1.0;
  }
  const Eigen::Vector3d xyz = quaternion.vec();
  const double xyz_norm = xyz.norm();
  if (xyz_norm < 1.0e-12) {
    return 2.0 * xyz;
  }
  const double angle = std::atan2(xyz_norm, quaternion.w());
  return 2.0 * angle * xyz / xyz_norm;
}

inline Eigen::Quaterniond quaternion_exp(const Eigen::Vector3d & tangent)
{
  const double theta = tangent.norm();
  if (theta < 1.0e-12) {
    Eigen::Quaterniond identity_plus(1.0, 0.5 * tangent.x(), 0.5 * tangent.y(), 0.5 * tangent.z());
    return identity_plus.normalized();
  }
  const double half_theta = 0.5 * theta;
  const double sin_half = std::sin(half_theta);
  const Eigen::Vector3d axis = tangent / theta;
  return Eigen::Quaterniond(std::cos(half_theta), sin_half * axis.x(), sin_half * axis.y(), sin_half * axis.z()).normalized();
}

inline Eigen::Matrix3d adjoint(const Eigen::Quaterniond & q)
{
  return q.normalized().toRotationMatrix();
}

inline Eigen::Vector3d lie_bracket(const Eigen::Vector3d & a, const Eigen::Vector3d & b)
{
  return a.cross(b);
}

}  // namespace spline
}  // namespace gaussian_lic_tracking
