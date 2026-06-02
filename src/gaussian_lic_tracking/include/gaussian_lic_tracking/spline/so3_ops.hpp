// SPDX-License-Identifier: GPL-3.0-or-later
//
// Minimal SO(3) primitives shared by the cumulative B-spline view and the
// continuous-time analytic factors. Mirrors the subset of Sophus used by
// Coco-LIC's CeresSplineHelper::evaluate_lie without adding a Sophus runtime
// dependency.
//
// Operations are templated on the scalar type so they compose with Ceres
// AutoDiff (`ceres::Jet<double, N>`) in addition to plain `double`. The
// `double` overloads are preserved for non-Ceres callers via type deduction
// on `Eigen::Quaternion<T>` / `Eigen::Matrix<T, 3, 1>`.

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

template <typename Derived>
auto skew_t(const Eigen::MatrixBase<Derived> & v)
  -> Eigen::Matrix<typename Derived::Scalar, 3, 3>
{
  using T = typename Derived::Scalar;
  Eigen::Matrix<T, 3, 3> m;
  m <<
    T(0.0), -v[2], v[1],
    v[2], T(0.0), -v[0],
    -v[1], v[0], T(0.0);
  return m;
}

template <typename T>
Eigen::Matrix<T, 3, 1> quaternion_log_t(Eigen::Quaternion<T> quaternion)
{
  quaternion.normalize();
  if (quaternion.w() < T(0.0)) {
    quaternion.coeffs() *= T(-1.0);
  }
  const Eigen::Matrix<T, 3, 1> xyz = quaternion.vec();
  const T xyz_norm = xyz.norm();
  if (xyz_norm < T(1.0e-12)) {
    return T(2.0) * xyz;
  }
  using std::atan2;
  const T angle = atan2(xyz_norm, quaternion.w());
  return T(2.0) * angle * xyz / xyz_norm;
}

template <typename T>
Eigen::Quaternion<T> quaternion_exp_t(const Eigen::Matrix<T, 3, 1> & tangent)
{
  const T theta = tangent.norm();
  if (theta < T(1.0e-12)) {
    Eigen::Quaternion<T> identity_plus(
      T(1.0),
      T(0.5) * tangent[0],
      T(0.5) * tangent[1],
      T(0.5) * tangent[2]);
    return identity_plus.normalized();
  }
  using std::cos;
  using std::sin;
  const T half_theta = T(0.5) * theta;
  const T sin_half = sin(half_theta);
  const Eigen::Matrix<T, 3, 1> axis = tangent / theta;
  return Eigen::Quaternion<T>(
    cos(half_theta),
    sin_half * axis[0],
    sin_half * axis[1],
    sin_half * axis[2]).normalized();
}

template <typename T>
Eigen::Matrix<T, 3, 3> adjoint_t(const Eigen::Quaternion<T> & q)
{
  return q.normalized().toRotationMatrix();
}

template <typename Derived1, typename Derived2>
auto lie_bracket_t(
  const Eigen::MatrixBase<Derived1> & a,
  const Eigen::MatrixBase<Derived2> & b)
  -> Eigen::Matrix<typename Derived1::Scalar, 3, 1>
{
  return a.cross(b);
}

// Concrete-double overloads preserved for non-templated callers.
inline Eigen::Vector3d quaternion_log(Eigen::Quaterniond quaternion)
{
  return quaternion_log_t<double>(quaternion);
}

inline Eigen::Quaterniond quaternion_exp(const Eigen::Vector3d & tangent)
{
  return quaternion_exp_t<double>(tangent);
}

inline Eigen::Matrix3d adjoint(const Eigen::Quaterniond & q)
{
  return adjoint_t<double>(q);
}

inline Eigen::Vector3d lie_bracket(const Eigen::Vector3d & a, const Eigen::Vector3d & b)
{
  return a.cross(b);
}

// Right Jacobian of SO(3): exp(phi + delta) ~= exp(phi) * exp(Jr(phi) * delta).
// Closed form (matches Sophus::rightJacobianSO3): Jr = I - ((1-cos t)/t^2) [phi]_x
//   + ((t - sin t)/t^3) [phi]_x^2, with the small-angle Taylor expansion near 0.
// This is the load-bearing primitive for the analytic cumulative-B-spline SO(3)
// knot Jacobians (So3SplineView::EvaluateRpNURBS).
inline Eigen::Matrix3d right_jacobian_so3(const Eigen::Vector3d & phi)
{
  const Eigen::Matrix3d W = skew(phi);
  const double theta2 = phi.squaredNorm();
  if (theta2 < 1.0e-10) {
    return Eigen::Matrix3d::Identity() - 0.5 * W + (1.0 / 6.0) * (W * W);
  }
  const double theta = std::sqrt(theta2);
  const double a = (1.0 - std::cos(theta)) / theta2;
  const double b = (theta - std::sin(theta)) / (theta2 * theta);
  return Eigen::Matrix3d::Identity() - a * W + b * (W * W);
}

// Inverse right Jacobian (matches Sophus::rightJacobianInvSO3):
//   Jr^{-1} = I + 0.5 [phi]_x + (1/t^2 - (1+cos t)/(2 t sin t)) [phi]_x^2.
inline Eigen::Matrix3d right_jacobian_inv_so3(const Eigen::Vector3d & phi)
{
  const Eigen::Matrix3d W = skew(phi);
  const double theta2 = phi.squaredNorm();
  if (theta2 < 1.0e-10) {
    return Eigen::Matrix3d::Identity() + 0.5 * W + (1.0 / 12.0) * (W * W);
  }
  const double theta = std::sqrt(theta2);
  const double c =
    1.0 / theta2 - (1.0 + std::cos(theta)) / (2.0 * theta * std::sin(theta));
  return Eigen::Matrix3d::Identity() + 0.5 * W + c * (W * W);
}

}  // namespace spline
}  // namespace gaussian_lic_tracking
