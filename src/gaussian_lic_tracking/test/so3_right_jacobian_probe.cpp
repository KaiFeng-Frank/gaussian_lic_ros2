// SPDX-License-Identifier: GPL-3.0-or-later
//
// Validates the SO(3) right Jacobian and its inverse (so3_ops.hpp), the
// load-bearing primitive for the analytic cumulative-B-spline knot Jacobians
// needed by the tight photometric factor. Two checks per test rotation:
//   (a) Jr(phi) * Jr_inv(phi) == I
//   (b) defining identity: log(exp(phi)^-1 * exp(phi + delta)) ~= Jr(phi) * delta

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gaussian_lic_tracking/spline/so3_ops.hpp>

using gaussian_lic_tracking::spline::quaternion_exp;
using gaussian_lic_tracking::spline::quaternion_log;
using gaussian_lic_tracking::spline::right_jacobian_inv_so3;
using gaussian_lic_tracking::spline::right_jacobian_so3;

int main()
{
  const std::vector<Eigen::Vector3d> phis = {
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Vector3d(0.3, -0.2, 0.5),
    Eigen::Vector3d(1.2, 0.7, -0.9),
    Eigen::Vector3d(2.5, -1.5, 0.4),
    Eigen::Vector3d(1.0e-6, 2.0e-6, -1.0e-6),
  };

  for (const auto & phi : phis) {
    const Eigen::Matrix3d jr = right_jacobian_so3(phi);
    const Eigen::Matrix3d jr_inv = right_jacobian_inv_so3(phi);

    // (a) Jr * Jr_inv == I
    const double inv_err =
      (jr * jr_inv - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff();
    if (inv_err > 1.0e-9) {
      std::fprintf(stderr,
        "Jr*Jr_inv != I: err=%.3e phi=(%.3f,%.3f,%.3f)\n",
        inv_err, phi.x(), phi.y(), phi.z());
      return 1;
    }

    // (b) log(exp(phi)^-1 * exp(phi+delta)) ~= Jr(phi) * delta
    const double eps = 1.0e-6;
    for (int k = 0; k < 3; ++k) {
      Eigen::Vector3d delta = Eigen::Vector3d::Zero();
      delta[k] = eps;
      const Eigen::Quaterniond q0 = quaternion_exp(phi);
      const Eigen::Quaterniond q1 = quaternion_exp(Eigen::Vector3d(phi + delta));
      const Eigen::Vector3d lhs = quaternion_log((q0.inverse() * q1).normalized());
      const Eigen::Vector3d rhs = jr * delta;
      const double e = (lhs - rhs).norm() / eps;
      if (e > 1.0e-4) {
        std::fprintf(stderr,
          "right-jac identity mismatch: e=%.3e phi=(%.3f,%.3f,%.3f) axis=%d\n",
          e, phi.x(), phi.y(), phi.z(), k);
        return 1;
      }
    }
  }

  std::printf("so3_right_jacobian_probe ok\n");
  return 0;
}
