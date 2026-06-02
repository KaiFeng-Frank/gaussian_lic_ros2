// SPDX-License-Identifier: GPL-3.0-or-later
//
// Numeric-vs-analytic validation of the tight photometric factor's analytic
// Jacobian (ContinuousTimePhotometricFactor::evaluate_with_knot_jacobian).
// A synthetic linear-gradient image makes the image gradient exact, so this
// validates the full pose chain: image-gradient * pinhole * d_pC/d_pose *
// spline-knot Jacobian, w.r.t. every rotation and position control knot.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gaussian_lic_tracking/spline/continuous_time_photometric_factor.hpp>
#include <gaussian_lic_tracking/spline/so3_ops.hpp>

using gaussian_lic_tracking::spline::CameraExtrinsics;
using gaussian_lic_tracking::spline::CameraIntrinsics;
using gaussian_lic_tracking::spline::ContinuousTimePhotometricFactor;
using gaussian_lic_tracking::spline::PhotometricFactorState;
using gaussian_lic_tracking::spline::quaternion_exp;

int main()
{
  constexpr int N = ContinuousTimePhotometricFactor::N;

  PhotometricFactorState st;
  st.rotation_knots[0] = quaternion_exp(Eigen::Vector3d(0.02, -0.01, 0.03));
  st.rotation_knots[1] = quaternion_exp(Eigen::Vector3d(0.01, 0.02, -0.01));
  st.rotation_knots[2] = quaternion_exp(Eigen::Vector3d(-0.02, 0.01, 0.02));
  st.rotation_knots[3] = quaternion_exp(Eigen::Vector3d(0.01, -0.02, 0.01));
  st.position_knots[0] = Eigen::Vector3d(0.00, 0.00, 0.00);
  st.position_knots[1] = Eigen::Vector3d(0.05, -0.03, 0.02);
  st.position_knots[2] = Eigen::Vector3d(-0.02, 0.04, -0.01);
  st.position_knots[3] = Eigen::Vector3d(0.03, 0.01, 0.02);

  const double u = 0.4;
  const double inv_dt = 1.0;
  const CameraIntrinsics intrinsics{400.0, 400.0, 320.0, 240.0};
  CameraExtrinsics extrinsics;
  extrinsics.q_camera_to_imu = quaternion_exp(Eigen::Vector3d(0.01, -0.02, 0.0));
  extrinsics.p_camera_in_imu = Eigen::Vector3d(0.05, 0.0, 0.02);
  const Eigen::Vector3d point_world(0.5, 0.3, 5.0);

  const int half = 2;
  const std::vector<double> reference(static_cast<std::size_t>(2 * half * 2 * half), 0.0);
  const auto sampler = [](double uu, double vv, bool & valid) {
    valid = true;
    return 0.7 * uu - 0.4 * vv + 10.0;  // linear -> exact gradient (0.7, -0.4)
  };

  ContinuousTimePhotometricFactor factor(
    u, inv_dt, point_world, half, reference, sampler, intrinsics, extrinsics, 1.0);

  ContinuousTimePhotometricFactor::PhotometricKnotJacobian jac;
  if (!factor.evaluate_with_knot_jacobian(st, jac) || !jac.valid) {
    std::fprintf(stderr, "photometric jacobian evaluate returned invalid\n");
    return 1;
  }
  const int m_res = static_cast<int>(jac.residuals.size());

  const double eps = 1.0e-6;
  double max_err = 0.0;
  double max_ana = 0.0;

  // Rotation knots (right perturbation).
  for (int i = 0; i < N; ++i) {
    for (int k = 0; k < 3; ++k) {
      Eigen::Vector3d e = Eigen::Vector3d::Zero();
      e[k] = eps;
      PhotometricFactorState sp = st;
      sp.rotation_knots[i] = (st.rotation_knots[i] * quaternion_exp(e)).normalized();
      bool valid = false;
      const std::vector<double> rp = factor.residual(sp, valid);
      for (int r = 0; r < m_res; ++r) {
        const double num = (rp[static_cast<std::size_t>(r)] - jac.residuals[static_cast<std::size_t>(r)]) / eps;
        const double ana = jac.d_res_d_rot_knot[i](r, k);
        max_err = std::max(max_err, std::abs(num - ana));
        max_ana = std::max(max_ana, std::abs(ana));
      }
    }
  }
  // Position knots.
  for (int i = 0; i < N; ++i) {
    for (int k = 0; k < 3; ++k) {
      PhotometricFactorState sp = st;
      sp.position_knots[i][k] += eps;
      bool valid = false;
      const std::vector<double> rp = factor.residual(sp, valid);
      for (int r = 0; r < m_res; ++r) {
        const double num = (rp[static_cast<std::size_t>(r)] - jac.residuals[static_cast<std::size_t>(r)]) / eps;
        const double ana = jac.d_res_d_pos_knot[i](r, k);
        max_err = std::max(max_err, std::abs(num - ana));
        max_ana = std::max(max_ana, std::abs(ana));
      }
    }
  }

  std::printf(
    "continuous_time_photometric_jacobian_probe: max_abs_err=%.3e max_ana=%.3e rel=%.3e\n",
    max_err, max_ana, max_err / (1.0 + max_ana));
  if (max_err > 1.0e-3 * (1.0 + max_ana)) {
    std::fprintf(stderr, "photometric analytic Jacobian disagrees with numeric\n");
    return 1;
  }
  std::printf("continuous_time_photometric_jacobian_probe ok\n");
  return 0;
}
