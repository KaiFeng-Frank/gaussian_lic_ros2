// SPDX-License-Identifier: GPL-3.0-or-later
//
// Numeric-vs-analytic validation of the cumulative-B-spline SO(3) knot Jacobian
// (SplitSplineView::rotation_with_knot_jacobian, ported from Coco-LIC
// So3SplineView::EvaluateRpNURBS). For each knot i and axis k it perturbs the
// knot on the right (knot_i -> knot_i * exp(eps*e_k)), re-evaluates R(u), and
// compares the finite-difference column to d_val_d_knot[i].col(k) under both
// the right and left R(u)-perturbation conventions (reporting which matches).

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gaussian_lic_tracking/spline/so3_ops.hpp>
#include <gaussian_lic_tracking/spline/split_spline_view.hpp>

using gaussian_lic_tracking::spline::quaternion_exp;
using gaussian_lic_tracking::spline::quaternion_log;
using gaussian_lic_tracking::spline::SplitSplineView;

int main()
{
  constexpr int N = SplitSplineView<>::N;

  std::array<Eigen::Quaterniond, N> rot_knots;
  rot_knots[0] = quaternion_exp(Eigen::Vector3d(0.10, -0.20, 0.30));
  rot_knots[1] = quaternion_exp(Eigen::Vector3d(0.00, 0.40, -0.10));
  rot_knots[2] = quaternion_exp(Eigen::Vector3d(-0.30, 0.10, 0.20));
  rot_knots[3] = quaternion_exp(Eigen::Vector3d(0.20, 0.20, -0.40));
  std::array<Eigen::Vector3d, N> pos_knots;
  for (int i = 0; i < N; ++i) {
    pos_knots[i] = Eigen::Vector3d::Zero();
  }

  const double u = 0.4;
  const double inv_dt = 1.0;
  SplitSplineView<> view(rot_knots, pos_knots, u, inv_dt);

  // Consistency: the Jacobian method's R(u) must match rotation().
  const Eigen::Quaterniond r_nom = view.rotation();
  SplitSplineView<>::So3KnotJacobian jac;
  const Eigen::Quaterniond r_jac = view.rotation_with_knot_jacobian(jac);
  const double consist =
    quaternion_log((r_nom.inverse() * r_jac).normalized()).norm();
  if (consist > 1.0e-9) {
    std::fprintf(stderr, "rotation_with_knot_jacobian disagrees with rotation(): %.3e\n", consist);
    return 1;
  }

  const double eps = 1.0e-6;
  double max_right = 0.0;
  double max_left = 0.0;
  for (int i = 0; i < N; ++i) {
    for (int k = 0; k < 3; ++k) {
      Eigen::Vector3d e = Eigen::Vector3d::Zero();
      e[k] = eps;
      std::array<Eigen::Quaterniond, N> kp = rot_knots;
      kp[i] = (rot_knots[i] * quaternion_exp(e)).normalized();
      SplitSplineView<> vp(kp, pos_knots, u, inv_dt);
      const Eigen::Quaterniond rp = vp.rotation();
      const Eigen::Vector3d num_right =
        quaternion_log((r_nom.inverse() * rp).normalized()) / eps;
      const Eigen::Vector3d num_left =
        quaternion_log((rp * r_nom.inverse()).normalized()) / eps;
      const Eigen::Vector3d ana = jac.d_val_d_knot[i].col(k);
      max_right = std::max(max_right, (num_right - ana).norm());
      max_left = std::max(max_left, (num_left - ana).norm());
    }
  }

  std::printf(
    "so3_knot_jacobian_probe: max_right_err=%.3e max_left_err=%.3e\n",
    max_right, max_left);
  const double best = std::min(max_right, max_left);
  if (best > 1.0e-3) {
    std::fprintf(stderr,
      "SO(3) knot Jacobian matches NEITHER convention (right=%.3e left=%.3e)\n",
      max_right, max_left);
    return 1;
  }

  // --- Position knot Jacobian (linear: d_p/d_knot_i = coeff[i] * I_3) ---
  std::array<Eigen::Vector3d, N> pk;
  pk[0] = Eigen::Vector3d(0.5, -1.0, 2.0);
  pk[1] = Eigen::Vector3d(1.0, 0.5, -0.5);
  pk[2] = Eigen::Vector3d(-0.5, 2.0, 1.0);
  pk[3] = Eigen::Vector3d(2.0, -0.5, 0.5);
  SplitSplineView<> pview(rot_knots, pk, u, inv_dt);
  SplitSplineView<>::R3KnotJacobian pjac;
  const Eigen::Vector3d p_nom = pview.position_with_knot_jacobian(pjac);
  if ((p_nom - pview.position_world()).norm() > 1.0e-9) {
    std::fprintf(stderr, "position_with_knot_jacobian disagrees with position_world()\n");
    return 1;
  }
  double max_pos_err = 0.0;
  for (int i = 0; i < N; ++i) {
    for (int k = 0; k < 3; ++k) {
      std::array<Eigen::Vector3d, N> pkp = pk;
      pkp[i][k] += eps;
      SplitSplineView<> vp2(rot_knots, pkp, u, inv_dt);
      const Eigen::Vector3d num = (vp2.position_world() - p_nom) / eps;
      Eigen::Vector3d ana = Eigen::Vector3d::Zero();
      ana[k] = pjac.d_val_d_knot[i];
      max_pos_err = std::max(max_pos_err, (num - ana).norm());
    }
  }
  if (max_pos_err > 1.0e-6) {
    std::fprintf(stderr, "position knot Jacobian wrong: %.3e\n", max_pos_err);
    return 1;
  }

  std::printf(
    "so3_knot_jacobian_probe ok (SO3 convention=%s, pos_err=%.3e)\n",
    max_right < max_left ? "right-perturbation of R(u)" : "left-perturbation of R(u)",
    max_pos_err);
  return 0;
}
