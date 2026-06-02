// SPDX-License-Identifier: GPL-3.0-or-later
//
// Verifies the ROS2-native cumulative B-spline helper matches finite-difference
// time derivatives and reproduces analytic ground truth for canonical inputs.
//
// This is the foundation probe for the continuous-time tracker port. The
// downstream IMU/LOAM/photometric NURBS factors all rely on
// `SplitSplineView::evaluate()` producing the correct rotation, body-frame
// angular velocity, world-frame velocity, and world-frame acceleration.

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gaussian_lic_tracking/spline/ceres_spline_helper.hpp>
#include <gaussian_lic_tracking/spline/split_spline_view.hpp>
#include <gaussian_lic_tracking/spline/spline_common.hpp>

using gaussian_lic_tracking::spline::CubicSplineHelper;
using gaussian_lic_tracking::spline::SplitSplineView;
using gaussian_lic_tracking::spline::compute_blending_matrix;
using gaussian_lic_tracking::spline::compute_blending_matrix_nonuniform_cubic;

namespace
{

bool nearly_equal(double a, double b, double tol)
{
  return std::abs(a - b) <= tol;
}

bool nearly_equal_vec(const Eigen::Vector3d & a, const Eigen::Vector3d & b, double tol)
{
  return (a - b).cwiseAbs().maxCoeff() <= tol;
}

void check_uniform_cubic_blending_matrix()
{
  // The uniform cubic B-spline blending matrix has row=basis_index, col=power
  // (consistent with `coeff = M * [1, u, u^2, u^3]^T` yielding the four basis
  // function values directly). Row 0 is B_0(u) = (1 - 3u + 3u^2 - u^3)/6, etc.
  const Eigen::Matrix4d expected = (Eigen::Matrix4d() <<
    1.0, -3.0,  3.0, -1.0,
    4.0,  0.0, -6.0,  3.0,
    1.0,  3.0,  3.0, -3.0,
    0.0,  0.0,  0.0,  1.0).finished() / 6.0;
  const Eigen::Matrix4d actual = compute_blending_matrix<4, double, false>();
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      if (!nearly_equal(actual(r, c), expected(r, c), 1.0e-12)) {
        std::fprintf(stderr,
          "blending matrix mismatch at (%d, %d): expected %.6f got %.6f\n",
          r, c, expected(r, c), actual(r, c));
        std::exit(1);
      }
    }
  }
}

void check_nonuniform_blending_matrix_reduces_to_uniform()
{
  // Increment-1 correctness anchor: the non-uniform cubic blending matrix
  // (ported from Coco-LIC se3_spline.h InitBlendMat), when fed UNIFORM knot
  // spacing, must reproduce the uniform compute_blending_matrix<4>() to ~1e-9.
  // This proves the non-uniform machinery is correct, so the gated on-flag path
  // (with uniform knots) is exactly equivalent to the off-flag uniform path.
  const Eigen::Matrix4d uniform_noncumu = compute_blending_matrix<4, double, false>();
  const Eigen::Matrix4d uniform_cumu = compute_blending_matrix<4, double, true>();
  const double dts[] = {0.05, 0.1, 0.025, 1.0};
  for (double dt : dts) {
    const std::array<double, 6> knot_times_s = {
      0.0, dt, 2.0 * dt, 3.0 * dt, 4.0 * dt, 5.0 * dt};
    const double err_noncumu =
      (compute_blending_matrix_nonuniform_cubic(knot_times_s, false) - uniform_noncumu)
        .cwiseAbs()
        .maxCoeff();
    const double err_cumu =
      (compute_blending_matrix_nonuniform_cubic(knot_times_s, true) - uniform_cumu)
        .cwiseAbs()
        .maxCoeff();
    if (err_noncumu > 1.0e-9) {
      std::fprintf(stderr,
        "non-uniform blending matrix (non-cumulative) does not reduce to uniform "
        "at dt=%.4f: max abs diff %.3e > 1e-9\n",
        dt, err_noncumu);
      std::exit(1);
    }
    if (err_cumu > 1.0e-9) {
      std::fprintf(stderr,
        "non-uniform blending matrix (cumulative) does not reduce to uniform "
        "at dt=%.4f: max abs diff %.3e > 1e-9\n",
        dt, err_cumu);
      std::exit(1);
    }
  }
}

void check_position_spline_constant_velocity()
{
  // Four collinear knots: positions 0, 1, 2, 3 with knot spacing dt=0.1 s.
  // Expect linear interpolation at all u: position 1 + u, velocity 10 m/s,
  // acceleration 0 m/s^2.
  const double dt = 0.1;
  const double inv_dt = 1.0 / dt;
  std::array<Eigen::Vector3d, 4> knots = {
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Vector3d(1.0, 0.0, 0.0),
    Eigen::Vector3d(2.0, 0.0, 0.0),
    Eigen::Vector3d(3.0, 0.0, 0.0)
  };
  for (double u : {0.0, 0.25, 0.5, 0.75, 0.999999}) {
    const Eigen::Vector3d p = CubicSplineHelper::evaluate_rd<3, 0>(knots, u, inv_dt);
    const Eigen::Vector3d v = CubicSplineHelper::evaluate_rd<3, 1>(knots, u, inv_dt);
    const Eigen::Vector3d a = CubicSplineHelper::evaluate_rd<3, 2>(knots, u, inv_dt);
    const Eigen::Vector3d expected_p(1.0 + u, 0.0, 0.0);
    const Eigen::Vector3d expected_v(10.0, 0.0, 0.0);
    if (!nearly_equal_vec(p, expected_p, 1.0e-9)) {
      std::fprintf(stderr,
        "position spline value mismatch at u=%.3f: got (%.6f, %.6f, %.6f)\n",
        u, p.x(), p.y(), p.z());
      std::exit(1);
    }
    if (!nearly_equal_vec(v, expected_v, 1.0e-9)) {
      std::fprintf(stderr,
        "position spline velocity mismatch at u=%.3f: got (%.6f, %.6f, %.6f)\n",
        u, v.x(), v.y(), v.z());
      std::exit(1);
    }
    if (!nearly_equal_vec(a, Eigen::Vector3d::Zero(), 1.0e-7)) {
      std::fprintf(stderr,
        "position spline acceleration mismatch at u=%.3f: got (%.6f, %.6f, %.6f)\n",
        u, a.x(), a.y(), a.z());
      std::exit(1);
    }
  }
}

void check_position_spline_finite_difference()
{
  // Cubic translation knots p_i = i + 0.5 * i^2; analytic continuous motion
  // matches f(t) = a + b * t + c * t^2. Verify spline velocity/acceleration
  // outputs against finite differences of the value evaluator.
  const double dt = 0.05;
  const double inv_dt = 1.0 / dt;
  std::array<Eigen::Vector3d, 4> knots;
  for (int i = 0; i < 4; ++i) {
    const double pos = i + 0.5 * static_cast<double>(i * i);
    knots[i] = Eigen::Vector3d(pos, 2.0 * pos, -pos);
  }
  const double eps = 1.0e-6;
  for (double u : {0.1, 0.4, 0.7}) {
    const double u_minus = std::max(0.0, u - eps);
    const double u_plus = std::min(1.0, u + eps);
    const auto p_minus = CubicSplineHelper::evaluate_rd<3, 0>(knots, u_minus, inv_dt);
    const auto p_plus = CubicSplineHelper::evaluate_rd<3, 0>(knots, u_plus, inv_dt);
    const Eigen::Vector3d v_fd = (p_plus - p_minus) / ((u_plus - u_minus) * dt);
    const auto v = CubicSplineHelper::evaluate_rd<3, 1>(knots, u, inv_dt);
    if (!nearly_equal_vec(v, v_fd, 1.0e-3)) {
      std::fprintf(stderr,
        "position spline velocity finite-diff mismatch at u=%.2f: spline=(%.6f, %.6f, %.6f) fd=(%.6f, %.6f, %.6f)\n",
        u, v.x(), v.y(), v.z(), v_fd.x(), v_fd.y(), v_fd.z());
      std::exit(1);
    }

    const auto v_minus = CubicSplineHelper::evaluate_rd<3, 1>(knots, u_minus, inv_dt);
    const auto v_plus = CubicSplineHelper::evaluate_rd<3, 1>(knots, u_plus, inv_dt);
    const Eigen::Vector3d a_fd = (v_plus - v_minus) / ((u_plus - u_minus) * dt);
    const auto a = CubicSplineHelper::evaluate_rd<3, 2>(knots, u, inv_dt);
    if (!nearly_equal_vec(a, a_fd, 1.0e-2)) {
      std::fprintf(stderr,
        "position spline acceleration finite-diff mismatch at u=%.2f: spline=(%.6f, %.6f, %.6f) fd=(%.6f, %.6f, %.6f)\n",
        u, a.x(), a.y(), a.z(), a_fd.x(), a_fd.y(), a_fd.z());
      std::exit(1);
    }
  }
}

void check_so3_spline_constant_omega()
{
  // Knots represent constant body-frame angular velocity ω = 1 rad/s about z
  // with knot spacing dt = 0.1 s. Expect body-frame angular velocity to
  // recover ω at every internal u.
  const double dt = 0.1;
  const double inv_dt = 1.0 / dt;
  const Eigen::Vector3d omega(0.0, 0.0, 1.0);
  std::array<Eigen::Quaterniond, 4> knots;
  for (int i = 0; i < 4; ++i) {
    const double angle = i * dt;
    knots[i] = Eigen::Quaterniond(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()));
  }
  for (double u : {0.0, 0.25, 0.5, 0.75, 0.999999}) {
    Eigen::Quaterniond q;
    Eigen::Vector3d omega_eval;
    Eigen::Vector3d alpha_eval;
    CubicSplineHelper::evaluate_lie_so3(knots, u, inv_dt, &q, &omega_eval, &alpha_eval);
    if (!nearly_equal_vec(omega_eval, omega, 1.0e-6)) {
      std::fprintf(stderr,
        "SO(3) spline omega mismatch at u=%.3f: got (%.6f, %.6f, %.6f)\n",
        u, omega_eval.x(), omega_eval.y(), omega_eval.z());
      std::exit(1);
    }
    if (!nearly_equal_vec(alpha_eval, Eigen::Vector3d::Zero(), 1.0e-5)) {
      std::fprintf(stderr,
        "SO(3) spline alpha mismatch at u=%.3f: got (%.6f, %.6f, %.6f)\n",
        u, alpha_eval.x(), alpha_eval.y(), alpha_eval.z());
      std::exit(1);
    }
    // Verify rotation lies on the constant-rate trajectory at u=0 and u=1.
    const double t = (1 + u) * dt;  // segment between knots[1] and knots[2]
    const Eigen::Quaterniond expected(Eigen::AngleAxisd(t, Eigen::Vector3d::UnitZ()));
    const Eigen::Quaterniond delta = expected.inverse() * q;
    const double sign_correction = (delta.w() < 0.0) ? -1.0 : 1.0;
    if (std::abs(sign_correction * delta.w() - 1.0) > 1.0e-6 ||
      delta.vec().cwiseAbs().maxCoeff() > 1.0e-6)
    {
      std::fprintf(stderr,
        "SO(3) spline rotation mismatch at u=%.3f t=%.6f: q=(%.6f, %.6f, %.6f, %.6f) expected=(%.6f, %.6f, %.6f, %.6f)\n",
        u, t, q.w(), q.x(), q.y(), q.z(),
        expected.w(), expected.x(), expected.y(), expected.z());
      std::exit(1);
    }
  }
}

void check_split_view_basic()
{
  // Combined SE(3) trajectory: position translates at 5 m/s along x, rotation
  // tumbles at 1 rad/s about z. Expect SplitSplineView::evaluate to return
  // matching world-frame velocity / body-frame omega at u=0.5.
  const double dt = 0.05;
  const double inv_dt = 1.0 / dt;
  std::array<Eigen::Vector3d, 4> pos_knots;
  std::array<Eigen::Quaterniond, 4> rot_knots;
  for (int i = 0; i < 4; ++i) {
    pos_knots[i] = Eigen::Vector3d(5.0 * i * dt, 0.0, 0.0);
    rot_knots[i] = Eigen::Quaterniond(Eigen::AngleAxisd(1.0 * i * dt, Eigen::Vector3d::UnitZ()));
  }
  SplitSplineView<4> view(rot_knots, pos_knots, 0.5, inv_dt);
  const auto state = view.evaluate(true);
  if (!nearly_equal_vec(state.v_w_b, Eigen::Vector3d(5.0, 0.0, 0.0), 1.0e-8)) {
    std::fprintf(stderr,
      "split view world velocity mismatch: got (%.6f, %.6f, %.6f)\n",
      state.v_w_b.x(), state.v_w_b.y(), state.v_w_b.z());
    std::exit(1);
  }
  if (!nearly_equal_vec(state.omega_b, Eigen::Vector3d(0.0, 0.0, 1.0), 1.0e-6)) {
    std::fprintf(stderr,
      "split view body omega mismatch: got (%.6f, %.6f, %.6f)\n",
      state.omega_b.x(), state.omega_b.y(), state.omega_b.z());
    std::exit(1);
  }
  if (!nearly_equal_vec(state.a_w_b, Eigen::Vector3d::Zero(), 1.0e-5)) {
    std::fprintf(stderr,
      "split view world acceleration mismatch: got (%.6f, %.6f, %.6f)\n",
      state.a_w_b.x(), state.a_w_b.y(), state.a_w_b.z());
    std::exit(1);
  }
}

}  // namespace

int main()
{
  try {
    check_uniform_cubic_blending_matrix();
    check_nonuniform_blending_matrix_reduces_to_uniform();
    check_position_spline_constant_velocity();
    check_position_spline_finite_difference();
    check_so3_spline_constant_omega();
    check_split_view_basic();
  } catch (const std::exception & exception) {
    std::fprintf(stderr, "ceres_spline_helper_probe exception: %s\n", exception.what());
    return 1;
  }
  std::printf("ceres_spline_helper_probe ok\n");
  return 0;
}
