// SPDX-License-Identifier: GPL-3.0-or-later
//
// Uniform B-spline blending matrix helpers ported from the Basalt-style
// CeresSplineHelper used by Coco-LIC (external/Coco-LIC/src/spline/spline_common.h).
// Pure ROS2-native re-implementation: no Sophus, no Ceres, no Boost.

#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <Eigen/Core>

namespace gaussian_lic_tracking
{
namespace spline
{

constexpr int kPositionSplineOrder = 4;
constexpr int kBiasSplineOrder = 3;

constexpr std::uint64_t binomial_coefficient(std::uint64_t n, std::uint64_t k)
{
  if (k > n) {
    return 0;
  }
  std::uint64_t r = 1;
  for (std::uint64_t d = 1; d <= k; ++d) {
    r *= n--;
    r /= d;
  }
  return r;
}

template <int N, typename Scalar = double, bool Cumulative = false>
Eigen::Matrix<Scalar, N, N> compute_blending_matrix()
{
  Eigen::Matrix<double, N, N> matrix;
  matrix.setZero();

  for (int i = 0; i < N; ++i) {
    for (int j = 0; j < N; ++j) {
      double sum = 0.0;
      for (int s = j; s < N; ++s) {
        const double sign = ((s - j) % 2 == 0) ? 1.0 : -1.0;
        sum += sign *
               static_cast<double>(binomial_coefficient(N, s - j)) *
               std::pow(N - s - 1.0, N - 1.0 - i);
      }
      matrix(j, i) = static_cast<double>(binomial_coefficient(N - 1, N - 1 - i)) * sum;
    }
  }

  if (Cumulative) {
    for (int i = 0; i < N; ++i) {
      for (int j = i + 1; j < N; ++j) {
        matrix.row(i) += matrix.row(j);
      }
    }
  }

  std::uint64_t factorial = 1;
  for (int i = 2; i < N; ++i) {
    factorial *= static_cast<std::uint64_t>(i);
  }

  return (matrix / static_cast<double>(factorial)).template cast<Scalar>();
}

// Non-uniform cubic (N == kPositionSplineOrder == 4) B-spline blending matrix.
//
// Ported VERBATIM from Coco-LIC's non-uniform path
// (external/Coco-LIC/src/spline/se3_spline.h InitBlendMat, lines 620-662).
// Upstream builds the per-segment matrix from the ACTUAL knot times of the
// 6-knot window [ti_minus_2 .. ti_plus_3] surrounding the active cubic segment
// [ti, ti_plus_1). Here `knot_times_s` carries those six knot times already
// converted to seconds (the caller multiplies the int64 ns stamps by 1e-9),
// with the index mapping matching upstream knts[1..6]:
//   knot_times_s[0] = ti_minus_2   (upstream knts[1])
//   knot_times_s[1] = ti_minus_1   (upstream knts[2])
//   knot_times_s[2] = ti           (upstream knts[3])
//   knot_times_s[3] = ti_plus_1    (upstream knts[4])
//   knot_times_s[4] = ti_plus_2    (upstream knts[5])
//   knot_times_s[5] = ti_plus_3    (upstream knts[6])
//
// On UNIFORM knot spacing this reproduces compute_blending_matrix<4,double,false>()
// (non-cumulative) / <4,double,true>() (cumulative) to <= 4.44e-16, so the
// gated non-uniform path is exactly equivalent to the uniform path when knots
// stay at fixed dt (increment 1). Cubic-only by design (kPositionSplineOrder=4).
inline Eigen::Matrix4d compute_blending_matrix_nonuniform_cubic(
  const std::array<double, 6> & knot_times_s, bool cumulative = false)
{
  Eigen::Matrix4d blending_mat = Eigen::Matrix4d::Zero();

  const double ti_minus_2 = knot_times_s[0];
  const double ti_minus_1 = knot_times_s[1];
  const double ti = knot_times_s[2];
  const double ti_plus_1 = knot_times_s[3];
  const double ti_plus_2 = knot_times_s[4];
  const double ti_plus_3 = knot_times_s[5];

  blending_mat(0, 0) = (ti_plus_1 - ti) * (ti_plus_1 - ti) / ((ti_plus_1 - ti_minus_1) * (ti_plus_1 - ti_minus_2));
  blending_mat(0, 2) = (ti - ti_minus_1) * (ti - ti_minus_1) / ((ti_plus_2 - ti_minus_1) * (ti_plus_1 - ti_minus_1));
  blending_mat(1, 2) = 3 * (ti_plus_1 - ti) * (ti - ti_minus_1) / ((ti_plus_2 - ti_minus_1) * (ti_plus_1 - ti_minus_1));
  blending_mat(2, 2) = 3 * (ti_plus_1 - ti) * (ti_plus_1 - ti) / ((ti_plus_2 - ti_minus_1) * (ti_plus_1 - ti_minus_1));
  blending_mat(3, 3) = (ti_plus_1 - ti) * (ti_plus_1 - ti) / ((ti_plus_3 - ti) * (ti_plus_2 - ti));
  blending_mat(0, 1) = 1 - blending_mat(0, 0) - blending_mat(0, 2);
  blending_mat(0, 3) = 0;
  blending_mat(1, 0) = -3 * blending_mat(0, 0);
  blending_mat(1, 1) = 3 * blending_mat(0, 0) - blending_mat(1, 2);
  blending_mat(1, 3) = 0;
  blending_mat(2, 0) = 3 * blending_mat(0, 0);
  blending_mat(2, 1) = -3 * blending_mat(0, 0) - blending_mat(2, 2);
  blending_mat(2, 3) = 0;
  blending_mat(3, 0) = -blending_mat(0, 0);
  blending_mat(3, 2) = -blending_mat(2, 2) / 3 - blending_mat(3, 3) - (ti_plus_1 - ti) * (ti_plus_1 - ti) / ((ti_plus_2 - ti) * (ti_plus_2 - ti_minus_1));
  blending_mat(3, 1) = blending_mat(0, 0) - blending_mat(3, 2) - blending_mat(3, 3);
  blending_mat = (blending_mat.transpose()).eval();

  if (cumulative) {
    Eigen::Matrix4d cumu_blending_mat = blending_mat;
    for (int i = 0; i < 4; ++i) {
      for (int j = i + 1; j < 4; ++j) {
        cumu_blending_mat.row(i) += cumu_blending_mat.row(j);
      }
    }
    return cumu_blending_mat;
  }
  return blending_mat;
}

template <int N, typename Scalar = double>
Eigen::Matrix<Scalar, N, N> compute_base_coefficients()
{
  Eigen::Matrix<double, N, N> base_coefficients;
  base_coefficients.setZero();
  base_coefficients.row(0).setOnes();

  constexpr int DEG = N - 1;
  int order = DEG;
  for (int n = 1; n < N; ++n) {
    for (int i = DEG - order; i < N; ++i) {
      base_coefficients(n, i) =
        static_cast<double>(order - DEG + i) * base_coefficients(n - 1, i);
    }
    --order;
  }
  return base_coefficients.template cast<Scalar>();
}

}  // namespace spline
}  // namespace gaussian_lic_tracking
