// SPDX-License-Identifier: GPL-3.0-or-later
//
// Uniform B-spline blending matrix helpers ported from the Basalt-style
// CeresSplineHelper used by Coco-LIC (external/Coco-LIC/src/spline/spline_common.h).
// Pure ROS2-native re-implementation: no Sophus, no Ceres, no Boost.

#pragma once

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
