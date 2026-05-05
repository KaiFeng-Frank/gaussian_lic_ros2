// SPDX-License-Identifier: GPL-3.0-or-later

#include <gaussian_lic_tracking/visual_factor.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <Eigen/Cholesky>

namespace gaussian_lic_tracking
{
namespace
{
double parabolic_subpixel_offset(
  const double minus_cost,
  const double center_cost,
  const double plus_cost)
{
  const double denominator = minus_cost - 2.0 * center_cost + plus_cost;
  if (std::abs(denominator) < 1.0e-12) {
    return 0.0;
  }
  const double offset = 0.5 * (minus_cost - plus_cost) / denominator;
  return std::clamp(offset, -0.5, 0.5);
}
}  // namespace

VisualFactor::VisualFactor(const size_t max_pixels)
{
  set_max_pixels(max_pixels);
}

void VisualFactor::set_max_pixels(const size_t max_pixels)
{
  if (max_pixels == 0U) {
    throw std::runtime_error("visual factor max_pixels must be positive");
  }
  max_pixels_ = max_pixels;
}

VisualResidual VisualFactor::evaluate(const VisualFrame & reference, const VisualFrame & candidate) const
{
  return evaluate_shifted(reference, candidate, 0, 0);
}

VisualPhotometricLinearization VisualFactor::linearize_translation(
  const VisualFrame & reference,
  const VisualFrame & candidate) const
{
  VisualPhotometricLinearization output;
  if (reference.width < 3U || reference.height < 3U ||
    reference.width != candidate.width || reference.height != candidate.height)
  {
    return output;
  }
  const size_t pixel_count = reference.width * reference.height;
  if (reference.gray.size() != pixel_count || candidate.gray.size() != pixel_count) {
    return output;
  }

  const size_t stride = pixel_count > max_pixels_
    ? static_cast<size_t>(std::ceil(static_cast<double>(pixel_count) / static_cast<double>(max_pixels_)))
    : 1U;
  for (size_t index = 0; index < pixel_count; index += stride) {
    const size_t x = index % reference.width;
    const size_t y = index / reference.width;
    if (x == 0U || y == 0U || x + 1U >= reference.width || y + 1U >= reference.height) {
      continue;
    }
    const auto at = [&candidate](const size_t px, const size_t py) {
        return static_cast<double>(candidate.gray[py * candidate.width + px]);
      };
    const double residual = static_cast<double>(candidate.gray[index] - reference.gray[index]);
    const Eigen::Vector2d jacobian{
      0.5 * (at(x + 1U, y) - at(x - 1U, y)),
      0.5 * (at(x, y + 1U) - at(x, y - 1U))};
    output.hessian += jacobian * jacobian.transpose();
    output.rhs.noalias() -= jacobian * residual;
    output.cost += 0.5 * residual * residual;
    ++output.compared_pixels;
  }
  if (output.compared_pixels == 0U || !output.hessian.allFinite() || !output.rhs.allFinite()) {
    return output;
  }
  const Eigen::LDLT<Eigen::Matrix2d> ldlt(output.hessian);
  if (ldlt.info() != Eigen::Success) {
    return output;
  }
  output.gauss_newton_step = ldlt.solve(output.rhs);
  output.valid = output.gauss_newton_step.allFinite();
  return output;
}

VisualResidual VisualFactor::evaluate_shifted(
  const VisualFrame & reference,
  const VisualFrame & candidate,
  const int dx,
  const int dy) const
{
  VisualResidual residual;
  if (reference.width == 0U || reference.height == 0U ||
    reference.width != candidate.width || reference.height != candidate.height)
  {
    return residual;
  }
  const int width = static_cast<int>(reference.width);
  const int height = static_cast<int>(reference.height);
  if (std::abs(dx) >= width || std::abs(dy) >= height) {
    return residual;
  }
  const size_t pixel_count = reference.width * reference.height;
  if (reference.gray.size() != pixel_count || candidate.gray.size() != pixel_count) {
    return residual;
  }

  const size_t stride = pixel_count > max_pixels_
    ? static_cast<size_t>(std::ceil(static_cast<double>(pixel_count) / static_cast<double>(max_pixels_)))
    : 1U;
  double abs_sum = 0.0;
  double sq_sum = 0.0;
  size_t compared = 0U;
  for (size_t index = 0; index < pixel_count; index += stride) {
    const int x = static_cast<int>(index % reference.width);
    const int y = static_cast<int>(index / reference.width);
    const int candidate_x = x + dx;
    const int candidate_y = y + dy;
    if (candidate_x < 0 || candidate_x >= width || candidate_y < 0 || candidate_y >= height) {
      continue;
    }
    const size_t candidate_index =
      static_cast<size_t>(candidate_y) * candidate.width + static_cast<size_t>(candidate_x);
    const double diff = static_cast<double>(candidate.gray[candidate_index] - reference.gray[index]);
    abs_sum += std::abs(diff);
    sq_sum += diff * diff;
    ++compared;
  }
  if (compared == 0U) {
    return residual;
  }
  residual.valid = true;
  residual.compared_pixels = compared;
  residual.mean_abs_error = abs_sum / static_cast<double>(compared);
  residual.rmse = std::sqrt(sq_sum / static_cast<double>(compared));
  return residual;
}

VisualAlignment VisualFactor::estimate_translation(
  const VisualFrame & reference,
  const VisualFrame & candidate,
  const int max_shift_px) const
{
  VisualAlignment best;
  if (max_shift_px < 0) {
    return best;
  }
  for (int dy = -max_shift_px; dy <= max_shift_px; ++dy) {
    for (int dx = -max_shift_px; dx <= max_shift_px; ++dx) {
      const auto residual = evaluate_shifted(reference, candidate, dx, dy);
      if (!residual.valid) {
        continue;
      }
      if (!best.valid || residual.rmse < best.rmse) {
        best.valid = true;
        best.dx = dx;
        best.dy = dy;
        best.subpixel_dx = static_cast<double>(dx);
        best.subpixel_dy = static_cast<double>(dy);
        best.compared_pixels = residual.compared_pixels;
        best.mean_abs_error = residual.mean_abs_error;
        best.rmse = residual.rmse;
      }
    }
  }
  if (best.valid && best.rmse > 1.0e-9) {
    const auto x_minus = evaluate_shifted(reference, candidate, best.dx - 1, best.dy);
    const auto x_plus = evaluate_shifted(reference, candidate, best.dx + 1, best.dy);
    if (x_minus.valid && x_plus.valid) {
      best.subpixel_dx =
        static_cast<double>(best.dx) + parabolic_subpixel_offset(x_minus.rmse, best.rmse, x_plus.rmse);
    }
    const auto y_minus = evaluate_shifted(reference, candidate, best.dx, best.dy - 1);
    const auto y_plus = evaluate_shifted(reference, candidate, best.dx, best.dy + 1);
    if (y_minus.valid && y_plus.valid) {
      best.subpixel_dy =
        static_cast<double>(best.dy) + parabolic_subpixel_offset(y_minus.rmse, best.rmse, y_plus.rmse);
    }
  }
  return best;
}

}  // namespace gaussian_lic_tracking
