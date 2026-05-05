// SPDX-License-Identifier: GPL-3.0-or-later

#include <gaussian_lic_tracking/visual_factor.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

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
