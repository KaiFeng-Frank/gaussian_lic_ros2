// SPDX-License-Identifier: GPL-3.0-or-later

#include <gaussian_lic_tracking/visual_factor.hpp>

#include <cmath>
#include <stdexcept>

namespace gaussian_lic_tracking
{

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
  VisualResidual residual;
  if (reference.width == 0U || reference.height == 0U ||
    reference.width != candidate.width || reference.height != candidate.height)
  {
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
    const double diff = static_cast<double>(candidate.gray[index] - reference.gray[index]);
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

}  // namespace gaussian_lic_tracking
