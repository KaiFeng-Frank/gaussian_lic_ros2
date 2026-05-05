// SPDX-License-Identifier: GPL-3.0-or-later

#include <gaussian_lic_tracking/visual_factor.hpp>

#include <cmath>
#include <iostream>

int main()
{
  gaussian_lic_tracking::VisualFrame reference;
  reference.stamp_ns = 0;
  reference.width = 4;
  reference.height = 4;
  reference.gray.resize(16);
  for (size_t index = 0; index < reference.gray.size(); ++index) {
    reference.gray[index] = static_cast<float>(index) / 32.0F;
  }

  gaussian_lic_tracking::VisualFrame candidate = reference;
  gaussian_lic_tracking::VisualFactor factor(16);
  const auto identical = factor.evaluate(reference, candidate);
  if (!identical.valid || identical.rmse != 0.0 || identical.mean_abs_error != 0.0) {
    std::cerr << "identical visual frames should have zero residual\n";
    return 1;
  }

  for (auto & value : candidate.gray) {
    value += 0.1F;
  }
  const auto shifted = factor.evaluate(reference, candidate);
  std::cout << "visual_factor_probe compared=" << shifted.compared_pixels
            << " mae=" << shifted.mean_abs_error
            << " rmse=" << shifted.rmse << "\n";
  if (!shifted.valid) {
    std::cerr << "shifted visual residual is invalid\n";
    return 1;
  }
  if (std::abs(shifted.mean_abs_error - 0.1) > 1.0e-6 ||
    std::abs(shifted.rmse - 0.1) > 1.0e-6)
  {
    std::cerr << "shifted visual residual is wrong\n";
    return 1;
  }

  gaussian_lic_tracking::VisualFrame alignment_reference;
  alignment_reference.width = 8;
  alignment_reference.height = 8;
  alignment_reference.gray.assign(alignment_reference.width * alignment_reference.height, 0.0F);
  for (size_t y = 0; y < alignment_reference.height; ++y) {
    for (size_t x = 0; x < alignment_reference.width; ++x) {
      alignment_reference.gray[y * alignment_reference.width + x] =
        static_cast<float>(0.1 * static_cast<double>(x) + 0.03 * static_cast<double>(y));
    }
  }
  gaussian_lic_tracking::VisualFrame alignment_candidate = alignment_reference;
  for (size_t y = 0; y < alignment_reference.height; ++y) {
    for (size_t x = 0; x + 1 < alignment_reference.width; ++x) {
      alignment_candidate.gray[y * alignment_candidate.width + x + 1] =
        alignment_reference.gray[y * alignment_reference.width + x];
    }
  }
  const auto alignment = factor.estimate_translation(alignment_reference, alignment_candidate, 2);
  std::cout << " visual_alignment dx=" << alignment.dx
            << " dy=" << alignment.dy
            << " subpixel_dx=" << alignment.subpixel_dx
            << " subpixel_dy=" << alignment.subpixel_dy
            << " rmse=" << alignment.rmse;
  if (!alignment.valid || alignment.dx != 1 || alignment.dy != 0 ||
    std::abs(alignment.subpixel_dx - 1.0) > 1.0e-6 ||
    std::abs(alignment.subpixel_dy) > 1.0e-6 ||
    alignment.rmse > 1.0e-6)
  {
    std::cerr << "visual alignment failed to recover integer translation\n";
    return 1;
  }

  gaussian_lic_tracking::VisualFrame subpixel_reference;
  subpixel_reference.width = 32;
  subpixel_reference.height = 32;
  subpixel_reference.gray.resize(subpixel_reference.width * subpixel_reference.height);
  for (size_t y = 0; y < subpixel_reference.height; ++y) {
    for (size_t x = 0; x < subpixel_reference.width; ++x) {
      const double dx = static_cast<double>(x) - 15.0;
      const double dy = static_cast<double>(y) - 16.0;
      subpixel_reference.gray[y * subpixel_reference.width + x] =
        static_cast<float>(std::exp(-(dx * dx + dy * dy) / 40.0));
    }
  }
  auto bilinear = [&subpixel_reference](const double x, const double y) -> float {
      if (x < 0.0 || y < 0.0 ||
        x >= static_cast<double>(subpixel_reference.width - 1U) ||
        y >= static_cast<double>(subpixel_reference.height - 1U))
      {
        return 0.0F;
      }
      const auto x0 = static_cast<size_t>(std::floor(x));
      const auto y0 = static_cast<size_t>(std::floor(y));
      const double tx = x - static_cast<double>(x0);
      const double ty = y - static_cast<double>(y0);
      auto at = [&subpixel_reference](const size_t px, const size_t py) {
          return subpixel_reference.gray[py * subpixel_reference.width + px];
        };
      return static_cast<float>(
        (1.0 - tx) * (1.0 - ty) * at(x0, y0) +
        tx * (1.0 - ty) * at(x0 + 1U, y0) +
        (1.0 - tx) * ty * at(x0, y0 + 1U) +
        tx * ty * at(x0 + 1U, y0 + 1U));
    };
  gaussian_lic_tracking::VisualFrame subpixel_candidate = subpixel_reference;
  constexpr double expected_subpixel_dx = 1.25;
  constexpr double expected_subpixel_dy = -0.35;
  for (size_t y = 0; y < subpixel_candidate.height; ++y) {
    for (size_t x = 0; x < subpixel_candidate.width; ++x) {
      subpixel_candidate.gray[y * subpixel_candidate.width + x] =
        bilinear(
        static_cast<double>(x) - expected_subpixel_dx,
        static_cast<double>(y) - expected_subpixel_dy);
    }
  }
  gaussian_lic_tracking::VisualFactor dense_factor(
    subpixel_reference.width * subpixel_reference.height);
  const auto subpixel_alignment =
    dense_factor.estimate_translation(subpixel_reference, subpixel_candidate, 3);
  std::cout << " subpixel_alignment dx=" << subpixel_alignment.subpixel_dx
            << " dy=" << subpixel_alignment.subpixel_dy;
  if (!subpixel_alignment.valid ||
    std::abs(subpixel_alignment.subpixel_dx - expected_subpixel_dx) > 0.15 ||
    std::abs(subpixel_alignment.subpixel_dy - expected_subpixel_dy) > 0.15)
  {
    std::cerr << "visual alignment failed to recover subpixel translation\n";
    return 1;
  }
  const auto linearization =
    dense_factor.linearize_translation(subpixel_reference, subpixel_candidate);
  std::cout << " photometric_step=" << linearization.gauss_newton_step.transpose()
            << " photometric_cost=" << linearization.cost;
  if (!linearization.valid ||
    linearization.compared_pixels == 0U ||
    std::abs(linearization.gauss_newton_step.x() - expected_subpixel_dx) > 0.25 ||
    std::abs(linearization.gauss_newton_step.y() - expected_subpixel_dy) > 0.25)
  {
    std::cerr << "visual photometric linearization failed to recover translation step\n";
    return 1;
  }

  candidate.width = 2;
  if (factor.evaluate(reference, candidate).valid) {
    std::cerr << "shape-mismatched visual frames should be invalid\n";
    return 1;
  }

  std::cout << "visual_factor_probe OK\n";
  return 0;
}
