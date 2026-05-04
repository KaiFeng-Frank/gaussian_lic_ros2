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

  candidate.width = 2;
  if (factor.evaluate(reference, candidate).valid) {
    std::cerr << "shape-mismatched visual frames should be invalid\n";
    return 1;
  }

  std::cout << "visual_factor_probe OK\n";
  return 0;
}
