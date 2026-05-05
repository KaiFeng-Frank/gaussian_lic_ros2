// SPDX-License-Identifier: GPL-3.0-or-later

#include <gaussian_lic_tracking/sliding_window_optimizer.hpp>

#include <cmath>
#include <iostream>

int main()
{
  gaussian_lic_tracking::SlidingWindowConfig config;
  config.max_iterations = 4;
  gaussian_lic_tracking::SlidingWindowOptimizer optimizer(config);

  gaussian_lic_tracking::SlidingWindowState state;
  state.stamp_ns = 42;
  state.p_w_i = Eigen::Vector3d{0.0, 0.0, 0.2};
  optimizer.add_or_update_state(state);

  gaussian_lic_tracking::SlidingWindowPointToPlaneFactor factor;
  factor.stamp_ns = state.stamp_ns;
  factor.weight = 100.0;
  for (int y = -2; y <= 2; ++y) {
    for (int x = -2; x <= 2; ++x) {
      factor.frame_points_i.emplace_back(0.1 * static_cast<double>(x), 0.1 * static_cast<double>(y), 0.0);
      factor.target_points_w.emplace_back(0.1 * static_cast<double>(x), 0.1 * static_cast<double>(y), 0.0);
      factor.target_normals_w.emplace_back(0.0, 0.0, 1.0);
      factor.point_weights.push_back(1.0);
    }
  }
  optimizer.add_point_to_plane_factor(factor);

  const auto summary = optimizer.optimize();
  gaussian_lic_tracking::SlidingWindowState optimized;
  if (!optimizer.get_state(state.stamp_ns, optimized)) {
    std::cerr << "optimized plane state is missing\n";
    return 1;
  }
  const double z_error = std::abs(optimized.p_w_i.z());
  std::cout << "sliding_window_plane_factor_probe plane_factors=" << summary.plane_factor_count
            << " initial_cost=" << summary.initial_cost
            << " final_cost=" << summary.final_cost
            << " z_error=" << z_error << "\n";
  if (!summary.converged || summary.plane_factor_count != 1U ||
    summary.final_cost >= summary.initial_cost || z_error > 1.0e-8)
  {
    std::cerr << "point-to-plane factor failed to constrain plane distance\n";
    return 1;
  }
  std::cout << "sliding_window_plane_factor_probe OK\n";
  return 0;
}
