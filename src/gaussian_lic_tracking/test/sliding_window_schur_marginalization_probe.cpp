// SPDX-License-Identifier: GPL-3.0-or-later

#include <gaussian_lic_tracking/sliding_window_optimizer.hpp>

#include <iostream>

int main()
{
  constexpr int steps = 40;
  constexpr int64_t dt_ns = 10000000LL;

  gaussian_lic_tracking::SlidingWindowConfig config;
  config.max_states = 2;
  config.max_iterations = 3;
  config.marginalization_prior_weight = 0.0;
  gaussian_lic_tracking::SlidingWindowOptimizer optimizer(config);

  gaussian_lic_tracking::SlidingWindowState start;
  start.stamp_ns = 0;
  start.p_w_i = Eigen::Vector3d::Zero();
  optimizer.add_or_update_state(start);

  gaussian_lic_tracking::SlidingWindowState middle;
  middle.stamp_ns = steps * dt_ns;
  middle.p_w_i = Eigen::Vector3d{-0.1, 0.05, 0.0};
  middle.v_w_i = Eigen::Vector3d{0.0, -0.05, 0.0};
  optimizer.add_or_update_state(middle);

  gaussian_lic_tracking::ImuPreintegrator preintegration;
  preintegration.reset(start.stamp_ns);
  for (int i = 1; i <= steps; ++i) {
    preintegration.add_measurement(
      static_cast<int64_t>(i) * dt_ns,
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d{1.0, 0.0, 0.0});
  }
  gaussian_lic_tracking::SlidingWindowImuFactor factor;
  factor.from_stamp_ns = start.stamp_ns;
  factor.to_stamp_ns = middle.stamp_ns;
  factor.preintegration = preintegration;
  factor.weight = 5.0;
  factor.bias_weight = 1.0;
  optimizer.add_imu_factor(factor);

  gaussian_lic_tracking::SlidingWindowState newest;
  newest.stamp_ns = middle.stamp_ns + dt_ns;
  newest.p_w_i = Eigen::Vector3d{0.2, 0.0, 0.0};
  optimizer.add_or_update_state(newest);

  const auto normal = optimizer.build_normal_equation(config.damping);
  const auto summary = optimizer.optimize();
  std::cout << "sliding_window_schur_marginalization_probe states=" << summary.state_count
            << " dense_priors=" << summary.dense_prior_count
            << " state_priors=" << summary.state_prior_count
            << " marginalized=" << summary.marginalized_state_count
            << " schur=" << summary.schur_marginalization_count
            << " fallback=" << summary.fallback_marginalization_prior_count
            << " variable_count=" << normal.variable_count
            << " normal_cost=" << normal.cost
            << " final_cost=" << summary.final_cost << "\n";
  if (!normal.valid || summary.state_count != 2U || summary.dense_prior_count != 1U ||
    summary.state_prior_count != 0U || summary.marginalized_state_count != 1U ||
    summary.schur_marginalization_count != 1U ||
    summary.fallback_marginalization_prior_count != 0U ||
    normal.variable_count != 30U || summary.final_cost > normal.cost)
  {
    std::cerr << "Schur marginalization did not produce a retained dense window prior\n";
    return 1;
  }
  std::cout << "sliding_window_schur_marginalization_probe OK\n";
  return 0;
}
