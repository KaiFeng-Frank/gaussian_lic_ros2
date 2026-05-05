// SPDX-License-Identifier: GPL-3.0-or-later

#include <gaussian_lic_tracking/sliding_window_optimizer.hpp>

#include <cmath>
#include <iostream>

int main()
{
  gaussian_lic_tracking::SlidingWindowConfig config;
  config.max_states = 3;
  config.max_iterations = 4;
  config.marginalization_prior_weight = 25.0;
  gaussian_lic_tracking::SlidingWindowOptimizer optimizer(config);

  for (int i = 0; i < 5; ++i) {
    gaussian_lic_tracking::SlidingWindowState state;
    state.stamp_ns = static_cast<int64_t>(i);
    state.p_w_i = Eigen::Vector3d{static_cast<double>(i), 0.1 * static_cast<double>(i), 0.0};
    state.v_w_i = Eigen::Vector3d{1.0, 0.1, 0.0};
    state.gyro_bias = Eigen::Vector3d{0.01, 0.02, 0.03};
    state.accel_bias = Eigen::Vector3d{0.1, -0.2, 0.05};
    optimizer.add_or_update_state(state);
  }

  gaussian_lic_tracking::SlidingWindowState disturbed;
  if (!optimizer.get_state(2, disturbed)) {
    std::cerr << "expected retained anchor state is missing\n";
    return 1;
  }
  const Eigen::Vector3d expected_position = disturbed.p_w_i;
  const Eigen::Vector3d expected_velocity = disturbed.v_w_i;
  const Eigen::Vector3d expected_gyro_bias = disturbed.gyro_bias;
  const Eigen::Vector3d expected_accel_bias = disturbed.accel_bias;
  disturbed.p_w_i += Eigen::Vector3d{3.0, -2.0, 1.0};
  disturbed.v_w_i += Eigen::Vector3d{-0.4, 0.2, 0.1};
  disturbed.gyro_bias += Eigen::Vector3d{0.2, 0.0, -0.1};
  disturbed.accel_bias += Eigen::Vector3d{-0.3, 0.1, 0.2};
  optimizer.add_or_update_state(disturbed);

  const auto summary = optimizer.optimize();
  gaussian_lic_tracking::SlidingWindowState optimized;
  if (!optimizer.get_state(2, optimized)) {
    std::cerr << "optimized anchor state is missing\n";
    return 1;
  }

  const double position_error = (optimized.p_w_i - expected_position).norm();
  const double velocity_error = (optimized.v_w_i - expected_velocity).norm();
  const double gyro_bias_error = (optimized.gyro_bias - expected_gyro_bias).norm();
  const double accel_bias_error = (optimized.accel_bias - expected_accel_bias).norm();
  std::cout << "sliding_window_marginalization_probe states=" << summary.state_count
            << " state_priors=" << summary.state_prior_count
            << " marginalized=" << summary.marginalized_state_count
            << " schur=" << summary.schur_marginalization_count
            << " fallback=" << summary.fallback_marginalization_prior_count
            << " initial_cost=" << summary.initial_cost
            << " final_cost=" << summary.final_cost
            << " position_error=" << position_error
            << " velocity_error=" << velocity_error
            << " gyro_bias_error=" << gyro_bias_error
            << " accel_bias_error=" << accel_bias_error << "\n";

  if (!summary.converged || summary.state_count != config.max_states ||
    summary.state_prior_count == 0U || summary.marginalized_state_count < 2U ||
    summary.fallback_marginalization_prior_count < 2U ||
    summary.final_cost >= summary.initial_cost ||
    position_error > 1.0e-8 || velocity_error > 1.0e-8 ||
    gyro_bias_error > 1.0e-8 || accel_bias_error > 1.0e-8)
  {
    std::cerr << "sliding window marginalization prior failed to preserve the retained state\n";
    return 1;
  }
  std::cout << "sliding_window_marginalization_probe OK\n";
  return 0;
}
