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
  state.p_w_i = Eigen::Vector3d{0.4, -0.2, 0.1};
  state.v_w_i = Eigen::Vector3d{0.1, 0.2, -0.1};
  state.gyro_bias = Eigen::Vector3d{0.01, -0.02, 0.03};
  state.accel_bias = Eigen::Vector3d{-0.1, 0.05, 0.2};
  optimizer.add_or_update_state(state);

  gaussian_lic_tracking::SlidingWindowStatePrior prior;
  prior.stamp_ns = state.stamp_ns;
  prior.p_w_i = Eigen::Vector3d::Zero();
  prior.v_w_i = Eigen::Vector3d::Zero();
  prior.gyro_bias = Eigen::Vector3d::Zero();
  prior.accel_bias = Eigen::Vector3d::Zero();
  prior.sqrt_information = 2.0 * Eigen::Matrix<double, 15, 15>::Identity();
  prior.sqrt_information(6, 3) = 0.5;
  prior.sqrt_information(7, 4) = -0.25;
  optimizer.add_state_prior(prior);

  const auto summary = optimizer.optimize();
  gaussian_lic_tracking::SlidingWindowState optimized;
  if (!optimizer.get_state(state.stamp_ns, optimized)) {
    std::cerr << "optimized dense-prior state is missing\n";
    return 1;
  }

  const double position_error = optimized.p_w_i.norm();
  const double velocity_error = optimized.v_w_i.norm();
  const double gyro_bias_error = optimized.gyro_bias.norm();
  const double accel_bias_error = optimized.accel_bias.norm();
  std::cout << "sliding_window_dense_prior_probe state_priors=" << summary.state_prior_count
            << " initial_cost=" << summary.initial_cost
            << " final_cost=" << summary.final_cost
            << " position_error=" << position_error
            << " velocity_error=" << velocity_error
            << " gyro_bias_error=" << gyro_bias_error
            << " accel_bias_error=" << accel_bias_error << "\n";

  if (!summary.converged || summary.state_prior_count != 1U ||
    summary.final_cost >= summary.initial_cost ||
    position_error > 1.0e-8 || velocity_error > 1.0e-8 ||
    gyro_bias_error > 1.0e-8 || accel_bias_error > 1.0e-8)
  {
    std::cerr << "dense sliding-window prior failed to constrain the full state\n";
    return 1;
  }
  std::cout << "sliding_window_dense_prior_probe OK\n";
  return 0;
}
