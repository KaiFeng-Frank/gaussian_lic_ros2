// SPDX-License-Identifier: GPL-3.0-or-later

#include <gaussian_lic_tracking/sliding_window_optimizer.hpp>

#include <cmath>
#include <iostream>

int main()
{
  constexpr int steps = 80;
  constexpr int64_t dt_ns = 12500000LL;
  const int64_t end_stamp_ns = static_cast<int64_t>(steps) * dt_ns;
  const gaussian_lic_tracking::ImuBias true_bias{
    Eigen::Vector3d{0.015, -0.02, 0.035},
    Eigen::Vector3d{0.12, -0.08, 0.05}};

  gaussian_lic_tracking::ImuPreintegrator preintegration;
  preintegration.reset(0);
  for (int i = 1; i <= steps; ++i) {
    preintegration.add_measurement(
      static_cast<int64_t>(i) * dt_ns,
      true_bias.gyro,
      true_bias.accel);
  }

  gaussian_lic_tracking::SlidingWindowConfig config;
  config.max_iterations = 12;
  config.damping = 1.0e-7;
  gaussian_lic_tracking::SlidingWindowOptimizer optimizer(config);

  gaussian_lic_tracking::SlidingWindowState start;
  start.stamp_ns = 0;
  start.fixed = false;
  optimizer.add_or_update_state(start);

  gaussian_lic_tracking::SlidingWindowState end;
  end.stamp_ns = end_stamp_ns;
  end.p_w_i = Eigen::Vector3d{0.08, -0.04, 0.02};
  end.v_w_i = Eigen::Vector3d{0.1, -0.05, 0.03};
  optimizer.add_or_update_state(end);

  auto add_motion_prior = [&optimizer](const int64_t stamp_ns) {
      gaussian_lic_tracking::SlidingWindowStatePrior prior;
      prior.stamp_ns = stamp_ns;
      prior.rotation_weight = 1000.0;
      prior.velocity_weight = 1000.0;
      prior.position_weight = 1000.0;
      prior.gyro_bias_weight = 0.0;
      prior.accel_bias_weight = 0.0;
      optimizer.add_state_prior(prior);
    };
  add_motion_prior(start.stamp_ns);
  add_motion_prior(end.stamp_ns);

  gaussian_lic_tracking::SlidingWindowImuFactor factor;
  factor.from_stamp_ns = start.stamp_ns;
  factor.to_stamp_ns = end.stamp_ns;
  factor.preintegration = preintegration;
  factor.weight = 100.0;
  factor.bias_weight = 100.0;
  optimizer.add_imu_factor(factor);

  const auto summary = optimizer.optimize();
  gaussian_lic_tracking::SlidingWindowState optimized_start;
  gaussian_lic_tracking::SlidingWindowState optimized_end;
  if (!optimizer.get_state(start.stamp_ns, optimized_start) ||
    !optimizer.get_state(end.stamp_ns, optimized_end))
  {
    std::cerr << "optimized bias-observability states are missing\n";
    return 1;
  }

  const double start_gyro_error = (optimized_start.gyro_bias - true_bias.gyro).norm();
  const double start_accel_error = (optimized_start.accel_bias - true_bias.accel).norm();
  const double end_gyro_error = (optimized_end.gyro_bias - true_bias.gyro).norm();
  const double end_accel_error = (optimized_end.accel_bias - true_bias.accel).norm();
  std::cout << "sliding_window_bias_observability_probe iterations=" << summary.iterations
            << " initial_cost=" << summary.initial_cost
            << " final_cost=" << summary.final_cost
            << " gyro_observability=" << summary.gyro_bias_observability
            << " accel_observability=" << summary.accel_bias_observability
            << " start_gyro_error=" << start_gyro_error
            << " start_accel_error=" << start_accel_error
            << " end_gyro_error=" << end_gyro_error
            << " end_accel_error=" << end_accel_error << "\n";

  if (!summary.converged || summary.final_cost >= summary.initial_cost ||
    summary.gyro_bias_observability <= 0.0 || summary.accel_bias_observability <= 0.0 ||
    start_gyro_error > 1.0e-4 || start_accel_error > 1.0e-4 ||
    end_gyro_error > 1.0e-4 || end_accel_error > 1.0e-4)
  {
    std::cerr << "sliding window failed to observe IMU bias from external motion constraints\n";
    return 1;
  }

  std::cout << "sliding_window_bias_observability_probe OK\n";
  return 0;
}
