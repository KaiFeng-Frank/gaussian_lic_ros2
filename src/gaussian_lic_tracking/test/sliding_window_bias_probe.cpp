// SPDX-License-Identifier: GPL-3.0-or-later

#include <gaussian_lic_tracking/sliding_window_optimizer.hpp>

#include <cmath>
#include <iostream>

int main()
{
  constexpr int steps = 100;
  constexpr int64_t dt_ns = 10000000LL;
  const gaussian_lic_tracking::ImuBias true_bias{
    Eigen::Vector3d{0.0, 0.0, 0.05},
    Eigen::Vector3d{0.2, -0.1, 0.0}};

  gaussian_lic_tracking::ImuPreintegrator preintegration;
  preintegration.reset(0);
  for (int i = 1; i <= steps; ++i) {
    preintegration.add_measurement(
      static_cast<int64_t>(i) * dt_ns,
      true_bias.gyro,
      Eigen::Vector3d{1.0, 0.0, 0.0} + true_bias.accel);
  }

  gaussian_lic_tracking::SlidingWindowConfig config;
  config.max_iterations = 6;
  gaussian_lic_tracking::SlidingWindowOptimizer optimizer(config);

  gaussian_lic_tracking::SlidingWindowState start;
  start.stamp_ns = 0;
  start.gyro_bias = true_bias.gyro;
  start.accel_bias = true_bias.accel;
  start.fixed = true;
  optimizer.add_or_update_state(start);

  gaussian_lic_tracking::SlidingWindowState end;
  end.stamp_ns = steps * dt_ns;
  end.p_w_i = Eigen::Vector3d{-0.2, 0.1, 0.0};
  end.v_w_i = Eigen::Vector3d{0.0, -0.1, 0.0};
  optimizer.add_or_update_state(end);

  gaussian_lic_tracking::SlidingWindowImuFactor factor;
  factor.from_stamp_ns = start.stamp_ns;
  factor.to_stamp_ns = end.stamp_ns;
  factor.preintegration = preintegration;
  factor.weight = 10.0;
  factor.bias_weight = 100.0;
  optimizer.add_imu_factor(factor);

  const auto summary = optimizer.optimize();
  gaussian_lic_tracking::SlidingWindowState optimized;
  if (!optimizer.get_state(end.stamp_ns, optimized)) {
    std::cerr << "optimized endpoint is missing\n";
    return 1;
  }

  const double position_error = (optimized.p_w_i - Eigen::Vector3d{0.5, 0.0, 0.0}).norm();
  const double velocity_error = (optimized.v_w_i - Eigen::Vector3d{1.0, 0.0, 0.0}).norm();
  const double gyro_bias_error = (optimized.gyro_bias - true_bias.gyro).norm();
  const double accel_bias_error = (optimized.accel_bias - true_bias.accel).norm();
  std::cout << "sliding_window_bias_probe samples=" << preintegration.sample_count()
            << " iterations=" << summary.iterations
            << " initial_cost=" << summary.initial_cost
            << " final_cost=" << summary.final_cost
            << " position_error=" << position_error
            << " velocity_error=" << velocity_error
            << " gyro_bias_error=" << gyro_bias_error
            << " accel_bias_error=" << accel_bias_error << "\n";

  if (!summary.converged || summary.final_cost >= summary.initial_cost ||
    position_error > 1.0e-8 || velocity_error > 1.0e-8 ||
    gyro_bias_error > 1.0e-8 || accel_bias_error > 1.0e-8)
  {
    std::cerr << "sliding window optimizer failed to honor IMU bias continuity/reintegration\n";
    return 1;
  }
  std::cout << "sliding_window_bias_probe OK\n";
  return 0;
}
