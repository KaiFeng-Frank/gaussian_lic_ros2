// SPDX-License-Identifier: GPL-3.0-or-later

#include <gaussian_lic_tracking/sliding_window_optimizer.hpp>

#include <cmath>
#include <iostream>

int main()
{
  constexpr int steps = 100;
  constexpr int64_t dt_ns = 10000000LL;

  gaussian_lic_tracking::ImuPreintegrator preintegration;
  preintegration.reset(0);
  for (int i = 1; i <= steps; ++i) {
    preintegration.add_measurement(
      static_cast<int64_t>(i) * dt_ns,
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d{1.0, 0.0, 0.0});
  }

  gaussian_lic_tracking::SlidingWindowOptimizer optimizer;
  gaussian_lic_tracking::SlidingWindowState start;
  start.stamp_ns = 0;
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
  optimizer.add_imu_factor(factor);

  const auto summary = optimizer.optimize();
  gaussian_lic_tracking::SlidingWindowState optimized;
  if (!optimizer.get_state(end.stamp_ns, optimized)) {
    std::cerr << "optimized endpoint is missing\n";
    return 1;
  }

  const double position_error = (optimized.p_w_i - Eigen::Vector3d{0.5, 0.0, 0.0}).norm();
  const double velocity_error = (optimized.v_w_i - Eigen::Vector3d{1.0, 0.0, 0.0}).norm();
  std::cout << "sliding_window_optimizer_probe states=" << summary.state_count
            << " imu_factors=" << summary.imu_factor_count
            << " iterations=" << summary.iterations
            << " initial_cost=" << summary.initial_cost
            << " final_cost=" << summary.final_cost
            << " position_error=" << position_error
            << " velocity_error=" << velocity_error << "\n";

  if (!summary.converged || summary.final_cost >= summary.initial_cost ||
    position_error > 1.0e-8 || velocity_error > 1.0e-8)
  {
    std::cerr << "sliding window optimizer failed to close the IMU factor\n";
    return 1;
  }
  std::cout << "sliding_window_optimizer_probe OK\n";
  return 0;
}
