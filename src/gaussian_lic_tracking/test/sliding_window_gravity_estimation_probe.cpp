// SPDX-License-Identifier: GPL-3.0-or-later

// Deterministic probe for the decoupled sliding-window gravity refinement.
// A static IMU (no rotation, no motion) reads specific force equal to -gravity.
// With the two window states pinned static, the IMU residual is exactly zero
// only when the world gravity direction is correct, so refine_gravity_estimate
// must recover the true gravity from a coarsely-tilted initialization while
// preserving the (known) gravity magnitude.

#include <gaussian_lic_tracking/sliding_window_optimizer.hpp>

#include <cmath>
#include <iostream>

int main()
{
  constexpr double kPi = 3.14159265358979323846;
  const Eigen::Vector3d gravity_true{0.0, 0.0, -9.80665};
  const double gravity_magnitude = gravity_true.norm();

  // Specific force a stationary accelerometer reports (reaction to gravity).
  const Eigen::Vector3d specific_force = -gravity_true;  // [0, 0, +9.80665]

  constexpr int steps = 80;
  constexpr int64_t dt_ns = 12500000LL;  // 12.5 ms -> 1.0 s total
  const int64_t end_stamp_ns = static_cast<int64_t>(steps) * dt_ns;

  gaussian_lic_tracking::ImuPreintegrator preintegration;
  preintegration.reset(0);
  for (int i = 1; i <= steps; ++i) {
    preintegration.add_measurement(
      static_cast<int64_t>(i) * dt_ns,
      Eigen::Vector3d::Zero(),
      specific_force);
  }

  gaussian_lic_tracking::SlidingWindowConfig config;
  config.max_iterations = 4;
  config.estimate_gravity = true;
  config.gravity_estimation_prior_weight = 0.0;  // pure IMU recovery for the probe
  config.gravity_estimation_min_factors = 1U;
  gaussian_lic_tracking::SlidingWindowOptimizer optimizer(config);

  gaussian_lic_tracking::SlidingWindowState start;
  start.stamp_ns = 0;
  start.fixed = true;
  optimizer.add_or_update_state(start);

  gaussian_lic_tracking::SlidingWindowState end;
  end.stamp_ns = end_stamp_ns;
  end.fixed = true;
  optimizer.add_or_update_state(end);

  // Coarse initialization error: tilt the gravity ~6 deg, keep the magnitude.
  const double tilt_rad = 6.0 * kPi / 180.0;
  const Eigen::Vector3d tilt_axis = Eigen::Vector3d{1.0, -0.5, 0.0}.normalized();
  const Eigen::Vector3d gravity_wrong =
    Eigen::AngleAxisd(tilt_rad, tilt_axis) * gravity_true;

  gaussian_lic_tracking::SlidingWindowImuFactor factor;
  factor.from_stamp_ns = start.stamp_ns;
  factor.to_stamp_ns = end.stamp_ns;
  factor.preintegration = preintegration;
  factor.gravity_w = gravity_wrong;
  factor.weight = 100.0;
  factor.bias_weight = 100.0;
  factor.gyro_bias_random_walk_sigma = 0.5;
  factor.accel_bias_random_walk_sigma = 2.0;
  optimizer.add_imu_factor(factor);

  const double initial_tilt_error = (gravity_wrong - gravity_true).norm();

  gaussian_lic_tracking::SlidingWindowSummary summary;
  for (int iteration = 0; iteration < 5; ++iteration) {
    summary = optimizer.optimize();
  }

  const Eigen::Vector3d gravity_estimate = optimizer.gravity_estimate();
  const double residual_tilt_error = (gravity_estimate - gravity_true).norm();
  const double magnitude_error = std::abs(gravity_estimate.norm() - gravity_magnitude);

  std::cout << "sliding_window_gravity_estimation_probe"
            << " updates=" << optimizer.gravity_estimation_update_count()
            << " initial_tilt_error=" << initial_tilt_error
            << " residual_tilt_error=" << residual_tilt_error
            << " magnitude_error=" << magnitude_error
            << " estimate=[" << gravity_estimate.x() << ", " << gravity_estimate.y()
            << ", " << gravity_estimate.z() << "]\n";

  if (optimizer.gravity_estimation_update_count() == 0U ||
    initial_tilt_error < 0.5 ||
    residual_tilt_error > 1.0e-3 ||
    magnitude_error > 1.0e-6)
  {
    std::cerr << "gravity estimation failed to recover the true gravity direction\n";
    return 1;
  }

  std::cout << "sliding_window_gravity_estimation_probe OK\n";
  return 0;
}
