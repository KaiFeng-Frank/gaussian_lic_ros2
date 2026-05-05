// SPDX-License-Identifier: GPL-3.0-or-later

#include <gaussian_lic_tracking/imu_propagator.hpp>
#include <gaussian_lic_tracking/time.hpp>

#include <cmath>
#include <iostream>

int main()
{
  gaussian_lic_tracking::ImuPropagator propagator;
  gaussian_lic_tracking::ImuState initial;
  initial.stamp_ns = 0;
  propagator.reset(initial);

  constexpr int steps = 100;
  constexpr int64_t dt_ns = 10000000LL;
  const Eigen::Vector3d omega(0.0, 0.0, M_PI * 0.5);
  const Eigen::Vector3d zero_accel(0.0, 0.0, 0.0);
  for (int i = 1; i <= steps; ++i) {
    propagator.add_measurement(static_cast<int64_t>(i) * dt_ns, omega, zero_accel);
  }

  const auto yaw_state = propagator.state();
  const double yaw = Eigen::AngleAxisd(yaw_state.q_w_i).angle();
  const double yaw_error = std::abs(yaw - M_PI * 0.5);
  if (yaw_state.stamp_ns != steps * dt_ns) {
    std::cerr << "IMU yaw propagation stamp is wrong\n";
    return 1;
  }

  propagator.reset(initial);
  const Eigen::Vector3d zero_omega(0.0, 0.0, 0.0);
  const Eigen::Vector3d accel(1.0, 0.0, 0.0);
  for (int i = 1; i <= steps; ++i) {
    propagator.add_measurement(static_cast<int64_t>(i) * dt_ns, zero_omega, accel);
  }
  const auto accel_state = propagator.state();
  const double position_x_error = std::abs(accel_state.p_w_i.x() - 0.5);
  const double velocity_x_error = std::abs(accel_state.v_w_i.x() - 1.0);
  gaussian_lic_tracking::ImuState midpoint_state;
  if (!propagator.query_state(500000000LL, midpoint_state)) {
    std::cerr << "IMU history query failed\n";
    return 1;
  }
  const double midpoint_position_error = std::abs(midpoint_state.p_w_i.x() - 0.125);
  const double midpoint_velocity_error = std::abs(midpoint_state.v_w_i.x() - 0.5);

  std::cout << "imu_propagator_probe stamp_ns=" << accel_state.stamp_ns
            << " yaw_error=" << yaw_error
            << " position_x_error=" << position_x_error
            << " velocity_x_error=" << velocity_x_error
            << " midpoint_position_x_error=" << midpoint_position_error
            << " midpoint_velocity_x_error=" << midpoint_velocity_error
            << " history_size=" << propagator.history_size() << "\n";
  if (accel_state.stamp_ns != steps * dt_ns) {
    std::cerr << "IMU acceleration propagation stamp is wrong\n";
    return 1;
  }
  if (yaw_error > 1.0e-12 || position_x_error > 1.0e-12 || velocity_x_error > 1.0e-12 ||
    midpoint_position_error > 1.0e-12 || midpoint_velocity_error > 1.0e-12)
  {
    std::cerr << "IMU propagation drift exceeds tolerance\n";
    return 1;
  }

  std::cout << "imu_propagator_probe OK\n";
  return 0;
}
