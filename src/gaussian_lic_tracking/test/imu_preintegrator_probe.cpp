// SPDX-License-Identifier: GPL-3.0-or-later

#include <gaussian_lic_tracking/imu_preintegrator.hpp>
#include <gaussian_lic_tracking/time.hpp>

#include <cmath>
#include <exception>
#include <iostream>
#include <limits>

int main()
{
  constexpr int steps = 100;
  constexpr int64_t dt_ns = 10000000LL;
  gaussian_lic_tracking::ImuPreintegrator preintegrator;
  preintegrator.reset(0);

  const Eigen::Vector3d omega(0.0, 0.0, M_PI * 0.5);
  const Eigen::Vector3d accel(1.0, 0.0, 0.0);
  for (int i = 1; i <= steps; ++i) {
    preintegrator.add_measurement(static_cast<int64_t>(i) * dt_ns, omega, accel);
  }

  gaussian_lic_tracking::ImuState start;
  start.stamp_ns = 0;
  gaussian_lic_tracking::ImuState end;
  end.stamp_ns = steps * dt_ns;
  end.q_w_i = preintegrator.delta_q();
  end.v_w_i = preintegrator.delta_v();
  end.p_w_i = preintegrator.delta_p();

  const auto residual = preintegrator.residual(start, end);
  std::cout << "imu_preintegrator_probe dt=" << preintegrator.delta_t_s()
            << " rotation_norm=" << residual.rotation_norm
            << " velocity_norm=" << residual.velocity_norm
            << " position_norm=" << residual.position_norm
            << " delta_p=" << preintegrator.delta_p().transpose()
            << " delta_v=" << preintegrator.delta_v().transpose() << "\n";
  if (std::abs(preintegrator.delta_t_s() - 1.0) > 1.0e-12 ||
    residual.rotation_norm > 1.0e-12 ||
    residual.velocity_norm > 1.0e-12 ||
    residual.position_norm > 1.0e-12)
  {
    std::cerr << "IMU preintegration residual does not close on its propagated endpoint\n";
    return 1;
  }

  bool rejected_bad_measurement = false;
  try {
    preintegrator.add_measurement(
      (steps + 1) * dt_ns,
      Eigen::Vector3d{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0},
      accel);
  } catch (const std::exception &) {
    rejected_bad_measurement = true;
  }
  bool rejected_bad_quaternion = false;
  try {
    auto bad_end = end;
    bad_end.q_w_i.coeffs().setZero();
    (void)preintegrator.residual(start, bad_end);
  } catch (const std::exception &) {
    rejected_bad_quaternion = true;
  }
  if (!rejected_bad_measurement || !rejected_bad_quaternion) {
    std::cerr << "IMU preintegration validation failed to reject invalid inputs\n";
    return 1;
  }

  gaussian_lic_tracking::ImuPreintegrator auto_started;
  auto_started.add_measurement(100, omega, accel);
  auto_started.add_measurement(100 + dt_ns, omega, accel);
  const auto auto_reintegrated = auto_started.reintegrated({});
  if (auto_reintegrated.start_stamp_ns() != auto_started.start_stamp_ns() ||
    auto_reintegrated.end_stamp_ns() != auto_started.end_stamp_ns() ||
    auto_reintegrated.sample_count() != auto_started.sample_count() ||
    std::abs(auto_reintegrated.delta_t_s() - auto_started.delta_t_s()) > 1.0e-12 ||
    (auto_reintegrated.delta_p() - auto_started.delta_p()).norm() > 1.0e-12 ||
    (auto_reintegrated.delta_v() - auto_started.delta_v()).norm() > 1.0e-12)
  {
    std::cerr << "IMU preintegration failed to re-integrate an auto-start sample at the span start\n";
    return 1;
  }
  std::cout << "imu_preintegrator_probe OK\n";
  return 0;
}
