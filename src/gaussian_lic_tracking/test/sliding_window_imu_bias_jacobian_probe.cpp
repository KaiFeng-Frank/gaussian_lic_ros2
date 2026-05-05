// SPDX-License-Identifier: GPL-3.0-or-later

#include <gaussian_lic_tracking/sliding_window_optimizer.hpp>

#include <cmath>
#include <iostream>

int main()
{
  gaussian_lic_tracking::SlidingWindowOptimizer optimizer;

  gaussian_lic_tracking::SlidingWindowState start;
  start.stamp_ns = 0;
  start.gyro_bias = Eigen::Vector3d{0.01, -0.02, 0.03};
  start.accel_bias = Eigen::Vector3d{-0.1, 0.05, 0.2};
  optimizer.add_or_update_state(start);

  gaussian_lic_tracking::SlidingWindowState end;
  end.stamp_ns = 1000000000;
  end.gyro_bias = Eigen::Vector3d{0.015, -0.017, 0.028};
  end.accel_bias = Eigen::Vector3d{-0.08, 0.04, 0.22};
  optimizer.add_or_update_state(end);

  gaussian_lic_tracking::ImuPreintegrator preintegration;
  preintegration.reset(start.stamp_ns);
  constexpr int kSamples = 20;
  for (int i = 1; i <= kSamples; ++i) {
    const int64_t stamp_ns = static_cast<int64_t>(i) * 50000000LL;
    preintegration.add_measurement(
      stamp_ns,
      Eigen::Vector3d{0.0, 0.0, 0.0},
      Eigen::Vector3d{0.0, 0.0, 0.0});
  }

  gaussian_lic_tracking::SlidingWindowImuFactor factor;
  factor.from_stamp_ns = start.stamp_ns;
  factor.to_stamp_ns = end.stamp_ns;
  factor.preintegration = preintegration;
  factor.weight = 1.0;
  factor.bias_weight = 9.0;
  optimizer.add_imu_factor(factor);

  const auto normal = optimizer.build_normal_equation(0.0);
  if (!normal.valid || normal.jacobian.rows() != 15 || normal.jacobian.cols() != 30) {
    std::cerr << "IMU bias Jacobian normal equation has wrong dimensions\n";
    return 1;
  }

  Eigen::MatrixXd expected = Eigen::MatrixXd::Zero(6, 30);
  const double scale = std::sqrt(factor.bias_weight);
  expected.block<3, 3>(0, 9) = -scale * Eigen::Matrix3d::Identity();
  expected.block<3, 3>(3, 12) = -scale * Eigen::Matrix3d::Identity();
  expected.block<3, 3>(0, 24) = scale * Eigen::Matrix3d::Identity();
  expected.block<3, 3>(3, 27) = scale * Eigen::Matrix3d::Identity();

  const Eigen::MatrixXd actual = normal.jacobian.block(9, 0, 6, 30);
  const double max_abs_error = (actual - expected).cwiseAbs().maxCoeff();
  std::cout << "sliding_window_imu_bias_jacobian_probe rows=" << normal.jacobian.rows()
            << " cols=" << normal.jacobian.cols()
            << " max_abs_error=" << max_abs_error << "\n";
  if (max_abs_error > 1.0e-12) {
    std::cerr << "IMU bias continuity analytic Jacobian is wrong\n";
    return 1;
  }

  std::cout << "sliding_window_imu_bias_jacobian_probe OK\n";
  return 0;
}
