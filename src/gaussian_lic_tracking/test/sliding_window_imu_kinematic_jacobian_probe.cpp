// SPDX-License-Identifier: GPL-3.0-or-later

#include <gaussian_lic_tracking/sliding_window_optimizer.hpp>

#include <cmath>
#include <iostream>

namespace
{
gaussian_lic_tracking::SlidingWindowState perturb_state(
  gaussian_lic_tracking::SlidingWindowState state,
  const int local_column,
  const double delta)
{
  if (local_column < 3) {
    Eigen::Vector3d axis = Eigen::Vector3d::Zero();
    axis[local_column] = 1.0;
    state.q_w_i =
      (Eigen::Quaterniond(Eigen::AngleAxisd(delta, axis)) *
      state.q_w_i).normalized();
  } else if (local_column < 6) {
    state.v_w_i[local_column - 3] += delta;
  } else if (local_column < 9) {
    state.p_w_i[local_column - 6] += delta;
  } else if (local_column < 12) {
    state.gyro_bias[local_column - 9] += delta;
  } else {
    state.accel_bias[local_column - 12] += delta;
  }
  return state;
}

Eigen::VectorXd residual_for(
  const gaussian_lic_tracking::SlidingWindowState & start,
  const gaussian_lic_tracking::SlidingWindowState & end,
  const gaussian_lic_tracking::SlidingWindowImuFactor & factor)
{
  gaussian_lic_tracking::SlidingWindowOptimizer optimizer;
  optimizer.add_or_update_state(start);
  optimizer.add_or_update_state(end);
  optimizer.add_imu_factor(factor);
  return optimizer.build_normal_equation(0.0).residual;
}
}  // namespace

int main()
{
  gaussian_lic_tracking::SlidingWindowState start;
  start.stamp_ns = 0;
  start.q_w_i =
    Eigen::Quaterniond(Eigen::AngleAxisd(0.15, Eigen::Vector3d::UnitZ())) *
    Eigen::Quaterniond(Eigen::AngleAxisd(-0.08, Eigen::Vector3d::UnitY()));
  start.p_w_i = Eigen::Vector3d{0.3, -0.2, 1.1};
  start.v_w_i = Eigen::Vector3d{0.4, -0.1, 0.2};
  start.gyro_bias = Eigen::Vector3d{0.01, -0.02, 0.015};
  start.accel_bias = Eigen::Vector3d{-0.04, 0.03, 0.02};

  gaussian_lic_tracking::SlidingWindowState end;
  end.stamp_ns = 1000000000;
  end.q_w_i =
    Eigen::Quaterniond(Eigen::AngleAxisd(-0.04, Eigen::Vector3d::UnitX())) *
    start.q_w_i;
  end.p_w_i = Eigen::Vector3d{0.9, -0.25, 1.4};
  end.v_w_i = Eigen::Vector3d{0.55, -0.05, 0.18};
  end.gyro_bias = Eigen::Vector3d{0.012, -0.019, 0.014};
  end.accel_bias = Eigen::Vector3d{-0.035, 0.028, 0.021};

  gaussian_lic_tracking::ImuPreintegrator preintegration;
  preintegration.reset(start.stamp_ns);
  constexpr int kSamples = 20;
  for (int i = 1; i <= kSamples; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(kSamples);
    const int64_t stamp_ns = static_cast<int64_t>(i) * 50000000LL;
    preintegration.add_measurement(
      stamp_ns,
      Eigen::Vector3d{0.02 + 0.01 * t, -0.015, 0.03 - 0.005 * t},
      Eigen::Vector3d{0.1, -0.05 + 0.02 * t, 0.03});
  }

  gaussian_lic_tracking::SlidingWindowImuFactor factor;
  factor.from_stamp_ns = start.stamp_ns;
  factor.to_stamp_ns = end.stamp_ns;
  factor.preintegration = preintegration;
  factor.gravity_w = Eigen::Vector3d{0.0, 0.0, -9.81};
  factor.weight = 3.7;
  factor.rotation_weight = 1.4;
  factor.velocity_weight = 0.8;
  factor.position_weight = 2.1;
  factor.bias_weight = 2.25;
  factor.sqrt_information = Eigen::Matrix<double, 9, 9>::Zero();
  for (Eigen::Index index = 0; index < 9; ++index) {
    factor.sqrt_information(index, index) = 0.35 + 0.05 * static_cast<double>(index);
  }
  factor.sqrt_information(0, 3) = 0.02;
  factor.sqrt_information(1, 4) = -0.015;
  factor.sqrt_information(2, 6) = 0.01;
  factor.sqrt_information(5, 8) = -0.025;

  gaussian_lic_tracking::SlidingWindowOptimizer optimizer;
  optimizer.add_or_update_state(start);
  optimizer.add_or_update_state(end);
  optimizer.add_imu_factor(factor);
  const auto normal = optimizer.build_normal_equation(0.0);
  if (!normal.valid || normal.jacobian.rows() != 15 || normal.jacobian.cols() != 30) {
    std::cerr << "IMU kinematic Jacobian normal equation has wrong dimensions\n";
    return 1;
  }

  constexpr double kEpsilon = 2.0e-7;
  Eigen::MatrixXd numeric(normal.jacobian.rows(), normal.jacobian.cols());
  for (Eigen::Index column = 0; column < normal.jacobian.cols(); ++column) {
    auto plus_start = start;
    auto minus_start = start;
    auto plus_end = end;
    auto minus_end = end;
    if (column < 15) {
      plus_start = perturb_state(plus_start, static_cast<int>(column), kEpsilon);
      minus_start = perturb_state(minus_start, static_cast<int>(column), -kEpsilon);
    } else {
      const int local_column = static_cast<int>(column - 15);
      plus_end = perturb_state(plus_end, local_column, kEpsilon);
      minus_end = perturb_state(minus_end, local_column, -kEpsilon);
    }
    numeric.col(column) =
      (residual_for(plus_start, plus_end, factor) -
      residual_for(minus_start, minus_end, factor)) / (2.0 * kEpsilon);
  }

  const double max_abs_error = (normal.jacobian - numeric).cwiseAbs().maxCoeff();
  std::cout << "sliding_window_imu_kinematic_jacobian_probe rows=" << normal.jacobian.rows()
            << " cols=" << normal.jacobian.cols()
            << " max_abs_error=" << max_abs_error << "\n";
  if (max_abs_error > 5.0e-5) {
    std::cerr << "IMU kinematic analytic Jacobian does not match numeric residual\n";
    return 1;
  }

  std::cout << "sliding_window_imu_kinematic_jacobian_probe OK\n";
  return 0;
}
