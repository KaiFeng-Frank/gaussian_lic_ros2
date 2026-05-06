// SPDX-License-Identifier: GPL-3.0-or-later

#include <gaussian_lic_tracking/sliding_window_optimizer.hpp>

#include <cmath>
#include <iostream>

namespace
{
Eigen::Quaterniond yaw_quaternion(const double yaw_rad)
{
  return Eigen::Quaterniond(Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()));
}

double yaw_error_rad(const Eigen::Quaterniond & expected, const Eigen::Quaterniond & actual)
{
  Eigen::Quaterniond error = (expected.normalized().inverse() * actual.normalized()).normalized();
  if (error.w() < 0.0) {
    error.coeffs() *= -1.0;
  }
  return 2.0 * error.vec().norm();
}
}  // namespace

int main()
{
  gaussian_lic_tracking::SlidingWindowConfig config;
  config.max_iterations = 12;
  config.damping = 1.0e-8;
  gaussian_lic_tracking::SlidingWindowOptimizer optimizer(config);

  gaussian_lic_tracking::SlidingWindowState previous;
  previous.stamp_ns = 0;
  previous.fixed = true;
  previous.p_w_i = Eigen::Vector3d::Zero();
  previous.q_w_i = yaw_quaternion(0.0);
  previous.v_w_i = Eigen::Vector3d{1.0, -0.5, 0.25};
  optimizer.add_or_update_state(previous);

  gaussian_lic_tracking::SlidingWindowState current;
  current.stamp_ns = 1000000000LL;
  current.p_w_i = Eigen::Vector3d{1.6, -1.2, 0.8};
  current.q_w_i = yaw_quaternion(0.9);
  current.v_w_i = Eigen::Vector3d{1.7, -0.1, 0.6};
  current.gyro_bias = Eigen::Vector3d{0.1, -0.04, 0.02};
  current.accel_bias = Eigen::Vector3d{-0.08, 0.03, 0.05};
  optimizer.add_or_update_state(current);

  gaussian_lic_tracking::SlidingWindowState next;
  next.stamp_ns = 2000000000LL;
  next.fixed = true;
  next.p_w_i = Eigen::Vector3d{2.0, -1.0, 0.5};
  next.q_w_i = yaw_quaternion(0.4);
  next.v_w_i = previous.v_w_i;
  optimizer.add_or_update_state(next);

  gaussian_lic_tracking::SlidingWindowTrajectorySmoothnessFactor factor;
  factor.previous_stamp_ns = previous.stamp_ns;
  factor.current_stamp_ns = current.stamp_ns;
  factor.next_stamp_ns = next.stamp_ns;
  factor.rotation_rate_weight = 10.0;
  factor.position_rate_weight = 10.0;
  factor.velocity_acceleration_weight = 10.0;
  factor.gyro_bias_rate_weight = 10.0;
  factor.accel_bias_rate_weight = 10.0;
  optimizer.add_trajectory_smoothness_factor(factor);

  const auto normal = optimizer.build_normal_equation(0.0);
  if (!normal.valid || normal.jacobian.rows() != 15 || normal.jacobian.cols() != 15) {
    std::cerr << "smoothness factor normal equation has wrong dimensions\n";
    return 1;
  }
  if (normal.numeric_jacobian_block_count != 0U || normal.numeric_jacobian_column_count != 0U) {
    std::cerr << "smoothness factor unexpectedly used global numeric Jacobian fallback\n";
    return 1;
  }
  Eigen::MatrixXd expected_linear = Eigen::MatrixXd::Zero(12, 15);
  const double scale = -2.0 * std::sqrt(10.0);
  expected_linear.block<3, 3>(0, 6) = scale * Eigen::Matrix3d::Identity();
  expected_linear.block<3, 3>(3, 3) = scale * Eigen::Matrix3d::Identity();
  expected_linear.block<3, 3>(6, 9) = scale * Eigen::Matrix3d::Identity();
  expected_linear.block<3, 3>(9, 12) = scale * Eigen::Matrix3d::Identity();
  const double max_linear_jacobian_error =
    (normal.jacobian.block(3, 0, 12, 15) - expected_linear).cwiseAbs().maxCoeff();

  const auto summary = optimizer.optimize();
  gaussian_lic_tracking::SlidingWindowState optimized;
  if (!optimizer.get_state(current.stamp_ns, optimized)) {
    std::cerr << "optimized middle state is missing\n";
    return 1;
  }

  const Eigen::Vector3d expected_position = 0.5 * (previous.p_w_i + next.p_w_i);
  const Eigen::Quaterniond expected_orientation = yaw_quaternion(0.2);
  const double position_error = (optimized.p_w_i - expected_position).norm();
  const double velocity_error = (optimized.v_w_i - previous.v_w_i).norm();
  const double gyro_bias_error = optimized.gyro_bias.norm();
  const double accel_bias_error = optimized.accel_bias.norm();
  const double orientation_error = yaw_error_rad(expected_orientation, optimized.q_w_i);

  std::cout << "sliding_window_smoothness_factor_probe smoothness="
            << summary.smoothness_factor_count
            << " initial_cost=" << summary.initial_cost
            << " final_cost=" << summary.final_cost
            << " position_error=" << position_error
            << " velocity_error=" << velocity_error
            << " orientation_error=" << orientation_error
            << " gyro_bias_error=" << gyro_bias_error
            << " accel_bias_error=" << accel_bias_error
            << " max_linear_jacobian_error=" << max_linear_jacobian_error
            << " numeric_jacobian_blocks=" << summary.numeric_jacobian_block_count
            << " numeric_jacobian_columns=" << summary.numeric_jacobian_column_count << "\n";

  if (!summary.converged || summary.smoothness_factor_count != 1U ||
    summary.final_cost >= summary.initial_cost ||
    position_error > 1.0e-8 || velocity_error > 1.0e-8 ||
    orientation_error > 1.0e-8 || gyro_bias_error > 1.0e-8 ||
    accel_bias_error > 1.0e-8 || max_linear_jacobian_error > 1.0e-12 ||
    summary.numeric_jacobian_block_count != 0U ||
    summary.numeric_jacobian_column_count != 0U)
  {
    std::cerr << "trajectory smoothness factor failed to recover the constant-rate midpoint\n";
    return 1;
  }

  std::cout << "sliding_window_smoothness_factor_probe OK\n";
  return 0;
}
