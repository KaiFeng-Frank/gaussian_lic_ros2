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

Eigen::Quaterniond axis_angle_quaternion(const Eigen::Vector3d & axis, const double angle_rad)
{
  return Eigen::Quaterniond(Eigen::AngleAxisd(angle_rad, axis.normalized()));
}

Eigen::Vector3d quaternion_log_vector(Eigen::Quaterniond quaternion)
{
  quaternion.normalize();
  if (quaternion.w() < 0.0) {
    quaternion.coeffs() *= -1.0;
  }
  const Eigen::Vector3d vector = quaternion.vec();
  const double vector_norm = vector.norm();
  if (vector_norm < 1.0e-12) {
    return 2.0 * vector;
  }
  const double angle = 2.0 * std::atan2(vector_norm, quaternion.w());
  return angle * vector / vector_norm;
}

Eigen::Vector3d relative_rotation_vector(
  const Eigen::Quaterniond & from,
  const Eigen::Quaterniond & to)
{
  return quaternion_log_vector(from.normalized().inverse() * to.normalized());
}

Eigen::Vector3d smoothness_rotation_residual(
  const gaussian_lic_tracking::SlidingWindowTrajectorySmoothnessFactor & factor,
  const gaussian_lic_tracking::SlidingWindowState & previous,
  const gaussian_lic_tracking::SlidingWindowState & current,
  const gaussian_lic_tracking::SlidingWindowState & next)
{
  const double previous_dt_s =
    static_cast<double>(current.stamp_ns - previous.stamp_ns) / 1.0e9;
  const double next_dt_s =
    static_cast<double>(next.stamp_ns - current.stamp_ns) / 1.0e9;
  return std::sqrt(factor.rotation_rate_weight) *
         (relative_rotation_vector(current.q_w_i, next.q_w_i) / next_dt_s -
         relative_rotation_vector(previous.q_w_i, current.q_w_i) / previous_dt_s);
}

void apply_left_rotation_delta(Eigen::Quaterniond & q_w_i, const Eigen::Vector3d & dtheta)
{
  const double angle = dtheta.norm();
  if (angle > 1.0e-12) {
    q_w_i = (Eigen::Quaterniond(Eigen::AngleAxisd(angle, dtheta / angle)) * q_w_i).normalized();
  }
}

Eigen::Matrix<double, 3, 45> finite_difference_smoothness_rotation_jacobian(
  const gaussian_lic_tracking::SlidingWindowTrajectorySmoothnessFactor & factor,
  const gaussian_lic_tracking::SlidingWindowState & previous,
  const gaussian_lic_tracking::SlidingWindowState & current,
  const gaussian_lic_tracking::SlidingWindowState & next)
{
  constexpr double kEpsilon = 1.0e-7;
  Eigen::Matrix<double, 3, 45> jacobian = Eigen::Matrix<double, 3, 45>::Zero();
  for (int state_index = 0; state_index < 3; ++state_index) {
    for (Eigen::Index axis = 0; axis < 3; ++axis) {
      auto plus_previous = previous;
      auto plus_current = current;
      auto plus_next = next;
      auto minus_previous = previous;
      auto minus_current = current;
      auto minus_next = next;
      Eigen::Vector3d dtheta = Eigen::Vector3d::Zero();
      dtheta[axis] = kEpsilon;
      if (state_index == 0) {
        apply_left_rotation_delta(plus_previous.q_w_i, dtheta);
        apply_left_rotation_delta(minus_previous.q_w_i, -dtheta);
      } else if (state_index == 1) {
        apply_left_rotation_delta(plus_current.q_w_i, dtheta);
        apply_left_rotation_delta(minus_current.q_w_i, -dtheta);
      } else {
        apply_left_rotation_delta(plus_next.q_w_i, dtheta);
        apply_left_rotation_delta(minus_next.q_w_i, -dtheta);
      }
      jacobian.col(state_index * 15 + axis) =
        (smoothness_rotation_residual(factor, plus_previous, plus_current, plus_next) -
        smoothness_rotation_residual(factor, minus_previous, minus_current, minus_next)) /
        (2.0 * kEpsilon);
    }
  }
  return jacobian;
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

  gaussian_lic_tracking::SlidingWindowState jacobian_previous;
  jacobian_previous.stamp_ns = 0;
  jacobian_previous.q_w_i =
    axis_angle_quaternion(Eigen::Vector3d{0.7, -0.2, 0.3}, 0.35);
  gaussian_lic_tracking::SlidingWindowState jacobian_current;
  jacobian_current.stamp_ns = 1300000000LL;
  jacobian_current.q_w_i =
    axis_angle_quaternion(Eigen::Vector3d{-0.1, 0.9, 0.4}, -0.55);
  gaussian_lic_tracking::SlidingWindowState jacobian_next;
  jacobian_next.stamp_ns = 2500000000LL;
  jacobian_next.q_w_i =
    axis_angle_quaternion(Eigen::Vector3d{0.2, 0.4, 0.8}, 0.25);
  gaussian_lic_tracking::SlidingWindowTrajectorySmoothnessFactor jacobian_factor;
  jacobian_factor.previous_stamp_ns = jacobian_previous.stamp_ns;
  jacobian_factor.current_stamp_ns = jacobian_current.stamp_ns;
  jacobian_factor.next_stamp_ns = jacobian_next.stamp_ns;
  jacobian_factor.rotation_rate_weight = 7.0;
  jacobian_factor.position_rate_weight = 0.0;
  jacobian_factor.velocity_acceleration_weight = 0.0;
  jacobian_factor.gyro_bias_rate_weight = 0.0;
  jacobian_factor.accel_bias_rate_weight = 0.0;
  gaussian_lic_tracking::SlidingWindowOptimizer jacobian_optimizer(config);
  jacobian_optimizer.add_or_update_state(jacobian_previous);
  jacobian_optimizer.add_or_update_state(jacobian_current);
  jacobian_optimizer.add_or_update_state(jacobian_next);
  jacobian_optimizer.add_trajectory_smoothness_factor(jacobian_factor);
  const auto jacobian_normal = jacobian_optimizer.build_normal_equation(0.0);
  const Eigen::Matrix<double, 3, 45> expected_rotation_jacobian =
    finite_difference_smoothness_rotation_jacobian(
    jacobian_factor, jacobian_previous, jacobian_current, jacobian_next);
  const double max_rotation_jacobian_error =
    (jacobian_normal.jacobian.block(0, 0, 3, 45) - expected_rotation_jacobian)
    .cwiseAbs()
    .maxCoeff();
  if (!jacobian_normal.valid || jacobian_normal.jacobian.rows() != 18 ||
    jacobian_normal.jacobian.cols() != 45 || max_rotation_jacobian_error > 1.0e-5)
  {
    std::cerr << "smoothness analytic rotation Jacobian mismatch: "
              << max_rotation_jacobian_error << "\n";
    return 1;
  }

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
  if (!normal.valid || normal.jacobian.rows() != 18 || normal.jacobian.cols() != 15) {
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
