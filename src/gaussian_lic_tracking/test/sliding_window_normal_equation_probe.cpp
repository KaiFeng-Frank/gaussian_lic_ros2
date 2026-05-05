// SPDX-License-Identifier: GPL-3.0-or-later

#include <gaussian_lic_tracking/sliding_window_optimizer.hpp>

#include <iostream>

int main()
{
  gaussian_lic_tracking::SlidingWindowConfig config;
  config.numeric_epsilon = 1.0e-7;
  gaussian_lic_tracking::SlidingWindowOptimizer optimizer(config);

  gaussian_lic_tracking::SlidingWindowState state;
  state.stamp_ns = 100;
  state.v_w_i = Eigen::Vector3d{0.2, -0.1, 0.05};
  state.p_w_i = Eigen::Vector3d{1.0, -2.0, 0.5};
  state.gyro_bias = Eigen::Vector3d{0.01, -0.02, 0.03};
  state.accel_bias = Eigen::Vector3d{-0.1, 0.05, 0.2};
  optimizer.add_or_update_state(state);

  gaussian_lic_tracking::SlidingWindowStatePrior prior;
  prior.stamp_ns = state.stamp_ns;
  prior.sqrt_information = Eigen::Matrix<double, 15, 15>::Identity();
  optimizer.add_state_prior(prior);

  const auto normal = optimizer.build_normal_equation(0.0);
  if (!normal.valid || normal.variable_count != 15U || normal.residual.size() != 15 ||
    normal.jacobian.rows() != 15 || normal.jacobian.cols() != 15 ||
    normal.hessian.rows() != 15 || normal.hessian.cols() != 15 ||
    normal.rhs.size() != 15)
  {
    std::cerr << "normal-equation dimensions are invalid\n";
    return 1;
  }
  const Eigen::VectorXd step = normal.hessian.ldlt().solve(normal.rhs);
  if (!step.allFinite()) {
    std::cerr << "normal-equation step is invalid\n";
    return 1;
  }
  const double symmetry_error = (normal.hessian - normal.hessian.transpose()).norm();
  const double velocity_step_error = (step.segment<3>(3) + state.v_w_i).norm();
  const double position_step_error = (step.segment<3>(6) + state.p_w_i).norm();
  const double gyro_bias_step_error = (step.segment<3>(9) + state.gyro_bias).norm();
  const double accel_bias_step_error = (step.segment<3>(12) + state.accel_bias).norm();
  std::cout << "sliding_window_normal_equation_probe cost=" << normal.cost
            << " symmetry_error=" << symmetry_error
            << " velocity_step_error=" << velocity_step_error
            << " position_step_error=" << position_step_error
            << " gyro_bias_step_error=" << gyro_bias_step_error
            << " accel_bias_step_error=" << accel_bias_step_error << "\n";
  if (normal.cost <= 0.0 || symmetry_error > 1.0e-12 ||
    velocity_step_error > 1.0e-7 || position_step_error > 1.0e-7 ||
    gyro_bias_step_error > 1.0e-7 || accel_bias_step_error > 1.0e-7)
  {
    std::cerr << "normal-equation step does not match the state-prior residual\n";
    return 1;
  }
  std::cout << "sliding_window_normal_equation_probe OK\n";
  return 0;
}
