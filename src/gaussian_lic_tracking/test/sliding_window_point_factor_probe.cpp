// SPDX-License-Identifier: GPL-3.0-or-later

#include <gaussian_lic_tracking/sliding_window_optimizer.hpp>

#include <cmath>
#include <iostream>

int main()
{
  gaussian_lic_tracking::SlidingWindowConfig config;
  config.max_iterations = 8;
  gaussian_lic_tracking::SlidingWindowOptimizer optimizer(config);

  const Eigen::Quaterniond true_q(Eigen::AngleAxisd(0.18, Eigen::Vector3d::UnitZ()));
  const Eigen::Vector3d true_p(1.0, -0.5, 0.25);
  std::vector<Eigen::Vector3d> frame_points{
    Eigen::Vector3d{0.0, 0.0, 0.0},
    Eigen::Vector3d{1.0, 0.0, 0.0},
    Eigen::Vector3d{0.0, 1.0, 0.0},
    Eigen::Vector3d{0.0, 0.0, 1.0},
    Eigen::Vector3d{0.5, -0.2, 0.3}};

  gaussian_lic_tracking::SlidingWindowState state;
  state.stamp_ns = 100;
  state.p_w_i = Eigen::Vector3d{0.2, -0.1, 0.0};
  state.q_w_i = Eigen::Quaterniond(Eigen::AngleAxisd(-0.05, Eigen::Vector3d::UnitZ()));
  optimizer.add_or_update_state(state);

  gaussian_lic_tracking::SlidingWindowPointToPointFactor factor;
  factor.stamp_ns = state.stamp_ns;
  factor.frame_points_i = frame_points;
  factor.weight = 20.0;
  for (const auto & point_i : frame_points) {
    factor.target_points_w.push_back(true_q * point_i + true_p);
  }
  optimizer.add_point_to_point_factor(factor);

  const auto summary = optimizer.optimize();
  gaussian_lic_tracking::SlidingWindowState optimized;
  if (!optimizer.get_state(state.stamp_ns, optimized)) {
    std::cerr << "optimized point-factor state is missing\n";
    return 1;
  }
  const double position_error = (optimized.p_w_i - true_p).norm();
  const Eigen::Quaterniond rotation_error = (true_q.inverse() * optimized.q_w_i).normalized();
  const double rotation_error_norm = 2.0 * rotation_error.vec().norm();
  std::cout << "sliding_window_point_factor_probe point_factors=" << summary.point_factor_count
            << " initial_cost=" << summary.initial_cost
            << " final_cost=" << summary.final_cost
            << " position_error=" << position_error
            << " rotation_error_norm=" << rotation_error_norm << "\n";

  if (!summary.converged || summary.point_factor_count != 1U ||
    summary.final_cost >= summary.initial_cost ||
    position_error > 1.0e-7 || rotation_error_norm > 1.0e-7)
  {
    std::cerr << "sliding window point-to-point factor failed to recover the pose\n";
    return 1;
  }
  std::cout << "sliding_window_point_factor_probe OK\n";
  return 0;
}
