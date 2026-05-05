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
    summary.point_factor_cost > 1.0e-18 ||
    position_error > 1.0e-7 || rotation_error_norm > 1.0e-7)
  {
    std::cerr << "sliding window point-to-point factor failed to recover the pose\n";
    return 1;
  }

  gaussian_lic_tracking::SlidingWindowOptimizer robust_optimizer(config);
  gaussian_lic_tracking::SlidingWindowState robust_state;
  robust_state.stamp_ns = 200;
  robust_optimizer.add_or_update_state(robust_state);
  gaussian_lic_tracking::SlidingWindowPosePrior rotation_prior;
  rotation_prior.stamp_ns = robust_state.stamp_ns;
  rotation_prior.q_w_i = Eigen::Quaterniond::Identity();
  rotation_prior.rotation_weight = 100.0;
  rotation_prior.translation_weight = 0.0;
  robust_optimizer.add_pose_prior(rotation_prior);

  gaussian_lic_tracking::SlidingWindowPointToPointFactor robust_factor;
  robust_factor.stamp_ns = robust_state.stamp_ns;
  robust_factor.weight = 80.0;
  robust_factor.huber_delta_m = 0.05;
  const Eigen::Vector3d robust_true_p{0.35, -0.2, 0.1};
  for (int i = 0; i < 24; ++i) {
    const Eigen::Vector3d point_i{
      0.02 * static_cast<double>(i % 6),
      0.03 * static_cast<double>(i / 6),
      0.01 * static_cast<double>(i % 3)};
    robust_factor.frame_points_i.push_back(point_i);
    robust_factor.target_points_w.push_back(point_i + robust_true_p);
    robust_factor.point_weights.push_back(1.0);
  }
  robust_factor.frame_points_i.emplace_back(0.1, -0.2, 0.05);
  robust_factor.target_points_w.emplace_back(4.0, -3.0, 2.0);
  robust_factor.point_weights.push_back(1.0);
  robust_optimizer.add_point_to_point_factor(robust_factor);

  const auto robust_summary = robust_optimizer.optimize();
  gaussian_lic_tracking::SlidingWindowState robust_optimized;
  if (!robust_optimizer.get_state(robust_state.stamp_ns, robust_optimized)) {
    std::cerr << "robust optimized point-factor state is missing\n";
    return 1;
  }
  const double robust_position_error = (robust_optimized.p_w_i - robust_true_p).norm();
  std::cout << "sliding_window_point_factor_robust_probe initial_cost="
            << robust_summary.initial_cost
            << " final_cost=" << robust_summary.final_cost
            << " point_cost=" << robust_summary.point_factor_cost
            << " pose_prior_cost=" << robust_summary.pose_prior_cost
            << " position_error=" << robust_position_error << "\n";
  const double robust_cost_sum =
    robust_summary.point_factor_cost + robust_summary.pose_prior_cost;
  if (!robust_summary.converged || robust_summary.final_cost >= robust_summary.initial_cost ||
    robust_summary.point_factor_cost <= 0.0 ||
    std::abs(robust_summary.final_cost - robust_cost_sum) > 1.0e-9 ||
    robust_position_error > 0.03)
  {
    std::cerr << "dynamic Huber point-to-point weighting failed to reject an outlier\n";
    return 1;
  }

  std::cout << "sliding_window_point_factor_probe OK\n";
  return 0;
}
