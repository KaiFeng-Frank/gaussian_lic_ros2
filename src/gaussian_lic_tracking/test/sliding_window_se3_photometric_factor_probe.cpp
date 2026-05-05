// SPDX-License-Identifier: GPL-3.0-or-later

#include <gaussian_lic_tracking/sliding_window_optimizer.hpp>

#include <cmath>
#include <iostream>

#include <Eigen/Geometry>

int main()
{
  gaussian_lic_tracking::SlidingWindowConfig config;
  config.max_iterations = 8;
  gaussian_lic_tracking::SlidingWindowOptimizer optimizer(config);

  gaussian_lic_tracking::SlidingWindowState state;
  state.stamp_ns = 1000000000LL;
  optimizer.add_or_update_state(state);

  gaussian_lic_tracking::SlidingWindowSe3PhotometricFactor factor;
  factor.stamp_ns = state.stamp_ns;
  factor.reference_p_w_i = Eigen::Vector3d::Zero();
  factor.reference_q_w_i = Eigen::Quaterniond::Identity();
  factor.target_delta << 0.01, -0.015, 0.02, 0.12, -0.07, 0.04;
  factor.sqrt_information.setIdentity();
  factor.weight = 25.0;
  optimizer.add_se3_photometric_factor(factor);

  const auto summary = optimizer.optimize();
  gaussian_lic_tracking::SlidingWindowState optimized;
  if (!optimizer.get_state(state.stamp_ns, optimized)) {
    std::cerr << "optimized SE3 photometric state is missing\n";
    return 1;
  }

  const Eigen::AngleAxisd expected_rotation(
    factor.target_delta.template segment<3>(0).norm(),
    factor.target_delta.template segment<3>(0).normalized());
  const Eigen::Quaterniond expected_q(expected_rotation);
  const Eigen::Vector3d expected_p = factor.target_delta.template segment<3>(3);
  const double rotation_error =
    2.0 * (expected_q.inverse() * optimized.q_w_i.normalized()).vec().norm();
  const double translation_error = (optimized.p_w_i - expected_p).norm();
  std::cout << "sliding_window_se3_photometric_factor_probe se3_factors="
            << summary.se3_photometric_factor_count
            << " initial_cost=" << summary.initial_cost
            << " final_cost=" << summary.final_cost
            << " rotation_error=" << rotation_error
            << " translation_error=" << translation_error
            << " optimized_p=" << optimized.p_w_i.transpose() << "\n";

  if (!summary.converged || summary.se3_photometric_factor_count != 1U ||
    summary.final_cost >= summary.initial_cost ||
    rotation_error > 1.0e-6 || translation_error > 1.0e-8)
  {
    std::cerr << "sliding window SE3 photometric factor failed to recover target pose delta\n";
    return 1;
  }

  gaussian_lic_tracking::SlidingWindowOptimizer robust_optimizer(config);
  gaussian_lic_tracking::SlidingWindowState robust_state;
  robust_state.stamp_ns = 2000000000LL;
  robust_optimizer.add_or_update_state(robust_state);

  gaussian_lic_tracking::SlidingWindowSe3PhotometricFactor inlier;
  inlier.stamp_ns = robust_state.stamp_ns;
  inlier.reference_p_w_i = Eigen::Vector3d::Zero();
  inlier.reference_q_w_i = Eigen::Quaterniond::Identity();
  inlier.target_delta.setZero();
  inlier.target_delta[3] = 0.02;
  inlier.sqrt_information.setIdentity();
  inlier.weight = 10.0;
  inlier.huber_delta = 0.1;
  robust_optimizer.add_se3_photometric_factor(inlier);

  gaussian_lic_tracking::SlidingWindowSe3PhotometricFactor outlier = inlier;
  outlier.target_delta[3] = 2.0;
  robust_optimizer.add_se3_photometric_factor(outlier);

  const auto robust_summary = robust_optimizer.optimize();
  gaussian_lic_tracking::SlidingWindowState robust_optimized;
  if (!robust_optimizer.get_state(robust_state.stamp_ns, robust_optimized)) {
    std::cerr << "robust optimized SE3 photometric state is missing\n";
    return 1;
  }
  const double robust_x = robust_optimized.p_w_i.x();
  std::cout << " robust_se3_factors=" << robust_summary.se3_photometric_factor_count
            << " robust_x=" << robust_x
            << " robust_final_cost=" << robust_summary.final_cost;
  if (!robust_summary.converged || robust_summary.se3_photometric_factor_count != 2U ||
    robust_x > 0.3)
  {
    std::cerr << "\nSE3 photometric Huber kernel failed to downweight the outlier\n";
    return 1;
  }

  std::cout << "sliding_window_se3_photometric_factor_probe OK\n";
  return 0;
}
