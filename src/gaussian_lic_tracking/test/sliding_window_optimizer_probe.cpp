// SPDX-License-Identifier: GPL-3.0-or-later

#include <gaussian_lic_tracking/sliding_window_optimizer.hpp>

#include <cmath>
#include <iostream>

int main()
{
  constexpr int steps = 100;
  constexpr int64_t dt_ns = 10000000LL;

  gaussian_lic_tracking::ImuPreintegrator preintegration;
  preintegration.reset(0);
  for (int i = 1; i <= steps; ++i) {
    preintegration.add_measurement(
      static_cast<int64_t>(i) * dt_ns,
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d{1.0, 0.0, 0.0});
  }

  gaussian_lic_tracking::SlidingWindowOptimizer optimizer;
  gaussian_lic_tracking::SlidingWindowState start;
  start.stamp_ns = 0;
  start.fixed = true;
  optimizer.add_or_update_state(start);

  gaussian_lic_tracking::SlidingWindowState end;
  end.stamp_ns = steps * dt_ns;
  end.p_w_i = Eigen::Vector3d{-0.2, 0.1, 0.0};
  end.v_w_i = Eigen::Vector3d{0.0, -0.1, 0.0};
  optimizer.add_or_update_state(end);

  gaussian_lic_tracking::SlidingWindowImuFactor factor;
  factor.from_stamp_ns = start.stamp_ns;
  factor.to_stamp_ns = end.stamp_ns;
  factor.preintegration = preintegration;
  factor.weight = 10.0;
  optimizer.add_imu_factor(factor);

  const auto summary = optimizer.optimize();
  gaussian_lic_tracking::SlidingWindowState optimized;
  if (!optimizer.get_state(end.stamp_ns, optimized)) {
    std::cerr << "optimized endpoint is missing\n";
    return 1;
  }

  const double position_error = (optimized.p_w_i - Eigen::Vector3d{0.5, 0.0, 0.0}).norm();
  const double velocity_error = (optimized.v_w_i - Eigen::Vector3d{1.0, 0.0, 0.0}).norm();
  std::cout << "sliding_window_optimizer_probe states=" << summary.state_count
            << " imu_factors=" << summary.imu_factor_count
            << " normal_rows=" << summary.normal_equation_rows
            << " normal_cols=" << summary.normal_equation_cols
            << " normal_rank=" << summary.normal_equation_rank
            << " normal_condition=" << summary.normal_equation_condition_number
            << " iterations=" << summary.iterations
            << " initial_cost=" << summary.initial_cost
            << " final_cost=" << summary.final_cost
            << " position_error=" << position_error
            << " velocity_error=" << velocity_error << "\n";

  if (!summary.converged || summary.final_cost >= summary.initial_cost ||
    summary.linearization_failure_count != 0U ||
    summary.linear_solve_failure_count != 0U ||
    position_error > 1.0e-8 || velocity_error > 1.0e-8)
  {
    std::cerr << "sliding window optimizer failed to close the IMU factor\n";
    return 1;
  }
  if (summary.normal_equation_rows == 0U || summary.normal_equation_cols == 0U ||
    summary.normal_equation_rank == 0U ||
    summary.normal_equation_min_singular_value <= 0.0 ||
    summary.normal_equation_max_singular_value < summary.normal_equation_min_singular_value ||
    !std::isfinite(summary.normal_equation_condition_number) ||
    summary.normal_equation_degenerate)
  {
    std::cerr << "sliding window optimizer did not report valid normal-equation health\n";
    return 1;
  }

  gaussian_lic_tracking::SlidingWindowConfig guarded_config;
  guarded_config.max_normal_equation_condition = 1.0;
  gaussian_lic_tracking::SlidingWindowOptimizer guarded_optimizer(guarded_config);
  guarded_optimizer.add_or_update_state(start);
  guarded_optimizer.add_or_update_state(end);
  guarded_optimizer.add_imu_factor(factor);
  const auto guarded_summary = guarded_optimizer.optimize();
  if (!guarded_summary.normal_equation_degenerate ||
    guarded_summary.accepted_steps != 0U ||
    guarded_summary.rejected_steps == 0U)
  {
    std::cerr << "sliding window optimizer did not gate an over-conditioned normal equation\n";
    return 1;
  }

  gaussian_lic_tracking::SlidingWindowOptimizer relative_optimizer;
  gaussian_lic_tracking::SlidingWindowState relative_start;
  relative_start.stamp_ns = 0;
  relative_start.fixed = true;
  relative_optimizer.add_or_update_state(relative_start);
  gaussian_lic_tracking::SlidingWindowState relative_end;
  relative_end.stamp_ns = dt_ns;
  relative_end.p_w_i = Eigen::Vector3d{0.1, -0.2, 0.05};
  relative_optimizer.add_or_update_state(relative_end);
  gaussian_lic_tracking::SlidingWindowRelativeTranslationFactor relative_factor;
  relative_factor.from_stamp_ns = relative_start.stamp_ns;
  relative_factor.to_stamp_ns = relative_end.stamp_ns;
  relative_factor.delta_p_w = Eigen::Vector3d{0.4, -0.1, 0.2};
  relative_factor.delta_q_from_to =
    Eigen::Quaterniond(Eigen::AngleAxisd(0.15, Eigen::Vector3d::UnitZ()));
  relative_factor.weight = 25.0;
  relative_factor.huber_delta_m = 0.0;
  relative_factor.rotation_weight = 25.0;
  relative_factor.rotation_huber_delta_rad = 0.0;
  relative_optimizer.add_relative_translation_factor(relative_factor);
  const auto relative_summary = relative_optimizer.optimize();
  gaussian_lic_tracking::SlidingWindowState relative_optimized;
  if (!relative_optimizer.get_state(relative_end.stamp_ns, relative_optimized)) {
    std::cerr << "relative translation endpoint is missing\n";
    return 1;
  }
  const double relative_error =
    (relative_optimized.p_w_i - relative_factor.delta_p_w).norm();
  const double relative_rotation_error =
    relative_optimized.q_w_i.angularDistance(relative_factor.delta_q_from_to);
  if (!relative_summary.converged ||
    relative_summary.relative_translation_factor_count != 1U ||
    relative_summary.relative_translation_factor_cost >= relative_summary.initial_cost ||
    relative_summary.numeric_jacobian_block_count != 0U ||
    relative_summary.numeric_jacobian_column_count != 0U ||
    relative_error > 1.0e-8 ||
    relative_rotation_error > 1.0e-8)
  {
    std::cerr << "relative translation/rotation BA factor failed, translation_error="
              << relative_error << " rotation_error=" << relative_rotation_error
              << " numeric_blocks=" << relative_summary.numeric_jacobian_block_count
              << " numeric_columns=" << relative_summary.numeric_jacobian_column_count << "\n";
    return 1;
  }

  gaussian_lic_tracking::SlidingWindowOptimizer local_relative_optimizer;
  gaussian_lic_tracking::SlidingWindowState local_start;
  local_start.stamp_ns = 0;
  local_start.fixed = true;
  local_start.q_w_i =
    Eigen::Quaterniond(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()));
  local_start.p_w_i = Eigen::Vector3d{1.0, -2.0, 0.5};
  local_relative_optimizer.add_or_update_state(local_start);
  gaussian_lic_tracking::SlidingWindowState local_end;
  local_end.stamp_ns = dt_ns;
  local_end.p_w_i = local_start.p_w_i;
  local_end.q_w_i = local_start.q_w_i;
  local_relative_optimizer.add_or_update_state(local_end);
  gaussian_lic_tracking::SlidingWindowRelativeTranslationFactor local_factor;
  local_factor.from_stamp_ns = local_start.stamp_ns;
  local_factor.to_stamp_ns = local_end.stamp_ns;
  local_factor.delta_p_w = Eigen::Vector3d{0.2, -0.05, 0.1};
  local_factor.delta_q_from_to =
    Eigen::Quaterniond(Eigen::AngleAxisd(-0.1, Eigen::Vector3d::UnitY()));
  local_factor.translation_in_from_frame = true;
  local_factor.weight = 25.0;
  local_factor.huber_delta_m = 0.0;
  local_factor.rotation_weight = 25.0;
  local_factor.rotation_huber_delta_rad = 0.0;
  local_relative_optimizer.add_relative_translation_factor(local_factor);
  const auto local_relative_summary = local_relative_optimizer.optimize();
  gaussian_lic_tracking::SlidingWindowState local_relative_optimized;
  if (!local_relative_optimizer.get_state(local_end.stamp_ns, local_relative_optimized)) {
    std::cerr << "local relative endpoint is missing\n";
    return 1;
  }
  const Eigen::Vector3d expected_local_p =
    local_start.p_w_i + local_start.q_w_i * local_factor.delta_p_w;
  const Eigen::Quaterniond expected_local_q =
    (local_start.q_w_i * local_factor.delta_q_from_to).normalized();
  const double local_relative_error =
    (local_relative_optimized.p_w_i - expected_local_p).norm();
  const double local_relative_rotation_error =
    local_relative_optimized.q_w_i.angularDistance(expected_local_q);
  if (!local_relative_summary.converged ||
    local_relative_summary.relative_translation_factor_count != 1U ||
    local_relative_summary.numeric_jacobian_block_count != 0U ||
    local_relative_summary.numeric_jacobian_column_count != 0U ||
    local_relative_error > 1.0e-8 ||
    local_relative_rotation_error > 1.0e-8)
  {
    std::cerr << "local-frame relative BA factor failed, translation_error="
              << local_relative_error << " rotation_error="
              << local_relative_rotation_error
              << " numeric_blocks=" << local_relative_summary.numeric_jacobian_block_count
              << " numeric_columns=" << local_relative_summary.numeric_jacobian_column_count
              << "\n";
    return 1;
  }
  std::cout << "sliding_window_optimizer_probe OK\n";
  return 0;
}
