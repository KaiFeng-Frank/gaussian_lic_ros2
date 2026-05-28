// SPDX-License-Identifier: GPL-3.0-or-later

#include <gaussian_lic_tracking/sliding_window_optimizer.hpp>

#include <algorithm>
#include <cmath>
#include <exception>
#include <iostream>

int main()
{
  gaussian_lic_tracking::SlidingWindowConfig config;
  config.max_iterations = 4;
  gaussian_lic_tracking::SlidingWindowOptimizer optimizer(config);

  gaussian_lic_tracking::SlidingWindowState state;
  state.stamp_ns = 100;
  state.p_w_i = Eigen::Vector3d{0.0, 0.0, 0.0};
  optimizer.add_or_update_state(state);

  gaussian_lic_tracking::SlidingWindowVisualAlignmentFactor factor;
  factor.stamp_ns = state.stamp_ns;
  factor.reference_p_w_i = state.p_w_i;
  factor.measured_shift_px = Eigen::Vector2d{12.0, -7.0};
  factor.meters_per_pixel = 0.01;
  factor.weight = 50.0;
  optimizer.add_visual_alignment_factor(factor);
  optimizer.add_visual_alignment_factor(factor);

  const auto summary = optimizer.optimize();
  gaussian_lic_tracking::SlidingWindowState optimized;
  if (!optimizer.get_state(state.stamp_ns, optimized)) {
    std::cerr << "optimized visual-factor state is missing\n";
    return 1;
  }

  const Eigen::Vector3d expected{0.12, -0.07, 0.0};
  const double xy_error = (optimized.p_w_i.head<2>() - expected.head<2>()).norm();
  std::cout << "sliding_window_visual_factor_probe visual_factors="
            << summary.visual_factor_count
            << " initial_cost=" << summary.initial_cost
            << " final_cost=" << summary.final_cost
            << " xy_error=" << xy_error
            << " optimized_p=" << optimized.p_w_i.transpose() << "\n";

  if (!summary.converged || summary.visual_factor_count != 1U ||
    summary.visual_factor_replacement_count != 1U ||
    summary.final_cost >= summary.initial_cost || xy_error > 1.0e-8)
  {
    std::cerr << "sliding window visual alignment factor failed to recover planar correction\n";
    return 1;
  }

  gaussian_lic_tracking::SlidingWindowOptimizer robust_optimizer(config);
  gaussian_lic_tracking::SlidingWindowState robust_state;
  robust_state.stamp_ns = 200;
  robust_state.p_w_i = Eigen::Vector3d::Zero();
  robust_optimizer.add_or_update_state(robust_state);

  gaussian_lic_tracking::SlidingWindowVisualAlignmentFactor inlier;
  inlier.stamp_ns = robust_state.stamp_ns;
  inlier.reference_p_w_i = robust_state.p_w_i;
  inlier.measured_shift_px = Eigen::Vector2d{2.0, 0.0};
  inlier.meters_per_pixel = 0.01;
  inlier.weight = 10.0;
  inlier.huber_delta_m = 0.05;
  robust_optimizer.add_visual_alignment_factor(inlier);

  gaussian_lic_tracking::SlidingWindowVisualAlignmentFactor outlier = inlier;
  outlier.source_id = 1U;
  outlier.measured_shift_px = Eigen::Vector2d{200.0, 0.0};
  robust_optimizer.add_visual_alignment_factor(outlier);

  const auto robust_summary = robust_optimizer.optimize();
  gaussian_lic_tracking::SlidingWindowState robust_optimized;
  if (!robust_optimizer.get_state(robust_state.stamp_ns, robust_optimized)) {
    std::cerr << "robust optimized visual-factor state is missing\n";
    return 1;
  }
  const double robust_x = robust_optimized.p_w_i.x();
  std::cout << " robust_visual_factors=" << robust_summary.visual_factor_count
            << " robust_x=" << robust_x
            << " robust_final_cost=" << robust_summary.final_cost;
  if (!robust_summary.converged || robust_summary.visual_factor_count != 2U ||
    robust_x > 0.2)
  {
    std::cerr << "\nvisual alignment Huber kernel failed to downweight the outlier\n";
    return 1;
  }

  gaussian_lic_tracking::SlidingWindowConfig limited_config;
  limited_config.max_iterations = 1;
  limited_config.max_translation_step_m = 0.05;
  gaussian_lic_tracking::SlidingWindowOptimizer limited_optimizer(limited_config);
  gaussian_lic_tracking::SlidingWindowState limited_state;
  limited_state.stamp_ns = 300;
  limited_state.p_w_i = Eigen::Vector3d::Zero();
  limited_optimizer.add_or_update_state(limited_state);
  gaussian_lic_tracking::SlidingWindowVisualAlignmentFactor limited_factor;
  limited_factor.stamp_ns = limited_state.stamp_ns;
  limited_factor.reference_p_w_i = limited_state.p_w_i;
  limited_factor.measured_shift_px = Eigen::Vector2d{100.0, 0.0};
  limited_factor.meters_per_pixel = 0.01;
  limited_factor.weight = 100.0;
  limited_optimizer.add_visual_alignment_factor(limited_factor);
  const auto limited_summary = limited_optimizer.optimize();
  gaussian_lic_tracking::SlidingWindowState limited_optimized;
  if (!limited_optimizer.get_state(limited_state.stamp_ns, limited_optimized)) {
    std::cerr << "step-limited visual-factor state is missing\n";
    return 1;
  }
  const double limited_translation = limited_optimized.p_w_i.norm();
  std::cout << " limited_translation=" << limited_translation
            << " limited_steps=" << limited_summary.limited_steps
            << " limited_final_cost=" << limited_summary.final_cost;
  if (!limited_summary.converged || limited_summary.limited_steps != 1U ||
    limited_translation > 0.050001)
  {
    std::cerr << "\nsliding window translation step limiter failed\n";
    return 1;
  }

  gaussian_lic_tracking::SlidingWindowOptimizer interpolation_optimizer(config);
  gaussian_lic_tracking::SlidingWindowState interpolation_a;
  interpolation_a.stamp_ns = 400;
  interpolation_a.p_w_i = Eigen::Vector3d::Zero();
  gaussian_lic_tracking::SlidingWindowState interpolation_b = interpolation_a;
  interpolation_b.stamp_ns = 500;
  interpolation_optimizer.add_or_update_state(interpolation_a);
  interpolation_optimizer.add_or_update_state(interpolation_b);

  gaussian_lic_tracking::SlidingWindowVisualAlignmentFactor interpolation_factor;
  interpolation_factor.stamp_ns = 450;
  interpolation_factor.source_id = 7U;
  interpolation_factor.support_stamp_ns = {interpolation_a.stamp_ns, interpolation_b.stamp_ns};
  interpolation_factor.support_weights = {0.25, 0.75};
  interpolation_factor.reference_p_w_i = Eigen::Vector3d::Zero();
  interpolation_factor.measured_shift_px = Eigen::Vector2d{40.0, -20.0};
  interpolation_factor.meters_per_pixel = 0.01;
  interpolation_factor.weight = 50.0;
  interpolation_optimizer.add_visual_alignment_factor(interpolation_factor);
  const auto interpolation_normal = interpolation_optimizer.build_normal_equation();
  const double interpolation_scale = std::sqrt(interpolation_factor.weight);
  const double interpolation_jacobian_error = std::max(
    std::max(
      std::abs(interpolation_normal.jacobian(0, 6) - 0.25 * interpolation_scale),
      std::abs(interpolation_normal.jacobian(1, 7) - 0.25 * interpolation_scale)),
    std::max(
      std::abs(interpolation_normal.jacobian(0, 21) - 0.75 * interpolation_scale),
      std::abs(interpolation_normal.jacobian(1, 22) - 0.75 * interpolation_scale)));
  const auto interpolation_summary = interpolation_optimizer.optimize();
  gaussian_lic_tracking::SlidingWindowState interpolation_a_out;
  gaussian_lic_tracking::SlidingWindowState interpolation_b_out;
  if (!interpolation_optimizer.get_state(interpolation_a.stamp_ns, interpolation_a_out) ||
    !interpolation_optimizer.get_state(interpolation_b.stamp_ns, interpolation_b_out))
  {
    std::cerr << "\ninterpolated visual-factor states are missing\n";
    return 1;
  }
  const Eigen::Vector2d interpolation_target{0.4, -0.2};
  const Eigen::Vector2d interpolation_xy =
    (0.25 * interpolation_a_out.p_w_i + 0.75 * interpolation_b_out.p_w_i).head<2>();
  const double interpolation_error = (interpolation_xy - interpolation_target).norm();
  std::cout << " interpolated_xy_error=" << interpolation_error
            << " interpolated_numeric_blocks="
            << interpolation_summary.numeric_jacobian_block_count
            << " interpolated_jacobian_error=" << interpolation_jacobian_error;
	  if (!interpolation_normal.valid || interpolation_normal.numeric_jacobian_block_count != 0U ||
	    interpolation_jacobian_error > 1.0e-12 || !interpolation_summary.converged ||
	    interpolation_summary.numeric_jacobian_block_count != 0U ||
	    interpolation_summary.final_cost >= interpolation_summary.initial_cost ||
	    interpolation_error > 1.0e-7)
	  {
	    std::cerr << "\ninterpolated visual factor failed analytic continuous-time support solve\n";
	    return 1;
	  }
	  gaussian_lic_tracking::SlidingWindowOptimizer masked_optimizer(config);
	  gaussian_lic_tracking::SlidingWindowState masked_state;
	  masked_state.stamp_ns = 600;
	  masked_state.p_w_i = Eigen::Vector3d::Zero();
	  masked_optimizer.add_or_update_state(masked_state);
	  gaussian_lic_tracking::SlidingWindowVisualAlignmentFactor masked_factor;
	  masked_factor.stamp_ns = masked_state.stamp_ns;
	  masked_factor.reference_p_w_i = masked_state.p_w_i;
	  masked_factor.measured_shift_px = Eigen::Vector2d{50.0, -30.0};
	  masked_factor.component_weight_xy = Eigen::Vector2d{0.0, 1.0};
	  masked_factor.meters_per_pixel = 0.01;
	  masked_factor.weight = 50.0;
	  masked_optimizer.add_visual_alignment_factor(masked_factor);
	  const auto masked_normal = masked_optimizer.build_normal_equation();
	  const auto masked_summary = masked_optimizer.optimize();
	  gaussian_lic_tracking::SlidingWindowState masked_out;
	  if (!masked_optimizer.get_state(masked_state.stamp_ns, masked_out)) {
	    std::cerr << "\nmasked visual-factor state is missing\n";
	    return 1;
	  }
	  const double masked_x_abs = std::abs(masked_out.p_w_i.x());
	  const double masked_y_error = std::abs(masked_out.p_w_i.y() + 0.3);
	  bool rejected_zero_axis_factor = false;
	  try {
	    gaussian_lic_tracking::SlidingWindowVisualAlignmentFactor invalid_masked_factor =
	      masked_factor;
	    invalid_masked_factor.source_id = 1U;
	    invalid_masked_factor.component_weight_xy = Eigen::Vector2d::Zero();
	    masked_optimizer.add_visual_alignment_factor(invalid_masked_factor);
	  } catch (const std::exception &) {
	    rejected_zero_axis_factor = true;
	  }
	  std::cout << " masked_x_abs=" << masked_x_abs
	            << " masked_y_error=" << masked_y_error
	            << " masked_jacobian_x=" << masked_normal.jacobian(0, 6)
	            << " masked_jacobian_y=" << masked_normal.jacobian(1, 7);
	  if (!masked_normal.valid || std::abs(masked_normal.jacobian(0, 6)) > 1.0e-12 ||
	    std::abs(masked_normal.jacobian(1, 7) - std::sqrt(masked_factor.weight)) > 1.0e-12 ||
	    !masked_summary.converged || masked_x_abs > 1.0e-12 || masked_y_error > 1.0e-8 ||
	    !rejected_zero_axis_factor)
	  {
	    std::cerr << "\nvisual component axis mask failed\n";
	    return 1;
	  }
	  std::cout << "sliding_window_visual_factor_probe OK\n";
	  return 0;
	}
