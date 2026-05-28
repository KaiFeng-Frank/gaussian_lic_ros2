// SPDX-License-Identifier: GPL-3.0-or-later

#include <gaussian_lic_tracking/sliding_window_optimizer.hpp>

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace
{

gaussian_lic_tracking::SlidingWindowState make_state(const int64_t stamp_ns, const double x)
{
  gaussian_lic_tracking::SlidingWindowState state;
  state.stamp_ns = stamp_ns;
  state.p_w_i = Eigen::Vector3d{x, 0.1 * x, -0.05 * x};
  state.v_w_i = Eigen::Vector3d{0.2, -0.1, 0.05};
  state.gyro_bias = Eigen::Vector3d{0.01, -0.02, 0.005};
  state.accel_bias = Eigen::Vector3d{0.1, -0.05, 0.02};
  return state;
}

gaussian_lic_tracking::SlidingWindowStatePrior make_full_state_prior(
  const gaussian_lic_tracking::SlidingWindowState & state)
{
  gaussian_lic_tracking::SlidingWindowStatePrior prior;
  prior.stamp_ns = state.stamp_ns;
  prior.p_w_i = state.p_w_i;
  prior.q_w_i = state.q_w_i;
  prior.v_w_i = state.v_w_i;
  prior.gyro_bias = state.gyro_bias;
  prior.accel_bias = state.accel_bias;
  prior.sqrt_information = Eigen::Matrix<double, 15, 15>::Identity();
  prior.rotation_weight = 1.0;
  prior.velocity_weight = 1.0;
  prior.position_weight = 1.0;
  prior.gyro_bias_weight = 1.0;
  prior.accel_bias_weight = 1.0;
  return prior;
}

bool check_deferred_marginalization_includes_current_frame_priors()
{
  gaussian_lic_tracking::SlidingWindowConfig config;
  config.max_states = 3;
  config.max_iterations = 1;
  config.marginalization_prior_weight = 0.0;
  gaussian_lic_tracking::SlidingWindowOptimizer optimizer(config);

  for (int i = 0; i < 4; ++i) {
    const auto state = make_state(static_cast<int64_t>(i), static_cast<double>(i));
    optimizer.add_or_update_state(state);
    optimizer.add_state_prior(make_full_state_prior(state));
  }

  const auto summary = optimizer.optimize();
  std::cout << "deferred_marginalization_prior_probe states=" << summary.state_count
            << " dense_priors=" << summary.dense_prior_count
            << " dense_prior_cols=" << summary.dense_prior_cols
            << " dense_prior_rank=" << summary.dense_prior_rank
            << " marginalized=" << summary.marginalized_state_count
            << " schur=" << summary.schur_marginalization_count << "\n";
  if (summary.state_count != config.max_states || summary.dense_prior_count != 1U ||
    summary.dense_prior_cols != 45U || summary.dense_prior_rank != 45U ||
    summary.marginalized_state_count != 1U || summary.schur_marginalization_count != 1U)
  {
    std::cerr << "deferred marginalization did not include current-frame priors\n";
    return false;
  }
  return true;
}

gaussian_lic_tracking::SlidingWindowRelativeTranslationFactor make_relative_factor(
  const int64_t from_stamp_ns,
  const int64_t to_stamp_ns,
  const Eigen::Vector3d & delta_p_w)
{
  gaussian_lic_tracking::SlidingWindowRelativeTranslationFactor factor;
  factor.from_stamp_ns = from_stamp_ns;
  factor.to_stamp_ns = to_stamp_ns;
  factor.delta_p_w = delta_p_w;
  factor.weight = 10.0;
  factor.huber_delta_m = 1.0;
  return factor;
}

bool check_chained_marginalized_prior_survives_retained_state_marginalization()
{
  gaussian_lic_tracking::SlidingWindowConfig config;
  config.max_states = 2;
  config.max_iterations = 1;
  config.marginalization_prior_weight = 0.0;
  gaussian_lic_tracking::SlidingWindowOptimizer optimizer(config);

  std::vector<gaussian_lic_tracking::SlidingWindowState> states;
  for (int i = 0; i < 4; ++i) {
    states.push_back(make_state(static_cast<int64_t>(i), static_cast<double>(i)));
    optimizer.add_or_update_state(states.back());
    optimizer.add_state_prior(make_full_state_prior(states.back()));
    if (i > 0) {
      optimizer.add_relative_translation_factor(
        make_relative_factor(
          states[static_cast<size_t>(i - 1)].stamp_ns,
          states[static_cast<size_t>(i)].stamp_ns,
          states[static_cast<size_t>(i)].p_w_i - states[static_cast<size_t>(i - 1)].p_w_i));
    }
    if (i == 2) {
      const auto first_summary = optimizer.optimize();
      if (first_summary.marginalized_backsubstitution_count == 0U) {
        std::cerr << "first marginalization did not retain a back-substitution record\n";
        return false;
      }
    }
  }

  const auto second_summary = optimizer.optimize();
  if (second_summary.marginalized_backsubstitution_chain_update_count == 0U) {
    std::cerr << "second marginalization did not propagate the old back-substitution record\n";
    return false;
  }

  gaussian_lic_tracking::SlidingWindowVisualAlignmentFactor late_visual;
  late_visual.stamp_ns = states.front().stamp_ns;
  late_visual.source_id = 21U;
  late_visual.measured_shift_px = Eigen::Vector2d{0.25, -0.1};
  late_visual.meters_per_pixel = 0.05;
  late_visual.weight = 1.0;
  late_visual.huber_delta_m = 1.0;
  if (!optimizer.add_marginalized_visual_alignment_prior(late_visual)) {
    std::cerr << "chained late visual factor did not survive retained-state marginalization\n";
    return false;
  }

  const auto late_summary = optimizer.optimize();
  std::cout << "chained_marginalized_visual_prior_probe active_backsubs="
            << late_summary.marginalized_backsubstitution_count
            << " chain_updates="
            << late_summary.marginalized_backsubstitution_chain_update_count
            << " visual_marg_priors="
            << late_summary.visual_marginalization_prior_count << "\n";
  if (late_summary.marginalized_backsubstitution_chain_update_count == 0U ||
    late_summary.visual_marginalization_prior_count == 0U)
  {
    std::cerr << "chained marginalized prior was not retained in the BA window\n";
    return false;
  }
  return true;
}

bool check_interpolated_marginalized_prior_preserves_continuous_time_stamp()
{
  gaussian_lic_tracking::SlidingWindowConfig config;
  config.max_states = 2;
  config.max_iterations = 1;
  config.marginalization_prior_weight = 0.0;
  config.max_state_gap_s = 2.0;
  gaussian_lic_tracking::SlidingWindowOptimizer optimizer(config);

  constexpr int64_t dt_ns = 1000000000LL;
  std::vector<gaussian_lic_tracking::SlidingWindowState> states;
  for (int i = 0; i < 5; ++i) {
    states.push_back(make_state(static_cast<int64_t>(i) * dt_ns, static_cast<double>(i)));
    optimizer.add_or_update_state(states.back());
    optimizer.add_state_prior(make_full_state_prior(states.back()));
    if (i > 0) {
      optimizer.add_relative_translation_factor(
        make_relative_factor(
          states[static_cast<size_t>(i - 1)].stamp_ns,
          states[static_cast<size_t>(i)].stamp_ns,
          states[static_cast<size_t>(i)].p_w_i - states[static_cast<size_t>(i - 1)].p_w_i));
    }
  }

  const auto marginalization_summary = optimizer.optimize();
  if (marginalization_summary.marginalized_backsubstitution_count < 2U) {
    std::cerr << "not enough marginalized back-substitution records for interpolation\n";
    return false;
  }

  gaussian_lic_tracking::SlidingWindowVisualAlignmentFactor interpolated_visual;
  interpolated_visual.stamp_ns = states.front().stamp_ns + dt_ns / 2;
  interpolated_visual.source_id = 31U;
  interpolated_visual.measured_shift_px = Eigen::Vector2d{0.2, -0.15};
  interpolated_visual.meters_per_pixel = 0.04;
  interpolated_visual.weight = 1.0;
  interpolated_visual.huber_delta_m = 1.0;
  if (!optimizer.add_marginalized_visual_alignment_prior(interpolated_visual)) {
    std::cerr << "interpolated late visual factor did not build a marginalized prior\n";
    return false;
  }

  const auto late_summary = optimizer.optimize();
  std::cout << "interpolated_marginalized_visual_prior_probe active_backsubs="
            << late_summary.marginalized_backsubstitution_count
            << " interpolations="
            << late_summary.marginalized_backsubstitution_interpolation_count
            << " visual_marg_priors="
            << late_summary.visual_marginalization_prior_count << "\n";
  if (late_summary.marginalized_backsubstitution_interpolation_count == 0U ||
    late_summary.visual_marginalization_prior_count == 0U)
  {
    std::cerr << "interpolated marginalized prior was not retained in the BA window\n";
    return false;
  }
  return true;
}

bool check_marginalized_active_boundary_prior_preserves_continuous_time_stamp()
{
  gaussian_lic_tracking::SlidingWindowConfig config;
  config.max_states = 2;
  config.max_iterations = 1;
  config.marginalization_prior_weight = 0.0;
  config.max_state_gap_s = 2.0;
  gaussian_lic_tracking::SlidingWindowOptimizer optimizer(config);

  constexpr int64_t dt_ns = 1000000000LL;
  std::vector<gaussian_lic_tracking::SlidingWindowState> states;
  for (int i = 0; i < 3; ++i) {
    states.push_back(make_state(static_cast<int64_t>(i) * dt_ns, static_cast<double>(i)));
    optimizer.add_or_update_state(states.back());
    optimizer.add_state_prior(make_full_state_prior(states.back()));
    if (i > 0) {
      optimizer.add_relative_translation_factor(
        make_relative_factor(
          states[static_cast<size_t>(i - 1)].stamp_ns,
          states[static_cast<size_t>(i)].stamp_ns,
          states[static_cast<size_t>(i)].p_w_i - states[static_cast<size_t>(i - 1)].p_w_i));
    }
  }

  const auto marginalization_summary = optimizer.optimize();
  if (marginalization_summary.marginalized_backsubstitution_count == 0U) {
    std::cerr << "missing marginalized back-substitution record for active-boundary interpolation\n";
    return false;
  }

  gaussian_lic_tracking::SlidingWindowVisualAlignmentFactor boundary_visual;
  boundary_visual.stamp_ns = states.front().stamp_ns + dt_ns / 2;
  boundary_visual.source_id = 41U;
  boundary_visual.measured_shift_px = Eigen::Vector2d{-0.15, 0.25};
  boundary_visual.meters_per_pixel = 0.03;
  boundary_visual.weight = 1.0;
  boundary_visual.huber_delta_m = 1.0;
  if (!optimizer.add_marginalized_visual_alignment_prior(boundary_visual)) {
    std::cerr << "marginalized-active boundary visual factor did not build a prior\n";
    return false;
  }

  gaussian_lic_tracking::SlidingWindowSe3PhotometricFactor boundary_se3;
  boundary_se3.stamp_ns = states.front().stamp_ns + dt_ns / 2;
  boundary_se3.source_id = 42U;
  boundary_se3.target_delta = Eigen::Matrix<double, 6, 1>::Zero();
  boundary_se3.target_delta(0) = 0.005;
  boundary_se3.target_delta(3) = -0.02;
  boundary_se3.sqrt_information = Eigen::Matrix<double, 6, 6>::Identity();
  boundary_se3.weight = 1.0;
  boundary_se3.huber_delta = 1.0;
  if (!optimizer.add_marginalized_se3_photometric_prior(boundary_se3)) {
    std::cerr << "marginalized-active boundary SE3 factor did not build a prior\n";
    return false;
  }

  const auto late_summary = optimizer.optimize();
  std::cout << "marginalized_active_boundary_visual_se3_prior_probe active_backsubs="
            << late_summary.marginalized_backsubstitution_count
            << " interpolations="
            << late_summary.marginalized_backsubstitution_interpolation_count
            << " visual_marg_priors="
            << late_summary.visual_marginalization_prior_count
            << " se3_marg_priors="
            << late_summary.se3_photometric_marginalization_prior_count << "\n";
  if (late_summary.marginalized_backsubstitution_interpolation_count < 2U ||
    late_summary.visual_marginalization_prior_count == 0U ||
    late_summary.se3_photometric_marginalization_prior_count == 0U)
  {
    std::cerr << "marginalized-active boundary priors were not retained in the BA window\n";
    return false;
  }
  return true;
}

bool check_visual_marginalization_prior_can_remove_bias_columns()
{
  gaussian_lic_tracking::SlidingWindowConfig config;
  config.max_states = 2;
  config.max_iterations = 1;
  config.marginalization_prior_weight = 0.0;
  config.visual_marginalization_prior_zero_bias_columns = true;
  gaussian_lic_tracking::SlidingWindowOptimizer optimizer(config);

  std::vector<gaussian_lic_tracking::SlidingWindowState> states;
  for (int i = 0; i < 3; ++i) {
    states.push_back(make_state(static_cast<int64_t>(i), static_cast<double>(i)));
    optimizer.add_or_update_state(states.back());
    optimizer.add_state_prior(make_full_state_prior(states.back()));
    if (i > 0) {
      optimizer.add_relative_translation_factor(
        make_relative_factor(
          states[static_cast<size_t>(i - 1)].stamp_ns,
          states[static_cast<size_t>(i)].stamp_ns,
          states[static_cast<size_t>(i)].p_w_i - states[static_cast<size_t>(i - 1)].p_w_i));
    }
  }

  const auto marginalization_summary = optimizer.optimize();
  if (marginalization_summary.marginalized_backsubstitution_count == 0U) {
    std::cerr << "missing marginalized back-substitution for bias-column projection probe\n";
    return false;
  }
  const auto before = optimizer.build_normal_equation(config.damping);

  gaussian_lic_tracking::SlidingWindowVisualAlignmentFactor late_visual;
  late_visual.stamp_ns = states.front().stamp_ns;
  late_visual.source_id = 51U;
  late_visual.measured_shift_px = Eigen::Vector2d{0.4, -0.2};
  late_visual.meters_per_pixel = 0.05;
  late_visual.weight = 1.0;
  late_visual.huber_delta_m = 1.0;
  if (!optimizer.add_marginalized_visual_alignment_prior(late_visual)) {
    std::cerr << "bias-column projection visual prior was not created\n";
    return false;
  }

  gaussian_lic_tracking::SlidingWindowSe3PhotometricFactor late_se3;
  late_se3.stamp_ns = states.front().stamp_ns;
  late_se3.source_id = 52U;
  late_se3.target_delta = Eigen::Matrix<double, 6, 1>::Zero();
  late_se3.target_delta(0) = 0.003;
  late_se3.target_delta(3) = -0.015;
  late_se3.sqrt_information = Eigen::Matrix<double, 6, 6>::Identity();
  late_se3.weight = 1.0;
  late_se3.huber_delta = 1.0;
  if (!optimizer.add_marginalized_se3_photometric_prior(late_se3)) {
    std::cerr << "bias-column projection SE3 prior was not created\n";
    return false;
  }

  const auto after = optimizer.build_normal_equation(config.damping);
  if (!before.valid || !after.valid || before.hessian.rows() != after.hessian.rows() ||
    before.hessian.cols() != after.hessian.cols())
  {
    std::cerr << "invalid normal equations for bias-column projection probe\n";
    return false;
  }
  const Eigen::MatrixXd added_hessian = after.hessian - before.hessian;
  double max_bias_coupling = 0.0;
  for (
    Eigen::Index block_col = 0;
    block_col + 15 <= added_hessian.cols();
    block_col += 15)
  {
    max_bias_coupling = std::max(
      max_bias_coupling,
      added_hessian.block(block_col + 9, 0, 6, added_hessian.cols()).cwiseAbs().maxCoeff());
    max_bias_coupling = std::max(
      max_bias_coupling,
      added_hessian.block(0, block_col + 9, added_hessian.rows(), 6).cwiseAbs().maxCoeff());
  }

  std::cout << "visual_marginalization_prior_zero_bias_columns_probe max_bias_coupling="
            << max_bias_coupling << "\n";
  if (!std::isfinite(max_bias_coupling) || max_bias_coupling > 1.0e-9) {
    std::cerr << "late visual/SE3 marginalized priors still write bias columns\n";
    return false;
  }
  return true;
}

bool check_late_visual_priors_preserve_explicit_reference_pose()
{
  auto make_optimizer_with_marginalized_state = []() {
      gaussian_lic_tracking::SlidingWindowConfig config;
      config.max_states = 2;
      config.max_iterations = 1;
      config.marginalization_prior_weight = 0.0;
      gaussian_lic_tracking::SlidingWindowOptimizer optimizer(config);
      std::vector<gaussian_lic_tracking::SlidingWindowState> states;
      for (int i = 0; i < 3; ++i) {
        states.push_back(make_state(static_cast<int64_t>(i), static_cast<double>(i)));
        optimizer.add_or_update_state(states.back());
        optimizer.add_state_prior(make_full_state_prior(states.back()));
        if (i > 0) {
          optimizer.add_relative_translation_factor(
            make_relative_factor(
              states[static_cast<size_t>(i - 1)].stamp_ns,
              states[static_cast<size_t>(i)].stamp_ns,
              states[static_cast<size_t>(i)].p_w_i - states[static_cast<size_t>(i - 1)].p_w_i));
        }
      }
      const auto summary = optimizer.optimize();
      if (summary.marginalized_backsubstitution_count == 0U) {
        throw std::runtime_error("missing marginalized state for explicit reference probe");
      }
      return optimizer;
    };

  auto visual_cost_for_reference = [&make_optimizer_with_marginalized_state](
      const Eigen::Vector3d & reference_p_w_i) {
      auto optimizer = make_optimizer_with_marginalized_state();
      gaussian_lic_tracking::SlidingWindowVisualAlignmentFactor late_visual;
      late_visual.stamp_ns = 0;
      late_visual.source_id = 61U;
      late_visual.has_reference_pose = true;
      late_visual.reference_p_w_i = reference_p_w_i;
      late_visual.measured_shift_px = Eigen::Vector2d::Zero();
      late_visual.meters_per_pixel = 1.0;
      late_visual.weight = 1.0;
      late_visual.huber_delta_m = 10.0;
      if (!optimizer.add_marginalized_visual_alignment_prior(late_visual)) {
        throw std::runtime_error("explicit-reference visual prior was not created");
      }
      const auto normal = optimizer.build_normal_equation();
      if (!normal.valid) {
        throw std::runtime_error("explicit-reference visual normal equation is invalid");
      }
      return normal.cost;
    };

  auto se3_cost_for_reference = [&make_optimizer_with_marginalized_state](
      const Eigen::Vector3d & reference_p_w_i) {
      auto optimizer = make_optimizer_with_marginalized_state();
      gaussian_lic_tracking::SlidingWindowSe3PhotometricFactor late_se3;
      late_se3.stamp_ns = 0;
      late_se3.source_id = 62U;
      late_se3.has_reference_pose = true;
      late_se3.reference_p_w_i = reference_p_w_i;
      late_se3.reference_q_w_i = Eigen::Quaterniond::Identity();
      late_se3.target_delta = Eigen::Matrix<double, 6, 1>::Zero();
      late_se3.sqrt_information = Eigen::Matrix<double, 6, 6>::Identity();
      late_se3.weight = 1.0;
      late_se3.huber_delta = 10.0;
      if (!optimizer.add_marginalized_se3_photometric_prior(late_se3)) {
        throw std::runtime_error("explicit-reference SE3 prior was not created");
      }
      const auto normal = optimizer.build_normal_equation();
      if (!normal.valid) {
        throw std::runtime_error("explicit-reference SE3 normal equation is invalid");
      }
      return normal.cost;
    };

  try {
    const Eigen::Vector3d aligned_reference = make_state(0, 0.0).p_w_i;
    const Eigen::Vector3d shifted_reference = aligned_reference + Eigen::Vector3d{0.2, -0.1, 0.05};
    const double visual_aligned_cost = visual_cost_for_reference(aligned_reference);
    const double visual_shifted_cost = visual_cost_for_reference(shifted_reference);
    const double se3_aligned_cost = se3_cost_for_reference(aligned_reference);
    const double se3_shifted_cost = se3_cost_for_reference(shifted_reference);
    std::cout << "late_visual_reference_pose_probe visual_aligned_cost="
              << visual_aligned_cost
              << " visual_shifted_cost=" << visual_shifted_cost
              << " se3_aligned_cost=" << se3_aligned_cost
              << " se3_shifted_cost=" << se3_shifted_cost << "\n";
    if (!std::isfinite(visual_aligned_cost) || !std::isfinite(visual_shifted_cost) ||
      !std::isfinite(se3_aligned_cost) || !std::isfinite(se3_shifted_cost) ||
      visual_shifted_cost <= visual_aligned_cost + 1.0e-6 ||
      se3_shifted_cost <= se3_aligned_cost + 1.0e-6)
    {
      std::cerr << "late visual/SE3 marginalized priors ignored explicit reference pose\n";
      return false;
    }
  } catch (const std::exception & ex) {
    std::cerr << "explicit-reference marginalized prior probe failed: " << ex.what() << "\n";
    return false;
  }
  return true;
}

bool check_late_visual_priors_preserve_explicit_reference_support()
{
  auto make_optimizer_with_boundary_support = []() {
      gaussian_lic_tracking::SlidingWindowConfig config;
      config.max_states = 2;
      config.max_iterations = 1;
      config.marginalization_prior_weight = 0.0;
      gaussian_lic_tracking::SlidingWindowOptimizer optimizer(config);
      std::vector<gaussian_lic_tracking::SlidingWindowState> states;
      for (int i = 0; i < 3; ++i) {
        states.push_back(make_state(static_cast<int64_t>(i), static_cast<double>(i)));
        optimizer.add_or_update_state(states.back());
        optimizer.add_state_prior(make_full_state_prior(states.back()));
        if (i > 0) {
          optimizer.add_relative_translation_factor(
            make_relative_factor(
              states[static_cast<size_t>(i - 1)].stamp_ns,
              states[static_cast<size_t>(i)].stamp_ns,
              states[static_cast<size_t>(i)].p_w_i - states[static_cast<size_t>(i - 1)].p_w_i));
        }
      }
      const auto summary = optimizer.optimize();
      if (summary.marginalized_backsubstitution_count == 0U) {
        throw std::runtime_error("missing marginalized state for explicit support probe");
      }
      return std::make_pair(optimizer, states);
    };

  try {
    auto visual_case = make_optimizer_with_boundary_support();
    auto & visual_optimizer = visual_case.first;
    const auto & states = visual_case.second;
    const std::vector<int64_t> support_stamp_ns = {
      states[0].stamp_ns,
      states[1].stamp_ns};
    const std::vector<double> support_weights = {0.25, 0.75};
    const Eigen::Vector3d aligned_reference =
      support_weights[0] * states[0].p_w_i + support_weights[1] * states[1].p_w_i;
    const double visual_before = visual_optimizer.build_normal_equation().cost;
    gaussian_lic_tracking::SlidingWindowVisualAlignmentFactor late_visual;
    late_visual.stamp_ns = states[0].stamp_ns;
    late_visual.source_id = 81U;
    late_visual.support_stamp_ns = support_stamp_ns;
    late_visual.support_weights = support_weights;
    late_visual.has_reference_pose = true;
    late_visual.reference_p_w_i = aligned_reference;
    late_visual.measured_shift_px = Eigen::Vector2d::Zero();
    late_visual.meters_per_pixel = 1.0;
    late_visual.weight = 1.0;
    late_visual.huber_delta_m = 10.0;
    if (!visual_optimizer.add_marginalized_visual_alignment_prior(late_visual)) {
      throw std::runtime_error("support-aware visual prior was not created");
    }
    const double visual_aligned_increase =
      visual_optimizer.build_normal_equation().cost - visual_before;

    auto shifted_visual_case = make_optimizer_with_boundary_support();
    auto & shifted_visual_optimizer = shifted_visual_case.first;
    const double shifted_visual_before = shifted_visual_optimizer.build_normal_equation().cost;
    late_visual.reference_p_w_i = aligned_reference + Eigen::Vector3d{0.2, -0.1, 0.0};
    if (!shifted_visual_optimizer.add_marginalized_visual_alignment_prior(late_visual)) {
      throw std::runtime_error("shifted support-aware visual prior was not created");
    }
    const double visual_shifted_increase =
      shifted_visual_optimizer.build_normal_equation().cost - shifted_visual_before;

    auto se3_case = make_optimizer_with_boundary_support();
    auto & se3_optimizer = se3_case.first;
    const double se3_before = se3_optimizer.build_normal_equation().cost;
    gaussian_lic_tracking::SlidingWindowSe3PhotometricFactor late_se3;
    late_se3.stamp_ns = states[0].stamp_ns;
    late_se3.source_id = 82U;
    late_se3.support_stamp_ns = support_stamp_ns;
    late_se3.support_weights = support_weights;
    late_se3.has_reference_pose = true;
    late_se3.reference_p_w_i = aligned_reference;
    late_se3.reference_q_w_i = Eigen::Quaterniond::Identity();
    late_se3.target_delta = Eigen::Matrix<double, 6, 1>::Zero();
    late_se3.sqrt_information = Eigen::Matrix<double, 6, 6>::Identity();
    late_se3.weight = 1.0;
    late_se3.huber_delta = 10.0;
    if (!se3_optimizer.add_marginalized_se3_photometric_prior(late_se3)) {
      throw std::runtime_error("support-aware SE3 prior was not created");
    }
    const double se3_aligned_increase =
      se3_optimizer.build_normal_equation().cost - se3_before;

    auto shifted_se3_case = make_optimizer_with_boundary_support();
    auto & shifted_se3_optimizer = shifted_se3_case.first;
    const double shifted_se3_before = shifted_se3_optimizer.build_normal_equation().cost;
    late_se3.reference_p_w_i = aligned_reference + Eigen::Vector3d{0.2, -0.1, 0.05};
    if (!shifted_se3_optimizer.add_marginalized_se3_photometric_prior(late_se3)) {
      throw std::runtime_error("shifted support-aware SE3 prior was not created");
    }
    const double se3_shifted_increase =
      shifted_se3_optimizer.build_normal_equation().cost - shifted_se3_before;

    std::cout << "late_visual_reference_support_probe visual_aligned_increase="
              << visual_aligned_increase
              << " visual_shifted_increase=" << visual_shifted_increase
              << " se3_aligned_increase=" << se3_aligned_increase
              << " se3_shifted_increase=" << se3_shifted_increase << "\n";
    if (!std::isfinite(visual_aligned_increase) ||
      !std::isfinite(visual_shifted_increase) ||
      !std::isfinite(se3_aligned_increase) ||
      !std::isfinite(se3_shifted_increase) ||
      std::abs(visual_aligned_increase) > 1.0e-8 ||
      std::abs(se3_aligned_increase) > 1.0e-8 ||
      visual_shifted_increase <= visual_aligned_increase + 1.0e-6 ||
      se3_shifted_increase <= se3_aligned_increase + 1.0e-6)
    {
      std::cerr << "late visual/SE3 marginalized priors ignored explicit support stamps\n";
      return false;
    }
  } catch (const std::exception & ex) {
    std::cerr << "explicit-support marginalized prior probe failed: " << ex.what() << "\n";
    return false;
  }
  return true;
}

bool check_late_relative_translation_prior_survives_marginalization()
{
  gaussian_lic_tracking::SlidingWindowConfig config;
  config.max_states = 2;
  config.max_iterations = 1;
  config.marginalization_prior_weight = 0.0;
  gaussian_lic_tracking::SlidingWindowOptimizer optimizer(config);

  std::vector<gaussian_lic_tracking::SlidingWindowState> states;
  for (int i = 0; i < 3; ++i) {
    states.push_back(make_state(static_cast<int64_t>(i), static_cast<double>(i)));
    optimizer.add_or_update_state(states.back());
    optimizer.add_state_prior(make_full_state_prior(states.back()));
    if (i > 0) {
      optimizer.add_relative_translation_factor(
        make_relative_factor(
          states[static_cast<size_t>(i - 1)].stamp_ns,
          states[static_cast<size_t>(i)].stamp_ns,
          states[static_cast<size_t>(i)].p_w_i - states[static_cast<size_t>(i - 1)].p_w_i));
    }
  }

  const auto marginalization_summary = optimizer.optimize();
  if (marginalization_summary.marginalized_backsubstitution_count == 0U) {
    std::cerr << "missing marginalized back-substitution for late relative prior probe\n";
    return false;
  }

  auto late_relative = make_relative_factor(
    states.front().stamp_ns,
    states[1].stamp_ns,
    states[1].p_w_i - states.front().p_w_i);
  late_relative.source_id = 71U;
  late_relative.rotation_weight = 1.0;
  late_relative.rotation_huber_delta_rad = 1.0;
  if (!optimizer.add_marginalized_relative_translation_prior(late_relative)) {
    std::cerr << "late relative translation factor did not build a marginalized prior\n";
    return false;
  }

  const auto late_summary = optimizer.optimize();
  std::cout << "late_relative_translation_prior_probe dense_priors="
            << late_summary.dense_prior_count
            << " interpolations="
            << late_summary.marginalized_backsubstitution_interpolation_count
            << " relative_factors="
            << late_summary.relative_translation_factor_count << "\n";
  if (late_summary.dense_prior_count <= marginalization_summary.dense_prior_count ||
    late_summary.relative_translation_factor_count == 0U)
  {
    std::cerr << "late relative translation marginalized prior was not retained\n";
    return false;
  }
  return true;
}

}  // namespace

int main()
{
  constexpr int steps = 40;
  constexpr int64_t dt_ns = 10000000LL;

  gaussian_lic_tracking::SlidingWindowConfig config;
  config.max_states = 2;
  config.max_iterations = 3;
  config.marginalization_prior_weight = 0.0;
  gaussian_lic_tracking::SlidingWindowOptimizer optimizer(config);

  gaussian_lic_tracking::SlidingWindowState start;
  start.stamp_ns = 0;
  start.p_w_i = Eigen::Vector3d::Zero();
  optimizer.add_or_update_state(start);

  gaussian_lic_tracking::SlidingWindowState middle;
  middle.stamp_ns = steps * dt_ns;
  middle.p_w_i = Eigen::Vector3d{-0.1, 0.05, 0.0};
  middle.v_w_i = Eigen::Vector3d{0.0, -0.05, 0.0};
  optimizer.add_or_update_state(middle);

  gaussian_lic_tracking::ImuPreintegrator preintegration;
  preintegration.reset(start.stamp_ns);
  for (int i = 1; i <= steps; ++i) {
    preintegration.add_measurement(
      static_cast<int64_t>(i) * dt_ns,
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d{1.0, 0.0, 0.0});
  }
  gaussian_lic_tracking::SlidingWindowImuFactor factor;
  factor.from_stamp_ns = start.stamp_ns;
  factor.to_stamp_ns = middle.stamp_ns;
  factor.preintegration = preintegration;
  factor.weight = 5.0;
  factor.bias_weight = 1.0;
  optimizer.add_imu_factor(factor);

  gaussian_lic_tracking::SlidingWindowState newest;
  newest.stamp_ns = middle.stamp_ns + dt_ns;
  newest.p_w_i = Eigen::Vector3d{0.2, 0.0, 0.0};
  optimizer.add_or_update_state(newest);

  const auto normal = optimizer.build_normal_equation(config.damping);
  const auto summary = optimizer.optimize();
  std::cout << "sliding_window_schur_marginalization_probe states=" << summary.state_count
            << " dense_priors=" << summary.dense_prior_count
            << " state_priors=" << summary.state_prior_count
            << " marginalized=" << summary.marginalized_state_count
            << " schur=" << summary.schur_marginalization_count
            << " fallback=" << summary.fallback_marginalization_prior_count
            << " variable_count=" << normal.variable_count
            << " normal_cost=" << normal.cost
            << " final_cost=" << summary.final_cost << "\n";
  if (!normal.valid || summary.state_count != 2U || summary.dense_prior_count != 1U ||
    summary.state_prior_count != 0U || summary.marginalized_state_count != 1U ||
    summary.schur_marginalization_count != 1U ||
    summary.fallback_marginalization_prior_count != 0U ||
    normal.variable_count != 45U || summary.final_cost > normal.cost)
  {
    std::cerr << "Schur marginalization did not produce a retained dense window prior\n";
    return 1;
  }

  gaussian_lic_tracking::SlidingWindowVisualAlignmentFactor late_visual;
  late_visual.stamp_ns = start.stamp_ns;
  late_visual.source_id = 11U;
  late_visual.measured_shift_px = Eigen::Vector2d{1.0, -0.5};
  late_visual.meters_per_pixel = 0.02;
  late_visual.weight = 1.0;
  late_visual.huber_delta_m = 1.0;
  if (!optimizer.add_marginalized_visual_alignment_prior(late_visual)) {
    std::cerr << "late visual factor was not converted through Schur back-substitution\n";
    return 1;
  }

  gaussian_lic_tracking::SlidingWindowSe3PhotometricFactor late_se3;
  late_se3.stamp_ns = start.stamp_ns;
  late_se3.source_id = 12U;
  late_se3.target_delta = Eigen::Matrix<double, 6, 1>::Zero();
  late_se3.target_delta(3) = 0.01;
  late_se3.sqrt_information = Eigen::Matrix<double, 6, 6>::Identity();
  late_se3.weight = 1.0;
  late_se3.huber_delta = 1.0;
  if (!optimizer.add_marginalized_se3_photometric_prior(late_se3)) {
    std::cerr << "late SE3 photometric factor was not converted through Schur back-substitution\n";
    return 1;
  }
  const auto late_summary = optimizer.optimize();
  std::cout << "late_marginalized_visual_prior_probe dense_priors="
            << late_summary.dense_prior_count
            << " visual_marg_priors="
            << late_summary.visual_marginalization_prior_count
            << " se3_marg_priors="
            << late_summary.se3_photometric_marginalization_prior_count
            << "\n";
  if (late_summary.dense_prior_count < 3U ||
    late_summary.visual_marginalization_prior_count != 1U ||
    late_summary.se3_photometric_marginalization_prior_count != 1U)
  {
    std::cerr << "marginalized late visual priors were not retained in the BA normal equation\n";
    return 1;
  }
  if (!check_deferred_marginalization_includes_current_frame_priors()) {
    return 1;
  }
  if (!check_chained_marginalized_prior_survives_retained_state_marginalization()) {
    return 1;
  }
  if (!check_interpolated_marginalized_prior_preserves_continuous_time_stamp()) {
    return 1;
  }
  if (!check_marginalized_active_boundary_prior_preserves_continuous_time_stamp()) {
    return 1;
  }
  if (!check_visual_marginalization_prior_can_remove_bias_columns()) {
    return 1;
  }
  if (!check_late_visual_priors_preserve_explicit_reference_pose()) {
    return 1;
  }
  if (!check_late_visual_priors_preserve_explicit_reference_support()) {
    return 1;
  }
  if (!check_late_relative_translation_prior_survives_marginalization()) {
    return 1;
  }
  std::cout << "sliding_window_schur_marginalization_probe OK\n";
  return 0;
}
