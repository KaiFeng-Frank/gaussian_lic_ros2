// SPDX-License-Identifier: GPL-3.0-or-later

#include <gaussian_lic_tracking/sliding_window_optimizer.hpp>

#include <iostream>

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

  const auto late_summary = optimizer.optimize();
  std::cout << "marginalized_active_boundary_visual_prior_probe active_backsubs="
            << late_summary.marginalized_backsubstitution_count
            << " interpolations="
            << late_summary.marginalized_backsubstitution_interpolation_count
            << " visual_marg_priors="
            << late_summary.visual_marginalization_prior_count << "\n";
  if (late_summary.marginalized_backsubstitution_interpolation_count == 0U ||
    late_summary.visual_marginalization_prior_count == 0U)
  {
    std::cerr << "marginalized-active boundary prior was not retained in the BA window\n";
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
  std::cout << "sliding_window_schur_marginalization_probe OK\n";
  return 0;
}
