// SPDX-License-Identifier: GPL-3.0-or-later

#include <gaussian_lic_tracking/sliding_window_optimizer.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <Eigen/Dense>

namespace gaussian_lic_tracking
{
namespace
{
constexpr size_t kStateDof = 15U;

ImuState to_imu_state(const SlidingWindowState & state)
{
  ImuState output;
  output.stamp_ns = state.stamp_ns;
  output.p_w_i = state.p_w_i;
  output.q_w_i = state.q_w_i.normalized();
  output.v_w_i = state.v_w_i;
  output.gyro_bias = state.gyro_bias;
  output.accel_bias = state.accel_bias;
  return output;
}
}  // namespace

SlidingWindowOptimizer::SlidingWindowOptimizer(SlidingWindowConfig config)
{
  set_config(config);
}

void SlidingWindowOptimizer::set_config(const SlidingWindowConfig & config)
{
  if (config.max_states < 2U) {
    throw std::runtime_error("sliding window must keep at least two states");
  }
  if (config.max_iterations == 0U) {
    throw std::runtime_error("sliding window optimizer must run at least one iteration");
  }
  if (config.damping <= 0.0 || config.step_tolerance <= 0.0 || config.numeric_epsilon <= 0.0) {
    throw std::runtime_error("sliding window optimizer numeric settings must be positive");
  }
  if (config.marginalization_prior_weight < 0.0) {
    throw std::runtime_error("sliding window marginalization prior weight must be non-negative");
  }
  config_ = config;
}

void SlidingWindowOptimizer::clear()
{
  states_.clear();
  imu_factors_.clear();
  pose_priors_.clear();
  state_priors_.clear();
  point_factors_.clear();
  visual_factors_.clear();
  marginalized_state_count_ = 0U;
}

int SlidingWindowOptimizer::find_state_index(const int64_t stamp_ns) const
{
  const auto it = std::find_if(
    states_.begin(), states_.end(),
    [stamp_ns](const SlidingWindowState & state) {
      return state.stamp_ns == stamp_ns;
    });
  if (it == states_.end()) {
    return -1;
  }
  return static_cast<int>(std::distance(states_.begin(), it));
}

void SlidingWindowOptimizer::add_or_update_state(const SlidingWindowState & state)
{
  SlidingWindowState normalized = state;
  normalized.q_w_i.normalize();
  const int existing = find_state_index(normalized.stamp_ns);
  if (existing >= 0) {
    states_[static_cast<size_t>(existing)] = normalized;
  } else {
    states_.push_back(normalized);
    std::sort(
      states_.begin(), states_.end(),
      [](const SlidingWindowState & lhs, const SlidingWindowState & rhs) {
        return lhs.stamp_ns < rhs.stamp_ns;
      });
  }
  enforce_window_size();
}

bool SlidingWindowOptimizer::get_state(const int64_t stamp_ns, SlidingWindowState & state) const
{
  const int index = find_state_index(stamp_ns);
  if (index < 0) {
    return false;
  }
  state = states_[static_cast<size_t>(index)];
  return true;
}

void SlidingWindowOptimizer::add_imu_factor(const SlidingWindowImuFactor & factor)
{
  if (factor.to_stamp_ns <= factor.from_stamp_ns) {
    throw std::runtime_error("IMU factor timestamps must be strictly increasing");
  }
  if (factor.weight <= 0.0) {
    throw std::runtime_error("IMU factor weight must be positive");
  }
  imu_factors_.push_back(factor);
  enforce_window_size();
}

void SlidingWindowOptimizer::add_pose_prior(const SlidingWindowPosePrior & prior)
{
  if (prior.translation_weight < 0.0 || prior.rotation_weight < 0.0) {
    throw std::runtime_error("pose prior weights must be non-negative");
  }
  SlidingWindowPosePrior normalized = prior;
  normalized.q_w_i.normalize();
  pose_priors_.push_back(normalized);
  enforce_window_size();
}

void SlidingWindowOptimizer::add_state_prior(const SlidingWindowStatePrior & prior)
{
  if (prior.rotation_weight < 0.0 || prior.velocity_weight < 0.0 ||
    prior.position_weight < 0.0 || prior.gyro_bias_weight < 0.0 ||
    prior.accel_bias_weight < 0.0)
  {
    throw std::runtime_error("state prior weights must be non-negative");
  }
  SlidingWindowStatePrior normalized = prior;
  normalized.q_w_i.normalize();
  state_priors_.push_back(normalized);
  enforce_window_size();
}

void SlidingWindowOptimizer::add_point_to_point_factor(const SlidingWindowPointToPointFactor & factor)
{
  if (factor.weight <= 0.0) {
    throw std::runtime_error("point-to-point factor weight must be positive");
  }
  if (factor.frame_points_i.size() != factor.target_points_w.size()) {
    throw std::runtime_error("point-to-point factor source/target sizes must match");
  }
  if (factor.frame_points_i.empty()) {
    return;
  }
  point_factors_.push_back(factor);
  enforce_window_size();
}

void SlidingWindowOptimizer::add_visual_alignment_factor(const SlidingWindowVisualAlignmentFactor & factor)
{
  if (factor.weight <= 0.0 || factor.meters_per_pixel <= 0.0) {
    throw std::runtime_error("visual alignment factor weight and meters_per_pixel must be positive");
  }
  if (!factor.measured_shift_px.allFinite() || !factor.reference_p_w_i.allFinite()) {
    throw std::runtime_error("visual alignment factor values must be finite");
  }
  visual_factors_.push_back(factor);
  enforce_window_size();
}

SlidingWindowStatePrior SlidingWindowOptimizer::make_state_prior(const SlidingWindowState & state) const
{
  SlidingWindowStatePrior prior;
  prior.stamp_ns = state.stamp_ns;
  prior.p_w_i = state.p_w_i;
  prior.q_w_i = state.q_w_i.normalized();
  prior.v_w_i = state.v_w_i;
  prior.gyro_bias = state.gyro_bias;
  prior.accel_bias = state.accel_bias;
  prior.sqrt_information.setZero();
  prior.sqrt_information.template block<3, 3>(0, 0) =
    std::sqrt(config_.marginalization_prior_weight) * Eigen::Matrix3d::Identity();
  prior.sqrt_information.template block<3, 3>(3, 3) =
    std::sqrt(config_.marginalization_prior_weight) * Eigen::Matrix3d::Identity();
  prior.sqrt_information.template block<3, 3>(6, 6) =
    std::sqrt(config_.marginalization_prior_weight) * Eigen::Matrix3d::Identity();
  prior.sqrt_information.template block<3, 3>(9, 9) =
    std::sqrt(config_.marginalization_prior_weight) * Eigen::Matrix3d::Identity();
  prior.sqrt_information.template block<3, 3>(12, 12) =
    std::sqrt(config_.marginalization_prior_weight) * Eigen::Matrix3d::Identity();
  prior.rotation_weight = config_.marginalization_prior_weight;
  prior.velocity_weight = config_.marginalization_prior_weight;
  prior.position_weight = config_.marginalization_prior_weight;
  prior.gyro_bias_weight = config_.marginalization_prior_weight;
  prior.accel_bias_weight = config_.marginalization_prior_weight;
  return prior;
}

size_t SlidingWindowOptimizer::enforce_window_size()
{
  size_t marginalized = 0U;
  while (states_.size() > config_.max_states) {
    const int64_t stamp_ns = states_.front().stamp_ns;
    states_.erase(states_.begin());
    ++marginalized;
    ++marginalized_state_count_;
    imu_factors_.erase(
      std::remove_if(
        imu_factors_.begin(), imu_factors_.end(),
        [stamp_ns](const SlidingWindowImuFactor & factor) {
          return factor.from_stamp_ns == stamp_ns || factor.to_stamp_ns == stamp_ns;
        }),
      imu_factors_.end());
    pose_priors_.erase(
      std::remove_if(
        pose_priors_.begin(), pose_priors_.end(),
        [stamp_ns](const SlidingWindowPosePrior & prior) {
          return prior.stamp_ns == stamp_ns;
        }),
      pose_priors_.end());
    state_priors_.erase(
      std::remove_if(
        state_priors_.begin(), state_priors_.end(),
        [stamp_ns](const SlidingWindowStatePrior & prior) {
          return prior.stamp_ns == stamp_ns;
        }),
      state_priors_.end());
    point_factors_.erase(
      std::remove_if(
        point_factors_.begin(), point_factors_.end(),
        [stamp_ns](const SlidingWindowPointToPointFactor & factor) {
          return factor.stamp_ns == stamp_ns;
        }),
      point_factors_.end());
    visual_factors_.erase(
      std::remove_if(
        visual_factors_.begin(), visual_factors_.end(),
        [stamp_ns](const SlidingWindowVisualAlignmentFactor & factor) {
          return factor.stamp_ns == stamp_ns;
        }),
      visual_factors_.end());
    if (!states_.empty() && config_.marginalization_prior_weight > 0.0) {
      const int64_t anchor_stamp_ns = states_.front().stamp_ns;
      state_priors_.erase(
        std::remove_if(
          state_priors_.begin(), state_priors_.end(),
          [anchor_stamp_ns](const SlidingWindowStatePrior & prior) {
            return prior.stamp_ns == anchor_stamp_ns;
          }),
        state_priors_.end());
      state_priors_.push_back(make_state_prior(states_.front()));
    }
  }
  return marginalized;
}

std::vector<SlidingWindowOptimizer::VariableBlock> SlidingWindowOptimizer::variable_layout() const
{
  std::vector<VariableBlock> variables;
  variables.reserve(states_.size());
  size_t offset = 0U;
  for (size_t index = 0; index < states_.size(); ++index) {
    if (states_[index].fixed) {
      continue;
    }
    variables.push_back(VariableBlock{index, offset});
    offset += kStateDof;
  }
  return variables;
}

Eigen::Vector3d SlidingWindowOptimizer::rotation_residual(
  const Eigen::Quaterniond & measured_q,
  const Eigen::Quaterniond & predicted_q)
{
  Eigen::Quaterniond error = (measured_q.normalized().inverse() * predicted_q.normalized()).normalized();
  if (error.w() < 0.0) {
    error.coeffs() *= -1.0;
  }
  return 2.0 * error.vec();
}

Eigen::VectorXd SlidingWindowOptimizer::build_residual(
  const std::vector<SlidingWindowState> & states) const
{
  std::vector<double> values;
  values.reserve(
    imu_factors_.size() * 15U + pose_priors_.size() * 6U + state_priors_.size() * 15U +
    point_factors_.size() * 300U + visual_factors_.size() * 2U);

  auto append = [&values](const Eigen::VectorXd & residual) {
      for (Eigen::Index i = 0; i < residual.size(); ++i) {
        values.push_back(residual[i]);
      }
    };

  auto find_local = [&states](const int64_t stamp_ns) -> int {
      const auto it = std::find_if(
        states.begin(), states.end(),
        [stamp_ns](const SlidingWindowState & state) {
          return state.stamp_ns == stamp_ns;
        });
      if (it == states.end()) {
        return -1;
      }
      return static_cast<int>(std::distance(states.begin(), it));
    };

  for (const auto & factor : imu_factors_) {
    const int from = find_local(factor.from_stamp_ns);
    const int to = find_local(factor.to_stamp_ns);
    if (from < 0 || to < 0) {
      continue;
    }
    const ImuBias start_bias{
      states[static_cast<size_t>(from)].gyro_bias,
      states[static_cast<size_t>(from)].accel_bias};
    const auto corrected_preintegration = factor.preintegration.reintegrated(start_bias);
    const auto residual = corrected_preintegration.residual(
      to_imu_state(states[static_cast<size_t>(from)]),
      to_imu_state(states[static_cast<size_t>(to)]),
      factor.gravity_w);
    append(std::sqrt(factor.weight) * residual.residual);
    Eigen::Matrix<double, 6, 1> bias_residual;
    bias_residual.template segment<3>(0) =
      states[static_cast<size_t>(to)].gyro_bias - states[static_cast<size_t>(from)].gyro_bias;
    bias_residual.template segment<3>(3) =
      states[static_cast<size_t>(to)].accel_bias - states[static_cast<size_t>(from)].accel_bias;
    append(std::sqrt(factor.bias_weight) * bias_residual);
  }

  for (const auto & prior : pose_priors_) {
    const int index = find_local(prior.stamp_ns);
    if (index < 0) {
      continue;
    }
    const auto & state = states[static_cast<size_t>(index)];
    Eigen::Matrix<double, 6, 1> residual;
    residual.template segment<3>(0) =
      std::sqrt(prior.rotation_weight) * rotation_residual(prior.q_w_i, state.q_w_i);
    residual.template segment<3>(3) =
      std::sqrt(prior.translation_weight) * (state.p_w_i - prior.p_w_i);
    append(residual);
  }

  for (const auto & prior : state_priors_) {
    const int index = find_local(prior.stamp_ns);
    if (index < 0) {
      continue;
    }
    const auto & state = states[static_cast<size_t>(index)];
    Eigen::Matrix<double, 15, 1> residual;
    residual.template segment<3>(0) =
      rotation_residual(prior.q_w_i, state.q_w_i);
    residual.template segment<3>(3) =
      state.v_w_i - prior.v_w_i;
    residual.template segment<3>(6) =
      state.p_w_i - prior.p_w_i;
    residual.template segment<3>(9) =
      state.gyro_bias - prior.gyro_bias;
    residual.template segment<3>(12) =
      state.accel_bias - prior.accel_bias;
    Eigen::Matrix<double, 15, 15> sqrt_information = prior.sqrt_information;
    if (sqrt_information.isApprox(Eigen::Matrix<double, 15, 15>::Identity())) {
      sqrt_information.setZero();
      sqrt_information.template block<3, 3>(0, 0) =
        std::sqrt(prior.rotation_weight) * Eigen::Matrix3d::Identity();
      sqrt_information.template block<3, 3>(3, 3) =
        std::sqrt(prior.velocity_weight) * Eigen::Matrix3d::Identity();
      sqrt_information.template block<3, 3>(6, 6) =
        std::sqrt(prior.position_weight) * Eigen::Matrix3d::Identity();
      sqrt_information.template block<3, 3>(9, 9) =
        std::sqrt(prior.gyro_bias_weight) * Eigen::Matrix3d::Identity();
      sqrt_information.template block<3, 3>(12, 12) =
        std::sqrt(prior.accel_bias_weight) * Eigen::Matrix3d::Identity();
    }
    append(sqrt_information * residual);
  }

  for (const auto & factor : point_factors_) {
    const int index = find_local(factor.stamp_ns);
    if (index < 0) {
      continue;
    }
    const auto & state = states[static_cast<size_t>(index)];
    const double scale = std::sqrt(factor.weight);
    for (size_t point_index = 0; point_index < factor.frame_points_i.size(); ++point_index) {
      Eigen::Vector3d residual =
        state.q_w_i * factor.frame_points_i[point_index] + state.p_w_i -
        factor.target_points_w[point_index];
      append(scale * residual);
    }
  }

  for (const auto & factor : visual_factors_) {
    const int index = find_local(factor.stamp_ns);
    if (index < 0) {
      continue;
    }
    const auto & state = states[static_cast<size_t>(index)];
    Eigen::Vector2d target_xy;
    target_xy.x() = factor.reference_p_w_i.x() + factor.measured_shift_px.x() * factor.meters_per_pixel;
    target_xy.y() = factor.reference_p_w_i.y() + factor.measured_shift_px.y() * factor.meters_per_pixel;
    Eigen::Vector2d residual;
    residual.x() = state.p_w_i.x() - target_xy.x();
    residual.y() = state.p_w_i.y() - target_xy.y();
    append(std::sqrt(factor.weight) * residual);
  }

  Eigen::VectorXd residual(values.size());
  for (size_t i = 0; i < values.size(); ++i) {
    residual[static_cast<Eigen::Index>(i)] = values[i];
  }
  return residual;
}

double SlidingWindowOptimizer::compute_cost(const Eigen::VectorXd & residual) const
{
  return 0.5 * residual.squaredNorm();
}

void SlidingWindowOptimizer::apply_delta(
  std::vector<SlidingWindowState> & states,
  const std::vector<VariableBlock> & variables,
  const Eigen::VectorXd & delta,
  const double scale)
{
  for (const auto & variable : variables) {
    auto & state = states[variable.state_index];
    const auto offset = static_cast<Eigen::Index>(variable.offset);
    const Eigen::Vector3d dtheta = scale * delta.template segment<3>(offset);
    const Eigen::Vector3d dv = scale * delta.template segment<3>(offset + 3);
    const Eigen::Vector3d dp = scale * delta.template segment<3>(offset + 6);
    const Eigen::Vector3d dgyro_bias = scale * delta.template segment<3>(offset + 9);
    const Eigen::Vector3d daccel_bias = scale * delta.template segment<3>(offset + 12);
    const double angle = dtheta.norm();
    if (angle > 1.0e-12) {
      state.q_w_i = (Eigen::Quaterniond(Eigen::AngleAxisd(angle, dtheta / angle)) * state.q_w_i).normalized();
    }
    state.v_w_i += dv;
    state.p_w_i += dp;
    state.gyro_bias += dgyro_bias;
    state.accel_bias += daccel_bias;
  }
}

SlidingWindowSummary SlidingWindowOptimizer::optimize()
{
  SlidingWindowSummary summary;
  enforce_window_size();
  summary.marginalized_state_count = marginalized_state_count_;
  summary.state_count = states_.size();
  summary.imu_factor_count = imu_factors_.size();
  summary.pose_prior_count = pose_priors_.size();
  summary.state_prior_count = state_priors_.size();
  summary.point_factor_count = point_factors_.size();
  summary.visual_factor_count = visual_factors_.size();
  Eigen::VectorXd residual = build_residual(states_);
  summary.initial_cost = compute_cost(residual);
  summary.final_cost = summary.initial_cost;

  const auto variables = variable_layout();
  const size_t variable_count = variables.size() * kStateDof;
  if (residual.size() == 0 || variable_count == 0U) {
    summary.converged = true;
    return summary;
  }

  double damping = config_.damping;
  for (size_t iteration = 0; iteration < config_.max_iterations; ++iteration) {
    residual = build_residual(states_);
    const double current_cost = compute_cost(residual);
    Eigen::MatrixXd jacobian(residual.size(), static_cast<Eigen::Index>(variable_count));
    for (size_t column = 0; column < variable_count; ++column) {
      Eigen::VectorXd delta = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(variable_count));
      delta[static_cast<Eigen::Index>(column)] = config_.numeric_epsilon;
      auto plus_states = states_;
      auto minus_states = states_;
      apply_delta(plus_states, variables, delta, 1.0);
      apply_delta(minus_states, variables, delta, -1.0);
      jacobian.col(static_cast<Eigen::Index>(column)) =
        (build_residual(plus_states) - build_residual(minus_states)) /
        (2.0 * config_.numeric_epsilon);
    }

    Eigen::MatrixXd normal =
      jacobian.transpose() * jacobian +
      damping * Eigen::MatrixXd::Identity(
        static_cast<Eigen::Index>(variable_count),
        static_cast<Eigen::Index>(variable_count));
    const Eigen::VectorXd rhs = -jacobian.transpose() * residual;
    const Eigen::VectorXd step = normal.ldlt().solve(rhs);
    if (!step.allFinite()) {
      break;
    }

    auto candidate_states = states_;
    apply_delta(candidate_states, variables, step, 1.0);
    const double candidate_cost = compute_cost(build_residual(candidate_states));
    if (candidate_cost <= current_cost) {
      states_ = std::move(candidate_states);
      summary.iterations = iteration + 1U;
      summary.final_cost = candidate_cost;
      damping = std::max(damping * 0.1, config_.damping);
      if (step.norm() < config_.step_tolerance) {
        summary.converged = true;
        break;
      }
    } else {
      damping *= 10.0;
      summary.final_cost = current_cost;
    }
  }
  if (summary.iterations == config_.max_iterations || summary.final_cost < summary.initial_cost) {
    summary.converged = summary.final_cost <= summary.initial_cost;
  }
  return summary;
}

}  // namespace gaussian_lic_tracking
