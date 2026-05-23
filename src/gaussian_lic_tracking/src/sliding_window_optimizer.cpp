// SPDX-License-Identifier: GPL-3.0-or-later

#include <gaussian_lic_tracking/sliding_window_optimizer.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <Eigen/Dense>
#include <unsupported/Eigen/AutoDiff>

namespace gaussian_lic_tracking
{
namespace
{
constexpr size_t kStateDof = 15U;
constexpr size_t kSmoothnessResidualDof = 18U;
using BiasDerivativeVector = Eigen::Matrix<double, 6, 1>;
using BiasAutoDiff = Eigen::AutoDiffScalar<BiasDerivativeVector>;
using Vector3ad = Eigen::Matrix<BiasAutoDiff, 3, 1>;
using Quaternionad = Eigen::Quaternion<BiasAutoDiff>;

bool visual_support_is_valid(
  const std::vector<int64_t> & support_stamp_ns,
  const std::vector<double> & support_weights)
{
  if (support_stamp_ns.empty() && support_weights.empty()) {
    return true;
  }
  if (support_stamp_ns.empty() || support_stamp_ns.size() != support_weights.size() ||
    support_stamp_ns.size() > 2U)
  {
    return false;
  }
  double weight_sum = 0.0;
  int64_t previous_stamp_ns = std::numeric_limits<int64_t>::min();
  for (size_t index = 0; index < support_stamp_ns.size(); ++index) {
    const double weight = support_weights[index];
    if (!std::isfinite(weight) || weight < 0.0) {
      return false;
    }
    if (index > 0U && support_stamp_ns[index] <= previous_stamp_ns) {
      return false;
    }
    previous_stamp_ns = support_stamp_ns[index];
    weight_sum += weight;
  }
  return std::isfinite(weight_sum) && std::abs(weight_sum - 1.0) <= 1.0e-6;
}

template<typename FactorT>
bool visual_factor_references_stamp(const FactorT & factor, const int64_t stamp_ns)
{
  if (!factor.support_stamp_ns.empty()) {
    return std::find(
      factor.support_stamp_ns.begin(), factor.support_stamp_ns.end(), stamp_ns) !=
           factor.support_stamp_ns.end();
  }
  return factor.stamp_ns == stamp_ns;
}

template<typename FactorT, typename PredicateT>
bool visual_factor_has_stamp_matching(const FactorT & factor, PredicateT predicate)
{
  if (!factor.support_stamp_ns.empty()) {
    return std::any_of(
      factor.support_stamp_ns.begin(), factor.support_stamp_ns.end(), predicate);
  }
  return predicate(factor.stamp_ns);
}

BiasAutoDiff make_bias_autodiff(const double value, const int derivative_index = -1)
{
  BiasAutoDiff output(value);
  output.derivatives() = BiasDerivativeVector::Zero();
  if (derivative_index >= 0 && derivative_index < output.derivatives().size()) {
    output.derivatives()[derivative_index] = 1.0;
  }
  return output;
}

Vector3ad make_bias_autodiff_vector(const Eigen::Vector3d & value, const int derivative_offset = -1)
{
  Vector3ad output;
  for (Eigen::Index index = 0; index < 3; ++index) {
    output[index] = make_bias_autodiff(
      value[index],
      derivative_offset >= 0 ? derivative_offset + static_cast<int>(index) : -1);
  }
  return output;
}

Quaternionad make_bias_autodiff_quaternion(const Eigen::Quaterniond & value)
{
  return Quaternionad(
    make_bias_autodiff(value.w()),
    make_bias_autodiff(value.x()),
    make_bias_autodiff(value.y()),
    make_bias_autodiff(value.z()));
}

Quaternionad normalized_quaternion(const Quaternionad & quaternion)
{
  using std::sqrt;
  const BiasAutoDiff norm = sqrt(
    quaternion.w() * quaternion.w() + quaternion.x() * quaternion.x() +
    quaternion.y() * quaternion.y() + quaternion.z() * quaternion.z());
  return Quaternionad(
    quaternion.w() / norm,
    quaternion.x() / norm,
    quaternion.y() / norm,
    quaternion.z() / norm);
}

Vector3ad quaternion_log_vector_autodiff(Quaternionad quaternion)
{
  using std::atan2;
  using std::sqrt;

  quaternion = normalized_quaternion(quaternion);
  if (quaternion.w().value() < 0.0) {
    quaternion.coeffs() *= -1.0;
  }
  const Vector3ad vector = quaternion.vec();
  const BiasAutoDiff vector_norm = sqrt(vector.squaredNorm());
  if (vector_norm.value() < 1.0e-12) {
    return 2.0 * vector;
  }
  const BiasAutoDiff angle = 2.0 * atan2(vector_norm, quaternion.w());
  return (angle / vector_norm) * vector;
}

Vector3ad rotate_vector(const Quaternionad & quaternion, const Vector3ad & vector)
{
  return quaternion * vector;
}

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

bool quaternion_is_finite_and_nonzero(const Eigen::Quaterniond & quaternion)
{
  return quaternion.coeffs().allFinite() &&
         quaternion.norm() > std::numeric_limits<double>::epsilon();
}

bool state_is_finite(const SlidingWindowState & state)
{
  return state.p_w_i.allFinite() && state.v_w_i.allFinite() &&
         state.gyro_bias.allFinite() && state.accel_bias.allFinite() &&
         quaternion_is_finite_and_nonzero(state.q_w_i);
}

bool states_are_finite(const std::vector<SlidingWindowState> & states)
{
  return std::all_of(
    states.begin(), states.end(),
    [](const SlidingWindowState & state) {
      return state_is_finite(state);
    });
}

Eigen::MatrixXd sqrt_information_from_hessian(const Eigen::MatrixXd & hessian)
{
  if (hessian.rows() != hessian.cols() || hessian.rows() == 0 || !hessian.allFinite()) {
    return {};
  }
  const Eigen::MatrixXd symmetric = 0.5 * (hessian + hessian.transpose());
  const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(symmetric);
  if (solver.info() != Eigen::Success) {
    return {};
  }
  Eigen::VectorXd sqrt_values(solver.eigenvalues().size());
  for (Eigen::Index i = 0; i < solver.eigenvalues().size(); ++i) {
    sqrt_values[i] = std::sqrt(std::max(0.0, solver.eigenvalues()[i]));
  }
  return sqrt_values.asDiagonal() * solver.eigenvectors().transpose();
}

Eigen::Matrix3d skew_symmetric(const Eigen::Vector3d & value)
{
  Eigen::Matrix3d skew;
  skew << 0.0, -value.z(), value.y(),
    value.z(), 0.0, -value.x(),
    -value.y(), value.x(), 0.0;
  return skew;
}

double huber_weight(const double residual_norm, const double huber_delta)
{
  if (huber_delta <= 0.0 || residual_norm <= huber_delta) {
    return 1.0;
  }
  return huber_delta / std::max(residual_norm, std::numeric_limits<double>::epsilon());
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

Eigen::Matrix3d so3_left_jacobian_inverse(const Eigen::Vector3d & phi)
{
  const Eigen::Matrix3d phi_hat = skew_symmetric(phi);
  const Eigen::Matrix3d phi_hat_sq = phi_hat * phi_hat;
  const double theta_sq = phi.squaredNorm();
  if (theta_sq < 1.0e-12) {
    return Eigen::Matrix3d::Identity() - 0.5 * phi_hat + (1.0 / 12.0) * phi_hat_sq;
  }
  const double theta = std::sqrt(theta_sq);
  const double coefficient =
    (1.0 / theta_sq) - ((1.0 + std::cos(theta)) / (2.0 * theta * std::sin(theta)));
  return Eigen::Matrix3d::Identity() - 0.5 * phi_hat + coefficient * phi_hat_sq;
}

Eigen::Matrix3d rotation_residual_left_perturbation_jacobian(
  const Eigen::Quaterniond & reference_q,
  const Eigen::Quaterniond & state_q)
{
  return so3_left_jacobian_inverse(relative_rotation_vector(reference_q, state_q)) *
         reference_q.normalized().inverse().toRotationMatrix();
}

Eigen::Matrix3d rotation_residual_middle_perturbation_jacobian(
  const Eigen::Quaterniond & measured_delta_q,
  const Eigen::Quaterniond & predicted_delta_q)
{
  return so3_left_jacobian_inverse(relative_rotation_vector(measured_delta_q, predicted_delta_q)) *
         measured_delta_q.normalized().inverse().toRotationMatrix();
}

Eigen::Matrix<double, 9, 6> imu_preintegration_bias_residual_jacobian(
  const SlidingWindowImuFactor & factor,
  const SlidingWindowState & from_state,
  const SlidingWindowState & to_state)
{
  using std::cos;
  using std::sin;
  using std::sqrt;

  const Vector3ad gyro_bias = make_bias_autodiff_vector(from_state.gyro_bias, 0);
  const Vector3ad accel_bias = make_bias_autodiff_vector(from_state.accel_bias, 3);
  Quaternionad delta_q = make_bias_autodiff_quaternion(Eigen::Quaterniond::Identity());
  Vector3ad delta_v = make_bias_autodiff_vector(Eigen::Vector3d::Zero());
  Vector3ad delta_p = make_bias_autodiff_vector(Eigen::Vector3d::Zero());
  Vector3ad last_omega = make_bias_autodiff_vector(Eigen::Vector3d::Zero());
  Vector3ad last_accel = make_bias_autodiff_vector(Eigen::Vector3d::Zero());
  bool has_last = false;
  int64_t end_stamp_ns = factor.preintegration.start_stamp_ns();
  double delta_t_s = 0.0;

  for (const auto & sample : factor.preintegration.samples()) {
    const double dt = static_cast<double>(sample.stamp_ns - end_stamp_ns) / 1.0e9;
    const Vector3ad current_omega = make_bias_autodiff_vector(sample.angular_velocity_rad_s);
    const Vector3ad current_accel = make_bias_autodiff_vector(sample.linear_acceleration_m_s2);
    const Vector3ad previous_omega = has_last ? last_omega : current_omega;
    const Vector3ad previous_accel = has_last ? last_accel : current_accel;
    const Vector3ad omega = 0.5 * (previous_omega + current_omega) - gyro_bias;
    const Vector3ad accel_previous = previous_accel - accel_bias;
    const Vector3ad accel_current = current_accel - accel_bias;

    Quaternionad step_q = make_bias_autodiff_quaternion(Eigen::Quaterniond::Identity());
    const BiasAutoDiff omega_norm = sqrt(omega.squaredNorm());
    const BiasAutoDiff angle = omega_norm * dt;
    if (angle.value() > 1.0e-12) {
      const BiasAutoDiff half_angle = 0.5 * angle;
      const BiasAutoDiff sin_half = sin(half_angle);
      const Vector3ad axis = omega / omega_norm;
      step_q = Quaternionad(
        cos(half_angle),
        sin_half * axis.x(),
        sin_half * axis.y(),
        sin_half * axis.z());
    }
    const Quaternionad next_delta_q = normalized_quaternion(delta_q * step_q);
    const Vector3ad accel_local =
      0.5 * (rotate_vector(delta_q, accel_previous) +
      rotate_vector(next_delta_q, accel_current));

    delta_p += delta_v * dt + 0.5 * accel_local * dt * dt;
    delta_v += accel_local * dt;
    delta_q = next_delta_q;
    delta_t_s += dt;
    end_stamp_ns = sample.stamp_ns;
    last_omega = current_omega;
    last_accel = current_accel;
    has_last = true;
  }

  const Quaternionad q_start_inv =
    make_bias_autodiff_quaternion(from_state.q_w_i.normalized().inverse());
  const Quaternionad q_end = make_bias_autodiff_quaternion(to_state.q_w_i.normalized());
  const Quaternionad predicted_delta_q = normalized_quaternion(q_start_inv * q_end);
  const Vector3ad predicted_delta_v = rotate_vector(
    q_start_inv,
    make_bias_autodiff_vector(
      to_state.v_w_i - from_state.v_w_i - factor.gravity_w * delta_t_s));
  const Vector3ad predicted_delta_p = rotate_vector(
    q_start_inv,
    make_bias_autodiff_vector(
      to_state.p_w_i - from_state.p_w_i - from_state.v_w_i * delta_t_s -
      0.5 * factor.gravity_w * delta_t_s * delta_t_s));

  const Quaternionad error = normalized_quaternion(delta_q.conjugate() * predicted_delta_q);
  Eigen::Matrix<BiasAutoDiff, 9, 1> residual;
  residual.template segment<3>(0) = quaternion_log_vector_autodiff(error);
  residual.template segment<3>(3) = predicted_delta_v - delta_v;
  residual.template segment<3>(6) = predicted_delta_p - delta_p;

  Eigen::Matrix<double, 9, 6> jacobian;
  for (Eigen::Index row = 0; row < residual.rows(); ++row) {
    jacobian.row(row) = residual[row].derivatives().transpose();
  }
  return jacobian;
}

Eigen::Matrix<double, 15, 15> state_delta_left_perturbation_jacobian(
  const Eigen::Quaterniond & reference_q,
  const Eigen::Quaterniond & state_q)
{
  Eigen::Matrix<double, 15, 15> jacobian = Eigen::Matrix<double, 15, 15>::Zero();
  jacobian.template block<3, 3>(0, 0) =
    rotation_residual_left_perturbation_jacobian(reference_q, state_q);
  jacobian.template block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();
  jacobian.template block<3, 3>(6, 6) = Eigen::Matrix3d::Identity();
  jacobian.template block<3, 3>(9, 9) = Eigen::Matrix3d::Identity();
  jacobian.template block<3, 3>(12, 12) = Eigen::Matrix3d::Identity();
  return jacobian;
}

Eigen::Matrix<double, 15, 15> effective_state_prior_sqrt_information(
  const SlidingWindowStatePrior & prior)
{
  Eigen::Matrix<double, 15, 15> sqrt_information = prior.sqrt_information;
  if (!sqrt_information.isApprox(Eigen::Matrix<double, 15, 15>::Identity())) {
    return sqrt_information;
  }
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
  return sqrt_information;
}

bool has_full_imu_sqrt_information(const SlidingWindowImuFactor & factor)
{
  return factor.sqrt_information.cwiseAbs().maxCoeff() > 1.0e-12;
}

Eigen::Matrix<double, 9, 9> effective_imu_sqrt_information(
  const SlidingWindowImuFactor & factor)
{
  if (has_full_imu_sqrt_information(factor)) {
    return std::sqrt(factor.weight) * factor.sqrt_information;
  }
  Eigen::Matrix<double, 9, 9> sqrt_information = Eigen::Matrix<double, 9, 9>::Zero();
  sqrt_information.template block<3, 3>(0, 0) =
    std::sqrt(factor.weight * factor.rotation_weight) * Eigen::Matrix3d::Identity();
  sqrt_information.template block<3, 3>(3, 3) =
    std::sqrt(factor.weight * factor.velocity_weight) * Eigen::Matrix3d::Identity();
  sqrt_information.template block<3, 3>(6, 6) =
    std::sqrt(factor.weight * factor.position_weight) * Eigen::Matrix3d::Identity();
  return sqrt_information;
}

double bias_random_walk_time_scale(const SlidingWindowImuFactor & factor)
{
  if (factor.bias_random_walk_reference_dt_s <= 0.0) {
    return 1.0;
  }
  const double dt_s = factor.preintegration.delta_t_s();
  if (!std::isfinite(dt_s) || dt_s <= 0.0) {
    return 1.0;
  }
  return std::sqrt(factor.bias_random_walk_reference_dt_s / std::max(dt_s, 1.0e-9));
}

double bias_random_walk_covariance_from_samples(
  const SlidingWindowImuFactor & factor,
  const double sigma)
{
  if (!std::isfinite(sigma) || sigma <= 0.0) {
    return 0.0;
  }
  double covariance = 0.0;
  int64_t previous_stamp_ns = factor.preintegration.start_stamp_ns();
  for (const auto & sample : factor.preintegration.samples()) {
    const double dt_s = static_cast<double>(sample.stamp_ns - previous_stamp_ns) / 1.0e9;
    if (std::isfinite(dt_s) && dt_s > 0.0) {
      covariance += dt_s * dt_s * sigma * sigma;
    }
    previous_stamp_ns = sample.stamp_ns;
  }
  if (covariance <= 0.0) {
    const double dt_s = factor.preintegration.delta_t_s();
    if (std::isfinite(dt_s) && dt_s > 0.0) {
      covariance = dt_s * dt_s * sigma * sigma;
    }
  }
  return covariance;
}

Eigen::Matrix<double, 6, 1> effective_bias_sqrt_information(
  const SlidingWindowImuFactor & factor)
{
  Eigen::Matrix<double, 6, 1> sqrt_information;
  sqrt_information.setZero();
  const double legacy_time_scale = bias_random_walk_time_scale(factor);
  auto scale = [legacy_time_scale, &factor](
      const double group_weight,
      const double sigma,
      const double covariance) {
      const double base = std::sqrt(factor.bias_weight * group_weight);
      if (sigma > 0.0 && covariance > std::numeric_limits<double>::epsilon()) {
        return base / std::sqrt(covariance);
      }
      return legacy_time_scale * base;
    };
  const double gyro_covariance =
    bias_random_walk_covariance_from_samples(factor, factor.gyro_bias_random_walk_sigma);
  const double accel_covariance =
    bias_random_walk_covariance_from_samples(factor, factor.accel_bias_random_walk_sigma);
  sqrt_information.template segment<3>(0).setConstant(
    scale(factor.gyro_bias_weight, factor.gyro_bias_random_walk_sigma, gyro_covariance));
  sqrt_information.template segment<3>(3).setConstant(
    scale(factor.accel_bias_weight, factor.accel_bias_random_walk_sigma, accel_covariance));
  return sqrt_information;
}

Eigen::Matrix<double, kSmoothnessResidualDof, 1> smoothness_residual(
  const SlidingWindowTrajectorySmoothnessFactor & factor,
  const SlidingWindowState & previous,
  const SlidingWindowState & current,
  const SlidingWindowState & next)
{
  const double previous_dt_s =
    static_cast<double>(current.stamp_ns - previous.stamp_ns) / 1.0e9;
  const double next_dt_s =
    static_cast<double>(next.stamp_ns - current.stamp_ns) / 1.0e9;
  Eigen::Matrix<double, kSmoothnessResidualDof, 1> residual;
  residual.setZero();
  if (previous_dt_s <= 0.0 || next_dt_s <= 0.0) {
    return residual;
  }

  const Eigen::Vector3d previous_rotation_rate =
    relative_rotation_vector(previous.q_w_i, current.q_w_i) / previous_dt_s;
  const Eigen::Vector3d next_rotation_rate =
    relative_rotation_vector(current.q_w_i, next.q_w_i) / next_dt_s;
  residual.template segment<3>(0) =
    std::sqrt(factor.rotation_rate_weight) *
    (next_rotation_rate - previous_rotation_rate - factor.target_rotation_rate_delta);

  const Eigen::Vector3d previous_position_rate =
    (current.p_w_i - previous.p_w_i) / previous_dt_s;
  const Eigen::Vector3d next_position_rate =
    (next.p_w_i - current.p_w_i) / next_dt_s;
  residual.template segment<3>(3) =
    std::sqrt(factor.position_rate_weight) *
    (next_position_rate - previous_position_rate - factor.target_position_rate_delta);

  const Eigen::Vector3d previous_velocity_acceleration =
    (current.v_w_i - previous.v_w_i) / previous_dt_s;
  const Eigen::Vector3d next_velocity_acceleration =
    (next.v_w_i - current.v_w_i) / next_dt_s;
  residual.template segment<3>(6) =
    std::sqrt(factor.velocity_acceleration_weight) *
    (next_velocity_acceleration - previous_velocity_acceleration -
    factor.target_velocity_acceleration_delta);

  residual.template segment<3>(15) =
    std::sqrt(factor.position_velocity_consistency_weight) *
    (next_position_rate - 0.5 * (current.v_w_i + next.v_w_i));

  const Eigen::Vector3d previous_gyro_bias_rate =
    (current.gyro_bias - previous.gyro_bias) / previous_dt_s;
  const Eigen::Vector3d next_gyro_bias_rate =
    (next.gyro_bias - current.gyro_bias) / next_dt_s;
  residual.template segment<3>(9) =
    std::sqrt(factor.gyro_bias_rate_weight) *
    (next_gyro_bias_rate - previous_gyro_bias_rate - factor.target_gyro_bias_rate_delta);

  const Eigen::Vector3d previous_accel_bias_rate =
    (current.accel_bias - previous.accel_bias) / previous_dt_s;
  const Eigen::Vector3d next_accel_bias_rate =
    (next.accel_bias - current.accel_bias) / next_dt_s;
  residual.template segment<3>(12) =
    std::sqrt(factor.accel_bias_rate_weight) *
    (next_accel_bias_rate - previous_accel_bias_rate - factor.target_accel_bias_rate_delta);
  return residual;
}

Eigen::Matrix<double, 3, 9> smoothness_rotation_jacobian(
  const SlidingWindowTrajectorySmoothnessFactor & factor,
  const SlidingWindowState & previous,
  const SlidingWindowState & current,
  const SlidingWindowState & next)
{
  Eigen::Matrix<double, 3, 9> jacobian = Eigen::Matrix<double, 3, 9>::Zero();
  if (factor.rotation_rate_weight <= 0.0) {
    return jacobian;
  }
  const double previous_dt_s =
    static_cast<double>(current.stamp_ns - previous.stamp_ns) / 1.0e9;
  const double next_dt_s =
    static_cast<double>(next.stamp_ns - current.stamp_ns) / 1.0e9;
  if (previous_dt_s <= 0.0 || next_dt_s <= 0.0) {
    return jacobian;
  }

  const Eigen::Vector3d previous_phi =
    relative_rotation_vector(previous.q_w_i, current.q_w_i);
  const Eigen::Vector3d next_phi =
    relative_rotation_vector(current.q_w_i, next.q_w_i);
  const Eigen::Matrix3d previous_left_inv = so3_left_jacobian_inverse(previous_phi);
  const Eigen::Matrix3d next_left_inv = so3_left_jacobian_inverse(next_phi);
  const Eigen::Matrix3d previous_rotation = previous.q_w_i.normalized().inverse().toRotationMatrix();
  const Eigen::Matrix3d current_rotation = current.q_w_i.normalized().inverse().toRotationMatrix();
  const double scale = std::sqrt(factor.rotation_rate_weight);

  jacobian.template block<3, 3>(0, 0) =
    scale * previous_left_inv * previous_rotation / previous_dt_s;
  jacobian.template block<3, 3>(0, 3) =
    scale * (
    -next_left_inv * current_rotation / next_dt_s -
    previous_left_inv * previous_rotation / previous_dt_s);
  jacobian.template block<3, 3>(0, 6) =
    scale * next_left_inv * current_rotation / next_dt_s;
  return jacobian;
}
}  // namespace

SchurComplementResult compute_schur_complement(
  const Eigen::MatrixXd & h_mm,
  const Eigen::MatrixXd & h_mr,
  const Eigen::MatrixXd & h_rr,
  const Eigen::VectorXd & rhs_m,
  const Eigen::VectorXd & rhs_r,
  const double damping)
{
  SchurComplementResult result;
  if (h_mm.rows() != h_mm.cols() || h_rr.rows() != h_rr.cols() ||
    h_mr.rows() != h_mm.rows() || h_mr.cols() != h_rr.rows() ||
    rhs_m.size() != h_mm.rows() || rhs_r.size() != h_rr.rows() ||
    damping < 0.0)
  {
    return result;
  }

  Eigen::MatrixXd regularized_mm = h_mm;
  if (damping > 0.0) {
    regularized_mm += damping * Eigen::MatrixXd::Identity(h_mm.rows(), h_mm.cols());
  }
  const Eigen::LDLT<Eigen::MatrixXd> ldlt(regularized_mm);
  if (ldlt.info() != Eigen::Success) {
    return result;
  }
  const Eigen::MatrixXd solved_mr = ldlt.solve(h_mr);
  const Eigen::VectorXd solved_rhs = ldlt.solve(rhs_m);
  if (!solved_mr.allFinite() || !solved_rhs.allFinite()) {
    return result;
  }

  result.hessian = h_rr - h_mr.transpose() * solved_mr;
  result.rhs = rhs_r - h_mr.transpose() * solved_rhs;
  result.valid = result.hessian.allFinite() && result.rhs.allFinite();
  return result;
}

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
  if (!std::isfinite(config.damping) || !std::isfinite(config.step_tolerance) ||
    !std::isfinite(config.numeric_epsilon) ||
    config.damping <= 0.0 || config.step_tolerance <= 0.0 || config.numeric_epsilon <= 0.0)
  {
    throw std::runtime_error("sliding window optimizer numeric settings must be positive");
  }
  if (!std::isfinite(config.marginalization_prior_weight) ||
    config.marginalization_prior_weight < 0.0)
  {
    throw std::runtime_error(
      "sliding window marginalization prior weight must be finite and non-negative");
  }
  if (!std::isfinite(config.max_rotation_step_rad) || !std::isfinite(config.max_velocity_step_mps) ||
    !std::isfinite(config.max_translation_step_m) || !std::isfinite(config.max_bias_step) ||
    config.max_rotation_step_rad < 0.0 || config.max_velocity_step_mps < 0.0 ||
    config.max_translation_step_m < 0.0 || config.max_bias_step < 0.0)
  {
    throw std::runtime_error("sliding window step limits must be finite and non-negative");
  }
  if (!std::isfinite(config.max_normal_equation_condition) ||
    !std::isfinite(config.min_normal_equation_rank_ratio) ||
    !std::isfinite(config.max_state_gap_s) ||
    config.max_normal_equation_condition < 0.0 ||
    config.min_normal_equation_rank_ratio < 0.0 ||
    config.min_normal_equation_rank_ratio > 1.0 ||
    config.max_state_gap_s < 0.0)
  {
    throw std::runtime_error(
      "sliding window normal-equation health gates must be finite and in range");
  }
  config_ = config;
}

void SlidingWindowOptimizer::clear()
{
  states_.clear();
  imu_factors_.clear();
  pose_priors_.clear();
  state_priors_.clear();
  dense_priors_.clear();
  point_factors_.clear();
  plane_factors_.clear();
  visual_factors_.clear();
  se3_photometric_factors_.clear();
  relative_translation_factors_.clear();
  relative_distance_factors_.clear();
  smoothness_factors_.clear();
  imu_factor_replacement_count_ = 0U;
  point_factor_replacement_count_ = 0U;
  plane_factor_replacement_count_ = 0U;
  visual_factor_replacement_count_ = 0U;
  se3_photometric_factor_replacement_count_ = 0U;
  relative_translation_factor_replacement_count_ = 0U;
  relative_distance_factor_replacement_count_ = 0U;
  smoothness_factor_replacement_count_ = 0U;
  marginalized_state_count_ = 0U;
  schur_marginalization_count_ = 0U;
  fallback_marginalization_prior_count_ = 0U;
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
  if (!state_is_finite(state)) {
    throw std::runtime_error("sliding-window state values must be finite with a non-zero quaternion");
  }
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
  if (!std::isfinite(factor.weight) || !std::isfinite(factor.rotation_weight) ||
    !std::isfinite(factor.velocity_weight) || !std::isfinite(factor.position_weight) ||
    !std::isfinite(factor.bias_weight) || !std::isfinite(factor.gyro_bias_weight) ||
    !std::isfinite(factor.accel_bias_weight) ||
    !std::isfinite(factor.bias_random_walk_reference_dt_s) ||
    !std::isfinite(factor.gyro_bias_random_walk_sigma) ||
    !std::isfinite(factor.accel_bias_random_walk_sigma) ||
    factor.weight <= 0.0 ||
    factor.rotation_weight < 0.0 || factor.velocity_weight < 0.0 ||
    factor.position_weight < 0.0 || factor.bias_weight < 0.0 ||
    factor.gyro_bias_weight < 0.0 || factor.accel_bias_weight < 0.0 ||
    factor.bias_random_walk_reference_dt_s < 0.0 ||
    factor.gyro_bias_random_walk_sigma < 0.0 ||
    factor.accel_bias_random_walk_sigma < 0.0)
  {
    throw std::runtime_error("IMU factor weights must be finite and valid");
  }
  if (!factor.sqrt_information.allFinite()) {
    throw std::runtime_error("IMU factor sqrt-information must be finite");
  }
  if (!has_full_imu_sqrt_information(factor) &&
    factor.rotation_weight == 0.0 && factor.velocity_weight == 0.0 &&
    factor.position_weight == 0.0 &&
    factor.bias_weight * factor.gyro_bias_weight == 0.0 &&
    factor.bias_weight * factor.accel_bias_weight == 0.0)
  {
    throw std::runtime_error("IMU factor must keep at least one residual block active");
  }
  if (factor.preintegration.start_stamp_ns() != factor.from_stamp_ns ||
    factor.preintegration.end_stamp_ns() != factor.to_stamp_ns)
  {
    throw std::runtime_error("IMU factor preintegration span must match factor timestamps");
  }
  if (!factor.gravity_w.allFinite() || !factor.preintegration.delta_q().coeffs().allFinite() ||
    factor.preintegration.delta_q().norm() <= std::numeric_limits<double>::epsilon() ||
    !factor.preintegration.delta_v().allFinite() || !factor.preintegration.delta_p().allFinite() ||
    !std::isfinite(factor.preintegration.delta_t_s()))
  {
    throw std::runtime_error("IMU factor preintegration values must be finite");
  }
  const auto existing = std::find_if(
    imu_factors_.begin(), imu_factors_.end(),
    [&factor](const SlidingWindowImuFactor & candidate) {
      return candidate.from_stamp_ns == factor.from_stamp_ns &&
             candidate.to_stamp_ns == factor.to_stamp_ns;
    });
  if (existing == imu_factors_.end()) {
    imu_factors_.push_back(factor);
  } else {
    *existing = factor;
    ++imu_factor_replacement_count_;
  }
}

void SlidingWindowOptimizer::add_pose_prior(const SlidingWindowPosePrior & prior)
{
  if (!std::isfinite(prior.translation_weight) || !std::isfinite(prior.rotation_weight) ||
    prior.translation_weight < 0.0 || prior.rotation_weight < 0.0)
  {
    throw std::runtime_error("pose prior weights must be finite and non-negative");
  }
  if (!prior.p_w_i.allFinite() || !quaternion_is_finite_and_nonzero(prior.q_w_i)) {
    throw std::runtime_error("pose prior values must be finite with a non-zero quaternion");
  }
  SlidingWindowPosePrior normalized = prior;
  normalized.q_w_i.normalize();
  const auto existing = std::find_if(
    pose_priors_.begin(), pose_priors_.end(),
    [&normalized](const SlidingWindowPosePrior & candidate) {
      return candidate.stamp_ns == normalized.stamp_ns;
    });
  if (existing == pose_priors_.end()) {
    pose_priors_.push_back(normalized);
  } else {
    *existing = normalized;
  }
}

void SlidingWindowOptimizer::add_state_prior(const SlidingWindowStatePrior & prior)
{
  if (!std::isfinite(prior.rotation_weight) || !std::isfinite(prior.velocity_weight) ||
    !std::isfinite(prior.position_weight) || !std::isfinite(prior.gyro_bias_weight) ||
    !std::isfinite(prior.accel_bias_weight) ||
    prior.rotation_weight < 0.0 || prior.velocity_weight < 0.0 ||
    prior.position_weight < 0.0 || prior.gyro_bias_weight < 0.0 ||
    prior.accel_bias_weight < 0.0)
  {
    throw std::runtime_error("state prior weights must be finite and non-negative");
  }
  if (!prior.p_w_i.allFinite() || !prior.v_w_i.allFinite() ||
    !prior.gyro_bias.allFinite() || !prior.accel_bias.allFinite() ||
    !quaternion_is_finite_and_nonzero(prior.q_w_i) || !prior.sqrt_information.allFinite())
  {
    throw std::runtime_error("state prior values must be finite with a non-zero quaternion");
  }
  SlidingWindowStatePrior normalized = prior;
  normalized.q_w_i.normalize();
  const auto existing = std::find_if(
    state_priors_.begin(), state_priors_.end(),
    [&normalized](const SlidingWindowStatePrior & candidate) {
      return candidate.stamp_ns == normalized.stamp_ns;
    });
  if (existing == state_priors_.end()) {
    state_priors_.push_back(normalized);
  } else {
    *existing = normalized;
  }
}

void SlidingWindowOptimizer::add_dense_prior(const SlidingWindowDensePrior & prior)
{
  const auto expected_cols = static_cast<Eigen::Index>(prior.reference_states.size() * kStateDof);
  if (prior.stamp_ns.size() != prior.reference_states.size() ||
    prior.sqrt_information.cols() != expected_cols ||
    prior.target_delta.size() != expected_cols ||
    prior.sqrt_information.rows() == 0)
  {
    throw std::runtime_error("dense prior dimensions must match referenced states");
  }
  if (!prior.sqrt_information.allFinite() || !prior.target_delta.allFinite()) {
    throw std::runtime_error("dense prior values must be finite");
  }
  SlidingWindowDensePrior normalized = prior;
  for (size_t index = 0; index < normalized.reference_states.size(); ++index) {
    auto & state = normalized.reference_states[index];
    if (!state_is_finite(state)) {
      throw std::runtime_error(
        "dense prior reference states must be finite with non-zero quaternions");
    }
    if (normalized.stamp_ns[index] != state.stamp_ns) {
      throw std::runtime_error("dense prior stamps must match reference-state stamps");
    }
    if (index > 0U && normalized.stamp_ns[index] <= normalized.stamp_ns[index - 1U]) {
      throw std::runtime_error("dense prior stamps must be strictly increasing");
    }
    state.q_w_i.normalize();
  }
  dense_priors_.push_back(std::move(normalized));
}

void SlidingWindowOptimizer::add_point_to_point_factor(const SlidingWindowPointToPointFactor & factor)
{
  if (!std::isfinite(factor.weight) || !std::isfinite(factor.huber_delta_m) ||
    factor.weight <= 0.0 || factor.huber_delta_m < 0.0)
  {
    throw std::runtime_error(
      "point-to-point factor weight must be positive and Huber delta must be non-negative");
  }
  if (factor.frame_points_i.size() != factor.target_points_w.size()) {
    throw std::runtime_error("point-to-point factor source/target sizes must match");
  }
  if (!factor.point_weights.empty() && factor.point_weights.size() != factor.frame_points_i.size()) {
    throw std::runtime_error("point-to-point factor weights must match correspondences");
  }
  for (size_t index = 0; index < factor.frame_points_i.size(); ++index) {
    if (!factor.frame_points_i[index].allFinite() || !factor.target_points_w[index].allFinite()) {
      throw std::runtime_error("point-to-point factor correspondences must be finite");
    }
    if (!factor.point_weights.empty() &&
      (!std::isfinite(factor.point_weights[index]) || factor.point_weights[index] <= 0.0 ||
      factor.point_weights[index] > 1.0))
    {
      throw std::runtime_error("point-to-point factor weights must be finite and in (0, 1]");
    }
  }
  if (factor.frame_points_i.empty()) {
    return;
  }
  const auto existing = std::find_if(
    point_factors_.begin(), point_factors_.end(),
    [&factor](const SlidingWindowPointToPointFactor & candidate) {
      return candidate.stamp_ns == factor.stamp_ns && candidate.source_id == factor.source_id;
    });
  if (existing == point_factors_.end()) {
    point_factors_.push_back(factor);
  } else {
    *existing = factor;
    ++point_factor_replacement_count_;
  }
}

void SlidingWindowOptimizer::add_point_to_plane_factor(const SlidingWindowPointToPlaneFactor & factor)
{
  if (!std::isfinite(factor.weight) || !std::isfinite(factor.huber_delta_m) ||
    factor.weight <= 0.0 || factor.huber_delta_m < 0.0)
  {
    throw std::runtime_error(
      "point-to-plane factor weight must be positive and Huber delta must be non-negative");
  }
  if (factor.frame_points_i.size() != factor.target_points_w.size() ||
    factor.frame_points_i.size() != factor.target_normals_w.size())
  {
    throw std::runtime_error("point-to-plane factor source/target/normal sizes must match");
  }
  if (!factor.point_weights.empty() && factor.point_weights.size() != factor.frame_points_i.size()) {
    throw std::runtime_error("point-to-plane factor weights must match correspondences");
  }
  for (size_t index = 0; index < factor.frame_points_i.size(); ++index) {
    if (!factor.frame_points_i[index].allFinite() || !factor.target_points_w[index].allFinite() ||
      !factor.target_normals_w[index].allFinite())
    {
      throw std::runtime_error("point-to-plane factor correspondences must be finite");
    }
    if (factor.target_normals_w[index].squaredNorm() <= std::numeric_limits<double>::epsilon()) {
      throw std::runtime_error("point-to-plane factor normals must be non-zero");
    }
    if (!factor.point_weights.empty() &&
      (!std::isfinite(factor.point_weights[index]) || factor.point_weights[index] <= 0.0 ||
      factor.point_weights[index] > 1.0))
    {
      throw std::runtime_error("point-to-plane factor weights must be finite and in (0, 1]");
    }
  }
  if (factor.frame_points_i.empty()) {
    return;
  }
  const auto existing = std::find_if(
    plane_factors_.begin(), plane_factors_.end(),
    [&factor](const SlidingWindowPointToPlaneFactor & candidate) {
      return candidate.stamp_ns == factor.stamp_ns && candidate.source_id == factor.source_id;
    });
  if (existing == plane_factors_.end()) {
    plane_factors_.push_back(factor);
  } else {
    *existing = factor;
    ++plane_factor_replacement_count_;
  }
}

void SlidingWindowOptimizer::add_visual_alignment_factor(const SlidingWindowVisualAlignmentFactor & factor)
{
  if (!std::isfinite(factor.weight) || !std::isfinite(factor.meters_per_pixel) ||
    factor.weight <= 0.0 || factor.meters_per_pixel <= 0.0)
  {
    throw std::runtime_error(
      "visual alignment factor weight and meters_per_pixel must be finite and positive");
  }
  if (!std::isfinite(factor.huber_delta_m) || factor.huber_delta_m < 0.0) {
    throw std::runtime_error("visual alignment factor Huber delta must be finite and non-negative");
  }
  if (!factor.measured_shift_px.allFinite() || !factor.reference_p_w_i.allFinite()) {
    throw std::runtime_error("visual alignment factor values must be finite");
  }
  if (!visual_support_is_valid(factor.support_stamp_ns, factor.support_weights)) {
    throw std::runtime_error("visual alignment factor support stamps/weights must be valid");
  }
  const auto existing = std::find_if(
    visual_factors_.begin(), visual_factors_.end(),
    [&factor](const SlidingWindowVisualAlignmentFactor & candidate) {
      return candidate.stamp_ns == factor.stamp_ns && candidate.source_id == factor.source_id;
    });
  if (existing == visual_factors_.end()) {
    visual_factors_.push_back(factor);
  } else {
    *existing = factor;
    ++visual_factor_replacement_count_;
  }
}

void SlidingWindowOptimizer::add_se3_photometric_factor(const SlidingWindowSe3PhotometricFactor & factor)
{
  if (!std::isfinite(factor.weight) || factor.weight <= 0.0) {
    throw std::runtime_error("SE3 photometric factor weight must be finite and positive");
  }
  if (!std::isfinite(factor.huber_delta) || factor.huber_delta < 0.0) {
    throw std::runtime_error("SE3 photometric factor Huber delta must be finite and non-negative");
  }
  if (!factor.reference_p_w_i.allFinite() || !factor.target_delta.allFinite() ||
    !factor.sqrt_information.allFinite() ||
    !quaternion_is_finite_and_nonzero(factor.reference_q_w_i))
  {
    throw std::runtime_error("SE3 photometric factor values must be finite with a non-zero quaternion");
  }
  if (!visual_support_is_valid(factor.support_stamp_ns, factor.support_weights)) {
    throw std::runtime_error("SE3 photometric factor support stamps/weights must be valid");
  }
  SlidingWindowSe3PhotometricFactor normalized = factor;
  normalized.reference_q_w_i.normalize();
  const auto existing = std::find_if(
    se3_photometric_factors_.begin(), se3_photometric_factors_.end(),
    [&normalized](const SlidingWindowSe3PhotometricFactor & candidate) {
      return candidate.stamp_ns == normalized.stamp_ns &&
             candidate.source_id == normalized.source_id;
    });
  if (existing == se3_photometric_factors_.end()) {
    se3_photometric_factors_.push_back(normalized);
  } else {
    *existing = normalized;
    ++se3_photometric_factor_replacement_count_;
  }
}

void SlidingWindowOptimizer::add_relative_translation_factor(
  const SlidingWindowRelativeTranslationFactor & factor)
{
  if (factor.from_stamp_ns >= factor.to_stamp_ns) {
    throw std::runtime_error("relative translation factor timestamps must be strictly increasing");
  }
  if (!std::isfinite(factor.weight) || !std::isfinite(factor.huber_delta_m) ||
    !std::isfinite(factor.rotation_weight) ||
    !std::isfinite(factor.rotation_huber_delta_rad) ||
    factor.weight <= 0.0 || factor.huber_delta_m < 0.0 ||
    factor.rotation_weight < 0.0 || factor.rotation_huber_delta_rad < 0.0)
  {
    throw std::runtime_error(
      "relative translation factor weights must be positive/non-negative and Huber deltas must be non-negative");
  }
  if (!factor.delta_p_w.allFinite() ||
    !quaternion_is_finite_and_nonzero(factor.delta_q_from_to))
  {
    throw std::runtime_error("relative translation factor deltas must be finite");
  }
  SlidingWindowRelativeTranslationFactor normalized = factor;
  normalized.delta_q_from_to.normalize();
  const auto existing = std::find_if(
    relative_translation_factors_.begin(), relative_translation_factors_.end(),
    [&normalized](const SlidingWindowRelativeTranslationFactor & candidate) {
      return candidate.from_stamp_ns == normalized.from_stamp_ns &&
             candidate.to_stamp_ns == normalized.to_stamp_ns &&
             candidate.source_id == normalized.source_id;
    });
  if (existing == relative_translation_factors_.end()) {
    relative_translation_factors_.push_back(normalized);
  } else {
    *existing = normalized;
    ++relative_translation_factor_replacement_count_;
  }
}

void SlidingWindowOptimizer::add_relative_distance_factor(
  const SlidingWindowRelativeDistanceFactor & factor)
{
  if (factor.from_stamp_ns >= factor.to_stamp_ns) {
    throw std::runtime_error("relative distance factor timestamps must be strictly increasing");
  }
  if (!std::isfinite(factor.distance_m) || !std::isfinite(factor.weight) ||
    !std::isfinite(factor.huber_delta_m) ||
    factor.distance_m < 0.0 || factor.weight <= 0.0 || factor.huber_delta_m < 0.0)
  {
    throw std::runtime_error(
            "relative distance factor distance/weight must be finite and valid");
  }
  const auto existing = std::find_if(
    relative_distance_factors_.begin(), relative_distance_factors_.end(),
    [&factor](const SlidingWindowRelativeDistanceFactor & candidate) {
      return candidate.from_stamp_ns == factor.from_stamp_ns &&
             candidate.to_stamp_ns == factor.to_stamp_ns &&
             candidate.source_id == factor.source_id;
    });
  if (existing == relative_distance_factors_.end()) {
    relative_distance_factors_.push_back(factor);
  } else {
    *existing = factor;
    ++relative_distance_factor_replacement_count_;
  }
}

void SlidingWindowOptimizer::add_trajectory_smoothness_factor(
  const SlidingWindowTrajectorySmoothnessFactor & factor)
{
  if (factor.previous_stamp_ns >= factor.current_stamp_ns ||
    factor.current_stamp_ns >= factor.next_stamp_ns)
  {
    throw std::runtime_error("trajectory smoothness factor timestamps must be strictly increasing");
  }
  if (!std::isfinite(factor.rotation_rate_weight) ||
    !std::isfinite(factor.position_rate_weight) ||
    !std::isfinite(factor.velocity_acceleration_weight) ||
    !std::isfinite(factor.position_velocity_consistency_weight) ||
    !std::isfinite(factor.gyro_bias_rate_weight) ||
    !std::isfinite(factor.accel_bias_rate_weight) ||
    !factor.target_rotation_rate_delta.allFinite() ||
    !factor.target_position_rate_delta.allFinite() ||
    !factor.target_velocity_acceleration_delta.allFinite() ||
    !factor.target_gyro_bias_rate_delta.allFinite() ||
    !factor.target_accel_bias_rate_delta.allFinite() ||
    factor.rotation_rate_weight < 0.0 || factor.position_rate_weight < 0.0 ||
    factor.velocity_acceleration_weight < 0.0 ||
    factor.position_velocity_consistency_weight < 0.0 ||
    factor.gyro_bias_rate_weight < 0.0 ||
    factor.accel_bias_rate_weight < 0.0)
  {
    throw std::runtime_error(
            "trajectory smoothness factor targets must be finite and weights must be non-negative");
  }
  if (factor.rotation_rate_weight == 0.0 && factor.position_rate_weight == 0.0 &&
    factor.velocity_acceleration_weight == 0.0 &&
    factor.position_velocity_consistency_weight == 0.0 &&
    factor.gyro_bias_rate_weight == 0.0 &&
    factor.accel_bias_rate_weight == 0.0)
  {
    return;
  }
  const auto existing = std::find_if(
    smoothness_factors_.begin(), smoothness_factors_.end(),
    [&factor](const SlidingWindowTrajectorySmoothnessFactor & candidate) {
      return candidate.previous_stamp_ns == factor.previous_stamp_ns &&
             candidate.current_stamp_ns == factor.current_stamp_ns &&
             candidate.next_stamp_ns == factor.next_stamp_ns;
    });
  if (existing == smoothness_factors_.end()) {
    smoothness_factors_.push_back(factor);
  } else {
    *existing = factor;
    ++smoothness_factor_replacement_count_;
  }
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
    const bool added_schur_prior = add_schur_marginalization_prior_for_front();
    if (added_schur_prior) {
      ++schur_marginalization_count_;
    }
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
    dense_priors_.erase(
      std::remove_if(
        dense_priors_.begin(), dense_priors_.end(),
        [stamp_ns](const SlidingWindowDensePrior & prior) {
          return std::find(prior.stamp_ns.begin(), prior.stamp_ns.end(), stamp_ns) !=
                 prior.stamp_ns.end();
        }),
      dense_priors_.end());
    point_factors_.erase(
      std::remove_if(
        point_factors_.begin(), point_factors_.end(),
        [stamp_ns](const SlidingWindowPointToPointFactor & factor) {
          return factor.stamp_ns == stamp_ns;
        }),
      point_factors_.end());
    plane_factors_.erase(
      std::remove_if(
        plane_factors_.begin(), plane_factors_.end(),
        [stamp_ns](const SlidingWindowPointToPlaneFactor & factor) {
          return factor.stamp_ns == stamp_ns;
        }),
      plane_factors_.end());
    visual_factors_.erase(
      std::remove_if(
        visual_factors_.begin(), visual_factors_.end(),
        [stamp_ns](const SlidingWindowVisualAlignmentFactor & factor) {
          return visual_factor_references_stamp(factor, stamp_ns);
        }),
      visual_factors_.end());
    se3_photometric_factors_.erase(
      std::remove_if(
        se3_photometric_factors_.begin(), se3_photometric_factors_.end(),
        [stamp_ns](const SlidingWindowSe3PhotometricFactor & factor) {
          return visual_factor_references_stamp(factor, stamp_ns);
        }),
      se3_photometric_factors_.end());
    relative_translation_factors_.erase(
      std::remove_if(
        relative_translation_factors_.begin(), relative_translation_factors_.end(),
        [stamp_ns](const SlidingWindowRelativeTranslationFactor & factor) {
          return factor.from_stamp_ns == stamp_ns || factor.to_stamp_ns == stamp_ns;
        }),
      relative_translation_factors_.end());
    relative_distance_factors_.erase(
      std::remove_if(
        relative_distance_factors_.begin(), relative_distance_factors_.end(),
        [stamp_ns](const SlidingWindowRelativeDistanceFactor & factor) {
          return factor.from_stamp_ns == stamp_ns || factor.to_stamp_ns == stamp_ns;
        }),
      relative_distance_factors_.end());
    smoothness_factors_.erase(
      std::remove_if(
        smoothness_factors_.begin(), smoothness_factors_.end(),
        [stamp_ns](const SlidingWindowTrajectorySmoothnessFactor & factor) {
          return factor.previous_stamp_ns == stamp_ns || factor.current_stamp_ns == stamp_ns ||
                 factor.next_stamp_ns == stamp_ns;
        }),
      smoothness_factors_.end());
    if (!added_schur_prior && !states_.empty() && config_.marginalization_prior_weight > 0.0) {
      const int64_t anchor_stamp_ns = states_.front().stamp_ns;
      state_priors_.erase(
        std::remove_if(
          state_priors_.begin(), state_priors_.end(),
          [anchor_stamp_ns](const SlidingWindowStatePrior & prior) {
            return prior.stamp_ns == anchor_stamp_ns;
          }),
        state_priors_.end());
      state_priors_.push_back(make_state_prior(states_.front()));
      ++fallback_marginalization_prior_count_;
    }
  }
  if (marginalized > 0U) {
    prune_marginalized_factor_references();
  }
  return marginalized;
}

size_t SlidingWindowOptimizer::prune_marginalized_factor_references()
{
  if (states_.empty()) {
    const size_t removed =
      imu_factors_.size() + pose_priors_.size() + state_priors_.size() + dense_priors_.size() +
      point_factors_.size() + plane_factors_.size() + visual_factors_.size() +
      se3_photometric_factors_.size() + relative_translation_factors_.size() +
      relative_distance_factors_.size() +
      smoothness_factors_.size();
    imu_factors_.clear();
    pose_priors_.clear();
    state_priors_.clear();
    dense_priors_.clear();
    point_factors_.clear();
    plane_factors_.clear();
    visual_factors_.clear();
    se3_photometric_factors_.clear();
    relative_translation_factors_.clear();
    relative_distance_factors_.clear();
    smoothness_factors_.clear();
    return removed;
  }

  const int64_t window_begin_ns = states_.front().stamp_ns;
  auto before_active_window = [window_begin_ns](const int64_t stamp_ns) {
      return stamp_ns < window_begin_ns;
    };
  auto erase_and_count = [](auto & container, auto predicate) {
      const auto old_size = container.size();
      container.erase(std::remove_if(container.begin(), container.end(), predicate), container.end());
      return old_size - container.size();
    };

  size_t removed = 0U;
  removed += erase_and_count(
    imu_factors_,
    [before_active_window](const SlidingWindowImuFactor & factor) {
      return before_active_window(factor.from_stamp_ns) || before_active_window(factor.to_stamp_ns);
    });
  removed += erase_and_count(
    pose_priors_,
    [before_active_window](const SlidingWindowPosePrior & prior) {
      return before_active_window(prior.stamp_ns);
    });
  removed += erase_and_count(
    state_priors_,
    [before_active_window](const SlidingWindowStatePrior & prior) {
      return before_active_window(prior.stamp_ns);
    });
  removed += erase_and_count(
    dense_priors_,
    [before_active_window](const SlidingWindowDensePrior & prior) {
      return std::any_of(prior.stamp_ns.begin(), prior.stamp_ns.end(), before_active_window);
    });
  removed += erase_and_count(
    point_factors_,
    [before_active_window](const SlidingWindowPointToPointFactor & factor) {
      return before_active_window(factor.stamp_ns);
    });
  removed += erase_and_count(
    plane_factors_,
    [before_active_window](const SlidingWindowPointToPlaneFactor & factor) {
      return before_active_window(factor.stamp_ns);
    });
  removed += erase_and_count(
    visual_factors_,
    [before_active_window](const SlidingWindowVisualAlignmentFactor & factor) {
      return visual_factor_has_stamp_matching(factor, before_active_window);
    });
  removed += erase_and_count(
    se3_photometric_factors_,
    [before_active_window](const SlidingWindowSe3PhotometricFactor & factor) {
      return visual_factor_has_stamp_matching(factor, before_active_window);
    });
  removed += erase_and_count(
    relative_translation_factors_,
    [before_active_window](const SlidingWindowRelativeTranslationFactor & factor) {
      return before_active_window(factor.from_stamp_ns) ||
             before_active_window(factor.to_stamp_ns);
    });
  removed += erase_and_count(
    relative_distance_factors_,
    [before_active_window](const SlidingWindowRelativeDistanceFactor & factor) {
      return before_active_window(factor.from_stamp_ns) ||
             before_active_window(factor.to_stamp_ns);
    });
  removed += erase_and_count(
    smoothness_factors_,
    [before_active_window](const SlidingWindowTrajectorySmoothnessFactor & factor) {
      return before_active_window(factor.previous_stamp_ns) ||
             before_active_window(factor.current_stamp_ns) ||
             before_active_window(factor.next_stamp_ns);
    });
  return removed;
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
  return relative_rotation_vector(measured_q, predicted_q);
}

Eigen::Matrix<double, 15, 1> SlidingWindowOptimizer::state_delta(
  const SlidingWindowState & reference,
  const SlidingWindowState & state)
{
  Eigen::Matrix<double, 15, 1> delta;
  delta.template segment<3>(0) = rotation_residual(reference.q_w_i, state.q_w_i);
  delta.template segment<3>(3) = state.v_w_i - reference.v_w_i;
  delta.template segment<3>(6) = state.p_w_i - reference.p_w_i;
  delta.template segment<3>(9) = state.gyro_bias - reference.gyro_bias;
  delta.template segment<3>(12) = state.accel_bias - reference.accel_bias;
  return delta;
}

Eigen::VectorXd SlidingWindowOptimizer::build_residual(
  const std::vector<SlidingWindowState> & states) const
{
  return build_residual(states, nullptr);
}

Eigen::VectorXd SlidingWindowOptimizer::build_residual(
  const std::vector<SlidingWindowState> & states,
  SlidingWindowCostBreakdown * breakdown) const
{
  if (breakdown != nullptr) {
    *breakdown = SlidingWindowCostBreakdown{};
  }
  std::vector<double> values;
  size_t relative_translation_residual_dof = 0U;
  for (const auto & factor : relative_translation_factors_) {
    relative_translation_residual_dof += 3U + (factor.rotation_weight > 0.0 ? 3U : 0U);
  }
  values.reserve(
    imu_factors_.size() * 15U + pose_priors_.size() * 6U + state_priors_.size() * 15U +
    dense_priors_.size() * 30U + point_factors_.size() * 300U + plane_factors_.size() * 100U +
    visual_factors_.size() * 2U + se3_photometric_factors_.size() * 6U +
    relative_translation_residual_dof +
    relative_distance_factors_.size() +
    smoothness_factors_.size() * kSmoothnessResidualDof);

  auto append = [&values, breakdown](
      const Eigen::VectorXd & residual,
      double SlidingWindowCostBreakdown::* cost_field = nullptr) {
      for (Eigen::Index i = 0; i < residual.size(); ++i) {
        values.push_back(residual[i]);
      }
      if (breakdown != nullptr && cost_field != nullptr) {
        (*breakdown).*cost_field += 0.5 * residual.squaredNorm();
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
  auto resolve_support = [&find_local](
      const int64_t stamp_ns,
      const std::vector<int64_t> & support_stamp_ns,
      const std::vector<double> & support_weights) {
      std::vector<std::pair<int, double>> support;
      if (support_stamp_ns.empty()) {
        const int index = find_local(stamp_ns);
        if (index >= 0) {
          support.emplace_back(index, 1.0);
        }
        return support;
      }
      if (!visual_support_is_valid(support_stamp_ns, support_weights)) {
        return support;
      }
      support.reserve(support_stamp_ns.size());
      for (size_t index = 0; index < support_stamp_ns.size(); ++index) {
        const int local_index = find_local(support_stamp_ns[index]);
        if (local_index < 0) {
          support.clear();
          return support;
        }
        if (support_weights[index] > 1.0e-9) {
          support.emplace_back(local_index, support_weights[index]);
        }
      }
      return support;
    };
  auto interpolated_position = [&states](
      const std::vector<std::pair<int, double>> & support) {
      Eigen::Vector3d position = Eigen::Vector3d::Zero();
      for (const auto & item : support) {
        position += item.second * states[static_cast<size_t>(item.first)].p_w_i;
      }
      return position;
    };
  auto interpolated_orientation = [&states](
      const std::vector<std::pair<int, double>> & support) {
      if (support.empty()) {
        return Eigen::Quaterniond::Identity();
      }
      if (support.size() == 1U) {
        return states[static_cast<size_t>(support.front().first)].q_w_i.normalized();
      }
      const auto & first = states[static_cast<size_t>(support[0].first)];
      const auto & second = states[static_cast<size_t>(support[1].first)];
      return first.q_w_i.normalized().slerp(support[1].second, second.q_w_i.normalized()).normalized();
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
    append(
      effective_imu_sqrt_information(factor) * residual.residual,
      &SlidingWindowCostBreakdown::imu_cost);
    Eigen::Matrix<double, 6, 1> bias_residual;
    const Eigen::Matrix<double, 6, 1> bias_sqrt_information =
      effective_bias_sqrt_information(factor);
    bias_residual.template segment<3>(0) =
      states[static_cast<size_t>(to)].gyro_bias - states[static_cast<size_t>(from)].gyro_bias;
    bias_residual.template segment<3>(3) =
      states[static_cast<size_t>(to)].accel_bias - states[static_cast<size_t>(from)].accel_bias;
    bias_residual.template segment<3>(0).array() *=
      bias_sqrt_information.template segment<3>(0).array();
    bias_residual.template segment<3>(3).array() *=
      bias_sqrt_information.template segment<3>(3).array();
    append(bias_residual, &SlidingWindowCostBreakdown::imu_cost);
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
    append(residual, &SlidingWindowCostBreakdown::pose_prior_cost);
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
    append(
      effective_state_prior_sqrt_information(prior) * residual,
      &SlidingWindowCostBreakdown::state_prior_cost);
  }

  for (const auto & prior : dense_priors_) {
    const auto prior_cols = static_cast<Eigen::Index>(prior.reference_states.size() * kStateDof);
    if (prior.sqrt_information.cols() != prior_cols || prior.target_delta.size() != prior_cols) {
      continue;
    }
    Eigen::VectorXd delta(prior_cols);
    bool complete = true;
    for (size_t reference_index = 0; reference_index < prior.reference_states.size(); ++reference_index) {
      const int index = find_local(prior.stamp_ns[reference_index]);
      if (index < 0) {
        complete = false;
        break;
      }
      delta.template segment<15>(static_cast<Eigen::Index>(reference_index * kStateDof)) =
        state_delta(prior.reference_states[reference_index], states[static_cast<size_t>(index)]);
    }
    if (complete) {
      append(
        prior.sqrt_information * (delta - prior.target_delta),
        &SlidingWindowCostBreakdown::dense_prior_cost);
    }
  }

  for (const auto & factor : point_factors_) {
    const int index = find_local(factor.stamp_ns);
    if (index < 0) {
      continue;
    }
    const auto & state = states[static_cast<size_t>(index)];
    for (size_t point_index = 0; point_index < factor.frame_points_i.size(); ++point_index) {
      const double base_weight = factor.point_weights.empty()
        ? 1.0
        : std::max(0.0, factor.point_weights[point_index]);
      Eigen::Vector3d residual =
        state.q_w_i * factor.frame_points_i[point_index] + state.p_w_i -
        factor.target_points_w[point_index];
      const double robust_weight = huber_weight(residual.norm(), factor.huber_delta_m);
      const double scale = std::sqrt(factor.weight * base_weight * robust_weight);
      append(scale * residual, &SlidingWindowCostBreakdown::point_factor_cost);
    }
  }

  for (const auto & factor : plane_factors_) {
    const int index = find_local(factor.stamp_ns);
    if (index < 0) {
      continue;
    }
    const auto & state = states[static_cast<size_t>(index)];
    Eigen::VectorXd residual(static_cast<Eigen::Index>(factor.frame_points_i.size()));
    for (size_t point_index = 0; point_index < factor.frame_points_i.size(); ++point_index) {
      const double base_weight = factor.point_weights.empty()
        ? 1.0
        : std::max(0.0, factor.point_weights[point_index]);
      const Eigen::Vector3d point_w =
        state.q_w_i * factor.frame_points_i[point_index] + state.p_w_i;
      const double point_residual = factor.target_normals_w[point_index].normalized().dot(
        point_w - factor.target_points_w[point_index]);
      const double robust_weight = huber_weight(std::abs(point_residual), factor.huber_delta_m);
      const double scale = std::sqrt(factor.weight * base_weight * robust_weight);
      residual[static_cast<Eigen::Index>(point_index)] = scale * point_residual;
    }
    append(residual, &SlidingWindowCostBreakdown::plane_factor_cost);
  }

  for (const auto & factor : visual_factors_) {
    const auto support = resolve_support(
      factor.stamp_ns, factor.support_stamp_ns, factor.support_weights);
    if (support.empty()) {
      continue;
    }
    const Eigen::Vector3d position = interpolated_position(support);
    Eigen::Vector2d target_xy;
    target_xy.x() = factor.reference_p_w_i.x() + factor.measured_shift_px.x() * factor.meters_per_pixel;
    target_xy.y() = factor.reference_p_w_i.y() + factor.measured_shift_px.y() * factor.meters_per_pixel;
    Eigen::Vector2d residual;
    residual.x() = position.x() - target_xy.x();
    residual.y() = position.y() - target_xy.y();
    const double robust_weight = huber_weight(residual.norm(), factor.huber_delta_m);
    append(
      std::sqrt(factor.weight * robust_weight) * residual,
      &SlidingWindowCostBreakdown::visual_factor_cost);
  }

  for (const auto & factor : se3_photometric_factors_) {
    const auto support = resolve_support(
      factor.stamp_ns, factor.support_stamp_ns, factor.support_weights);
    if (support.empty()) {
      continue;
    }
    const Eigen::Vector3d position = interpolated_position(support);
    const Eigen::Quaterniond orientation = interpolated_orientation(support);
    Eigen::Matrix<double, 6, 1> delta;
    delta.template segment<3>(0) = rotation_residual(factor.reference_q_w_i, orientation);
    delta.template segment<3>(3) = position - factor.reference_p_w_i;
    const Eigen::Matrix<double, 6, 1> whitened_residual =
      factor.sqrt_information * (delta - factor.target_delta);
    const double robust_weight = huber_weight(whitened_residual.norm(), factor.huber_delta);
    append(
      std::sqrt(factor.weight * robust_weight) * whitened_residual,
      &SlidingWindowCostBreakdown::se3_photometric_factor_cost);
  }

  for (const auto & factor : relative_translation_factors_) {
    const int from = find_local(factor.from_stamp_ns);
    const int to = find_local(factor.to_stamp_ns);
    if (from < 0 || to < 0) {
      continue;
    }
    const Eigen::Vector3d predicted_delta_p =
      factor.translation_in_from_frame ?
      states[static_cast<size_t>(from)].q_w_i.conjugate() *
      (states[static_cast<size_t>(to)].p_w_i - states[static_cast<size_t>(from)].p_w_i) :
      states[static_cast<size_t>(to)].p_w_i - states[static_cast<size_t>(from)].p_w_i;
    const Eigen::Vector3d residual =
      predicted_delta_p - factor.delta_p_w;
    const double robust_weight = huber_weight(residual.norm(), factor.huber_delta_m);
    append(
      std::sqrt(factor.weight * robust_weight) * residual,
      &SlidingWindowCostBreakdown::relative_translation_factor_cost);
    if (factor.rotation_weight > 0.0) {
      const Eigen::Quaterniond predicted_delta_q =
        states[static_cast<size_t>(from)].q_w_i.conjugate() *
        states[static_cast<size_t>(to)].q_w_i;
      const Eigen::Vector3d rotation_residual =
        SlidingWindowOptimizer::rotation_residual(factor.delta_q_from_to, predicted_delta_q);
      const double rotation_robust_weight =
        huber_weight(rotation_residual.norm(), factor.rotation_huber_delta_rad);
      append(
        std::sqrt(factor.rotation_weight * rotation_robust_weight) * rotation_residual,
        &SlidingWindowCostBreakdown::relative_translation_factor_cost);
    }
  }

  for (const auto & factor : relative_distance_factors_) {
    const int from = find_local(factor.from_stamp_ns);
    const int to = find_local(factor.to_stamp_ns);
    if (from < 0 || to < 0) {
      continue;
    }
    const Eigen::Vector3d delta_p =
      states[static_cast<size_t>(to)].p_w_i - states[static_cast<size_t>(from)].p_w_i;
    const double predicted_distance = delta_p.norm();
    const double residual_value = predicted_distance - factor.distance_m;
    const double robust_weight = huber_weight(std::abs(residual_value), factor.huber_delta_m);
    Eigen::VectorXd residual(1);
    residual[0] = std::sqrt(factor.weight * robust_weight) * residual_value;
    append(residual, &SlidingWindowCostBreakdown::relative_distance_factor_cost);
  }

  for (const auto & factor : smoothness_factors_) {
    const int previous = find_local(factor.previous_stamp_ns);
    const int current = find_local(factor.current_stamp_ns);
    const int next = find_local(factor.next_stamp_ns);
    if (previous < 0 || current < 0 || next < 0) {
      continue;
    }
    append(
      smoothness_residual(
        factor,
        states[static_cast<size_t>(previous)],
        states[static_cast<size_t>(current)],
        states[static_cast<size_t>(next)]),
      &SlidingWindowCostBreakdown::smoothness_factor_cost);
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

std::vector<SlidingWindowOptimizer::NumericJacobianBlock> SlidingWindowOptimizer::fill_analytic_jacobian(
  const std::vector<SlidingWindowState> & states,
  const std::vector<VariableBlock> & variables,
  Eigen::MatrixXd & jacobian) const
{
  std::vector<NumericJacobianBlock> numeric_blocks;
  auto mark_numeric = [&numeric_blocks](
      const Eigen::Index row_start,
      const Eigen::Index row_size,
      const Eigen::Index column_start,
      const Eigen::Index column_size) {
      if (row_size > 0 && column_size > 0) {
        numeric_blocks.push_back(NumericJacobianBlock{
            row_start, row_size, column_start, column_size});
      }
    };
  auto fallback_to_numeric = [&numeric_blocks, &mark_numeric, &jacobian]() {
      numeric_blocks.clear();
      mark_numeric(0, jacobian.rows(), 0, jacobian.cols());
      return numeric_blocks;
    };
  auto rows_available = [&jacobian](const Eigen::Index start, const Eigen::Index size) {
      return start >= 0 && size >= 0 && start + size <= jacobian.rows();
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
  auto resolve_support = [&find_local](
      const int64_t stamp_ns,
      const std::vector<int64_t> & support_stamp_ns,
      const std::vector<double> & support_weights) {
      std::vector<std::pair<int, double>> support;
      if (support_stamp_ns.empty()) {
        const int index = find_local(stamp_ns);
        if (index >= 0) {
          support.emplace_back(index, 1.0);
        }
        return support;
      }
      if (!visual_support_is_valid(support_stamp_ns, support_weights)) {
        return support;
      }
      support.reserve(support_stamp_ns.size());
      for (size_t index = 0; index < support_stamp_ns.size(); ++index) {
        const int local_index = find_local(support_stamp_ns[index]);
        if (local_index < 0) {
          support.clear();
          return support;
        }
        if (support_weights[index] > 1.0e-9) {
          support.emplace_back(local_index, support_weights[index]);
        }
      }
      return support;
    };

  std::vector<Eigen::Index> variable_offsets(states.size(), -1);
  for (const auto & variable : variables) {
    if (variable.state_index < variable_offsets.size()) {
      variable_offsets[variable.state_index] = static_cast<Eigen::Index>(variable.offset);
    }
  }

  Eigen::Index row = 0;
  for (const auto & factor : imu_factors_) {
    const int from = find_local(factor.from_stamp_ns);
    const int to = find_local(factor.to_stamp_ns);
    if (from < 0 || to < 0) {
      continue;
    }
    const auto & from_state = states[static_cast<size_t>(from)];
    const auto & to_state = states[static_cast<size_t>(to)];
    const ImuBias start_bias{from_state.gyro_bias, from_state.accel_bias};
    const auto corrected_preintegration = factor.preintegration.reintegrated(start_bias);
    const Eigen::Matrix<double, 9, 9> imu_sqrt_information =
      effective_imu_sqrt_information(factor);
    const Eigen::Matrix3d r_i_w = from_state.q_w_i.normalized().inverse().toRotationMatrix();
    const Eigen::Quaterniond predicted_delta_q =
      from_state.q_w_i.normalized().inverse() * to_state.q_w_i.normalized();
    const Eigen::Matrix3d rotation_middle_jacobian =
      rotation_residual_middle_perturbation_jacobian(
      corrected_preintegration.delta_q(), predicted_delta_q);
    const Eigen::Vector3d delta_v_w =
      to_state.v_w_i - from_state.v_w_i -
      factor.gravity_w * corrected_preintegration.delta_t_s();
    const Eigen::Vector3d delta_p_w =
      to_state.p_w_i - from_state.p_w_i -
      from_state.v_w_i * corrected_preintegration.delta_t_s() -
      0.5 * factor.gravity_w * corrected_preintegration.delta_t_s() *
      corrected_preintegration.delta_t_s();
    if (!rows_available(row + 9, 6)) {
      return fallback_to_numeric();
    }
    const Eigen::Matrix<double, 6, 1> bias_sqrt_information =
      effective_bias_sqrt_information(factor);
    const double gyro_bias_scale = bias_sqrt_information[0];
    const double accel_bias_scale = bias_sqrt_information[3];
    const Eigen::Index from_offset = variable_offsets[static_cast<size_t>(from)];
    const Eigen::Index to_offset = variable_offsets[static_cast<size_t>(to)];
    if (from_offset >= 0) {
      const Eigen::Matrix<double, 9, 6> bias_residual_jacobian =
        imu_preintegration_bias_residual_jacobian(factor, from_state, to_state);
      Eigen::Matrix<double, 9, 15> raw_jacobian = Eigen::Matrix<double, 9, 15>::Zero();
      raw_jacobian.template block<3, 3>(0, 0) =
        rotation_middle_jacobian * (-r_i_w);
      raw_jacobian.template block<3, 3>(3, 0) =
        r_i_w * skew_symmetric(delta_v_w);
      raw_jacobian.template block<3, 3>(3, 3) = -r_i_w;
      raw_jacobian.template block<3, 3>(6, 0) =
        r_i_w * skew_symmetric(delta_p_w);
      raw_jacobian.template block<3, 3>(6, 3) =
        -corrected_preintegration.delta_t_s() * r_i_w;
      raw_jacobian.template block<3, 3>(6, 6) = -r_i_w;
      raw_jacobian.block(0, 9, 9, 6) = bias_residual_jacobian;
      jacobian.block(row, from_offset, 9, static_cast<Eigen::Index>(kStateDof)) =
        imu_sqrt_information * raw_jacobian;
      jacobian.template block<3, 3>(row + 9, from_offset + 9) =
        -gyro_bias_scale * Eigen::Matrix3d::Identity();
      jacobian.template block<3, 3>(row + 12, from_offset + 12) =
        -accel_bias_scale * Eigen::Matrix3d::Identity();
    }
    if (to_offset >= 0) {
      Eigen::Matrix<double, 9, 15> raw_jacobian = Eigen::Matrix<double, 9, 15>::Zero();
      raw_jacobian.template block<3, 3>(0, 0) = rotation_middle_jacobian * r_i_w;
      raw_jacobian.template block<3, 3>(3, 3) = r_i_w;
      raw_jacobian.template block<3, 3>(6, 6) = r_i_w;
      jacobian.block(row, to_offset, 9, static_cast<Eigen::Index>(kStateDof)) =
        imu_sqrt_information * raw_jacobian;
      jacobian.template block<3, 3>(row + 9, to_offset + 9) =
        gyro_bias_scale * Eigen::Matrix3d::Identity();
      jacobian.template block<3, 3>(row + 12, to_offset + 12) =
        accel_bias_scale * Eigen::Matrix3d::Identity();
    }
    row += 15;
  }

  for (const auto & prior : pose_priors_) {
    const int index = find_local(prior.stamp_ns);
    if (index < 0) {
      continue;
    }
    const Eigen::Index offset = variable_offsets[static_cast<size_t>(index)];
    if (!rows_available(row, 6)) {
      return fallback_to_numeric();
    }
    if (offset >= 0) {
      const auto & state = states[static_cast<size_t>(index)];
      jacobian.template block<3, 3>(row, offset) =
        std::sqrt(prior.rotation_weight) *
        rotation_residual_left_perturbation_jacobian(prior.q_w_i, state.q_w_i);
      jacobian.template block<3, 3>(row + 3, offset + 6) =
        std::sqrt(prior.translation_weight) * Eigen::Matrix3d::Identity();
    }
    row += 6;
  }

  for (const auto & prior : state_priors_) {
    const int index = find_local(prior.stamp_ns);
    if (index < 0) {
      continue;
    }
    const Eigen::Index offset = variable_offsets[static_cast<size_t>(index)];
    if (!rows_available(row, static_cast<Eigen::Index>(kStateDof))) {
      return fallback_to_numeric();
    }
    if (offset >= 0) {
      const auto & state = states[static_cast<size_t>(index)];
      jacobian.template block<15, 15>(row, offset) =
        effective_state_prior_sqrt_information(prior) *
        state_delta_left_perturbation_jacobian(prior.q_w_i, state.q_w_i);
    }
    row += 15;
  }

  for (const auto & prior : dense_priors_) {
    const auto prior_cols = static_cast<Eigen::Index>(prior.reference_states.size() * kStateDof);
    if (prior.sqrt_information.cols() != prior_cols || prior.target_delta.size() != prior_cols) {
      continue;
    }
    bool complete = true;
    std::vector<int> local_indices;
    local_indices.reserve(prior.stamp_ns.size());
    for (const auto stamp_ns : prior.stamp_ns) {
      const int local_index = find_local(stamp_ns);
      if (local_index < 0) {
        complete = false;
        break;
      }
      local_indices.push_back(local_index);
    }
    if (complete) {
      if (!rows_available(row, prior.sqrt_information.rows())) {
        return fallback_to_numeric();
      }
      for (size_t reference_index = 0; reference_index < local_indices.size(); ++reference_index) {
        const auto local_index = static_cast<size_t>(local_indices[reference_index]);
        const Eigen::Index offset = variable_offsets[local_index];
        if (offset < 0) {
          continue;
        }
        const Eigen::Index prior_offset =
          static_cast<Eigen::Index>(reference_index * kStateDof);
        jacobian.block(row, offset, prior.sqrt_information.rows(), static_cast<Eigen::Index>(kStateDof)) =
          prior.sqrt_information.block(
          0, prior_offset, prior.sqrt_information.rows(), static_cast<Eigen::Index>(kStateDof)) *
          state_delta_left_perturbation_jacobian(
          prior.reference_states[reference_index].q_w_i,
          states[local_index].q_w_i);
      }
      row += prior.sqrt_information.rows();
    }
  }

  for (const auto & factor : point_factors_) {
    const int index = find_local(factor.stamp_ns);
    if (index < 0) {
      continue;
    }
    const auto & state = states[static_cast<size_t>(index)];
    const Eigen::Index offset = variable_offsets[static_cast<size_t>(index)];
    for (size_t point_index = 0; point_index < factor.frame_points_i.size(); ++point_index) {
      if (!rows_available(row, 3)) {
        return fallback_to_numeric();
      }
      if (offset >= 0) {
        const double base_weight = factor.point_weights.empty()
          ? 1.0
          : std::max(0.0, factor.point_weights[point_index]);
        const Eigen::Vector3d rotated_point = state.q_w_i * factor.frame_points_i[point_index];
        const Eigen::Vector3d residual =
          rotated_point + state.p_w_i - factor.target_points_w[point_index];
        const double robust_weight = huber_weight(residual.norm(), factor.huber_delta_m);
        const double scale = std::sqrt(factor.weight * base_weight * robust_weight);
        jacobian.template block<3, 3>(row, offset) =
          scale * -skew_symmetric(rotated_point);
        jacobian.template block<3, 3>(row, offset + 6) =
          scale * Eigen::Matrix3d::Identity();
      }
      row += 3;
    }
  }

  for (const auto & factor : plane_factors_) {
    const int index = find_local(factor.stamp_ns);
    if (index < 0) {
      continue;
    }
    const auto & state = states[static_cast<size_t>(index)];
    const Eigen::Index offset = variable_offsets[static_cast<size_t>(index)];
    for (size_t point_index = 0; point_index < factor.frame_points_i.size(); ++point_index) {
      if (!rows_available(row, 1)) {
        return fallback_to_numeric();
      }
      if (offset >= 0) {
        const double base_weight = factor.point_weights.empty()
          ? 1.0
          : std::max(0.0, factor.point_weights[point_index]);
        const Eigen::Vector3d normal = factor.target_normals_w[point_index].normalized();
        const Eigen::Vector3d rotated_point = state.q_w_i * factor.frame_points_i[point_index];
        const double residual = normal.dot(
          rotated_point + state.p_w_i - factor.target_points_w[point_index]);
        const double robust_weight = huber_weight(std::abs(residual), factor.huber_delta_m);
        const double scale = std::sqrt(factor.weight * base_weight * robust_weight);
        jacobian.template block<1, 3>(row, offset) =
          scale * -normal.transpose() * skew_symmetric(rotated_point);
        jacobian.template block<1, 3>(row, offset + 6) =
          scale * normal.transpose();
      }
      ++row;
    }
  }

  for (const auto & factor : visual_factors_) {
    const auto support = resolve_support(
      factor.stamp_ns, factor.support_stamp_ns, factor.support_weights);
    if (support.empty()) {
      continue;
    }
    if (!rows_available(row, 2)) {
      return fallback_to_numeric();
    }
    Eigen::Vector2d target_xy;
    target_xy.x() = factor.reference_p_w_i.x() + factor.measured_shift_px.x() * factor.meters_per_pixel;
    target_xy.y() = factor.reference_p_w_i.y() + factor.measured_shift_px.y() * factor.meters_per_pixel;
    Eigen::Vector2d predicted_xy = Eigen::Vector2d::Zero();
    for (const auto & item : support) {
      predicted_xy += item.second * states[static_cast<size_t>(item.first)].p_w_i.head<2>();
    }
    const Eigen::Vector2d residual = predicted_xy - target_xy;
    const double scale = std::sqrt(factor.weight * huber_weight(residual.norm(), factor.huber_delta_m));
    for (const auto & item : support) {
      const Eigen::Index offset = variable_offsets[static_cast<size_t>(item.first)];
      if (offset >= 0) {
        jacobian(row, offset + 6) = scale * item.second;
        jacobian(row + 1, offset + 7) = scale * item.second;
      }
    }
    row += 2;
  }

  for (const auto & factor : se3_photometric_factors_) {
    const auto support = resolve_support(
      factor.stamp_ns, factor.support_stamp_ns, factor.support_weights);
    if (support.empty()) {
      continue;
    }
    if (!rows_available(row, 6)) {
      return fallback_to_numeric();
    }
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    for (const auto & item : support) {
      position += item.second * states[static_cast<size_t>(item.first)].p_w_i;
    }
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    if (support.size() == 1U) {
      orientation = states[static_cast<size_t>(support.front().first)].q_w_i.normalized();
    } else {
      const auto & first = states[static_cast<size_t>(support[0].first)];
      const auto & second = states[static_cast<size_t>(support[1].first)];
      orientation = first.q_w_i.normalized().slerp(
        support[1].second, second.q_w_i.normalized()).normalized();
    }
    Eigen::Matrix<double, 6, 1> delta;
    delta.template segment<3>(0) = rotation_residual(factor.reference_q_w_i, orientation);
    delta.template segment<3>(3) = position - factor.reference_p_w_i;
    const Eigen::Matrix<double, 6, 1> whitened_residual =
      factor.sqrt_information * (delta - factor.target_delta);
    const double robust_scale =
      std::sqrt(factor.weight * huber_weight(whitened_residual.norm(), factor.huber_delta));
    const Eigen::Matrix3d rotation_jacobian =
      rotation_residual_left_perturbation_jacobian(factor.reference_q_w_i, orientation);
    for (const auto & item : support) {
      const Eigen::Index offset = variable_offsets[static_cast<size_t>(item.first)];
      if (offset >= 0) {
        Eigen::Matrix<double, 6, 15> delta_jacobian = Eigen::Matrix<double, 6, 15>::Zero();
        delta_jacobian.template block<3, 3>(0, 0) = item.second * rotation_jacobian;
        delta_jacobian.template block<3, 3>(3, 6) = item.second * Eigen::Matrix3d::Identity();
        jacobian.block(row, offset, 6, static_cast<Eigen::Index>(kStateDof)) =
          robust_scale * factor.sqrt_information * delta_jacobian;
      }
    }
    row += 6;
  }

  for (const auto & factor : relative_translation_factors_) {
    const int from = find_local(factor.from_stamp_ns);
    const int to = find_local(factor.to_stamp_ns);
    if (from < 0 || to < 0) {
      continue;
    }
    if (!rows_available(row, 3)) {
      return fallback_to_numeric();
    }
    const Eigen::Matrix3d r_from_w =
      states[static_cast<size_t>(from)].q_w_i.normalized().inverse().toRotationMatrix();
    const Eigen::Vector3d delta_p_w =
      states[static_cast<size_t>(to)].p_w_i - states[static_cast<size_t>(from)].p_w_i;
    const Eigen::Vector3d predicted_delta_p =
      factor.translation_in_from_frame ?
      r_from_w * delta_p_w :
      delta_p_w;
    const Eigen::Vector3d residual = predicted_delta_p - factor.delta_p_w;
    const double scale = std::sqrt(
      factor.weight * huber_weight(residual.norm(), factor.huber_delta_m));
    const Eigen::Index from_offset = variable_offsets[static_cast<size_t>(from)];
    const Eigen::Index to_offset = variable_offsets[static_cast<size_t>(to)];
    if (factor.translation_in_from_frame) {
      if (from_offset >= 0) {
        jacobian.template block<3, 3>(row, from_offset) =
          scale * r_from_w * skew_symmetric(delta_p_w);
        jacobian.template block<3, 3>(row, from_offset + 6) =
          -scale * r_from_w;
      }
      if (to_offset >= 0) {
        jacobian.template block<3, 3>(row, to_offset + 6) =
          scale * r_from_w;
      }
    } else {
      if (from_offset >= 0) {
        jacobian.template block<3, 3>(row, from_offset + 6) =
          -scale * Eigen::Matrix3d::Identity();
      }
      if (to_offset >= 0) {
        jacobian.template block<3, 3>(row, to_offset + 6) =
          scale * Eigen::Matrix3d::Identity();
      }
    }
    row += 3;
    if (factor.rotation_weight > 0.0) {
      if (!rows_available(row, 3)) {
        return fallback_to_numeric();
      }
      const Eigen::Quaterniond predicted_delta_q =
        states[static_cast<size_t>(from)].q_w_i.normalized().inverse() *
        states[static_cast<size_t>(to)].q_w_i.normalized();
      const Eigen::Vector3d rotation_residual =
        SlidingWindowOptimizer::rotation_residual(factor.delta_q_from_to, predicted_delta_q);
      const double rotation_scale = std::sqrt(
        factor.rotation_weight *
        huber_weight(rotation_residual.norm(), factor.rotation_huber_delta_rad));
      const Eigen::Matrix3d rotation_middle_jacobian =
        rotation_residual_middle_perturbation_jacobian(
        factor.delta_q_from_to, predicted_delta_q);
      if (from_offset >= 0) {
        jacobian.template block<3, 3>(row, from_offset) =
          rotation_scale * rotation_middle_jacobian * (-r_from_w);
      }
      if (to_offset >= 0) {
        jacobian.template block<3, 3>(row, to_offset) =
          rotation_scale * rotation_middle_jacobian * r_from_w;
      }
      row += 3;
    }
  }

  for (const auto & factor : relative_distance_factors_) {
    const int from = find_local(factor.from_stamp_ns);
    const int to = find_local(factor.to_stamp_ns);
    if (from < 0 || to < 0) {
      continue;
    }
    if (!rows_available(row, 1)) {
      return fallback_to_numeric();
    }
    const Eigen::Vector3d delta_p =
      states[static_cast<size_t>(to)].p_w_i - states[static_cast<size_t>(from)].p_w_i;
    const double predicted_distance = delta_p.norm();
    const double residual_value = predicted_distance - factor.distance_m;
    const double scale = std::sqrt(
      factor.weight * huber_weight(std::abs(residual_value), factor.huber_delta_m));
    if (predicted_distance > 1.0e-12) {
      const Eigen::RowVector3d direction = delta_p.transpose() / predicted_distance;
      const Eigen::Index from_offset = variable_offsets[static_cast<size_t>(from)];
      const Eigen::Index to_offset = variable_offsets[static_cast<size_t>(to)];
      if (from_offset >= 0) {
        jacobian.template block<1, 3>(row, from_offset + 6) = -scale * direction;
      }
      if (to_offset >= 0) {
        jacobian.template block<1, 3>(row, to_offset + 6) = scale * direction;
      }
    } else {
      mark_numeric(row, 1, 0, jacobian.cols());
    }
    ++row;
  }

  for (const auto & factor : smoothness_factors_) {
    const int previous = find_local(factor.previous_stamp_ns);
    const int current = find_local(factor.current_stamp_ns);
    const int next = find_local(factor.next_stamp_ns);
    if (previous < 0 || current < 0 || next < 0) {
      continue;
    }
    if (!rows_available(row, static_cast<Eigen::Index>(kSmoothnessResidualDof))) {
      return fallback_to_numeric();
    }
    const double previous_dt_s =
      static_cast<double>(
      states[static_cast<size_t>(current)].stamp_ns -
      states[static_cast<size_t>(previous)].stamp_ns) / 1.0e9;
    const double next_dt_s =
      static_cast<double>(
      states[static_cast<size_t>(next)].stamp_ns -
      states[static_cast<size_t>(current)].stamp_ns) / 1.0e9;
    if (previous_dt_s <= 0.0 || next_dt_s <= 0.0) {
      row += static_cast<Eigen::Index>(kSmoothnessResidualDof);
      continue;
    }
    const Eigen::Matrix<double, 3, 9> rotation_jacobian =
      smoothness_rotation_jacobian(
      factor,
      states[static_cast<size_t>(previous)],
      states[static_cast<size_t>(current)],
      states[static_cast<size_t>(next)]);
    const struct SmoothnessBlock
    {
      int state_index;
      Eigen::Index rotation_column;
      double position_scale;
      double velocity_scale;
      double bias_scale;
    } blocks[3] = {
      {previous, 0, 1.0 / previous_dt_s, 1.0 / previous_dt_s, 1.0 / previous_dt_s},
      {current, 3, -1.0 / next_dt_s - 1.0 / previous_dt_s,
        -1.0 / next_dt_s - 1.0 / previous_dt_s,
        -1.0 / next_dt_s - 1.0 / previous_dt_s},
      {next, 6, 1.0 / next_dt_s, 1.0 / next_dt_s, 1.0 / next_dt_s}
    };
    for (const auto & block : blocks) {
      const Eigen::Index offset = variable_offsets[static_cast<size_t>(block.state_index)];
      if (offset < 0) {
        continue;
      }
      if (factor.rotation_rate_weight > 0.0) {
        jacobian.template block<3, 3>(row, offset) =
          rotation_jacobian.template block<3, 3>(0, block.rotation_column);
      }
      jacobian.template block<3, 3>(row + 3, offset + 6) =
        std::sqrt(factor.position_rate_weight) * block.position_scale *
        Eigen::Matrix3d::Identity();
      jacobian.template block<3, 3>(row + 6, offset + 3) =
        std::sqrt(factor.velocity_acceleration_weight) * block.velocity_scale *
        Eigen::Matrix3d::Identity();
      jacobian.template block<3, 3>(row + 9, offset + 9) =
        std::sqrt(factor.gyro_bias_rate_weight) * block.bias_scale *
        Eigen::Matrix3d::Identity();
      jacobian.template block<3, 3>(row + 12, offset + 12) =
        std::sqrt(factor.accel_bias_rate_weight) * block.bias_scale *
        Eigen::Matrix3d::Identity();
    }
    if (factor.position_velocity_consistency_weight > 0.0) {
      const double consistency_scale = std::sqrt(factor.position_velocity_consistency_weight);
      const Eigen::Index current_offset = variable_offsets[static_cast<size_t>(current)];
      const Eigen::Index next_offset = variable_offsets[static_cast<size_t>(next)];
      if (current_offset >= 0) {
        jacobian.template block<3, 3>(row + 15, current_offset + 6) =
          -consistency_scale / next_dt_s * Eigen::Matrix3d::Identity();
        jacobian.template block<3, 3>(row + 15, current_offset + 3) =
          -0.5 * consistency_scale * Eigen::Matrix3d::Identity();
      }
      if (next_offset >= 0) {
        jacobian.template block<3, 3>(row + 15, next_offset + 6) =
          consistency_scale / next_dt_s * Eigen::Matrix3d::Identity();
        jacobian.template block<3, 3>(row + 15, next_offset + 3) =
          -0.5 * consistency_scale * Eigen::Matrix3d::Identity();
      }
    }
    row += static_cast<Eigen::Index>(kSmoothnessResidualDof);
  }

  if (row != jacobian.rows()) {
    numeric_blocks.clear();
    mark_numeric(0, jacobian.rows(), 0, jacobian.cols());
  }
  return numeric_blocks;
}

bool SlidingWindowOptimizer::add_schur_marginalization_prior_for_front()
{
  if (states_.size() < 2U) {
    return false;
  }
  const auto variables = variable_layout();
  auto marginalized_it = std::find_if(
    variables.begin(), variables.end(),
    [](const VariableBlock & variable) {
      return variable.state_index == 0U;
    });
  if (marginalized_it == variables.end()) {
    return false;
  }

  std::vector<VariableBlock> retained_variables;
  retained_variables.reserve(variables.size() - 1U);
  for (const auto & variable : variables) {
    if (variable.state_index != 0U) {
      retained_variables.push_back(variable);
    }
  }
  if (retained_variables.empty()) {
    return false;
  }

  const auto normal = linearize(states_, variables, 0.0);
  if (!normal.valid || normal.variable_count != variables.size() * kStateDof) {
    return false;
  }
  const auto marginalized_offset = static_cast<Eigen::Index>(marginalized_it->offset);
  constexpr Eigen::Index marginalized_dim = static_cast<Eigen::Index>(kStateDof);
  const auto retained_dim = static_cast<Eigen::Index>(retained_variables.size() * kStateDof);
  Eigen::MatrixXd h_mr(marginalized_dim, retained_dim);
  Eigen::MatrixXd h_rr(retained_dim, retained_dim);
  Eigen::VectorXd rhs_r(retained_dim);

  std::vector<Eigen::Index> retained_columns;
  retained_columns.reserve(static_cast<size_t>(retained_dim));
  for (const auto & variable : retained_variables) {
    for (size_t dim = 0; dim < kStateDof; ++dim) {
      retained_columns.push_back(static_cast<Eigen::Index>(variable.offset + dim));
    }
  }
  for (Eigen::Index row = 0; row < marginalized_dim; ++row) {
    for (Eigen::Index col = 0; col < retained_dim; ++col) {
      h_mr(row, col) = normal.hessian(marginalized_offset + row, retained_columns[static_cast<size_t>(col)]);
    }
  }
  for (Eigen::Index row = 0; row < retained_dim; ++row) {
    rhs_r(row) = normal.rhs(retained_columns[static_cast<size_t>(row)]);
    for (Eigen::Index col = 0; col < retained_dim; ++col) {
      h_rr(row, col) = normal.hessian(
        retained_columns[static_cast<size_t>(row)],
        retained_columns[static_cast<size_t>(col)]);
    }
  }

  const auto schur = compute_schur_complement(
    normal.hessian.block(marginalized_offset, marginalized_offset, marginalized_dim, marginalized_dim),
    h_mr,
    h_rr,
    normal.rhs.segment(marginalized_offset, marginalized_dim),
    rhs_r,
    config_.damping);
  if (!schur.valid || schur.hessian.rows() != retained_dim || schur.rhs.size() != retained_dim) {
    return false;
  }
  if (schur.hessian.norm() <= 1.0e-12) {
    return false;
  }

  const Eigen::MatrixXd regularized =
    schur.hessian + config_.damping * Eigen::MatrixXd::Identity(retained_dim, retained_dim);
  const Eigen::VectorXd target_delta = regularized.ldlt().solve(schur.rhs);
  const Eigen::MatrixXd sqrt_information = sqrt_information_from_hessian(schur.hessian);
  if (!target_delta.allFinite() || sqrt_information.rows() == 0 || !sqrt_information.allFinite()) {
    return false;
  }

  SlidingWindowDensePrior prior;
  prior.sqrt_information = sqrt_information;
  prior.target_delta = target_delta;
  prior.stamp_ns.reserve(retained_variables.size());
  prior.reference_states.reserve(retained_variables.size());
  for (const auto & variable : retained_variables) {
    prior.stamp_ns.push_back(states_[variable.state_index].stamp_ns);
    prior.reference_states.push_back(states_[variable.state_index]);
  }
  dense_priors_.push_back(std::move(prior));
  return true;
}

size_t SlidingWindowOptimizer::count_orphan_factors() const
{
  auto missing_state = [this](const int64_t stamp_ns) {
      return find_state_index(stamp_ns) < 0;
    };
  size_t count = 0U;
  for (const auto & factor : imu_factors_) {
    if (missing_state(factor.from_stamp_ns) || missing_state(factor.to_stamp_ns)) {
      ++count;
    }
  }
  for (const auto & prior : pose_priors_) {
    if (missing_state(prior.stamp_ns)) {
      ++count;
    }
  }
  for (const auto & prior : state_priors_) {
    if (missing_state(prior.stamp_ns)) {
      ++count;
    }
  }
  for (const auto & prior : dense_priors_) {
    if (std::any_of(prior.stamp_ns.begin(), prior.stamp_ns.end(), missing_state)) {
      ++count;
    }
  }
  for (const auto & factor : point_factors_) {
    if (missing_state(factor.stamp_ns)) {
      ++count;
    }
  }
  for (const auto & factor : plane_factors_) {
    if (missing_state(factor.stamp_ns)) {
      ++count;
    }
  }
  for (const auto & factor : visual_factors_) {
    if (visual_factor_has_stamp_matching(factor, missing_state)) {
      ++count;
    }
  }
  for (const auto & factor : se3_photometric_factors_) {
    if (visual_factor_has_stamp_matching(factor, missing_state)) {
      ++count;
    }
  }
  for (const auto & factor : relative_translation_factors_) {
    if (missing_state(factor.from_stamp_ns) || missing_state(factor.to_stamp_ns)) {
      ++count;
    }
  }
  for (const auto & factor : relative_distance_factors_) {
    if (missing_state(factor.from_stamp_ns) || missing_state(factor.to_stamp_ns)) {
      ++count;
    }
  }
  for (const auto & factor : smoothness_factors_) {
    if (missing_state(factor.previous_stamp_ns) || missing_state(factor.current_stamp_ns) ||
      missing_state(factor.next_stamp_ns))
    {
      ++count;
    }
  }
  return count;
}

SlidingWindowNormalEquation SlidingWindowOptimizer::linearize(
  const std::vector<SlidingWindowState> & states,
  const std::vector<VariableBlock> & variables,
  const double damping) const
{
  SlidingWindowNormalEquation output;
  if (damping < 0.0) {
    return output;
  }
  output.residual = build_residual(states);
  output.cost = compute_cost(output.residual);
  output.variable_count = variables.size() * kStateDof;
  const auto variable_count = static_cast<Eigen::Index>(output.variable_count);
  output.jacobian = Eigen::MatrixXd::Zero(output.residual.size(), variable_count);
  output.hessian = Eigen::MatrixXd::Zero(variable_count, variable_count);
  output.rhs = Eigen::VectorXd::Zero(variable_count);
  if (output.variable_count == 0U) {
    output.valid = output.residual.allFinite();
    return output;
  }

  const auto numeric_blocks = fill_analytic_jacobian(states, variables, output.jacobian);
  output.numeric_jacobian_block_count = numeric_blocks.size();
  std::vector<bool> numeric_columns(output.variable_count, false);
  for (const auto & block : numeric_blocks) {
    const auto column_begin = static_cast<size_t>(std::max<Eigen::Index>(0, block.column_start));
    const auto column_end = static_cast<size_t>(
      std::min<Eigen::Index>(
        static_cast<Eigen::Index>(output.variable_count),
        block.column_start + block.column_size));
    for (size_t column = column_begin; column < column_end; ++column) {
      numeric_columns[column] = true;
    }
  }
  output.numeric_jacobian_column_count = static_cast<size_t>(
    std::count(numeric_columns.begin(), numeric_columns.end(), true));
  for (size_t column = 0; column < output.variable_count && !numeric_blocks.empty(); ++column) {
    const auto column_index = static_cast<Eigen::Index>(column);
    if (!numeric_columns[column]) {
      continue;
    }
    Eigen::VectorXd delta = Eigen::VectorXd::Zero(variable_count);
    delta[column_index] = config_.numeric_epsilon;
    auto plus_states = states;
    auto minus_states = states;
    apply_delta(plus_states, variables, delta, 1.0);
    apply_delta(minus_states, variables, delta, -1.0);
    const Eigen::VectorXd numeric_column =
      (build_residual(plus_states) - build_residual(minus_states)) /
      (2.0 * config_.numeric_epsilon);
    for (const auto & block : numeric_blocks) {
      if (column_index < block.column_start ||
        column_index >= block.column_start + block.column_size)
      {
        continue;
      }
      output.jacobian.block(block.row_start, column_index, block.row_size, 1) =
        numeric_column.segment(block.row_start, block.row_size);
    }
  }

  output.hessian = output.jacobian.transpose() * output.jacobian;
  if (damping > 0.0) {
    output.hessian += damping * Eigen::MatrixXd::Identity(variable_count, variable_count);
  }
  output.rhs = -output.jacobian.transpose() * output.residual;
  output.valid = output.residual.allFinite() && output.jacobian.allFinite() &&
    output.hessian.allFinite() && output.rhs.allFinite();
  return output;
}

SlidingWindowNormalEquation SlidingWindowOptimizer::build_normal_equation(
  const double damping) const
{
  return linearize(states_, variable_layout(), damping);
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
  summary.schur_marginalization_count = schur_marginalization_count_;
  summary.fallback_marginalization_prior_count = fallback_marginalization_prior_count_;
  summary.state_count = states_.size();
  summary.imu_factor_count = imu_factors_.size();
  summary.pose_prior_count = pose_priors_.size();
  summary.state_prior_count = state_priors_.size();
  summary.dense_prior_count = dense_priors_.size();
  summary.point_factor_count = point_factors_.size();
  summary.plane_factor_count = plane_factors_.size();
  summary.visual_factor_count = visual_factors_.size();
  summary.se3_photometric_factor_count = se3_photometric_factors_.size();
  summary.relative_translation_factor_count = relative_translation_factors_.size();
  summary.relative_distance_factor_count = relative_distance_factors_.size();
  summary.smoothness_factor_count = smoothness_factors_.size();
  summary.imu_factor_replacement_count = imu_factor_replacement_count_;
  summary.point_factor_replacement_count = point_factor_replacement_count_;
  summary.plane_factor_replacement_count = plane_factor_replacement_count_;
  summary.visual_factor_replacement_count = visual_factor_replacement_count_;
  summary.se3_photometric_factor_replacement_count = se3_photometric_factor_replacement_count_;
  summary.relative_translation_factor_replacement_count =
    relative_translation_factor_replacement_count_;
  summary.relative_distance_factor_replacement_count =
    relative_distance_factor_replacement_count_;
  summary.smoothness_factor_replacement_count = smoothness_factor_replacement_count_;
  summary.orphan_factor_count = count_orphan_factors();
  if (states_.size() >= 2U) {
    summary.min_state_dt_s = std::numeric_limits<double>::infinity();
    for (size_t index = 1U; index < states_.size(); ++index) {
      const double dt_s = static_cast<double>(
        states_[index].stamp_ns - states_[index - 1U].stamp_ns) / 1.0e9;
      summary.min_state_dt_s = std::min(summary.min_state_dt_s, dt_s);
      summary.max_state_dt_s = std::max(summary.max_state_dt_s, dt_s);
      if (dt_s <= 0.0 || (config_.max_state_gap_s > 0.0 && dt_s > config_.max_state_gap_s)) {
        summary.state_gap_degenerate = true;
      }
    }
    if (!std::isfinite(summary.min_state_dt_s)) {
      summary.min_state_dt_s = 0.0;
    }
  }
  Eigen::VectorXd residual = build_residual(states_);
  summary.initial_cost = compute_cost(residual);
  summary.final_cost = summary.initial_cost;

  const auto variables = variable_layout();
  const size_t variable_count = variables.size() * kStateDof;
  auto refresh_normal_equation_summary = [&summary](
      const SlidingWindowNormalEquation & normal) {
      summary.normal_equation_rows = 0U;
      summary.normal_equation_cols = 0U;
      summary.normal_equation_rank = 0U;
      summary.numeric_jacobian_block_count = 0U;
      summary.numeric_jacobian_column_count = 0U;
      summary.normal_equation_min_singular_value = 0.0;
      summary.normal_equation_max_singular_value = 0.0;
      summary.normal_equation_condition_number = 0.0;

      if (!normal.valid) {
        return;
      }
      summary.normal_equation_rows = static_cast<size_t>(normal.residual.size());
      summary.normal_equation_cols = static_cast<size_t>(normal.variable_count);
      summary.numeric_jacobian_block_count = normal.numeric_jacobian_block_count;
      summary.numeric_jacobian_column_count = normal.numeric_jacobian_column_count;
      if (normal.hessian.rows() == 0 || normal.hessian.cols() == 0) {
        return;
      }

      const Eigen::MatrixXd symmetric_hessian =
        0.5 * (normal.hessian + normal.hessian.transpose());
      const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(symmetric_hessian);
      if (solver.info() != Eigen::Success || solver.eigenvalues().size() == 0) {
        return;
      }
      const double max_singular = std::max(0.0, solver.eigenvalues().maxCoeff());
      const double threshold = static_cast<double>(normal.hessian.rows()) *
        std::numeric_limits<double>::epsilon() * std::max(max_singular, 1.0);
      double min_positive = std::numeric_limits<double>::infinity();
      for (Eigen::Index index = 0; index < solver.eigenvalues().size(); ++index) {
        const double singular = std::max(0.0, solver.eigenvalues()[index]);
        if (singular > threshold) {
          ++summary.normal_equation_rank;
          min_positive = std::min(min_positive, singular);
        }
      }
      summary.normal_equation_max_singular_value = max_singular;
      if (std::isfinite(min_positive)) {
        summary.normal_equation_min_singular_value = min_positive;
        summary.normal_equation_condition_number = max_singular / min_positive;
      }
    };
  auto refresh_dense_prior_summary = [&summary, this]() {
      summary.dense_prior_rows = 0U;
      summary.dense_prior_cols = 0U;
      summary.dense_prior_rank = 0U;
      summary.dense_prior_min_singular_value = 0.0;
      summary.dense_prior_max_singular_value = 0.0;
      double min_positive = std::numeric_limits<double>::infinity();
      for (const auto & prior : dense_priors_) {
        if (prior.sqrt_information.rows() == 0 || prior.sqrt_information.cols() == 0 ||
          !prior.sqrt_information.allFinite())
        {
          continue;
        }
        summary.dense_prior_rows += static_cast<size_t>(prior.sqrt_information.rows());
        summary.dense_prior_cols += static_cast<size_t>(prior.sqrt_information.cols());
        const Eigen::JacobiSVD<Eigen::MatrixXd> svd(prior.sqrt_information);
        if (svd.info() != Eigen::Success || svd.singularValues().size() == 0) {
          continue;
        }
        const double max_singular = std::max(0.0, svd.singularValues().maxCoeff());
        const double threshold = static_cast<double>(
          std::max(prior.sqrt_information.rows(), prior.sqrt_information.cols())) *
          std::numeric_limits<double>::epsilon() * std::max(max_singular, 1.0);
        summary.dense_prior_max_singular_value =
          std::max(summary.dense_prior_max_singular_value, max_singular);
        for (Eigen::Index i = 0; i < svd.singularValues().size(); ++i) {
          const double singular = svd.singularValues()[i];
          if (singular > threshold) {
            ++summary.dense_prior_rank;
            min_positive = std::min(min_positive, singular);
          }
        }
      }
      if (std::isfinite(min_positive)) {
        summary.dense_prior_min_singular_value = min_positive;
      }
    };
  auto refresh_bias_summary = [&summary, this, &variables](
      const SlidingWindowNormalEquation & normal) {
      summary.gyro_bias_norm = 0.0;
      summary.accel_bias_norm = 0.0;
      summary.gyro_bias_random_walk_sqrt_info_mean = 0.0;
      summary.accel_bias_random_walk_sqrt_info_mean = 0.0;
      summary.gyro_bias_random_walk_sqrt_info_max = 0.0;
      summary.accel_bias_random_walk_sqrt_info_max = 0.0;
      for (const auto & state : states_) {
        summary.gyro_bias_norm = std::max(summary.gyro_bias_norm, state.gyro_bias.norm());
        summary.accel_bias_norm = std::max(summary.accel_bias_norm, state.accel_bias.norm());
      }
      double gyro_bias_sqrt_info_sum = 0.0;
      double accel_bias_sqrt_info_sum = 0.0;
      size_t bias_sqrt_info_count = 0U;
      for (const auto & factor : imu_factors_) {
        const Eigen::Matrix<double, 6, 1> sqrt_information =
          effective_bias_sqrt_information(factor).cwiseAbs();
        const double gyro_sqrt_info = sqrt_information.template segment<3>(0).maxCoeff();
        const double accel_sqrt_info = sqrt_information.template segment<3>(3).maxCoeff();
        if (std::isfinite(gyro_sqrt_info) && std::isfinite(accel_sqrt_info)) {
          gyro_bias_sqrt_info_sum += gyro_sqrt_info;
          accel_bias_sqrt_info_sum += accel_sqrt_info;
          summary.gyro_bias_random_walk_sqrt_info_max =
            std::max(summary.gyro_bias_random_walk_sqrt_info_max, gyro_sqrt_info);
          summary.accel_bias_random_walk_sqrt_info_max =
            std::max(summary.accel_bias_random_walk_sqrt_info_max, accel_sqrt_info);
          ++bias_sqrt_info_count;
        }
      }
      if (bias_sqrt_info_count > 0U) {
        summary.gyro_bias_random_walk_sqrt_info_mean =
          gyro_bias_sqrt_info_sum / static_cast<double>(bias_sqrt_info_count);
        summary.accel_bias_random_walk_sqrt_info_mean =
          accel_bias_sqrt_info_sum / static_cast<double>(bias_sqrt_info_count);
      }
      summary.gyro_bias_observability = 0.0;
      summary.accel_bias_observability = 0.0;
      if (variables.empty()) {
        return;
      }
      if (!normal.valid || normal.hessian.rows() == 0) {
        return;
      }
      auto min_eigenvalue = [](const Eigen::Matrix3d & block) {
          const Eigen::Matrix3d symmetric = 0.5 * (block + block.transpose());
          const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(symmetric);
          if (solver.info() != Eigen::Success) {
            return 0.0;
          }
          return std::max(0.0, solver.eigenvalues().minCoeff());
        };
      for (const auto & variable : variables) {
        const auto offset = static_cast<Eigen::Index>(variable.offset);
        if (offset + static_cast<Eigen::Index>(kStateDof) > normal.hessian.rows()) {
          continue;
        }
        summary.gyro_bias_observability = std::max(
          summary.gyro_bias_observability,
          min_eigenvalue(normal.hessian.block<3, 3>(offset + 9, offset + 9)));
        summary.accel_bias_observability = std::max(
          summary.accel_bias_observability,
          min_eigenvalue(normal.hessian.block<3, 3>(offset + 12, offset + 12)));
      }
    };
  auto refresh_cost_summary = [&summary, this]() {
      SlidingWindowCostBreakdown breakdown;
      const Eigen::VectorXd final_residual = build_residual(states_, &breakdown);
      if (final_residual.allFinite()) {
        summary.final_cost = compute_cost(final_residual);
      }
      summary.imu_cost = breakdown.imu_cost;
      summary.pose_prior_cost = breakdown.pose_prior_cost;
      summary.state_prior_cost = breakdown.state_prior_cost;
      summary.dense_prior_cost = breakdown.dense_prior_cost;
      summary.point_factor_cost = breakdown.point_factor_cost;
      summary.plane_factor_cost = breakdown.plane_factor_cost;
      summary.visual_factor_cost = breakdown.visual_factor_cost;
      summary.se3_photometric_factor_cost = breakdown.se3_photometric_factor_cost;
      summary.relative_translation_factor_cost = breakdown.relative_translation_factor_cost;
      summary.relative_distance_factor_cost = breakdown.relative_distance_factor_cost;
      summary.smoothness_factor_cost = breakdown.smoothness_factor_cost;
    };
  refresh_dense_prior_summary();
  const auto initial_normal_equation = linearize(states_, variables, 0.0);
  refresh_normal_equation_summary(initial_normal_equation);
  auto normal_equation_is_degenerate = [&summary, this]() {
      if (summary.normal_equation_cols == 0U) {
        return false;
      }
      if (summary.normal_equation_rank == 0U) {
        return true;
      }
      if (config_.min_normal_equation_rank_ratio > 0.0) {
        const auto min_rank = static_cast<size_t>(
          std::ceil(
            static_cast<double>(summary.normal_equation_cols) *
            config_.min_normal_equation_rank_ratio));
        if (summary.normal_equation_rank < min_rank) {
          return true;
        }
      }
      return config_.max_normal_equation_condition > 0.0 &&
             summary.normal_equation_condition_number > config_.max_normal_equation_condition;
    };
  if (summary.orphan_factor_count > 0U) {
    summary.normal_equation_degenerate = true;
    ++summary.rejected_steps;
    refresh_bias_summary(initial_normal_equation);
    refresh_cost_summary();
    return summary;
  }
  if (summary.state_gap_degenerate) {
    summary.normal_equation_degenerate = true;
    ++summary.rejected_steps;
    refresh_bias_summary(initial_normal_equation);
    refresh_cost_summary();
    return summary;
  }
  if (residual.size() == 0 || variable_count == 0U) {
    summary.converged = true;
    refresh_bias_summary(initial_normal_equation);
    refresh_dense_prior_summary();
    refresh_normal_equation_summary(initial_normal_equation);
    refresh_cost_summary();
    return summary;
  }
  if (normal_equation_is_degenerate()) {
    summary.normal_equation_degenerate = true;
    ++summary.rejected_steps;
    refresh_bias_summary(initial_normal_equation);
    refresh_cost_summary();
    return summary;
  }

  double damping = config_.damping;
  summary.last_damping = damping;
  auto bounded_step_scale = [this, &variables](const Eigen::VectorXd & step) {
      double scale = 1.0;
      auto apply_limit = [&scale](const double norm, const double limit) {
          if (limit > 0.0 && norm > limit) {
            scale = std::min(scale, limit / norm);
          }
        };
      for (const auto & variable : variables) {
        const auto offset = static_cast<Eigen::Index>(variable.offset);
        if (offset + static_cast<Eigen::Index>(kStateDof) > step.size()) {
          continue;
        }
        apply_limit(step.template segment<3>(offset).norm(), config_.max_rotation_step_rad);
        apply_limit(step.template segment<3>(offset + 3).norm(), config_.max_velocity_step_mps);
        apply_limit(step.template segment<3>(offset + 6).norm(), config_.max_translation_step_m);
        apply_limit(step.template segment<3>(offset + 9).norm(), config_.max_bias_step);
        apply_limit(step.template segment<3>(offset + 12).norm(), config_.max_bias_step);
      }
      return scale;
    };
  for (size_t iteration = 0; iteration < config_.max_iterations; ++iteration) {
    const auto normal_equation = linearize(states_, variables, damping);
    if (!normal_equation.valid) {
      ++summary.linearization_failure_count;
      ++summary.rejected_steps;
      summary.normal_equation_degenerate = true;
      break;
    }
    const double current_cost = normal_equation.cost;
    const Eigen::LDLT<Eigen::MatrixXd> ldlt(normal_equation.hessian);
    if (ldlt.info() != Eigen::Success) {
      ++summary.linear_solve_failure_count;
      ++summary.rejected_steps;
      summary.normal_equation_degenerate = true;
      summary.final_cost = current_cost;
      break;
    }
    const Eigen::VectorXd step = ldlt.solve(normal_equation.rhs);
    if (!step.allFinite()) {
      ++summary.linear_solve_failure_count;
      ++summary.rejected_steps;
      summary.normal_equation_degenerate = true;
      summary.final_cost = current_cost;
      break;
    }

    auto candidate_states = states_;
    const double step_scale = bounded_step_scale(step);
    summary.last_step_norm = step.norm();
    summary.last_step_scale = step_scale;
    summary.last_damping = damping;
    if (step_scale < 1.0) {
      ++summary.limited_steps;
    }
    apply_delta(candidate_states, variables, step, step_scale);
    if (!states_are_finite(candidate_states)) {
      ++summary.invalid_candidate_steps;
      ++summary.rejected_steps;
      summary.final_cost = current_cost;
      damping *= 10.0;
      continue;
    }
    const double candidate_cost = compute_cost(build_residual(candidate_states));
    if (!std::isfinite(candidate_cost)) {
      ++summary.invalid_candidate_steps;
      ++summary.rejected_steps;
      summary.final_cost = current_cost;
      damping *= 10.0;
      continue;
    }
    if (candidate_cost <= current_cost) {
      states_ = std::move(candidate_states);
      summary.iterations = iteration + 1U;
      ++summary.accepted_steps;
      summary.final_cost = candidate_cost;
      damping = std::max(damping * 0.1, config_.damping);
      if (step_scale * step.norm() < config_.step_tolerance) {
        summary.converged = true;
        break;
      }
    } else {
      ++summary.rejected_steps;
      damping *= 10.0;
      summary.final_cost = current_cost;
    }
  }
  if (summary.iterations == config_.max_iterations || summary.final_cost < summary.initial_cost) {
    summary.converged = summary.final_cost <= summary.initial_cost;
  }
  const auto final_normal_equation = linearize(states_, variables, 0.0);
  refresh_bias_summary(final_normal_equation);
  refresh_dense_prior_summary();
  refresh_normal_equation_summary(final_normal_equation);
  summary.normal_equation_degenerate =
    summary.normal_equation_degenerate || normal_equation_is_degenerate();
  refresh_cost_summary();
  return summary;
}

}  // namespace gaussian_lic_tracking
