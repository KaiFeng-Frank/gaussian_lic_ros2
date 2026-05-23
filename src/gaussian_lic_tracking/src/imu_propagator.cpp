// SPDX-License-Identifier: GPL-3.0-or-later

#include <gaussian_lic_tracking/imu_propagator.hpp>
#include <gaussian_lic_tracking/time.hpp>

#include <cmath>
#include <algorithm>
#include <limits>
#include <optional>
#include <stdexcept>

namespace gaussian_lic_tracking
{

void ImuPropagator::reset(const ImuState & state)
{
  if (!state_is_valid(state)) {
    throw std::runtime_error("IMU reset state must be finite with a non-zero quaternion");
  }
  state_ = state;
  state_.q_w_i.normalize();
  history_.clear();
  measurement_history_.clear();
  push_history(state_);
  initialized_ = true;
  has_last_measurement_ = false;
}

void ImuPropagator::reset_with_measurement(
  const ImuState & state,
  const Eigen::Vector3d & angular_velocity_rad_s,
  const Eigen::Vector3d & linear_acceleration_m_s2)
{
  const ImuMeasurement measurement{state.stamp_ns, angular_velocity_rad_s, linear_acceleration_m_s2};
  if (!measurement_is_valid(measurement)) {
    throw std::runtime_error("IMU seed measurements must be finite");
  }
  reset(state);
  last_angular_velocity_rad_s_ = angular_velocity_rad_s;
  last_linear_acceleration_m_s2_ = linear_acceleration_m_s2;
  has_last_measurement_ = true;
  remember_measurement(measurement);
}

void ImuPropagator::set_gravity_w(const Eigen::Vector3d & gravity_w)
{
  if (!std::isfinite(gravity_w.x()) || !std::isfinite(gravity_w.y()) || !std::isfinite(gravity_w.z())) {
    throw std::runtime_error("IMU gravity vector must be finite");
  }
  gravity_w_ = gravity_w;
}

void ImuPropagator::set_max_history_size(const size_t max_history_size)
{
  if (max_history_size < 2U) {
    throw std::runtime_error("IMU history size must be at least 2");
  }
  max_history_size_ = max_history_size;
  max_measurement_history_size_ = max_history_size;
  while (history_.size() > max_history_size_) {
    history_.pop_front();
  }
  while (measurement_history_.size() > max_measurement_history_size_) {
    measurement_history_.pop_front();
  }
}

bool ImuPropagator::query_state(const int64_t stamp_ns, ImuState & state) const
{
  if (history_.empty() || stamp_ns < history_.front().stamp_ns || stamp_ns > history_.back().stamp_ns) {
    return false;
  }
  const auto upper = std::lower_bound(
    history_.begin(), history_.end(), stamp_ns,
    [](const ImuState & candidate, const int64_t query_stamp) {
      return candidate.stamp_ns < query_stamp;
    });
  if (upper == history_.end()) {
    state = history_.back();
    return state.stamp_ns == stamp_ns;
  }
  if (upper->stamp_ns == stamp_ns || upper == history_.begin()) {
    state = *upper;
    return true;
  }

  const auto lower = std::prev(upper);
  const double alpha = static_cast<double>(stamp_ns - lower->stamp_ns) /
    static_cast<double>(upper->stamp_ns - lower->stamp_ns);
  state.stamp_ns = stamp_ns;
  state.p_w_i = (1.0 - alpha) * lower->p_w_i + alpha * upper->p_w_i;
  state.v_w_i = (1.0 - alpha) * lower->v_w_i + alpha * upper->v_w_i;
  state.gyro_bias = (1.0 - alpha) * lower->gyro_bias + alpha * upper->gyro_bias;
  state.accel_bias = (1.0 - alpha) * lower->accel_bias + alpha * upper->accel_bias;
  state.q_w_i = lower->q_w_i.slerp(alpha, upper->q_w_i).normalized();
  return true;
}

std::optional<int64_t> ImuPropagator::oldest_history_stamp_ns() const
{
  if (history_.empty()) {
    return std::nullopt;
  }
  return history_.front().stamp_ns;
}

std::optional<int64_t> ImuPropagator::newest_history_stamp_ns() const
{
  if (history_.empty()) {
    return std::nullopt;
  }
  return history_.back().stamp_ns;
}

bool ImuPropagator::rebase_from_state(const ImuState & corrected_state)
{
  if (!initialized_ || !state_is_valid(corrected_state)) {
    return false;
  }

  std::optional<ImuMeasurement> seed_measurement;
  std::vector<ImuMeasurement> replay_measurements;
  replay_measurements.reserve(measurement_history_.size());
  for (const auto & measurement : measurement_history_) {
    if (measurement.stamp_ns <= corrected_state.stamp_ns) {
      seed_measurement = measurement;
    } else {
      replay_measurements.push_back(measurement);
    }
  }
  if (!seed_measurement.has_value()) {
    return false;
  }

  ImuState rebased = corrected_state;
  rebased.q_w_i.normalize();
  state_ = rebased;
  history_.clear();
  measurement_history_.clear();
  push_history(state_);
  last_angular_velocity_rad_s_ = seed_measurement->angular_velocity_rad_s;
  last_linear_acceleration_m_s2_ = seed_measurement->linear_acceleration_m_s2;
  has_last_measurement_ = true;
  remember_measurement(seed_measurement.value());

  try {
    for (const auto & measurement : replay_measurements) {
      add_measurement(
        measurement.stamp_ns,
        measurement.angular_velocity_rad_s,
        measurement.linear_acceleration_m_s2);
    }
  } catch (const std::exception &) {
    reset(corrected_state);
    last_angular_velocity_rad_s_ = seed_measurement->angular_velocity_rad_s;
    last_linear_acceleration_m_s2_ = seed_measurement->linear_acceleration_m_s2;
    has_last_measurement_ = true;
    remember_measurement(seed_measurement.value());
    return false;
  }
  return true;
}

std::vector<ImuMeasurement> ImuPropagator::measurements_after(const int64_t stamp_ns) const
{
  std::vector<ImuMeasurement> output;
  for (const auto & measurement : measurement_history_) {
    if (measurement.stamp_ns > stamp_ns) {
      output.push_back(measurement);
    }
  }
  return output;
}

void ImuPropagator::push_history(const ImuState & state)
{
  history_.push_back(state);
  while (history_.size() > max_history_size_) {
    history_.pop_front();
  }
}

void ImuPropagator::remember_measurement(const ImuMeasurement & measurement)
{
  if (!measurement_is_valid(measurement)) {
    throw std::runtime_error("IMU measurements must be finite");
  }
  if (!measurement_history_.empty() && measurement.stamp_ns <= measurement_history_.back().stamp_ns) {
    if (measurement.stamp_ns == measurement_history_.back().stamp_ns) {
      measurement_history_.back() = measurement;
      return;
    }
    throw std::runtime_error("IMU measurement history must be strictly increasing");
  }
  measurement_history_.push_back(measurement);
  while (measurement_history_.size() > max_measurement_history_size_) {
    measurement_history_.pop_front();
  }
}

bool ImuPropagator::state_is_valid(const ImuState & state)
{
  return state.p_w_i.allFinite() && state.v_w_i.allFinite() &&
         state.gyro_bias.allFinite() && state.accel_bias.allFinite() &&
         state.q_w_i.coeffs().allFinite() &&
         state.q_w_i.norm() > std::numeric_limits<double>::epsilon();
}

bool ImuPropagator::measurement_is_valid(const ImuMeasurement & measurement)
{
  return measurement.angular_velocity_rad_s.allFinite() &&
         measurement.linear_acceleration_m_s2.allFinite();
}

void ImuPropagator::add_measurement(
  const int64_t stamp_ns,
  const Eigen::Vector3d & angular_velocity_rad_s,
  const Eigen::Vector3d & linear_acceleration_m_s2)
{
  const ImuMeasurement measurement{stamp_ns, angular_velocity_rad_s, linear_acceleration_m_s2};
  if (!measurement_is_valid(measurement)) {
    throw std::runtime_error("IMU measurements must be finite");
  }
  if (!initialized_) {
    ImuState initial;
    initial.stamp_ns = stamp_ns;
    reset(initial);
    last_angular_velocity_rad_s_ = angular_velocity_rad_s;
    last_linear_acceleration_m_s2_ = linear_acceleration_m_s2;
    has_last_measurement_ = true;
    remember_measurement(measurement);
    return;
  }
  if (stamp_ns <= state_.stamp_ns) {
    throw std::runtime_error("IMU measurements must arrive in strictly increasing signed-ns order");
  }

  const double dt = static_cast<double>(stamp_ns - state_.stamp_ns) /
    static_cast<double>(kNanosecondsPerSecond);
  const Eigen::Vector3d previous_omega = has_last_measurement_ ?
    last_angular_velocity_rad_s_ : angular_velocity_rad_s;
  const Eigen::Vector3d previous_accel = has_last_measurement_ ?
    last_linear_acceleration_m_s2_ : linear_acceleration_m_s2;
  const Eigen::Vector3d omega =
    0.5 * (previous_omega + angular_velocity_rad_s) - state_.gyro_bias;
  const double angle = omega.norm() * dt;
  Eigen::Quaterniond delta_q = Eigen::Quaterniond::Identity();
  if (angle > 1.0e-12) {
    delta_q = Eigen::AngleAxisd(angle, omega.normalized());
  }

  const Eigen::Quaterniond q_next = (state_.q_w_i * delta_q).normalized();
  const Eigen::Vector3d accel_body_previous = previous_accel - state_.accel_bias;
  const Eigen::Vector3d accel_body_current = linear_acceleration_m_s2 - state_.accel_bias;
  const Eigen::Vector3d accel_world =
    0.5 * (state_.q_w_i * accel_body_previous + q_next * accel_body_current) + gravity_w_;

  state_.p_w_i += state_.v_w_i * dt + 0.5 * accel_world * dt * dt;
  state_.v_w_i += accel_world * dt;
  state_.q_w_i = q_next;
  state_.stamp_ns = stamp_ns;
  last_angular_velocity_rad_s_ = angular_velocity_rad_s;
  last_linear_acceleration_m_s2_ = linear_acceleration_m_s2;
  has_last_measurement_ = true;
  remember_measurement(measurement);
  push_history(state_);
}

}  // namespace gaussian_lic_tracking
