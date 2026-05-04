// SPDX-License-Identifier: GPL-3.0-or-later

#include <gaussian_lic_tracking/imu_propagator.hpp>
#include <gaussian_lic_tracking/time.hpp>

#include <cmath>
#include <stdexcept>

namespace gaussian_lic_tracking
{

void ImuPropagator::reset(const ImuState & state)
{
  state_ = state;
  state_.q_w_i.normalize();
  initialized_ = true;
}

void ImuPropagator::add_measurement(
  const int64_t stamp_ns,
  const Eigen::Vector3d & angular_velocity_rad_s,
  const Eigen::Vector3d & linear_acceleration_m_s2)
{
  if (!initialized_) {
    ImuState initial;
    initial.stamp_ns = stamp_ns;
    reset(initial);
    return;
  }
  if (stamp_ns <= state_.stamp_ns) {
    throw std::runtime_error("IMU measurements must arrive in strictly increasing signed-ns order");
  }

  const double dt = static_cast<double>(stamp_ns - state_.stamp_ns) /
    static_cast<double>(kNanosecondsPerSecond);
  const Eigen::Vector3d omega = angular_velocity_rad_s - state_.gyro_bias;
  const double angle = omega.norm() * dt;
  Eigen::Quaterniond delta_q = Eigen::Quaterniond::Identity();
  if (angle > 1.0e-12) {
    delta_q = Eigen::AngleAxisd(angle, omega.normalized());
  }

  const Eigen::Quaterniond q_next = (state_.q_w_i * delta_q).normalized();
  const Eigen::Vector3d accel_body = linear_acceleration_m_s2 - state_.accel_bias;
  const Eigen::Vector3d accel_world = state_.q_w_i * accel_body;

  state_.p_w_i += state_.v_w_i * dt + 0.5 * accel_world * dt * dt;
  state_.v_w_i += accel_world * dt;
  state_.q_w_i = q_next;
  state_.stamp_ns = stamp_ns;
}

}  // namespace gaussian_lic_tracking
