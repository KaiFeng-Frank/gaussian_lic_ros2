// SPDX-License-Identifier: GPL-3.0-or-later

#include <gaussian_lic_tracking/imu_preintegrator.hpp>
#include <gaussian_lic_tracking/time.hpp>

#include <cmath>
#include <limits>
#include <stdexcept>

namespace gaussian_lic_tracking
{

void ImuPreintegrator::reset(const int64_t start_stamp_ns, const ImuBias & bias)
{
  if (!bias.gyro.allFinite() || !bias.accel.allFinite()) {
    throw std::runtime_error("IMU preintegration bias must be finite");
  }
  bias_ = bias;
  start_stamp_ns_ = start_stamp_ns;
  end_stamp_ns_ = start_stamp_ns;
  delta_t_s_ = 0.0;
  delta_q_ = Eigen::Quaterniond::Identity();
  delta_v_.setZero();
  delta_p_.setZero();
  samples_.clear();
  has_last_measurement_ = false;
  initialized_ = true;
}

void ImuPreintegrator::add_measurement(
  const int64_t stamp_ns,
  const Eigen::Vector3d & angular_velocity_rad_s,
  const Eigen::Vector3d & linear_acceleration_m_s2)
{
  if (!angular_velocity_rad_s.allFinite() || !linear_acceleration_m_s2.allFinite()) {
    throw std::runtime_error("IMU preintegration measurements must be finite");
  }
  if (!initialized_) {
    reset(stamp_ns, bias_);
    last_angular_velocity_rad_s_ = angular_velocity_rad_s;
    last_linear_acceleration_m_s2_ = linear_acceleration_m_s2;
    samples_.push_back(ImuPreintegrationSample{
        stamp_ns, angular_velocity_rad_s, linear_acceleration_m_s2});
    has_last_measurement_ = true;
    return;
  }
  if (stamp_ns <= end_stamp_ns_) {
    throw std::runtime_error("IMU preintegration measurements must be strictly increasing");
  }

  const double dt = static_cast<double>(stamp_ns - end_stamp_ns_) /
    static_cast<double>(kNanosecondsPerSecond);
  const Eigen::Vector3d previous_omega = has_last_measurement_ ?
    last_angular_velocity_rad_s_ : angular_velocity_rad_s;
  const Eigen::Vector3d previous_accel = has_last_measurement_ ?
    last_linear_acceleration_m_s2_ : linear_acceleration_m_s2;
  const Eigen::Vector3d omega = 0.5 * (previous_omega + angular_velocity_rad_s) - bias_.gyro;
  const Eigen::Vector3d accel_previous = previous_accel - bias_.accel;
  const Eigen::Vector3d accel_current = linear_acceleration_m_s2 - bias_.accel;

  Eigen::Quaterniond step_q = Eigen::Quaterniond::Identity();
  const double angle = omega.norm() * dt;
  if (angle > 1.0e-12) {
    step_q = Eigen::Quaterniond(Eigen::AngleAxisd(angle, omega.normalized()));
  }
  const Eigen::Quaterniond next_delta_q = (delta_q_ * step_q).normalized();
  const Eigen::Vector3d accel_local =
    0.5 * (delta_q_ * accel_previous + next_delta_q * accel_current);

  delta_p_ += delta_v_ * dt + 0.5 * accel_local * dt * dt;
  delta_v_ += accel_local * dt;
  delta_q_ = next_delta_q;
  delta_t_s_ += dt;
  end_stamp_ns_ = stamp_ns;
  last_angular_velocity_rad_s_ = angular_velocity_rad_s;
  last_linear_acceleration_m_s2_ = linear_acceleration_m_s2;
  samples_.push_back(ImuPreintegrationSample{
      stamp_ns, angular_velocity_rad_s, linear_acceleration_m_s2});
  has_last_measurement_ = true;
}

Eigen::Vector3d ImuPreintegrator::rotation_residual(
  const Eigen::Quaterniond & measured_delta,
  const Eigen::Quaterniond & predicted_delta)
{
  Eigen::Quaterniond error = (measured_delta.normalized().inverse() * predicted_delta.normalized()).normalized();
  if (error.w() < 0.0) {
    error.coeffs() *= -1.0;
  }
  return 2.0 * error.vec();
}

ImuPreintegrationResidual ImuPreintegrator::residual(
  const ImuState & start_state,
  const ImuState & end_state,
  const Eigen::Vector3d & gravity_w) const
{
  if (!initialized_) {
    throw std::runtime_error("cannot compute IMU preintegration residual before reset");
  }
  if (!start_state.p_w_i.allFinite() || !start_state.v_w_i.allFinite() ||
    !start_state.q_w_i.coeffs().allFinite() ||
    !end_state.p_w_i.allFinite() || !end_state.v_w_i.allFinite() ||
    !end_state.q_w_i.coeffs().allFinite() || !gravity_w.allFinite())
  {
    throw std::runtime_error("IMU preintegration residual inputs must be finite");
  }
  if (start_state.q_w_i.norm() <= std::numeric_limits<double>::epsilon() ||
    end_state.q_w_i.norm() <= std::numeric_limits<double>::epsilon())
  {
    throw std::runtime_error("IMU preintegration residual quaternions must be non-zero");
  }
  const Eigen::Quaterniond q_start_inv = start_state.q_w_i.normalized().inverse();
  const Eigen::Quaterniond predicted_delta_q =
    q_start_inv * end_state.q_w_i.normalized();
  const Eigen::Vector3d predicted_delta_v =
    q_start_inv * (end_state.v_w_i - start_state.v_w_i - gravity_w * delta_t_s_);
  const Eigen::Vector3d predicted_delta_p =
    q_start_inv * (
    end_state.p_w_i - start_state.p_w_i - start_state.v_w_i * delta_t_s_ -
    0.5 * gravity_w * delta_t_s_ * delta_t_s_);

  ImuPreintegrationResidual output;
  output.residual.template segment<3>(0) = rotation_residual(delta_q_, predicted_delta_q);
  output.residual.template segment<3>(3) = predicted_delta_v - delta_v_;
  output.residual.template segment<3>(6) = predicted_delta_p - delta_p_;
  output.rotation_norm = output.residual.template segment<3>(0).norm();
  output.velocity_norm = output.residual.template segment<3>(3).norm();
  output.position_norm = output.residual.template segment<3>(6).norm();
  return output;
}

ImuPreintegrator ImuPreintegrator::reintegrated(const ImuBias & bias) const
{
  ImuPreintegrator output;
  output.reset(start_stamp_ns_, bias);
  for (const auto & sample : samples_) {
    if (sample.stamp_ns == output.end_stamp_ns_ && !output.has_last_measurement_) {
      if (!sample.angular_velocity_rad_s.allFinite() ||
        !sample.linear_acceleration_m_s2.allFinite())
      {
        throw std::runtime_error("IMU preintegration measurements must be finite");
      }
      output.last_angular_velocity_rad_s_ = sample.angular_velocity_rad_s;
      output.last_linear_acceleration_m_s2_ = sample.linear_acceleration_m_s2;
      output.samples_.push_back(sample);
      output.has_last_measurement_ = true;
      continue;
    }
    output.add_measurement(
      sample.stamp_ns,
      sample.angular_velocity_rad_s,
      sample.linear_acceleration_m_s2);
  }
  return output;
}

}  // namespace gaussian_lic_tracking
