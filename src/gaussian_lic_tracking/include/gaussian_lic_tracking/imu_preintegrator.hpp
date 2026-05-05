// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <cstdint>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gaussian_lic_tracking/imu_propagator.hpp>

namespace gaussian_lic_tracking
{

struct ImuBias
{
  Eigen::Vector3d gyro{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accel{Eigen::Vector3d::Zero()};
};

struct ImuPreintegrationResidual
{
  Eigen::Matrix<double, 9, 1> residual{Eigen::Matrix<double, 9, 1>::Zero()};
  double rotation_norm{0.0};
  double velocity_norm{0.0};
  double position_norm{0.0};
};

class ImuPreintegrator
{
public:
  void reset(int64_t start_stamp_ns, const ImuBias & bias = {});
  bool initialized() const { return initialized_; }

  void add_measurement(
    int64_t stamp_ns,
    const Eigen::Vector3d & angular_velocity_rad_s,
    const Eigen::Vector3d & linear_acceleration_m_s2);

  ImuPreintegrationResidual residual(
    const ImuState & start_state,
    const ImuState & end_state,
    const Eigen::Vector3d & gravity_w = Eigen::Vector3d::Zero()) const;

  int64_t start_stamp_ns() const { return start_stamp_ns_; }
  int64_t end_stamp_ns() const { return end_stamp_ns_; }
  double delta_t_s() const { return delta_t_s_; }
  const Eigen::Quaterniond & delta_q() const { return delta_q_; }
  const Eigen::Vector3d & delta_v() const { return delta_v_; }
  const Eigen::Vector3d & delta_p() const { return delta_p_; }

private:
  static Eigen::Vector3d rotation_residual(
    const Eigen::Quaterniond & measured_delta,
    const Eigen::Quaterniond & predicted_delta);

  ImuBias bias_;
  int64_t start_stamp_ns_{0};
  int64_t end_stamp_ns_{0};
  double delta_t_s_{0.0};
  Eigen::Quaterniond delta_q_{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d delta_v_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d delta_p_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d last_angular_velocity_rad_s_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d last_linear_acceleration_m_s2_{Eigen::Vector3d::Zero()};
  bool initialized_{false};
  bool has_last_measurement_{false};
};

}  // namespace gaussian_lic_tracking
