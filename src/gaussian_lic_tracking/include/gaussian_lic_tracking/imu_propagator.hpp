// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <cstdint>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace gaussian_lic_tracking
{

struct ImuState
{
  int64_t stamp_ns{0};
  Eigen::Quaterniond q_w_i{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d p_w_i{Eigen::Vector3d::Zero()};
  Eigen::Vector3d v_w_i{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gyro_bias{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accel_bias{Eigen::Vector3d::Zero()};
};

class ImuPropagator
{
public:
  void reset(const ImuState & state);
  const ImuState & state() const { return state_; }
  bool initialized() const { return initialized_; }

  void add_measurement(
    int64_t stamp_ns,
    const Eigen::Vector3d & angular_velocity_rad_s,
    const Eigen::Vector3d & linear_acceleration_m_s2);

private:
  ImuState state_;
  bool initialized_{false};
};

}  // namespace gaussian_lic_tracking
