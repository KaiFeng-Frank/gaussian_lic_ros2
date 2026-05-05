// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <cstdint>
#include <deque>

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
  size_t history_size() const { return history_.size(); }

  void set_gravity_w(const Eigen::Vector3d & gravity_w);
  const Eigen::Vector3d & gravity_w() const { return gravity_w_; }
  void set_max_history_size(size_t max_history_size);
  bool query_state(int64_t stamp_ns, ImuState & state) const;

  void add_measurement(
    int64_t stamp_ns,
    const Eigen::Vector3d & angular_velocity_rad_s,
    const Eigen::Vector3d & linear_acceleration_m_s2);

private:
  void push_history(const ImuState & state);

  ImuState state_;
  std::deque<ImuState> history_;
  Eigen::Vector3d gravity_w_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d last_angular_velocity_rad_s_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d last_linear_acceleration_m_s2_{Eigen::Vector3d::Zero()};
  size_t max_history_size_{4000};
  bool initialized_{false};
  bool has_last_measurement_{false};
};

}  // namespace gaussian_lic_tracking
