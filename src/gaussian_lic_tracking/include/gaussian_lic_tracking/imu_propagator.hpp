// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <cstdint>
#include <deque>
#include <optional>
#include <vector>

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

struct ImuMeasurement
{
  int64_t stamp_ns{0};
  Eigen::Vector3d angular_velocity_rad_s{Eigen::Vector3d::Zero()};
  Eigen::Vector3d linear_acceleration_m_s2{Eigen::Vector3d::Zero()};
};

class ImuPropagator
{
public:
  void reset(const ImuState & state);
  void reset_with_measurement(
    const ImuState & state,
    const Eigen::Vector3d & angular_velocity_rad_s,
    const Eigen::Vector3d & linear_acceleration_m_s2);
  const ImuState & state() const { return state_; }
  bool initialized() const { return initialized_; }
  size_t history_size() const { return history_.size(); }
  size_t measurement_history_size() const { return measurement_history_.size(); }
  std::optional<int64_t> oldest_history_stamp_ns() const;
  std::optional<int64_t> newest_history_stamp_ns() const;

  void set_gravity_w(const Eigen::Vector3d & gravity_w);
  const Eigen::Vector3d & gravity_w() const { return gravity_w_; }
  void set_max_history_size(size_t max_history_size);
  bool query_state(int64_t stamp_ns, ImuState & state) const;
  bool rebase_from_state(const ImuState & corrected_state);
  std::vector<ImuMeasurement> measurements_after(int64_t stamp_ns) const;

  void add_measurement(
    int64_t stamp_ns,
    const Eigen::Vector3d & angular_velocity_rad_s,
    const Eigen::Vector3d & linear_acceleration_m_s2);

private:
  void push_history(const ImuState & state);
  void remember_measurement(const ImuMeasurement & measurement);
  static bool state_is_valid(const ImuState & state);
  static bool measurement_is_valid(const ImuMeasurement & measurement);

  ImuState state_;
  std::deque<ImuState> history_;
  std::deque<ImuMeasurement> measurement_history_;
  Eigen::Vector3d gravity_w_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d last_angular_velocity_rad_s_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d last_linear_acceleration_m_s2_{Eigen::Vector3d::Zero()};
  size_t max_history_size_{4000};
  size_t max_measurement_history_size_{4000};
  bool initialized_{false};
  bool has_last_measurement_{false};
};

}  // namespace gaussian_lic_tracking
