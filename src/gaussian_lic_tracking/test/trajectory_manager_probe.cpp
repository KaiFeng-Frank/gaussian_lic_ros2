// SPDX-License-Identifier: GPL-3.0-or-later

#include <gaussian_lic_tracking/time.hpp>
#include <gaussian_lic_tracking/trajectory_manager.hpp>

#include <cmath>
#include <iostream>

int main()
{
  constexpr int64_t dt_ns = 50000000LL;
  gaussian_lic_tracking::TrajectoryManager trajectory(dt_ns);
  const Eigen::Vector3d velocity(2.0, -0.5, 0.25);
  constexpr double kYawRateRadS = 0.4;

  for (int i = 0; i < 8; ++i) {
    const int64_t stamp_ns = static_cast<int64_t>(i) * dt_ns;
    const double t_s = static_cast<double>(stamp_ns) / 1.0e9;
    gaussian_lic_tracking::TrajectoryPose pose;
    pose.stamp_ns = stamp_ns;
    pose.p_w_i = velocity * t_s;
    pose.q_w_i = Eigen::Quaterniond(Eigen::AngleAxisd(kYawRateRadS * t_s, Eigen::Vector3d::UnitZ()));
    trajectory.add_control_pose(pose);
  }

  double max_position_error = 0.0;
  double max_velocity_error = 0.0;
  double max_yaw_error = 0.0;
  for (int i = 2; i <= 8; ++i) {
    const int64_t stamp_ns = static_cast<int64_t>(i) * dt_ns / 2;
    gaussian_lic_tracking::TrajectoryPose query;
    if (!trajectory.query_pose(stamp_ns, query)) {
      std::cerr << "failed to query trajectory at " << stamp_ns << " ns\n";
      return 1;
    }
    const Eigen::Vector3d expected = velocity * (static_cast<double>(stamp_ns) / 1.0e9);
    max_position_error = std::max(max_position_error, (query.p_w_i - expected).norm());
    max_velocity_error = std::max(max_velocity_error, (query.v_w_i - velocity).norm());
    const Eigen::Quaterniond expected_q(
      Eigen::AngleAxisd(
        kYawRateRadS * static_cast<double>(stamp_ns) / 1.0e9,
        Eigen::Vector3d::UnitZ()));
    Eigen::Quaterniond yaw_error = (expected_q.inverse() * query.q_w_i).normalized();
    if (yaw_error.w() < 0.0) {
      yaw_error.coeffs() *= -1.0;
    }
    max_yaw_error = std::max(max_yaw_error, 2.0 * yaw_error.vec().norm());
  }

  gaussian_lic_tracking::TrajectoryPose before_update;
  if (!trajectory.query_pose(3 * dt_ns, before_update)) {
    std::cerr << "failed to query trajectory before control-pose update\n";
    return 1;
  }
  gaussian_lic_tracking::TrajectoryPose updated_control;
  updated_control.stamp_ns = 3 * dt_ns;
  updated_control.p_w_i = velocity * (static_cast<double>(updated_control.stamp_ns) / 1.0e9) +
    Eigen::Vector3d{0.0, 0.0, 1.0};
  updated_control.q_w_i = Eigen::Quaterniond(
    Eigen::AngleAxisd(
      kYawRateRadS * static_cast<double>(updated_control.stamp_ns) / 1.0e9,
      Eigen::Vector3d::UnitZ()));
  const size_t size_before_update = trajectory.size();
  trajectory.add_or_update_control_pose(updated_control);
  gaussian_lic_tracking::TrajectoryPose after_update;
  if (!trajectory.query_pose(3 * dt_ns, after_update)) {
    std::cerr << "failed to query trajectory after control-pose update\n";
    return 1;
  }
  const double update_delta_z = after_update.p_w_i.z() - before_update.p_w_i.z();
  if (trajectory.size() != size_before_update || update_delta_z < 0.5) {
    std::cerr << "trajectory control-pose update did not replace the existing knot\n";
    return 1;
  }

  builtin_interfaces::msg::Time negative_stamp;
  negative_stamp.sec = -1;
  negative_stamp.nanosec = 500000000U;
  const int64_t negative_ns = gaussian_lic_tracking::stamp_to_nanoseconds(negative_stamp);
  const auto roundtrip = gaussian_lic_tracking::nanoseconds_to_stamp(negative_ns);
  if (negative_ns != -500000000LL || roundtrip.sec != -1 || roundtrip.nanosec != 500000000U) {
    std::cerr << "signed nanosecond ROS2 time roundtrip failed\n";
    return 1;
  }

  std::cout << "trajectory_manager_probe controls=" << trajectory.size()
            << " dt_ns=" << trajectory.control_interval_ns()
            << " max_position_error=" << max_position_error
            << " max_velocity_error=" << max_velocity_error
            << " max_yaw_error=" << max_yaw_error
            << " update_delta_z=" << update_delta_z
            << " negative_ns=" << negative_ns << "\n";
  if (max_position_error > 1.0e-9 || max_velocity_error > 1.0e-9 ||
    max_yaw_error > 1.0e-9)
  {
    std::cerr << "constant-velocity cubic B-spline query error is too large\n";
    return 1;
  }
  std::cout << "trajectory_manager_probe OK\n";
  return 0;
}
