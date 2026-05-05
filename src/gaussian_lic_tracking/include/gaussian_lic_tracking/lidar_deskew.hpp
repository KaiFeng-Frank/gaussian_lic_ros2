// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

#include <Eigen/Core>

#include <gaussian_lic_tracking/trajectory_manager.hpp>

namespace gaussian_lic_tracking
{

struct TimedLidarPoint
{
  Eigen::Vector3d point_i{Eigen::Vector3d::Zero()};
  int64_t stamp_ns{0};
  bool has_stamp{false};
};

struct LidarDeskewResult
{
  std::vector<Eigen::Vector3d> points_i;
  size_t deskewed_count{0};
  double max_abs_time_offset_s{0.0};
};

using PoseLookup = std::function<bool(int64_t, TrajectoryPose &)>;

LidarDeskewResult deskew_lidar_points(
  const std::vector<TimedLidarPoint> & points,
  const TrajectoryPose & reference_pose,
  const PoseLookup & pose_lookup);

}  // namespace gaussian_lic_tracking
