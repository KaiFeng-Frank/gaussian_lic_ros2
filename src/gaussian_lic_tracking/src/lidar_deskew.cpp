// SPDX-License-Identifier: GPL-3.0-or-later

#include <gaussian_lic_tracking/lidar_deskew.hpp>
#include <gaussian_lic_tracking/time.hpp>

#include <algorithm>
#include <cmath>

namespace gaussian_lic_tracking
{

LidarDeskewResult deskew_lidar_points(
  const std::vector<TimedLidarPoint> & points,
  const TrajectoryPose & reference_pose,
  const PoseLookup & pose_lookup)
{
  LidarDeskewResult result;
  result.points_i.reserve(points.size());
  const Eigen::Quaterniond q_ref_inv = reference_pose.q_w_i.normalized().inverse();

  for (const auto & point : points) {
    Eigen::Vector3d corrected = point.point_i;
    if (point.has_stamp && pose_lookup) {
      TrajectoryPose point_pose;
      if (pose_lookup(point.stamp_ns, point_pose)) {
        point_pose.q_w_i.normalize();
        const Eigen::Vector3d point_w = point_pose.q_w_i * point.point_i + point_pose.p_w_i;
        corrected = q_ref_inv * (point_w - reference_pose.p_w_i);
        ++result.deskewed_count;
        result.max_abs_time_offset_s = std::max(
          result.max_abs_time_offset_s,
          std::abs(static_cast<double>(point.stamp_ns - reference_pose.stamp_ns)) /
          static_cast<double>(kNanosecondsPerSecond));
      }
    }
    result.points_i.push_back(corrected);
  }
  return result;
}

}  // namespace gaussian_lic_tracking
