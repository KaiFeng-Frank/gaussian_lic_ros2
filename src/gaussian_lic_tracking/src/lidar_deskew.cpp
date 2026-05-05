// SPDX-License-Identifier: GPL-3.0-or-later

#include <gaussian_lic_tracking/lidar_deskew.hpp>
#include <gaussian_lic_tracking/time.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace gaussian_lic_tracking
{
namespace
{
bool pose_is_finite(const TrajectoryPose & pose)
{
  return pose.p_w_i.allFinite() && pose.q_w_i.coeffs().allFinite() &&
         pose.q_w_i.norm() > std::numeric_limits<double>::epsilon();
}
}  // namespace

LidarDeskewResult deskew_lidar_points(
  const std::vector<TimedLidarPoint> & points,
  const TrajectoryPose & reference_pose,
  const PoseLookup & pose_lookup)
{
  LidarDeskewResult result;
  result.points_i.reserve(points.size());
  if (!pose_is_finite(reference_pose)) {
    for (const auto & point : points) {
      result.points_i.push_back(point.point_i);
    }
    return result;
  }
  const Eigen::Quaterniond q_ref_inv = reference_pose.q_w_i.normalized().inverse();

  for (const auto & point : points) {
    Eigen::Vector3d corrected = point.point_i;
    if (point.has_stamp && pose_lookup) {
      TrajectoryPose point_pose;
      if (pose_lookup(point.stamp_ns, point_pose) && pose_is_finite(point_pose)) {
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
