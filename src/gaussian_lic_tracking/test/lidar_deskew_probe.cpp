// SPDX-License-Identifier: GPL-3.0-or-later

#include <gaussian_lic_tracking/lidar_deskew.hpp>
#include <gaussian_lic_tracking/time.hpp>

#include <cmath>
#include <iostream>

int main()
{
  constexpr int64_t dt_ns = 10000000LL;
  constexpr int64_t reference_stamp_ns = 100000000LL;
  const Eigen::Vector3d velocity_w(1.0, 0.0, 0.0);
  const Eigen::Vector3d static_world_point(1.0, 0.2, -0.1);

  std::vector<gaussian_lic_tracking::TimedLidarPoint> points;
  for (int i = 0; i <= 10; ++i) {
    const int64_t stamp_ns = static_cast<int64_t>(i) * dt_ns;
    gaussian_lic_tracking::TimedLidarPoint point;
    point.stamp_ns = stamp_ns;
    point.has_stamp = true;
    const Eigen::Vector3d sensor_p_w = velocity_w *
      (static_cast<double>(stamp_ns) / static_cast<double>(gaussian_lic_tracking::kNanosecondsPerSecond));
    point.point_i = static_world_point - sensor_p_w;
    points.push_back(point);
  }

  gaussian_lic_tracking::TrajectoryPose reference_pose;
  reference_pose.stamp_ns = reference_stamp_ns;
  reference_pose.p_w_i = velocity_w * 0.1;
  reference_pose.q_w_i = Eigen::Quaterniond::Identity();
  const Eigen::Vector3d expected_reference_point = static_world_point - reference_pose.p_w_i;

  const auto result = gaussian_lic_tracking::deskew_lidar_points(
    points,
    reference_pose,
    [velocity_w](const int64_t stamp_ns, gaussian_lic_tracking::TrajectoryPose & pose) {
      pose.stamp_ns = stamp_ns;
      pose.p_w_i = velocity_w *
        (static_cast<double>(stamp_ns) / static_cast<double>(gaussian_lic_tracking::kNanosecondsPerSecond));
      pose.q_w_i = Eigen::Quaterniond::Identity();
      return true;
    });

  double max_error = 0.0;
  for (const auto & point : result.points_i) {
    max_error = std::max(max_error, (point - expected_reference_point).norm());
  }
  std::cout << "lidar_deskew_probe points=" << result.points_i.size()
            << " deskewed=" << result.deskewed_count
            << " max_error=" << max_error
            << " max_abs_time_offset_s=" << result.max_abs_time_offset_s << "\n";
  if (result.deskewed_count != points.size() || max_error > 1.0e-12) {
    std::cerr << "LiDAR deskew failed to compensate constant-velocity scan motion\n";
    return 1;
  }
  std::cout << "lidar_deskew_probe OK\n";
  return 0;
}
