// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace gaussian_lic_tracking
{

struct TrajectoryPose
{
  int64_t stamp_ns{0};
  Eigen::Vector3d p_w_i{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond q_w_i{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d v_w_i{Eigen::Vector3d::Zero()};
};

class TrajectoryManager
{
public:
  explicit TrajectoryManager(int64_t control_interval_ns = 50000000LL);

  void set_control_interval_ns(int64_t control_interval_ns);
  int64_t control_interval_ns() const { return control_interval_ns_; }

  void clear();
  void add_control_pose(const TrajectoryPose & pose);
  void add_or_update_control_pose(const TrajectoryPose & pose);
  bool query_pose(int64_t stamp_ns, TrajectoryPose & pose) const;
  size_t size() const { return control_poses_.size(); }

private:
  static double cubic_basis(size_t basis_index, double u);
  static double cubic_basis_derivative(size_t basis_index, double u);
  bool find_segment(int64_t stamp_ns, size_t & segment_index, double & u) const;

  int64_t control_interval_ns_{50000000LL};
  std::vector<TrajectoryPose> control_poses_;
};

}  // namespace gaussian_lic_tracking
