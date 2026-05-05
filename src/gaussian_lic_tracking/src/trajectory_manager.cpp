// SPDX-License-Identifier: GPL-3.0-or-later

#include <gaussian_lic_tracking/trajectory_manager.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace gaussian_lic_tracking
{

TrajectoryManager::TrajectoryManager(const int64_t control_interval_ns)
{
  set_control_interval_ns(control_interval_ns);
}

void TrajectoryManager::set_control_interval_ns(const int64_t control_interval_ns)
{
  if (control_interval_ns <= 0) {
    throw std::runtime_error("control interval must be positive nanoseconds");
  }
  control_interval_ns_ = control_interval_ns;
}

void TrajectoryManager::clear()
{
  control_poses_.clear();
}

void TrajectoryManager::add_control_pose(const TrajectoryPose & pose)
{
  if (!control_poses_.empty() && pose.stamp_ns <= control_poses_.back().stamp_ns) {
    throw std::runtime_error("trajectory control poses must be inserted in strictly increasing timestamp order");
  }
  TrajectoryPose normalized = pose;
  normalized.q_w_i.normalize();
  control_poses_.push_back(normalized);
}

double TrajectoryManager::cubic_basis(const size_t basis_index, const double u)
{
  const double u2 = u * u;
  const double u3 = u2 * u;
  switch (basis_index) {
    case 0:
      return (1.0 - 3.0 * u + 3.0 * u2 - u3) / 6.0;
    case 1:
      return (4.0 - 6.0 * u2 + 3.0 * u3) / 6.0;
    case 2:
      return (1.0 + 3.0 * u + 3.0 * u2 - 3.0 * u3) / 6.0;
    case 3:
      return u3 / 6.0;
    default:
      throw std::runtime_error("invalid cubic B-spline basis index");
  }
}

double TrajectoryManager::cubic_basis_derivative(const size_t basis_index, const double u)
{
  const double u2 = u * u;
  switch (basis_index) {
    case 0:
      return (-3.0 + 6.0 * u - 3.0 * u2) / 6.0;
    case 1:
      return (-12.0 * u + 9.0 * u2) / 6.0;
    case 2:
      return (3.0 + 6.0 * u - 9.0 * u2) / 6.0;
    case 3:
      return 0.5 * u2;
    default:
      throw std::runtime_error("invalid cubic B-spline basis index");
  }
}

Eigen::Vector3d quaternion_log(Eigen::Quaterniond quaternion)
{
  quaternion.normalize();
  if (quaternion.w() < 0.0) {
    quaternion.coeffs() *= -1.0;
  }
  const Eigen::Vector3d vector = quaternion.vec();
  const double vector_norm = vector.norm();
  if (vector_norm < 1.0e-12) {
    return Eigen::Vector3d::Zero();
  }
  return std::atan2(vector_norm, quaternion.w()) * vector / vector_norm;
}

Eigen::Quaterniond quaternion_exp(const Eigen::Vector3d & tangent)
{
  const double theta = tangent.norm();
  if (theta < 1.0e-12) {
    return Eigen::Quaterniond::Identity();
  }
  return Eigen::Quaterniond(
    std::cos(theta),
    std::sin(theta) * tangent.x() / theta,
    std::sin(theta) * tangent.y() / theta,
    std::sin(theta) * tangent.z() / theta).normalized();
}

Eigen::Quaterniond shortest_relative(
  const Eigen::Quaterniond & from,
  const Eigen::Quaterniond & to)
{
  Eigen::Quaterniond relative = (from.normalized().inverse() * to.normalized()).normalized();
  if (relative.w() < 0.0) {
    relative.coeffs() *= -1.0;
  }
  return relative;
}

Eigen::Quaterniond squad_control(
  const Eigen::Quaterniond & previous,
  const Eigen::Quaterniond & current,
  const Eigen::Quaterniond & next)
{
  const Eigen::Vector3d log_next = quaternion_log(shortest_relative(current, next));
  const Eigen::Vector3d log_previous = quaternion_log(shortest_relative(current, previous));
  return (current.normalized() * quaternion_exp(-0.25 * (log_next + log_previous))).normalized();
}

Eigen::Quaterniond squad(
  const Eigen::Quaterniond & q0,
  const Eigen::Quaterniond & q1,
  const Eigen::Quaterniond & s0,
  const Eigen::Quaterniond & s1,
  const double u)
{
  const Eigen::Quaterniond geodesic = q0.normalized().slerp(u, q1.normalized());
  const Eigen::Quaterniond control = s0.normalized().slerp(u, s1.normalized());
  return geodesic.slerp(2.0 * u * (1.0 - u), control).normalized();
}

bool TrajectoryManager::find_segment(int64_t stamp_ns, size_t & segment_index, double & u) const
{
  if (control_poses_.size() < 4) {
    return false;
  }
  const int64_t first_valid = control_poses_[1].stamp_ns;
  const int64_t last_valid = control_poses_[control_poses_.size() - 3U].stamp_ns;
  if (stamp_ns < first_valid || stamp_ns > last_valid) {
    return false;
  }

  const int64_t offset = stamp_ns - control_poses_.front().stamp_ns;
  const int64_t raw_index = offset / control_interval_ns_;
  if (raw_index < 1 || raw_index + 2 >= static_cast<int64_t>(control_poses_.size())) {
    return false;
  }
  segment_index = static_cast<size_t>(raw_index);
  const int64_t segment_start = control_poses_[segment_index].stamp_ns;
  u = static_cast<double>(stamp_ns - segment_start) / static_cast<double>(control_interval_ns_);
  u = std::clamp(u, 0.0, 1.0);
  return true;
}

bool TrajectoryManager::query_pose(const int64_t stamp_ns, TrajectoryPose & pose) const
{
  size_t segment_index = 0;
  double u = 0.0;
  if (!find_segment(stamp_ns, segment_index, u)) {
    return false;
  }

  pose.stamp_ns = stamp_ns;
  pose.p_w_i.setZero();
  pose.v_w_i.setZero();
  const double inv_dt_s = 1.0e9 / static_cast<double>(control_interval_ns_);
  for (size_t basis = 0; basis < 4; ++basis) {
    const auto & control_pose = control_poses_[segment_index + basis - 1U];
    pose.p_w_i += cubic_basis(basis, u) * control_pose.p_w_i;
    pose.v_w_i += cubic_basis_derivative(basis, u) * control_pose.p_w_i * inv_dt_s;
  }
  const auto & previous = control_poses_[segment_index - 1U].q_w_i;
  const auto & start = control_poses_[segment_index].q_w_i;
  const auto & end = control_poses_[segment_index + 1U].q_w_i;
  const auto & next = control_poses_[segment_index + 2U].q_w_i;
  pose.q_w_i = squad(
    start,
    end,
    squad_control(previous, start, end),
    squad_control(start, end, next),
    u);
  pose.q_w_i.normalize();
  return true;
}

}  // namespace gaussian_lic_tracking
