// SPDX-License-Identifier: GPL-3.0-or-later

#include <gaussian_lic_tracking/gaussian_snapshot.hpp>
#include <gaussian_lic_tracking/time.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace gaussian_lic_tracking
{

void GaussianSnapshot::clear()
{
  stamp_ns_ = 0;
  expected_total_count_ = 0U;
  expected_chunk_count_ = 0U;
  received_chunk_count_ = 0U;
  received_chunks_.clear();
  points_.clear();
}

void GaussianSnapshot::reset_sequence(
  const int64_t stamp_ns,
  const uint32_t total_count,
  const uint32_t chunk_count)
{
  stamp_ns_ = stamp_ns;
  expected_total_count_ = total_count;
  expected_chunk_count_ = chunk_count;
  received_chunk_count_ = 0U;
  received_chunks_.assign(static_cast<size_t>(chunk_count), false);
  points_.clear();
  points_.reserve(total_count);
}

bool GaussianSnapshot::ingest(const gaussian_lic_msgs::msg::GaussianArray & msg)
{
  if (msg.chunk_count == 0U || msg.chunk_index >= msg.chunk_count) {
    return false;
  }
  const int64_t msg_stamp_ns = stamp_to_nanoseconds(msg.header.stamp);
  const bool same_sequence =
    !received_chunks_.empty() &&
    msg_stamp_ns == stamp_ns_ &&
    msg.total_count == expected_total_count_ &&
    msg.chunk_count == expected_chunk_count_;
  if (!same_sequence) {
    reset_sequence(msg_stamp_ns, msg.total_count, msg.chunk_count);
  }

  const size_t chunk_index = static_cast<size_t>(msg.chunk_index);
  if (received_chunks_[chunk_index]) {
    return false;
  }
  received_chunks_[chunk_index] = true;
  ++received_chunk_count_;

  for (const auto & gaussian : msg.gaussians) {
    GaussianSnapshotPoint point;
    point.id = gaussian.id;
    point.xyz = Eigen::Vector3d{
      static_cast<double>(gaussian.xyz[0]),
      static_cast<double>(gaussian.xyz[1]),
      static_cast<double>(gaussian.xyz[2])};
    point.scale = Eigen::Vector3d{
      static_cast<double>(gaussian.scale[0]),
      static_cast<double>(gaussian.scale[1]),
      static_cast<double>(gaussian.scale[2])};
    point.opacity = static_cast<double>(gaussian.opacity);
    point.confidence = static_cast<double>(gaussian.confidence);
    point.flags = gaussian.flags;
    points_.push_back(point);
  }
  if (expected_total_count_ > 0U && points_.size() > expected_total_count_) {
    points_.resize(expected_total_count_);
  }
  return true;
}

bool GaussianSnapshot::complete() const
{
  return expected_chunk_count_ > 0U &&
    received_chunk_count_ == static_cast<size_t>(expected_chunk_count_) &&
    (expected_total_count_ == 0U || points_.size() == static_cast<size_t>(expected_total_count_));
}

Eigen::Vector3d GaussianSnapshot::centroid() const
{
  if (points_.empty()) {
    return Eigen::Vector3d::Zero();
  }
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  for (const auto & point : points_) {
    sum += point.xyz;
  }
  return sum / static_cast<double>(points_.size());
}

double GaussianSnapshot::mean_opacity() const
{
  if (points_.empty()) {
    return 0.0;
  }
  double sum = 0.0;
  for (const auto & point : points_) {
    sum += point.opacity;
  }
  return sum / static_cast<double>(points_.size());
}

SlidingWindowPointToPointFactor GaussianSnapshot::build_point_to_point_factor(
  const std::vector<Eigen::Vector3d> & frame_points_i,
  const TrajectoryPose & predicted_pose,
  const size_t min_points,
  const size_t max_frame_points,
  const double nearest_distance_m,
  const double min_opacity) const
{
  SlidingWindowPointToPointFactor factor;
  factor.stamp_ns = predicted_pose.stamp_ns;
  if (!complete() || frame_points_i.size() < min_points || points_.size() < min_points ||
    nearest_distance_m <= 0.0)
  {
    return factor;
  }

  const size_t stride = max_frame_points > 0U && frame_points_i.size() > max_frame_points
    ? static_cast<size_t>(std::ceil(static_cast<double>(frame_points_i.size()) / static_cast<double>(max_frame_points)))
    : 1U;
  const double max_distance_sq = nearest_distance_m * nearest_distance_m;
  const double robust_kernel_m = 0.5 * nearest_distance_m;
  factor.weight = 1.0 / std::max(max_distance_sq, 1.0e-12);
  for (size_t point_index = 0; point_index < frame_points_i.size(); point_index += stride) {
    const auto & point_i = frame_points_i[point_index];
    const Eigen::Vector3d point_w = predicted_pose.q_w_i * point_i + predicted_pose.p_w_i;
    double best_distance_sq = std::numeric_limits<double>::infinity();
    Eigen::Vector3d best_point_w = Eigen::Vector3d::Zero();
    for (const auto & gaussian : points_) {
      if (gaussian.opacity < min_opacity) {
        continue;
      }
      const double distance_sq = (gaussian.xyz - point_w).squaredNorm();
      if (distance_sq < best_distance_sq) {
        best_distance_sq = distance_sq;
        best_point_w = gaussian.xyz;
      }
    }
    if (best_distance_sq <= max_distance_sq) {
      const double residual_norm = std::sqrt(best_distance_sq);
      factor.frame_points_i.push_back(point_i);
      factor.target_points_w.push_back(best_point_w);
      factor.point_weights.push_back(std::min(1.0, robust_kernel_m / std::max(residual_norm, 1.0e-12)));
    }
  }
  if (factor.frame_points_i.size() < min_points) {
    factor.frame_points_i.clear();
    factor.target_points_w.clear();
    factor.point_weights.clear();
  }
  return factor;
}

}  // namespace gaussian_lic_tracking
