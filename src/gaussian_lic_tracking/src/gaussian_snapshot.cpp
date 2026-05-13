// SPDX-License-Identifier: GPL-3.0-or-later

#include <gaussian_lic_tracking/gaussian_snapshot.hpp>
#include <gaussian_lic_tracking/time.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace gaussian_lic_tracking
{

Eigen::Vector3d GaussianSnapshotPoint::normal_w() const
{
  if (!scale.allFinite() || !q_w_g.coeffs().allFinite() || q_w_g.norm() <= 1.0e-9) {
    return Eigen::Vector3d::Zero();
  }
  Eigen::Index min_axis = 0;
  scale.cwiseAbs().minCoeff(&min_axis);
  Eigen::Vector3d axis = Eigen::Vector3d::UnitX();
  if (min_axis == 1) {
    axis = Eigen::Vector3d::UnitY();
  } else if (min_axis == 2) {
    axis = Eigen::Vector3d::UnitZ();
  }
  Eigen::Vector3d normal = q_w_g.normalized() * axis;
  const double norm = normal.norm();
  if (!normal.allFinite() || norm <= 1.0e-9) {
    return Eigen::Vector3d::Zero();
  }
  return normal / norm;
}

double GaussianSnapshotPoint::normal_anisotropy() const
{
  if (!scale.allFinite()) {
    return 0.0;
  }
  const Eigen::Vector3d abs_scale = scale.cwiseAbs();
  const double max_scale = abs_scale.maxCoeff();
  const double min_scale = abs_scale.minCoeff();
  if (!std::isfinite(max_scale) || !std::isfinite(min_scale) || max_scale <= 1.0e-9) {
    return 0.0;
  }
  return std::clamp(1.0 - min_scale / max_scale, 0.0, 1.0);
}

void GaussianSnapshot::clear()
{
  stamp_ns_ = 0;
  expected_total_count_ = 0U;
  expected_chunk_count_ = 0U;
  received_chunk_count_ = 0U;
  received_chunks_.clear();
  points_.clear();
  pending_stamp_ns_ = 0;
  pending_expected_total_count_ = 0U;
  pending_expected_chunk_count_ = 0U;
  pending_received_chunk_count_ = 0U;
  pending_received_chunks_.clear();
  pending_points_.clear();
  invalidate_spatial_index();
}

void GaussianSnapshot::reset_sequence(
  const int64_t stamp_ns,
  const uint32_t total_count,
  const uint32_t chunk_count)
{
  pending_stamp_ns_ = stamp_ns;
  pending_expected_total_count_ = total_count;
  pending_expected_chunk_count_ = chunk_count;
  pending_received_chunk_count_ = 0U;
  pending_received_chunks_.assign(static_cast<size_t>(chunk_count), false);
  pending_points_.clear();
  pending_points_.reserve(total_count);
}

bool GaussianSnapshot::ingest(const gaussian_lic_msgs::msg::GaussianArray & msg)
{
  if (msg.chunk_count == 0U || msg.chunk_index >= msg.chunk_count) {
    return false;
  }
  const int64_t msg_stamp_ns = stamp_to_nanoseconds(msg.header.stamp);
  const bool same_sequence =
    !pending_received_chunks_.empty() &&
    msg_stamp_ns == pending_stamp_ns_ &&
    msg.total_count == pending_expected_total_count_ &&
    msg.chunk_count == pending_expected_chunk_count_;
  if (!same_sequence) {
    reset_sequence(msg_stamp_ns, msg.total_count, msg.chunk_count);
  }

  const size_t chunk_index = static_cast<size_t>(msg.chunk_index);
  if (pending_received_chunks_[chunk_index]) {
    return false;
  }
  pending_received_chunks_[chunk_index] = true;
  ++pending_received_chunk_count_;

  for (const auto & gaussian : msg.gaussians) {
    GaussianSnapshotPoint point;
    point.id = gaussian.id;
    point.xyz = Eigen::Vector3d{
      static_cast<double>(gaussian.xyz[0]),
      static_cast<double>(gaussian.xyz[1]),
      static_cast<double>(gaussian.xyz[2])};
    point.q_w_g = Eigen::Quaterniond{
      static_cast<double>(gaussian.rotation_xyzw[3]),
      static_cast<double>(gaussian.rotation_xyzw[0]),
      static_cast<double>(gaussian.rotation_xyzw[1]),
      static_cast<double>(gaussian.rotation_xyzw[2])};
    if (!point.q_w_g.coeffs().allFinite() || point.q_w_g.norm() <= 1.0e-9) {
      point.q_w_g = Eigen::Quaterniond::Identity();
    } else {
      point.q_w_g.normalize();
    }
    point.scale = Eigen::Vector3d{
      static_cast<double>(gaussian.scale[0]),
      static_cast<double>(gaussian.scale[1]),
      static_cast<double>(gaussian.scale[2])};
    point.opacity = static_cast<double>(gaussian.opacity);
    point.confidence = static_cast<double>(gaussian.confidence);
    point.flags = gaussian.flags;
    pending_points_.push_back(point);
  }
  if (pending_expected_total_count_ > 0U &&
    pending_points_.size() > pending_expected_total_count_)
  {
    pending_points_.resize(pending_expected_total_count_);
  }
  if (pending_complete()) {
    commit_pending_sequence();
  }
  return true;
}

bool GaussianSnapshot::pending_complete() const
{
  return pending_expected_chunk_count_ > 0U &&
    pending_received_chunk_count_ == static_cast<size_t>(pending_expected_chunk_count_) &&
    (pending_expected_total_count_ == 0U ||
    pending_points_.size() == static_cast<size_t>(pending_expected_total_count_));
}

void GaussianSnapshot::commit_pending_sequence()
{
  stamp_ns_ = pending_stamp_ns_;
  expected_total_count_ = pending_expected_total_count_;
  expected_chunk_count_ = pending_expected_chunk_count_;
  received_chunk_count_ = pending_received_chunk_count_;
  received_chunks_ = pending_received_chunks_;
  points_ = std::move(pending_points_);
  pending_points_.clear();
  invalidate_spatial_index();
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

size_t GaussianSnapshot::VoxelKeyHash::operator()(const VoxelKey & key) const
{
  const auto mix = [](uint64_t value) {
    value ^= value >> 30U;
    value *= 0xbf58476d1ce4e5b9ULL;
    value ^= value >> 27U;
    value *= 0x94d049bb133111ebULL;
    value ^= value >> 31U;
    return value;
  };
  const uint64_t hx = mix(static_cast<uint64_t>(key.x) + 0x9e3779b97f4a7c15ULL);
  const uint64_t hy = mix(static_cast<uint64_t>(key.y) + 0xbf58476d1ce4e5b9ULL);
  const uint64_t hz = mix(static_cast<uint64_t>(key.z) + 0x94d049bb133111ebULL);
  return static_cast<size_t>(hx ^ (hy << 1U) ^ (hz << 7U));
}

void GaussianSnapshot::invalidate_spatial_index()
{
  spatial_index_.clear();
  spatial_index_voxel_size_m_ = 0.0;
  spatial_index_min_opacity_ = -1.0;
  spatial_index_subsample_stride_ = 0;
  spatial_index_stamp_ns_ = 0;
  spatial_index_point_count_ = 0U;
}

GaussianSnapshot::VoxelKey GaussianSnapshot::voxel_key(
  const Eigen::Vector3d & point,
  const double voxel_size_m) const
{
  return VoxelKey{
    static_cast<int64_t>(std::floor(point.x() / voxel_size_m)),
    static_cast<int64_t>(std::floor(point.y() / voxel_size_m)),
    static_cast<int64_t>(std::floor(point.z() / voxel_size_m))};
}

void GaussianSnapshot::ensure_spatial_index(
  const double voxel_size_m,
  const double min_opacity,
  const int subsample_stride) const
{
  const int stride = std::max(1, subsample_stride);
  if (!complete() || !std::isfinite(voxel_size_m) || voxel_size_m <= 0.0) {
    return;
  }
  if (!spatial_index_.empty() &&
    spatial_index_voxel_size_m_ == voxel_size_m &&
    spatial_index_min_opacity_ == min_opacity &&
    spatial_index_subsample_stride_ == stride &&
    spatial_index_stamp_ns_ == stamp_ns_ &&
    spatial_index_point_count_ == points_.size())
  {
    return;
  }

  spatial_index_.clear();
  for (size_t index = 0U; index < points_.size(); index += static_cast<size_t>(stride)) {
    const auto & point = points_[index];
    if (!point.xyz.allFinite() || point.opacity < min_opacity) {
      continue;
    }
    spatial_index_[voxel_key(point.xyz, voxel_size_m)].push_back(index);
  }
  spatial_index_voxel_size_m_ = voxel_size_m;
  spatial_index_min_opacity_ = min_opacity;
  spatial_index_subsample_stride_ = stride;
  spatial_index_stamp_ns_ = stamp_ns_;
  spatial_index_point_count_ = points_.size();
}

GaussianSnapshotNearest GaussianSnapshot::find_nearest(
  const Eigen::Vector3d & query,
  const double max_distance_m,
  const double min_opacity,
  const int subsample_stride) const
{
  GaussianSnapshotNearest nearest;
  if (!complete() || points_.empty() || !query.allFinite() ||
    !std::isfinite(max_distance_m) || max_distance_m <= 0.0)
  {
    return nearest;
  }

  ensure_spatial_index(max_distance_m, min_opacity, subsample_stride);
  if (spatial_index_.empty()) {
    return nearest;
  }

  const VoxelKey center = voxel_key(query, max_distance_m);
  double best_distance_sq = max_distance_m * max_distance_m;
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dz = -1; dz <= 1; ++dz) {
        const VoxelKey key{center.x + dx, center.y + dy, center.z + dz};
        const auto bucket_it = spatial_index_.find(key);
        if (bucket_it == spatial_index_.end()) {
          continue;
        }
        for (const size_t point_index : bucket_it->second) {
          if (point_index >= points_.size()) {
            continue;
          }
          const auto & gaussian = points_[point_index];
          const double distance_sq = (gaussian.xyz - query).squaredNorm();
          if (distance_sq <= best_distance_sq) {
            best_distance_sq = distance_sq;
            nearest.matched = true;
            nearest.xyz = gaussian.xyz;
            nearest.distance_sq = distance_sq;
            nearest.point_index = point_index;
          }
        }
      }
    }
  }
  return nearest;
}

SlidingWindowPointToPointFactor GaussianSnapshot::build_point_to_point_factor(
  const std::vector<Eigen::Vector3d> & frame_points_i,
  const TrajectoryPose & predicted_pose,
  const size_t min_points,
  const size_t max_frame_points,
  const double nearest_distance_m,
  const double min_opacity,
  const bool residual_preweight) const
{
  SlidingWindowPointToPointFactor factor;
  factor.stamp_ns = predicted_pose.stamp_ns;
  factor.source_id = 2U;
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
    const auto nearest = find_nearest(point_w, nearest_distance_m, min_opacity, 1);
    if (nearest.matched && nearest.distance_sq <= max_distance_sq) {
      const double residual_norm = std::sqrt(nearest.distance_sq);
      factor.frame_points_i.push_back(point_i);
      factor.target_points_w.push_back(nearest.xyz);
      const double residual_weight = residual_preweight
        ? std::min(1.0, robust_kernel_m / std::max(residual_norm, 1.0e-12))
        : 1.0;
      factor.point_weights.push_back(residual_weight);
    }
  }
  if (factor.frame_points_i.size() < min_points) {
    factor.frame_points_i.clear();
    factor.target_points_w.clear();
    factor.point_weights.clear();
  }
  return factor;
}

SlidingWindowPointToPlaneFactor GaussianSnapshot::build_point_to_plane_factor(
  const std::vector<Eigen::Vector3d> & frame_points_i,
  const TrajectoryPose & predicted_pose,
  const size_t min_points,
  const size_t max_frame_points,
  const double nearest_distance_m,
  const double min_opacity,
  const double min_normal_anisotropy,
  const bool residual_preweight) const
{
  SlidingWindowPointToPlaneFactor factor;
  factor.stamp_ns = predicted_pose.stamp_ns;
  factor.source_id = 3U;
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
  const double anisotropy_threshold = std::clamp(
    std::isfinite(min_normal_anisotropy) ? min_normal_anisotropy : 0.0, 0.0, 1.0);
  factor.weight = 1.0 / std::max(max_distance_sq, 1.0e-12);
  for (size_t point_index = 0; point_index < frame_points_i.size(); point_index += stride) {
    const auto & point_i = frame_points_i[point_index];
    const Eigen::Vector3d point_w = predicted_pose.q_w_i * point_i + predicted_pose.p_w_i;
    const auto nearest = find_nearest(point_w, nearest_distance_m, min_opacity, 1);
    if (!nearest.matched || nearest.distance_sq > max_distance_sq ||
      nearest.point_index >= points_.size())
    {
      continue;
    }
    const auto & gaussian = points_[nearest.point_index];
    const double anisotropy = gaussian.normal_anisotropy();
    const Eigen::Vector3d normal_w = gaussian.normal_w();
    if (anisotropy < anisotropy_threshold || !normal_w.allFinite() || normal_w.norm() <= 1.0e-9) {
      continue;
    }
    const double residual_norm = std::sqrt(nearest.distance_sq);
    factor.frame_points_i.push_back(point_i);
    factor.target_points_w.push_back(nearest.xyz);
    factor.target_normals_w.push_back(normal_w);
    const double robust_weight =
      std::min(1.0, robust_kernel_m / std::max(residual_norm, 1.0e-12));
    const double residual_weight = residual_preweight ? robust_weight : 1.0;
    factor.point_weights.push_back(residual_weight * std::clamp(anisotropy, 0.0, 1.0));
  }
  if (factor.frame_points_i.size() < min_points) {
    factor.frame_points_i.clear();
    factor.target_points_w.clear();
    factor.target_normals_w.clear();
    factor.point_weights.clear();
  }
  return factor;
}

}  // namespace gaussian_lic_tracking
