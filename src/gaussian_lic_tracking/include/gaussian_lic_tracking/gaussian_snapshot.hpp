// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>

#include <gaussian_lic_msgs/msg/gaussian_array.hpp>
#include <gaussian_lic_tracking/sliding_window_optimizer.hpp>
#include <gaussian_lic_tracking/trajectory_manager.hpp>

namespace gaussian_lic_tracking
{

struct GaussianSnapshotPoint
{
  uint32_t id{0};
  Eigen::Vector3d xyz{Eigen::Vector3d::Zero()};
  Eigen::Vector3d scale{Eigen::Vector3d::Zero()};
  double opacity{0.0};
  double confidence{0.0};
  uint8_t flags{0};
};

struct GaussianSnapshotNearest
{
  bool matched{false};
  Eigen::Vector3d xyz{Eigen::Vector3d::Zero()};
  double distance_sq{0.0};
  size_t point_index{0U};
};

class GaussianSnapshot
{
public:
  void clear();
  bool ingest(const gaussian_lic_msgs::msg::GaussianArray & msg);

  bool complete() const;
  int64_t stamp_ns() const { return stamp_ns_; }
  uint32_t expected_total_count() const { return expected_total_count_; }
  uint32_t expected_chunk_count() const { return expected_chunk_count_; }
  size_t received_chunk_count() const { return received_chunk_count_; }
  size_t point_count() const { return points_.size(); }
  const std::vector<GaussianSnapshotPoint> & points() const { return points_; }

  Eigen::Vector3d centroid() const;
  double mean_opacity() const;
  GaussianSnapshotNearest find_nearest(
    const Eigen::Vector3d & query,
    double max_distance_m,
    double min_opacity,
    int subsample_stride) const;
  SlidingWindowPointToPointFactor build_point_to_point_factor(
    const std::vector<Eigen::Vector3d> & frame_points_i,
    const TrajectoryPose & predicted_pose,
    size_t min_points,
    size_t max_frame_points,
    double nearest_distance_m,
    double min_opacity) const;

private:
  struct VoxelKey
  {
    int64_t x{0};
    int64_t y{0};
    int64_t z{0};

    bool operator==(const VoxelKey & other) const
    {
      return x == other.x && y == other.y && z == other.z;
    }
  };

  struct VoxelKeyHash
  {
    size_t operator()(const VoxelKey & key) const;
  };

  void reset_sequence(int64_t stamp_ns, uint32_t total_count, uint32_t chunk_count);
  void invalidate_spatial_index();
  void ensure_spatial_index(double voxel_size_m, double min_opacity, int subsample_stride) const;
  VoxelKey voxel_key(const Eigen::Vector3d & point, double voxel_size_m) const;

  int64_t stamp_ns_{0};
  uint32_t expected_total_count_{0};
  uint32_t expected_chunk_count_{0};
  size_t received_chunk_count_{0};
  std::vector<bool> received_chunks_;
  std::vector<GaussianSnapshotPoint> points_;

  mutable std::unordered_map<VoxelKey, std::vector<size_t>, VoxelKeyHash> spatial_index_;
  mutable double spatial_index_voxel_size_m_{0.0};
  mutable double spatial_index_min_opacity_{-1.0};
  mutable int spatial_index_subsample_stride_{0};
  mutable int64_t spatial_index_stamp_ns_{0};
  mutable size_t spatial_index_point_count_{0U};
};

}  // namespace gaussian_lic_tracking
