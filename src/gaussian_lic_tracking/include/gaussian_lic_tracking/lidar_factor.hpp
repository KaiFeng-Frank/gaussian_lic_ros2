// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gaussian_lic_tracking/sliding_window_optimizer.hpp>
#include <gaussian_lic_tracking/trajectory_manager.hpp>

namespace gaussian_lic_tracking
{

struct LidarFactorConfig
{
  size_t min_points{32};
  size_t max_frame_points{2000};
  size_t max_map_points{20000};
  double nearest_distance_m{0.35};
  double correction_gain{0.7};
  double max_correction_m{0.25};
  double max_rotation_rad{0.08};
  double robust_kernel_m{0.15};
  size_t plane_min_neighbors{5};
  double plane_max_condition{0.2};
};

struct LidarCorrection
{
  bool applied{false};
  size_t matched_points{0};
  double mean_residual_m{0.0};
  Eigen::Vector3d delta_p_w{Eigen::Vector3d::Zero()};
};

struct LidarPoseCorrection
{
  bool applied{false};
  size_t matched_points{0};
  double mean_residual_m{0.0};
  Eigen::Vector3d delta_p_w{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond delta_q{Eigen::Quaterniond::Identity()};
};

class LidarFactor
{
public:
  explicit LidarFactor(LidarFactorConfig config = {});

  void set_config(const LidarFactorConfig & config);
  const LidarFactorConfig & config() const { return config_; }
  void clear();
  size_t map_size() const { return map_points_w_.size(); }

  LidarCorrection compute_translation_correction(
    const std::vector<Eigen::Vector3d> & frame_points_i,
    const TrajectoryPose & predicted_pose) const;

  LidarPoseCorrection compute_pose_correction(
    const std::vector<Eigen::Vector3d> & frame_points_i,
    const TrajectoryPose & predicted_pose) const;

  SlidingWindowPointToPointFactor build_point_to_point_factor(
    const std::vector<Eigen::Vector3d> & frame_points_i,
    const TrajectoryPose & predicted_pose) const;

  SlidingWindowPointToPlaneFactor build_point_to_plane_factor(
    const std::vector<Eigen::Vector3d> & frame_points_i,
    const TrajectoryPose & predicted_pose) const;

  void insert_keyframe(
    const std::vector<Eigen::Vector3d> & frame_points_i,
    const TrajectoryPose & pose);

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

  static std::vector<Eigen::Vector3d> sample_points(
    const std::vector<Eigen::Vector3d> & points,
    size_t max_points);
  static VoxelKey voxel_key_for_point(const Eigen::Vector3d & point, double voxel_size);

  void rebuild_spatial_index();
  bool find_nearest_map_point(
    const Eigen::Vector3d & point_w,
    double max_distance_sq,
    Eigen::Vector3d & best_point_w,
    double & best_distance_sq) const;
  std::vector<std::pair<double, Eigen::Vector3d>> find_nearest_map_neighbors(
    const Eigen::Vector3d & point_w,
    double max_distance_sq,
    size_t max_neighbors) const;

  LidarFactorConfig config_;
  std::vector<Eigen::Vector3d> map_points_w_;
  std::unordered_map<VoxelKey, std::vector<size_t>, VoxelKeyHash> map_voxels_;
  double voxel_size_m_{0.0};
};

}  // namespace gaussian_lic_tracking
