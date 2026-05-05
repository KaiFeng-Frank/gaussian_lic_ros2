// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <cstddef>
#include <cstdint>
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

  void insert_keyframe(
    const std::vector<Eigen::Vector3d> & frame_points_i,
    const TrajectoryPose & pose);

private:
  static std::vector<Eigen::Vector3d> sample_points(
    const std::vector<Eigen::Vector3d> & points,
    size_t max_points);

  LidarFactorConfig config_;
  std::vector<Eigen::Vector3d> map_points_w_;
};

}  // namespace gaussian_lic_tracking
