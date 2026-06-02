// SPDX-License-Identifier: GPL-3.0-or-later
//
// Per-frame voxel-grid plane feature extractor. Replaces the configurable
// single-plane prior with a real LIO-style planar primitive map: voxelize
// each PointCloud, fit a plane per voxel via covariance eigendecomposition,
// and emit one `LidarPointCorrespondence` per accepted voxel for the
// continuous-time LiDAR factor.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <gaussian_lic_tracking/spline/continuous_time_lidar_factor.hpp>

namespace gaussian_lic_tracking
{
namespace spline
{

struct VoxelKey
{
  std::int64_t x{0};
  std::int64_t y{0};
  std::int64_t z{0};

  bool operator==(const VoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey & k) const noexcept
  {
    std::size_t h = static_cast<std::size_t>(k.x);
    h ^= static_cast<std::size_t>(k.y) + 0x9e3779b9 + (h << 6) + (h >> 2);
    h ^= static_cast<std::size_t>(k.z) + 0x9e3779b9 + (h << 6) + (h >> 2);
    return h;
  }
};

struct LidarPlaneExtractorOptions
{
  double voxel_size_m{0.5};
  int min_points_per_voxel{5};
  // Planarity test: smallest eigenvalue / largest eigenvalue ratio threshold.
  double planar_eigenvalue_ratio{0.05};
  // Discard points whose distance to the recovered plane exceeds this margin
  // (m). Acts as an inlier filter for noisy voxels.
  double max_inlier_distance_m{0.10};
  // Optional minimum/maximum point range filter applied before voxelization.
  double min_range_m{0.3};
  double max_range_m{60.0};
  // Hard cap on the number of correspondences emitted per call.
  int max_correspondences{256};
  // Edge/line features: a voxel is linear (an edge) when the MIDDLE eigenvalue
  // is small relative to the LARGEST one, i.e. the points spread along a single
  // dominant direction. extract_edges() accepts a voxel when
  // lambda_mid / lambda_max <= linear_eigenvalue_ratio. Edge features observe
  // the yaw / along-corridor directions that plane (point-to-plane) features
  // leave degenerate.
  double linear_eigenvalue_ratio{0.1};
  int max_edge_correspondences{128};
};

// Plain-vanilla collection of accepted plane correspondences. Each item is
// ready to be fed to `ContinuousTimeLidarFactor` / TrajectoryEstimator.
struct ExtractedPlane
{
  Eigen::Vector3d centroid{Eigen::Vector3d::Zero()};
  Eigen::Vector3d normal{Eigen::Vector3d::UnitZ()};
  double offset{0.0};
  Eigen::Vector3d sample_point{Eigen::Vector3d::Zero()};
  int support_points{0};
  double eigenvalue_ratio{0.0};
};

// A linear ("edge") voxel: points spread along a single dominant direction.
// `direction` is the unit line tangent (largest-eigenvalue eigenvector);
// `centroid` is a point on the line. Fed to the continuous-time LiDAR factor
// as a point-to-line (kEdge) residual: r = |(p_map - edge_point) x edge_dir|.
struct ExtractedEdge
{
  Eigen::Vector3d centroid{Eigen::Vector3d::Zero()};
  Eigen::Vector3d direction{Eigen::Vector3d::UnitX()};
  Eigen::Vector3d sample_point{Eigen::Vector3d::Zero()};
  int support_points{0};
  // lambda_mid / lambda_max: smaller is "more linear".
  double eigenvalue_ratio{0.0};
};

struct PersistentPlaneMapOptions
{
  int max_planes{512};
  double max_point_to_plane_distance_m{0.25};
  double min_normal_dot{0.95};
  int min_observations_for_match{3};
};

struct PersistentPlane
{
  Eigen::Vector3d centroid_world{Eigen::Vector3d::Zero()};
  Eigen::Vector3d normal_world{Eigen::Vector3d::UnitZ()};
  double offset_world{0.0};
  int observations{0};
};

struct PersistentPlaneMatch
{
  int index{-1};
  Eigen::Vector4d plane{Eigen::Vector4d::Zero()};
  double point_to_plane_distance_m{0.0};
  double normal_dot{0.0};
};

class PersistentPlaneMap
{
public:
  explicit PersistentPlaneMap(const PersistentPlaneMapOptions & options = {})
  : options_(options)
  {
  }

  void set_options(const PersistentPlaneMapOptions & options) { options_ = options; }
  const PersistentPlaneMapOptions & options() const { return options_; }
  const std::vector<PersistentPlane> & planes() const { return planes_; }
  std::size_t size() const { return planes_.size(); }

  std::optional<PersistentPlaneMatch> match(
    const Eigen::Vector3d & centroid_world,
    const Eigen::Vector3d & normal_world) const
  {
    return match_impl(centroid_world, normal_world, true);
  }

  std::optional<PersistentPlaneMatch> match_for_update(
    const Eigen::Vector3d & centroid_world,
    const Eigen::Vector3d & normal_world) const
  {
    return match_impl(centroid_world, normal_world, false);
  }

private:
  std::optional<PersistentPlaneMatch> match_impl(
    const Eigen::Vector3d & centroid_world,
    const Eigen::Vector3d & normal_world,
    bool require_mature_observation_count) const
  {
    if (!centroid_world.allFinite() || !normal_world.allFinite()) {
      return std::nullopt;
    }
    const double normal_norm = normal_world.norm();
    if (normal_norm < 1.0e-9) {
      return std::nullopt;
    }
    const Eigen::Vector3d n = normal_world / normal_norm;

    std::optional<PersistentPlaneMatch> best;
    double best_distance = options_.max_point_to_plane_distance_m;
    for (std::size_t i = 0; i < planes_.size(); ++i) {
      const auto & plane = planes_[i];
      if (require_mature_observation_count &&
        plane.observations < options_.min_observations_for_match)
      {
        continue;
      }
      const double signed_normal_dot = plane.normal_world.dot(n);
      const double normal_dot = std::abs(signed_normal_dot);
      if (normal_dot < options_.min_normal_dot) {
        continue;
      }
      const double distance =
        std::abs(plane.normal_world.dot(centroid_world) + plane.offset_world);
      if (distance > best_distance) {
        continue;
      }
      Eigen::Vector4d plane_coeffs;
      plane_coeffs.head<3>() = plane.normal_world;
      plane_coeffs[3] = plane.offset_world;
      if (signed_normal_dot < 0.0) {
        plane_coeffs = -plane_coeffs;
      }
      PersistentPlaneMatch match;
      match.index = static_cast<int>(i);
      match.plane = plane_coeffs;
      match.point_to_plane_distance_m = distance;
      match.normal_dot = normal_dot;
      best = match;
      best_distance = distance;
    }
    return best;
  }

public:
  std::optional<int> add_or_update(
    const Eigen::Vector3d & centroid_world,
    const Eigen::Vector3d & normal_world,
    double offset_world)
  {
    if (!centroid_world.allFinite() || !normal_world.allFinite() ||
      !std::isfinite(offset_world))
    {
      return std::nullopt;
    }
    const double normal_norm = normal_world.norm();
    if (normal_norm < 1.0e-9) {
      return std::nullopt;
    }
    Eigen::Vector3d n = normal_world / normal_norm;
    double d = offset_world / normal_norm;

    const auto existing = match_for_update(centroid_world, n);
    if (existing) {
      auto & plane = planes_[static_cast<std::size_t>(existing->index)];
      if (plane.normal_world.dot(n) < 0.0) {
        n = -n;
        d = -d;
      }
      const double old_weight = static_cast<double>(std::max(1, plane.observations));
      plane.centroid_world =
        (old_weight * plane.centroid_world + centroid_world) / (old_weight + 1.0);
      plane.normal_world = (old_weight * plane.normal_world + n).normalized();
      // Keep the plane passing through the averaged centroid. This is more
      // stable than averaging signed offsets when normal signs were flipped.
      plane.offset_world = -plane.normal_world.dot(plane.centroid_world);
      plane.observations += 1;
      return existing->index;
    }

    if (options_.max_planes > 0 &&
      static_cast<int>(planes_.size()) >= options_.max_planes)
    {
      return std::nullopt;
    }
    PersistentPlane plane;
    plane.centroid_world = centroid_world;
    plane.normal_world = n;
    plane.offset_world = d;
    plane.observations = 1;
    planes_.push_back(plane);
    return static_cast<int>(planes_.size() - 1);
  }

private:
  PersistentPlaneMapOptions options_;
  std::vector<PersistentPlane> planes_;
};

class LidarPlaneExtractor
{
public:
  explicit LidarPlaneExtractor(const LidarPlaneExtractorOptions & options = {})
  : options_(options)
  {
  }

  void set_options(const LidarPlaneExtractorOptions & options) { options_ = options; }
  const LidarPlaneExtractorOptions & options() const { return options_; }

  std::vector<ExtractedPlane> extract(
    const std::vector<Eigen::Vector3d> & points_in_lidar_frame) const
  {
    std::vector<ExtractedPlane> output;
    if (points_in_lidar_frame.empty() || options_.voxel_size_m <= 0.0) {
      return output;
    }

    const double inv_voxel = 1.0 / options_.voxel_size_m;
    std::unordered_map<VoxelKey, std::vector<Eigen::Vector3d>, VoxelKeyHash> voxels;
    voxels.reserve(points_in_lidar_frame.size() / 4 + 4);

    for (const auto & p : points_in_lidar_frame) {
      if (!p.allFinite()) {
        continue;
      }
      const double range = p.norm();
      if (range < options_.min_range_m || range > options_.max_range_m) {
        continue;
      }
      VoxelKey key{
        static_cast<std::int64_t>(std::floor(p.x() * inv_voxel)),
        static_cast<std::int64_t>(std::floor(p.y() * inv_voxel)),
        static_cast<std::int64_t>(std::floor(p.z() * inv_voxel))};
      voxels[key].push_back(p);
    }

    output.reserve(voxels.size());

    for (auto & entry : voxels) {
      const auto & samples = entry.second;
      if (static_cast<int>(samples.size()) < options_.min_points_per_voxel) {
        continue;
      }

      Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
      for (const auto & p : samples) {
        centroid += p;
      }
      centroid /= static_cast<double>(samples.size());

      Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
      for (const auto & p : samples) {
        const Eigen::Vector3d d = p - centroid;
        covariance += d * d.transpose();
      }
      covariance /= static_cast<double>(samples.size());

      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(covariance);
      if (eig.info() != Eigen::Success) {
        continue;
      }
      const Eigen::Vector3d eigenvalues = eig.eigenvalues();
      const double lambda_min = eigenvalues[0];
      const double lambda_max = eigenvalues[2];
      if (lambda_max <= 0.0) {
        continue;
      }
      const double ratio = lambda_min / lambda_max;
      if (ratio > options_.planar_eigenvalue_ratio) {
        continue;
      }

      Eigen::Vector3d normal = eig.eigenvectors().col(0).normalized();
      // Orient the normal toward the LiDAR origin so plane signs are
      // consistent for downstream residual evaluation.
      if (normal.dot(-centroid) < 0.0) {
        normal = -normal;
      }
      const double offset = -normal.dot(centroid);

      // Inlier filter: drop the voxel if any point is far from the plane.
      double max_abs_dist = 0.0;
      for (const auto & p : samples) {
        max_abs_dist = std::max(
          max_abs_dist, std::abs(normal.dot(p) + offset));
      }
      if (max_abs_dist > options_.max_inlier_distance_m) {
        continue;
      }

      ExtractedPlane plane;
      plane.centroid = centroid;
      plane.normal = normal;
      plane.offset = offset;
      plane.sample_point = samples.front();
      plane.support_points = static_cast<int>(samples.size());
      plane.eigenvalue_ratio = ratio;
      output.push_back(plane);
      if (options_.max_correspondences > 0 &&
        static_cast<int>(output.size()) >= options_.max_correspondences)
      {
        break;
      }
    }

    return output;
  }

  // Convert an `ExtractedPlane` to a `LidarPointCorrespondence` ready for the
  // continuous-time factor. The plane (n, d) is expressed in the LiDAR
  // frame here; callers must compose with the world->map extrinsic before
  // feeding it to the estimator if the residual is evaluated in a different
  // frame.
  static LidarPointCorrespondence to_correspondence(const ExtractedPlane & plane)
  {
    LidarPointCorrespondence pc;
    pc.geometry = LidarFeatureGeometry::kPlane;
    pc.plane.head<3>() = plane.normal;
    pc.plane[3] = plane.offset;
    pc.point_lidar = plane.sample_point;
    return pc;
  }

  // Extract linear ("edge") features: voxels whose points spread along a single
  // dominant direction. Mirrors extract() but classifies linear voxels via the
  // middle/largest eigenvalue ratio. Kept independent of extract() so the
  // existing plane path is byte-for-byte unchanged.
  std::vector<ExtractedEdge> extract_edges(
    const std::vector<Eigen::Vector3d> & points_in_lidar_frame) const
  {
    std::vector<ExtractedEdge> output;
    if (points_in_lidar_frame.empty() || options_.voxel_size_m <= 0.0) {
      return output;
    }

    const double inv_voxel = 1.0 / options_.voxel_size_m;
    std::unordered_map<VoxelKey, std::vector<Eigen::Vector3d>, VoxelKeyHash> voxels;
    voxels.reserve(points_in_lidar_frame.size() / 4 + 4);
    for (const auto & p : points_in_lidar_frame) {
      if (!p.allFinite()) {
        continue;
      }
      const double range = p.norm();
      if (range < options_.min_range_m || range > options_.max_range_m) {
        continue;
      }
      VoxelKey key{
        static_cast<std::int64_t>(std::floor(p.x() * inv_voxel)),
        static_cast<std::int64_t>(std::floor(p.y() * inv_voxel)),
        static_cast<std::int64_t>(std::floor(p.z() * inv_voxel))};
      voxels[key].push_back(p);
    }

    output.reserve(voxels.size());
    for (auto & entry : voxels) {
      const auto & samples = entry.second;
      if (static_cast<int>(samples.size()) < options_.min_points_per_voxel) {
        continue;
      }

      Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
      for (const auto & p : samples) {
        centroid += p;
      }
      centroid /= static_cast<double>(samples.size());

      Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
      for (const auto & p : samples) {
        const Eigen::Vector3d d = p - centroid;
        covariance += d * d.transpose();
      }
      covariance /= static_cast<double>(samples.size());

      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(covariance);
      if (eig.info() != Eigen::Success) {
        continue;
      }
      // Eigenvalues are ascending: [0]=min, [1]=mid, [2]=max.
      const Eigen::Vector3d eigenvalues = eig.eigenvalues();
      const double lambda_mid = eigenvalues[1];
      const double lambda_max = eigenvalues[2];
      if (lambda_max <= 0.0) {
        continue;
      }
      const double ratio = lambda_mid / lambda_max;
      if (ratio > options_.linear_eigenvalue_ratio) {
        continue;  // not linear enough (plane-like or volumetric voxel)
      }

      const Eigen::Vector3d direction = eig.eigenvectors().col(2).normalized();
      // Inlier filter: every point must lie close to the recovered line.
      double max_abs_dist = 0.0;
      for (const auto & p : samples) {
        max_abs_dist = std::max(
          max_abs_dist, ((p - centroid).cross(direction)).norm());
      }
      if (max_abs_dist > options_.max_inlier_distance_m) {
        continue;
      }

      ExtractedEdge edge;
      edge.centroid = centroid;
      edge.direction = direction;
      edge.sample_point = samples.front();
      edge.support_points = static_cast<int>(samples.size());
      edge.eigenvalue_ratio = ratio;
      output.push_back(edge);
      if (options_.max_edge_correspondences > 0 &&
        static_cast<int>(output.size()) >= options_.max_edge_correspondences)
      {
        break;
      }
    }

    return output;
  }

  // Convert an `ExtractedEdge` (LiDAR frame) to a kEdge correspondence. The
  // edge point + tangent are in the LiDAR frame here; callers compose with the
  // world/map transform before feeding the estimator, exactly like planes.
  static LidarPointCorrespondence to_edge_correspondence(const ExtractedEdge & edge)
  {
    LidarPointCorrespondence pc;
    pc.geometry = LidarFeatureGeometry::kEdge;
    pc.edge_point = edge.centroid;
    pc.edge_normal = edge.direction;
    pc.point_lidar = edge.sample_point;
    return pc;
  }

private:
  LidarPlaneExtractorOptions options_;
};

}  // namespace spline
}  // namespace gaussian_lic_tracking
