// SPDX-License-Identifier: GPL-3.0-or-later
//
// LOAM-style scan-to-submap LiDAR association, ported faithfully from upstream
// Coco-LIC (external/Coco-LIC/src/lidar/lidar_handler.cpp FindCorrespondence).
//
// The continuous-time port previously matched per-scan voxel features only,
// which is self-referential (the local map drifts with the estimate) and does
// not pin pose across scans. Upstream instead matches each current-scan feature
// point against an ACCUMULATED keyframe submap via 5-nearest-neighbour line
// (corner) / plane (surface) fitting. This module reproduces that association
// without a PCL dependency: the submap is a voxel-hash grid that supports a
// 5-NN query, and the line/plane fits mirror upstream's covariance-eigen (line)
// and QR plane solve, including the same `s = 1 - 0.9*|d|...` quality scale.
//
// Each accepted association is emitted as a `LidarPointCorrespondence`:
//   corner -> kEdge  (edge_point = line centroid, edge_normal = line tangent)
//   surface-> kPlane (plane = (n, d))
// which the existing ContinuousTimeLidarFactor already consumes.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include <gaussian_lic_tracking/spline/continuous_time_lidar_factor.hpp>
#include <gaussian_lic_tracking/spline/lidar_plane_extractor.hpp>  // VoxelKey, VoxelKeyHash

namespace gaussian_lic_tracking
{
namespace spline
{

struct LoamSubmapOptions
{
  // Voxel size of the hash grid used for nearest-neighbour search (m).
  double search_voxel_size_m{0.5};
  // Upstream rejects a match when the 5th neighbour squared-distance > 1.0,
  // i.e. neighbour distance > 1.0 m.
  double max_neighbor_distance_m{1.0};
  // Submap capacity per feature type; oldest points are dropped first.
  std::size_t max_points{40000};
  // Corner/line acceptance: largest eigenvalue > ratio * middle eigenvalue.
  double line_eigenvalue_ratio{3.0};
  // Surface/plane validity: every fitting point within this distance (m).
  double plane_inlier_distance_m{0.2};
  // Reject associations whose quality scale falls to/below this (upstream 0.1).
  double min_scale{0.1};
};

// A voxel-hash point set supporting bounded insertion and 5-NN query. Faithful
// stand-in for upstream's pcl::KdTreeFLANN submap without the PCL dependency.
class VoxelHashCloud
{
public:
  explicit VoxelHashCloud(double voxel_size_m, std::size_t max_points)
  : voxel_size_m_(voxel_size_m > 0.0 ? voxel_size_m : 0.5),
    inv_voxel_(1.0 / (voxel_size_m > 0.0 ? voxel_size_m : 0.5)),
    max_points_(max_points)
  {
  }

  std::size_t size() const { return points_.size(); }
  void clear()
  {
    points_.clear();
    grid_.clear();
    write_cursor_ = 0;
  }

  void add_point(const Eigen::Vector3d & p)
  {
    if (!p.allFinite()) {
      return;
    }
    if (max_points_ > 0 && points_.size() >= max_points_) {
      // Ring-buffer eviction: overwrite the oldest point and rebuild its grid
      // membership lazily (the stale grid entry is filtered at query time).
      points_[write_cursor_] = p;
      grid_[key_of(p)].push_back(write_cursor_);
      write_cursor_ = (write_cursor_ + 1) % max_points_;
      return;
    }
    const std::size_t index = points_.size();
    points_.push_back(p);
    grid_[key_of(p)].push_back(index);
  }

  // Collect the (up to k) nearest neighbours within the 3x3x3 voxel block
  // around `query`. Returns the squared distances (ascending) and points.
  bool nearest_k(
    const Eigen::Vector3d & query, int k,
    std::vector<Eigen::Vector3d> & neighbors, double & kth_distance_sq) const
  {
    neighbors.clear();
    if (k <= 0 || points_.empty()) {
      return false;
    }
    const VoxelKey center = key_of(query);
    std::vector<std::pair<double, std::size_t>> candidates;
    for (std::int64_t dx = -1; dx <= 1; ++dx) {
      for (std::int64_t dy = -1; dy <= 1; ++dy) {
        for (std::int64_t dz = -1; dz <= 1; ++dz) {
          const VoxelKey key{center.x + dx, center.y + dy, center.z + dz};
          const auto it = grid_.find(key);
          if (it == grid_.end()) {
            continue;
          }
          for (const std::size_t index : it->second) {
            if (index >= points_.size()) {
              continue;
            }
            const double dist_sq = (points_[index] - query).squaredNorm();
            candidates.emplace_back(dist_sq, index);
          }
        }
      }
    }
    if (static_cast<int>(candidates.size()) < k) {
      return false;
    }
    std::partial_sort(
      candidates.begin(), candidates.begin() + k, candidates.end(),
      [](const auto & a, const auto & b) { return a.first < b.first; });
    neighbors.reserve(static_cast<std::size_t>(k));
    for (int i = 0; i < k; ++i) {
      neighbors.push_back(points_[candidates[static_cast<std::size_t>(i)].second]);
    }
    kth_distance_sq = candidates[static_cast<std::size_t>(k - 1)].first;
    return true;
  }

private:
  VoxelKey key_of(const Eigen::Vector3d & p) const
  {
    return VoxelKey{
      static_cast<std::int64_t>(std::floor(p.x() * inv_voxel_)),
      static_cast<std::int64_t>(std::floor(p.y() * inv_voxel_)),
      static_cast<std::int64_t>(std::floor(p.z() * inv_voxel_))};
  }

  double voxel_size_m_;
  double inv_voxel_;
  std::size_t max_points_;
  std::vector<Eigen::Vector3d> points_;
  std::unordered_map<VoxelKey, std::vector<std::size_t>, VoxelKeyHash> grid_;
  std::size_t write_cursor_{0};
};

// Accumulated corner + surface submap with LOAM 5-NN line/plane association.
// All points are stored in the map/world frame; query points must also be in
// the map frame (transform the current scan via the estimated pose first).
class LidarLoamSubmap
{
public:
  explicit LidarLoamSubmap(const LoamSubmapOptions & options = {})
  : options_(options),
    corner_cloud_(options.search_voxel_size_m, options.max_points),
    surface_cloud_(options.search_voxel_size_m, options.max_points)
  {
  }

  void set_options(const LoamSubmapOptions & options) { options_ = options; }
  const LoamSubmapOptions & options() const { return options_; }
  std::size_t corner_size() const { return corner_cloud_.size(); }
  std::size_t surface_size() const { return surface_cloud_.size(); }
  void clear()
  {
    corner_cloud_.clear();
    surface_cloud_.clear();
  }

  void add_corner_points_world(const std::vector<Eigen::Vector3d> & pts)
  {
    for (const auto & p : pts) {
      corner_cloud_.add_point(p);
    }
  }
  void add_surface_points_world(const std::vector<Eigen::Vector3d> & pts)
  {
    for (const auto & p : pts) {
      surface_cloud_.add_point(p);
    }
  }

  // Corner -> line. `point_lidar` is the original (lidar-frame) point kept for
  // the factor; `point_map` is that point transformed into the map frame by the
  // current pose estimate. Returns a kEdge correspondence on success.
  std::optional<LidarPointCorrespondence> associate_corner(
    const Eigen::Vector3d & point_lidar, const Eigen::Vector3d & point_map) const
  {
    std::vector<Eigen::Vector3d> nn;
    double kth_sq = 0.0;
    if (!corner_cloud_.nearest_k(point_map, 5, nn, kth_sq)) {
      return std::nullopt;
    }
    if (kth_sq > options_.max_neighbor_distance_m * options_.max_neighbor_distance_m) {
      return std::nullopt;
    }
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (const auto & p : nn) {
      centroid += p;
    }
    centroid /= 5.0;
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (const auto & p : nn) {
      const Eigen::Vector3d d = p - centroid;
      cov += d * d.transpose();
    }
    cov /= 5.0;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(cov);
    if (eig.info() != Eigen::Success) {
      return std::nullopt;
    }
    // Ascending eigenvalues: [0]<=[1]<=[2]. Upstream (descending) requires the
    // largest > 3 * second-largest -> here lambda[2] > ratio * lambda[1].
    const Eigen::Vector3d lambda = eig.eigenvalues();
    if (lambda[2] <= options_.line_eigenvalue_ratio * lambda[1]) {
      return std::nullopt;
    }
    const Eigen::Vector3d direction = eig.eigenvectors().col(2).normalized();
    // Point-to-line distance for the quality scale (upstream ld2).
    const double ld2 = ((point_map - centroid).cross(direction)).norm();
    const double s = 1.0 - 0.9 * std::fabs(ld2);
    if (s <= options_.min_scale) {
      return std::nullopt;
    }
    LidarPointCorrespondence pc;
    pc.geometry = LidarFeatureGeometry::kEdge;
    pc.edge_point = centroid;
    pc.edge_normal = direction;
    pc.point_lidar = point_lidar;
    pc.scale = s;
    return pc;
  }

  // Surface -> plane. Mirrors upstream's QR plane solve + inlier validation.
  std::optional<LidarPointCorrespondence> associate_surface(
    const Eigen::Vector3d & point_lidar, const Eigen::Vector3d & point_map) const
  {
    std::vector<Eigen::Vector3d> nn;
    double kth_sq = 0.0;
    if (!surface_cloud_.nearest_k(point_map, 5, nn, kth_sq)) {
      return std::nullopt;
    }
    if (kth_sq > options_.max_neighbor_distance_m * options_.max_neighbor_distance_m) {
      return std::nullopt;
    }
    // Solve [x y z] . n = -1 for the 5 neighbours, then normalise to (n, d).
    Eigen::Matrix<double, 5, 3> a;
    Eigen::Matrix<double, 5, 1> b;
    b.setConstant(-1.0);
    for (int j = 0; j < 5; ++j) {
      a.row(j) = nn[static_cast<std::size_t>(j)].transpose();
    }
    const Eigen::Vector3d x = a.colPivHouseholderQr().solve(b);
    const double norm = x.norm();
    if (!(norm > 1.0e-9) || !x.allFinite()) {
      return std::nullopt;
    }
    Eigen::Vector3d normal = x / norm;
    double d = 1.0 / norm;
    for (const auto & p : nn) {
      if (std::fabs(normal.dot(p) + d) > options_.plane_inlier_distance_m) {
        return std::nullopt;
      }
    }
    const double range = point_map.norm();
    const double range_sqrt = std::sqrt(std::max(range, 1.0e-6));
    const double pd2 = normal.dot(point_map) + d;
    const double s = 1.0 - 0.9 * std::fabs(pd2) / range_sqrt;
    if (s <= options_.min_scale) {
      return std::nullopt;
    }
    LidarPointCorrespondence pc;
    pc.geometry = LidarFeatureGeometry::kPlane;
    pc.plane.head<3>() = normal;
    pc.plane[3] = d;
    pc.point_lidar = point_lidar;
    pc.scale = s;
    return pc;
  }

private:
  LoamSubmapOptions options_;
  VoxelHashCloud corner_cloud_;
  VoxelHashCloud surface_cloud_;
};

}  // namespace spline
}  // namespace gaussian_lic_tracking
