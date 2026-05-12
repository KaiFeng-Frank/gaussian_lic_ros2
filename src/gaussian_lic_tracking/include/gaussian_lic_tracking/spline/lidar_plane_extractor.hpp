// SPDX-License-Identifier: GPL-3.0-or-later
//
// Per-frame voxel-grid plane feature extractor. Replaces the configurable
// single-plane prior with a real LIO-style planar primitive map: voxelize
// each PointCloud, fit a plane per voxel via covariance eigendecomposition,
// and emit one `LidarPointCorrespondence` per accepted voxel for the
// continuous-time LiDAR factor.

#pragma once

#include <cstdint>
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

private:
  LidarPlaneExtractorOptions options_;
};

}  // namespace spline
}  // namespace gaussian_lic_tracking
