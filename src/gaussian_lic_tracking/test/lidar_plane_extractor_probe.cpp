// SPDX-License-Identifier: GPL-3.0-or-later
//
// Verifies the voxel-grid plane feature extractor recovers ground + wall
// planes from a synthetic two-plane scene and rejects non-planar voxels.

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <vector>

#include <Eigen/Core>

#include <gaussian_lic_tracking/spline/lidar_plane_extractor.hpp>

using gaussian_lic_tracking::spline::ExtractedPlane;
using gaussian_lic_tracking::spline::LidarPlaneExtractor;
using gaussian_lic_tracking::spline::LidarPlaneExtractorOptions;

namespace
{

void check_ground_plane_recovery()
{
  std::vector<Eigen::Vector3d> points;
  // Dense ground plane: z = 0, x in [1, 5], y in [-2, 2].
  for (double x = 1.0; x <= 5.0; x += 0.1) {
    for (double y = -2.0; y <= 2.0; y += 0.1) {
      points.emplace_back(x, y, 0.0);
    }
  }
  LidarPlaneExtractorOptions options;
  options.voxel_size_m = 0.5;
  options.min_points_per_voxel = 8;
  options.planar_eigenvalue_ratio = 0.05;
  options.max_inlier_distance_m = 0.02;
  LidarPlaneExtractor extractor(options);
  const auto planes = extractor.extract(points);
  if (planes.empty()) {
    std::fprintf(stderr, "ground plane recovery returned no planes\n");
    std::exit(1);
  }
  for (const auto & plane : planes) {
    const Eigen::Vector3d expected_normal(0.0, 0.0, 1.0);
    const double dot_match = std::abs(plane.normal.dot(expected_normal));
    if (dot_match < 0.99) {
      std::fprintf(stderr,
        "ground plane normal mismatch: got (%.4f, %.4f, %.4f) |dot|=%.4f\n",
        plane.normal.x(), plane.normal.y(), plane.normal.z(), dot_match);
      std::exit(1);
    }
    if (std::abs(plane.offset) > 0.05) {
      std::fprintf(stderr,
        "ground plane offset mismatch: got %.4f\n", plane.offset);
      std::exit(1);
    }
  }
}

void check_two_orthogonal_planes()
{
  std::vector<Eigen::Vector3d> points;
  // Ground plane.
  for (double x = 1.0; x <= 4.0; x += 0.1) {
    for (double y = -1.5; y <= 1.5; y += 0.1) {
      points.emplace_back(x, y, 0.0);
    }
  }
  // Wall plane at x = 5.
  for (double y = -1.5; y <= 1.5; y += 0.1) {
    for (double z = 0.1; z <= 3.0; z += 0.1) {
      points.emplace_back(5.0, y, z);
    }
  }
  LidarPlaneExtractorOptions options;
  options.voxel_size_m = 0.5;
  options.min_points_per_voxel = 8;
  options.planar_eigenvalue_ratio = 0.05;
  options.max_inlier_distance_m = 0.02;
  LidarPlaneExtractor extractor(options);
  const auto planes = extractor.extract(points);

  bool found_ground = false;
  bool found_wall = false;
  for (const auto & plane : planes) {
    const Eigen::Vector3d n = plane.normal;
    if (std::abs(n.z()) > 0.95) {
      found_ground = true;
    }
    if (std::abs(n.x()) > 0.95) {
      found_wall = true;
    }
  }
  if (!found_ground || !found_wall) {
    std::fprintf(stderr,
      "two-plane scene missing components: ground=%d wall=%d (total planes=%zu)\n",
      static_cast<int>(found_ground),
      static_cast<int>(found_wall),
      planes.size());
    std::exit(1);
  }
}

void check_rejects_random_noise()
{
  std::vector<Eigen::Vector3d> points;
  // Pseudo-random fill: not planar.
  unsigned seed = 7;
  for (int i = 0; i < 4000; ++i) {
    seed = seed * 1103515245 + 12345;
    const double x = 2.0 + 0.5 * ((seed % 1000) / 500.0 - 1.0);
    seed = seed * 1103515245 + 12345;
    const double y = 0.5 * ((seed % 1000) / 500.0 - 1.0);
    seed = seed * 1103515245 + 12345;
    const double z = 1.0 + 0.5 * ((seed % 1000) / 500.0 - 1.0);
    points.emplace_back(x, y, z);
  }
  LidarPlaneExtractorOptions options;
  options.voxel_size_m = 1.0;
  options.min_points_per_voxel = 50;
  options.planar_eigenvalue_ratio = 0.02;
  options.max_inlier_distance_m = 0.05;
  LidarPlaneExtractor extractor(options);
  const auto planes = extractor.extract(points);
  if (!planes.empty()) {
    std::fprintf(stderr,
      "random-noise extractor recovered %zu spurious planes\n", planes.size());
    std::exit(1);
  }
}

}  // namespace

int main()
{
  try {
    check_ground_plane_recovery();
    check_two_orthogonal_planes();
    check_rejects_random_noise();
  } catch (const std::exception & exception) {
    std::fprintf(stderr,
      "lidar_plane_extractor_probe exception: %s\n", exception.what());
    return 1;
  }
  std::printf("lidar_plane_extractor_probe ok\n");
  return 0;
}
