// SPDX-License-Identifier: GPL-3.0-or-later
//
// Validates the LOAM scan-to-submap association (ported from upstream Coco-LIC
// FindCorrespondence): a line submap yields a kEdge correspondence with the
// correct tangent, a plane submap yields a kPlane correspondence with the
// correct normal, and a far query associates with nothing.

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include <Eigen/Core>

#include <gaussian_lic_tracking/spline/lidar_loam_submap.hpp>

using gaussian_lic_tracking::spline::LidarFeatureGeometry;
using gaussian_lic_tracking::spline::LidarLoamSubmap;
using gaussian_lic_tracking::spline::LoamSubmapOptions;

int main()
{
  LoamSubmapOptions options;
  options.search_voxel_size_m = 0.5;
  options.max_neighbor_distance_m = 1.0;
  LidarLoamSubmap submap(options);

  // --- Corner / line: a vertical line at (2, 1, z) ---
  std::vector<Eigen::Vector3d> line_points;
  for (double z = 0.0; z <= 2.0; z += 0.05) {
    line_points.emplace_back(2.0, 1.0, z);
  }
  submap.add_corner_points_world(line_points);
  if (submap.corner_size() != line_points.size()) {
    std::fprintf(stderr, "corner submap size mismatch\n");
    return 1;
  }
  const Eigen::Vector3d corner_query(2.02, 1.01, 1.0);
  const auto corner = submap.associate_corner(corner_query, corner_query);
  if (!corner) {
    std::fprintf(stderr, "corner association returned nothing\n");
    return 1;
  }
  if (corner->geometry != LidarFeatureGeometry::kEdge) {
    std::fprintf(stderr, "corner association is not kEdge\n");
    return 1;
  }
  const double tangent_dot = std::abs(corner->edge_normal.dot(Eigen::Vector3d(0.0, 0.0, 1.0)));
  if (tangent_dot < 0.99) {
    std::fprintf(stderr, "corner tangent mismatch |dot|=%.4f\n", tangent_dot);
    return 1;
  }

  // --- Surface / plane: a horizontal plane at z = 2 (not through origin) ---
  std::vector<Eigen::Vector3d> plane_points;
  for (double x = 1.0; x <= 3.0; x += 0.1) {
    for (double y = 0.0; y <= 2.0; y += 0.1) {
      plane_points.emplace_back(x, y, 2.0);
    }
  }
  submap.add_surface_points_world(plane_points);
  const Eigen::Vector3d plane_query(2.0, 1.0, 2.03);
  const auto surface = submap.associate_surface(plane_query, plane_query);
  if (!surface) {
    std::fprintf(stderr, "surface association returned nothing\n");
    return 1;
  }
  if (surface->geometry != LidarFeatureGeometry::kPlane) {
    std::fprintf(stderr, "surface association is not kPlane\n");
    return 1;
  }
  const double normal_dot =
    std::abs(surface->plane.head<3>().dot(Eigen::Vector3d(0.0, 0.0, 1.0)));
  if (normal_dot < 0.99) {
    std::fprintf(stderr, "surface normal mismatch |dot|=%.4f\n", normal_dot);
    return 1;
  }
  // Residual of the query against the recovered plane should be ~0.03 m.
  const double residual =
    std::abs(surface->plane.head<3>().dot(plane_query) + surface->plane[3]);
  if (residual > 0.1) {
    std::fprintf(stderr, "surface residual too large: %.4f\n", residual);
    return 1;
  }

  // --- Negative: a far query must not associate ---
  const Eigen::Vector3d far(50.0, 50.0, 50.0);
  if (submap.associate_corner(far, far) || submap.associate_surface(far, far)) {
    std::fprintf(stderr, "far query wrongly associated\n");
    return 1;
  }

  std::printf("lidar_loam_submap_probe ok\n");
  return 0;
}
