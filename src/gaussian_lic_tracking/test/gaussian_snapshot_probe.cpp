// SPDX-License-Identifier: GPL-3.0-or-later

#include <gaussian_lic_tracking/gaussian_snapshot.hpp>

#include <cmath>
#include <iostream>

namespace
{
gaussian_lic_msgs::msg::Gaussian make_gaussian(
  const uint32_t id,
  const float x,
  const float y,
  const float z,
  const float opacity)
{
  gaussian_lic_msgs::msg::Gaussian gaussian;
  gaussian.id = id;
  gaussian.xyz = {x, y, z};
  gaussian.scale = {0.1F, 0.1F, 0.1F};
  gaussian.opacity = opacity;
  gaussian.confidence = 1.0F;
  return gaussian;
}
}  // namespace

int main()
{
  gaussian_lic_tracking::GaussianSnapshot snapshot;
  gaussian_lic_msgs::msg::GaussianArray first;
  first.header.stamp.sec = 12;
  first.total_count = 3;
  first.chunk_index = 0;
  first.chunk_count = 2;
  first.gaussians.push_back(make_gaussian(1U, 0.0F, 0.0F, 0.0F, 0.2F));
  first.gaussians.push_back(make_gaussian(2U, 1.0F, 0.0F, 0.0F, 0.4F));

  gaussian_lic_msgs::msg::GaussianArray second = first;
  second.chunk_index = 1;
  second.gaussians.clear();
  second.gaussians.push_back(make_gaussian(3U, 0.0F, 1.0F, 2.0F, 0.8F));

  if (!snapshot.ingest(first) || snapshot.complete()) {
    std::cerr << "first Gaussian snapshot chunk handling failed\n";
    return 1;
  }
  if (snapshot.ingest(first)) {
    std::cerr << "duplicate Gaussian snapshot chunk was accepted\n";
    return 1;
  }
  if (!snapshot.ingest(second) || !snapshot.complete()) {
    std::cerr << "Gaussian snapshot did not become complete\n";
    return 1;
  }

  const Eigen::Vector3d centroid = snapshot.centroid();
  const double centroid_error = (centroid - Eigen::Vector3d{1.0 / 3.0, 1.0 / 3.0, 2.0 / 3.0}).norm();
  const double opacity_error = std::abs(snapshot.mean_opacity() - (0.2 + 0.4 + 0.8) / 3.0);
  std::cout << "gaussian_snapshot_probe points=" << snapshot.point_count()
            << " chunks=" << snapshot.received_chunk_count() << "/"
            << snapshot.expected_chunk_count()
            << " centroid_error=" << centroid_error
            << " opacity_error=" << opacity_error << "\n";
  if (snapshot.point_count() != 3U || centroid_error > 1.0e-6 || opacity_error > 1.0e-6) {
    std::cerr << "Gaussian snapshot aggregate statistics are wrong\n";
    return 1;
  }

  gaussian_lic_tracking::TrajectoryPose predicted_pose;
  predicted_pose.stamp_ns = 77;
  predicted_pose.q_w_i = Eigen::Quaterniond::Identity();
  std::vector<Eigen::Vector3d> frame_points{
    Eigen::Vector3d{0.0, 0.0, 0.0},
    Eigen::Vector3d{1.0, 0.0, 0.0},
    Eigen::Vector3d{0.0, 1.0, 2.0}};
  const auto point_factor = snapshot.build_point_to_point_factor(
    frame_points, predicted_pose, 2U, 100U, 0.05, 0.1);
  std::cout << "gaussian_snapshot_point_factor correspondences="
            << point_factor.frame_points_i.size()
            << " weight=" << point_factor.weight
            << " robust_weights=" << point_factor.point_weights.size() << "\n";
  if (point_factor.frame_points_i.size() != frame_points.size() ||
    point_factor.frame_points_i.size() != point_factor.target_points_w.size() ||
    point_factor.point_weights.size() != point_factor.frame_points_i.size())
  {
    std::cerr << "Gaussian snapshot point-to-point factor is invalid\n";
    return 1;
  }
  for (const double weight : point_factor.point_weights) {
    if (weight <= 0.0 || weight > 1.0) {
      std::cerr << "Gaussian snapshot robust point weight is outside (0, 1]\n";
      return 1;
    }
  }
  std::cout << "gaussian_snapshot_probe OK\n";
  return 0;
}
