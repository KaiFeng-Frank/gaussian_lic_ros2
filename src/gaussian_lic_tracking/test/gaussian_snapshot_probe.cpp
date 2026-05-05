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
  std::cout << "gaussian_snapshot_probe OK\n";
  return 0;
}
