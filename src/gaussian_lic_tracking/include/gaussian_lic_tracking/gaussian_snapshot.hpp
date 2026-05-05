// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <Eigen/Core>

#include <gaussian_lic_msgs/msg/gaussian_array.hpp>

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

private:
  void reset_sequence(int64_t stamp_ns, uint32_t total_count, uint32_t chunk_count);

  int64_t stamp_ns_{0};
  uint32_t expected_total_count_{0};
  uint32_t expected_chunk_count_{0};
  size_t received_chunk_count_{0};
  std::vector<bool> received_chunks_;
  std::vector<GaussianSnapshotPoint> points_;
};

}  // namespace gaussian_lic_tracking
