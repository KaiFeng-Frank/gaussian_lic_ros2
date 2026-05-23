// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <Eigen/Core>
#include <opencv2/core.hpp>

#include <gaussian_lic_mapping/frame_data.hpp>

namespace gaussian_lic_mapping
{

struct CameraFrameRecord
{
  builtin_interfaces::msg::Time stamp;
  builtin_interfaces::msg::Time pointcloud_stamp;
  builtin_interfaces::msg::Time pose_stamp;
  builtin_interfaces::msg::Time image_stamp;
  builtin_interfaces::msg::Time depth_stamp;
  bool has_depth_stamp{false};
  uint64_t frame_index{0};
  bool is_keyframe{false};
  std::string image_name;
  int width{0};
  int height{0};
  cv::Mat image_rgb_float;
  cv::Mat depth_m_float;
  Eigen::Matrix3d r_wc{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d t_wc{Eigen::Vector3d::Zero()};
};

class MapperDataset
{
public:
  const CameraFrameRecord & add_frame(MapperFrameData && frame, size_t test_frame_stride = 1);
  void clear_pending_points();
  void trim_map_points(size_t max_points);

  size_t all_frame_count() const { return all_frame_count_; }
  size_t train_frame_count() const { return train_frames_.size(); }
  size_t test_frame_count() const { return test_frames_.size(); }
  size_t pending_point_count() const { return pending_points_world_.size(); }
  size_t map_point_count() const { return map_points_world_.size(); }
  size_t total_point_count() const { return total_point_count_; }
  size_t skipped_nonpositive_depth_count() const { return skipped_nonpositive_depth_count_; }
  size_t skipped_max_depth_count() const { return skipped_max_depth_count_; }
  size_t skipped_unprojected_count() const { return skipped_unprojected_count_; }
  size_t skipped_occluded_count() const { return skipped_occluded_count_; }

  const std::vector<CameraFrameRecord> & train_frames() const { return train_frames_; }
  const std::vector<CameraFrameRecord> & test_frames() const { return test_frames_; }
  const std::vector<Eigen::Vector3f> & pending_points_world() const { return pending_points_world_; }
  const std::vector<Eigen::Vector3f> & pending_colors_rgb() const { return pending_colors_rgb_; }
  const std::vector<float> & pending_depths_m() const { return pending_depths_m_; }
  const std::vector<Eigen::Vector3f> & map_points_world() const { return map_points_world_; }
  const std::vector<Eigen::Vector3f> & map_colors_rgb() const { return map_colors_rgb_; }

private:
  static std::string make_image_name(bool is_keyframe, uint64_t frame_index);

  size_t all_frame_count_{0};
  size_t total_point_count_{0};
  size_t skipped_nonpositive_depth_count_{0};
  size_t skipped_max_depth_count_{0};
  size_t skipped_unprojected_count_{0};
  size_t skipped_occluded_count_{0};
  std::vector<CameraFrameRecord> train_frames_;
  std::vector<CameraFrameRecord> test_frames_;
  CameraFrameRecord skipped_test_frame_;
  std::vector<Eigen::Vector3f> pending_points_world_;
  std::vector<Eigen::Vector3f> pending_colors_rgb_;
  std::vector<float> pending_depths_m_;
  std::vector<Eigen::Vector3f> map_points_world_;
  std::vector<Eigen::Vector3f> map_colors_rgb_;
};

}  // namespace gaussian_lic_mapping
