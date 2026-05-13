// SPDX-License-Identifier: GPL-3.0-or-later

#include <gaussian_lic_mapping/mapper_dataset.hpp>

#include <algorithm>
#include <iomanip>
#include <sstream>

namespace gaussian_lic_mapping
{

const CameraFrameRecord & MapperDataset::add_frame(
  MapperFrameData && frame, const size_t test_frame_stride)
{
  CameraFrameRecord record;
  record.stamp = frame.stamp;
  record.frame_index = frame.frame_index;
  record.is_keyframe = frame.is_keyframe;
  record.image_name = make_image_name(frame.is_keyframe, frame.frame_index);
  record.width = frame.width;
  record.height = frame.height;
  record.image_rgb_float = std::move(frame.image_rgb_float);
  record.depth_m_float = std::move(frame.depth_m_float);
  record.r_wc = frame.r_wc;
  record.t_wc = frame.t_wc;
  const bool is_keyframe = record.is_keyframe;

  pending_points_world_.reserve(pending_points_world_.size() + frame.points.size());
  pending_colors_rgb_.reserve(pending_colors_rgb_.size() + frame.points.size());
  pending_depths_m_.reserve(pending_depths_m_.size() + frame.points.size());
  map_points_world_.reserve(map_points_world_.size() + frame.points.size());
  map_colors_rgb_.reserve(map_colors_rgb_.size() + frame.points.size());

  for (const MapperPoint & point : frame.points) {
    pending_points_world_.push_back(point.xyz_world);
    pending_colors_rgb_.push_back(point.color_rgb);
    pending_depths_m_.push_back(point.depth_m);
    map_points_world_.push_back(point.xyz_world);
    map_colors_rgb_.push_back(point.color_rgb);
  }

  total_point_count_ += frame.points.size();
  skipped_nonpositive_depth_count_ += frame.skipped_points_nonpositive_depth;
  skipped_max_depth_count_ += frame.skipped_points_max_depth;
  skipped_unprojected_count_ += frame.skipped_points_unprojected;
  skipped_occluded_count_ += frame.skipped_points_occluded;

  const size_t stride = std::max<size_t>(test_frame_stride, 1U);
  if (is_keyframe) {
    train_frames_.push_back(std::move(record));
  } else if (stride > 1U && (record.frame_index % stride) != 0U) {
    skipped_test_frame_ = std::move(record);
  } else {
    test_frames_.push_back(std::move(record));
  }
  ++all_frame_count_;
  if (is_keyframe) {
    return train_frames_.back();
  }
  if (stride > 1U && (skipped_test_frame_.frame_index % stride) != 0U) {
    return skipped_test_frame_;
  }
  return test_frames_.back();
}

void MapperDataset::clear_pending_points()
{
  pending_points_world_.clear();
  pending_colors_rgb_.clear();
  pending_depths_m_.clear();
}

void MapperDataset::trim_map_points(const size_t max_points)
{
  if (max_points == 0 || map_points_world_.size() <= max_points) {
    return;
  }

  const size_t overflow = map_points_world_.size() - max_points;
  const auto erase_count =
    static_cast<std::vector<Eigen::Vector3f>::difference_type>(overflow);
  map_points_world_.erase(map_points_world_.begin(), map_points_world_.begin() + erase_count);
  map_colors_rgb_.erase(map_colors_rgb_.begin(), map_colors_rgb_.begin() + erase_count);
}

std::string MapperDataset::make_image_name(const bool is_keyframe, const uint64_t frame_index)
{
  std::ostringstream out;
  out << (is_keyframe ? "train_" : "test_")
      << std::setw(4) << std::setfill('0') << frame_index
      << ".jpg";
  return out.str();
}

}  // namespace gaussian_lic_mapping
