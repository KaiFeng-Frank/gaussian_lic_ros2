// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <opencv2/core.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace gaussian_lic_mapping
{

struct AlignedRosFrame
{
  builtin_interfaces::msg::Time stamp;
  bool has_stamp{false};
  sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud;
  geometry_msgs::msg::PoseStamped::ConstSharedPtr pose;
  sensor_msgs::msg::Image::ConstSharedPtr image;
  sensor_msgs::msg::Image::ConstSharedPtr depth;
};

struct MapperPoint
{
  Eigen::Vector3f xyz_world{Eigen::Vector3f::Zero()};
  Eigen::Vector3f color_rgb{Eigen::Vector3f::Ones()};
  float depth_m{0.0F};
};

struct CameraIntrinsics
{
  double fx{1.0};
  double fy{1.0};
  double cx{0.5};
  double cy{0.5};
};

struct CameraExtrinsics
{
  Eigen::Quaterniond q_pose_camera{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d p_pose_camera{Eigen::Vector3d::Zero()};
};

enum class PointCloudCoordinates
{
  kWorld,
  kSensor,
};

struct MapperFrameData
{
  builtin_interfaces::msg::Time stamp;
  builtin_interfaces::msg::Time pointcloud_stamp;
  builtin_interfaces::msg::Time pose_stamp;
  builtin_interfaces::msg::Time image_stamp;
  builtin_interfaces::msg::Time depth_stamp;
  bool has_depth_stamp{false};
  uint64_t frame_index{0};
  bool is_keyframe{false};
  int width{0};
  int height{0};
  cv::Mat image_rgb_float;
  cv::Mat depth_m_float;
  Eigen::Quaterniond q_wc{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d t_wc{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d r_wc{Eigen::Matrix3d::Identity()};
  std::vector<MapperPoint> points;
  size_t skipped_points_nonpositive_depth{0};
  size_t skipped_points_max_depth{0};
  size_t skipped_points_unprojected{0};
  size_t skipped_points_occluded{0};
};

MapperFrameData convert_aligned_frame(
  const AlignedRosFrame & frame,
  uint64_t frame_index,
  int select_every_k_frame,
  const CameraIntrinsics & intrinsics = CameraIntrinsics{},
  PointCloudCoordinates pointcloud_coordinates = PointCloudCoordinates::kWorld,
  const CameraExtrinsics & camera_extrinsics = CameraExtrinsics{},
  double max_depth_m = 0.0,
  bool require_projected_color = false,
  bool zbuffer_projected_points = false);

}  // namespace gaussian_lic_mapping
