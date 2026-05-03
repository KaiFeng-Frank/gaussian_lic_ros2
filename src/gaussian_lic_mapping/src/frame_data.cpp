// SPDX-License-Identifier: GPL-3.0-or-later

#include <gaussian_lic_mapping/frame_data.hpp>

#include <algorithm>
#include <cstring>
#include <stdexcept>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/point_field.hpp>

namespace gaussian_lic_mapping
{
namespace
{

const sensor_msgs::msg::PointField * find_field(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const char * name)
{
  const auto it = std::find_if(
    cloud.fields.begin(), cloud.fields.end(),
    [name](const sensor_msgs::msg::PointField & field) {
      return field.name == name;
    });
  return it == cloud.fields.end() ? nullptr : &(*it);
}

template<typename T>
T read_value(const uint8_t * ptr)
{
  T value{};
  std::memcpy(&value, ptr, sizeof(T));
  return value;
}

double read_numeric_field(const uint8_t * base, const sensor_msgs::msg::PointField & field)
{
  const uint8_t * ptr = base + field.offset;
  switch (field.datatype) {
    case sensor_msgs::msg::PointField::INT8:
      return static_cast<double>(read_value<int8_t>(ptr));
    case sensor_msgs::msg::PointField::UINT8:
      return static_cast<double>(read_value<uint8_t>(ptr));
    case sensor_msgs::msg::PointField::INT16:
      return static_cast<double>(read_value<int16_t>(ptr));
    case sensor_msgs::msg::PointField::UINT16:
      return static_cast<double>(read_value<uint16_t>(ptr));
    case sensor_msgs::msg::PointField::INT32:
      return static_cast<double>(read_value<int32_t>(ptr));
    case sensor_msgs::msg::PointField::UINT32:
      return static_cast<double>(read_value<uint32_t>(ptr));
    case sensor_msgs::msg::PointField::FLOAT32:
      return static_cast<double>(read_value<float>(ptr));
    case sensor_msgs::msg::PointField::FLOAT64:
      return read_value<double>(ptr);
    default:
      throw std::runtime_error("unsupported PointCloud2 field datatype");
  }
}

uint32_t read_rgb_bits(const uint8_t * base, const sensor_msgs::msg::PointField & field)
{
  const uint8_t * ptr = base + field.offset;
  if (field.datatype == sensor_msgs::msg::PointField::FLOAT32) {
    const float packed = read_value<float>(ptr);
    uint32_t bits = 0;
    std::memcpy(&bits, &packed, sizeof(bits));
    return bits;
  }
  if (field.datatype == sensor_msgs::msg::PointField::UINT32 ||
    field.datatype == sensor_msgs::msg::PointField::INT32)
  {
    return read_value<uint32_t>(ptr);
  }
  throw std::runtime_error("rgb/rgba PointCloud2 field must be FLOAT32 or UINT32");
}

cv::Mat convert_image_to_rgb_float(const sensor_msgs::msg::Image & image_msg)
{
  namespace enc = sensor_msgs::image_encodings;

  cv_bridge::CvImagePtr cv_ptr;
  if (image_msg.encoding == enc::BGR8) {
    cv_ptr = cv_bridge::toCvCopy(image_msg, enc::BGR8);
    cv::Mat rgb;
    cv::cvtColor(cv_ptr->image, rgb, cv::COLOR_BGR2RGB);
    rgb.convertTo(rgb, CV_32FC3, 1.0 / 255.0);
    return rgb;
  }

  if (image_msg.encoding == enc::RGB8) {
    cv_ptr = cv_bridge::toCvCopy(image_msg, enc::RGB8);
    cv::Mat rgb;
    cv_ptr->image.convertTo(rgb, CV_32FC3, 1.0 / 255.0);
    return rgb;
  }

  if (image_msg.encoding == enc::MONO8) {
    cv_ptr = cv_bridge::toCvCopy(image_msg, enc::MONO8);
    cv::Mat rgb_u8;
    cv::cvtColor(cv_ptr->image, rgb_u8, cv::COLOR_GRAY2RGB);
    cv::Mat rgb;
    rgb_u8.convertTo(rgb, CV_32FC3, 1.0 / 255.0);
    return rgb;
  }

  throw std::runtime_error("unsupported image encoding for Gaussian-LIC mapper: " + image_msg.encoding);
}

cv::Mat convert_depth_to_float_m(const sensor_msgs::msg::Image & depth_msg)
{
  namespace enc = sensor_msgs::image_encodings;

  if (depth_msg.encoding == enc::TYPE_32FC1) {
    return cv_bridge::toCvCopy(depth_msg, enc::TYPE_32FC1)->image;
  }

  if (depth_msg.encoding == enc::TYPE_16UC1 || depth_msg.encoding == enc::MONO16) {
    cv::Mat depth_m;
    cv_bridge::toCvCopy(depth_msg, depth_msg.encoding)->image.convertTo(depth_m, CV_32FC1, 0.001);
    return depth_m;
  }

  throw std::runtime_error("unsupported depth encoding for Gaussian-LIC mapper: " + depth_msg.encoding);
}

std::vector<MapperPoint> convert_pointcloud(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const Eigen::Quaterniond & q_wc,
  const Eigen::Vector3d & t_wc,
  size_t & skipped_nonpositive_depth)
{
  skipped_nonpositive_depth = 0;
  const size_t point_count = static_cast<size_t>(cloud.width) * static_cast<size_t>(cloud.height);
  std::vector<MapperPoint> points;
  points.reserve(point_count);

  if (point_count == 0) {
    return points;
  }

  const auto * x_field = find_field(cloud, "x");
  const auto * y_field = find_field(cloud, "y");
  const auto * z_field = find_field(cloud, "z");
  if (!x_field || !y_field || !z_field) {
    throw std::runtime_error("PointCloud2 must contain x/y/z fields");
  }

  const auto * rgb_field = find_field(cloud, "rgb");
  if (!rgb_field) {
    rgb_field = find_field(cloud, "rgba");
  }
  const auto * r_field = find_field(cloud, "r");
  const auto * g_field = find_field(cloud, "g");
  const auto * b_field = find_field(cloud, "b");

  const Eigen::Matrix3d r_cw = q_wc.toRotationMatrix().transpose();
  const Eigen::Vector3d t_cw = -r_cw * t_wc;

  for (size_t i = 0; i < point_count; ++i) {
    const uint8_t * base = cloud.data.data() + i * cloud.point_step;

    const Eigen::Vector3d xyz_world{
      read_numeric_field(base, *x_field),
      read_numeric_field(base, *y_field),
      read_numeric_field(base, *z_field)};

    const Eigen::Vector3d xyz_cam = r_cw * xyz_world + t_cw;
    if (xyz_cam.z() <= 0.0) {
      ++skipped_nonpositive_depth;
      continue;
    }

    Eigen::Vector3f color_rgb{1.0F, 1.0F, 1.0F};
    if (rgb_field) {
      const uint32_t rgb = read_rgb_bits(base, *rgb_field);
      color_rgb.x() = static_cast<float>((rgb >> 16U) & 0xFFU) / 255.0F;
      color_rgb.y() = static_cast<float>((rgb >> 8U) & 0xFFU) / 255.0F;
      color_rgb.z() = static_cast<float>(rgb & 0xFFU) / 255.0F;
    } else if (r_field && g_field && b_field) {
      color_rgb.x() = static_cast<float>(read_numeric_field(base, *r_field)) / 255.0F;
      color_rgb.y() = static_cast<float>(read_numeric_field(base, *g_field)) / 255.0F;
      color_rgb.z() = static_cast<float>(read_numeric_field(base, *b_field)) / 255.0F;
    }

    points.push_back(MapperPoint{
      xyz_world.cast<float>(),
      color_rgb,
      static_cast<float>(xyz_cam.z())});
  }

  return points;
}

}  // namespace

MapperFrameData convert_aligned_frame(
  const AlignedRosFrame & frame,
  const uint64_t frame_index,
  const int select_every_k_frame)
{
  if (!frame.pointcloud || !frame.pose || !frame.image || !frame.depth) {
    throw std::runtime_error("cannot convert incomplete aligned ROS frame");
  }
  if (select_every_k_frame <= 0) {
    throw std::runtime_error("select_every_k_frame must be positive");
  }

  MapperFrameData out;
  out.stamp = frame.pointcloud->header.stamp;
  out.frame_index = frame_index;
  out.is_keyframe = ((frame_index + 1U) % static_cast<uint64_t>(select_every_k_frame)) == 0U;
  out.image_rgb_float = convert_image_to_rgb_float(*frame.image);
  out.depth_m_float = convert_depth_to_float_m(*frame.depth);
  out.width = out.image_rgb_float.cols;
  out.height = out.image_rgb_float.rows;

  out.q_wc = Eigen::Quaterniond{
    frame.pose->pose.orientation.w,
    frame.pose->pose.orientation.x,
    frame.pose->pose.orientation.y,
    frame.pose->pose.orientation.z};
  out.q_wc.normalize();
  out.t_wc = Eigen::Vector3d{
    frame.pose->pose.position.x,
    frame.pose->pose.position.y,
    frame.pose->pose.position.z};
  out.r_wc = out.q_wc.toRotationMatrix();

  out.points = convert_pointcloud(
    *frame.pointcloud, out.q_wc, out.t_wc, out.skipped_points_nonpositive_depth);

  return out;
}

}  // namespace gaussian_lic_mapping
