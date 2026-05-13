// SPDX-License-Identifier: GPL-3.0-or-later

#include <gaussian_lic_mapping/frame_data.hpp>

#include <algorithm>
#include <cmath>
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

bool has_valid_projection_intrinsics(const CameraIntrinsics & intrinsics)
{
  return std::isfinite(intrinsics.fx) && std::isfinite(intrinsics.fy) &&
    std::isfinite(intrinsics.cx) && std::isfinite(intrinsics.cy) &&
    intrinsics.fx > 0.0 && intrinsics.fy > 0.0;
}

bool project_to_image_pixel(
  const int width,
  const int height,
  const CameraIntrinsics & intrinsics,
  const Eigen::Vector3d & xyz_cam,
  int & u,
  int & v)
{
  if (width <= 0 || height <= 0 || !has_valid_projection_intrinsics(intrinsics) || xyz_cam.z() <= 0.0)
  {
    return false;
  }

  const double u_float = intrinsics.fx * xyz_cam.x() / xyz_cam.z() + intrinsics.cx;
  const double v_float = intrinsics.fy * xyz_cam.y() / xyz_cam.z() + intrinsics.cy;
  if (!std::isfinite(u_float) || !std::isfinite(v_float)) {
    return false;
  }

  u = static_cast<int>(std::floor(u_float));
  v = static_cast<int>(std::floor(v_float));
  return u >= 0 && v >= 0 && u < width && v < height;
}

bool sample_projected_image_color(
  const cv::Mat & image_rgb_float,
  const CameraIntrinsics & intrinsics,
  const Eigen::Vector3d & xyz_cam,
  Eigen::Vector3f & color_rgb,
  int * projected_u = nullptr,
  int * projected_v = nullptr)
{
  if (image_rgb_float.empty() || image_rgb_float.type() != CV_32FC3) {
    return false;
  }

  int u = 0;
  int v = 0;
  if (!project_to_image_pixel(image_rgb_float.cols, image_rgb_float.rows, intrinsics, xyz_cam, u, v)) {
    return false;
  }
  if (projected_u) {
    *projected_u = u;
  }
  if (projected_v) {
    *projected_v = v;
  }

  const cv::Vec3f & rgb = image_rgb_float.at<cv::Vec3f>(v, u);
  color_rgb.x() = std::clamp(rgb[0], 0.0F, 1.0F);
  color_rgb.y() = std::clamp(rgb[1], 0.0F, 1.0F);
  color_rgb.z() = std::clamp(rgb[2], 0.0F, 1.0F);
  return true;
}

float normalize_intensity_color(
  const double value,
  const sensor_msgs::msg::PointField & field)
{
  double normalized = value;
  switch (field.datatype) {
    case sensor_msgs::msg::PointField::UINT8:
      normalized = value / 255.0;
      break;
    case sensor_msgs::msg::PointField::UINT16:
      normalized = value / 65535.0;
      break;
    case sensor_msgs::msg::PointField::UINT32:
      normalized = value / 4294967295.0;
      break;
    default:
      if (value > 1.0) {
        normalized = value / 255.0;
      }
      break;
  }
  return static_cast<float>(std::clamp(normalized, 0.0, 1.0));
}

bool sample_intensity_color(
  const uint8_t * base,
  const sensor_msgs::msg::PointField * intensity_field,
  Eigen::Vector3f & color_rgb)
{
  if (!intensity_field) {
    return false;
  }
  const float intensity = normalize_intensity_color(
    read_numeric_field(base, *intensity_field), *intensity_field);
  color_rgb = Eigen::Vector3f::Constant(intensity);
  return true;
}

cv::Mat make_projected_depth_image(
  const std::vector<MapperPoint> & points,
  const int width,
  const int height,
  const CameraIntrinsics & intrinsics,
  const Eigen::Quaterniond & q_wc,
  const Eigen::Vector3d & t_wc)
{
  cv::Mat depth_m(height, width, CV_32FC1, cv::Scalar(0.0F));
  if (width <= 0 || height <= 0 || !has_valid_projection_intrinsics(intrinsics)) {
    return depth_m;
  }

  const Eigen::Matrix3d r_cw = q_wc.toRotationMatrix().transpose();
  for (const MapperPoint & point : points) {
    const Eigen::Vector3d xyz_world = point.xyz_world.cast<double>();
    const Eigen::Vector3d xyz_cam = r_cw * (xyz_world - t_wc);
    if (xyz_cam.z() <= 0.0) {
      continue;
    }

    const double u_float = intrinsics.fx * xyz_cam.x() / xyz_cam.z() + intrinsics.cx;
    const double v_float = intrinsics.fy * xyz_cam.y() / xyz_cam.z() + intrinsics.cy;
    if (!std::isfinite(u_float) || !std::isfinite(v_float)) {
      continue;
    }

    // Match scripts/frontend_raw_to_ros1_mapper_contract.py::make_projected_depth.
    // RGB sampling intentionally stays floor-based because the ROS1 converter uses floor there.
    const int u = static_cast<int>(std::nearbyint(u_float));
    const int v = static_cast<int>(std::nearbyint(v_float));
    if (u < 0 || v < 0 || u >= width || v >= height) {
      continue;
    }

    float & current_depth = depth_m.at<float>(v, u);
    const float candidate_depth = static_cast<float>(xyz_cam.z());
    if (current_depth <= 0.0F || candidate_depth < current_depth) {
      current_depth = candidate_depth;
    }
  }

  return depth_m;
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
  const Eigen::Quaterniond & q_w_pose,
  const Eigen::Vector3d & t_w_pose,
  const Eigen::Quaterniond & q_wc,
  const Eigen::Vector3d & t_wc,
  const cv::Mat & image_rgb_float,
  const CameraIntrinsics & intrinsics,
  const PointCloudCoordinates pointcloud_coordinates,
  const CameraExtrinsics & camera_extrinsics,
  const double max_depth_m,
  const bool require_projected_color,
  const bool zbuffer_projected_points,
  size_t & skipped_nonpositive_depth,
  size_t & skipped_max_depth,
  size_t & skipped_unprojected,
  size_t & skipped_occluded)
{
  skipped_nonpositive_depth = 0;
  skipped_max_depth = 0;
  skipped_unprojected = 0;
  skipped_occluded = 0;
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
  const auto * intensity_field = find_field(cloud, "intensity");
  if (!intensity_field) {
    intensity_field = find_field(cloud, "reflectivity");
  }
  const bool has_explicit_color = rgb_field || (r_field && g_field && b_field);
  const bool use_projected_zbuffer =
    zbuffer_projected_points && require_projected_color && !has_explicit_color && !image_rgb_float.empty() &&
    image_rgb_float.type() == CV_32FC3 && has_valid_projection_intrinsics(intrinsics);
  std::vector<int> nearest_point_by_pixel;
  std::vector<double> nearest_depth_by_pixel;
  if (use_projected_zbuffer) {
    const size_t pixel_count =
      static_cast<size_t>(image_rgb_float.cols) * static_cast<size_t>(image_rgb_float.rows);
    nearest_point_by_pixel.assign(pixel_count, -1);
    nearest_depth_by_pixel.assign(pixel_count, std::numeric_limits<double>::infinity());
  }

  const Eigen::Matrix3d r_cw = q_wc.toRotationMatrix().transpose();
  const Eigen::Vector3d t_cw = -r_cw * t_wc;
  const Eigen::Matrix3d r_w_pose = q_w_pose.toRotationMatrix();
  const Eigen::Quaterniond q_camera_pose = camera_extrinsics.q_pose_camera.inverse();

  for (size_t i = 0; i < point_count; ++i) {
    const uint8_t * base = cloud.data.data() + i * cloud.point_step;

    const Eigen::Vector3d xyz_input{
      read_numeric_field(base, *x_field),
      read_numeric_field(base, *y_field),
      read_numeric_field(base, *z_field)};

    Eigen::Vector3d xyz_world = xyz_input;
    Eigen::Vector3d xyz_cam = xyz_input;
    if (pointcloud_coordinates == PointCloudCoordinates::kSensor) {
      xyz_world = r_w_pose * xyz_input + t_w_pose;
      xyz_cam = q_camera_pose * (xyz_input - camera_extrinsics.p_pose_camera);
    } else {
      xyz_cam = r_cw * xyz_world + t_cw;
    }
    if (xyz_cam.z() <= 0.0) {
      ++skipped_nonpositive_depth;
      continue;
    }
    if (max_depth_m > 0.0 && xyz_cam.z() > max_depth_m) {
      ++skipped_max_depth;
      continue;
    }

    Eigen::Vector3f color_rgb{1.0F, 1.0F, 1.0F};
    int projected_u = -1;
    int projected_v = -1;
    if (rgb_field) {
      const uint32_t rgb = read_rgb_bits(base, *rgb_field);
      color_rgb.x() = static_cast<float>((rgb >> 16U) & 0xFFU) / 255.0F;
      color_rgb.y() = static_cast<float>((rgb >> 8U) & 0xFFU) / 255.0F;
      color_rgb.z() = static_cast<float>(rgb & 0xFFU) / 255.0F;
    } else if (r_field && g_field && b_field) {
      color_rgb.x() = static_cast<float>(read_numeric_field(base, *r_field)) / 255.0F;
      color_rgb.y() = static_cast<float>(read_numeric_field(base, *g_field)) / 255.0F;
      color_rgb.z() = static_cast<float>(read_numeric_field(base, *b_field)) / 255.0F;
    } else if (!sample_projected_image_color(
        image_rgb_float, intrinsics, xyz_cam, color_rgb, &projected_u, &projected_v))
    {
      if (require_projected_color) {
        ++skipped_unprojected;
        continue;
      }
      (void)sample_intensity_color(base, intensity_field, color_rgb);
    }

    MapperPoint point{
      xyz_world.cast<float>(),
      color_rgb,
      static_cast<float>(xyz_cam.z())};
    if (use_projected_zbuffer) {
      if (projected_u < 0 || projected_v < 0) {
        ++skipped_unprojected;
        continue;
      }
      const size_t pixel_index =
        static_cast<size_t>(projected_v) * static_cast<size_t>(image_rgb_float.cols) +
        static_cast<size_t>(projected_u);
      if (pixel_index >= nearest_point_by_pixel.size()) {
        ++skipped_unprojected;
        continue;
      }
      const int previous_index = nearest_point_by_pixel[pixel_index];
      if (previous_index >= 0 && xyz_cam.z() >= nearest_depth_by_pixel[pixel_index]) {
        ++skipped_occluded;
        continue;
      }
      if (previous_index >= 0) {
        points[static_cast<size_t>(previous_index)] = point;
        nearest_depth_by_pixel[pixel_index] = xyz_cam.z();
        ++skipped_occluded;
        continue;
      }
      nearest_point_by_pixel[pixel_index] = static_cast<int>(points.size());
      nearest_depth_by_pixel[pixel_index] = xyz_cam.z();
    }
    points.push_back(point);
  }

  return points;
}

}  // namespace

MapperFrameData convert_aligned_frame(
  const AlignedRosFrame & frame,
  const uint64_t frame_index,
  const int select_every_k_frame,
  const CameraIntrinsics & intrinsics,
  const PointCloudCoordinates pointcloud_coordinates,
  const CameraExtrinsics & camera_extrinsics,
  const double max_depth_m,
  const bool require_projected_color,
  const bool zbuffer_projected_points)
{
  if (!frame.pointcloud || !frame.pose || !frame.image) {
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
  const Eigen::Quaterniond q_w_pose = out.q_wc;
  const Eigen::Vector3d t_w_pose = out.t_wc;
  const Eigen::Quaterniond q_pose_camera = camera_extrinsics.q_pose_camera.normalized();
  out.q_wc = (q_w_pose * q_pose_camera).normalized();
  out.t_wc = t_w_pose + q_w_pose * camera_extrinsics.p_pose_camera;
  out.r_wc = out.q_wc.toRotationMatrix();

  out.points = convert_pointcloud(
    *frame.pointcloud, q_w_pose, t_w_pose, out.q_wc, out.t_wc, out.image_rgb_float, intrinsics,
    pointcloud_coordinates, camera_extrinsics, max_depth_m, require_projected_color,
    zbuffer_projected_points,
    out.skipped_points_nonpositive_depth, out.skipped_points_max_depth,
    out.skipped_points_unprojected, out.skipped_points_occluded);
  if (frame.depth) {
    out.depth_m_float = convert_depth_to_float_m(*frame.depth);
  } else {
    out.depth_m_float = make_projected_depth_image(
      out.points, out.width, out.height, intrinsics, out.q_wc, out.t_wc);
  }

  return out;
}

}  // namespace gaussian_lic_mapping
