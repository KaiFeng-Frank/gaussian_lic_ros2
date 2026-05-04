// SPDX-License-Identifier: GPL-3.0-or-later

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace
{

std::string lowercase(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return value;
}

double stamp_to_sec(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1.0e-9;
}

struct UnitQuaternion
{
  double w{1.0};
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

UnitQuaternion multiply(const UnitQuaternion & lhs, const UnitQuaternion & rhs)
{
  UnitQuaternion out;
  out.w = lhs.w * rhs.w - lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z;
  out.x = lhs.w * rhs.x + lhs.x * rhs.w + lhs.y * rhs.z - lhs.z * rhs.y;
  out.y = lhs.w * rhs.y - lhs.x * rhs.z + lhs.y * rhs.w + lhs.z * rhs.x;
  out.z = lhs.w * rhs.z + lhs.x * rhs.y - lhs.y * rhs.x + lhs.z * rhs.w;
  return out;
}

void normalize(UnitQuaternion & q)
{
  const double norm = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  if (!std::isfinite(norm) || norm <= std::numeric_limits<double>::epsilon()) {
    q = UnitQuaternion{};
    return;
  }
  q.w /= norm;
  q.x /= norm;
  q.y /= norm;
  q.z /= norm;
}

UnitQuaternion delta_quaternion(
  const double wx, const double wy, const double wz, const double dt)
{
  const double omega = std::sqrt(wx * wx + wy * wy + wz * wz);
  if (!std::isfinite(omega) || omega <= std::numeric_limits<double>::epsilon() || dt <= 0.0) {
    return UnitQuaternion{};
  }

  const double half_angle = 0.5 * omega * dt;
  const double scale = std::sin(half_angle) / omega;
  UnitQuaternion dq;
  dq.w = std::cos(half_angle);
  dq.x = wx * scale;
  dq.y = wy * scale;
  dq.z = wz * scale;
  normalize(dq);
  return dq;
}

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

template<typename T>
void write_value(uint8_t * ptr, const T value)
{
  std::memcpy(ptr, &value, sizeof(T));
}

double read_float_field(const uint8_t * base, const sensor_msgs::msg::PointField & field)
{
  const uint8_t * ptr = base + field.offset;
  if (field.datatype == sensor_msgs::msg::PointField::FLOAT32) {
    return static_cast<double>(read_value<float>(ptr));
  }
  if (field.datatype == sensor_msgs::msg::PointField::FLOAT64) {
    return read_value<double>(ptr);
  }
  throw std::runtime_error("pointcloud transform requires FLOAT32 or FLOAT64 x/y/z fields");
}

void write_float_field(
  uint8_t * base,
  const sensor_msgs::msg::PointField & field,
  const double value)
{
  uint8_t * ptr = base + field.offset;
  if (field.datatype == sensor_msgs::msg::PointField::FLOAT32) {
    write_value<float>(ptr, static_cast<float>(value));
    return;
  }
  if (field.datatype == sensor_msgs::msg::PointField::FLOAT64) {
    write_value<double>(ptr, value);
    return;
  }
  throw std::runtime_error("pointcloud transform requires FLOAT32 or FLOAT64 x/y/z fields");
}

}  // namespace

class Lic2ContractAdapter final : public rclcpp::Node
{
public:
  Lic2ContractAdapter()
  : rclcpp::Node("lic2_contract_adapter")
  {
    raw_image_topic_ = declare_parameter<std::string>("raw_image_topic", "/camera/image");
    raw_camera_info_topic_ =
      declare_parameter<std::string>("raw_camera_info_topic", "/camera/camera_info");
    raw_depth_topic_ = declare_parameter<std::string>("raw_depth_topic", "/camera/depth");
    raw_pointcloud_topic_ = declare_parameter<std::string>("raw_pointcloud_topic", "/livox/lidar");
    raw_imu_topic_ = declare_parameter<std::string>("raw_imu_topic", "/imu");
    pose_stamped_topic_ =
      declare_parameter<std::string>("pose_stamped_topic", "/gaussian_lic/frontend/pose");
    raw_odometry_topic_ =
      declare_parameter<std::string>("raw_odometry_topic", "/gaussian_lic/frontend/input_odometry");

    image_topic_ = declare_parameter<std::string>("image_topic", "/image_for_gs");
    camera_info_topic_ =
      declare_parameter<std::string>("camera_info_topic", "/camera_info_for_gs");
    depth_topic_ = declare_parameter<std::string>("depth_topic", "/depth_for_gs");
    pointcloud_topic_ = declare_parameter<std::string>("pointcloud_topic", "/points_for_gs");
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/pose_for_gs");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu_for_gs");
    frontend_odometry_topic_ =
      declare_parameter<std::string>("frontend_odometry_topic", "/gaussian_lic/frontend/odometry");
    frontend_path_topic_ =
      declare_parameter<std::string>("frontend_path_topic", "/gaussian_lic/frontend/path");

    enable_depth_passthrough_ = declare_parameter<bool>("enable_depth_passthrough", true);
    enable_imu_passthrough_ = declare_parameter<bool>("enable_imu_passthrough", true);
    identity_pose_fallback_ = declare_parameter<bool>("identity_pose_fallback", false);
    imu_pose_fallback_ = declare_parameter<bool>("imu_pose_fallback", false);
    imu_integration_max_dt_sec_ = declare_parameter<double>("imu_integration_max_dt_sec", 0.05);
    pointcloud_transform_profile_ =
      lowercase(declare_parameter<std::string>("pointcloud_transform_profile", "identity"));
    transform_pointcloud_to_camera_frame_ =
      declare_parameter<bool>("transform_pointcloud_to_camera_frame", false);
    pointcloud_transform_target_frame_ =
      declare_parameter<std::string>("pointcloud_transform_target_frame", "camera");
    configure_pointcloud_transform();
    world_frame_ = declare_parameter<std::string>("world_frame", "map");
    frontend_child_frame_ = declare_parameter<std::string>("frontend_child_frame", "base_link");
    publish_tf_ = declare_parameter<bool>("publish_tf", false);
    max_path_length_ = declare_parameter<int>("max_path_length", 5000);
    report_period_sec_ = declare_parameter<double>("report_period_sec", 2.0);
    if (max_path_length_ <= 0) {
      RCLCPP_WARN(get_logger(), "max_path_length must be positive; using 1");
      max_path_length_ = 1;
    }

    const auto qos = make_sensor_qos();
    const rclcpp::QoS path_qos = rclcpp::QoS(1).reliable().transient_local();

    image_pub_ = create_publisher<sensor_msgs::msg::Image>(image_topic_, qos);
    camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic_, qos);
    depth_pub_ = create_publisher<sensor_msgs::msg::Image>(depth_topic_, qos);
    pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud_topic_, qos);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_, qos);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_, qos);
    frontend_odometry_pub_ =
      create_publisher<nav_msgs::msg::Odometry>(frontend_odometry_topic_, qos);
    frontend_path_pub_ = create_publisher<nav_msgs::msg::Path>(frontend_path_topic_, path_qos);
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      raw_image_topic_, qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
        image_pub_->publish(*msg);
        ++image_count_;
      });
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      raw_camera_info_topic_, qos,
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
        camera_info_pub_->publish(*msg);
        ++camera_info_count_;
      });
    pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      raw_pointcloud_topic_, qos,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        if (transform_pointcloud_to_camera_frame_) {
          try {
            pointcloud_pub_->publish(transform_pointcloud(*msg));
            ++transformed_pointcloud_count_;
          } catch (const std::exception & ex) {
            ++pointcloud_transform_error_count_;
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 2000,
              "failed to transform pointcloud; forwarding original cloud: %s", ex.what());
            pointcloud_pub_->publish(*msg);
          }
        } else {
          pointcloud_pub_->publish(*msg);
        }
        ++pointcloud_count_;
        if (imu_pose_fallback_) {
          publish_imu_pose_fallback(msg->header.stamp);
        } else if (identity_pose_fallback_) {
          publish_identity_pose(msg->header.stamp);
        }
      });
    pose_stamped_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_stamped_topic_, qos,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
        publish_frontend_pose(*msg, false, true);
      });
    odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      raw_odometry_topic_, qos,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = msg->header;
        pose.pose = msg->pose.pose;
        publish_frontend_pose(pose, true, false);
        ++odometry_count_;
      });

    if (enable_depth_passthrough_) {
      depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
        raw_depth_topic_, qos,
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
          depth_pub_->publish(*msg);
          ++depth_count_;
        });
    }
    if (enable_imu_passthrough_) {
      imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        raw_imu_topic_, qos,
        [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
          imu_pub_->publish(*msg);
          ++imu_count_;
          if (imu_pose_fallback_) {
            integrate_imu_orientation(*msg);
          }
        });
    }

    if (report_period_sec_ > 0.0) {
      report_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(report_period_sec_)),
        [this]() { report_counts(); });
    }

    RCLCPP_INFO(
      get_logger(),
      "LIC2 contract adapter: raw image=%s pointcloud=%s pose=%s raw_odom=%s -> mapper "
      "image=%s pointcloud=%s pose=%s frontend_odom=%s frontend_path=%s "
      "fallback(identity=%s imu=%s) pointcloud_transform=%s profile=%s",
      raw_image_topic_.c_str(), raw_pointcloud_topic_.c_str(), pose_stamped_topic_.c_str(),
      raw_odometry_topic_.c_str(), image_topic_.c_str(), pointcloud_topic_.c_str(),
      pose_topic_.c_str(), frontend_odometry_topic_.c_str(), frontend_path_topic_.c_str(),
      identity_pose_fallback_ ? "true" : "false", imu_pose_fallback_ ? "true" : "false",
      transform_pointcloud_to_camera_frame_ ? "true" : "false",
      pointcloud_transform_profile_.c_str());
  }

private:
  void configure_pointcloud_transform()
  {
    pointcloud_transform_rotation_ = declare_parameter<std::vector<double>>(
      "pointcloud_transform_rotation",
      std::vector<double>{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
    pointcloud_transform_translation_ = declare_parameter<std::vector<double>>(
      "pointcloud_transform_translation",
      std::vector<double>{0.0, 0.0, 0.0});

    if (pointcloud_transform_profile_ == "fastlivo2" ||
      pointcloud_transform_profile_ == "fastlivo2cameralidar")
    {
      transform_pointcloud_to_camera_frame_ = true;
      pointcloud_transform_rotation_ = {
        0.006101930, -0.999863000, -0.015417200,
        -0.006154490, 0.015379600, -0.999863000,
        0.999962000, 0.006195980, -0.006059800};
      pointcloud_transform_translation_ = {0.019438453, 0.104689079, -0.025195139};
    }

    if (pointcloud_transform_rotation_.size() != 9U) {
      throw std::runtime_error("pointcloud_transform_rotation must contain 9 row-major values");
    }
    if (pointcloud_transform_translation_.size() != 3U) {
      throw std::runtime_error("pointcloud_transform_translation must contain 3 values");
    }
  }

  rclcpp::QoS make_sensor_qos()
  {
    int depth = declare_parameter<int>("sensor_qos_depth", 5);
    if (depth <= 0) {
      RCLCPP_WARN(get_logger(), "sensor_qos_depth must be positive; using 1");
      depth = 1;
    }

    const auto history = lowercase(declare_parameter<std::string>("sensor_qos_history", "keep_last"));
    const auto reliability =
      lowercase(declare_parameter<std::string>("sensor_qos_reliability", "best_effort"));

    rclcpp::QoS qos(static_cast<size_t>(depth));
    if (history == "keep_all") {
      qos.keep_all();
    } else if (history != "keep_last") {
      RCLCPP_WARN(get_logger(), "unknown sensor_qos_history '%s'; using keep_last", history.c_str());
    }

    if (reliability == "reliable") {
      qos.reliable();
    } else {
      if (reliability != "best_effort") {
        RCLCPP_WARN(
          get_logger(), "unknown sensor_qos_reliability '%s'; using best_effort",
          reliability.c_str());
      }
      qos.best_effort();
    }

    qos.durability_volatile();
    return qos;
  }

  sensor_msgs::msg::PointCloud2 transform_pointcloud(const sensor_msgs::msg::PointCloud2 & cloud)
  {
    sensor_msgs::msg::PointCloud2 out = cloud;
    if (!pointcloud_transform_target_frame_.empty()) {
      out.header.frame_id = pointcloud_transform_target_frame_;
    }

    const auto * x_field = find_field(out, "x");
    const auto * y_field = find_field(out, "y");
    const auto * z_field = find_field(out, "z");
    if (!x_field || !y_field || !z_field) {
      throw std::runtime_error("PointCloud2 must contain x/y/z fields");
    }

    const size_t point_count = static_cast<size_t>(out.width) * static_cast<size_t>(out.height);
    for (size_t i = 0; i < point_count; ++i) {
      uint8_t * base = out.data.data() + i * out.point_step;
      const double x = read_float_field(base, *x_field);
      const double y = read_float_field(base, *y_field);
      const double z = read_float_field(base, *z_field);
      const auto & r = pointcloud_transform_rotation_;
      const auto & t = pointcloud_transform_translation_;
      const double tx = r[0] * x + r[1] * y + r[2] * z + t[0];
      const double ty = r[3] * x + r[4] * y + r[5] * z + t[1];
      const double tz = r[6] * x + r[7] * y + r[8] * z + t[2];
      write_float_field(base, *x_field, tx);
      write_float_field(base, *y_field, ty);
      write_float_field(base, *z_field, tz);
    }
    return out;
  }

  void publish_identity_pose(const builtin_interfaces::msg::Time & stamp)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = world_frame_;
    pose.pose.orientation.w = 1.0;
    publish_frontend_pose(pose, false, false);
    ++identity_pose_count_;
  }

  void publish_imu_pose_fallback(const builtin_interfaces::msg::Time & stamp)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = world_frame_;
    pose.pose.orientation.w = imu_orientation_.w;
    pose.pose.orientation.x = imu_orientation_.x;
    pose.pose.orientation.y = imu_orientation_.y;
    pose.pose.orientation.z = imu_orientation_.z;
    publish_frontend_pose(pose, false, false);
    ++imu_pose_fallback_count_;
  }

  void integrate_imu_orientation(const sensor_msgs::msg::Imu & imu)
  {
    const double stamp_sec = stamp_to_sec(imu.header.stamp);
    if (!have_last_imu_stamp_) {
      last_imu_stamp_sec_ = stamp_sec;
      have_last_imu_stamp_ = true;
      return;
    }

    const double dt = stamp_sec - last_imu_stamp_sec_;
    last_imu_stamp_sec_ = stamp_sec;
    if (dt <= 0.0 || dt > imu_integration_max_dt_sec_) {
      return;
    }

    const double wx = imu.angular_velocity.x;
    const double wy = imu.angular_velocity.y;
    const double wz = imu.angular_velocity.z;
    if (!std::isfinite(wx) || !std::isfinite(wy) || !std::isfinite(wz)) {
      return;
    }

    imu_orientation_ = multiply(imu_orientation_, delta_quaternion(wx, wy, wz, dt));
    normalize(imu_orientation_);
    ++imu_integration_count_;
  }

  void publish_frontend_pose(
    const geometry_msgs::msg::PoseStamped & pose, bool from_odometry, bool count_pose_input)
  {
    pose_pub_->publish(pose);

    nav_msgs::msg::Odometry odometry;
    odometry.header = pose.header;
    odometry.child_frame_id = frontend_child_frame_;
    odometry.pose.pose = pose.pose;
    frontend_odometry_pub_->publish(odometry);

    frontend_path_.header = pose.header;
    frontend_path_.poses.push_back(pose);
    while (frontend_path_.poses.size() > static_cast<size_t>(max_path_length_)) {
      frontend_path_.poses.erase(frontend_path_.poses.begin());
    }
    frontend_path_pub_->publish(frontend_path_);

    if (tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = pose.header;
      transform.child_frame_id = frontend_child_frame_;
      transform.transform.translation.x = pose.pose.position.x;
      transform.transform.translation.y = pose.pose.position.y;
      transform.transform.translation.z = pose.pose.position.z;
      transform.transform.rotation = pose.pose.orientation;
      tf_broadcaster_->sendTransform(transform);
    }

    ++pose_count_;
    if (count_pose_input && !from_odometry) {
      ++pose_stamped_count_;
    }
  }

  void report_counts()
  {
    RCLCPP_INFO(
      get_logger(),
      "forwarded image=%zu camera_info=%zu depth=%zu points=%zu pose=%zu pose_input=%zu odom=%zu "
      "identity_pose=%zu imu_pose=%zu imu=%zu imu_integrations=%zu transformed_points=%zu "
      "transform_errors=%zu",
      image_count_, camera_info_count_, depth_count_, pointcloud_count_, pose_count_,
      pose_stamped_count_, odometry_count_, identity_pose_count_, imu_pose_fallback_count_,
      imu_count_, imu_integration_count_, transformed_pointcloud_count_,
      pointcloud_transform_error_count_);
  }

  std::string raw_image_topic_;
  std::string raw_camera_info_topic_;
  std::string raw_depth_topic_;
  std::string raw_pointcloud_topic_;
  std::string raw_imu_topic_;
  std::string pose_stamped_topic_;
  std::string raw_odometry_topic_;
  std::string image_topic_;
  std::string camera_info_topic_;
  std::string depth_topic_;
  std::string pointcloud_topic_;
  std::string pose_topic_;
  std::string imu_topic_;
  std::string frontend_odometry_topic_;
  std::string frontend_path_topic_;
  std::string world_frame_;
  std::string frontend_child_frame_;
  bool enable_depth_passthrough_{true};
  bool enable_imu_passthrough_{true};
  bool identity_pose_fallback_{false};
  bool imu_pose_fallback_{false};
  bool transform_pointcloud_to_camera_frame_{false};
  bool publish_tf_{false};
  int max_path_length_{5000};
  double report_period_sec_{2.0};
  double imu_integration_max_dt_sec_{0.05};
  std::string pointcloud_transform_profile_{"identity"};
  std::string pointcloud_transform_target_frame_{"camera"};
  std::vector<double> pointcloud_transform_rotation_{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  std::vector<double> pointcloud_transform_translation_{0.0, 0.0, 0.0};
  bool have_last_imu_stamp_{false};
  double last_imu_stamp_sec_{0.0};
  UnitQuaternion imu_orientation_;

  size_t image_count_{0};
  size_t camera_info_count_{0};
  size_t depth_count_{0};
  size_t pointcloud_count_{0};
  size_t pose_count_{0};
  size_t pose_stamped_count_{0};
  size_t odometry_count_{0};
  size_t identity_pose_count_{0};
  size_t imu_pose_fallback_count_{0};
  size_t imu_integration_count_{0};
  size_t transformed_pointcloud_count_{0};
  size_t pointcloud_transform_error_count_{0};
  size_t imu_count_{0};

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr frontend_odometry_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr frontend_path_pub_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_stamped_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::TimerBase::SharedPtr report_timer_;
  nav_msgs::msg::Path frontend_path_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Lic2ContractAdapter>());
  rclcpp::shutdown();
  return 0;
}
