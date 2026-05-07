// SPDX-License-Identifier: GPL-3.0-or-later

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <limits>
#include <mutex>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
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

constexpr int64_t kNanosecondsPerSecond = 1000000000LL;

struct QosProfileParams
{
  std::string reliability{"best_effort"};
  std::string history{"keep_last"};
  int depth{5};
};

int64_t stamp_to_nsec(const builtin_interfaces::msg::Time & stamp)
{
  if (stamp.nanosec >= static_cast<uint32_t>(kNanosecondsPerSecond)) {
    throw std::invalid_argument("ROS2 Time.nanosec must be less than 1e9");
  }
  return static_cast<int64_t>(stamp.sec) * kNanosecondsPerSecond +
    static_cast<int64_t>(stamp.nanosec);
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

std::array<double, 9> rotation_matrix(const UnitQuaternion & q)
{
  return {
    1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    2.0 * (q.x * q.y - q.z * q.w),
    2.0 * (q.x * q.z + q.y * q.w),
    2.0 * (q.x * q.y + q.z * q.w),
    1.0 - 2.0 * (q.x * q.x + q.z * q.z),
    2.0 * (q.y * q.z - q.x * q.w),
    2.0 * (q.x * q.z - q.y * q.w),
    2.0 * (q.y * q.z + q.x * q.w),
    1.0 - 2.0 * (q.x * q.x + q.y * q.y)};
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
    rotate_pointcloud_with_imu_pose_ =
      declare_parameter<bool>("rotate_pointcloud_with_imu_pose", true);
    pointcloud_use_stamp_imu_orientation_ =
      declare_parameter<bool>("pointcloud_use_stamp_imu_orientation", true);
    imu_orientation_history_size_ =
      declare_parameter<int>("imu_orientation_history_size", 50000);
    if (imu_orientation_history_size_ < 1) {
      RCLCPP_WARN(get_logger(), "imu_orientation_history_size must be positive; using 1");
      imu_orientation_history_size_ = 1;
    }
    sync_image_to_pointcloud_ = declare_parameter<bool>("sync_image_to_pointcloud", false);
    sync_image_pointcloud_tolerance_sec_ =
      declare_parameter<double>("sync_image_pointcloud_tolerance_sec", 0.01);
    if (sync_image_pointcloud_tolerance_sec_ < 0.0) {
      RCLCPP_WARN(get_logger(), "sync_image_pointcloud_tolerance_sec must be non-negative; using 0");
      sync_image_pointcloud_tolerance_sec_ = 0.0;
    }
    sync_image_pointcloud_tolerance_nsec_ =
      static_cast<int64_t>(std::llround(sync_image_pointcloud_tolerance_sec_ * 1.0e9));
    sync_pointcloud_queue_size_ = declare_parameter<int>("sync_pointcloud_queue_size", 32);
    if (sync_pointcloud_queue_size_ < 1) {
      RCLCPP_WARN(get_logger(), "sync_pointcloud_queue_size must be positive; using 1");
      sync_pointcloud_queue_size_ = 1;
    }
    imu_integration_max_dt_sec_ = declare_parameter<double>("imu_integration_max_dt_sec", 0.05);
    pointcloud_transform_profile_ =
      lowercase(declare_parameter<std::string>("pointcloud_transform_profile", "identity"));
    transform_pointcloud_to_camera_frame_ =
      declare_parameter<bool>("transform_pointcloud_to_camera_frame", false);
    pointcloud_transform_target_frame_ =
      declare_parameter<std::string>("pointcloud_transform_target_frame", "camera");
    pointcloud_filter_min_z_ =
      declare_parameter<double>("pointcloud_filter_min_z", -std::numeric_limits<double>::max());
    pointcloud_filter_max_z_ = declare_parameter<double>("pointcloud_filter_max_z", 0.0);
    pointcloud_filter_min_points_ = declare_parameter<int>("pointcloud_filter_min_points", 0);
    if (pointcloud_filter_min_points_ < 0) {
      RCLCPP_WARN(get_logger(), "pointcloud_filter_min_points must be non-negative; disabling");
      pointcloud_filter_min_points_ = 0;
    }
    configure_pointcloud_transform();
    world_frame_ = declare_parameter<std::string>("world_frame", "map");
    frontend_child_frame_ = declare_parameter<std::string>("frontend_child_frame", "base_link");
    publish_tf_ = declare_parameter<bool>("publish_tf", false);
    max_path_length_ = declare_parameter<int>("max_path_length", 5000);
    report_period_sec_ = declare_parameter<double>("report_period_sec", 2.0);
    sensor_qos_reliability_ = declare_parameter<std::string>("sensor_qos_reliability", "best_effort");
    sensor_qos_history_ = declare_parameter<std::string>("sensor_qos_history", "keep_last");
    sensor_qos_depth_ = declare_parameter<int>("sensor_qos_depth", 5);
    raw_image_qos_ = declare_topic_qos("raw_image");
    raw_camera_info_qos_ = declare_topic_qos("raw_camera_info");
    raw_depth_qos_ = declare_topic_qos("raw_depth");
    raw_pointcloud_qos_ = declare_topic_qos("raw_pointcloud");
    raw_imu_qos_ = declare_topic_qos("raw_imu");
    pose_stamped_qos_ = declare_topic_qos("pose_stamped");
    raw_odometry_qos_ = declare_topic_qos("raw_odometry");
    image_qos_ = declare_topic_qos("image");
    camera_info_qos_ = declare_topic_qos("camera_info");
    depth_qos_ = declare_topic_qos("depth");
    pointcloud_qos_ = declare_topic_qos("pointcloud");
    pose_qos_ = declare_topic_qos("pose");
    imu_qos_ = declare_topic_qos("imu");
    frontend_odometry_qos_ = declare_topic_qos("frontend_odometry");
    if (max_path_length_ <= 0) {
      RCLCPP_WARN(get_logger(), "max_path_length must be positive; using 1");
      max_path_length_ = 1;
    }

    const auto raw_image_qos = make_sensor_qos("raw_image", raw_image_qos_);
    const auto raw_camera_info_qos = make_sensor_qos("raw_camera_info", raw_camera_info_qos_);
    const auto raw_depth_qos = make_sensor_qos("raw_depth", raw_depth_qos_);
    const auto raw_pointcloud_qos = make_sensor_qos("raw_pointcloud", raw_pointcloud_qos_);
    const auto raw_imu_qos = make_sensor_qos("raw_imu", raw_imu_qos_);
    const auto pose_stamped_qos = make_sensor_qos("pose_stamped", pose_stamped_qos_);
    const auto raw_odometry_qos = make_sensor_qos("raw_odometry", raw_odometry_qos_);
    const auto image_qos = make_sensor_qos("image", image_qos_);
    const auto camera_info_qos = make_sensor_qos("camera_info", camera_info_qos_);
    const auto depth_qos = make_sensor_qos("depth", depth_qos_);
    const auto pointcloud_qos = make_sensor_qos("pointcloud", pointcloud_qos_);
    const auto pose_qos = make_sensor_qos("pose", pose_qos_);
    const auto imu_qos = make_sensor_qos("imu", imu_qos_);
    const auto frontend_odometry_qos =
      make_sensor_qos("frontend_odometry", frontend_odometry_qos_);
    const rclcpp::QoS path_qos = rclcpp::QoS(1).reliable().transient_local();

    image_pub_ = create_publisher<sensor_msgs::msg::Image>(image_topic_, image_qos);
    camera_info_pub_ =
      create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic_, camera_info_qos);
    depth_pub_ = create_publisher<sensor_msgs::msg::Image>(depth_topic_, depth_qos);
    pointcloud_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud_topic_, pointcloud_qos);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_, pose_qos);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_, imu_qos);
    frontend_odometry_pub_ =
      create_publisher<nav_msgs::msg::Odometry>(frontend_odometry_topic_, frontend_odometry_qos);
    frontend_path_pub_ = create_publisher<nav_msgs::msg::Path>(frontend_path_topic_, path_qos);
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      raw_image_topic_, raw_image_qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
        if (sync_image_to_pointcloud_) {
          {
            std::lock_guard<std::mutex> lock(latest_visual_mutex_);
            latest_image_ = std::make_shared<sensor_msgs::msg::Image>(*msg);
          }
          process_pending_pointclouds();
        } else {
          image_pub_->publish(*msg);
          ++image_count_;
        }
      });
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      raw_camera_info_topic_, raw_camera_info_qos,
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
        if (sync_image_to_pointcloud_) {
          {
            std::lock_guard<std::mutex> lock(latest_visual_mutex_);
            latest_camera_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*msg);
          }
          process_pending_pointclouds();
        } else {
          camera_info_pub_->publish(*msg);
          ++camera_info_count_;
        }
      });
    pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      raw_pointcloud_topic_, raw_pointcloud_qos,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        if (sync_image_to_pointcloud_) {
          const auto visual_state = pointcloud_visual_sync_state(msg->header.stamp);
          if (visual_state == VisualSyncState::kWaiting) {
            defer_pointcloud(msg);
            return;
          }
          if (visual_state == VisualSyncState::kMissed) {
            ++visual_sync_missed_pointcloud_count_;
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 2000,
              "using nearest future image/camera_info for pointcloud after exact visual sync was missed");
          }
        }
        handle_pointcloud(msg);
      });
    pose_stamped_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_stamped_topic_, pose_stamped_qos,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
        publish_frontend_pose(*msg, false, true);
      });
    odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      raw_odometry_topic_, raw_odometry_qos,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = msg->header;
        pose.pose = msg->pose.pose;
        publish_frontend_pose(pose, true, false);
        ++odometry_count_;
      });

    if (enable_depth_passthrough_) {
      depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
        raw_depth_topic_, raw_depth_qos,
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
          depth_pub_->publish(*msg);
          ++depth_count_;
        });
    }
    if (enable_imu_passthrough_) {
      imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        raw_imu_topic_, raw_imu_qos,
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
      "fallback(identity=%s imu=%s) image_sync=%s pointcloud_transform=%s profile=%s "
      "pointcloud_filter(min_z=%.6f max_z=%.6f min_points=%d) "
      "imu_stamp_lookup=%s imu_history_size=%d visual_sync_tol=%.6f sync_queue=%d",
      raw_image_topic_.c_str(), raw_pointcloud_topic_.c_str(), pose_stamped_topic_.c_str(),
      raw_odometry_topic_.c_str(), image_topic_.c_str(), pointcloud_topic_.c_str(),
      pose_topic_.c_str(), frontend_odometry_topic_.c_str(), frontend_path_topic_.c_str(),
      identity_pose_fallback_ ? "true" : "false", imu_pose_fallback_ ? "true" : "false",
      sync_image_to_pointcloud_ ? "true" : "false",
      needs_pointcloud_transform() ? "true" : "false",
      pointcloud_transform_profile_.c_str(), pointcloud_filter_min_z_, pointcloud_filter_max_z_,
      pointcloud_filter_min_points_, pointcloud_use_stamp_imu_orientation_ ? "true" : "false",
      imu_orientation_history_size_, sync_image_pointcloud_tolerance_sec_,
      sync_pointcloud_queue_size_);
  }

private:
  enum class VisualSyncState
  {
    kWaiting,
    kReady,
    kMissed
  };

  bool needs_pointcloud_transform() const
  {
    return transform_pointcloud_to_camera_frame_ ||
      (imu_pose_fallback_ && rotate_pointcloud_with_imu_pose_);
  }

  bool pointcloud_filter_enabled() const
  {
    return pointcloud_filter_max_z_ > 0.0 ||
      pointcloud_filter_min_z_ > -std::numeric_limits<double>::max() / 2.0 ||
      pointcloud_filter_min_points_ > 0;
  }

  VisualSyncState pointcloud_visual_sync_state(
    const builtin_interfaces::msg::Time & stamp)
  {
    const int64_t point_nsec = stamp_to_nsec(stamp);
    std::lock_guard<std::mutex> lock(latest_visual_mutex_);
    if (!latest_image_ || !latest_camera_info_) {
      return VisualSyncState::kWaiting;
    }

    const int64_t image_nsec = stamp_to_nsec(latest_image_->header.stamp);
    const int64_t camera_info_nsec = stamp_to_nsec(latest_camera_info_->header.stamp);
    const int64_t min_visual_nsec = std::min(image_nsec, camera_info_nsec);
    const int64_t max_visual_nsec = std::max(image_nsec, camera_info_nsec);
    if (max_visual_nsec < point_nsec - sync_image_pointcloud_tolerance_nsec_) {
      return VisualSyncState::kWaiting;
    }
    if (min_visual_nsec > point_nsec + sync_image_pointcloud_tolerance_nsec_) {
      return VisualSyncState::kMissed;
    }
    return VisualSyncState::kReady;
  }

  void defer_pointcloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(pending_pointcloud_mutex_);
    pending_pointclouds_.push_back(std::move(msg));
    ++visual_sync_deferred_pointcloud_count_;
    while (pending_pointclouds_.size() > static_cast<size_t>(sync_pointcloud_queue_size_)) {
      pending_pointclouds_.pop_front();
      ++visual_sync_dropped_pointcloud_count_;
      ++dropped_pointcloud_count_;
    }
  }

  void process_pending_pointclouds()
  {
    while (true) {
      sensor_msgs::msg::PointCloud2::ConstSharedPtr msg;
      {
        std::lock_guard<std::mutex> lock(pending_pointcloud_mutex_);
        if (pending_pointclouds_.empty()) {
          return;
        }
        msg = pending_pointclouds_.front();
      }

      const auto state = pointcloud_visual_sync_state(msg->header.stamp);
      if (state == VisualSyncState::kWaiting) {
        return;
      }

      {
        std::lock_guard<std::mutex> lock(pending_pointcloud_mutex_);
        if (pending_pointclouds_.empty() || pending_pointclouds_.front() != msg) {
          continue;
        }
        pending_pointclouds_.pop_front();
      }

      if (state == VisualSyncState::kMissed) {
        ++visual_sync_missed_pointcloud_count_;
      }
      ++visual_sync_released_pointcloud_count_;
      handle_pointcloud(msg);
    }
  }

  void handle_pointcloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    std::optional<sensor_msgs::msg::PointCloud2> cloud;
    if (needs_pointcloud_transform() || pointcloud_filter_enabled()) {
      try {
        cloud = transform_pointcloud(*msg);
        if (needs_pointcloud_transform()) {
          ++transformed_pointcloud_count_;
        }
      } catch (const std::exception & ex) {
        ++pointcloud_transform_error_count_;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "failed to transform/filter pointcloud%s: %s",
          pointcloud_filter_enabled() ? "; dropping cloud because filtering is required" :
          "; forwarding original cloud",
          ex.what());
        if (pointcloud_filter_enabled()) {
          ++dropped_pointcloud_count_;
          return;
        }
        cloud = *msg;
      }
    } else {
      cloud = *msg;
    }
    if (!cloud) {
      ++dropped_pointcloud_count_;
      return;
    }
    publish_synced_visuals(msg->header.stamp);
    pointcloud_pub_->publish(*cloud);
    ++pointcloud_count_;
    if (imu_pose_fallback_) {
      publish_imu_pose_fallback(msg->header.stamp);
    } else if (identity_pose_fallback_) {
      publish_identity_pose(msg->header.stamp);
    }
  }

  void publish_synced_visuals(const builtin_interfaces::msg::Time & stamp)
  {
    if (!sync_image_to_pointcloud_) {
      return;
    }

    std::shared_ptr<sensor_msgs::msg::Image> image;
    std::shared_ptr<sensor_msgs::msg::CameraInfo> camera_info;
    {
      std::lock_guard<std::mutex> lock(latest_visual_mutex_);
      if (latest_image_) {
        image = std::make_shared<sensor_msgs::msg::Image>(*latest_image_);
      }
      if (latest_camera_info_) {
        camera_info = std::make_shared<sensor_msgs::msg::CameraInfo>(*latest_camera_info_);
      }
    }

    if (image) {
      image->header.stamp = stamp;
      image_pub_->publish(*image);
      ++image_count_;
    }
    if (camera_info) {
      camera_info->header.stamp = stamp;
      camera_info_pub_->publish(*camera_info);
      ++camera_info_count_;
    }
  }

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

  QosProfileParams declare_topic_qos(const std::string & prefix)
  {
    QosProfileParams params;
    params.reliability =
      declare_parameter<std::string>(prefix + "_qos_reliability", sensor_qos_reliability_);
    params.history = declare_parameter<std::string>(prefix + "_qos_history", sensor_qos_history_);
    params.depth = declare_parameter<int>(prefix + "_qos_depth", sensor_qos_depth_);
    return params;
  }

  rclcpp::QoS make_sensor_qos(const char * stream_name, const QosProfileParams & params)
  {
    const int depth = std::max(params.depth, 1);
    rclcpp::QoS qos(static_cast<size_t>(depth));
    const auto history = lowercase(params.history);
    if (history == "keep_all") {
      qos.keep_all();
    } else if (history != "keep_last") {
      RCLCPP_WARN(
        get_logger(), "unknown %s_qos_history '%s'; using keep_last",
        stream_name, history.c_str());
    }

    const auto reliability = lowercase(params.reliability);
    if (reliability == "reliable") {
      qos.reliable();
    } else {
      if (reliability != "best_effort") {
        RCLCPP_WARN(
          get_logger(), "unknown %s_qos_reliability '%s'; using best_effort",
          stream_name, reliability.c_str());
      }
      qos.best_effort();
    }

    qos.durability_volatile();
    return qos;
  }

  std::optional<sensor_msgs::msg::PointCloud2> transform_pointcloud(
    const sensor_msgs::msg::PointCloud2 & cloud)
  {
    sensor_msgs::msg::PointCloud2 out = cloud;
    const bool rotate_with_imu = imu_pose_fallback_ && rotate_pointcloud_with_imu_pose_;
    const UnitQuaternion imu_orientation = pointcloud_use_stamp_imu_orientation_ ?
      imu_orientation_at_or_before(cloud.header.stamp) :
      current_imu_orientation();
    const auto imu_rotation = rotation_matrix(imu_orientation);
    if (rotate_with_imu) {
      out.header.frame_id = world_frame_;
    } else if (!pointcloud_transform_target_frame_.empty()) {
      out.header.frame_id = pointcloud_transform_target_frame_;
    }

    const auto * x_field = find_field(out, "x");
    const auto * y_field = find_field(out, "y");
    const auto * z_field = find_field(out, "z");
    if (!x_field || !y_field || !z_field) {
      throw std::runtime_error("PointCloud2 must contain x/y/z fields");
    }

    const size_t point_count = static_cast<size_t>(out.width) * static_cast<size_t>(out.height);
    const bool filter_points = pointcloud_filter_enabled();
    std::vector<uint8_t> filtered;
    if (filter_points) {
      filtered.resize(point_count * static_cast<size_t>(out.point_step));
    }
    size_t kept_points = 0;
    for (size_t i = 0; i < point_count; ++i) {
      const uint8_t * source = cloud.data.data() + i * cloud.point_step;
      const double x = read_float_field(source, *x_field);
      const double y = read_float_field(source, *y_field);
      const double z = read_float_field(source, *z_field);
      const auto & r = pointcloud_transform_rotation_;
      const auto & t = pointcloud_transform_translation_;
      double tx = x;
      double ty = y;
      double tz = z;
      if (transform_pointcloud_to_camera_frame_) {
        tx = r[0] * x + r[1] * y + r[2] * z + t[0];
        ty = r[3] * x + r[4] * y + r[5] * z + t[1];
        tz = r[6] * x + r[7] * y + r[8] * z + t[2];
      }
      if (filter_points) {
        if (!std::isfinite(tx) || !std::isfinite(ty) || !std::isfinite(tz)) {
          continue;
        }
        if (tz <= pointcloud_filter_min_z_) {
          continue;
        }
        if (pointcloud_filter_max_z_ > 0.0 && tz > pointcloud_filter_max_z_) {
          continue;
        }
      }
      if (rotate_with_imu) {
        const double wx = imu_rotation[0] * tx + imu_rotation[1] * ty + imu_rotation[2] * tz;
        const double wy = imu_rotation[3] * tx + imu_rotation[4] * ty + imu_rotation[5] * tz;
        const double wz = imu_rotation[6] * tx + imu_rotation[7] * ty + imu_rotation[8] * tz;
        tx = wx;
        ty = wy;
        tz = wz;
      }
      uint8_t * target = filter_points ?
        (filtered.data() + kept_points * static_cast<size_t>(out.point_step)) :
        (out.data.data() + i * out.point_step);
      if (filter_points) {
        std::memcpy(target, source, out.point_step);
      }
      write_float_field(target, *x_field, tx);
      write_float_field(target, *y_field, ty);
      write_float_field(target, *z_field, tz);
      ++kept_points;
    }
    if (filter_points) {
      if (pointcloud_filter_min_points_ > 0 &&
        kept_points < static_cast<size_t>(pointcloud_filter_min_points_))
      {
        ++filtered_pointcloud_drop_count_;
        return std::nullopt;
      }
      filtered.resize(kept_points * static_cast<size_t>(out.point_step));
      out.data = std::move(filtered);
      out.height = 1;
      out.width = static_cast<uint32_t>(kept_points);
      out.row_step = static_cast<uint32_t>(kept_points * static_cast<size_t>(out.point_step));
      out.is_dense = false;
      filtered_pointcloud_input_points_ += point_count;
      filtered_pointcloud_output_points_ += kept_points;
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
    const UnitQuaternion imu_orientation = pointcloud_use_stamp_imu_orientation_ ?
      imu_orientation_at_or_before(stamp) :
      current_imu_orientation();
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = world_frame_;
    pose.pose.orientation.w = imu_orientation.w;
    pose.pose.orientation.x = imu_orientation.x;
    pose.pose.orientation.y = imu_orientation.y;
    pose.pose.orientation.z = imu_orientation.z;
    publish_frontend_pose(pose, false, false);
    ++imu_pose_fallback_count_;
  }

  void integrate_imu_orientation(const sensor_msgs::msg::Imu & imu)
  {
    const int64_t stamp_nsec = stamp_to_nsec(imu.header.stamp);
    std::lock_guard<std::mutex> lock(imu_mutex_);
    if (!have_last_imu_stamp_) {
      last_imu_stamp_nsec_ = stamp_nsec;
      have_last_imu_stamp_ = true;
      record_imu_orientation_locked(stamp_nsec);
      return;
    }

    const int64_t dt_nsec = stamp_nsec - last_imu_stamp_nsec_;
    last_imu_stamp_nsec_ = stamp_nsec;
    const double dt = static_cast<double>(dt_nsec) / static_cast<double>(kNanosecondsPerSecond);
    if (dt <= 0.0 || dt > imu_integration_max_dt_sec_) {
      record_imu_orientation_locked(stamp_nsec);
      return;
    }

    const double wx = imu.angular_velocity.x;
    const double wy = imu.angular_velocity.y;
    const double wz = imu.angular_velocity.z;
    if (!std::isfinite(wx) || !std::isfinite(wy) || !std::isfinite(wz)) {
      record_imu_orientation_locked(stamp_nsec);
      return;
    }

    imu_orientation_ = multiply(imu_orientation_, delta_quaternion(wx, wy, wz, dt));
    normalize(imu_orientation_);
    record_imu_orientation_locked(stamp_nsec);
    ++imu_integration_count_;
  }

  void record_imu_orientation_locked(const int64_t stamp_nsec)
  {
    imu_orientation_history_.push_back({stamp_nsec, imu_orientation_});
    while (imu_orientation_history_.size() > static_cast<size_t>(imu_orientation_history_size_)) {
      imu_orientation_history_.pop_front();
    }
  }

  UnitQuaternion current_imu_orientation() const
  {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    return imu_orientation_;
  }

  UnitQuaternion imu_orientation_at_or_before(const builtin_interfaces::msg::Time & stamp) const
  {
    const int64_t stamp_nsec = stamp_to_nsec(stamp);
    std::lock_guard<std::mutex> lock(imu_mutex_);
    for (auto it = imu_orientation_history_.rbegin(); it != imu_orientation_history_.rend(); ++it) {
      if (it->first <= stamp_nsec) {
        return it->second;
      }
    }
    return UnitQuaternion{};
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
      "transform_errors=%zu dropped_points=%zu filter_drops=%zu filter_points=%zu/%zu "
      "visual_sync(deferred=%zu released=%zu missed=%zu queue_drops=%zu)",
      image_count_, camera_info_count_, depth_count_, pointcloud_count_, pose_count_,
      pose_stamped_count_, odometry_count_, identity_pose_count_, imu_pose_fallback_count_,
      imu_count_, imu_integration_count_, transformed_pointcloud_count_,
      pointcloud_transform_error_count_, dropped_pointcloud_count_, filtered_pointcloud_drop_count_,
      filtered_pointcloud_output_points_, filtered_pointcloud_input_points_,
      visual_sync_deferred_pointcloud_count_, visual_sync_released_pointcloud_count_,
      visual_sync_missed_pointcloud_count_, visual_sync_dropped_pointcloud_count_);
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
  bool rotate_pointcloud_with_imu_pose_{true};
  bool pointcloud_use_stamp_imu_orientation_{true};
  int imu_orientation_history_size_{50000};
  bool sync_image_to_pointcloud_{false};
  double sync_image_pointcloud_tolerance_sec_{0.01};
  int64_t sync_image_pointcloud_tolerance_nsec_{10000000LL};
  int sync_pointcloud_queue_size_{32};
  bool transform_pointcloud_to_camera_frame_{false};
  double pointcloud_filter_min_z_{-std::numeric_limits<double>::max()};
  double pointcloud_filter_max_z_{0.0};
  int pointcloud_filter_min_points_{0};
  bool publish_tf_{false};
  int max_path_length_{5000};
  double report_period_sec_{2.0};
  std::string sensor_qos_reliability_{"best_effort"};
  std::string sensor_qos_history_{"keep_last"};
  int sensor_qos_depth_{5};
  QosProfileParams raw_image_qos_;
  QosProfileParams raw_camera_info_qos_;
  QosProfileParams raw_depth_qos_;
  QosProfileParams raw_pointcloud_qos_;
  QosProfileParams raw_imu_qos_;
  QosProfileParams pose_stamped_qos_;
  QosProfileParams raw_odometry_qos_;
  QosProfileParams image_qos_;
  QosProfileParams camera_info_qos_;
  QosProfileParams depth_qos_;
  QosProfileParams pointcloud_qos_;
  QosProfileParams pose_qos_;
  QosProfileParams imu_qos_;
  QosProfileParams frontend_odometry_qos_;
  double imu_integration_max_dt_sec_{0.05};
  std::string pointcloud_transform_profile_{"identity"};
  std::string pointcloud_transform_target_frame_{"camera"};
  std::vector<double> pointcloud_transform_rotation_{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  std::vector<double> pointcloud_transform_translation_{0.0, 0.0, 0.0};
  bool have_last_imu_stamp_{false};
  int64_t last_imu_stamp_nsec_{0};
  UnitQuaternion imu_orientation_;
  std::deque<std::pair<int64_t, UnitQuaternion>> imu_orientation_history_;
  mutable std::mutex imu_mutex_;
  std::mutex latest_visual_mutex_;
  std::mutex pending_pointcloud_mutex_;
  std::shared_ptr<sensor_msgs::msg::Image> latest_image_;
  std::shared_ptr<sensor_msgs::msg::CameraInfo> latest_camera_info_;
  std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr> pending_pointclouds_;

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
  size_t dropped_pointcloud_count_{0};
  size_t filtered_pointcloud_drop_count_{0};
  size_t filtered_pointcloud_input_points_{0};
  size_t filtered_pointcloud_output_points_{0};
  size_t visual_sync_deferred_pointcloud_count_{0};
  size_t visual_sync_released_pointcloud_count_{0};
  size_t visual_sync_missed_pointcloud_count_{0};
  size_t visual_sync_dropped_pointcloud_count_{0};
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
