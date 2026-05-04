// SPDX-License-Identifier: GPL-3.0-or-later

#include <algorithm>
#include <memory>
#include <string>

#include <gaussian_lic_tracking/imu_propagator.hpp>
#include <gaussian_lic_tracking/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>

class TrackingNode final : public rclcpp::Node
{
public:
  explicit TrackingNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("tracking_node", options)
  {
    raw_image_topic_ = declare_parameter<std::string>("raw_image_topic", "/camera/image");
    raw_camera_info_topic_ = declare_parameter<std::string>("raw_camera_info_topic", "/camera/camera_info");
    raw_depth_topic_ = declare_parameter<std::string>("raw_depth_topic", "/camera/depth");
    raw_pointcloud_topic_ = declare_parameter<std::string>("raw_pointcloud_topic", "/livox/lidar");
    raw_imu_topic_ = declare_parameter<std::string>("raw_imu_topic", "/imu");
    image_topic_ = declare_parameter<std::string>("image_topic", "/image_for_gs");
    camera_info_topic_ = declare_parameter<std::string>("camera_info_topic", "/camera_info_for_gs");
    depth_topic_ = declare_parameter<std::string>("depth_topic", "/depth_for_gs");
    pointcloud_topic_ = declare_parameter<std::string>("pointcloud_topic", "/points_for_gs");
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/pose_for_gs");
    odometry_topic_ = declare_parameter<std::string>("odometry_topic", "/gaussian_lic/frontend/odometry");
    path_topic_ = declare_parameter<std::string>("path_topic", "/gaussian_lic/frontend/path");
    world_frame_ = declare_parameter<std::string>("world_frame", "map");
    child_frame_ = declare_parameter<std::string>("child_frame", "base_link");
    publish_tf_ = declare_parameter<bool>("publish_tf", false);
    max_path_length_ = declare_parameter<int>("max_path_length", 5000);
    sensor_qos_depth_ = declare_parameter<int>("sensor_qos_depth", 5);
    sensor_qos_reliability_ = declare_parameter<std::string>("sensor_qos_reliability", "best_effort");

    auto qos = make_sensor_qos();
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      raw_image_topic_, qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
        image_pub_->publish(*msg);
      });
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      raw_camera_info_topic_, qos,
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
        camera_info_pub_->publish(*msg);
      });
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      raw_depth_topic_, qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
        depth_pub_->publish(*msg);
      });
    pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      raw_pointcloud_topic_, qos,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        handle_pointcloud(*msg);
      });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      raw_imu_topic_, qos,
      [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
        handle_imu(*msg);
      });

    image_pub_ = create_publisher<sensor_msgs::msg::Image>(image_topic_, qos);
    camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic_, qos);
    depth_pub_ = create_publisher<sensor_msgs::msg::Image>(depth_topic_, qos);
    pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud_topic_, qos);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_, 20);
    odometry_pub_ = create_publisher<nav_msgs::msg::Odometry>(odometry_topic_, 20);
    path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic_, rclcpp::QoS(1).transient_local().reliable());
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }
  }

private:
  rclcpp::QoS make_sensor_qos() const
  {
    rclcpp::QoS qos(static_cast<size_t>(std::max(sensor_qos_depth_, 1)));
    qos.keep_last(static_cast<size_t>(std::max(sensor_qos_depth_, 1)));
    qos.durability_volatile();
    if (sensor_qos_reliability_ == "reliable") {
      qos.reliable();
    } else {
      qos.best_effort();
    }
    return qos;
  }

  void handle_imu(const sensor_msgs::msg::Imu & msg)
  {
    try {
      const int64_t stamp_ns = gaussian_lic_tracking::stamp_to_nanoseconds(msg.header.stamp);
      imu_propagator_.add_measurement(
        stamp_ns,
        Eigen::Vector3d{
          msg.angular_velocity.x,
          msg.angular_velocity.y,
          msg.angular_velocity.z},
        Eigen::Vector3d{
          msg.linear_acceleration.x,
          msg.linear_acceleration.y,
          msg.linear_acceleration.z});
    } catch (const std::exception & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "failed to propagate IMU tracking state: %s", ex.what());
    }
  }

  void handle_pointcloud(const sensor_msgs::msg::PointCloud2 & msg)
  {
    pointcloud_pub_->publish(msg);

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = msg.header.stamp;
    pose.header.frame_id = world_frame_;
    if (imu_propagator_.initialized()) {
      const auto & state = imu_propagator_.state();
      pose.pose.position.x = state.p_w_i.x();
      pose.pose.position.y = state.p_w_i.y();
      pose.pose.position.z = state.p_w_i.z();
      pose.pose.orientation.x = state.q_w_i.x();
      pose.pose.orientation.y = state.q_w_i.y();
      pose.pose.orientation.z = state.q_w_i.z();
      pose.pose.orientation.w = state.q_w_i.w();
    } else {
      pose.pose.orientation.w = 1.0;
    }
    pose_pub_->publish(pose);

    nav_msgs::msg::Odometry odom;
    odom.header = pose.header;
    odom.child_frame_id = child_frame_;
    odom.pose.pose = pose.pose;
    odometry_pub_->publish(odom);

    path_.header = pose.header;
    path_.poses.push_back(pose);
    while (max_path_length_ > 0 && path_.poses.size() > static_cast<size_t>(max_path_length_)) {
      path_.poses.erase(path_.poses.begin());
    }
    path_pub_->publish(path_);

    if (tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header = pose.header;
      tf.child_frame_id = child_frame_;
      tf.transform.translation.x = pose.pose.position.x;
      tf.transform.translation.y = pose.pose.position.y;
      tf.transform.translation.z = pose.pose.position.z;
      tf.transform.rotation = pose.pose.orientation;
      tf_broadcaster_->sendTransform(tf);
    }
  }

  std::string raw_image_topic_;
  std::string raw_camera_info_topic_;
  std::string raw_depth_topic_;
  std::string raw_pointcloud_topic_;
  std::string raw_imu_topic_;
  std::string image_topic_;
  std::string camera_info_topic_;
  std::string depth_topic_;
  std::string pointcloud_topic_;
  std::string pose_topic_;
  std::string odometry_topic_;
  std::string path_topic_;
  std::string world_frame_;
  std::string child_frame_;
  bool publish_tf_{false};
  int max_path_length_{5000};
  int sensor_qos_depth_{5};
  std::string sensor_qos_reliability_{"best_effort"};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  gaussian_lic_tracking::ImuPropagator imu_propagator_;
  nav_msgs::msg::Path path_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrackingNode>());
  rclcpp::shutdown();
  return 0;
}
