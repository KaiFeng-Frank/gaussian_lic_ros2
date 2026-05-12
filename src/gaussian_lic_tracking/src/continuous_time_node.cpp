// SPDX-License-Identifier: GPL-3.0-or-later
//
// Independent ROS2 node that exercises the newly-ported continuous-time
// tracker on real-world streams. Subscribes to a raw IMU topic, feeds the
// samples into `ContinuousTimeSlidingWindowEstimator`, periodically solves,
// and publishes the optimized body pose as a `nav_msgs/Odometry` message
// plus an aggregated `Path`.
//
// This node is deliberately separate from `tracking_node` so it can be
// validated against real bags without risking regressions in the existing
// 12/12 strict parity matrix.

#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <memory>
#include <mutex>
#include <sstream>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <gaussian_lic_tracking/spline/continuous_time_sliding_window.hpp>
#include <gaussian_lic_tracking/spline/lidar_plane_extractor.hpp>

namespace gaussian_lic_tracking
{

namespace
{

int64_t to_signed_nanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1'000'000'000LL +
         static_cast<int64_t>(stamp.nanosec);
}

builtin_interfaces::msg::Time to_ros_time(int64_t stamp_ns)
{
  builtin_interfaces::msg::Time t;
  if (stamp_ns < 0) {
    stamp_ns = 0;
  }
  t.sec = static_cast<int32_t>(stamp_ns / 1'000'000'000LL);
  t.nanosec = static_cast<uint32_t>(stamp_ns % 1'000'000'000LL);
  return t;
}

}  // namespace

class ContinuousTimeNode : public rclcpp::Node
{
public:
  ContinuousTimeNode()
  : rclcpp::Node("continuous_time_node")
  {
    raw_imu_topic_ = declare_parameter<std::string>(
      "raw_imu_topic", "/imu_for_gs");
    external_odometry_prior_topic_ = declare_parameter<std::string>(
      "external_odometry_prior_topic", "");
    enable_external_odometry_prior_ =
      declare_parameter<bool>("enable_external_odometry_prior", false);
    // Mount rotation: q_imu_in_prior, expressed in (x, y, z, w).
    // For datasets whose ground truth is in a robot-base frame that differs
    // from the IMU sensor frame (e.g. M2DGR uses a base frame rotated ~180°
    // about x relative to the IMU), this rotation gets right-multiplied
    // into the prior's pose before it becomes the seed orientation.
    const auto prior_to_imu_param = declare_parameter<std::vector<double>>(
      "prior_to_imu_rotation_xyzw", std::vector<double>{0.0, 0.0, 0.0, 1.0});
    if (prior_to_imu_param.size() == 4) {
      prior_to_imu_rotation_ = Eigen::Quaterniond(
        prior_to_imu_param[3],
        prior_to_imu_param[0],
        prior_to_imu_param[1],
        prior_to_imu_param[2]).normalized();
    }
    odometry_topic_ = declare_parameter<std::string>(
      "odometry_topic", "/gaussian_lic/continuous_time/odometry");
    path_topic_ = declare_parameter<std::string>(
      "path_topic", "/gaussian_lic/continuous_time/path");
    body_frame_id_ = declare_parameter<std::string>("body_frame_id", "imu_link");
    world_frame_id_ = declare_parameter<std::string>("world_frame_id", "map");

    spline::ContinuousTimeSlidingWindowOptions options;
    options.dt_s = declare_parameter<double>("knot_interval_seconds", 0.05);
    options.window_knot_count =
      static_cast<int>(declare_parameter<int>("window_knot_count", 8));
    options.marginalize_oldest_count =
      static_cast<int>(declare_parameter<int>("marginalize_oldest_count", 1));
    options.max_iterations_per_step =
      static_cast<int>(declare_parameter<int>("max_iterations_per_step", 12));
    options.lidar_huber_delta_m =
      declare_parameter<double>("lidar_huber_delta_m", 0.10);
    lidar_huber_delta_m_ = options.lidar_huber_delta_m;
    const auto gravity_param =
      declare_parameter<std::vector<double>>(
      "gravity_world", std::vector<double>{0.0, 0.0, -9.81});
    if (gravity_param.size() == 3) {
      options.gravity_world =
        Eigen::Vector3d(gravity_param[0], gravity_param[1], gravity_param[2]);
    }
    options.hold_gravity_constant =
      declare_parameter<bool>("hold_gravity_constant", true);
    options.hold_accel_bias_constant =
      declare_parameter<bool>("hold_accel_bias_constant", false);
    options.hold_gyro_bias_constant =
      declare_parameter<bool>("hold_gyro_bias_constant", false);
    options.max_position_update_m =
      declare_parameter<double>("max_position_update_m", 2.0);
    options.max_rotation_update_rad =
      declare_parameter<double>("max_rotation_update_rad", 0.50);

    step_period_seconds_ =
      declare_parameter<double>("step_period_seconds", 0.10);
    seed_min_imu_count_ =
      static_cast<int>(declare_parameter<int>("seed_min_imu_count", 25));
    max_path_history_ =
      static_cast<int>(declare_parameter<int>("max_path_history", 5000));

    output_tum_path_ = declare_parameter<std::string>("output_tum_path", "");
    if (!output_tum_path_.empty()) {
      output_tum_stream_.open(output_tum_path_, std::ios::out | std::ios::trunc);
      if (!output_tum_stream_) {
        RCLCPP_WARN(
          get_logger(),
          "could not open output_tum_path '%s' for writing",
          output_tum_path_.c_str());
      } else {
        output_tum_stream_ << "# stamp_s tx ty tz qx qy qz qw" << std::endl;
        output_tum_stream_.flush();
      }
    }

    raw_pointcloud_topic_ = declare_parameter<std::string>(
      "raw_pointcloud_topic", "/points_for_gs");
    pointcloud_enable_ =
      declare_parameter<bool>("pointcloud_enable", true);
    pointcloud_subsample_stride_ =
      static_cast<int>(declare_parameter<int>("pointcloud_subsample_stride", 50));
    pointcloud_max_points_per_msg_ =
      static_cast<int>(declare_parameter<int>("pointcloud_max_points_per_msg", 256));
    pointcloud_min_range_m_ =
      declare_parameter<double>("pointcloud_min_range_m", 0.3);
    pointcloud_max_range_m_ =
      declare_parameter<double>("pointcloud_max_range_m", 30.0);
    pointcloud_factor_weight_ =
      declare_parameter<double>("pointcloud_factor_weight", 0.1);

    enable_imu_gravity_autocal_ =
      declare_parameter<bool>("enable_imu_gravity_autocal", true);
    // Default off for launches: voxel-plane factors are useful only when the
    // persistent world-frame plane map has enough observations to avoid the
    // old same-frame identity constraint. Parity scripts enable this path
    // explicitly while the tracker is still being RMSE-tuned.
    enable_voxel_plane_extraction_ =
      declare_parameter<bool>("enable_voxel_plane_extraction", false);
    enable_persistent_plane_map_ =
      declare_parameter<bool>("enable_persistent_plane_map", true);
    enable_persistent_point_map_ =
      declare_parameter<bool>("enable_persistent_point_map", false);
    persistent_point_map_nearest_distance_m_ =
      declare_parameter<double>("persistent_point_map_nearest_distance_m", 0.35);
    persistent_point_map_factor_weight_ =
      declare_parameter<double>("persistent_point_map_factor_weight", 0.05);
    persistent_point_map_subsample_stride_ = static_cast<int>(
      declare_parameter<int>("persistent_point_map_subsample_stride", 20));
    persistent_point_map_max_points_ = static_cast<int>(
      declare_parameter<int>("persistent_point_map_max_points", 20000));
    persistent_point_map_max_correspondences_ = static_cast<int>(
      declare_parameter<int>("persistent_point_map_max_correspondences", 64));
    spline::LidarPlaneExtractorOptions extractor_options;
    extractor_options.voxel_size_m =
      declare_parameter<double>("voxel_plane_size_m", 0.5);
    extractor_options.min_points_per_voxel = static_cast<int>(
      declare_parameter<int>("voxel_plane_min_points", 8));
    extractor_options.planar_eigenvalue_ratio =
      declare_parameter<double>("voxel_plane_eigen_ratio", 0.05);
    extractor_options.max_inlier_distance_m =
      declare_parameter<double>("voxel_plane_max_inlier_m", 0.1);
    extractor_options.max_correspondences = static_cast<int>(
      declare_parameter<int>("voxel_plane_max_correspondences", 64));
    extractor_options.min_range_m = pointcloud_min_range_m_;
    extractor_options.max_range_m = pointcloud_max_range_m_;
    plane_extractor_.set_options(extractor_options);
    spline::PersistentPlaneMapOptions plane_map_options;
    plane_map_options.max_planes = static_cast<int>(
      declare_parameter<int>("persistent_plane_map_max_planes", 512));
    plane_map_options.max_point_to_plane_distance_m =
      declare_parameter<double>("persistent_plane_map_match_distance_m", 0.25);
    plane_map_options.min_normal_dot =
      declare_parameter<double>("persistent_plane_map_min_normal_dot", 0.95);
    plane_map_options.min_observations_for_match = static_cast<int>(
      declare_parameter<int>("persistent_plane_map_min_observations_for_match", 3));
    persistent_plane_map_.set_options(plane_map_options);

    const auto plane_param = declare_parameter<std::vector<double>>(
      "lidar_ground_plane", std::vector<double>{0.0, 0.0, 1.0, 0.0});
    if (plane_param.size() == 4) {
      lidar_plane_ << plane_param[0], plane_param[1], plane_param[2], plane_param[3];
      const double n_norm = lidar_plane_.head<3>().norm();
      if (n_norm > 1.0e-9) {
        lidar_plane_ /= n_norm;
      }
    }

    const auto extrinsic_translation = declare_parameter<std::vector<double>>(
      "lidar_to_imu_translation", std::vector<double>{0.0, 0.0, 0.0});
    if (extrinsic_translation.size() == 3) {
      lidar_to_imu_translation_ = Eigen::Vector3d(
        extrinsic_translation[0],
        extrinsic_translation[1],
        extrinsic_translation[2]);
    }
    const auto extrinsic_rotation_xyzw = declare_parameter<std::vector<double>>(
      "lidar_to_imu_rotation_xyzw", std::vector<double>{0.0, 0.0, 0.0, 1.0});
    if (extrinsic_rotation_xyzw.size() == 4) {
      lidar_to_imu_rotation_ = Eigen::Quaterniond(
        extrinsic_rotation_xyzw[3],
        extrinsic_rotation_xyzw[0],
        extrinsic_rotation_xyzw[1],
        extrinsic_rotation_xyzw[2]).normalized();
    }

    estimator_ =
      std::make_unique<spline::ContinuousTimeSlidingWindowEstimator>(options);

    rclcpp::QoS imu_qos(rclcpp::KeepLast(200));
    imu_qos.best_effort();
    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      raw_imu_topic_, imu_qos,
      std::bind(&ContinuousTimeNode::on_imu, this, std::placeholders::_1));

    if (pointcloud_enable_) {
      rclcpp::QoS pc_qos(rclcpp::KeepLast(20));
      pc_qos.best_effort();
      pointcloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        raw_pointcloud_topic_, pc_qos,
        std::bind(&ContinuousTimeNode::on_pointcloud, this, std::placeholders::_1));
    }

    if (enable_external_odometry_prior_ && !external_odometry_prior_topic_.empty()) {
      rclcpp::QoS prior_qos(rclcpp::KeepLast(20));
      prior_qos.reliable();
      external_odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
        external_odometry_prior_topic_, prior_qos,
        std::bind(&ContinuousTimeNode::on_external_odometry_prior, this, std::placeholders::_1));
    }

    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
      odometry_topic_, rclcpp::QoS(50));
    path_publisher_ = create_publisher<nav_msgs::msg::Path>(
      path_topic_, rclcpp::QoS(10));

    step_timer_ = create_wall_timer(
      std::chrono::duration<double>(step_period_seconds_),
      std::bind(&ContinuousTimeNode::on_step_timer, this));

    RCLCPP_INFO(
      get_logger(),
      "continuous_time_node ready (imu=%s, odom=%s, dt=%.3fs, window=%d knots)",
      raw_imu_topic_.c_str(), odometry_topic_.c_str(),
      options.dt_s, options.window_knot_count);

    step_period_ns_ = static_cast<int64_t>(std::llround(step_period_seconds_ * 1.0e9));
    knot_interval_ns_ =
      static_cast<int64_t>(std::llround(options.dt_s * 1.0e9));
  }

private:
  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    if (!msg) {
      return;
    }
    const int64_t stamp_ns = to_signed_nanoseconds(msg->header.stamp);
    if (stamp_ns <= 0) {
      return;
    }
    if (!last_imu_stamp_valid_ || stamp_ns > last_imu_stamp_ns_) {
      last_imu_stamp_ns_ = stamp_ns;
      last_imu_stamp_valid_ = true;
    } else {
      // Non-monotonic stamp — drop to preserve estimator semantics.
      ++dropped_imu_count_;
      return;
    }

    spline::ImuSample sample;
    sample.gyro = Eigen::Vector3d(
      msg->angular_velocity.x,
      msg->angular_velocity.y,
      msg->angular_velocity.z);
    sample.accel = Eigen::Vector3d(
      msg->linear_acceleration.x,
      msg->linear_acceleration.y,
      msg->linear_acceleration.z);
    if (!sample.gyro.allFinite() || !sample.accel.allFinite()) {
      ++rejected_imu_count_;
      return;
    }

    std::lock_guard<std::mutex> lock(estimator_mutex_);
    if (!initialized_) {
      seed_imu_buffer_.push_back({stamp_ns, sample});
      if (static_cast<int>(seed_imu_buffer_.size()) >= seed_min_imu_count_) {
        seed_window_from_buffer();
        initialized_ = true;
      }
      return;
    }
    estimator_->add_imu_sample(stamp_ns, sample);
    ++accepted_imu_count_;
  }

  void on_external_odometry_prior(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!msg) {
      return;
    }
    const auto & pose = msg->pose.pose;
    Eigen::Vector3d p(pose.position.x, pose.position.y, pose.position.z);
    Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x,
      pose.orientation.y, pose.orientation.z);
    if (!p.allFinite() || !q.coeffs().allFinite() ||
      q.norm() < 1.0e-6)
    {
      ++rejected_prior_count_;
      return;
    }
    std::lock_guard<std::mutex> lock(estimator_mutex_);
    if (initialized_) {
      return;   // Prior only used for the seed; ignore subsequent messages.
    }
    latest_prior_position_ = p;
    latest_prior_orientation_ = q.normalized();
    have_prior_pose_ = true;
    ++accepted_prior_count_;
  }

  void on_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (!msg || !pointcloud_enable_) {
      return;
    }
    const int64_t stamp_ns = to_signed_nanoseconds(msg->header.stamp);
    if (stamp_ns <= 0) {
      return;
    }
    std::lock_guard<std::mutex> lock(estimator_mutex_);
    if (!initialized_) {
      return;
    }
    spline::LidarExtrinsics extrinsics;
    extrinsics.q_lidar_to_imu = lidar_to_imu_rotation_;
    extrinsics.p_lidar_in_imu = lidar_to_imu_translation_;

    if (enable_voxel_plane_extraction_) {
      std::vector<Eigen::Vector3d> points;
      points.reserve(static_cast<std::size_t>(msg->width) * msg->height / 4);
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        const float fx = *iter_x;
        const float fy = *iter_y;
        const float fz = *iter_z;
        if (!std::isfinite(fx) || !std::isfinite(fy) || !std::isfinite(fz)) {
          continue;
        }
        points.emplace_back(
          static_cast<double>(fx),
          static_cast<double>(fy),
          static_cast<double>(fz));
      }
      const auto planes = plane_extractor_.extract(points);

      // Transform each LiDAR-frame plane to world frame via the estimator's
      // current pose at the scan stamp. World-frame planes become stationary
      // constraints — the factor then truly pulls the trajectory back when
      // it drifts, instead of being a same-frame identity check.
      Eigen::Quaterniond q_b_w_at_scan = Eigen::Quaterniond::Identity();
      Eigen::Vector3d p_b_w_at_scan = Eigen::Vector3d::Zero();
      const bool have_scan_pose =
        estimator_->query_pose(stamp_ns, q_b_w_at_scan, p_b_w_at_scan);
      // Compose: world = R_b_w * R_l_b * p_lidar + R_b_w * p_l_b + p_b_w
      //                = R_w_l * p_lidar + p_w_l   (where R_w_l = R_b_w * R_l_b,
      //                                                  p_w_l = R_b_w * p_l_b + p_b_w)
      // Plane in LiDAR frame: n_l^T p_l + d_l = 0
      // After substituting p_l = R_w_l^T (p_w - p_w_l):
      //   n_l^T R_w_l^T (p_w - p_w_l) + d_l = 0
      //   (R_w_l n_l)^T p_w - (R_w_l n_l)^T p_w_l + d_l = 0
      // So n_world = R_w_l n_l, d_world = d_l - n_world^T p_w_l.
      const Eigen::Matrix3d R_l_b = extrinsics.q_lidar_to_imu.toRotationMatrix();
      const Eigen::Vector3d p_l_b = extrinsics.p_lidar_in_imu;
      const Eigen::Matrix3d R_b_w = q_b_w_at_scan.toRotationMatrix();
      const Eigen::Matrix3d R_w_l = R_b_w * R_l_b;
      const Eigen::Vector3d p_w_l = R_b_w * p_l_b + p_b_w_at_scan;

      int accepted = 0;
      if (enable_persistent_point_map_ && have_scan_pose) {
        accepted += add_persistent_point_map_correspondences(
          stamp_ns, points, R_w_l, p_w_l, extrinsics);
      }
      for (const auto & plane : planes) {
        Eigen::Vector3d n_world = Eigen::Vector3d::Zero();
        Eigen::Vector3d centroid_world = Eigen::Vector3d::Zero();
        double d_world = 0.0;
        bool has_world_plane = false;
        if (have_scan_pose) {
          n_world = (R_w_l * plane.normal).normalized();
          centroid_world = R_w_l * plane.centroid + p_w_l;
          d_world = plane.offset - n_world.dot(p_w_l);
          has_world_plane = n_world.allFinite() && centroid_world.allFinite() &&
            std::isfinite(d_world);
        }

        spline::LidarPointCorrespondence pc;
        pc.geometry = spline::LidarFeatureGeometry::kPlane;
        pc.point_lidar = plane.sample_point;
        if (enable_persistent_plane_map_ && has_world_plane) {
          const auto match = persistent_plane_map_.match(centroid_world, n_world);
          if (match) {
            pc.plane = match->plane;
            estimator_->add_lidar_correspondence(
              stamp_ns, pc, extrinsics, pointcloud_factor_weight_,
              lidar_huber_delta_m_);
            ++accepted;
            ++persistent_plane_map_matches_;
          }
          if (persistent_plane_map_.add_or_update(centroid_world, n_world, d_world)) {
            ++persistent_plane_map_updates_;
          }
          continue;
        }
        if (has_world_plane) {
          pc.plane.head<3>() = n_world;
          pc.plane[3] = d_world;
        } else {
          // Falling back to LiDAR-frame plane until the estimator has a
          // valid pose at the scan stamp — this still degenerates into
          // a same-frame identity for the first few frames.
          pc.plane.head<3>() = plane.normal;
          pc.plane[3] = plane.offset;
        }
        estimator_->add_lidar_correspondence(
          stamp_ns, pc, extrinsics, pointcloud_factor_weight_,
          lidar_huber_delta_m_);
        ++accepted;
      }
      accepted_pointcloud_correspondences_ += static_cast<std::size_t>(accepted);
      ++pointcloud_messages_;
      return;
    }

    spline::LidarPointCorrespondence pc;
    pc.geometry = spline::LidarFeatureGeometry::kPlane;
    pc.plane = lidar_plane_;

    int accepted = 0;
    int stride_counter = 0;
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      if (stride_counter++ % std::max(1, pointcloud_subsample_stride_) != 0) {
        continue;
      }
      const float fx = *iter_x;
      const float fy = *iter_y;
      const float fz = *iter_z;
      if (!std::isfinite(fx) || !std::isfinite(fy) || !std::isfinite(fz)) {
        continue;
      }
      const double rx = static_cast<double>(fx);
      const double ry = static_cast<double>(fy);
      const double rz = static_cast<double>(fz);
      const double range = std::sqrt(rx * rx + ry * ry + rz * rz);
      if (range < pointcloud_min_range_m_ || range > pointcloud_max_range_m_) {
        continue;
      }
      pc.point_lidar = Eigen::Vector3d(rx, ry, rz);
      estimator_->add_lidar_correspondence(
        stamp_ns, pc, extrinsics, pointcloud_factor_weight_,
        lidar_huber_delta_m_);
      ++accepted;
      if (pointcloud_max_points_per_msg_ > 0 &&
        accepted >= pointcloud_max_points_per_msg_)
      {
        break;
      }
    }
    accepted_pointcloud_correspondences_ += static_cast<std::size_t>(accepted);
    ++pointcloud_messages_;
  }

  void on_step_timer()
  {
    std::lock_guard<std::mutex> lock(estimator_mutex_);
    if (!initialized_) {
      return;
    }
    const bool stepped = estimator_->step();
    if (!stepped) {
      return;
    }
    const auto & diagnostics = estimator_->diagnostics();
    if (diagnostics.rejected_solver_steps > last_logged_rejected_solver_steps_) {
      last_logged_rejected_solver_steps_ = diagnostics.rejected_solver_steps;
      RCLCPP_WARN(
        get_logger(),
        "continuous-time solve update rejected: total=%zu max_dp=%.3f m max_dtheta=%.3f rad",
        diagnostics.rejected_solver_steps,
        diagnostics.last_rejected_position_update_m,
        diagnostics.last_rejected_rotation_update_rad);
    }
    publish_latest_pose();
  }

  bool find_nearest_persistent_point(
    const Eigen::Vector3d & point_world,
    Eigen::Vector3d & nearest_world,
    double & nearest_distance_sq) const
  {
    if (persistent_points_world_.empty() || !point_world.allFinite()) {
      return false;
    }
    const double max_distance_sq =
      persistent_point_map_nearest_distance_m_ * persistent_point_map_nearest_distance_m_;
    nearest_distance_sq = max_distance_sq;
    bool found = false;
    for (const auto & candidate : persistent_points_world_) {
      const double distance_sq = (candidate - point_world).squaredNorm();
      if (distance_sq <= nearest_distance_sq) {
        nearest_distance_sq = distance_sq;
        nearest_world = candidate;
        found = true;
      }
    }
    return found;
  }

  int add_persistent_point_map_correspondences(
    int64_t stamp_ns,
    const std::vector<Eigen::Vector3d> & points_lidar,
    const Eigen::Matrix3d & R_w_l,
    const Eigen::Vector3d & p_w_l,
    const spline::LidarExtrinsics & extrinsics)
  {
    if (!R_w_l.allFinite() || !p_w_l.allFinite()) {
      return 0;
    }
    int accepted = 0;
    int stride_counter = 0;
    for (const auto & point_lidar : points_lidar) {
      if (persistent_point_map_subsample_stride_ > 1 &&
        stride_counter++ % persistent_point_map_subsample_stride_ != 0)
      {
        continue;
      }
      const double range = point_lidar.norm();
      if (!std::isfinite(range) || range < pointcloud_min_range_m_ ||
        range > pointcloud_max_range_m_)
      {
        continue;
      }
      const Eigen::Vector3d point_world = R_w_l * point_lidar + p_w_l;
      Eigen::Vector3d nearest_world = Eigen::Vector3d::Zero();
      double nearest_distance_sq = 0.0;
      if (find_nearest_persistent_point(point_world, nearest_world, nearest_distance_sq)) {
        for (int axis = 0; axis < 3; ++axis) {
          spline::LidarPointCorrespondence pc;
          pc.geometry = spline::LidarFeatureGeometry::kPlane;
          pc.point_lidar = point_lidar;
          pc.plane.setZero();
          pc.plane[axis] = 1.0;
          pc.plane[3] = -nearest_world[axis];
          estimator_->add_lidar_correspondence(
            stamp_ns, pc, extrinsics, persistent_point_map_factor_weight_,
            lidar_huber_delta_m_);
        }
        ++accepted;
        ++persistent_point_map_matches_;
        if (persistent_point_map_max_correspondences_ > 0 &&
          accepted >= persistent_point_map_max_correspondences_)
        {
          break;
        }
      }
      if (persistent_point_map_max_points_ <= 0 ||
        static_cast<int>(persistent_points_world_.size()) < persistent_point_map_max_points_)
      {
        persistent_points_world_.push_back(point_world);
        ++persistent_point_map_updates_;
      }
    }
    return accepted * 3;
  }

  void seed_window_from_buffer()
  {
    if (seed_imu_buffer_.empty() || !estimator_) {
      return;
    }
    const int64_t start_stamp = seed_imu_buffer_.front().first;

    // Gravity autocalibration: assume the seed window is static, average
    // accelerometer samples to recover the gravity direction in body frame,
    // then derive a roll/pitch rotation that aligns body z with -gravity_world.
    // Yaw stays 0 (not observable from gravity alone). Mirrors the upstream
    // Coco-LIC / `tracking_node` `enable_imu_gravity_autocalibration` default.
    Eigen::Quaterniond seed_orientation = Eigen::Quaterniond::Identity();
    Eigen::Vector3d seed_position = Eigen::Vector3d::Zero();

    // External pose prior takes precedence over gravity autocal — if a
    // ground-truth or external SLAM frontend has been publishing on
    // `external_odometry_prior_topic`, use the most recent pose as the
    // seed instead of identity. The prior is composed with the configured
    // IMU mount rotation so the resulting orientation expresses the IMU
    // body in the world frame the estimator works in.
    if (enable_external_odometry_prior_ && have_prior_pose_) {
      seed_orientation =
        (latest_prior_orientation_ * prior_to_imu_rotation_).normalized();
      seed_position = latest_prior_position_;
      RCLCPP_INFO(
        get_logger(),
        "seed from external prior + mount: p=(%.3f, %.3f, %.3f) q=(%.3f, %.3f, %.3f, %.3f)",
        seed_position.x(), seed_position.y(), seed_position.z(),
        seed_orientation.w(), seed_orientation.x(),
        seed_orientation.y(), seed_orientation.z());
    } else if (enable_imu_gravity_autocal_) {
      Eigen::Vector3d accel_sum = Eigen::Vector3d::Zero();
      for (const auto & sample : seed_imu_buffer_) {
        accel_sum += sample.second.accel;
      }
      const Eigen::Vector3d accel_mean =
        accel_sum / static_cast<double>(seed_imu_buffer_.size());
      const double accel_mean_norm = accel_mean.norm();
      if (accel_mean_norm > 1.0e-3) {
        const Eigen::Vector3d gravity_world = estimator_->gravity_world();
        const double gravity_world_norm = gravity_world.norm();
        if (gravity_world_norm > 1.0e-3) {
          // Stationary accel reads `R_b_w^T * (-g_w)` (no motion + zero bias
          // initially). So `R_b_w^T * (-g_w) = accel_mean` → we need a
          // rotation that maps `-g_w/|g_w|` (body z when level) onto the
          // measured direction `accel_mean/|accel_mean|`.
          const Eigen::Vector3d gravity_dir = (-gravity_world / gravity_world_norm);
          const Eigen::Vector3d accel_dir = accel_mean / accel_mean_norm;
          seed_orientation = Eigen::Quaterniond::FromTwoVectors(
            gravity_dir, accel_dir).inverse().normalized();
        }
      }
      RCLCPP_INFO(
        get_logger(),
        "gravity autocal: accel_mean=(%.3f, %.3f, %.3f) -> q_w_b=(%.3f, %.3f, %.3f, %.3f)",
        accel_mean.x(), accel_mean.y(), accel_mean.z(),
        seed_orientation.w(), seed_orientation.x(),
        seed_orientation.y(), seed_orientation.z());
    }

    std::vector<Eigen::Quaterniond> rot_knots(
      static_cast<std::size_t>(estimator_->N + 4),
      seed_orientation);
    std::vector<Eigen::Vector3d> pos_knots(
      rot_knots.size(), seed_position);
    estimator_->initialize(start_stamp, rot_knots, pos_knots);

    for (const auto & sample : seed_imu_buffer_) {
      estimator_->add_imu_sample(sample.first, sample.second);
    }
    estimator_->step();
    seed_imu_buffer_.clear();
    RCLCPP_INFO(
      get_logger(),
      "continuous_time_node seeded window with %d IMU samples", seed_min_imu_count_);
  }

  void publish_latest_pose()
  {
    if (!estimator_) {
      return;
    }
    const int64_t newest = estimator_->newest_knot_stamp_ns();
    const int64_t query_ns = newest - 2 * knot_interval_ns_;
    // Rate limit: only publish when query stamp advances. The Ceres step
    // can fire many times per timer batch when the executor catches up
    // after a busy IMU/PointCloud callback; without this guard, the same
    // pose can be emitted hundreds of times.
    if (last_published_query_ns_ != 0 && query_ns <= last_published_query_ns_) {
      return;
    }
    last_published_query_ns_ = query_ns;
    Eigen::Quaterniond q;
    Eigen::Vector3d p;
    if (!estimator_->query_pose(query_ns, q, p)) {
      return;
    }
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = to_ros_time(query_ns);
    odom.header.frame_id = world_frame_id_;
    odom.child_frame_id = body_frame_id_;
    odom.pose.pose.position.x = p.x();
    odom.pose.pose.position.y = p.y();
    odom.pose.pose.position.z = p.z();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    odom_publisher_->publish(odom);

    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = odom.header;
    pose_stamped.pose = odom.pose.pose;
    if (static_cast<int>(published_path_.poses.size()) >= max_path_history_) {
      published_path_.poses.erase(
        published_path_.poses.begin(),
        published_path_.poses.begin() +
        static_cast<int>(published_path_.poses.size()) - max_path_history_ + 1);
    }
    published_path_.header = pose_stamped.header;
    published_path_.poses.push_back(pose_stamped);
    path_publisher_->publish(published_path_);

    if (output_tum_stream_.is_open()) {
      const double stamp_s = static_cast<double>(query_ns) * 1.0e-9;
      const double position_bound = 1.0e6;
      if (p.allFinite() && q.coeffs().allFinite() &&
        std::abs(p.x()) < position_bound &&
        std::abs(p.y()) < position_bound &&
        std::abs(p.z()) < position_bound)
      {
        std::ostringstream line;
        line.setf(std::ios::fixed);
        line.precision(9);
        line << stamp_s;
        line.precision(6);
        line << ' ' << p.x() << ' ' << p.y() << ' ' << p.z();
        line << ' ' << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w();
        output_tum_stream_ << line.str() << '\n';
        output_tum_stream_.flush();
        ++tum_lines_written_;
      }
    }
  }

  std::string raw_imu_topic_;
  std::string raw_pointcloud_topic_;
  std::string external_odometry_prior_topic_;
  std::string odometry_topic_;
  std::string path_topic_;
  std::string body_frame_id_;
  std::string world_frame_id_;

  std::unique_ptr<spline::ContinuousTimeSlidingWindowEstimator> estimator_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr external_odometry_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::TimerBase::SharedPtr step_timer_;

  std::mutex estimator_mutex_;
  bool initialized_{false};
  std::vector<std::pair<int64_t, spline::ImuSample>> seed_imu_buffer_;
  int seed_min_imu_count_{25};
  int max_path_history_{5000};
  nav_msgs::msg::Path published_path_;

  bool last_imu_stamp_valid_{false};
  int64_t last_imu_stamp_ns_{0};
  std::size_t accepted_imu_count_{0};
  std::size_t dropped_imu_count_{0};
  std::size_t rejected_imu_count_{0};

  std::string output_tum_path_;
  std::ofstream output_tum_stream_;
  std::size_t tum_lines_written_{0};

  bool pointcloud_enable_{true};
  int pointcloud_subsample_stride_{50};
  int pointcloud_max_points_per_msg_{256};
  double pointcloud_min_range_m_{0.3};
  double pointcloud_max_range_m_{30.0};
  double pointcloud_factor_weight_{0.1};
  double lidar_huber_delta_m_{0.10};
  std::size_t accepted_pointcloud_correspondences_{0};
  std::size_t pointcloud_messages_{0};

  Eigen::Vector4d lidar_plane_{0.0, 0.0, 1.0, 0.0};
  Eigen::Vector3d lidar_to_imu_translation_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond lidar_to_imu_rotation_{Eigen::Quaterniond::Identity()};

  bool enable_imu_gravity_autocal_{true};
  bool enable_voxel_plane_extraction_{false};
  bool enable_persistent_plane_map_{true};
  bool enable_external_odometry_prior_{false};
  bool have_prior_pose_{false};
  Eigen::Vector3d latest_prior_position_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond latest_prior_orientation_{Eigen::Quaterniond::Identity()};
  Eigen::Quaterniond prior_to_imu_rotation_{Eigen::Quaterniond::Identity()};
  std::size_t accepted_prior_count_{0};
  std::size_t rejected_prior_count_{0};
  spline::LidarPlaneExtractor plane_extractor_{};
  spline::PersistentPlaneMap persistent_plane_map_{};
  std::size_t persistent_plane_map_matches_{0};
  std::size_t persistent_plane_map_updates_{0};
  bool enable_persistent_point_map_{false};
  double persistent_point_map_nearest_distance_m_{0.35};
  double persistent_point_map_factor_weight_{0.05};
  int persistent_point_map_subsample_stride_{20};
  int persistent_point_map_max_points_{20000};
  int persistent_point_map_max_correspondences_{64};
  std::vector<Eigen::Vector3d> persistent_points_world_;
  std::size_t persistent_point_map_matches_{0};
  std::size_t persistent_point_map_updates_{0};

  double step_period_seconds_{0.10};
  int64_t step_period_ns_{0};
  int64_t knot_interval_ns_{50000000};
  int64_t last_published_query_ns_{0};
  std::size_t last_logged_rejected_solver_steps_{0};
};

}  // namespace gaussian_lic_tracking

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<gaussian_lic_tracking::ContinuousTimeNode>());
  rclcpp::shutdown();
  return 0;
}
