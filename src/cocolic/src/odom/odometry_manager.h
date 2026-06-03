/*
 * Coco-LIC: Continuous-Time Tightly-Coupled LiDAR-Inertial-Camera Odometry using Non-Uniform B-spline
 * Copyright (C) 2023 Xiaolei Lang
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

// ROS2 Jazzy port: dropped <ros/ros.h>. Camera front-end = slimmed R3LIVE
// (camera/r3live_slim.hpp) — the visual-front-end subset (optical-flow tracking
// + RGB map) feeding the ported image_feature_factor; full LICO for sub-cm.
#include <odom/msg_manager.h>
#include <odom/odometry_viewer.h>
#include <odom/trajectory_manager.h>

#include <imu/imu_state_estimator.h>
#include <imu/imu_initializer.h>
#include <lidar/lidar_handler.h>

#include <condition_variable>
#include <mutex>
#include <thread>

#include <camera/r3live_slim.hpp>

// GL2 step-1: write the /*_for_gs streams (track A's stable poses) to a
// mapper_contract rosbag2 -> replay to the CUDA 3DGS mapper -> PSNR.
// GL2 step-2: ALSO publish them live (gs_live_publish) so the mapper runs
// concurrently and feeds /gaussian_lic/rendered_feedback back into track A's BA.
#include <rosbag2_cpp/writer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <gaussian_lic_msgs/msg/rendered_feedback.hpp>
#include <map>
#include <mutex>

namespace cocolic
{

  enum KnotDensity
  {
    gear1 = 1, // 0.1
    gear2 = 2, // 0.05
    gear3 = 3, // 0.033
    gear4 = 4  // 0.025
  };
  class OdometryManager
  {
  public:
    OdometryManager(const YAML::Node &node);

    void RunBag();

    void RunInSubscribeMode();

    double SaveOdometry();

    std::vector<int> cp_num_vec;

    int GetKnotDensity(double gyro_norm, double acce_norm)
    {
      if (gyro_norm < 0.0 || acce_norm < 0.0)
      {
        std::cout << RED << "gyro_norm/acce_norm is wrong!" << RESET << std::endl;
      }

      int gyro_density = -1, acce_density = -1;

      acce_norm = std::abs(acce_norm - gravity_norm_);
      // LOG(INFO) << "[acce_norm] " << acce_norm;
      if (acce_norm < 0.5)
      { // [0, 0.5)
        acce_density = KnotDensity::gear1;
      }
      else if (acce_norm < 1.0)
      { // [0.5, 1.0)
        acce_density = KnotDensity::gear2;
      }
      else if (acce_norm < 5.0)
      { // [1.0, 5.0)
        acce_density = KnotDensity::gear3;
      }
      else
      { // [5.0, -)
        acce_density = KnotDensity::gear4;
      }

      // LOG(INFO) << "[gyro_norm] " << gyro_norm;
      if (gyro_norm < 0.5)
      { // [0, 0.5)
        gyro_density = KnotDensity::gear1;
      }
      else if (gyro_norm < 1.0)
      { // [0.5, 1.0)
        gyro_density = KnotDensity::gear2;
      }
      else if (gyro_norm < 5.0)
      { // [1.0, 5.0)
        gyro_density = KnotDensity::gear3;
      }
      else
      { // [5.0, -)
        gyro_density = KnotDensity::gear4;
      }

      return std::max(gyro_density, acce_density);
    };

  protected:
    bool CreateCacheFolder(const std::string &config_path,
                           const std::string &bag_path);

    void SolveLICO();

    void ProcessLICData();

    void ProcessImageData();

    bool PrepareTwoSegMsgs(int seg_idx);

    void UpdateTwoSeg();

    bool PrepareMsgs();

    void UpdateOneSeg();

    void SetInitialState();

    void PublishCloudAndTrajectory();

    void Publish3DGSMappingData(const NextMsgs& cur_msg);

    // GL2 step-1: for_gs mapper-contract bag writer (track A -> 3DGS mapper).
    void OpenForGsWriter(const std::string &out_dir);

    // GL2 step-2: create live /*_for_gs publishers for concurrent mapper coupling.
    void OpenForGsLivePublishers();
    void WriteForGsFrame(const cv::Mat &img_bgr, const cv::Mat &depth32f,
                         const Eigen::Quaterniond &q_wc, const Eigen::Vector3d &t_wc,
                         const Eigen::aligned_vector<Eigen::Vector3d> &pts,
                         const Eigen::aligned_vector<Eigen::Vector3i> &rgb,
                         int64_t stamp_ns);

  protected:
    OdometryMode odometry_mode_;

    MsgManager::Ptr msg_manager_;

    bool is_initialized_;
    IMUInitializer::Ptr imu_initializer_;

    Trajectory::Ptr trajectory_;
    TrajectoryManager::Ptr trajectory_manager_;

    LidarHandler::Ptr lidar_handler_;

    R3LIVE::Ptr camera_handler_;

    int64_t t_begin_add_cam_; //

    OdometryViewer odom_viewer_;

    int update_every_k_knot_;

    /// [nurbs]
    double t_add_;
    int64_t t_add_ns_;
    bool non_uniform_;
    double distance0_;

    int cp_add_num_coarse_;
    int cp_add_num_refine_;

    int lidar_iter_;
    bool use_lidar_scale_;

    std::string cache_path_;

    double pasue_time_;

    TimeStatistics time_summary_;

    struct SysTimeOffset
    {
      SysTimeOffset(double t1, double t2, double t3, double t4)
          : timestamp(t1), t_lidar(t2), t_cam(t3), t_imu(t4) {}
      double timestamp = 0;
      double t_lidar = 0;
      double t_cam = 0;
      double t_imu = 0;
    };
    std::vector<SysTimeOffset> sys_t_offset_vec_;

  private:
    double gravity_norm_;

    int64_t traj_max_time_ns_cur;
    int64_t traj_max_time_ns_next;
    int64_t traj_max_time_ns_next_next;

    int cp_add_num_cur;
    int cp_add_num_next;
    int cp_add_num_next_next;

    bool is_evo_viral_;

    double ave_r_thresh_;
    double ave_a_thresh_;

    VPointCloud sub_map_cur_frame_point_;

    Eigen::aligned_vector<Eigen::Vector3d> v_points_;
    Eigen::aligned_vector<Eigen::Vector2d> px_obss_;

    Eigen::Matrix3d K_;

    bool if_3dgs_;
    int lidar_skip_;

    // GL2 step-1 for_gs writer state
    std::shared_ptr<rosbag2_cpp::Writer> forgs_writer_;
    bool forgs_open_ = false;

    // GL2 step-2 live-publish state (concurrent mapper coupling)
    bool gs_live_publish_ = false;
    rclcpp::Node::SharedPtr gs_node_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_gs_img_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_gs_depth_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_gs_caminfo_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_gs_pose_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_gs_points_;

    // GL2 step-2bc: rendered_feedback subscription + cache (closed-loop coupling).
    rclcpp::Subscription<gaussian_lic_msgs::msg::RenderedFeedback>::SharedPtr sub_gs_feedback_;
    std::mutex gs_fb_mutex_;
    std::map<int64_t, gaussian_lic_msgs::msg::RenderedFeedback> gs_feedback_;  // keyed by observed_stamp ns
    int64_t gs_fb_count_ = 0;
    int64_t gs_fb_last_observed_ns_ = 0;
    void SpinForGsFeedback();  // drain subscription + timing probe (call each RunBag iter)
    // GL2 lockstep pacing (OQ4 fix): after publishing a frame, wait until mapper
    // feedback catches to within max_lag of it — caps feedback staleness.
    bool gs_lockstep_ = false;
    int64_t gs_lockstep_max_lag_ns_ = 500000000;   // 0.5 s
    int64_t gs_lockstep_timeout_ms_ = 2000;        // 2.0 s
    void PaceForGsFeedback(int64_t pub_abs_ns);

    // GL2 step-2c: build per-frame render-photometric reference (rendered-map patches
    // at each pnp pixel) and hand it to trajectory_manager before the LIC solve.
    bool enable_render_photometric_ = false;
    double rp_weight_ = 0.5;
    int rp_patch_half_ = 2;
    void BuildRenderPhotometric();

    std::queue<int64_t> time_buf;  // img timestamp
    std::queue<LiDARFeature> lidar_buf;  // lidarfeature in local
    std::queue<cv::Mat> img_buf;  // undistorted
    std::vector<PosCloud::Ptr> lidarpoints;
  };

} // namespace cocolic
