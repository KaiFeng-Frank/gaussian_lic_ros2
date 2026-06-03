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

// ROS2 port: dropped <eigen_conversions/eigen_msg.h> (ROS1; its tf::*EigenToMsg
// users live in the stubbed viewer / #if 0'd camera-viz blocks).
#include <odom/odometry_manager.h>
#include <numeric>

#include <fstream>
#include <iomanip>
#include <string>
#include <sstream>
#include <filesystem>  // ROS2 port: std::filesystem (was boost::filesystem)
#include <cstring>     // GL2: memcpy for for_gs PointCloud2 packing
#include <chrono>     // GL2: lockstep pacing timing
#include <thread>     // GL2: sleep_for in lockstep pacing
#include <opencv2/imgproc.hpp>  // GL2: cv::cvtColor for render-photometric

std::fstream rgb_file;
std::fstream img_file;

namespace cocolic
{

  OdometryManager::OdometryManager(const YAML::Node &node)
      : odometry_mode_(LIO), is_initialized_(false)
  {
    // ROS2 port: project_path was a ROS param; now read from the config yaml.
    std::string config_path =
        node["project_path"] ? node["project_path"].as<std::string>() : "";
    config_path += "/config";

    std::string lidar_yaml = node["lidar_yaml"].as<std::string>();
    YAML::Node lidar_node = YAML::LoadFile(config_path + lidar_yaml);

    std::string imu_yaml = node["imu_yaml"].as<std::string>();
    YAML::Node imu_node = YAML::LoadFile(config_path + imu_yaml);

    std::string cam_yaml = config_path + node["camera_yaml"].as<std::string>();
    YAML::Node cam_node = YAML::LoadFile(cam_yaml);

    odometry_mode_ = OdometryMode(node["odometry_mode"].as<int>());
    std::cout << "\n🥥 Odometry Mode: ";
    if (odometry_mode_ == LICO)
    {
      std::cout << "LiDAR-Inertial-Camera Odometry 🥥" << std::endl;
    }
    else if (odometry_mode_ == LIO)
    {
      std::cout << "LiDAR-Inertial Odometry 🥥" << std::endl;
    }

    // extrinsic: sensor to imu
    ExtrinsicParam EP_LtoI, EP_CtoI, EP_ItoI, EP_MtoI;
    EP_LtoI.Init(lidar_node["lidar0"]["Extrinsics"]);
    if (odometry_mode_ == LICO)
      EP_CtoI.Init(cam_node["CameraExtrinsics"]);
    if (node["IMUExtrinsics"])
      EP_ItoI.Init(imu_node["IMUExtrinsics"]);
    EP_MtoI.Init(imu_node["MarkerExtrinsics"]);

    trajectory_ = std::make_shared<Trajectory>(-1, 0);
    trajectory_->SetSensorExtrinsics(SensorType::LiDARSensor, EP_LtoI);
    trajectory_->SetSensorExtrinsics(SensorType::CameraSensor, EP_CtoI);
    trajectory_->SetSensorExtrinsics(SensorType::IMUSensor, EP_ItoI);
    trajectory_->SetSensorExtrinsics(SensorType::Marker, EP_MtoI);

    // non-uniform b-spline
    t_add_ = node["t_add"].as<double>();
    t_add_ns_ = t_add_ * S_TO_NS;
    non_uniform_ = node["non_uniform"].as<bool>();
    distance0_ = node["distance0"].as<double>();

    // lidar
    lidar_iter_ = node["lidar_iter"].as<int>();
    use_lidar_scale_ = node["use_lidar_scale"].as<bool>();
    lidar_handler_ = std::make_shared<LidarHandler>(lidar_node, trajectory_);
    std::cout << "\n🍺 The number of multiple LiDARs is " << lidar_node["num_lidars"].as<int>() << "." << std::endl;

    // imu
    imu_initializer_ = std::make_shared<IMUInitializer>(imu_node);
    gravity_norm_ = imu_initializer_->GetGravity().norm();

    // camera
    camera_handler_ = std::make_shared<R3LIVE>(cam_node, EP_CtoI);
    t_begin_add_cam_ = node["t_begin_add_cam"].as<double>() * S_TO_NS;
    v_points_.clear();
    px_obss_.clear();
    double fx = cam_node["cam_fx"].as<double>();
    double fy = cam_node["cam_fy"].as<double>();
    double cx = cam_node["cam_cx"].as<double>();
    double cy = cam_node["cam_cy"].as<double>();
    K_ << fx, 0.0, cx,
        0.0, fy, cy,
        0.0, 0.0, 1.0;

    // trajectory parameterized by b-spline
    trajectory_manager_ = std::make_shared<TrajectoryManager>(node, config_path, trajectory_);
    trajectory_manager_->use_lidar_scale = use_lidar_scale_;
    trajectory_manager_->SetIntrinsic(K_);

    int division_coarse = node["division_coarse"].as<int>();
    cp_add_num_coarse_ = division_coarse;
    trajectory_manager_->SetDivisionParam(division_coarse, -1);

    odom_viewer_.SetPublisher();  // ROS2 port: stub no-op

    msg_manager_ = std::make_shared<MsgManager>(node, config_path);  // load rosbag

    // ROS2 port: pasue_time/verbose were ROS params → read from yaml or default.
    pasue_time_ = node["pasue_time"] ? node["pasue_time"].as<double>() : -1.0;
    bool verbose = node["verbose"] ? node["verbose"].as<bool>() : false;
    trajectory_manager_->verbose = verbose;

    // evaluation
    is_evo_viral_ = node["is_evo_viral"].as<bool>();
    CreateCacheFolder(config_path, msg_manager_->bag_path_);

    // gaussian-lic
    if_3dgs_ = node["if_3dgs"].as<bool>();
    lidar_skip_ = node["lidar_skip"].as<int>();
    // GL2 step-2: when true, publish /*_for_gs live to a concurrent mapper
    // (tight coupling) instead of writing the offline mapper_contract bag.
    gs_live_publish_ = node["gs_live_publish"] ? node["gs_live_publish"].as<bool>() : false;
    // GL2 lockstep pacing (OQ4 fix): pace track A to mapper feedback throughput.
    gs_lockstep_ = node["gs_lockstep"] ? node["gs_lockstep"].as<bool>() : false;
    if (node["gs_lockstep_max_lag_s"])
      gs_lockstep_max_lag_ns_ = (int64_t)(node["gs_lockstep_max_lag_s"].as<double>() * 1e9);
    if (node["gs_lockstep_timeout_s"])
      gs_lockstep_timeout_ms_ = (int64_t)(node["gs_lockstep_timeout_s"].as<double>() * 1e3);
    // GL2 step-2c: render-photometric coupling (rendered map → track A pose factor).
    enable_render_photometric_ = node["enable_render_photometric"] ? node["enable_render_photometric"].as<bool>() : false;
    if (node["render_photo_weight"]) rp_weight_ = node["render_photo_weight"].as<double>();
    if (node["render_photo_patch_half"]) rp_patch_half_ = node["render_photo_patch_half"].as<int>();
    // GL2 demo: time-windowed LiDAR degradation (good -> bad -> good) for an asymmetric
    // scenario where the good-segment map independently corrects the degraded segment.
    if (node["lidar_degrade_window_start_s"] && node["lidar_degrade_window_end_s"])
    {
      double ds = node["lidar_degrade_window_start_s"].as<double>();
      double de = node["lidar_degrade_window_end_s"].as<double>();
      double df = node["lidar_degrade_factor"] ? node["lidar_degrade_factor"].as<double>() : 1.0;
      trajectory_manager_->SetLidarDegradeWindow(ds, de, df);
      std::cout << "\n[GL2] LiDAR degrade window [" << ds << "," << de << ")s factor " << df << "\n";
    }
    lidarpoints.clear();

    std::cout << std::fixed << std::setprecision(4);
    // LOG(INFO) << std::fixed << std::setprecision(4);
  }

  bool OdometryManager::CreateCacheFolder(const std::string &config_path,
                                          const std::string &bag_path)
  {
    // ROS2 port: boost::filesystem → std::filesystem (C++17). Relaxed the
    // ".bag"-only check: the ROS2 bag is a directory (mcap/db3), so derive the
    // run name from the path's final component (filename, or stem if it has an
    // extension) and create the cache dir.
    namespace fs = std::filesystem;
    fs::path path_cfg(config_path);
    fs::path path_bag(bag_path);
    std::string bag_name_ = path_bag.has_extension() ? path_bag.stem().string()
                                                     : path_bag.filename().string();
    if (bag_name_.empty()) bag_name_ = "run";

    std::string cache_path_parent_ = path_cfg.parent_path().string();
    cache_path_ = cache_path_parent_ + "/data/" + bag_name_;
    std::error_code ec;
    fs::create_directories(cache_path_parent_ + "/data", ec);
    return true;
  }

  void OdometryManager::RunBag()
  {
    // ROS2 port: while(ros::ok()) → finite loop; SpinBagOnce sets has_valid_msg_
    // false at end-of-bag (break below).
    while (true)
    {
      /// [1] process a newly arrived frame of data: lidar or imu or camera
      msg_manager_->SpinBagOnce();
      if (!msg_manager_->has_valid_msg_)
      {
        break;
      }

      /// GL2 step-2bc: drain rendered_feedback from the concurrent mapper (no-op
      /// until gs_node_ exists, i.e. gs_live_publish mode after first image frame).
      SpinForGsFeedback();

      /// [2] static initialization, do not move at the begging!
      if (!is_initialized_)
      {
        while (!msg_manager_->imu_buf_.empty())
        {
          imu_initializer_->FeedIMUData(msg_manager_->imu_buf_.front());
          msg_manager_->imu_buf_.pop_front();
        }

        if (imu_initializer_->StaticInitialIMUState())
        {
          SetInitialState();
          std::cout << "\n🍺 Static initialization succeeds.\n";
          std::cout << "\n🍺 Trajectory start time: " << trajectory_->GetDataStartTime() << " ns.\n";
        }
        else
        {
          continue;
        }
      }

      /// [3] prepare data for the latest time interval delta_t
      static bool is_two_seg_prepared = false;
      static int seg_msg_cnt = 0;
      if (!is_two_seg_prepared)
      {
        if (PrepareTwoSegMsgs(seg_msg_cnt))  // prepare interval0 and interval1
        {
          seg_msg_cnt++;
        }
        if (seg_msg_cnt == 2)  // if interval0 and interval1 are ready
        {
          is_two_seg_prepared = true;
          UpdateTwoSeg();
          trajectory_->InitBlendMat();  // blending matrix is computed by knots of b-spline
        }
        else
        {
          continue;
        }
      }

      /// [4] update trajectory segment in the latest time interval delta_t
      if (PrepareMsgs())
      {
        // decide control point placement in the time interval delta_t by imu
        UpdateOneSeg();
        int offset = cp_add_num_cur + cp_add_num_next + cp_add_num_next_next;
        for (int i = 0; i < cp_add_num_cur; i++)
        {
          trajectory_->AddBlendMat(offset - i);  // blending matrix is computed by knots of b-spline
        }
        trajectory_manager_->SetDivision(cp_add_num_cur);
        trajectory_->startIdx = trajectory_->knts.size() - 1 - offset - 2;  // 2 serves as a margin or tolerance
        if (trajectory_->startIdx < 0)
        {
          trajectory_->startIdx = 0;
        }

        // fusing lidar-imu-camera to update the trajectory
        SolveLICO();

        // deep copy
        msg_manager_->cur_msgs = NextMsgs();
        msg_manager_->cur_msgs = msg_manager_->next_msgs;
        msg_manager_->cur_msgs.image = msg_manager_->next_msgs.image.clone();
        msg_manager_->next_msgs = NextMsgs();
        msg_manager_->next_msgs = msg_manager_->next_next_msgs;
        msg_manager_->next_msgs.image = msg_manager_->next_next_msgs.image.clone();
        msg_manager_->next_next_msgs = NextMsgs();

        traj_max_time_ns_cur = traj_max_time_ns_next;
        traj_max_time_ns_next = traj_max_time_ns_next_next;
        cp_add_num_cur = cp_add_num_next;
        cp_add_num_next = cp_add_num_next_next;

        while (msg_manager_->image_buf_.size() > 10)
        {
          msg_manager_->image_buf_.pop_front();
        }
      }
    }

    // GL2 step-1: flush + close the mapper_contract writer before shutdown —
    // rosbag2's cache-consumer thread must be joined or std::terminate fires.
    if (forgs_open_)
    {
      forgs_writer_->close();
      forgs_writer_.reset();
      forgs_open_ = false;
      std::cout << "[GL2] mapper_contract bag closed.\n";
    }
  }

  void OdometryManager::SolveLICO()
  {
    msg_manager_->LogInfo();
    if (msg_manager_->cur_msgs.lidar_timestamp < 0)
    {
      // LOG(INFO) << "CANT SolveLICO!";
    }

    // lic optimization
    ProcessLICData();

    // prior update
    trajectory_manager_->UpdateLICPrior(
        lidar_handler_->GetPointCorrespondence());

    // remove old imu data
    auto &msg = msg_manager_->cur_msgs;
    trajectory_manager_->UpdateLiDARAttribute(msg.lidar_timestamp,
                                              msg.lidar_max_timestamp);
  }

  void OdometryManager::ProcessLICData()
  {
    auto &msg = msg_manager_->cur_msgs;  // fake points with timestamp -1 exist up to now
    msg.CheckData();

    bool process_image = msg.if_have_image && msg.image_timestamp > t_begin_add_cam_;
    if (process_image)
    {
      // LOG(INFO) << "Process " << msg.scan_num << " scans in ["
      //           << msg.lidar_timestamp * NS_TO_S << ", " << msg.lidar_max_timestamp * NS_TO_S << "]"
      //           << "; image_time: " << msg.image_timestamp * NS_TO_S;
    }
    else
    {
      // LOG(INFO) << "Process " << msg.scan_num << " scans in ["
      //           << msg.lidar_timestamp * NS_TO_S << ", " << msg.lidar_max_timestamp * NS_TO_S << "]";
    }

    /// [1] transform the format of lidar pointcloud -> feature_cur_、feature_cur_ds_
    lidar_handler_->FeatureCloudHandler(msg.lidar_timestamp, msg.lidar_max_timestamp,
                                     msg.lidar_corner_cloud, msg.lidar_surf_cloud, msg.lidar_raw_cloud);  // fake points are removed

    /// [2] coarsely optimize trajectory based on prior、imu（served as good initial values）
    trajectory_manager_->PredictTrajectory(msg.lidar_timestamp, msg.lidar_max_timestamp,
                                           traj_max_time_ns_cur, cp_add_num_cur, non_uniform_);

    /// [3] update lidar local map
    int active_idx = trajectory_->numKnots() - 1 - cp_add_num_cur - 2;
    trajectory_->SetActiveTime(trajectory_->knts[active_idx]);
    lidar_handler_->UpdateLidarSubMap();

    /// [4] update visual local map（tracking map points for the current image frame）
    // after upate, m_map_rgb_pts_in_last_frame_pos = m_map_rgb_pts_in_current_frame_pos
    v_points_.clear();
    px_obss_.clear();
    // ROS2 port: camera/LICO functional path active (slimmed R3LIVE produces the
    // map-point→pixel correspondences v_points_/px_obss_ for the estimator).
    if (process_image)
    {
      SE3d Twc = trajectory_->GetCameraPoseNURBS(msg.image_timestamp);
      camera_handler_->UpdateVisualSubMap(msg.image, msg.image_timestamp * NS_TO_S, Twc.unit_quaternion(), Twc.translation());
      auto &map_rgb_pts_in_last_frame_pos = camera_handler_->op_track.m_map_rgb_pts_in_last_frame_pos;
      for (auto it = map_rgb_pts_in_last_frame_pos.begin(); it != map_rgb_pts_in_last_frame_pos.end(); it++)
      {
        RGB_pts *rgb_pt = ((RGB_pts *)it->first);
        v_points_.push_back(Eigen::Vector3d(rgb_pt->get_pos()(0, 0), rgb_pt->get_pos()(1, 0), rgb_pt->get_pos()(2, 0)));
        px_obss_.push_back(Eigen::Vector2d(it->second.x, it->second.y));
      }

#if 0  // ROS2 port: viz only (ROS1 cv_bridge / ros::Time::now)
      if (odom_viewer_.pub_track_img_.getNumSubscribers() != 0 || odom_viewer_.pub_sub_visual_map_.getNumSubscribers() != 0)
      {
        cv::Mat img_debug = camera_handler_->img_pose_->m_img.clone();
        VPointCloud visual_sub_map_debug;  // optical flow + ransac *2 -> 3d association（red）
        visual_sub_map_debug.clear();

        for (auto it = map_rgb_pts_in_last_frame_pos.begin(); it != map_rgb_pts_in_last_frame_pos.end(); it++)
        {
          RGB_pts *rgb_pt = ((RGB_pts *)it->first);
          cv::circle(img_debug, it->second, 2, cv::Scalar(0, 255, 0), -1, 8);  // optical flow + ransac *2 -> 2d association（green）
          VPoint temp_map;
          temp_map.x = rgb_pt->get_pos()(0, 0);
          temp_map.y = rgb_pt->get_pos()(1, 0);
          temp_map.z = rgb_pt->get_pos()(2, 0);
          temp_map.intensity = 0.;
          visual_sub_map_debug.push_back(temp_map);
        }

        cv_bridge::CvImage out_msg;
        out_msg.header.stamp = ros::Time::now();
        out_msg.encoding = sensor_msgs::image_encodings::BGR8;
        out_msg.image = img_debug;
        odom_viewer_.PublishTrackImg(out_msg.toImageMsg());
        odom_viewer_.PublishSubVisualMap(visual_sub_map_debug);
      }
#endif
    }

    /// [5] finely optimize trajectory based on prior、lidar、imu、camera
    // GL2 step-2c: build render-photometric reference once per frame (used in all iters).
    if (process_image)
      BuildRenderPhotometric();
    for (int iter = 0; iter < lidar_iter_; ++iter)
    {
      lidar_handler_->GetLoamFeatureAssociation();

      if (process_image)
      {
        trajectory_manager_->UpdateTrajectoryWithLIC(
            iter, msg.image_timestamp,
            lidar_handler_->GetPointCorrespondence(), v_points_, px_obss_, 8);
      }
      else
      {
        trajectory_manager_->UpdateTrajectoryWithLIC(
            iter, msg.image_timestamp,
            lidar_handler_->GetPointCorrespondence(), {}, {}, 8);
        trajectory_manager_->SetProcessCurImg(false);
      }
    }
    PublishCloudAndTrajectory();

    /// [6] update visual global map
    PosCloud::Ptr cloud_undistort = PosCloud::Ptr(new PosCloud);
    auto latest_feature_before_active_time = lidar_handler_->GetFeatureCurrent();
    PosCloud::Ptr cloud_distort = latest_feature_before_active_time.surface_features;
    if (cloud_distort->size() != 0)
    {
      trajectory_->UndistortScanInG(*cloud_distort, latest_feature_before_active_time.timestamp, *cloud_undistort);
      camera_handler_->UpdateVisualGlobalMap(cloud_undistort, latest_feature_before_active_time.time_max * NS_TO_S);
    }

    /// [7] associate new map points for the current image frame
    if (process_image)
    {
      SE3d Twc = trajectory_->GetCameraPoseNURBS(msg.image_timestamp);
      camera_handler_->AssociateNewPointsToCurrentImg(Twc.unit_quaternion(), Twc.translation());

#if 0  // ROS2 port: viz only (ROS1 cv_bridge / ros::Time::now)
      if (odom_viewer_.pub_undistort_scan_in_cur_img_.getNumSubscribers() != 0)
      {
        cv::Mat img_debug = camera_handler_->img_pose_->m_img.clone();
        {
          for (int i = 0; i < cloud_undistort->points.size(); i++)
          {
            auto pt = cloud_undistort->points[i];
            Eigen::Vector3d pt_e(pt.x, pt.y, pt.z);
            Eigen::Matrix3d Rwc = Twc.unit_quaternion().toRotationMatrix();
            Eigen::Vector3d twc = Twc.translation();
            Eigen::Vector3d pt_cam = Rwc.transpose() * pt_e - Rwc.transpose() * twc;
            double X = pt_cam.x(), Y = pt_cam.y(), Z = pt_cam.z();
            cv::Point2f pix(K_(0, 0) * X / Z + K_(0, 2), K_(1, 1) * Y / Z + K_(1, 2));
            cv::circle(img_debug, pix, 2, cv::Scalar(0, 0, 255), -1, 8);
          }
          cv_bridge::CvImage out_msg;
          out_msg.header.stamp = ros::Time::now();
          out_msg.encoding = sensor_msgs::image_encodings::BGR8;
          out_msg.image = img_debug;
          odom_viewer_.PublishUndistortScanInCurImg(out_msg.toImageMsg());
        }
      }

      if (odom_viewer_.pub_old_and_new_added_points_in_cur_img_.getNumSubscribers() != 0)
      {
        cv::Mat img_debug = camera_handler_->img_pose_->m_img.clone();
        auto obss = camera_handler_->op_track.m_map_rgb_pts_in_last_frame_pos;
        for (auto it = obss.begin(); it != obss.end(); it++)
        {
          cv::Point2f pix = it->second;
          cv::circle(img_debug, pix, 2, cv::Scalar(0, 255, 0), -1, 8);
        }

        cv_bridge::CvImage out_msg;
        out_msg.header.stamp = ros::Time::now();
        out_msg.encoding = sensor_msgs::image_encodings::BGR8;
        out_msg.image = img_debug;
        odom_viewer_.PublishOldAndNewAddedPointsInCurImg(out_msg.toImageMsg());
      }
#endif
    }

    /// [new] for Gaussian-LIC (camera/3DGS — GL2 step-1 mapper contract)
    if (process_image && if_3dgs_)
    {
      Publish3DGSMappingData(msg);
    }

    /// [8] visualize tf in rviz
    auto pose = trajectory_->GetLidarPoseNURBS(msg.lidar_timestamp);
    auto pose_debug = trajectory_->GetCameraPoseNURBS(msg.lidar_timestamp);
    odom_viewer_.PublishTF(pose.unit_quaternion(), pose.translation(), "lidar",
                           "map");
    odom_viewer_.PublishTF(pose_debug.unit_quaternion(), pose_debug.translation(), "camera",
                           "map");
    odom_viewer_.PublishTF(trajectory_manager_->GetGlobalFrame(),
                           Eigen::Vector3d::Zero(), "map", "global");
  }

  bool OdometryManager::PrepareTwoSegMsgs(int seg_idx)
  {
    if (!is_initialized_)
      return false;

    int64_t data_start_time = trajectory_->GetDataStartTime();
    for (auto &data : msg_manager_->lidar_buf_)
    {
      if (!data.is_time_wrt_traj_start)
      {
        data.ToRelativeMeasureTime(data_start_time);                             //
        msg_manager_->lidar_max_timestamps_[data.lidar_id] = data.max_timestamp; //
      }
    }
    for (auto &data : msg_manager_->image_buf_)
    {
      if (!data.is_time_wrt_traj_start)
      {
        data.ToRelativeMeasureTime(data_start_time);
        msg_manager_->image_max_timestamp_ = data.timestamp;
      }
    }
    msg_manager_->RemoveBeginData(data_start_time, 0);

    //
    //
    int64_t traj_max_time_ns = trajectory_->maxTimeNsNURBS() + t_add_ns_ * (seg_idx + 1);

    //
    //
    bool have_msg = false;
    if (seg_idx == 0)
    {
      int64_t traj_last_max_time_ns = trajectory_->maxTimeNsNURBS();
      have_msg = msg_manager_->GetMsgs(msg_manager_->cur_msgs, traj_last_max_time_ns, traj_max_time_ns, data_start_time);
    }
    if (seg_idx == 1)
    {
      int64_t traj_last_max_time_ns = trajectory_->maxTimeNsNURBS() + t_add_ns_;
      have_msg = msg_manager_->GetMsgs(msg_manager_->next_msgs, traj_last_max_time_ns, traj_max_time_ns, data_start_time);
    }

    //
    if (have_msg)
    {
      while (!msg_manager_->imu_buf_.empty())
      {
        trajectory_manager_->AddIMUData(msg_manager_->imu_buf_.front());
        msg_manager_->imu_buf_.pop_front();
      }
      if (seg_idx == 0)
      {
        traj_max_time_ns_cur = traj_max_time_ns;
      }
      if (seg_idx == 1)
      {
        traj_max_time_ns_next = traj_max_time_ns;
      }
      return true;
    }
    else
    {
      return false;
    }
  }

  void OdometryManager::UpdateTwoSeg()
  {
    auto imu_datas = trajectory_manager_->GetIMUData();

    /// update the first seg
    {
      int cp_add_num = cp_add_num_coarse_;
      Eigen::Vector3d aver_r = Eigen::Vector3d::Zero(), aver_a = Eigen::Vector3d::Zero();
      double var_r = 0, var_a = 0;
      int cnt = 0;
      for (int i = 0; i < imu_datas.size(); i++)
      {
        if (imu_datas[i].timestamp < trajectory_->maxTimeNsNURBS() ||
            imu_datas[i].timestamp >= traj_max_time_ns_cur)
          continue;
        cnt++;
        aver_r += imu_datas[i].gyro;
        aver_a += imu_datas[i].accel;
      }
      aver_r /= cnt;
      aver_a /= cnt;
      for (int i = 0; i < imu_datas.size(); i++)
      {
        if (imu_datas[i].timestamp < trajectory_->maxTimeNURBS() ||
            imu_datas[i].timestamp >= traj_max_time_ns_cur)
          continue;
        var_r += (imu_datas[i].gyro - aver_r).transpose() * (imu_datas[i].gyro - aver_r);
        var_a += (imu_datas[i].accel - aver_a).transpose() * (imu_datas[i].accel - aver_a);
      }
      var_r = sqrt(var_r / (cnt - 1));
      var_a = sqrt(var_a / (cnt - 1));
      // LOG(INFO) << "[aver_r_first] " << aver_r.norm() << " | [aver_a_first] " << aver_a.norm();
      // LOG(INFO) << "[var_r_first] " << var_r << " | [var_a_first] " << var_a;

      if (non_uniform_)
      {
        cp_add_num = GetKnotDensity(aver_r.norm(), aver_a.norm());
      }
      // LOG(INFO) << "[cp_add_num_first] " << cp_add_num;
      cp_num_vec.push_back(cp_add_num);

      int64_t step = (traj_max_time_ns_cur - trajectory_->maxTimeNsNURBS()) / cp_add_num;
      // LOG(INFO) << "[extend_step_first] " << step;
      for (int i = 0; i < cp_add_num - 1; i++)
      {
        int64_t time = trajectory_->maxTimeNsNURBS() + step * (i + 1);
        trajectory_->AddKntNs(time);
      }
      trajectory_->AddKntNs(traj_max_time_ns_cur);

      cp_add_num_cur = cp_add_num;
    }

    /// update the second seg
    {
      int cp_add_num = cp_add_num_coarse_;
      Eigen::Vector3d aver_r = Eigen::Vector3d::Zero(), aver_a = Eigen::Vector3d::Zero();
      double var_r = 0, var_a = 0;
      int cnt = 0;
      for (int i = 0; i < imu_datas.size(); i++)
      {
        if (imu_datas[i].timestamp < traj_max_time_ns_cur ||
            imu_datas[i].timestamp >= traj_max_time_ns_next)
          continue;
        cnt++;
        aver_r += imu_datas[i].gyro;
        aver_a += imu_datas[i].accel;
      }
      aver_r /= cnt;
      aver_a /= cnt;
      for (int i = 0; i < imu_datas.size(); i++)
      {
        if (imu_datas[i].timestamp < traj_max_time_ns_cur ||
            imu_datas[i].timestamp >= traj_max_time_ns_next)
          continue;
        var_r += (imu_datas[i].gyro - aver_r).transpose() * (imu_datas[i].gyro - aver_r);
        var_a += (imu_datas[i].accel - aver_a).transpose() * (imu_datas[i].accel - aver_a);
      }
      var_r = sqrt(var_r / (cnt - 1));
      var_a = sqrt(var_a / (cnt - 1));
      // LOG(INFO) << "[aver_r_second] " << aver_r.norm() << " | [aver_a_second] " << aver_a.norm();
      // LOG(INFO) << "[var_r_second] " << var_r << " | [var_a_second] " << var_a;

      if (non_uniform_)
      {
        cp_add_num = GetKnotDensity(aver_r.norm(), aver_a.norm());
      }
      // LOG(INFO) << "[cp_add_num_second] " << cp_add_num;
      cp_num_vec.push_back(cp_add_num);

      int64_t step = (traj_max_time_ns_next - traj_max_time_ns_cur) / cp_add_num;
      // LOG(INFO) << "[extend_step_second] " << step;
      for (int i = 0; i < cp_add_num - 1; i++)
      {
        int64_t time = traj_max_time_ns_cur + step * (i + 1);
        trajectory_->AddKntNs(time);
      }
      trajectory_->AddKntNs(traj_max_time_ns_next);

      cp_add_num_next = cp_add_num;
    }
  }

  bool OdometryManager::PrepareMsgs()
  {
    if (!is_initialized_)
      return false;

    int64_t data_start_time = trajectory_->GetDataStartTime();
    for (auto &data : msg_manager_->lidar_buf_)
    {
      if (!data.is_time_wrt_traj_start)
      {
        data.ToRelativeMeasureTime(data_start_time);                             //
        msg_manager_->lidar_max_timestamps_[data.lidar_id] = data.max_timestamp; //
      }
    }
    for (auto &data : msg_manager_->image_buf_)
    {
      if (!data.is_time_wrt_traj_start)
      {
        data.ToRelativeMeasureTime(data_start_time);
        msg_manager_->image_max_timestamp_ = data.timestamp;
      }
    }
    msg_manager_->RemoveBeginData(data_start_time, 0);

    int64_t traj_max_time_ns = traj_max_time_ns_next + t_add_ns_;

    int64_t traj_last_max_time_ns = traj_max_time_ns_next;
    bool have_msg = msg_manager_->GetMsgs(msg_manager_->next_next_msgs, traj_last_max_time_ns, traj_max_time_ns, data_start_time);

    if (have_msg)
    {
      while (!msg_manager_->imu_buf_.empty())
      {
        trajectory_manager_->AddIMUData(msg_manager_->imu_buf_.front());
        msg_manager_->imu_buf_.pop_front();
      }
      traj_max_time_ns_next_next = traj_max_time_ns;
      return true;
    }
    else
    {
      return false;
    }
  }

  void OdometryManager::UpdateOneSeg()
  {
    auto imu_datas = trajectory_manager_->GetIMUData();

    /// update the first seg
    {
      int cp_add_num = cp_add_num_coarse_;
      Eigen::Vector3d aver_r = Eigen::Vector3d::Zero(), aver_a = Eigen::Vector3d::Zero();
      double var_r = 0, var_a = 0;
      int cnt = 0;
      for (int i = 0; i < imu_datas.size(); i++)
      {
        if (imu_datas[i].timestamp < traj_max_time_ns_next ||
            imu_datas[i].timestamp >= traj_max_time_ns_next_next)
          continue;
        cnt++;
        aver_r += imu_datas[i].gyro;
        aver_a += imu_datas[i].accel;
      }
      aver_r /= cnt;
      aver_a /= cnt;
      for (int i = 0; i < imu_datas.size(); i++)
      {
        if (imu_datas[i].timestamp < traj_max_time_ns_next ||
            imu_datas[i].timestamp >= traj_max_time_ns_next_next)
          continue;
        var_r += (imu_datas[i].gyro - aver_r).transpose() * (imu_datas[i].gyro - aver_r);
        var_a += (imu_datas[i].accel - aver_a).transpose() * (imu_datas[i].accel - aver_a);
      }
      var_r = sqrt(var_r / (cnt - 1));
      var_a = sqrt(var_a / (cnt - 1));
      // LOG(INFO) << "[aver_r_new] " << aver_r.norm() << " | [aver_a_new] " << aver_a.norm();
      // LOG(INFO) << "[var_r_new] " << var_r << " | [var_a_new] " << var_a;

      if (non_uniform_)
      {
        cp_add_num = GetKnotDensity(aver_r.norm(), aver_a.norm());
      }
      // LOG(INFO) << "[cp_add_num_new] " << cp_add_num;
      cp_num_vec.push_back(cp_add_num);

      int64_t step = (traj_max_time_ns_next_next - traj_max_time_ns_next) / cp_add_num;
      // LOG(INFO) << "[extend_step_new] " << step;
      for (int i = 0; i < cp_add_num - 1; i++)
      {
        int64_t time = traj_max_time_ns_next + step * (i + 1);
        trajectory_->AddKntNs(time);
      }
      trajectory_->AddKntNs(traj_max_time_ns_next_next);

      cp_add_num_next_next = cp_add_num;
    }
  }

  void OdometryManager::SetInitialState()
  {
    if (is_initialized_)
    {
      assert(trajectory_->GetDataStartTime() > 0 && "data start time < 0");
      std::cout << RED << "[Error] system state has been initialized" << RESET << std::endl;
      return;
    }

    is_initialized_ = true;

    if (imu_initializer_->InitialDone())
    {
      SystemState sys_state = imu_initializer_->GetIMUState(); // I0toG
      trajectory_manager_->SetSystemState(sys_state, distance0_);

      trajectory_manager_->AddIMUData(imu_initializer_->GetIMUData().back());
      msg_manager_->imu_buf_.clear();
    }
    assert(trajectory_->GetDataStartTime() > 0 && "data start time < 0");
  }

  void OdometryManager::PublishCloudAndTrajectory()
  {
    odom_viewer_.PublishDenseCloud(trajectory_, lidar_handler_->GetFeatureMapDs(),
                                   lidar_handler_->GetFeatureCurrent());

    odom_viewer_.PublishSplineTrajectory(
        trajectory_, 0.0, trajectory_->maxTimeNURBS(), 0.1);
  }

  // GL2 step-1: open a mapper_contract rosbag2 for the /*_for_gs streams.
  void OdometryManager::OpenForGsWriter(const std::string &out_dir)
  {
    namespace fs = std::filesystem;
    std::error_code ec;
    fs::remove_all(out_dir, ec);  // fresh mapper_contract bag each run
    forgs_writer_ = std::make_shared<rosbag2_cpp::Writer>();
    rosbag2_storage::StorageOptions so;
    so.uri = out_dir;
    so.storage_id = "mcap";
    rosbag2_cpp::ConverterOptions co;
    co.input_serialization_format = "cdr";
    co.output_serialization_format = "cdr";
    forgs_writer_->open(so, co);
    forgs_open_ = true;
    std::cout << "\n[GL2] for_gs mapper-contract writer -> " << out_dir << "\n";
  }

  // GL2 step-2: create live /*_for_gs publishers (best_effort/keep_last to match
  // the mapper's sensor QoS) so the CUDA mapper can run concurrently with track A.
  void OdometryManager::OpenForGsLivePublishers()
  {
    if (gs_node_) return;
    gs_node_ = rclcpp::Node::make_shared("cocolic_gs_frontend");
    // GL2 step-2 flow control: RELIABLE + deep history so the mapper buffers
    // every frame and reliable backpressure paces track A to mapper throughput
    // (best_effort dropped ~90% when track A out-paced the mapper).
    auto qos = rclcpp::QoS(rclcpp::KeepLast(200)).reliable();
    pub_gs_img_ = gs_node_->create_publisher<sensor_msgs::msg::Image>("/image_for_gs", qos);
    pub_gs_depth_ = gs_node_->create_publisher<sensor_msgs::msg::Image>("/depth_for_gs", qos);
    pub_gs_caminfo_ = gs_node_->create_publisher<sensor_msgs::msg::CameraInfo>("/camera_info_for_gs", qos);
    pub_gs_pose_ = gs_node_->create_publisher<geometry_msgs::msg::PoseStamped>("/pose_for_gs", qos);
    pub_gs_points_ = gs_node_->create_publisher<sensor_msgs::msg::PointCloud2>("/points_for_gs", qos);

    // GL2 step-2bc: subscribe to the mapper's rendered feedback (reliable, to match
    // the mapper's rendered_feedback_qos). Lightweight callback: stash by observed_stamp.
    auto fb_qos = rclcpp::QoS(rclcpp::KeepLast(64)).reliable();
    sub_gs_feedback_ = gs_node_->create_subscription<gaussian_lic_msgs::msg::RenderedFeedback>(
        "/gaussian_lic/rendered_feedback", fb_qos,
        [this](gaussian_lic_msgs::msg::RenderedFeedback::SharedPtr m) {
          int64_t obs_ns = static_cast<int64_t>(m->observed_stamp.sec) * 1000000000LL +
                           m->observed_stamp.nanosec;
          std::lock_guard<std::mutex> lk(gs_fb_mutex_);
          gs_feedback_[obs_ns] = *m;
          gs_fb_count_++;
          gs_fb_last_observed_ns_ = obs_ns;
          while (gs_feedback_.size() > 200) gs_feedback_.erase(gs_feedback_.begin());
        });
    std::cout << "\n[GL2] for_gs LIVE pubs + rendered_feedback sub up (closed-loop coupling)\n";
  }

  // GL2 step-2c: build the per-frame render-photometric reference. For each visible
  // map point (v_points_ / px_obss_), sample a patch from the mapper's latest RENDERED
  // image at that pixel (the map's expected appearance) -> reference; the observed gray
  // image is the sample target. Hands both to trajectory_manager for the LIC solve.
  void OdometryManager::BuildRenderPhotometric()
  {
    if (!gs_live_publish_ || !enable_render_photometric_ || !gs_node_)
    {
      trajectory_manager_->ClearRenderPhotometric();
      return;
    }
    gaussian_lic_msgs::msg::RenderedFeedback fb;
    bool have = false;
    {
      std::lock_guard<std::mutex> lk(gs_fb_mutex_);
      if (!gs_feedback_.empty()) { fb = gs_feedback_.rbegin()->second; have = true; }
    }
    if (!have || v_points_.empty())
    {
      trajectory_manager_->ClearRenderPhotometric();
      return;
    }
    cv::Mat rendered_gray;
    try { rendered_gray = cv_bridge::toCvCopy(fb.image, "mono8")->image; }
    catch (...) { trajectory_manager_->ClearRenderPhotometric(); return; }
    if (rendered_gray.empty() || camera_handler_->img_pose_->m_img.empty())
    {
      trajectory_manager_->ClearRenderPhotometric();
      return;
    }
    cv::Mat observed_gray;
    cv::cvtColor(camera_handler_->img_pose_->m_img, observed_gray, cv::COLOR_BGR2GRAY);
    const int W = observed_gray.cols, H = observed_gray.rows;
    if (rendered_gray.cols != W || rendered_gray.rows != H)
    {
      trajectory_manager_->ClearRenderPhotometric();
      return;  // OQ6: require resolution parity
    }
    const int half = rp_patch_half_;
    const int ps = 2 * half;
    const int margin = half + 3;  // factor reads ±(half+2) around projected pixel; no bounds-check inside
    std::vector<std::vector<float>> patches(v_points_.size());
    std::vector<char> valid(v_points_.size(), 0);
    int n_valid = 0;
    for (size_t i = 0; i < v_points_.size(); ++i)
    {
      int u = (int)std::round(px_obss_[i].x());
      int v = (int)std::round(px_obss_[i].y());
      if (u < margin || u >= W - margin || v < margin || v >= H - margin) continue;
      std::vector<float> p(ps * ps);
      for (int x = 0; x < ps; ++x)
        for (int y = 0; y < ps; ++y)
          p[x * ps + y] = (float)rendered_gray.at<uint8_t>(v + (x - half), u + (y - half));
      patches[i] = std::move(p);
      valid[i] = 1;
      ++n_valid;
    }
    static int64_t bp_cnt = 0;
    if (++bp_cnt % 25 == 0)
      std::cout << "[GL2 render-photo] valid_patches=" << n_valid << "/" << v_points_.size() << "\n";
    trajectory_manager_->SetRenderPhotometric(observed_gray, std::move(patches), std::move(valid), half, rp_weight_);
  }

  // GL2 lockstep pacing (OQ4 fix): block after a published frame until the mapper's
  // latest feedback corresponds to a frame within max_lag of pub_abs_ns (or timeout).
  // Caps feedback staleness to ~one mapper-frame latency so the photometric factor
  // gets a rendered view that still overlaps the current observation.
  void OdometryManager::PaceForGsFeedback(int64_t pub_abs_ns)
  {
    if (!gs_node_ || !gs_live_publish_ || !gs_lockstep_) return;
    auto t0 = std::chrono::steady_clock::now();
    while (rclcpp::ok())
    {
      rclcpp::spin_some(gs_node_);
      int64_t last_fb;
      { std::lock_guard<std::mutex> lk(gs_fb_mutex_); last_fb = gs_fb_last_observed_ns_; }
      if (last_fb != 0 && last_fb >= pub_abs_ns - gs_lockstep_max_lag_ns_) break;
      auto el = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - t0).count();
      if (el > gs_lockstep_timeout_ms_) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }

  // GL2 step-2bc: drain the rendered_feedback subscription each RunBag iteration +
  // timing probe answering OQ4 — is feedback fresh enough to refine still-active knots?
  void OdometryManager::SpinForGsFeedback()
  {
    if (!gs_node_) return;
    rclcpp::spin_some(gs_node_);
    static int64_t probe_cnt = 0;
    int64_t n, last_obs;
    {
      std::lock_guard<std::mutex> lk(gs_fb_mutex_);
      n = gs_fb_count_; last_obs = gs_fb_last_observed_ns_;
    }
    if (n > 0 && (++probe_cnt % 25 == 0))
    {
      // active_time is relative to data start; feedback observed_stamp is absolute.
      int64_t active_abs = trajectory_->GetActiveTime() + trajectory_->GetDataStartTime();
      double lag_s = static_cast<double>(active_abs - last_obs) / 1e9;
      std::cout << "[GL2 probe] feedback_count=" << n
                << " last_obs_lag_vs_active=" << lag_s << "s (>0 = feedback older than active window edge)\n";
    }
  }

  // GL2 step-1/2: build one frame of the tracking-to-mapping interface (track A's
  // stable poses) — image(bgr8)/depth(32FC1 m)/camera_info(K_)/pose(Twc)/points(xyzrgb)
  // — then write it to the mapper_contract bag (step-1) and/or publish it live (step-2).
  void OdometryManager::WriteForGsFrame(
      const cv::Mat &img_bgr, const cv::Mat &depth32f,
      const Eigen::Quaterniond &q_wc, const Eigen::Vector3d &t_wc,
      const Eigen::aligned_vector<Eigen::Vector3d> &pts,
      const Eigen::aligned_vector<Eigen::Vector3i> &rgb, int64_t stamp_ns)
  {
    if (!forgs_open_ && !gs_live_publish_) return;
    rclcpp::Time stamp(stamp_ns);
    std_msgs::msg::Header hdr;
    hdr.stamp = stamp;
    hdr.frame_id = "camera";

    sensor_msgs::msg::Image img_msg;
    { cv_bridge::CvImage cvi; cvi.header = hdr; cvi.encoding = "bgr8"; cvi.image = img_bgr;
      img_msg = *cvi.toImageMsg(); }

    sensor_msgs::msg::Image depth_msg;
    { cv_bridge::CvImage cvi; cvi.header = hdr; cvi.encoding = "32FC1"; cvi.image = depth32f;
      depth_msg = *cvi.toImageMsg(); }

    sensor_msgs::msg::CameraInfo ci; ci.header = hdr;
    ci.width = img_bgr.cols; ci.height = img_bgr.rows;
    ci.k = {K_(0,0),0.0,K_(0,2), 0.0,K_(1,1),K_(1,2), 0.0,0.0,1.0};

    geometry_msgs::msg::PoseStamped ps; ps.header = hdr;
    ps.pose.position.x = t_wc.x(); ps.pose.position.y = t_wc.y(); ps.pose.position.z = t_wc.z();
    ps.pose.orientation.x = q_wc.x(); ps.pose.orientation.y = q_wc.y();
    ps.pose.orientation.z = q_wc.z(); ps.pose.orientation.w = q_wc.w();

    sensor_msgs::msg::PointCloud2 pc; pc.header = hdr;
    pc.height = 1; pc.width = static_cast<uint32_t>(pts.size());
    pc.is_bigendian = false; pc.is_dense = true;
    auto add_field = [&](const std::string &nm, uint32_t off) {
      sensor_msgs::msg::PointField f; f.name = nm; f.offset = off;
      f.datatype = sensor_msgs::msg::PointField::FLOAT32; f.count = 1;
      pc.fields.push_back(f);
    };
    add_field("x", 0); add_field("y", 4); add_field("z", 8); add_field("rgb", 12);
    pc.point_step = 16; pc.row_step = pc.point_step * pc.width;
    pc.data.resize(static_cast<size_t>(pc.row_step));
    for (size_t i = 0; i < pts.size(); ++i) {
      float xyz[3] = {(float)pts[i].x(), (float)pts[i].y(), (float)pts[i].z()};
      uint32_t r = (uint32_t)std::min(255, std::max(0, rgb[i].x()));
      uint32_t g = (uint32_t)std::min(255, std::max(0, rgb[i].y()));
      uint32_t b = (uint32_t)std::min(255, std::max(0, rgb[i].z()));
      uint32_t packed = (r << 16) | (g << 8) | b;
      float rgbf; std::memcpy(&rgbf, &packed, 4);
      uint8_t *p = pc.data.data() + i * pc.point_step;
      std::memcpy(p + 0, &xyz[0], 4); std::memcpy(p + 4, &xyz[1], 4);
      std::memcpy(p + 8, &xyz[2], 4); std::memcpy(p + 12, &rgbf, 4);
    }

    if (forgs_open_) {  // step-1: persist to mapper_contract bag
      forgs_writer_->write(img_msg, "/image_for_gs", stamp);
      forgs_writer_->write(depth_msg, "/depth_for_gs", stamp);
      forgs_writer_->write(ci, "/camera_info_for_gs", stamp);
      forgs_writer_->write(ps, "/pose_for_gs", stamp);
      forgs_writer_->write(pc, "/points_for_gs", stamp);
    }
    if (gs_live_publish_) {  // step-2: live to the concurrent mapper
      pub_gs_img_->publish(img_msg);
      pub_gs_depth_->publish(depth_msg);
      pub_gs_caminfo_->publish(ci);
      pub_gs_pose_->publish(ps);
      pub_gs_points_->publish(pc);
    }
  }

  void OdometryManager::Publish3DGSMappingData(const NextMsgs& msg)
  {
    // GL2: generate the tracking→mapping interface (track A's stable poses).
    // step-2 (gs_live_publish): publish live to a concurrent mapper;
    // step-1 (default): persist to the mapper_contract rosbag2 for offline replay.
    if (gs_live_publish_) {
      if (!gs_node_) OpenForGsLivePublishers();
    } else {
      if (!forgs_open_) OpenForGsWriter(cache_path_ + "_mapper_contract");
    }

    time_buf.push(msg.image_timestamp);
    lidar_buf.push(lidar_handler_->GetFeatureCurrent());
    img_buf.push(camera_handler_->img_pose_->m_img);

    while(1)
    {
      int64_t active_time = trajectory_->GetActiveTime();
      if (time_buf.front() < active_time && lidar_buf.front().time_max < active_time)
      {
        auto time = time_buf.front();
        auto lidar = lidar_buf.front();
        auto img = img_buf.front();
        time_buf.pop();
        lidar_buf.pop();
        img_buf.pop();

        PosCloud::Ptr cloud_undistort_ds = PosCloud::Ptr(new PosCloud);
        // PosCloud::Ptr cloud_distort_ds = lidar.surface_features;
        PosCloud::Ptr cloud_distort_ds = lidar.full_cloud;
        if (cloud_distort_ds->size() != 0)
        {
          trajectory_->UndistortScanInG(*cloud_distort_ds, lidar.timestamp, *cloud_undistort_ds);
          lidarpoints.push_back(cloud_undistort_ds);
        }

        auto pose_cam = trajectory_->GetCameraPoseNURBS(time);
        auto inv_pose_cam = pose_cam.inverse();
        auto cam_K = camera_handler_->m_camera_intrinsic;
        double fx = cam_K(0, 0), fy = cam_K(1, 1);
        double cx = cam_K(0, 2), cy = cam_K(1, 2);
        int H = camera_handler_->img_pose_->m_img.rows;
        int W = camera_handler_->img_pose_->m_img.cols;

        // depth
        cv::Mat depthmap = cv::Mat::zeros(H, W, CV_32FC1);
        for (int j = std::max(0, int(lidarpoints.size()) - 5); j < lidarpoints.size(); j++)
        {
          auto lidarpoint = lidarpoints[j];
          for (int i = 0; i < lidarpoint->size(); i++)
          {
            auto pt = lidarpoint->points[i];
            Eigen::Vector3d pt_w = Eigen::Vector3d(pt.x, pt.y, pt.z);
            Eigen::Vector3d pt_c = inv_pose_cam.unit_quaternion().toRotationMatrix() * pt_w + inv_pose_cam.translation();
            double depth = pt_c(2);
            pt_c /= pt_c(2);
            double u = fx * pt_c(0) + cx;
            double v = fy * pt_c(1) + cy;
            int i_u = std::round(u), i_v = std::round(v);
            if (depth <= 0) continue;
            if (!((i_u >= 0 && i_u < W && i_v >= 0 && i_v < H))) continue;

            float& current_depth = depthmap.at<float>(i_v, i_u);
            if (current_depth == 0 || depth < current_depth)
            {
                current_depth = depth;
            }
          }
        }
        while (lidarpoints.size() > 5)
        {
          lidarpoints.erase(lidarpoints.begin());
        }
        // points
        int filter_cnt = 0;
        int skip = lidar_skip_;
        Eigen::aligned_vector<Eigen::Vector3d> new_points;
        Eigen::aligned_vector<Eigen::Vector3i> new_colors;
        for (int i = 0; i < cloud_undistort_ds->points.size(); i += skip)
        {
          auto pt = cloud_undistort_ds->points[i];
          Eigen::Vector3d pt_w = Eigen::Vector3d(pt.x, pt.y, pt.z);
          Eigen::Vector3d pt_c = inv_pose_cam.unit_quaternion().toRotationMatrix() * pt_w + inv_pose_cam.translation();
          if (pt_c(2) < 0.01)
          {
            filter_cnt++;
            continue;
          }
          pt_c /= pt_c(2);
          double u = fx * pt_c(0) + cx;
          double v = fy * pt_c(1) + cy;
          if (u < 0 || u > W - 1)
          {
            filter_cnt++;
            continue;
          }
          new_points.push_back(Eigen::Vector3d(pt.x, pt.y, pt.z));

          int i_u = std::round(u), i_v = std::round(v);
          int blue = 0, green = 0, red = 0;
          if (i_u >= 0 && i_u < W && i_v >= 0 && i_v < H)
          {
            int u0 = std::floor(u), v0 = std::floor(v);
            int u1 = std::min(u0 + 1, W - 1), v1 = std::min(v0 + 1, H - 1);
            double du = u - u0, dv = v - v0;

            cv::Vec3b c00 = camera_handler_->img_pose_->m_img.at<cv::Vec3b>(v0, u0);
            cv::Vec3b c10 = camera_handler_->img_pose_->m_img.at<cv::Vec3b>(v0, u1);
            cv::Vec3b c01 = camera_handler_->img_pose_->m_img.at<cv::Vec3b>(v1, u0);
            cv::Vec3b c11 = camera_handler_->img_pose_->m_img.at<cv::Vec3b>(v1, u1);

            Eigen::Vector3d color00(c00[0], c00[1], c00[2]);
            Eigen::Vector3d color10(c10[0], c10[1], c10[2]);
            Eigen::Vector3d color01(c01[0], c01[1], c01[2]);
            Eigen::Vector3d color11(c11[0], c11[1], c11[2]);

            Eigen::Vector3d interpolated_color =
                (1 - du) * (1 - dv) * color00 +
                du * (1 - dv) * color10 +
                (1 - du) * dv * color01 +
                du * dv * color11;
            blue = std::round(interpolated_color.x());
            green = std::round(interpolated_color.y());
            red = std::round(interpolated_color.z());
          }
          new_colors.push_back(Eigen::Vector3i(red, green, blue));
        }
        // GL2 step-1: emit the full frame (image/depth/pose/points) to the
        // mapper_contract bag in one shot — track A's pose is the contract.
        WriteForGsFrame(img, depthmap, pose_cam.unit_quaternion(),
                        pose_cam.translation(), new_points, new_colors,
                        time + trajectory_->GetDataStartTime());
        // GL2 lockstep: wait for the mapper to catch up before advancing (caps feedback lag).
        PaceForGsFeedback(time + trajectory_->GetDataStartTime());
      }
      else break;
    }
  }

  double OdometryManager::SaveOdometry()
  {
    std::string descri;
    if (odometry_mode_ == LICO)
      descri = "LICO";
    else if (odometry_mode_ == LIO)
      descri = "LIO";

    if (msg_manager_->NumLiDAR() > 1)
      descri = descri + "2";

    // ROS2 port: dropped unused ros::Time-based filename suffix (t_str unused).

    int idx = -1;
    int64_t true_maxtime = trajectory_->maxTimeNsNURBS();
    for (int i = trajectory_->knts.size() - 1; i >= 0; i--)
    {
      if (true_maxtime == trajectory_->knts[i])
      {
        idx = i;
        break;
      }
    }
    idx -= 1;
    int64_t maxtime = trajectory_->knts[idx];
    maxtime = trajectory_->maxTimeNsNURBS() - 0.1 * S_TO_NS;

    trajectory_->ToTUMTxt(cache_path_ + "_" + descri + ".txt", maxtime, is_evo_viral_,
                          0.01);  // 100Hz pose querying

    // int sum_cp = std::accumulate(cp_num_vec.begin(), cp_num_vec.end(), 0);
    // std::cout << GREEN << "ave_cp_num " << sum_cp * 1.0 / cp_num_vec.size() << RESET << std::endl;

    return trajectory_->maxTimeNURBS();
  }

} // namespace cocolic
