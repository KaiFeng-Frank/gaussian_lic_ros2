/*
 * Coco-LIC ROS2 Jazzy port — slimmed R3LIVE visual front-end (S4).
 *
 * Coco-LIC uses its OWN continuous-time B-spline optimizer, so it needs only
 * R3LIVE's visual FRONT-END (optical-flow tracking + RGB map) to produce
 * map-point→pixel correspondences for the (already-ported) image_feature_factor.
 * This class is the minimal subset of upstream R3LIVE for that role: it drops
 * the entire R3LIVE LIO / VIO-ESIKF / ROS publishing machinery (and so does NOT
 * pull common_lib.h). It exposes exactly the ctor + 3 methods odometry_manager
 * calls: UpdateVisualGlobalMap / UpdateVisualSubMap / AssociateNewPointsToCurrentImg.
 *
 * Effective VIO params are upstream's offline defaults + hardcoded overrides
 * (no r3live launch yaml ships with Coco-LIC; values verified from r3live.hpp).
 */
#pragma once

#include <memory>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include "rgb_map/pointcloud_rgbd.hpp"   // Global_map, RGB_pts
#include "rgb_map/rgbmap_tracker.hpp"    // Rgbmap_tracker
#include "rgb_map/image_frame.hpp"       // Image_frame

#include <utils/parameter_struct.h>      // cocolic::ExtrinsicParam, PosCloud

namespace cocolic
{

  class R3LIVE
  {
  public:
    typedef std::shared_ptr<R3LIVE> Ptr;

    R3LIVE(const YAML::Node &node, const ExtrinsicParam &EP_CtoI)
    {
      // --- camera intrinsics / distortion / extrinsics (from camera.yaml) ---
      m_vio_image_width = node["image_width"].as<double>();
      m_vio_image_heigh = node["image_height"].as<double>();
      double cam_fx = node["cam_fx"].as<double>();
      double cam_fy = node["cam_fy"].as<double>();
      double cam_cx = node["cam_cx"].as<double>();
      double cam_cy = node["cam_cy"].as<double>();
      m_camera_intrinsic << cam_fx, 0.0, cam_cx,
                            0.0, cam_fy, cam_cy,
                            0.0, 0.0, 1.0;
      double cam_d0 = node["cam_d0"].as<double>();
      double cam_d1 = node["cam_d1"].as<double>();
      double cam_d2 = node["cam_d2"].as<double>();
      double cam_d3 = node["cam_d3"].as<double>();
      double cam_d4 = node["cam_d4"].as<double>();
      m_camera_dist_coeffs << cam_d0, cam_d1, cam_d2, cam_d3, cam_d4;
      m_camera_ext_R = EP_CtoI.q.toRotationMatrix();
      m_camera_ext_t = EP_CtoI.p;

      // --- rgb map setup (verbatim from upstream ctor) ---
      m_map_rgb_pts.set_minmum_dis(m_minumum_rgb_pts_size);
      m_map_rgb_pts.m_recent_visited_voxel_activated_time =
          m_recent_visited_voxel_activated_time;

      img_pose_ = std::make_shared<Image_frame>();
      handle_first_img_done_ = false;
      frame_idx_ = 0;
    }
    ~R3LIVE() {}

    // The 3 methods Coco-LIC's odometry_manager calls (defined in r3live_slim.cpp,
    // ported verbatim from r3live_vio.cpp).
    void UpdateVisualGlobalMap(const PosCloud::Ptr &cloud_undistort, double lidar_scan_time_max);
    void UpdateVisualSubMap(const cv::Mat &img_in, double img_time,
                            const Eigen::Quaterniond &q_wc, const Eigen::Vector3d &t_wc);
    void AssociateNewPointsToCurrentImg(const Eigen::Quaterniond &q_wc, const Eigen::Vector3d &t_wc);

  public:
    // RGB map + optical-flow tracker + current image frame.
    Global_map m_map_rgb_pts;
    Rgbmap_tracker op_track;
    std::shared_ptr<Image_frame> img_pose_;

    // camera params
    // CRITICAL: RowMajor — upstream does `g_cam_K << m_camera_intrinsic.data()[0..8]`
    // which reads .data() assuming row-major storage. A plain (column-major)
    // Eigen::Matrix3d transposes K → cx,cy in the wrong cells → degenerate
    // undistort maps → black remapped image → optical-flow tracking fails.
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> m_camera_intrinsic;
    Eigen::Matrix<double, 5, 1> m_camera_dist_coeffs;
    Eigen::Matrix3d m_camera_ext_R;
    Eigen::Vector3d m_camera_ext_t;
    Eigen::Matrix3d g_cam_K;
    Eigen::Matrix<double, 5, 1> g_cam_dist;
    int m_vio_image_width = 0;
    int m_vio_image_heigh = 0;
    cv::Mat intrinsic, dist_coeffs, m_ud_map1, m_ud_map2;

    bool cam_init = false;
    bool handle_first_img_done_ = false;
    int frame_idx_ = 0;

    // VIO front-end params (upstream offline defaults + hardcoded overrides).
    int m_track_windows_size = 40;
    int m_maximum_vio_tracked_pts = 600;
    double m_tracker_minimum_depth = 0.1;
    double m_tracker_maximum_depth = 200.0;
    int m_append_global_map_point_step = 4;
    double m_minumum_rgb_pts_size = 0.01;
    double m_recent_visited_voxel_activated_time = 0.0;
  };

} // namespace cocolic
