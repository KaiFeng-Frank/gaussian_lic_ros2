/*
 * Coco-LIC ROS2 Jazzy port — slimmed R3LIVE visual front-end (S4).
 * The 3 methods are ported VERBATIM from upstream r3live_vio.cpp:1300-1400
 * (only the enclosing class changed from the full R3LIVE to the slim one).
 */
#include "r3live_slim.hpp"

#include <utils/eigen_utils.hpp>  // Eigen::aligned_vector

// ROS2 port: this global is declared extern by the camera modules (rgb_map) and
// defined in upstream r3live_vio.cpp (not built here) — provide the definition.
Common_tools::Cost_time_logger g_cost_time_logger;

namespace cocolic
{

  void R3LIVE::UpdateVisualGlobalMap(const PosCloud::Ptr &cloud_undistort, double lidar_scan_time_max)
  {
    m_map_rgb_pts.my_append_points_to_global_map(*cloud_undistort, lidar_scan_time_max, nullptr, m_append_global_map_point_step);
  }

  void R3LIVE::UpdateVisualSubMap(const cv::Mat &img_in, double img_time, const Eigen::Quaterniond &q_wc, const Eigen::Vector3d &t_wc)
  {
    // [1]
    if (!cam_init)
    {
      cam_init = true;
      double *intrinsic_data = m_camera_intrinsic.data();
      double *camera_dist_data = m_camera_dist_coeffs.data();
      g_cam_K << intrinsic_data[0], intrinsic_data[1], intrinsic_data[2],
          intrinsic_data[3], intrinsic_data[4], intrinsic_data[5],
          intrinsic_data[6], intrinsic_data[7], intrinsic_data[8];
      g_cam_dist = Eigen::Map<Eigen::Matrix<double, 5, 1>>(camera_dist_data);
      cv::eigen2cv(g_cam_K, intrinsic);
      cv::eigen2cv(g_cam_dist, dist_coeffs);
      cv::initUndistortRectifyMap(intrinsic, dist_coeffs, cv::Mat(), intrinsic, cv::Size(m_vio_image_width, m_vio_image_heigh),
                                  CV_16SC2, m_ud_map1, m_ud_map2);

      op_track.set_intrinsic(g_cam_K, g_cam_dist * 0, cv::Size(m_vio_image_width, m_vio_image_heigh));
      op_track.m_maximum_vio_tracked_pts = m_maximum_vio_tracked_pts;
      m_map_rgb_pts.m_minimum_depth_for_projection = m_tracker_minimum_depth;
      m_map_rgb_pts.m_maximum_depth_for_projection = m_tracker_maximum_depth;
    }

    // [2]
    if (img_in.empty())
    {
      std::cout << "[wrong img]\n";
    }
    img_pose_ = std::make_shared<Image_frame>(g_cam_K);
    cv::Mat img_in_clone = img_in.clone();
    img_pose_->m_timestamp = img_time;
    img_pose_->m_raw_img = img_in_clone;
    cv::remap(img_in_clone, img_pose_->m_img, m_ud_map1, m_ud_map2, cv::INTER_LINEAR);
    img_pose_->init_cubic_interpolation();
    img_pose_->image_equalize();
    img_pose_->set_pose(q_wc, t_wc); // Twc

    // [3]
    if (!handle_first_img_done_)
    {
      img_pose_->set_frame_idx(frame_idx_);
      std::vector<cv::Point2f> pts_2d_vec;
      std::vector<std::shared_ptr<RGB_pts>> rgb_pts_vec;
      if (m_map_rgb_pts.m_rgb_pts_vec.size() <= 100)
      {
        return;
      }
      Eigen::aligned_vector<Eigen::Vector3d> fake_points;
      Eigen::aligned_vector<Eigen::Vector2d> fake_pixs;
      m_map_rgb_pts.selection_points_for_projection(false, fake_points, fake_pixs, img_pose_, &rgb_pts_vec, &pts_2d_vec, m_track_windows_size / 2);
      if (rgb_pts_vec.size() >= 10)
      {
        handle_first_img_done_ = true;
        op_track.init(img_pose_, rgb_pts_vec, pts_2d_vec);
        frame_idx_++;
        op_track.last_img = img_pose_->m_img_gray.clone();
      }
      return;
    }

    frame_idx_++;

    // [4]
    op_track.track_img(img_pose_, -20);
    op_track.inlier_aft_fmat = op_track.m_last_tracked_pts.size();

    // [5]
    op_track.remove_outlier_using_ransac_pnp(img_pose_, 1);
    op_track.inlier_aft_pnp = op_track.m_last_tracked_pts.size();

    op_track.last_img = img_pose_->m_img_gray.clone();
  }

  void R3LIVE::AssociateNewPointsToCurrentImg(const Eigen::Quaterniond &q_wc, const Eigen::Vector3d &t_wc)
  {
    if (!handle_first_img_done_)
    {
      return;
    }
    img_pose_->set_pose(q_wc, t_wc); // Twc
    img_pose_->m_fov_margin = -0.4;
    op_track.update_and_append_track_pts(img_pose_, m_map_rgb_pts, m_track_windows_size / 2, 1000000);
  }

} // namespace cocolic
