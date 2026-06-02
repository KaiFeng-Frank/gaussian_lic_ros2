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

// ROS2 Jazzy port: OdometryViewer is ENTIRELY visualization (RViz publishers +
// TF broadcaster + debug msgs). NONE of it feeds back into the estimator or the
// trajectory, so for the offline odometry replay it is stubbed to no-ops. The
// original ROS1 viz (rosbag/tf/nav_msgs/visualization_msgs/cocolic debug msgs,
// ~877 lines) is intentionally dropped; restoring live viz is a separate RViz2
// port. The stub keeps exactly the surface odometry_manager.cpp touches:
//   members accessed via .getNumSubscribers(): pub_track_img_,
//     pub_undistort_scan_in_cur_img_, pub_old_and_new_added_points_in_cur_img_,
//     pub_sub_visual_map_  (StubPub::getNumSubscribers()==0 ⇒ guarded viz blocks
//     are dead);
//   methods: SetPublisher, PublishTF, PublishTrackImg, PublishUndistortScanInCurImg,
//     PublishOldAndNewAddedPointsInCurImg, PublishSubVisualMap,
//     PublishSplineTrajectory, PublishDenseCloud, Publish3DGS{Image,Depth,Pose,Points}.

#include <Eigen/Dense>
#include <map>
#include <string>
#include <utility>

namespace cocolic
{

  enum SplineViewerType
  {
    Init = 0,
    Loop = 2,
  };

  class OdometryViewer
  {
  public:
    // Stub publisher: getNumSubscribers()==0 makes every guarded viz block in
    // odometry_manager a no-op; publish() does nothing.
    struct StubPub
    {
      int getNumSubscribers() const { return 0; }
      template <class T>
      void publish(const T &) const {}
    };

    // Members accessed directly (.getNumSubscribers()) by odometry_manager.
    StubPub pub_track_img_;
    StubPub pub_undistort_scan_in_cur_img_;
    StubPub pub_old_and_new_added_points_in_cur_img_;
    StubPub pub_sub_visual_map_;

    // SetPublisher: accepts the old (ros::NodeHandle&) call or no args.
    template <class... A>
    void SetPublisher(A &&...) {}

    // All Publish* used by odometry_manager → no-ops (variadic accepts any args,
    // only instantiated when called, so arg types never need to exist here).
    template <class... A> void PublishTF(A &&...) {}
    template <class... A> void PublishTrackImg(A &&...) {}
    template <class... A> void PublishUndistortScanInCurImg(A &&...) {}
    template <class... A> void PublishOldAndNewAddedPointsInCurImg(A &&...) {}
    template <class... A> void PublishSubVisualMap(A &&...) {}
    template <class... A> void PublishSplineTrajectory(A &&...) {}
    template <class... A> void PublishDenseCloud(A &&...) {}
    template <class... A> void Publish3DGSImage(A &&...) {}
    template <class... A> void Publish3DGSDepth(A &&...) {}
    template <class... A> void Publish3DGSPose(A &&...) {}
    template <class... A> void Publish3DGSPoints(A &&...) {}
  };

} // namespace cocolic
