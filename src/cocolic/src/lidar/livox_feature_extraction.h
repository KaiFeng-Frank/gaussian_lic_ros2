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

// ROS2 Jazzy port: dropped <ros/ros.h>, <sensor_msgs/PointCloud2.h>,
// <livox_ros_driver/CustomMsg.h> (ROS1-only). The Livox input is delivered via
// CustomMsgLite (below), populated from the offset_time_full PointCloud2.
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include "lidar_feature.h"

#include <utils/cloud_tool.h>
#include <utils/mypcl_cloud_type.h>
#include <utils/parameter_struct.h>

#include <cstdint>
#include <memory>
#include <vector>
#include <cmath>
// #define M_PI 3.14159265358979
#define IS_VALID(a) ((abs(a) > 1e8) ? true : false)

namespace cocolic
{

  enum Feature
  {
    Nor,
    Poss_Plane,
    Real_Plane,
    Edge_Jump,
    Edge_Plane,
    Wire,
    ZeroPoint
  };

  enum Surround
  {
    Prev,
    Next
  };

  enum E_jump
  {
    Nr_nor,
    Nr_zero,
    Nr_180,
    Nr_inf,
    Nr_blind
  };

  struct orgtype
  {
    double range;
    double dista;
    double angle[2];
    double intersect;
    E_jump edj[2];
    Feature ftype;
    orgtype()
    {
      range = 0;
      edj[Prev] = Nr_nor;
      edj[Next] = Nr_nor;
      ftype = Nor;
      intersect = 2;
    }
  };

  // ROS2 port replacement for livox_ros_driver::CustomMsg / CustomPoint.
  // Populated from the offset_time_full PointCloud2 (20-byte stride). Holds
  // exactly the members upstream ParsePointCloud/giveFeature read, so the
  // feature-extraction bodies stay verbatim (lidar_msg-> preserved via ConstPtr).
  struct CustomPointLite
  {
    float x, y, z;
    uint8_t reflectivity;  // <- PointCloud2 intensity
    uint8_t tag;
    uint8_t line;
    uint32_t offset_time;  // ns offset within scan (Livox per-point time)
  };

  struct CustomMsgLite
  {
    typedef std::shared_ptr<const CustomMsgLite> ConstPtr;
    typedef std::shared_ptr<CustomMsgLite> Ptr;
    int64_t timebase = 0;       // scan base time (ns) = header stamp
    uint32_t point_num = 0;
    std::vector<CustomPointLite> points;
  };

  class LivoxFeatureExtraction
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<LivoxFeatureExtraction> Ptr;

    LivoxFeatureExtraction(const YAML::Node &node);

    //
    bool ParsePointCloud(const CustomMsgLite::ConstPtr &lidar_msg,
                         RTPointCloud::Ptr out_cloud);

    //
    bool ParsePointCloudR3LIVE(const CustomMsgLite::ConstPtr &lidar_msg,
                         RTPointCloud::Ptr out_cloud);

    //
    bool ParsePointCloudNoFeature(const CustomMsgLite::ConstPtr &lidar_msg,
                                  RTPointCloud::Ptr out_cloud);

    // test
    void LivoxHandler(const CustomMsgLite::ConstPtr &lidar_msg);

    inline RTPointCloud::Ptr GetCornerFeature() const { return p_corner_cloud; }

    inline RTPointCloud::Ptr GetSurfaceFeature() const { return p_surface_cloud; }

    inline int GetScanNumber() const { return n_scan; }

    void clearState();

  private:
    void AllocateMemory();

    void giveFeature(RTPointCloud &pl, std::vector<orgtype> &types,
                     RTPointCloud &pl_corn, RTPointCloud &pl_surf);

    void giveFeatureR3LIVE(RTPointCloud &pl, std::vector<orgtype> &types,
                     RTPointCloud &pl_corn, RTPointCloud &pl_surf);

    int checkPlane(const RTPointCloud &pl, std::vector<orgtype> &types,
                   uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct);

    bool checkCorner(const RTPointCloud &pl, std::vector<orgtype> &types, uint i,
                     Surround nor_dir);

    void PublishCloud(std::string frame_id);  // ROS2 port: no-op stub (debug viz)

  private:
    // ROS2 port: dropped ros::NodeHandle + debug-viz publishers/subscriber.

    int n_scan;
    double blind, inf_bound;
    int group_size;
    double disA, disB;
    double limit_maxmid, limit_midmin, limit_maxmin;
    double p2l_ratio;
    double jump_up_limit, jump_down_limit;
    double edgea, edgeb;
    double smallp_intersect;
    double smallp_ratio;
    int point_filter_num;

    double vx, vy, vz;

    RTPointCloud::Ptr p_corner_cloud;
    RTPointCloud::Ptr p_surface_cloud;
    RTPointCloud::Ptr p_full_cloud;

    MODE work_mode_;
  };
} // namespace cocolic
