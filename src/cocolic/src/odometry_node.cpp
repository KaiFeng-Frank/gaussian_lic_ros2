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

// ROS2 Jazzy port of Coco-LIC's odometry_node. The offline replay reads the bag
// itself (msg_manager via rosbag2), so this is a plain executable: it does not
// spin a node. config_path comes from argv[1] (was the ROS1 "config_path" param).
#include <glog/logging.h>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include <iostream>
#include <string>

#include <odom/odometry_manager.h>

using namespace cocolic;

int main(int argc, char **argv)
{
  google::InitGoogleLogging(argv[0]);
  rclcpp::init(argc, argv);  // for rclcpp::Serialization / rosbag2 plumbing

  std::string config_path = (argc > 1) ? argv[1] : "ct_odometry.yaml";
  std::cout << "Odometry load " << config_path << ".\n";

  YAML::Node config_node = YAML::LoadFile(config_path);
  std::cout << "\n🥥 Start Coco-LIC Odometry (ROS2 Jazzy port) 🥥\n";

  OdometryManager odom_manager(config_node);
  odom_manager.RunBag();

  double t_traj_max = odom_manager.SaveOdometry();
  (void)t_traj_max;
  std::cout << "\n✨ All Done.\n\n";

  rclcpp::shutdown();
  return 0;
}
