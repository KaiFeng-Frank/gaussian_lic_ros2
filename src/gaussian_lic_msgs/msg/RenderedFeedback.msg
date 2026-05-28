# SPDX-License-Identifier: GPL-3.0-or-later

std_msgs/Header header
sensor_msgs/Image image
sensor_msgs/Image observed_image
geometry_msgs/Pose source_pose

builtin_interfaces/Time observed_stamp
builtin_interfaces/Time pose_stamp
builtin_interfaces/Time pointcloud_stamp

uint64 frame_index
uint64 rendered_preview_index
string render_mode
