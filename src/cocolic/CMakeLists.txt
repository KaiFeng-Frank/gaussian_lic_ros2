cmake_minimum_required(VERSION 3.16)
project(cocolic)

# Faithful mechanical ROS2 port of upstream Coco-LIC. Incremental: this stage
# builds ONLY the ROS-agnostic core (sophus_lib + spline + utils) as a static
# library to validate the port approach + Ceres 2.2 compatibility. Later stages
# add odom/lidar/imu/camera and the ROS2 node glue.

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3 -Wno-sign-compare -Wno-unused -Wno-comment -pthread)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)  # ROS2 port: pcl::fromROSMsg for Velodyne (M2DGR)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
# ROS2 port: only common + kdtree are used (transformPointCloud, KdTreeFLANN).
# Dropped 'io' — it pulls system GDAL/HDF5 whose libcurl clashes with conda's,
# and pcl::io is used only by the (not-built) camera module.
find_package(PCL 1.10 REQUIRED COMPONENTS common kdtree)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenMP)

# glog (upstream uses <glog/logging.h>)
find_library(GLOG_LIB glog REQUIRED)
# lz4 — transitive PCL/FLANN dependency, must be explicit at link time.
find_library(LZ4_LIB NAMES lz4 liblz4 REQUIRED)
# System libcurl (CURL_OPENSSL_4): if PCL still drags in libpcl_io→gdal/hdf5,
# the conda libcurl on the link path lacks the versioned symbols. Force the
# system one (NO_DEFAULT_PATH avoids conda). Harmless if pcl_io is not linked.
find_library(SYS_CURL_LIB NAMES curl
  PATHS /usr/lib/x86_64-linux-gnu /lib/x86_64-linux-gnu NO_DEFAULT_PATH)

# Upstream resolves includes relative to src/ (e.g. sophus_lib/so3.hpp,
# ../utils/x from spline/). Mirror that with src/ on the include path.
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/include   # include/ros compat shim
  # S4 camera module include paths (R3LIVE visual front-end).
  ${CMAKE_CURRENT_SOURCE_DIR}/src/camera
  ${CMAKE_CURRENT_SOURCE_DIR}/src/camera/tools
  ${CMAKE_CURRENT_SOURCE_DIR}/src/camera/rgb_map
  ${CMAKE_CURRENT_SOURCE_DIR}/src/camera/optical_flow
  ${CMAKE_CURRENT_SOURCE_DIR}/src/camera/loam/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src/camera/loam/include/kd_tree
  ${CMAKE_CURRENT_SOURCE_DIR}/src/camera/loam/include/FOV_Checker
  ${EIGEN3_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Core library: the ROS-agnostic continuous-time spline + params (verbatim).
add_library(cocolic_core STATIC
  src/spline/trajectory.cpp
  src/utils/parameter_struct.cpp
  src/odom/trajectory_estimator.cpp
  src/odom/factor/analytic_diff/marginalization_factor.cpp
  src/odom/trajectory_manager.cpp
  src/lidar/livox_feature_extraction.cpp
  src/lidar/velodyne_feature_extraction.cpp
  src/lidar/lidar_handler.cpp
  src/imu/imu_initializer.cpp
  src/imu/imu_state_estimator.cpp
  src/odom/msg_manager.cpp
  src/odom/odometry_manager.cpp
)
target_link_libraries(cocolic_core
  ${CERES_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  yaml-cpp
  ${GLOG_LIB}
)
# msg_manager.cpp uses rosbag2 + rclcpp serialization + cv_bridge + sensor_msgs.
ament_target_dependencies(cocolic_core
  rclcpp rosbag2_cpp rosbag2_storage cv_bridge pcl_conversions sensor_msgs std_msgs geometry_msgs)
if(OpenMP_CXX_FOUND)
  target_link_libraries(cocolic_core OpenMP::OpenMP_CXX)
endif()

# S4 camera modules (R3LIVE visual front-end). Built as a SEPARATE lib first to
# validate they compile under ROS2/PCL1.14/OpenCV4 without risking the working
# LIO core. These 6 are ROS-clean (verified). Integrated into the slimmed R3LIVE later.
add_library(cocolic_camera STATIC
  src/camera/rgb_map/image_frame.cpp
  src/camera/optical_flow/lkpyramid.cpp
  src/camera/loam/include/kd_tree/ikd_Tree.cpp
  src/camera/loam/include/FOV_Checker/FOV_Checker.cpp
  src/camera/rgb_map/pointcloud_rgbd.cpp
  src/camera/rgb_map/rgbmap_tracker.cpp
  src/camera/r3live_slim.cpp
)
target_link_libraries(cocolic_camera
  ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES} ${GLOG_LIB})
if(OpenMP_CXX_FOUND)
  target_link_libraries(cocolic_camera OpenMP::OpenMP_CXX)
endif()

# Custom messages (ROS2 rosidl port of upstream msg/*.msg).
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/FeatureCloud.msg
  msg/ImuArray.msg
  msg/PoseArray.msg
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)

# Main odometry node (ROS2 port of upstream odometry_node).
add_executable(odometry_node src/odometry_node.cpp)
# cocolic_core (odometry_manager) now calls the slimmed R3LIVE in cocolic_camera.
# --start-group/--end-group resolves any cross-lib symbol order between them.
target_link_libraries(odometry_node
  -Wl,--start-group cocolic_core cocolic_camera -Wl,--end-group
  ${GLOG_LIB} ${LZ4_LIB} ${SYS_CURL_LIB})
ament_target_dependencies(odometry_node rclcpp rosbag2_cpp rosbag2_storage sensor_msgs)
install(TARGETS odometry_node DESTINATION lib/${PROJECT_NAME})

add_executable(bag_ingest_test test/bag_ingest_test.cpp)
ament_target_dependencies(bag_ingest_test rclcpp rosbag2_cpp sensor_msgs)
install(TARGETS bag_ingest_test DESTINATION lib/${PROJECT_NAME})

# cm-grade ingest validation: offset_time_full.mcap per-point time channel.
add_executable(ingest_offset_time_test test/ingest_offset_time_test.cpp)
ament_target_dependencies(ingest_offset_time_test rclcpp rosbag2_cpp rosbag2_storage sensor_msgs)
install(TARGETS ingest_offset_time_test DESTINATION lib/${PROJECT_NAME})

# Verify LoadBag storage auto-detect (empty storage_id) for sqlite3 + mcap dirs.
add_executable(storage_autodetect_test test/storage_autodetect_test.cpp)
ament_target_dependencies(storage_autodetect_test rclcpp rosbag2_cpp rosbag2_storage)
install(TARGETS storage_autodetect_test DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  add_test(NAME cocolic_bag_ingest_test COMMAND $<TARGET_FILE:bag_ingest_test>)
  add_test(NAME cocolic_ingest_offset_time_test COMMAND $<TARGET_FILE:ingest_offset_time_test>)
  add_test(NAME cocolic_storage_autodetect_test COMMAND $<TARGET_FILE:storage_autodetect_test>)
endif()

ament_package()
