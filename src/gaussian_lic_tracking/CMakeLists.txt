cmake_minimum_required(VERSION 3.16)
project(gaussian_lic_tracking)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(trajectory_manager
  src/imu_propagator.cpp
  src/lidar_factor.cpp
  src/trajectory_manager.cpp
  src/visual_factor.cpp
)
target_include_directories(trajectory_manager PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
ament_target_dependencies(trajectory_manager
  builtin_interfaces
)
target_link_libraries(trajectory_manager Eigen3::Eigen)

add_executable(trajectory_manager_probe
  test/trajectory_manager_probe.cpp
)
target_include_directories(trajectory_manager_probe PRIVATE include)
target_link_libraries(trajectory_manager_probe
  trajectory_manager
  Eigen3::Eigen
)
ament_target_dependencies(trajectory_manager_probe builtin_interfaces)

add_executable(imu_propagator_probe
  test/imu_propagator_probe.cpp
)
target_include_directories(imu_propagator_probe PRIVATE include)
target_link_libraries(imu_propagator_probe
  trajectory_manager
  Eigen3::Eigen
)
ament_target_dependencies(imu_propagator_probe builtin_interfaces)

add_executable(lidar_factor_probe
  test/lidar_factor_probe.cpp
)
target_include_directories(lidar_factor_probe PRIVATE include)
target_link_libraries(lidar_factor_probe
  trajectory_manager
  Eigen3::Eigen
)
ament_target_dependencies(lidar_factor_probe builtin_interfaces)

add_executable(visual_factor_probe
  test/visual_factor_probe.cpp
)
target_include_directories(visual_factor_probe PRIVATE include)
target_link_libraries(visual_factor_probe
  trajectory_manager
)
ament_target_dependencies(visual_factor_probe builtin_interfaces)

add_executable(tracking_node
  src/tracking_node.cpp
)
target_include_directories(tracking_node PRIVATE include)
target_link_libraries(tracking_node
  trajectory_manager
  Eigen3::Eigen
)
ament_target_dependencies(tracking_node
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  tf2_ros
)

install(TARGETS trajectory_manager trajectory_manager_probe imu_propagator_probe lidar_factor_probe visual_factor_probe tracking_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

ament_package()
