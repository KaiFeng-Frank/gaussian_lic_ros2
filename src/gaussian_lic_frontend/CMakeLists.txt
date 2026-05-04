cmake_minimum_required(VERSION 3.16)
project(gaussian_lic_frontend)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(lic2_contract_adapter
  src/lic2_contract_adapter_node.cpp
)
ament_target_dependencies(lic2_contract_adapter
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  tf2_ros
)

install(TARGETS lic2_contract_adapter
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
