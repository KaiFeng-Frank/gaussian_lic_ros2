cmake_minimum_required(VERSION 3.16)
project(gaussian_lic_tracking)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(trajectory_manager
  src/imu_propagator.cpp
  src/trajectory_manager.cpp
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

install(TARGETS trajectory_manager trajectory_manager_probe imu_propagator_probe
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

ament_package()
