cmake_minimum_required(VERSION 3.16)
project(gaussian_lic_tracking)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(gaussian_lic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(trajectory_manager
  src/gaussian_snapshot.cpp
  src/imu_preintegrator.cpp
  src/imu_propagator.cpp
  src/lidar_deskew.cpp
  src/lidar_factor.cpp
  src/sliding_window_optimizer.cpp
  src/trajectory_manager.cpp
  src/visual_factor.cpp
)
target_include_directories(trajectory_manager PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
ament_target_dependencies(trajectory_manager
  builtin_interfaces
  gaussian_lic_msgs
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

add_executable(gaussian_snapshot_probe
  test/gaussian_snapshot_probe.cpp
)
target_include_directories(gaussian_snapshot_probe PRIVATE include)
target_link_libraries(gaussian_snapshot_probe
  trajectory_manager
  Eigen3::Eigen
)
ament_target_dependencies(gaussian_snapshot_probe builtin_interfaces gaussian_lic_msgs)

add_executable(imu_propagator_probe
  test/imu_propagator_probe.cpp
)
target_include_directories(imu_propagator_probe PRIVATE include)
target_link_libraries(imu_propagator_probe
  trajectory_manager
  Eigen3::Eigen
)
ament_target_dependencies(imu_propagator_probe builtin_interfaces)

add_executable(imu_preintegrator_probe
  test/imu_preintegrator_probe.cpp
)
target_include_directories(imu_preintegrator_probe PRIVATE include)
target_link_libraries(imu_preintegrator_probe
  trajectory_manager
  Eigen3::Eigen
)
ament_target_dependencies(imu_preintegrator_probe builtin_interfaces)

add_executable(lidar_factor_probe
  test/lidar_factor_probe.cpp
)
target_include_directories(lidar_factor_probe PRIVATE include)
target_link_libraries(lidar_factor_probe
  trajectory_manager
  Eigen3::Eigen
)
ament_target_dependencies(lidar_factor_probe builtin_interfaces)

add_executable(lidar_deskew_probe
  test/lidar_deskew_probe.cpp
)
target_include_directories(lidar_deskew_probe PRIVATE include)
target_link_libraries(lidar_deskew_probe
  trajectory_manager
  Eigen3::Eigen
)
ament_target_dependencies(lidar_deskew_probe builtin_interfaces)

add_executable(sliding_window_optimizer_probe
  test/sliding_window_optimizer_probe.cpp
)
target_include_directories(sliding_window_optimizer_probe PRIVATE include)
target_link_libraries(sliding_window_optimizer_probe
  trajectory_manager
  Eigen3::Eigen
)
ament_target_dependencies(sliding_window_optimizer_probe builtin_interfaces)

add_executable(sliding_window_bias_probe
  test/sliding_window_bias_probe.cpp
)
target_include_directories(sliding_window_bias_probe PRIVATE include)
target_link_libraries(sliding_window_bias_probe
  trajectory_manager
  Eigen3::Eigen
)
ament_target_dependencies(sliding_window_bias_probe builtin_interfaces)

add_executable(sliding_window_bias_observability_probe
  test/sliding_window_bias_observability_probe.cpp
)
target_include_directories(sliding_window_bias_observability_probe PRIVATE include)
target_link_libraries(sliding_window_bias_observability_probe
  trajectory_manager
  Eigen3::Eigen
)
ament_target_dependencies(sliding_window_bias_observability_probe builtin_interfaces)

add_executable(sliding_window_imu_bias_jacobian_probe
  test/sliding_window_imu_bias_jacobian_probe.cpp
)
target_include_directories(sliding_window_imu_bias_jacobian_probe PRIVATE include)
target_link_libraries(sliding_window_imu_bias_jacobian_probe
  trajectory_manager
  Eigen3::Eigen
)
ament_target_dependencies(sliding_window_imu_bias_jacobian_probe builtin_interfaces)

add_executable(sliding_window_imu_kinematic_jacobian_probe
  test/sliding_window_imu_kinematic_jacobian_probe.cpp
)
target_include_directories(sliding_window_imu_kinematic_jacobian_probe PRIVATE include)
target_link_libraries(sliding_window_imu_kinematic_jacobian_probe
  trajectory_manager
  Eigen3::Eigen
)
ament_target_dependencies(sliding_window_imu_kinematic_jacobian_probe builtin_interfaces)

add_executable(sliding_window_marginalization_probe
  test/sliding_window_marginalization_probe.cpp
)
target_include_directories(sliding_window_marginalization_probe PRIVATE include)
target_link_libraries(sliding_window_marginalization_probe
  trajectory_manager
  Eigen3::Eigen
)
ament_target_dependencies(sliding_window_marginalization_probe builtin_interfaces)

add_executable(sliding_window_dense_prior_probe
  test/sliding_window_dense_prior_probe.cpp
)
target_include_directories(sliding_window_dense_prior_probe PRIVATE include)
target_link_libraries(sliding_window_dense_prior_probe
  trajectory_manager
  Eigen3::Eigen
)
ament_target_dependencies(sliding_window_dense_prior_probe builtin_interfaces)

add_executable(schur_complement_probe
  test/schur_complement_probe.cpp
)
target_include_directories(schur_complement_probe PRIVATE include)
target_link_libraries(schur_complement_probe
  trajectory_manager
  Eigen3::Eigen
)
ament_target_dependencies(schur_complement_probe builtin_interfaces)

add_executable(sliding_window_normal_equation_probe
  test/sliding_window_normal_equation_probe.cpp
)
target_include_directories(sliding_window_normal_equation_probe PRIVATE include)
target_link_libraries(sliding_window_normal_equation_probe
  trajectory_manager
  Eigen3::Eigen
)
ament_target_dependencies(sliding_window_normal_equation_probe builtin_interfaces)

add_executable(sliding_window_schur_marginalization_probe
  test/sliding_window_schur_marginalization_probe.cpp
)
target_include_directories(sliding_window_schur_marginalization_probe PRIVATE include)
target_link_libraries(sliding_window_schur_marginalization_probe
  trajectory_manager
  Eigen3::Eigen
)
ament_target_dependencies(sliding_window_schur_marginalization_probe builtin_interfaces)

add_executable(sliding_window_point_factor_probe
  test/sliding_window_point_factor_probe.cpp
)
target_include_directories(sliding_window_point_factor_probe PRIVATE include)
target_link_libraries(sliding_window_point_factor_probe
  trajectory_manager
  Eigen3::Eigen
)
ament_target_dependencies(sliding_window_point_factor_probe builtin_interfaces)

add_executable(sliding_window_plane_factor_probe
  test/sliding_window_plane_factor_probe.cpp
)
target_include_directories(sliding_window_plane_factor_probe PRIVATE include)
target_link_libraries(sliding_window_plane_factor_probe
  trajectory_manager
  Eigen3::Eigen
)
ament_target_dependencies(sliding_window_plane_factor_probe builtin_interfaces)

add_executable(sliding_window_geometric_jacobian_probe
  test/sliding_window_geometric_jacobian_probe.cpp
)
target_include_directories(sliding_window_geometric_jacobian_probe PRIVATE include)
target_link_libraries(sliding_window_geometric_jacobian_probe
  trajectory_manager
  Eigen3::Eigen
)
ament_target_dependencies(sliding_window_geometric_jacobian_probe builtin_interfaces)

add_executable(sliding_window_prior_jacobian_probe
  test/sliding_window_prior_jacobian_probe.cpp
)
target_include_directories(sliding_window_prior_jacobian_probe PRIVATE include)
target_link_libraries(sliding_window_prior_jacobian_probe
  trajectory_manager
  Eigen3::Eigen
)
ament_target_dependencies(sliding_window_prior_jacobian_probe builtin_interfaces)

add_executable(sliding_window_visual_factor_probe
  test/sliding_window_visual_factor_probe.cpp
)
target_include_directories(sliding_window_visual_factor_probe PRIVATE include)
target_link_libraries(sliding_window_visual_factor_probe
  trajectory_manager
  Eigen3::Eigen
)
ament_target_dependencies(sliding_window_visual_factor_probe builtin_interfaces)

add_executable(sliding_window_se3_photometric_factor_probe
  test/sliding_window_se3_photometric_factor_probe.cpp
)
target_include_directories(sliding_window_se3_photometric_factor_probe PRIVATE include)
target_link_libraries(sliding_window_se3_photometric_factor_probe
  trajectory_manager
  Eigen3::Eigen
)
ament_target_dependencies(sliding_window_se3_photometric_factor_probe builtin_interfaces)

add_executable(visual_factor_probe
  test/visual_factor_probe.cpp
)
target_include_directories(visual_factor_probe PRIVATE include)
target_link_libraries(visual_factor_probe
  trajectory_manager
)
ament_target_dependencies(visual_factor_probe builtin_interfaces)

add_executable(visual_se3_jacobian_probe
  test/visual_se3_jacobian_probe.cpp
)
target_include_directories(visual_se3_jacobian_probe PRIVATE include)
target_link_libraries(visual_se3_jacobian_probe
  trajectory_manager
  Eigen3::Eigen
)
ament_target_dependencies(visual_se3_jacobian_probe builtin_interfaces)

add_executable(tracking_node
  src/tracking_node.cpp
)
target_include_directories(tracking_node PRIVATE include)
target_link_libraries(tracking_node
  trajectory_manager
  Eigen3::Eigen
)
ament_target_dependencies(tracking_node
  gaussian_lic_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  tf2_ros
)

install(TARGETS trajectory_manager trajectory_manager_probe gaussian_snapshot_probe imu_propagator_probe imu_preintegrator_probe lidar_factor_probe lidar_deskew_probe sliding_window_optimizer_probe sliding_window_bias_probe sliding_window_bias_observability_probe sliding_window_imu_bias_jacobian_probe sliding_window_imu_kinematic_jacobian_probe sliding_window_marginalization_probe sliding_window_dense_prior_probe schur_complement_probe sliding_window_normal_equation_probe sliding_window_schur_marginalization_probe sliding_window_point_factor_probe sliding_window_plane_factor_probe sliding_window_geometric_jacobian_probe sliding_window_prior_jacobian_probe sliding_window_visual_factor_probe sliding_window_se3_photometric_factor_probe visual_factor_probe visual_se3_jacobian_probe tracking_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  enable_testing()
  add_test(NAME trajectory_manager_probe COMMAND $<TARGET_FILE:trajectory_manager_probe>)
  add_test(NAME gaussian_snapshot_probe COMMAND $<TARGET_FILE:gaussian_snapshot_probe>)
  add_test(NAME imu_propagator_probe COMMAND $<TARGET_FILE:imu_propagator_probe>)
  add_test(NAME imu_preintegrator_probe COMMAND $<TARGET_FILE:imu_preintegrator_probe>)
  add_test(NAME lidar_factor_probe COMMAND $<TARGET_FILE:lidar_factor_probe>)
  add_test(NAME lidar_deskew_probe COMMAND $<TARGET_FILE:lidar_deskew_probe>)
  add_test(NAME sliding_window_optimizer_probe COMMAND $<TARGET_FILE:sliding_window_optimizer_probe>)
  add_test(NAME sliding_window_bias_probe COMMAND $<TARGET_FILE:sliding_window_bias_probe>)
  add_test(NAME sliding_window_bias_observability_probe COMMAND $<TARGET_FILE:sliding_window_bias_observability_probe>)
  add_test(NAME sliding_window_imu_bias_jacobian_probe COMMAND $<TARGET_FILE:sliding_window_imu_bias_jacobian_probe>)
  add_test(NAME sliding_window_imu_kinematic_jacobian_probe COMMAND $<TARGET_FILE:sliding_window_imu_kinematic_jacobian_probe>)
  add_test(NAME sliding_window_marginalization_probe COMMAND $<TARGET_FILE:sliding_window_marginalization_probe>)
  add_test(NAME sliding_window_dense_prior_probe COMMAND $<TARGET_FILE:sliding_window_dense_prior_probe>)
  add_test(NAME schur_complement_probe COMMAND $<TARGET_FILE:schur_complement_probe>)
  add_test(NAME sliding_window_normal_equation_probe COMMAND $<TARGET_FILE:sliding_window_normal_equation_probe>)
  add_test(NAME sliding_window_schur_marginalization_probe COMMAND $<TARGET_FILE:sliding_window_schur_marginalization_probe>)
  add_test(NAME sliding_window_point_factor_probe COMMAND $<TARGET_FILE:sliding_window_point_factor_probe>)
  add_test(NAME sliding_window_plane_factor_probe COMMAND $<TARGET_FILE:sliding_window_plane_factor_probe>)
  add_test(NAME sliding_window_geometric_jacobian_probe COMMAND $<TARGET_FILE:sliding_window_geometric_jacobian_probe>)
  add_test(NAME sliding_window_prior_jacobian_probe COMMAND $<TARGET_FILE:sliding_window_prior_jacobian_probe>)
  add_test(NAME sliding_window_visual_factor_probe COMMAND $<TARGET_FILE:sliding_window_visual_factor_probe>)
  add_test(NAME sliding_window_se3_photometric_factor_probe COMMAND $<TARGET_FILE:sliding_window_se3_photometric_factor_probe>)
  add_test(NAME visual_factor_probe COMMAND $<TARGET_FILE:visual_factor_probe>)
  add_test(NAME visual_se3_jacobian_probe COMMAND $<TARGET_FILE:visual_se3_jacobian_probe>)
endif()

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
