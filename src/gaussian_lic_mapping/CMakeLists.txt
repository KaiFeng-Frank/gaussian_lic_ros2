cmake_minimum_required(VERSION 3.16)
project(gaussian_lic_mapping)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
option(GAUSSIAN_LIC_ENABLE_TORCH "Build optional libtorch/CUDA mapper backend probes" OFF)
option(GAUSSIAN_LIC_ENABLE_CUDA "Enable strict CUDA/Torch Gaussian backend build plumbing" OFF)

if(GAUSSIAN_LIC_ENABLE_CUDA AND NOT GAUSSIAN_LIC_ENABLE_TORCH)
  message(FATAL_ERROR "GAUSSIAN_LIC_ENABLE_CUDA requires GAUSSIAN_LIC_ENABLE_TORCH=ON")
endif()

if(GAUSSIAN_LIC_ENABLE_CUDA)
  enable_language(CUDA)
  if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES OR CMAKE_CUDA_ARCHITECTURES STREQUAL "")
    set(CMAKE_CUDA_ARCHITECTURES 86 89 100 120)
  endif()
  set(CMAKE_CUDA_STANDARD 17)
  set(CMAKE_CUDA_STANDARD_REQUIRED ON)
  find_package(CUDAToolkit REQUIRED)
endif()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(gaussian_lic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

set(MAPPING_NODE_SOURCES
  src/frame_data.cpp
  src/mapper_dataset.cpp
  src/mapping_node.cpp
)

set(MAPPING_NODE_DEPS
  cv_bridge
  gaussian_lic_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  tf2_ros
)

add_executable(mapping_node
  ${MAPPING_NODE_SOURCES}
)
target_include_directories(mapping_node PRIVATE
  include
)
ament_target_dependencies(mapping_node
  ${MAPPING_NODE_DEPS}
)
target_link_libraries(mapping_node
  Eigen3::Eigen
  ${OpenCV_LIBS}
)

add_library(mapping_node_component SHARED
  ${MAPPING_NODE_SOURCES}
)
target_compile_definitions(mapping_node_component PRIVATE GAUSSIAN_LIC_MAPPING_COMPOSITION=1)
target_include_directories(mapping_node_component PRIVATE
  include
)
ament_target_dependencies(mapping_node_component
  ${MAPPING_NODE_DEPS}
  rclcpp_components
)
target_link_libraries(mapping_node_component
  Eigen3::Eigen
  ${OpenCV_LIBS}
)
rclcpp_components_register_nodes(mapping_node_component "MappingNode")

install(TARGETS mapping_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS mapping_node_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(GAUSSIAN_LIC_ENABLE_TORCH)
  if(NOT Torch_DIR AND EXISTS "$ENV{HOME}/Software/libtorch/share/cmake/Torch")
    set(Torch_DIR "$ENV{HOME}/Software/libtorch/share/cmake/Torch" CACHE PATH "Path to TorchConfig.cmake")
  endif()

  find_package(Torch REQUIRED)

  add_library(torch_backend src/torch_backend.cpp)
  set_target_properties(torch_backend PROPERTIES POSITION_INDEPENDENT_CODE ON)
  target_include_directories(torch_backend PUBLIC include)
  ament_target_dependencies(torch_backend
    geometry_msgs
    sensor_msgs
  )
  target_compile_options(torch_backend PRIVATE ${TORCH_CXX_FLAGS})
  target_link_libraries(torch_backend
    Eigen3::Eigen
    ${OpenCV_LIBS}
    ${TORCH_LIBRARIES}
  )
  if(GAUSSIAN_LIC_ENABLE_CUDA)
    add_library(gaussian_lic_cuda_ops
      cuda/simple_knn/simple_knn.cu
      cuda/simple_knn/spatial.cu
      cuda/sparse_adam/sparse_adam.cu
    )
    set_target_properties(gaussian_lic_cuda_ops PROPERTIES
      POSITION_INDEPENDENT_CODE ON
      CUDA_SEPARABLE_COMPILATION ON
    )
    target_include_directories(gaussian_lic_cuda_ops PUBLIC
      include
      cuda/simple_knn
    )
    target_compile_options(gaussian_lic_cuda_ops PRIVATE ${TORCH_CXX_FLAGS})
    target_link_libraries(gaussian_lic_cuda_ops
      ${TORCH_LIBRARIES}
      CUDA::cudart
    )

    target_compile_definitions(torch_backend PUBLIC GAUSSIAN_LIC_ENABLE_CUDA=1)
    target_link_libraries(torch_backend gaussian_lic_cuda_ops CUDA::cudart)
  endif()

  add_executable(torch_backend_probe
    src/mapper_dataset.cpp
    src/torch_backend_probe.cpp
  )
  target_include_directories(torch_backend_probe PRIVATE include)
  target_compile_options(torch_backend_probe PRIVATE ${TORCH_CXX_FLAGS})
  target_link_libraries(torch_backend_probe
    torch_backend
    ${TORCH_LIBRARIES}
  )
  if(GAUSSIAN_LIC_ENABLE_CUDA)
    target_compile_definitions(torch_backend_probe PRIVATE GAUSSIAN_LIC_ENABLE_CUDA=1)
    target_link_libraries(torch_backend_probe CUDA::cudart)

    add_executable(simple_knn_probe src/simple_knn_probe.cpp)
    target_include_directories(simple_knn_probe PRIVATE include)
    target_compile_options(simple_knn_probe PRIVATE ${TORCH_CXX_FLAGS})
    target_link_libraries(simple_knn_probe
      gaussian_lic_cuda_ops
      ${TORCH_LIBRARIES}
      CUDA::cudart
    )

    add_executable(sparse_adam_probe src/sparse_adam_probe.cpp)
    target_include_directories(sparse_adam_probe PRIVATE include)
    target_compile_options(sparse_adam_probe PRIVATE ${TORCH_CXX_FLAGS})
    target_link_libraries(sparse_adam_probe
      gaussian_lic_cuda_ops
      ${TORCH_LIBRARIES}
      CUDA::cudart
    )
  endif()

  target_compile_definitions(mapping_node PRIVATE GAUSSIAN_LIC_ENABLE_TORCH=1)
  target_compile_options(mapping_node PRIVATE ${TORCH_CXX_FLAGS})
  target_link_libraries(mapping_node
    torch_backend
    ${TORCH_LIBRARIES}
  )
  target_compile_definitions(mapping_node_component PRIVATE GAUSSIAN_LIC_ENABLE_TORCH=1)
  target_compile_options(mapping_node_component PRIVATE ${TORCH_CXX_FLAGS})
  target_link_libraries(mapping_node_component
    torch_backend
    ${TORCH_LIBRARIES}
  )

  install(TARGETS torch_backend
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
  if(GAUSSIAN_LIC_ENABLE_CUDA)
    install(TARGETS gaussian_lic_cuda_ops
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION bin
    )
    install(TARGETS simple_knn_probe
      RUNTIME DESTINATION lib/${PROJECT_NAME}
    )
    install(TARGETS sparse_adam_probe
      RUNTIME DESTINATION lib/${PROJECT_NAME}
    )
  endif()
  install(TARGETS torch_backend_probe
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endif()

ament_package()
