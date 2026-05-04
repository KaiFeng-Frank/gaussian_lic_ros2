// SPDX-License-Identifier: GPL-3.0-or-later

#include <iostream>
#include <utility>

#include <opencv2/core.hpp>
#include <torch/torch.h>

#include <gaussian_lic_mapping/torch_backend.hpp>

int main()
{
  gaussian_lic_mapping::CameraFrameRecord frame;
  frame.frame_index = 0;
  frame.is_keyframe = true;
  frame.image_name = "train_0000.jpg";
  frame.width = 1;
  frame.height = 1;
  frame.image_rgb_float = cv::Mat(1, 1, CV_32FC3, cv::Scalar(0.25F, 0.5F, 0.75F)).clone();
  frame.depth_m_float = cv::Mat(1, 1, CV_32FC1, cv::Scalar(1.0F)).clone();

  const auto camera = gaussian_lic_mapping::make_torch_camera(
    frame, 1.0, 1.0, 0.5, 0.5, torch::kCPU, torch::kCPU);

  gaussian_lic_mapping::MapperDataset dataset;
  gaussian_lic_mapping::MapperFrameData frame_data;
  frame_data.frame_index = 0;
  frame_data.is_keyframe = true;
  frame_data.width = 1;
  frame_data.height = 1;
  frame_data.image_rgb_float = frame.image_rgb_float.clone();
  frame_data.depth_m_float = frame.depth_m_float.clone();
  gaussian_lic_mapping::MapperPoint point;
  point.xyz_world = Eigen::Vector3f(0.0F, 0.0F, 1.0F);
  point.color_rgb = Eigen::Vector3f(0.25F, 0.5F, 0.75F);
  point.depth_m = 1.0F;
  frame_data.points.push_back(point);
  dataset.add_frame(std::move(frame_data));
  auto gaussian_map = gaussian_lic_mapping::initialize_gaussian_map(
    dataset, 3, 1.0, 1.0, 1.0, torch::kCPU);
  const auto gaussian_count_after_init = gaussian_map.foreground_count + gaussian_map.skybox_count;

  dataset.clear_pending_points();
  gaussian_lic_mapping::MapperFrameData second_frame_data;
  second_frame_data.frame_index = 1;
  second_frame_data.is_keyframe = true;
  second_frame_data.width = 1;
  second_frame_data.height = 1;
  second_frame_data.image_rgb_float = frame.image_rgb_float.clone();
  second_frame_data.depth_m_float = frame.depth_m_float.clone();
  gaussian_lic_mapping::MapperPoint second_point;
  second_point.xyz_world = Eigen::Vector3f(0.1F, 0.0F, 1.0F);
  second_point.color_rgb = Eigen::Vector3f(0.75F, 0.5F, 0.25F);
  second_point.depth_m = 1.0F;
  second_frame_data.points.push_back(second_point);
  dataset.add_frame(std::move(second_frame_data));
  const auto appended_count = gaussian_lic_mapping::append_pending_points_to_gaussian_map(
    gaussian_map, dataset, 3, 1.0, 1.0, 1.0, torch::kCPU);

  std::cout << "torch_version=" << TORCH_VERSION << "\n";
  std::cout << "cuda_available=" << (torch::cuda::is_available() ? "true" : "false") << "\n";
  std::cout << "image_sizes=" << camera.original_image.sizes() << "\n";
  std::cout << "depth_sizes=" << camera.original_depth.sizes() << "\n";
  std::cout << "world_view_sizes=" << camera.world_view_transform.sizes() << "\n";
  std::cout << "projection_sizes=" << camera.projection_matrix.sizes() << "\n";
  std::cout << "gaussian_xyz_sizes=" << gaussian_map.xyz.sizes() << "\n";
  std::cout << "gaussian_features_dc_sizes=" << gaussian_map.features_dc.sizes() << "\n";
  std::cout << "gaussian_features_rest_sizes=" << gaussian_map.features_rest.sizes() << "\n";
  std::cout << "gaussian_count_after_init=" << gaussian_count_after_init << "\n";
  std::cout << "appended_count=" << appended_count << "\n";
  std::cout << "gaussian_count=" << gaussian_map.foreground_count + gaussian_map.skybox_count << "\n";
  return 0;
}
