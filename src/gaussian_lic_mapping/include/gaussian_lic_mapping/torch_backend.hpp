// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <string>

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <torch/torch.h>

#include <gaussian_lic_mapping/mapper_dataset.hpp>

namespace gaussian_lic_mapping
{

struct TorchCamera
{
  std::string image_name;
  int image_width{0};
  int image_height{0};
  float fx{0.0F};
  float fy{0.0F};
  float cx{0.0F};
  float cy{0.0F};
  float fov_x{0.0F};
  float fov_y{0.0F};
  torch::Tensor original_image;
  torch::Tensor original_depth;
  torch::Tensor world_view_transform;
  torch::Tensor projection_matrix;
  torch::Tensor full_proj_transform;
  torch::Tensor camera_center;
};

struct TorchGaussianMap
{
  torch::Tensor xyz;
  torch::Tensor features_dc;
  torch::Tensor features_rest;
  torch::Tensor scaling;
  torch::Tensor rotation;
  torch::Tensor opacity;
  int sh_degree{0};
  size_t foreground_count{0};
  size_t skybox_count{0};
};

torch::Tensor cv_mat_to_torch_tensor_float32(
  const cv::Mat & mat,
  torch::Device device,
  bool use_pinned_memory = false);

TorchCamera make_torch_camera(
  const CameraFrameRecord & frame,
  double fx,
  double fy,
  double cx,
  double cy,
  torch::Device image_device = torch::kCPU,
  torch::Device geometry_device = torch::kCPU);

TorchGaussianMap initialize_gaussian_map(
  const MapperDataset & dataset,
  int sh_degree,
  double scaling_scale,
  double fx,
  double fy,
  torch::Device device = torch::kCPU,
  int skybox_points_num = 0,
  double skybox_radius = 1000.0);

size_t append_pending_points_to_gaussian_map(
  TorchGaussianMap & map,
  const MapperDataset & dataset,
  int sh_degree,
  double scaling_scale,
  double fx,
  double fy,
  torch::Device device = torch::kCPU);

}  // namespace gaussian_lic_mapping
