// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <string>

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <torch/torch.h>

#include <gaussian_lic_mapping/backend_config.hpp>
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
  torch::Tensor xyz_exp_avg;
  torch::Tensor xyz_exp_avg_sq;
  torch::Tensor features_dc_exp_avg;
  torch::Tensor features_dc_exp_avg_sq;
  torch::Tensor features_rest_exp_avg;
  torch::Tensor features_rest_exp_avg_sq;
  torch::Tensor scaling_exp_avg;
  torch::Tensor scaling_exp_avg_sq;
  torch::Tensor rotation_exp_avg;
  torch::Tensor rotation_exp_avg_sq;
  torch::Tensor opacity_exp_avg;
  torch::Tensor opacity_exp_avg_sq;
  torch::Tensor xyz_gradient_accum;
  torch::Tensor xyz_gradient_vector_accum;
  torch::Tensor xyz_gradient_denom;
  torch::Tensor max_radii2d;
  torch::Tensor visibility_miss_count;
  int sh_degree{0};
  size_t foreground_count{0};
  size_t skybox_count{0};
};

struct TorchOptimizationResult
{
  int steps{0};
  size_t supervised_count{0};
  size_t rasterized_visible_count{0};
  size_t projected_visible_count{0};
  size_t projection_depth_valid_count{0};
  size_t projection_finite_count{0};
  size_t projection_in_bounds_count{0};
  float projection_min_z{0.0F};
  float projection_max_z{0.0F};
  float projection_min_u{0.0F};
  float projection_max_u{0.0F};
  float projection_min_v{0.0F};
  float projection_max_v{0.0F};
  float photometric_l1{0.0F};
};

struct TorchRenderResult
{
  torch::Tensor rendered_image;
  torch::Tensor rendered_depth;
  torch::Tensor radii;
  torch::Tensor final_transmittance;
  size_t visible_count{0};
};

struct TorchPruneResult
{
  size_t before_count{0};
  size_t after_count{0};
  size_t removed_count{0};
};

struct TorchDensifyResult
{
  size_t before_count{0};
  size_t after_count{0};
  size_t cloned_count{0};
  size_t split_parent_count{0};
  size_t split_child_count{0};
  size_t removed_parent_count{0};
};

struct TorchOpacityResetResult
{
  size_t reset_count{0};
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

TorchGaussianMap initialize_gaussian_map(
  const MapperDataset & dataset,
  const GaussianBackendConfig & config,
  double fx,
  double fy,
  torch::Device device = torch::kCPU);

size_t append_pending_points_to_gaussian_map(
  TorchGaussianMap & map,
  const MapperDataset & dataset,
  int sh_degree,
  double scaling_scale,
  double fx,
  double fy,
  torch::Device device = torch::kCPU);

size_t append_pending_points_to_gaussian_map(
  TorchGaussianMap & map,
  const MapperDataset & dataset,
  const GaussianBackendConfig & config,
  double fx,
  double fy,
  torch::Device device = torch::kCPU);

size_t append_pending_points_to_gaussian_map(
  TorchGaussianMap & map,
  const MapperDataset & dataset,
  const CameraFrameRecord & current_keyframe,
  const GaussianBackendConfig & config,
  double fx,
  double fy,
  double cx,
  double cy,
  torch::Device device = torch::kCPU);

TorchOptimizationResult optimize_gaussian_map_from_camera(
  TorchGaussianMap & map,
  const TorchCamera & camera,
  const GaussianBackendConfig & config,
  int steps,
  torch::Device device = torch::kCPU);

TorchRenderResult render_gaussian_map_from_camera(
  const TorchGaussianMap & map,
  const TorchCamera & camera,
  const GaussianBackendConfig & config,
  torch::Device device = torch::kCPU);

TorchDensifyResult densify_gaussian_map(
  TorchGaussianMap & map,
  const GaussianBackendConfig & config);

TorchPruneResult prune_gaussian_map(
  TorchGaussianMap & map,
  const GaussianBackendConfig & config);

TorchOpacityResetResult reset_gaussian_opacity(
  TorchGaussianMap & map,
  const GaussianBackendConfig & config);

}  // namespace gaussian_lic_mapping
