// SPDX-License-Identifier: GPL-3.0-or-later

#include <cuda_runtime.h>
#include <torch/torch.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <utility>

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include <gaussian_lic_mapping/backend_config.hpp>
#include <gaussian_lic_mapping/torch_backend.hpp>

namespace
{

float logit(const float value)
{
  return std::log(value / (1.0F - value));
}

cv::Mat make_target_image(const int height, const int width)
{
  cv::Mat image(height, width, CV_32FC3);
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const float u = static_cast<float>(x) / static_cast<float>(std::max(width - 1, 1));
      const float v = static_cast<float>(y) / static_cast<float>(std::max(height - 1, 1));
      image.at<cv::Vec3f>(y, x) = cv::Vec3f(
        0.10F + 0.80F * u,
        0.20F + 0.60F * v,
        0.80F - 0.50F * u);
    }
  }
  return image;
}

gaussian_lic_mapping::MapperFrameData make_frame_data(const cv::Mat & image, const cv::Mat & depth)
{
  gaussian_lic_mapping::MapperFrameData frame;
  frame.frame_index = 0;
  frame.is_keyframe = true;
  frame.width = image.cols;
  frame.height = image.rows;
  frame.image_rgb_float = image.clone();
  frame.depth_m_float = depth.clone();

  const std::array<Eigen::Vector3f, 3> xyz{
    Eigen::Vector3f(0.0F, 0.0F, 1.0F),
    Eigen::Vector3f(-0.18F, 0.10F, 1.1F),
    Eigen::Vector3f(0.20F, -0.12F, 0.9F)};
  const std::array<Eigen::Vector3f, 3> color{
    Eigen::Vector3f(0.25F, 0.25F, 0.25F),
    Eigen::Vector3f(0.10F, 0.35F, 0.55F),
    Eigen::Vector3f(0.60F, 0.20F, 0.15F)};

  for (size_t i = 0; i < xyz.size(); ++i) {
    gaussian_lic_mapping::MapperPoint point;
    point.xyz_world = xyz[i];
    point.color_rgb = color[i];
    point.depth_m = xyz[i].z();
    frame.points.push_back(point);
  }
  return frame;
}

double max_delta(const torch::Tensor & before, const torch::Tensor & after)
{
  return torch::max(torch::abs(after.detach() - before.detach())).to(torch::kCPU).item<double>();
}

}  // namespace

int main()
{
  if (!torch::cuda::is_available()) {
    std::cerr << "CUDA is not available\n";
    return 2;
  }

  constexpr int width = 96;
  constexpr int height = 64;
  const auto image = make_target_image(height, width);
  const auto depth = cv::Mat(height, width, CV_32FC1, cv::Scalar(1.0F));

  gaussian_lic_mapping::MapperDataset dataset;
  dataset.add_frame(make_frame_data(image, depth));
  auto map = gaussian_lic_mapping::initialize_gaussian_map(
    dataset,
    3,
    48.0,
    48.0,
    48.0,
    torch::kCUDA);

  {
    torch::NoGradGuard no_grad;
    map.opacity.fill_(logit(0.85F));
    map.scaling.copy_(torch::tensor(
      {{std::log(0.95F), std::log(0.62F), std::log(0.28F)},
       {std::log(0.85F), std::log(0.45F), std::log(0.32F)},
       {std::log(0.65F), std::log(0.90F), std::log(0.25F)}},
      torch::TensorOptions().dtype(torch::kFloat32).device(torch::kCUDA)));
    map.rotation.copy_(torch::tensor(
      {{0.9238795F, 0.0F, 0.0F, 0.3826834F},
       {0.9659258F, 0.0F, 0.2588190F, 0.0F},
       {0.9659258F, 0.2588190F, 0.0F, 0.0F}},
      torch::TensorOptions().dtype(torch::kFloat32).device(torch::kCUDA)));
  }

  gaussian_lic_mapping::CameraFrameRecord frame;
  frame.frame_index = 0;
  frame.is_keyframe = true;
  frame.image_name = "cuda_backend_probe";
  frame.width = width;
  frame.height = height;
  frame.image_rgb_float = image.clone();
  frame.depth_m_float = depth.clone();
  const auto camera = gaussian_lic_mapping::make_torch_camera(
    frame,
    48.0,
    48.0,
    static_cast<double>(width) * 0.5,
    static_cast<double>(height) * 0.5,
    torch::kCUDA,
    torch::kCUDA);

  const auto xyz_before = map.xyz.detach().clone();
  const auto dc_before = map.features_dc.detach().clone();
  const auto rest_before = map.features_rest.detach().clone();
  const auto scaling_before = map.scaling.detach().clone();
  const auto rotation_before = map.rotation.detach().clone();
  const auto opacity_before = map.opacity.detach().clone();

  gaussian_lic_mapping::GaussianBackendConfig config;
  config.enable_photometric_optimization = true;
  config.position_lr = 0.0005;
  config.feature_lr = 0.01;
  config.opacity_lr = 0.02;
  config.scaling_lr = 0.005;
  config.rotation_lr = 0.002;
  config.lambda_dssim = 0.2;
  config.lambda_depth = 0.005;
  config.optimize_depth = true;
  config.white_background = false;
  const auto result = gaussian_lic_mapping::optimize_gaussian_map_from_camera(
    map, camera, config, 2, torch::kCUDA);
  const auto render_result = gaussian_lic_mapping::render_gaussian_map_from_camera(
    map, camera, config, torch::kCUDA);
  cudaDeviceSynchronize();

  const double xyz_delta = max_delta(xyz_before, map.xyz);
  const double dc_delta = max_delta(dc_before, map.features_dc);
  const double rest_delta = max_delta(rest_before, map.features_rest);
  const double scaling_delta = max_delta(scaling_before, map.scaling);
  const double rotation_delta = max_delta(rotation_before, map.rotation);
  const double opacity_delta = max_delta(opacity_before, map.opacity);
  const size_t count_before_densify = map.foreground_count + map.skybox_count;
  config.enable_density_control = true;
  config.enable_densification = true;
  config.densify_grad_threshold = 0.0;
  config.densify_scene_extent = 1.0;
  config.densify_percent_dense = 0.50;
  config.densify_max_new_gaussians = 8;
  const auto densify_result = gaussian_lic_mapping::densify_gaussian_map(map, config);
  config.enable_densification = false;
  config.max_foreground_gaussians = 4;
  config.prune_min_opacity = 0.0;
  const auto prune_result = gaussian_lic_mapping::prune_gaussian_map(map, config);
  const auto reset_result = gaussian_lic_mapping::reset_gaussian_opacity(map, config);
  const bool adam_state_defined =
    map.xyz_exp_avg.defined() &&
    map.features_dc_exp_avg.defined() &&
    map.features_rest_exp_avg.defined() &&
    map.scaling_exp_avg.defined() &&
    map.rotation_exp_avg.defined() &&
    map.opacity_exp_avg.defined();
  const bool finite_loss = std::isfinite(result.photometric_l1);
  const auto max_render = render_result.rendered_image.max().to(torch::kCPU).item<float>();

  std::cout << "torch_cuda_backend_probe steps=" << result.steps
            << " supervised=" << result.supervised_count
            << " l1=" << result.photometric_l1
            << " render_visible=" << render_result.visible_count
            << " max_render=" << max_render
            << " xyz_delta=" << xyz_delta
            << " dc_delta=" << dc_delta
            << " rest_delta=" << rest_delta
            << " scaling_delta=" << scaling_delta
            << " rotation_delta=" << rotation_delta
            << " opacity_delta=" << opacity_delta
            << " densify_before=" << count_before_densify
            << " densify_after=" << densify_result.after_count
            << " cloned=" << densify_result.cloned_count
            << " split_children=" << densify_result.split_child_count
            << " prune_removed=" << prune_result.removed_count
            << " reset_count=" << reset_result.reset_count
            << " adam_state_defined=" << (adam_state_defined ? "true" : "false")
            << "\n";

  if (result.steps != 2 || result.supervised_count == 0 || !finite_loss) {
    std::cerr << "CUDA backend optimization did not produce a valid supervised loss\n";
    return 1;
  }
  if (render_result.visible_count == 0 || !std::isfinite(max_render) || max_render <= 0.0F) {
    std::cerr << "CUDA backend public render path produced a blank preview\n";
    return 1;
  }
  if (!adam_state_defined) {
    std::cerr << "CUDA backend did not initialize sparse Adam state for all parameter groups\n";
    return 1;
  }
  if (dc_delta <= 0.0 || opacity_delta <= 0.0) {
    std::cerr << "CUDA backend did not update color and opacity parameter groups\n";
    return 1;
  }
  if (xyz_delta <= 0.0 && scaling_delta <= 0.0 && rotation_delta <= 0.0 && rest_delta <= 0.0) {
    std::cerr << "CUDA backend did not update any geometric/high-order parameter group\n";
    return 1;
  }
  if (densify_result.after_count <= count_before_densify || densify_result.split_child_count == 0) {
    std::cerr << "CUDA backend densification did not split high-gradient Gaussians\n";
    return 1;
  }
  if (prune_result.removed_count == 0 || reset_result.reset_count == 0) {
    std::cerr << "CUDA backend density-control prune/reset did not execute\n";
    return 1;
  }

  std::cout << "torch_cuda_backend_probe OK\n";
  return 0;
}
