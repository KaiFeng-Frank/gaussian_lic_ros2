// SPDX-License-Identifier: GPL-3.0-or-later

#include "rasterize_points.h"

#include <cuda_runtime.h>
#include <torch/torch.h>

#include <cmath>
#include <iostream>

namespace
{

torch::Tensor empty_cuda_float(const torch::TensorOptions & options)
{
  return torch::empty({0}, options);
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
  constexpr int gaussian_count = 1;
  const auto float_options = torch::TensorOptions().dtype(torch::kFloat32).device(torch::kCUDA);

  auto background = torch::zeros({3}, float_options).contiguous();
  auto means3d = torch::tensor({{0.0F, 0.0F, 1.0F}}, float_options).contiguous();
  auto colors = torch::tensor({{0.9F, 0.2F, 0.1F}}, float_options).contiguous();
  auto opacity = torch::full({gaussian_count, 1}, 0.99F, float_options).contiguous();
  auto scales = torch::full({gaussian_count, 3}, 1.00F, float_options).contiguous();
  auto rotations = torch::tensor({{1.0F, 0.0F, 0.0F, 0.0F}}, float_options).contiguous();
  auto cov3d_precomp = empty_cuda_float(float_options);
  auto viewmatrix = torch::eye(4, float_options).contiguous();
  constexpr float znear = 0.01F;
  constexpr float zfar = 100.0F;
  constexpr float zscale = zfar / (zfar - znear);
  constexpr float zoffset = -(zfar * znear) / (zfar - znear);
  auto projmatrix = torch::tensor(
    {{1.0F, 0.0F, 0.0F, 0.0F},
     {0.0F, 1.0F, 0.0F, 0.0F},
     {0.0F, 0.0F, zscale, 1.0F},
     {0.0F, 0.0F, zoffset, 0.0F}},
    float_options).contiguous();
  auto dc = torch::zeros({gaussian_count, 1, 3}, float_options).contiguous();
  auto sh = torch::zeros({gaussian_count, 15, 3}, float_options).contiguous();
  auto campos = torch::zeros({3}, float_options).contiguous();

  constexpr float tan_fovx = 1.0F;
  constexpr float tan_fovy = 1.0F;
  constexpr float limit_neg = -1.3F;
  constexpr float limit_pos = 1.3F;

  auto forward = RasterizeGaussiansCUDA(
    background,
    means3d,
    colors,
    opacity,
    scales,
    rotations,
    1.0F,
    cov3d_precomp,
    viewmatrix,
    projmatrix,
    tan_fovx,
    tan_fovy,
    height,
    width,
    limit_neg,
    limit_pos,
    limit_neg,
    limit_pos,
    dc,
    sh,
    3,
    campos,
    false,
    true,
    false);
  cudaDeviceSynchronize();

  const int rendered = std::get<0>(forward);
  const int bucket_count = std::get<1>(forward);
  const auto color = std::get<2>(forward);
  const auto final_transmittance = std::get<3>(forward);
  const auto depth = std::get<4>(forward);
  const auto radii = std::get<5>(forward);
  const auto geom_buffer = std::get<6>(forward);
  const auto binning_buffer = std::get<7>(forward);
  const auto image_buffer = std::get<8>(forward);
  const auto sample_buffer = std::get<9>(forward);

  const auto max_color = color.max().item<float>();
  const auto max_depth = depth.max().item<float>();
  const auto max_radius = radii.max().item<int>();
  const auto min_final_t = final_transmittance.min().item<float>();

  auto backward = RasterizeGaussiansBackwardCUDA(
    background,
    means3d,
    radii,
    colors,
    scales,
    rotations,
    1.0F,
    cov3d_precomp,
    viewmatrix,
    projmatrix,
    tan_fovx,
    tan_fovy,
    limit_neg,
    limit_pos,
    limit_neg,
    limit_pos,
    torch::ones_like(color).contiguous(),
    torch::ones_like(depth).contiguous(),
    dc,
    sh,
    3,
    campos,
    geom_buffer,
    rendered,
    binning_buffer,
    image_buffer,
    bucket_count,
    sample_buffer,
    0.0F,
    true);
  cudaDeviceSynchronize();

  const auto grad_means = std::get<3>(backward);
  const auto grad_opacity = std::get<2>(backward);
  const auto grad_scales = std::get<7>(backward);
  const auto grad_rotations = std::get<8>(backward);
  const bool gradients_finite =
    torch::isfinite(grad_means).all().item<bool>() &&
    torch::isfinite(grad_opacity).all().item<bool>() &&
    torch::isfinite(grad_scales).all().item<bool>() &&
    torch::isfinite(grad_rotations).all().item<bool>();

  std::cout << "rasterizer_probe rendered=" << rendered
            << " buckets=" << bucket_count
            << " max_color=" << max_color
            << " max_depth=" << max_depth
            << " max_radius=" << max_radius
            << " min_final_T=" << min_final_t
            << " gradients_finite=" << (gradients_finite ? "true" : "false") << "\n";

  if (rendered <= 0 || bucket_count <= 0) {
    std::cerr << "rasterizer forward produced no rendered samples\n";
    return 1;
  }
  if (!std::isfinite(max_color) || max_color <= 0.0F) {
    std::cerr << "rasterizer forward produced blank color\n";
    return 1;
  }
  if (!std::isfinite(max_depth) || max_depth <= 0.0F) {
    std::cerr << "rasterizer forward produced blank depth\n";
    return 1;
  }
  if (max_radius <= 0 || !std::isfinite(min_final_t) || min_final_t >= 1.0F) {
    std::cerr << "rasterizer forward produced invalid radii/transmittance\n";
    return 1;
  }
  if (!gradients_finite) {
    std::cerr << "rasterizer backward produced non-finite gradients\n";
    return 1;
  }

  std::cout << "rasterizer_probe OK\n";
  return 0;
}
