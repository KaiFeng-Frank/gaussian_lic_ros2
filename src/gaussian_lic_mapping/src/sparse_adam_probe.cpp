// SPDX-License-Identifier: GPL-3.0-or-later

#include "gaussian_lic_mapping/cuda/sparse_adam.hpp"

#include <cuda_runtime.h>
#include <torch/torch.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

namespace
{

void reference_step(
  std::vector<float> & parameter,
  const std::vector<float> & gradient,
  std::vector<float> & exp_avg,
  std::vector<float> & exp_avg_sq,
  const std::vector<bool> & visible,
  int width,
  float learning_rate,
  float beta1,
  float beta2,
  float epsilon)
{
  for (size_t gaussian = 0; gaussian < visible.size(); ++gaussian) {
    if (!visible[gaussian]) {
      continue;
    }
    for (int channel = 0; channel < width; ++channel) {
      const size_t index = gaussian * static_cast<size_t>(width) + static_cast<size_t>(channel);
      const float grad = gradient[index];
      exp_avg[index] = beta1 * exp_avg[index] + (1.0F - beta1) * grad;
      exp_avg_sq[index] = beta2 * exp_avg_sq[index] + (1.0F - beta2) * grad * grad;
      parameter[index] += -learning_rate * exp_avg[index] / (std::sqrt(exp_avg_sq[index]) + epsilon);
    }
  }
}

std::vector<float> tensor_to_vector(const torch::Tensor & tensor)
{
  auto cpu = tensor.to(torch::kCPU).contiguous();
  const float * data = cpu.data_ptr<float>();
  return std::vector<float>(data, data + cpu.numel());
}

float max_abs_diff(const std::vector<float> & lhs, const std::vector<float> & rhs)
{
  float max_error = 0.0F;
  for (size_t i = 0; i < lhs.size(); ++i) {
    max_error = std::max(max_error, std::abs(lhs[i] - rhs[i]));
  }
  return max_error;
}

}  // namespace

int main()
{
  if (!torch::cuda::is_available()) {
    std::cerr << "CUDA is not available\n";
    return 2;
  }

  constexpr int gaussian_count = 100000;
  constexpr int width = 4;
  constexpr float learning_rate = 0.005F;
  constexpr float beta1 = 0.9F;
  constexpr float beta2 = 0.999F;
  constexpr float epsilon = 1.0e-8F;

  torch::manual_seed(20260504);
  auto options = torch::TensorOptions().dtype(torch::kFloat32).device(torch::kCUDA);
  auto parameter = torch::randn({gaussian_count, width}, options).contiguous();
  auto gradient = torch::randn({gaussian_count, width}, options).contiguous();
  auto exp_avg = torch::zeros_like(parameter).contiguous();
  auto exp_avg_sq = torch::zeros_like(parameter).contiguous();
  auto visible_mask = torch::zeros({gaussian_count}, torch::TensorOptions().dtype(torch::kBool).device(torch::kCUDA));
  auto visible_cpu = std::vector<bool>(gaussian_count, false);
  {
    auto visible_host = torch::zeros({gaussian_count}, torch::TensorOptions().dtype(torch::kBool));
    auto accessor = visible_host.accessor<bool, 1>();
    for (int i = 0; i < gaussian_count; ++i) {
      const bool visible = (i % 7) == 0 || (i % 23) == 0;
      accessor[i] = visible;
      visible_cpu[static_cast<size_t>(i)] = visible;
    }
    visible_mask.copy_(visible_host);
  }

  auto ref_parameter = tensor_to_vector(parameter);
  const auto ref_gradient = tensor_to_vector(gradient);
  auto ref_exp_avg = tensor_to_vector(exp_avg);
  auto ref_exp_avg_sq = tensor_to_vector(exp_avg_sq);

  gaussian_lic_mapping::cuda_ops::sparse_adam_step(
    parameter, gradient, exp_avg, exp_avg_sq, visible_mask, learning_rate, beta1, beta2, epsilon);
  cudaDeviceSynchronize();
  reference_step(
    ref_parameter, ref_gradient, ref_exp_avg, ref_exp_avg_sq, visible_cpu, width, learning_rate, beta1, beta2, epsilon);

  const auto parameter_error = max_abs_diff(tensor_to_vector(parameter), ref_parameter);
  const auto exp_avg_error = max_abs_diff(tensor_to_vector(exp_avg), ref_exp_avg);
  const auto exp_avg_sq_error = max_abs_diff(tensor_to_vector(exp_avg_sq), ref_exp_avg_sq);

  cudaEvent_t start = nullptr;
  cudaEvent_t stop = nullptr;
  cudaEventCreate(&start);
  cudaEventCreate(&stop);
  cudaEventRecord(start);
  gaussian_lic_mapping::cuda_ops::sparse_adam_step(
    parameter, gradient, exp_avg, exp_avg_sq, visible_mask, learning_rate, beta1, beta2, epsilon);
  cudaEventRecord(stop);
  cudaEventSynchronize(stop);
  float elapsed_ms = 0.0F;
  cudaEventElapsedTime(&elapsed_ms, start, stop);
  cudaEventDestroy(start);
  cudaEventDestroy(stop);

  std::cout << "sparse_adam_probe gaussians=" << gaussian_count
            << " width=" << width
            << " parameter_error=" << parameter_error
            << " exp_avg_error=" << exp_avg_error
            << " exp_avg_sq_error=" << exp_avg_sq_error
            << " elapsed_ms=" << elapsed_ms << "\n";

  if (
    !std::isfinite(parameter_error) || parameter_error > 1.0e-6F ||
    !std::isfinite(exp_avg_error) || exp_avg_error > 1.0e-7F ||
    !std::isfinite(exp_avg_sq_error) || exp_avg_sq_error > 1.0e-8F)
  {
    std::cerr << "sparse Adam update mismatch exceeds tolerance\n";
    return 1;
  }
  if (!std::isfinite(elapsed_ms) || elapsed_ms > 5.0F) {
    std::cerr << "sparse Adam update exceeded 5 ms target\n";
    return 1;
  }

  std::cout << "sparse_adam_probe OK\n";
  return 0;
}
