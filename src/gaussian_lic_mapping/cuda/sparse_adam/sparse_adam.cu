// SPDX-License-Identifier: GPL-3.0-or-later

#include "gaussian_lic_mapping/cuda/sparse_adam.hpp"

#include <c10/cuda/CUDAException.h>
#include <cooperative_groups.h>

#include <cstdint>

namespace cg = cooperative_groups;

namespace gaussian_lic_mapping::cuda_ops
{
namespace
{

__global__ void sparse_adam_step_kernel(
  float * parameter,
  const float * gradient,
  float * exp_avg,
  float * exp_avg_sq,
  const bool * visible_mask,
  float learning_rate,
  float beta1,
  float beta2,
  float epsilon,
  uint32_t gaussian_count,
  uint32_t parameter_width)
{
  const auto parameter_index = cg::this_grid().thread_rank();
  const uint32_t gaussian_index = parameter_index / parameter_width;
  if (gaussian_index >= gaussian_count || !visible_mask[gaussian_index]) {
    return;
  }

  const float grad = gradient[parameter_index];
  float avg = exp_avg[parameter_index];
  float avg_sq = exp_avg_sq[parameter_index];
  avg = beta1 * avg + (1.0F - beta1) * grad;
  avg_sq = beta2 * avg_sq + (1.0F - beta2) * grad * grad;
  parameter[parameter_index] += -learning_rate * avg / (sqrtf(avg_sq) + epsilon);
  exp_avg[parameter_index] = avg;
  exp_avg_sq[parameter_index] = avg_sq;
}

void check_tensor(const torch::Tensor & tensor, const char * name)
{
  TORCH_CHECK(tensor.is_cuda(), name, " must be a CUDA tensor");
  TORCH_CHECK(tensor.dtype() == torch::kFloat32, name, " must be float32");
  TORCH_CHECK(tensor.is_contiguous(), name, " must be contiguous");
}

}  // namespace

void sparse_adam_step(
  torch::Tensor & parameter,
  const torch::Tensor & gradient,
  torch::Tensor & exp_avg,
  torch::Tensor & exp_avg_sq,
  const torch::Tensor & visible_mask,
  float learning_rate,
  float beta1,
  float beta2,
  float epsilon)
{
  check_tensor(parameter, "parameter");
  check_tensor(gradient, "gradient");
  check_tensor(exp_avg, "exp_avg");
  check_tensor(exp_avg_sq, "exp_avg_sq");
  TORCH_CHECK(visible_mask.is_cuda(), "visible_mask must be a CUDA tensor");
  TORCH_CHECK(visible_mask.dtype() == torch::kBool, "visible_mask must be bool");
  TORCH_CHECK(visible_mask.is_contiguous(), "visible_mask must be contiguous");
  TORCH_CHECK(parameter.sizes() == gradient.sizes(), "parameter and gradient shapes must match");
  TORCH_CHECK(parameter.sizes() == exp_avg.sizes(), "parameter and exp_avg shapes must match");
  TORCH_CHECK(parameter.sizes() == exp_avg_sq.sizes(), "parameter and exp_avg_sq shapes must match");
  TORCH_CHECK(parameter.dim() >= 2, "parameter must have a leading Gaussian dimension and at least one value dimension");

  const auto gaussian_count = static_cast<uint32_t>(parameter.size(0));
  TORCH_CHECK(visible_mask.numel() == gaussian_count, "visible_mask length must match parameter.size(0)");
  const auto parameter_width = static_cast<uint32_t>(parameter.numel() / gaussian_count);
  TORCH_CHECK(parameter_width > 0, "parameter width must be non-zero");

  const auto total = static_cast<uint32_t>(parameter.numel());
  sparse_adam_step_kernel<<<(total + 255U) / 256U, 256U>>>(
    parameter.data_ptr<float>(),
    gradient.data_ptr<float>(),
    exp_avg.data_ptr<float>(),
    exp_avg_sq.data_ptr<float>(),
    visible_mask.data_ptr<bool>(),
    learning_rate,
    beta1,
    beta2,
    epsilon,
    gaussian_count,
    parameter_width);
  C10_CUDA_KERNEL_LAUNCH_CHECK();
}

}  // namespace gaussian_lic_mapping::cuda_ops
