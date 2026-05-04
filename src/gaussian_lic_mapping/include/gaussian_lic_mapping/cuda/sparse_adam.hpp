// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <torch/torch.h>

namespace gaussian_lic_mapping::cuda_ops
{

void sparse_adam_step(
  torch::Tensor & parameter,
  const torch::Tensor & gradient,
  torch::Tensor & exp_avg,
  torch::Tensor & exp_avg_sq,
  const torch::Tensor & visible_mask,
  float learning_rate,
  float beta1,
  float beta2,
  float epsilon);

}  // namespace gaussian_lic_mapping::cuda_ops
