#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export CUDA_HOME="${CUDA_HOME:-/usr/local/cuda-12.8}"
export TORCH_DIR="${TORCH_DIR:-${HOME}/Software/libtorch/share/cmake/Torch}"
export GAUSSIAN_LIC_ENABLE_TORCH=ON
export GAUSSIAN_LIC_ENABLE_CUDA=ON
export CMAKE_CUDA_ARCHITECTURES="${CMAKE_CUDA_ARCHITECTURES:-86;89;100;120}"

args=("$@")
has_clean_cache=false
for arg in "${args[@]}"; do
  if [[ "${arg}" == "--cmake-clean-cache" ]]; then
    has_clean_cache=true
    break
  fi
done
if [[ "${has_clean_cache}" != "true" ]]; then
  args+=(--cmake-clean-cache)
fi

exec "${ROOT_DIR}/scripts/build_ros2.sh" "${args[@]}"
