#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
CUDA_HOME="${CUDA_HOME:-/usr/local/cuda-12.8}"

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u
cd "${ROOT_DIR}"

cmake_args=(
  --no-warn-unused-cli
  -DPython3_EXECUTABLE=/usr/bin/python3
  -DPYTHON_EXECUTABLE=/usr/bin/python3
  -DGAUSSIAN_LIC_ENABLE_TORCH="${GAUSSIAN_LIC_ENABLE_TORCH:-OFF}"
  -DGAUSSIAN_LIC_ENABLE_CUDA="${GAUSSIAN_LIC_ENABLE_CUDA:-OFF}"
  -DGAUSSIAN_LIC_ENABLE_TENSORRT="${GAUSSIAN_LIC_ENABLE_TENSORRT:-OFF}"
)

if [[ "${GAUSSIAN_LIC_ENABLE_TORCH:-OFF}" == "ON" || "${GAUSSIAN_LIC_ENABLE_CUDA:-OFF}" == "ON" || "${GAUSSIAN_LIC_ENABLE_TENSORRT:-OFF}" == "ON" ]]; then
  export PATH="${CUDA_HOME}/bin:/usr/local/cuda/bin:${PATH}"
  cmake_args+=(
    -DCUDAToolkit_ROOT="${CUDA_HOME}"
  )
fi

if [[ "${GAUSSIAN_LIC_ENABLE_TENSORRT:-OFF}" == "ON" ]]; then
  cmake_args+=(
    -DTENSORRT_ROOT="${TENSORRT_ROOT:-${HOME}/Software/TensorRT-8.6.1.6}"
  )
fi

if [[ "${GAUSSIAN_LIC_ENABLE_TORCH:-OFF}" == "ON" || "${GAUSSIAN_LIC_ENABLE_CUDA:-OFF}" == "ON" ]]; then
  torch_dir="${TORCH_DIR:-${HOME}/Software/libtorch/share/cmake/Torch}"
  cuda_arch="${CMAKE_CUDA_ARCHITECTURES:-}"
  if [[ -z "${cuda_arch}" ]] && command -v nvidia-smi >/dev/null 2>&1; then
    cuda_arch="$(nvidia-smi --query-gpu=compute_cap --format=csv,noheader 2>/dev/null | head -1 | tr -d '.')"
  fi
  cuda_arch="${cuda_arch:-native}"
  cuda_compiler="${CMAKE_CUDA_COMPILER:-${CUDA_HOME}/bin/nvcc}"

  cmake_args+=(
    -DGAUSSIAN_LIC_ENABLE_TORCH=ON
    -DGAUSSIAN_LIC_ENABLE_CUDA="${GAUSSIAN_LIC_ENABLE_CUDA:-OFF}"
    -DTorch_DIR="${torch_dir}"
    -DCMAKE_CUDA_COMPILER="${cuda_compiler}"
    -DCMAKE_CUDA_ARCHITECTURES="${cuda_arch}"
  )
fi

colcon build --symlink-install \
  --cmake-args "${cmake_args[@]}" \
  "$@"
