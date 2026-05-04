#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u
cd "${ROOT_DIR}"

cmake_args=(
  --no-warn-unused-cli
  -DPython3_EXECUTABLE=/usr/bin/python3
  -DPYTHON_EXECUTABLE=/usr/bin/python3
  -DGAUSSIAN_LIC_ENABLE_TORCH="${GAUSSIAN_LIC_ENABLE_TORCH:-OFF}"
)

if [[ "${GAUSSIAN_LIC_ENABLE_TORCH:-OFF}" == "ON" ]]; then
  export PATH="/usr/local/cuda/bin:${PATH}"
  torch_dir="${TORCH_DIR:-${HOME}/Software/libtorch/share/cmake/Torch}"
  cuda_arch="${CMAKE_CUDA_ARCHITECTURES:-}"
  if [[ -z "${cuda_arch}" ]] && command -v nvidia-smi >/dev/null 2>&1; then
    cuda_arch="$(nvidia-smi --query-gpu=compute_cap --format=csv,noheader 2>/dev/null | head -1 | tr -d '.')"
  fi
  cuda_arch="${cuda_arch:-native}"

  cmake_args+=(
    -DGAUSSIAN_LIC_ENABLE_TORCH=ON
    -DTorch_DIR="${torch_dir}"
    -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc
    -DCMAKE_CUDA_ARCHITECTURES="${cuda_arch}"
  )
fi

colcon build --symlink-install \
  --cmake-args "${cmake_args[@]}" \
  "$@"
