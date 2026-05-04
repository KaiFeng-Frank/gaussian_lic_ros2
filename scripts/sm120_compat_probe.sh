#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
set -euo pipefail

CUDA_HOME="${CUDA_HOME:-/usr/local/cuda-12.8}"
TORCH_DIR="${TORCH_DIR:-${HOME}/Software/libtorch/share/cmake/Torch}"
BUILD_DIR="${BUILD_DIR:-/tmp/gaussian_lic_sm120_probe}"

if [[ ! -x "${CUDA_HOME}/bin/nvcc" ]]; then
  echo "Missing nvcc at ${CUDA_HOME}/bin/nvcc" >&2
  exit 2
fi
if [[ ! -f "${TORCH_DIR}/TorchConfig.cmake" ]]; then
  echo "Missing TorchConfig.cmake under ${TORCH_DIR}" >&2
  exit 2
fi
if ! command -v nvidia-smi >/dev/null 2>&1; then
  echo "nvidia-smi is required for the sm_120 runtime probe" >&2
  exit 2
fi

rm -rf "${BUILD_DIR}"
mkdir -p "${BUILD_DIR}/src"

cat >"${BUILD_DIR}/src/kernel.cu" <<'CUDA'
#include <cuda_runtime.h>

__global__ void add_kernel(const float * lhs, const float * rhs, float * out, int n)
{
  const int index = blockIdx.x * blockDim.x + threadIdx.x;
  if (index < n) {
    out[index] = lhs[index] + rhs[index];
  }
}

extern "C" void launch_add_kernel(const float * lhs, const float * rhs, float * out, int n)
{
  const int threads = 256;
  const int blocks = (n + threads - 1) / threads;
  add_kernel<<<blocks, threads>>>(lhs, rhs, out, n);
}
CUDA

cat >"${BUILD_DIR}/src/main.cpp" <<'CPP'
#include <cuda_runtime.h>
#include <torch/torch.h>

#include <cmath>
#include <iostream>

extern "C" void launch_add_kernel(const float * lhs, const float * rhs, float * out, int n);

int main()
{
  if (!torch::cuda::is_available()) {
    std::cerr << "torch CUDA is not available\n";
    return 2;
  }
  int device = -1;
  cudaGetDevice(&device);
  cudaDeviceProp properties{};
  cudaGetDeviceProperties(&properties, device);
  std::cout << "device=" << properties.name << " cc="
            << properties.major << "." << properties.minor << "\n";
  if (properties.major != 12 || properties.minor != 0) {
    std::cerr << "expected compute capability 12.0 for sm_120 probe\n";
    return 3;
  }

  constexpr int count = 1024;
  auto options = torch::TensorOptions().dtype(torch::kFloat32).device(torch::kCUDA);
  auto lhs = torch::arange(count, options);
  auto rhs = torch::ones({count}, options) * 2.0F;
  auto out = torch::empty({count}, options);
  launch_add_kernel(lhs.data_ptr<float>(), rhs.data_ptr<float>(), out.data_ptr<float>(), count);
  cudaDeviceSynchronize();

  auto expected = lhs + rhs;
  const float max_error = torch::max(torch::abs(out - expected)).item<float>();
  std::cout << "max_error=" << max_error << "\n";
  if (!std::isfinite(max_error) || max_error > 0.0F) {
    return 4;
  }
  std::cout << "sm_120 compat probe OK\n";
  return 0;
}
CPP

cat >"${BUILD_DIR}/CMakeLists.txt" <<'CMAKE'
cmake_minimum_required(VERSION 3.22)
project(sm120_compat_probe LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_ARCHITECTURES 120)

find_package(CUDAToolkit REQUIRED)
find_package(Torch REQUIRED)

add_executable(sm120_compat_probe src/main.cpp src/kernel.cu)
target_compile_options(sm120_compat_probe PRIVATE ${TORCH_CXX_FLAGS})
target_link_libraries(sm120_compat_probe PRIVATE ${TORCH_LIBRARIES} CUDA::cudart)
set_target_properties(sm120_compat_probe PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
CMAKE

cmake -S "${BUILD_DIR}" -B "${BUILD_DIR}/build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CUDA_COMPILER="${CUDA_HOME}/bin/nvcc" \
  -DCUDAToolkit_ROOT="${CUDA_HOME}" \
  -DTorch_DIR="${TORCH_DIR}"
cmake --build "${BUILD_DIR}/build" -j"$(nproc)"
"${BUILD_DIR}/build/sm120_compat_probe"
