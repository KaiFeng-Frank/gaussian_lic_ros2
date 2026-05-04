# Upstream And Toolchain Lock

Generated for the paper-level Gaussian-LIC/Gaussian-LIC2 ROS2 porting track.

## Upstream Revisions

```text
Gaussian-LIC/Gaussian-LIC2: APRIL-ZJU/Gaussian-LIC cd4c122dfad7e93255fe6862ac2c2b205e844786
Coco-LIC reference:        APRIL-ZJU/Coco-LIC    4ead7e423f67b3e15a86de4ee551bb7c752f2cd8
```

`APRIL-ZJU/Gaussian-LIC` currently exposes only the `master` branch. No separate
Gaussian-LIC2 tag or branch is available from `git ls-remote` at the time of
this lock. The upstream README announces the Gaussian-LIC2 release and the
current HEAD includes LIC2-labeled depth completion plus the updated Gaussian
mapping/rasterizer surface.

## Local Toolchain

```text
OS: Ubuntu 24.04
ROS2: Jazzy
GPU: NVIDIA GeForce RTX 5070 Ti Laptop GPU
Compute capability: 12.0
Driver: 580.126.09
CUDA toolkit: /usr/local/cuda-12.8
CUDA compiler: /usr/local/cuda-12.8/bin/nvcc
libtorch: /home/frank/Software/libtorch
TensorRT: /home/frank/Software/TensorRT-8.6.1.6
```

The global `PATH` may not contain `nvcc`; strict CUDA build scripts must use
`CUDA_HOME=/usr/local/cuda-12.8` or pass `CMAKE_CUDA_COMPILER` explicitly.

The local `libtorch_cuda.so` contains native `sm_120`/`compute_120` code, so a
PyTorch source build is not the default path. Rebuild PyTorch or switch to an
NGC image only if `scripts/sm120_compat_probe.sh` fails.

## Build Profiles

Default ROS2 build keeps CUDA optional:

```bash
./scripts/build_ros2.sh
```

Strict CUDA/Torch plumbing build:

```bash
./scripts/build_cuda_strict.sh --packages-select gaussian_lic_mapping
```

The strict wrapper clears the package CMake cache by default because switching
between non-CUDA and CUDA compilers changes cache-sensitive CMake variables.

The strict profile sets:

```text
GAUSSIAN_LIC_ENABLE_TORCH=ON
GAUSSIAN_LIC_ENABLE_CUDA=ON
CMAKE_CUDA_ARCHITECTURES=86;89;100;120
CUDA_HOME=/usr/local/cuda-12.8
Torch_DIR=/home/frank/Software/libtorch/share/cmake/Torch
```

## P0 Acceptance

P0 is accepted only when all commands pass:

```bash
./scripts/sm120_compat_probe.sh
./scripts/build_cuda_strict.sh --packages-select gaussian_lic_mapping
```

Passing P0 does not imply the CUDA rasterizer, optimizer, depth completion, or
native tracker are ported. It only proves the machine and build system can
compile and run native `sm_120` CUDA code linked with libtorch.
