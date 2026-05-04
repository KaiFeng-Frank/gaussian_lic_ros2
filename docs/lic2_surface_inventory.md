# Gaussian-LIC2 Surface Inventory

This inventory locks the public upstream surface used for the ROS2 Jazzy
paper-level migration. It is based on `external/Gaussian-LIC` at
`cd4c122dfad7e93255fe6862ac2c2b205e844786`.

## Repository State

`APRIL-ZJU/Gaussian-LIC` currently has no separate LIC2 branch or tag. The public
`master` README announces Gaussian-LIC2 and the checked tree contains the
Gaussian mapping, rasterizer, sparse optimizer, fused SSIM, simple-knn, and
TensorRT depth-completion sources needed by the mapping backend. The run path
still uses Coco-LIC for odometry/mapper input topics.

## Mapping Backend Files

```text
src/mapping.cpp
src/mapping.h
src/gaussian.cpp
src/gaussian.h
src/camera.h
src/tensor_utils.h
src/loss_utils.h
src/optim_utils.h
src/depth_completer.cpp
src/depth_completer.h
```

Important behavior to preserve in ROS2:

- `mapping.cpp` aligns `/points_for_gs`, `/pose_for_gs`, `/image_for_gs`, and
  `/depth_for_gs` at 10 ms tolerance.
- `Dataset::addFrame()` colorizes and accumulates keyframe points, optionally
  calls `DepthCompleter`, and stores `Camera` training/test views.
- `GaussianModel::initialize()` builds tensors for `xyz`, `features_dc`,
  `features_rest`, `scaling`, `rotation`, `opacity`, and optional exposure.
- `trainingSetup()` creates SparseGaussianAdam parameter groups for all six
  Gaussian parameter tensors.
- `optimize()` calls `render()`, uses RGB L1, fused SSIM, optional depth L1, and
  SparseGaussianAdam with the visible Gaussian mask.
- `extend()` appends new frame points through `densificationPostfix()` after
  alpha/depth/image-bound filtering.
- `saveMap()` writes upstream-compatible Gaussian PLY properties.

## CUDA Source Files

```text
src/simple-knn/spatial.cu
src/simple-knn/simple_knn.cu
src/fused-ssim/ssim.cu
src/rasterizer/rasterize_points.cu
src/rasterizer/cuda_rasterizer/adam.cu
src/rasterizer/cuda_rasterizer/forward.cu
src/rasterizer/cuda_rasterizer/backward.cu
src/rasterizer/cuda_rasterizer/rasterizer_impl.cu
```

The migrated CUDA modules are:

```text
gaussian_lic_mapping/cuda/simple_knn.hpp  dist_cuda2(points[N,3])
gaussian_lic_mapping/cuda/sparse_adam.hpp sparse_adam_step(...)
gaussian_lic_mapping/cuda/fused_ssim.hpp  fused_ssim(...)
gaussian_lic_mapping/cuda/rasterizer/*    Gaussian rasterizer forward/backward
```

These now build as native ROS2/ament CMake CUDA targets in the strict CUDA
profile.

## Densification Scope

The public upstream does not expose classic 3DGS functions named
`densify_and_clone` or `densify_and_split`. The upstream growth mechanism visible
in `cd4c122` is:

```text
extend(dataset, gaussians)
GaussianModel::densificationPostfix(...)
```

The ROS2 port now has both upstream-style foreground append and a stricter
gradient-aware clone/split density-control path for the CUDA backend. The latter
is a parity-risk item to audit during strict reproduction because it is not
present under those names in the currently released public tree.

## LIC2-Specific 2D Primitive Check

The public `cd4c122` tree does not contain a distinct 2D Gaussian primitive,
surface-normal tensor, `normal_consistency_loss`, or mapper-side scale axis
collapse. Gaussian tensors remain `xyz[N,3]`, `scaling[N,3]`, `rotation[N,4]`,
`features_dc[N,1,3]`, `features_rest[N,15,3]`, and `opacity[N,1]`.

The skybox implementation in `GaussianModel::initialize()` remains the sampled
sphere path using `skybox_points_num`, `skybox_radius`, RGB DC seeds
`0.7/0.8/0.95`, and `distCUDA2`-derived scales. The ROS2 mapper mirrors that
surface; there is no released LIC2 skybox patch beyond this to port.

## Frontend/Tracking Surface

`external/Gaussian-LIC` does not contain a standalone native LIC2 frontend or
tracking executable. It expects Coco-LIC to publish the mapper contract topics.

Coco-LIC reference files for the eventual native ROS2 tracker:

```text
external/Coco-LIC/src/spline/*
external/Coco-LIC/src/odom/*
external/Coco-LIC/src/lidar/*
external/Coco-LIC/src/camera/r3live*
external/Coco-LIC/src/camera/loam/*
external/Coco-LIC/src/imu/*
```

Until that package is ported, `gaussian_lic_frontend/lic2_contract_adapter`
remains an adapter for externally supplied pose/odometry and raw sensor topics,
not the paper tracker.

## Strict Gate Status

`baseline_readiness.py --strict` now delegates to the combined reproduction
report instead of only checking that files exist. The strict report requires
novel-view PSNR, SSIM, and LPIPS metrics in `metrics.json`, applies the paper
gate as a default 5% regression limit, and also requires matched baseline/current
render images to pass PSNR/SSIM similarity thresholds.

If PSNR, SSIM, or LPIPS are missing or null, strict readiness fails explicitly.
The remaining blocker is producing the `CBD_Building_01` ROS1 baseline and ROS2
current artifacts from the real official bag with the native tracker path.
