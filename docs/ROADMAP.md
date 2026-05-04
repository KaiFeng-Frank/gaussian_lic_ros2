# Roadmap

The public release plan is milestone-based:

- `v0.1.0` Baseline & Infrastructure
- `v0.2.0` Gaussian-LIC2 ROS2 Native Frontend
- `v0.3.0` Gaussian Mapping Full Backend
- `v0.4.0` Strict FAST-LIVO2 Reproduction

See `docs/RELEASE_MILESTONES.md` for release criteria and artifacts.

## Current Truth

The current repository is a strong ROS2 plumbing and reproducibility checkpoint, not a complete paper-level Gaussian-LIC/Gaussian-LIC2 algorithm port.

Completed proof/evidence scope:

- ROS2 workspace, messages, launch, composable mapper node, dataset profiles, bag contracts, artifact extraction, and CI artifact gates.
- Native mapper input contract and `lic2_contract_adapter` boundary for raw camera/LiDAR/IMU plus pose/odometry.
- Optional Torch Gaussian tensor path with keyframe initialization, skybox seeding, foreground append, gradient-aware densification, multi-criteria pruning, opacity reset scheduling, CUDA rasterizer preview, CUDA rasterizer/fused-SSIM/depth loss, visibility-masked SparseGaussianAdam updates for the full Gaussian parameter set, and optional TensorRT/SPNet depth completion.
- Official FAST-LIVO2 `Bright_Screen_Wall` substitute report with `metrics`, `trajectory`, `point_cloud`, and `gaussian_color` gates passing.

Incomplete paper-algorithm scope:

- `lic2_contract_adapter` is not the native continuous-time Gaussian-LIC/Coco-LIC tracker.
- TensorRT/SPNet depth completion requires a generated SPNet TensorRT engine path at runtime; no `.engine` artifact is checked into this repository.
- The public LIC2 upstream surface does not currently expose separate 2D Gaussian primitive or skybox patches beyond the checked mapper backend.
- Strict `CBD_Building_01` paper reproduction is blocked until the official bag is available locally and the native algorithm path is complete.

## Immediate Next Step

Strict baseline data gate:

```text
./scripts/baseline_readiness.py --dataset-root /home/frank/data/fast_livo --sequence CBD_Building_01
```

First concrete action:

```bash
./scripts/fetch_fastlivo2_sequence.py \
  --sequence CBD_Building_01 \
  --output-dir /home/frank/data/fast_livo
```

Current execution gate:

```bash
./scripts/upstream_baseline_build_attempt.sh
./scripts/baseline_readiness.py \
  --dataset-root /home/frank/data/fast_livo \
  --baseline-dir baseline/fastlivo2/CBD_Building_01 \
  --current-results-dir results/fastlivo2/current \
  --sequence CBD_Building_01 \
  --strict
```

As of 2026-05-04, `CBD_Building_01` is being fetched from the official Google
Drive file ID discovered by `scripts/fetch_fastlivo2_sequence.py`. The upstream
ROS1 `gs_mapping` target builds successfully in Docker with the local OpenCV
4.10 CUDA fallback and TensorRT 8.6.1.6. Strict OpenCV 4.7.0 is still incomplete
locally.

To avoid waiting on the quota-blocked strict sequence, the current executable substitute is the official FAST-LIVO2 `Bright_Screen_Wall` bag, curated to the first 8 seconds. The latest colorized combined proof chain has:

```text
baseline/fastlivo2/Bright_Screen_Wall_fastlivo2_color_8s/
results/fastlivo2/Bright_Screen_Wall_current_extrinsic_color_fullseq/reproduction_report.json
```

That Bright substitute passing report is not a replacement for the strict paper gate.

Required baseline artifacts:

```text
trajectory.tum
point_cloud.ply
renders/
metrics.json
run.log
baseline_manifest.json
```

## Milestone 0: ROS2 Foundation

- [x] Create a clean ROS2 workspace layout.
- [x] Add public Gaussian/status messages.
- [x] Add a launch entrypoint.
- [x] Add topic probe and status stub nodes.
- [x] Add native mapping-node scaffold.
- [x] Add upstream fetch helper.
- [x] Build successfully on local ROS2 Jazzy.

## Milestone 1: Input Contract

- [x] Add image, camera info, point cloud, IMU, and TF topic surfaces for local smoke tests.
- [x] Add rosbag2 smoke-test instructions.
- [x] Add calibration file schema.
- [x] Add dataset-specific ROS2 mapping profiles for FAST-LIVO, FAST-LIVO2, M2DGR, MCD, and R3LIVE.
- [x] Add a native ROS2 frontend contract adapter from raw sensor/pose topics to mapper topics.
- [x] Add a `frontend_raw` bag contract for raw camera/LiDAR/IMU/pose validation before adapter replay.
- [x] Add a `frontend_sensor_raw` contract and FAST-LIVO2 ROS1-to-ROS2 converter for real camera/LiDAR/IMU bags without pose.
- [x] Add IMU gyro orientation fallback for raw frontend bags without odometry.
- [x] Add FAST-LIVO2 camera-LiDAR static transform profile for raw pointcloud projection.
- [ ] Add full real-dataset topic remaps/adapters once the native Gaussian-LIC2 frontend/tracking port is in place.

## Milestone 2: Gaussian-LIC Mapping Port

- [x] Inventory ROS1 APIs used by upstream Gaussian-LIC.
- [x] Replace the four-topic ROS1 input synchronization surface with native ROS2.
- [x] Convert aligned ROS2 frames into mapper-ready image/depth/pose/point data.
- [x] Consume CameraInfo intrinsics with parameter fallback.
- [x] Accumulate converted frames into a non-torch `MapperDataset`.
- [x] Publish ROS2 odometry, path, and accumulated map-point topics from the native mapper.
- [x] Add optional `map -> camera` TF broadcasting.
- [x] Port upstream Gaussian-LIC backend parameter names into ROS2 profiles.
- [ ] Replace remaining ROS1 node handles, publishers, and launch files.
- [ ] Isolate CUDA/Gaussian map code from ROS middleware code.
- [x] Add optional libtorch/CUDA build path.
- [x] Add live `MapperDataset -> TorchCamera` conversion behind a build flag.
- [x] Add foreground `MapperDataset -> TorchGaussianMap` initialization behind a build flag.
- [x] Add keyframe-gated skybox seeding and incremental foreground Gaussian insertion.
- [x] Publish initialized and extended Gaussian map as chunked `GaussianArray`.
- [x] Add save-map service for initialized Gaussian PLY export.
- [x] Add save-map fallback for accumulated debug XYZRGB PLY export.
- [x] Publish CPU projected-map preview on `/gaussian_lic/rendered_image`.
- [x] Publish a Torch GaussianMap splat preview for `render_mode:=rasterizer`.
- [x] Add optional keyframe photometric Torch optimization for Gaussian DC color/opacity tensors.
- [x] Add optional Torch Gaussian pruning by opacity and foreground count cap.
- [x] Register `mapping_node` as an `rclcpp_components` composable node.
- [x] Add configurable input sensor QoS for ROS2 driver/rosbag2 compatibility.
- [x] Fill `MappingStatus` tracking/mapping rate fields from live frame counters.
- [x] Make TensorRT depth completion optional at the ROS2 mapper input boundary.
- [x] Port the native TensorRT/SPNet depth completion wrapper and mapper lazy-loading path.
- [x] Port upstream CUDA simple-knn `distCUDA2` with a strict probe.
- [x] Port upstream fused-ssim CUDA forward/backward with scalar and gradient probes.
- [x] Port upstream CUDA Gaussian rasterizer forward/backward operator with a strict smoke probe.
- [x] Port upstream visibility-masked SparseGaussianAdam CUDA update with a strict probe.
- [x] Wire the CUDA rasterizer into the ROS2 Torch backend loss path with L1 + fused-SSIM + optional depth supervision.
- [x] Replace the CUDA Torch backend manual SGD path with visibility-masked SparseGaussianAdam updates for xyz/features/scaling/rotation/opacity.
- [x] Port gradient-aware densification split/clone scheduling.
- [x] Extend pruning with opacity, foreground cap, screen radius, world-scale, and long-invisible criteria.
- [x] Add opacity reset scheduling for long-running Gaussian optimization.
- [x] Publish mapper status.
- [x] Replace CPU Gaussian splat preview with upstream CUDA Gaussian rasterizer output in the strict CUDA path.

## Milestone 3: Gaussian-LIC2 Frontend/Tracking Port

- [x] Inventory the released Gaussian-LIC2 surface in `external/Gaussian-LIC`, including depth completion, rasterizer/optimizer changes, and any frontend/tracking code available in upstream commits.
- [x] Inventory the Coco-LIC continuous-time frontend source surface for the native ROS2 tracker port.
- [x] Add `gaussian_lic_frontend/lic2_contract_adapter` as the ROS2 boundary for raw camera/LiDAR/IMU/pose inputs.
- [x] Port Livox/custom point handling to ROS2.
- [x] Add a native timestamp-safe cubic B-spline trajectory manager foundation with a constant-velocity probe.
- [x] Add a native signed-nanosecond IMU propagation foundation with a deterministic gyro/accel probe.
- [x] Add a native ROS2 tracking node surface that publishes mapper input topics plus odometry, path, and optional TF.
- [x] Add a native LiDAR nearest-neighbor residual factor with bounded local map, PointCloud2 parsing, and deterministic correction probe.
- [x] Add a native visual photometric residual factor and subscribe tracking to mapper rendered-image output.
- [x] Subscribe native tracking to mapper `GaussianArray` snapshot chunks for the Gaussian-map reverse channel.
- [ ] Replace dynamic reconfigure/global parameters with ROS2 parameters.
- [x] Publish odometry, path, TF, and Gaussian mapper input topics natively.
- [ ] Keep any ROS1 bridge mode clearly marked as temporary.

## Milestone 4: Usability

- [x] Docker image with NVIDIA runtime support for the non-torch ROS2 smoke-test path.
- [x] Add RViz2 launch option and default visualization config.
- [x] One-command synthetic smoke-test script.
- [x] One-command synthetic rosbag2 generation and replay smoke test.
- [ ] Screenshots and short demo video.
- [x] Common errors page.
- [x] Status schema page.
- [x] Offline rosbag2 artifact extraction CLI skeleton.
- [x] Performance regression comparison script.
- [x] Jazzy/Humble build-only CI skeleton.
- [x] Python-only CI artifact gates for baseline manifests and reproduction reports.
- [x] FAST-LIVO2 data fetch/readiness scripts for executable baseline status.
- [x] Reproducible ROS1 upstream baseline build-attempt script for the local Noetic Docker environment.
- [x] Add a ROS2 current-results collector that records mapper outputs and writes `trajectory.tum`, `point_cloud.ply`, and `metrics.json`.
- [x] Add `--strict` reproduction readiness that fails on missing PSNR/SSIM/LPIPS and enforces the 5% paper metric gate.
- [x] Jazzy CI smoke tests with a generated mini rosbag2 sequence.
- [x] ROS1 upstream baseline archive for the official curated FAST-LIVO2 `Bright_Screen_Wall_curated_8s` substitute.
- [x] ROS1 `.bag` to rosbag2 `.mcap` converter backend.
- [x] PR comparison report on real or curated mini-sequence artifacts.
- [ ] Strict `CBD_Building_01` ROS1 upstream baseline archive and report after the official data source becomes downloadable.
