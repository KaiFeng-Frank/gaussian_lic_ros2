# Gaussian-LIC ROS2

Native ROS2 engineering port for [Gaussian-LIC/Gaussian-LIC2](https://github.com/APRIL-ZJU/Gaussian-LIC).

The long-term target is a paper-level ROS2 reproduction of LiDAR-Inertial-Camera Gaussian Splatting SLAM: native ROS2 tracking, native Gaussian mapping, rosbag2 replay, offline reproducibility tooling, and strict comparison against the ROS1 upstream baseline.

This repository is **not a ROS1 bridge wrapper**. It is a clean ROS2 workspace that keeps algorithm code isolated from middleware glue so the tracking and mapping backends can be ported incrementally without locking the project into ROS1-era APIs.

## Current Release State

This repository is now an executable ROS2 porting checkpoint for the public Gaussian-LIC/Gaussian-LIC2 code path. It has native ROS2 message, launch, adapter, mapper, CUDA/Torch Gaussian backend plumbing, initial native tracking factors, artifact extraction, strict replay/readiness tooling, an official FAST-LIVO2 Bright substitute proof chain, and a passing local strict `CBD_Building_01` reproduction report against the archived ROS1 upstream baseline.

It is **not yet** the complete full-paper Gaussian-LIC2 ROS2 port: the native tracker still needs full Coco-LIC2-grade sliding-window joint optimization. The current strict pass is for the mapper-contract/CUDA current path against the locally archived ROS1 upstream baseline.

Available now:

- Native ROS2 packages for messages, launch/config, mapping scaffold, and test tools.
- `gaussian_lic_frontend/lic2_contract_adapter` for routing raw ROS2 camera/LiDAR/IMU/pose topics into the mapper contract.
- Optional Livox `CustomMsg` to `PointCloud2` bridge for raw Livox driver packets.
- ROS2 synchronization/conversion for the Gaussian-LIC mapper input contract:
  `/points_for_gs`, `/pose_for_gs`, `/image_for_gs`, `/camera_info_for_gs`, `/depth_for_gs`, `/imu_for_gs`.
- Odometry, path, TF, debug map cloud, rendered debug preview, status, GaussianArray, and SaveMap interfaces.
- Optional libtorch/CUDA path for keyframe-gated Gaussian tensor initialization, skybox seeding, incremental foreground insertion, CUDA rasterizer preview/loss, fused SSIM, SparseGaussianAdam, gradient-aware densification, multi-criteria pruning, opacity reset, and optional TensorRT/SPNet depth-completion wrapper.
- Native tracking package with signed-nanosecond trajectory and IMU primitives, invalid ROS2 builtin-stamp nanosecond rejection at mapper/frontend boundaries, non-monotonic tracking stream stamp rejection with status counters, non-finite IMU measurement rejection before propagation/preintegration, LiDAR invalid-point/invalid-point-time/out-of-range point-time status counters, CameraInfo/image/depth/rendered invalid-frame counters, mapper per-input QoS overrides, frontend-adapter per-stream input/output QoS controls, and native tracking per-stream raw-input/mapper-output QoS controls for image/point cloud/pose/CameraInfo/depth/IMU/odometry streams, finite CameraInfo intrinsic gating before mapper/tracker state updates, finite LiDAR-to-IMU/camera-to-IMU extrinsic, IMU gravity, LiDAR keyframe-threshold, tracking QoS depth, visual/SE3 photometric gate, LiDAR-factor, cache-size, and sliding-window BA-bound validation before tracker startup, signed-nanosecond image/LiDAR/IMU status gates, bounded best-effort tracking sensor QoS exposed through launch/status gates, executor callback serialization for ROS1-style estimator mutation order, ROS2-configurable LiDAR-to-IMU and camera-to-IMU extrinsics, signed-nanosecond visual-factor freshness gating with bounded nearest-stamp rendered/depth frame caches, IMU history interpolation, raw-sample IMU preintegration/reintegration with finite-input validation and per-block rotation/velocity/position residual weights, optimized pose/velocity/bias feedback into odometry, the continuous-time trajectory-control cache, and safe IMU re-anchoring, IMU bias continuity and marginalization-prior anchoring inside the optional sliding window, bias observability status/probes, reusable normal-equation linearization, factor-agnostic BA solve triggering when non-IMU LiDAR/visual factors are available, bounded LM state increments, Schur-complement retained-window dense priors with rank/singular-value health, timestamp-safe cubic B-spline position/velocity plus SO(3) cubic orientation trajectory queries for deskew fallback, default-enabled three-state continuous-time trajectory smoothness factors with analytic linear Jacobian blocks, per-point LiDAR deskew with invalid-pose and scan-time-bound rejection, bounded 6-DoF LiDAR residual correction with finite-sample/pose validation and Huber-weighted Kabsch centroids/covariance, corrected-pose LiDAR/Gaussian correspondence construction before BA ingestion, robust direct LiDAR/Gaussian-map point-to-point window factors with `(0, 1]` correspondence-weight validation, LiDAR point-to-plane window factors, Huber-robust visual-alignment and SE3 photometric window factors, analytic geometric Jacobians for point-to-point, point-to-plane, visual-alignment, and full current IMU preintegration factors, visual residual/direct subpixel image-alignment comparison against mapper renders, runtime-gated 2-DoF photometric translation linearization, analytic SE3 camera photometric pixel Jacobians, multi-sample normal equations, robust runtime rendered/current/depth sampling into optional SE3 photometric window factors, optional visual-alignment window factors, chunk-complete GaussianMap snapshot caching, odometry/path/status/optional TF publication, and deterministic probes.
- Dataset profile YAMLs derived from upstream Gaussian-LIC: FAST-LIVO, FAST-LIVO2, M2DGR, MCD, and R3LIVE.
- `gaussian_lic_offline` CLI for rosbag2 artifact extraction without launching ROS nodes.
- FAST-LIVO2 ROS1-to-ROS2 raw frontend conversion, ROS1 mapper-contract conversion, upstream ROS1 baseline runner, strict rosbag2 replay wrapper, baseline manifest/readiness gates, and combined reproduction reports.
- Current executable Bright substitute report with `metrics`, `trajectory`, `point_cloud`, and dedicated Torch Gaussian `gaussian_color` gates passing.
- Strict FAST-LIVO2 `CBD_Building_01` report with trajectory, PSNR/SSIM/LPIPS, GT-associated render pairs, and Chamfer gates passing locally.
- SPNet TensorRT engine generation for the local `sm_120` GPU via TensorRT 10.9, with the generated FP16 engine kept outside git.
- Native tracking probes are registered with CTest, so `colcon test --packages-select gaussian_lic_tracking` runs trajectory, IMU, LiDAR, sliding-window, bias observability, geometric Jacobian, Gaussian snapshot, trajectory smoothness, SE3 photometric Jacobian/factor, and visual checks automatically. `scripts/tracking_smoke_test.sh` also verifies the launch path through `/gaussian_lic/frontend/status`, including signed-nanosecond time status, tracking QoS, executor callback serialization, dense-prior health, bias observability, optimized IMU re-anchors, B-spline trajectory control poses, nonzero trajectory smoothness factors, accepted/rejected/limited BA step health, factor skip counters, visual alignment, photometric linearization status, rendered/depth cache diagnostics, and SE3 photometric sample quality/rejection counters.

Still pending:

- Full Coco-LIC2-grade frontend/tracking algorithm: production sliding-window joint BA with dataset-validated IMU/LiDAR/camera factors beyond the current gated foundation.
- Full native Coco-LIC2 frontend/tracking parity beyond the mapper-contract reproduction path.

## Progress Ledger

| Scope | Current status |
| --- | --- |
| ROS2 workspace, messages, launch, composable node, profiles, bag checks, artifact gates | Complete for the current porting surface |
| FAST-LIVO2 Bright substitute evidence chain | Complete and executable with `metrics`, `trajectory`, `point_cloud`, and `gaussian_color` passing |
| Strict `CBD_Building_01` paper-data gate | Official bag is local; ROS1 baseline is archived; ROS2 current collection/report passes the strict local gate |
| Paper-level Gaussian-LIC/Gaussian-LIC2 algorithm migration | Mapper CUDA/Torch backend, strict mapper-contract parity, and local SPNet TensorRT engine generation are in place; full Coco-LIC2 tracking BA remains |

The Bright substitute report is a regression/evidence chain for current ROS2 plumbing and Torch Gaussian tensor boundaries. It is not a claim that the full paper algorithm has been ported.

## Current Executable Proof Chain

The current Bright proof chain uses the official FAST-LIVO2 `Bright_Screen_Wall` bag as a substitute for fast regression coverage. It compares ROS2 current artifacts against an upstream ROS1 Gaussian-LIC/Gaussian-LIC2 baseline and adds a Torch Gaussian color gate in the same report.

Validated artifacts:

```text
/home/frank/data/fast_livo/Bright_Screen_Wall_mapper_contract_fastlivo2_color_8s.bag
baseline/fastlivo2/Bright_Screen_Wall_fastlivo2_color_8s/
results/fastlivo2/Bright_Screen_Wall_current_extrinsic_color_fullseq/reproduction_report.json
results/fastlivo2/Bright_Screen_Wall_current_extrinsic_color_torch/point_cloud.ply
```

Refresh the combined report from existing artifacts:

```bash
./scripts/run_curated_fastlivo2_report.sh \
  --fastlivo2-camera-lidar-transform \
  --colorize-pointcloud \
  --sequence Bright_Screen_Wall_fastlivo2_color_8s \
  --mapper-bag /home/frank/data/fast_livo/Bright_Screen_Wall_mapper_contract_fastlivo2_color_8s.bag \
  --baseline-dir baseline/fastlivo2/Bright_Screen_Wall_fastlivo2_color_8s \
  --current-dir results/fastlivo2/Bright_Screen_Wall_current_extrinsic_color_fullseq \
  --skip-convert \
  --skip-current \
  --skip-baseline \
  --skip-build \
  --gaussian-color-current-point-cloud results/fastlivo2/Bright_Screen_Wall_current_extrinsic_color_torch/point_cloud.ply
```

Expected output:

```text
reproduction report OK: sequence=Bright_Screen_Wall_fastlivo2_color_8s metrics=PASS trajectory=PASS point_cloud=PASS gaussian_color=PASS
```

The latest local report has Torch Gaussian mean RGB drift `11.657 < 40.0`.

## Algorithmic Gaps

The mapper backend now has the major CUDA/Torch surfaces in tree, but the full paper system is not finished.

- The native tracker has timestamp-safe trajectory/IMU primitives, ROS2-configurable LiDAR-to-IMU and camera-to-IMU extrinsics, IMU history interpolation, raw-sample IMU preintegration/reintegration, optimized pose/velocity/bias feedback into odometry, continuous-time trajectory controls, and safe IMU re-anchoring, IMU bias continuity residuals, per-block IMU rotation/velocity/position residual weights, dense marginalization-prior anchoring, bias observability metrics/probes, reusable normal-equation linearization, normal-equation rank/condition diagnostics, configurable rank/condition solve guards, factor-agnostic BA solve triggering, bounded LM state increments, Schur-complement retained-window dense priors, timestamp-safe cubic B-spline position/velocity plus SO(3) cubic orientation trajectory queries for deskew fallback, default-enabled three-state trajectory smoothness factors with analytic linear Jacobian blocks, per-point LiDAR deskew with invalid-pose rejection, Huber-weighted bounded 6-DoF LiDAR pose correction, corrected-pose LiDAR/Gaussian correspondence construction, robust direct LiDAR/Gaussian-map point-to-point window factors with bounded robust weights, LiDAR point-to-plane window factors, Huber-robust visual-alignment and SE3 photometric window factors, analytic geometric Jacobians for point-to-point, point-to-plane, visual-alignment, full current IMU preintegration factors including start-bias reintegration sensitivity, pose-prior, state-prior, dense-prior, IMU bias-continuity, and SE3 photometric window rows, analytic SE3 camera photometric pixel Jacobians with multi-sample normal-equation solving, runtime rendered/current/depth extraction into default-enabled SE3 photometric window factors with camera-to-IMU adjoint conversion plus body-frame photometric Hessian/sqrt-information, nearest-stamp cached render/depth selection, and signed-nanosecond image/LiDAR/depth/render freshness gates, chunk-complete Gaussian snapshot caching, a default-enabled sliding-window optimizer foundation, direct visual residual/subpixel alignment, runtime-gated photometric translation linearization, and default-enabled visual-alignment window factors, but it is not yet the full continuous-time Coco-LIC2 production BA stack.
- Full joint BA remains to be ported beyond the current deskewed bounded 6-DoF LiDAR correction, 2-DoF visual translation linearization gate, SE3 photometric window-factor runtime gate, dense-prior health gate, and bias observability gate into a dataset-validated continuous-time Coco-LIC2 production frontend.
- TensorRT/SPNet depth completion has a native optional wrapper and a local TensorRT 10.9 FP16 engine benchmark; the generated `.engine` is hardware/runtime-specific and intentionally not checked in.
- Strict paper reproduction now has the local `CBD_Building_01` bag, ROS1 upstream baseline artifacts, strict CUDA current collection, final-map render-pair extraction, and a passing local strict report. Full native tracker parity remains outside this mapper-contract gate.

## Platform

Primary tested platform:

```text
Ubuntu 24.04
ROS2 Jazzy
CUDA 12.8
libtorch 2.7.0
```

Humble is represented in CI as build-only scaffolding; Jazzy is the first-class development target. Runtime helper scripts default to `ROS_DISTRO=jazzy` and can be pointed at another installed distro by exporting `ROS_DISTRO` before running them.

On this machine, use the ASCII workspace path:

```bash
cd /home/frank/gaussian_lic_ros2
```

`/home/frank/桌面/gaussian_lic_ros2` is only a symlink. Some ROS2 interface-generation tools are fragile under non-ASCII parent paths.

## Quick Start

Build and run the default synthetic smoke test:

```bash
source /opt/ros/jazzy/setup.bash
./scripts/build_ros2.sh
source install/setup.bash
./scripts/smoke_test.sh --tf
```

Run the native tracking probe suite:

```bash
colcon test --packages-select gaussian_lic_tracking --event-handlers console_direct+
colcon test-result --verbose
```

Expected local result is `17 tests, 0 errors, 0 failures, 0 skipped`.

Run the full local verification wrapper:

```bash
./scripts/verify_workspace.sh --full-profiles
```

The build wrapper pins ROS2 interface generation to `/usr/bin/python3` so conda Python does not get selected by CMake. `scripts/build_jazzy.sh` remains as a compatibility wrapper around `scripts/build_ros2.sh`.

## Synthetic Rosbag2 Demo

Generate a small synthetic bag:

```bash
./scripts/create_synthetic_bag.sh --output bags/synthetic_gs_demo
```

Generate a synthetic raw frontend bag for the LIC2 adapter boundary:

```bash
./scripts/create_synthetic_bag.sh \
  --frontend-raw \
  --output bags/synthetic_frontend_raw_demo
```

Use `--frontend-raw-odometry` to record `/gaussian_lic/frontend/input_odometry` instead of `/gaussian_lic/frontend/pose`.

Replay it through the native mapper:

```bash
./scripts/smoke_test.sh --bag bags/synthetic_gs_demo --tf
```

Run the same generated-bag replay smoke used by the Jazzy CI leg:

```bash
./scripts/ci_replay_smoke.sh --bag bags/ci_synthetic_gs_demo --duration 4 --timeout 20
```

Add `--frontend-bag bags/ci_synthetic_frontend_raw_demo` to choose the raw frontend bag path used by the LIC2 adapter replay leg.
Add `--artifact-dir /path/to/reports` to keep the generated bag contract reports, offline `metrics.json`, `trajectory.tum`, and `point_cloud_debug.ply`. GitHub Actions uploads those files as `jazzy-replay-artifacts`.
The script also writes `replay_summary.md`, which CI appends to the job summary.

Run the same bag with a dataset profile:

```bash
./scripts/smoke_test.sh \
  --bag bags/synthetic_gs_demo \
  --config src/gaussian_lic_bringup/config/fastlivo2.yaml \
  --render-mode debug_cpu \
  --skip-rendered-data-check \
  --tf
```

The synthetic demo image is `1x1`; real dataset intrinsics can project the synthetic point outside the image, so profile smoke tests skip the red-pixel assertion.

Exercise the native LIC2 frontend contract adapter with synthetic raw sensor topics:

```bash
./scripts/smoke_test.sh --frontend-adapter --tf
```

This publishes `/camera/image`, `/camera/camera_info`, `/camera/depth`, `/livox/lidar`, `/imu`, and `/gaussian_lic/frontend/pose`, then lets `lic2_contract_adapter` forward them into `/image_for_gs`, `/camera_info_for_gs`, `/depth_for_gs`, `/points_for_gs`, `/imu_for_gs`, and `/pose_for_gs`. The adapter also publishes `/gaussian_lic/frontend/odometry` and `/gaussian_lic/frontend/path` from incoming pose/odometry. It is a ROS2 boundary adapter, not the finished LIC2 odometry algorithm.
Add `--frontend-odometry-input` to publish synthetic `/gaussian_lic/frontend/input_odometry` instead of PoseStamped.

## Offline Mode

Extract reproducibility artifacts from a rosbag2 directory without starting a ROS launch:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run gaussian_lic_tools gaussian_lic_offline \
  --bag bags/synthetic_gs_demo \
  --output /tmp/gaussian_lic_offline_demo
```

Current outputs:

```text
trajectory.tum
point_cloud_debug.ply
metrics.json
```

This is the seed for the future `gaussian_lic_offline` reproduction binary. The current implementation reads mapper contract topics, preserves PointCloud2 `rgb`/`rgba` or `r/g/b` colors in the debug PLY when available, and writes metrics for topic rates, trajectory path length, point-cloud bounds, and color coverage. It does not yet run the full Gaussian-LIC2 algorithm offline.

The same artifact extractor can read ROS1 `.bag` inputs directly when the optional `rosbags` package is installed in that Python environment:

```bash
/usr/bin/python3 -m pip install --user rosbags
PYTHONPATH=src/gaussian_lic_tools \
  python3 -m gaussian_lic_tools.offline \
  --bag /path/to/input.bag \
  --bag-format ros1 \
  --output /tmp/gaussian_lic_ros1_offline_demo
```

Direct ROS1 extraction writes the same `trajectory.tum`, `point_cloud_debug.ply`, and `metrics.json` files, including `bag_format` and `storage_identifier` fields, so converted and original bags can be compared with the same scripts.

Validate that a rosbag2 directory has the mapper input contract before replaying it:

```bash
ros2 run gaussian_lic_tools gaussian_lic_bag_check \
  --bag bags/synthetic_gs_demo \
  --json
```

For intermediate replay bags that only contain the mapper's synchronized core topics, use:

```bash
ros2 run gaussian_lic_tools gaussian_lic_bag_check \
  --bag /path/to/intermediate_bag \
  --contract mapper_minimal \
  --json
```

`mapper_minimal` requires `/points_for_gs`, `/pose_for_gs`, and `/image_for_gs`. It still reports `/camera_info_for_gs`, `/depth_for_gs`, and `/imu_for_gs` when present, but missing optional topics do not fail the check. Use it with `require_depth_topic:=false` and profile intrinsics when replaying bags that do not carry depth or CameraInfo.

Validate a raw sensor bag before feeding it through `lic2_contract_adapter`:

```bash
ros2 run gaussian_lic_tools gaussian_lic_bag_check \
  --bag bags/synthetic_frontend_raw_demo \
  --contract frontend_raw \
  --json
```

`frontend_raw` requires raw image, CameraInfo, PointCloud2 LiDAR, and IMU topics, requires at least one pose source from `/gaussian_lic/frontend/pose` or `/gaussian_lic/frontend/input_odometry`, and treats `/camera/depth` as optional.

For true LIC2 frontend input bags that do not have pose yet, use:

```bash
ros2 run gaussian_lic_tools gaussian_lic_bag_check \
  --bag /path/to/frontend_sensor_raw_bag \
  --contract frontend_sensor_raw \
  --json
```

Convert an official FAST-LIVO2 ROS1 bag into that ROS2 raw-sensor contract:

```bash
PYTHONPATH=/home/frank/.cache/gaussian_lic_ros2/rosbags-venv/lib/python3.12/site-packages \
  /usr/bin/python3 scripts/fastlivo2_ros1_to_frontend_raw.py \
  --input /home/frank/data/fast_livo/Bright_Screen_Wall.bag \
  --output /home/frank/data/fast_livo/Bright_Screen_Wall_frontend_raw \
  --overwrite
```

Replay a minimal rosbag2 mapper bag with:

```bash
./scripts/smoke_test.sh \
  --bag /path/to/intermediate_ros2_bag \
  --minimal-inputs \
  --config src/gaussian_lic_bringup/config/r3live.yaml
```

The same checker can inspect ROS1 `.bag` metadata before conversion when the optional `rosbags` package is installed:

```bash
/usr/bin/python3 -m pip install --user rosbags
ros2 run gaussian_lic_tools gaussian_lic_bag_check \
  --bag /path/to/input.bag \
  --bag-format ros1 \
  --contract mapper_minimal \
  --json
```

## ROS1 Bag Conversion

Convert an archived ROS1 `.bag` into a ROS2 bag directory for replay:

```bash
/usr/bin/python3 -m pip install --user rosbags
./scripts/convert_ros1_bag_to_rosbag2.py \
  --input /path/to/input.bag \
  --output bags/input_ros2 \
  --storage mcap \
  --dst-typestore ros2_jazzy
```

Use repeated `--topic` or `--include-topic` arguments to convert only the mapper contract topics when preparing focused reproduction bags.

## Torch Backend Probe

Build the optional libtorch/CUDA path:

```bash
GAUSSIAN_LIC_ENABLE_TORCH=ON ./scripts/build_ros2.sh --packages-select gaussian_lic_mapping
source install/setup.bash
ros2 run gaussian_lic_mapping torch_backend_probe
```

Expected shape output includes:

```text
image_sizes=[3, 1, 1]
depth_sizes=[1, 1]
gaussian_xyz_sizes=[2, 3]
gaussian_features_dc_sizes=[2, 1, 3]
gaussian_features_rest_sizes=[2, 15, 3]
gaussian_count_after_init=1
appended_count=1
gaussian_count=2
```

On the tested machine, CUDA is available. TensorRT is optional for the ROS2 porting surface; the local upstream ROS1 baseline Docker path has also been validated with TensorRT 8.6.1.6.

For SPNet depth-completion engine generation on the local RTX 5070 Ti
(`sm_120`) machine, TensorRT 8.6.1.6 is too old for engine building. Use the
local TensorRT 10.9 CUDA 12.8 SDK layout generated by
`scripts/install_local_tensorrt_10_9.sh` instead.

Current local SPNet runtime status:

```text
/home/frank/Software/TensorRT-10.9.0.34-cuda12.8
/home/frank/Software/TensorRT-engines/spnet_512_640_fp16.engine
```

The latest `trtexec` benchmark reports mean latency `26.4492 ms` at
`512x640`, below the 30 ms/frame target. See
`docs/SPNET_RUNTIME_STATUS.md` for hashes and the TensorRT 8.6 incompatibility
note.

Rebuild the local engine with:

```bash
./scripts/install_local_tensorrt_10_9.sh
SPNET_PYTHON=/home/frank/.cache/gaussian_lic_ros2/quality-venv/bin/python \
TENSORRT_ROOT=/home/frank/Software/TensorRT-10.9.0.34-cuda12.8 \
  ./scripts/build_spnet_engine.sh --output-dir /home/frank/Software/TensorRT-engines
```

## ROS2 Interfaces

Default logical sensor contract:

```text
/camera/image
/camera/camera_info
/livox/lidar
/imu
/tf
/tf_static
```

Current mapper contract:

```text
/points_for_gs       sensor_msgs/msg/PointCloud2
/pose_for_gs         geometry_msgs/msg/PoseStamped
/image_for_gs        sensor_msgs/msg/Image
/camera_info_for_gs  sensor_msgs/msg/CameraInfo
/depth_for_gs        sensor_msgs/msg/Image
/imu_for_gs          sensor_msgs/msg/Imu
```

`gaussian_lic_frontend/lic2_contract_adapter` provides the first native frontend boundary. Its default raw inputs are:

```text
/camera/image
/camera/camera_info
/camera/depth
/livox/lidar
/imu
/gaussian_lic/frontend/pose
/gaussian_lic/frontend/input_odometry
```

It republishes raw sensor and pose/odometry inputs into the mapper contract above. It also publishes:

```text
/gaussian_lic/frontend/odometry
/gaussian_lic/frontend/path
```

If only odometry is available, set `raw_odometry_topic` and it converts `nav_msgs/msg/Odometry` into `/pose_for_gs` while keeping the stable frontend output topics separate from the raw odometry input.

When `/points_for_gs` has no `rgb`, `rgba`, or `r/g/b` fields, the mapper projects each valid camera-frame point into `/image_for_gs` using the active `CameraInfo` intrinsics and samples RGB from the image. Points outside the image keep the white fallback color.

Use `./scripts/smoke_test.sh --image-color-fallback-check` to verify that fallback path with a synthetic uncolored cloud.

Set `require_depth_topic:=false` to run the mapper without synchronized `/depth_for_gs`; it will create a sparse metric depth image by projecting valid map points into the current image. The default profile keeps depth required, while upstream-derived dataset profiles allow missing depth so replay is not blocked when TensorRT/SPNet depth completion is unavailable.

Current outputs:

```text
/gaussian_lic/odometry
/gaussian_lic/path
/gaussian_lic/map_points
/gaussian_lic/rendered_image
/gaussian_lic/gaussian_map
/gaussian_lic/status
/gaussian_lic/save_map
```

`/gaussian_lic/rendered_image` currently defaults to the CUDA rasterizer path:

```text
render_mode:=rasterizer
torch_gaussian_device:=cuda
```

The packaged dataset profiles and launch defaults are biased toward the paper
gate path now: Torch camera conversion, Gaussian initialization, CUDA device,
photometric optimization, pruning, append-only upstream-style extension, and
rasterizer preview are on by default. `scripts/collect_current_results.sh`
still preserves its explicit `--torch` contract: when the script is called
without `--torch`, it passes launch overrides that disable Torch even though the
general launch defaults are CUDA-first. ROS2 gradient split/clone densification
is available as an explicit diagnostic switch, but it is off by default because
the released Gaussian-LIC2 path uses append-only `extend()`.
`rendered_image_mode` is kept only as a deprecated compatibility alias.

See [docs/STATUS_SCHEMA.md](docs/STATUS_SCHEMA.md) for the status topic schema.

## Dataset Profiles

Packaged profiles:

```text
src/gaussian_lic_bringup/config/fastlivo.yaml
src/gaussian_lic_bringup/config/fastlivo2.yaml
src/gaussian_lic_bringup/config/m2dgr.yaml
src/gaussian_lic_bringup/config/mcd.yaml
src/gaussian_lic_bringup/config/r3live.yaml
```

Validate profile schema:

```bash
./scripts/check_profiles.py
```

These profiles currently cover the native mapper surface: topic names, global and per-input QoS, synchronization, upstream camera intrinsics, image size, depth-completion toggles, Gaussian initialization/map-growth parameters, upstream optimizer/loss/exposure parameter names, output topics, and active profile labels. The contract adapter and native tracking launch also expose per-stream raw-input and mapper-output QoS so selected streams can be promoted without making every camera/LiDAR/IMU/depth/pose topic reliable. The native tracking launch keeps high-rate tracking inputs at bounded `best_effort` by default, exposes per-point LiDAR deskew controls for common `offset_time`, `time`, `timestamp`, and `t` fields while preserving signed-nanosecond estimator math, rejects invalid builtin stamp nanosecond fields, non-monotonic tracking stream stamps, non-finite IMU measurements, non-finite CameraInfo intrinsics, non-finite extrinsic parameters, non-finite IMU gravity, invalid LiDAR keyframe thresholds, invalid cache sizes, invalid visual/SE3 photometric gates, invalid LiDAR thresholds, and invalid sliding-window BA bounds before mapper/frontend arithmetic, rejects invalid deskew poses before they can create NaN points, publishes latest signed-nanosecond image/LiDAR/IMU stamps, stamp-regression counters, and invalid-IMU counters for smoke gating, and serializes tracking callbacks by default so a future multi-threaded executor cannot concurrently mutate IMU/LiDAR/image estimator state. The sliding-window optimizer is now default-enabled for production BA runtime coverage, with reusable normal-equation linearization, rank/condition health diagnostics and solve guards, factor-agnostic solve triggering when LiDAR/visual/smoothness factors arrive without a valid IMU factor, bounded LM state increments plus accepted/rejected step status, dense prior anchoring, Schur-complement retained-window priors with rank/singular-value health, optimized pose/velocity/bias feedback into odometry, trajectory controls, and safe IMU re-anchoring, timestamp-safe cubic B-spline position/velocity and SO(3) cubic orientation trajectory queries for deskew fallback, default-enabled three-state trajectory smoothness factors, bias observability status, Huber-weighted LiDAR 6-DoF pose correction, corrected-pose LiDAR/Gaussian correspondence construction, per-correspondence robust LiDAR/Gaussian-map weights bounded to `(0, 1]`, LiDAR point-to-plane factors, Huber-robust visual-alignment and SE3 photometric window factors, analytic geometric and full current IMU preintegration Jacobians, robust analytic SE3 camera photometric normal equations/window factors from rendered/current/depth images selected through bounded nearest-stamp render/depth caches with body-frame sqrt-information, runtime status for 2-DoF and SE3 photometric linearization quality, and subpixel visual alignment in place. Full real-dataset frontend parity still needs Coco-LIC2-grade production BA.

## Performance Regression

Compare current metrics against a baseline:

```bash
./scripts/perf_regression.py \
  --metrics results/fastlivo2/current/metrics.json \
  --baseline baseline/fastlivo2/CBD_Building_01/metrics.json
```

Default gates check:

```text
tracking_hz
mapping_hz
mean_iteration_ms
```

A regression greater than 15% fails by default.

Compare a current TUM trajectory against an archived ROS1 baseline:

```bash
./scripts/trajectory_compare.py \
  --baseline baseline/fastlivo2/CBD_Building_01/trajectory.tum \
  --current results/fastlivo2/current/trajectory.tum \
  --output results/fastlivo2/current/trajectory_compare.json \
  --max-rmse-m 0.05 \
  --max-path-drift 0.05
```

The trajectory gate associates poses by timestamp, reports translation RMSE/mean/max plus path-length drift, and exits non-zero when any threshold fails. Use `--align first` only for datasets whose baseline and current trajectories differ by a known initial translation offset.

Compare a current PLY map against an archived ROS1 baseline:

```bash
./scripts/pointcloud_compare.py \
  --baseline baseline/fastlivo2/CBD_Building_01/point_cloud.ply \
  --current results/fastlivo2/current/point_cloud.ply \
  --output results/fastlivo2/current/pointcloud_compare.json \
  --voxel-size 0.05 \
  --max-chamfer-rmse-m 0.15
```

The point-cloud gate supports ASCII and binary little-endian PLY files with standard `x/y/z` vertex properties. It reports point-count ratio, centroid drift, bidirectional nearest-neighbor RMSE/mean/max, unmatched ratio, and RGB mean drift when both PLY files include `red/green/blue`. Add `--derive-gaussian-rgb` when the PLY stores Gaussian color in upstream `f_dc_0..2` coefficients instead of explicit RGB fields.

Generate one combined reproduction report:

```bash
./scripts/reproduction_report.py \
  --baseline-dir baseline/fastlivo2/CBD_Building_01 \
  --current-dir results/fastlivo2/current \
  --output results/fastlivo2/current/reproduction_report.json \
  --markdown results/fastlivo2/current/reproduction_report.md
```

The combined report validates the baseline manifest and runs metrics, trajectory, and PLY map drift gates in one command. It exits non-zero when any gate fails and is intended as the future CI comparison artifact.

For the current Bright substitute proof chain, add a dedicated Torch Gaussian color gate:

```bash
./scripts/reproduction_report.py \
  --baseline-dir baseline/fastlivo2/Bright_Screen_Wall_fastlivo2_color_8s \
  --current-dir results/fastlivo2/Bright_Screen_Wall_current_extrinsic_color_fullseq \
  --sequence Bright_Screen_Wall_fastlivo2_color_8s \
  --metric-key debug_points \
  --trajectory-align first \
  --min-trajectory-coverage 0.4 \
  --max-association-dt 0.2 \
  --pointcloud-align centroid \
  --max-pointcloud-points 50000 \
  --max-nearest-m 0.5 \
  --max-chamfer-rmse-m 0.5 \
  --max-chamfer-mean-m 0.5 \
  --max-chamfer-max-m 0.5 \
  --max-unmatched-ratio 0.25 \
  --min-point-count-ratio 0.5 \
  --max-point-count-ratio 5.0 \
  --max-centroid-drift-m 0.5 \
  --gaussian-color-current-point-cloud results/fastlivo2/Bright_Screen_Wall_current_extrinsic_color_torch/point_cloud.ply \
  --output results/fastlivo2/Bright_Screen_Wall_current_extrinsic_color_fullseq/reproduction_report.json \
  --markdown results/fastlivo2/Bright_Screen_Wall_current_extrinsic_color_fullseq/reproduction_report.md
```

That report includes five gates: `baseline_manifest`, `metrics`, `trajectory`, `point_cloud`, and `gaussian_color`.

Run the same Python-only artifact gates used by GitHub Actions locally:

```bash
./scripts/verify_artifact_gates.sh
```

Set `GAUSSIAN_LIC_ARTIFACT_DIR=/path/to/reports` to choose where the synthetic JSON/Markdown reports are written. In GitHub Actions those files are uploaded as the `artifact-gate-reports` workflow artifact.

Check real FAST-LIVO2 baseline readiness:

```bash
./scripts/baseline_readiness.py \
  --dataset-root /home/frank/data/fast_livo \
  --baseline-dir baseline/fastlivo2/CBD_Building_01 \
  --current-results-dir results/fastlivo2/current \
  --sequence CBD_Building_01 \
  --strict \
  --markdown baseline_readiness.md
```

Fetch the official FAST-LIVO2 quick-start bag only when the data gate reports missing raw data:

```bash
./scripts/fetch_fastlivo2_sequence.py \
  --sequence CBD_Building_01 \
  --output-dir /home/frank/data/fast_livo
```

Run or resume the strict chain from the local `CBD_Building_01` bag:

```bash
./scripts/run_strict_cbd_pipeline.sh \
  --overwrite \
  --current-torch-max-foreground 1500000
```

The strict script defaults to CUDA rasterizer mode, `torch_gaussian_device=cuda`,
`--current-torch-optimization-steps 100`,
`--current-torch-optimization-sampling upstream_random`, a fixed
`--current-torch-optimization-seed 20260505` for reproducible strict reports, a
slowed current playback rate of `0.25`, a 60 second post-playback settle window,
a 1.5M foreground Gaussian cap, uniform foreground count-cap policy with
`torch_gaussian_prune_min_opacity=0.0`,
upstream-style alpha-hole filtering before Gaussian extension, disabled ROS2
gradient split/clone densification, disabled opacity resets, and disabled
high-rate `GaussianArray` publication. In this path the step count means up to
100 accumulated train-frame optimizer samples per keyframe, and the selected
frames are sampled and shuffled in the same style as upstream Gaussian-LIC
`optimize()` instead of being processed as a sorted, evenly spaced list that
always forces the newest frame. Set the seed to `0` to use `std::random_device`
like upstream's non-deterministic run. Gradient split/clone densification is off
by default because Gaussian-LIC2's released mapping path grows the map through
append-only `extend()`; enabling the ROS2-only split/clone path produced large
blurred splats and a much lower strict PSNR/SSIM run locally. Opacity reset is
also off by default because Gaussian-LIC2's released optimizer does not run a
periodic reset; leaving the ROS2 recovery heuristic on can reset the whole
foreground to opacity 0.01 immediately before final-map evaluation. The slowed
playback is deliberate: the strict CUDA path is heavier than the live preview
path, so 1x rosbag2 replay can underfeed the final-map metric gate by dropping
unprocessed frames. The settle window lets the mapper consume queued frames
before `SaveMap` writes final-map render pairs. Disabling the visualization
Gaussian map topic keeps the strict recorder from writing tens of GiB of
high-frequency chunked Gaussian messages that are not used by the metric gate.
CUDA current collection also defaults
`PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True` unless the caller has already
set it, which reduces late-run allocator fragmentation around million Gaussian
maps.

That command converts the official ROS1 bag to a sqlite-backed ROS2
`frontend_raw` bag, audits replay timing, writes the ROS1 mapper-contract bag,
runs the upstream ROS1 baseline container, collects ROS2 current artifacts, and
emits strict readiness and reproduction reports. During report generation it also
fills `metrics.json::quality`: the baseline uses upstream GT, while the current
run uses its saved `gt/` frames so dropped/reordered replay frames do not get
scored against the wrong source image. Render-pair comparison uses decoded GT
hashes to associate ROS1 and ROS2 frame identities before sampling pairs. If
`${HOME}/.cache/gaussian_lic_ros2/quality-venv/bin/python` exists, it is used for
CPU LPIPS evaluation; otherwise set `QUALITY_PYTHON` to a Python environment with
`torch`, `torchvision`, `numpy`, and `Pillow`.

Latest local strict run, 2026-05-05:

- ROS1 baseline visual dump: 1186 render/GT pairs, 237 train and 949 novel frames.
- ROS2 current strict path: CUDA rasterizer, final-map evaluation, IMU-fallback
  world-frame point-cloud rotation enabled, ROS1-compatible projected-depth
  nearest-pixel rounding, high-rate GaussianArray publication disabled, 1.5M
  foreground cap with no opacity pruning and uniform count-cap policy,
  alpha-hole extension filtering
  enabled at threshold `0.99`, ROS2 gradient split/clone densification disabled,
  opacity reset disabled, deterministic upstream-random optimization-frame
  sampling enabled with seed `20260505`,
  `PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True`, 1063 render/GT pairs, 212
  train and 851 novel frames, trajectory coverage 89.63%. The trajectory gate
  passes and the optimizer finishes with `gaussian_opt_errors=0`,
  `gaussian_densified_total=0`, and `gaussian_opacity_resets=0`.
- Current vs ROS1 quality passes the paper metric gate: ROS2 novel PSNR
  12.65 dB vs ROS1 12.70 dB, ROS2 novel SSIM 0.369 vs ROS1 0.364, and ROS2
  novel LPIPS 0.742 vs ROS1 0.751. The GT-associated 64-pair ROS1-vs-ROS2
  render summary is mean PSNR 21.60 dB and mean SSIM 0.785.
- Chamfer/point-cloud parity passes: centroid drift 0.148 m, bidirectional
  nearest mean 0.0994 m, bidirectional RMSE 0.1455 m, unmatched ratio 2.83%,
  and declared point-count ratio 0.978.

Visual review artifacts from the same strict run:

```text
docs/assets/strict_cbd_montage.jpg
docs/assets/strict_cbd_render_demo.gif
```

So the strict chain now passes the local `CBD_Building_01` reproduction gate
for the mapper-contract/CUDA current path. The remaining paper-port work is full
native Coco-LIC2 tracking, not missing render pairs, disabled CUDA runtime,
missing SPNet engine generation, or failing PSNR/SSIM/LPIPS/Chamfer parity.

## Release Roadmap

The public plan is release-driven:

```text
v0.1.0  Baseline & Infrastructure
v0.2.0  Gaussian-LIC2 ROS2 Native Frontend
v0.3.0  Gaussian Mapping Full Backend
v0.4.0  Strict FAST-LIVO2 Reproduction
```

Strict paper-data action: run `scripts/run_strict_cbd_pipeline.sh` from the local
`CBD_Building_01` bag, keep the ROS1 baseline archive fixed, and confirm the
ROS2 current report stays inside the 5% gate.

If the local raw bag is missing, the data fetch is executable:

```bash
./scripts/fetch_fastlivo2_sequence.py \
  --sequence CBD_Building_01 \
  --output-dir /home/frank/data/fast_livo
```

Then run the original Gaussian-LIC/Gaussian-LIC2 upstream on FAST-LIVO2 and archive artifacts under:

```text
baseline/fastlivo2/<sequence>/
```

Validate and fingerprint the archived baseline before comparing ROS2 results:

```bash
./scripts/baseline_manifest.py \
  --baseline baseline/fastlivo2/CBD_Building_01 \
  --sequence CBD_Building_01 \
  --write
```

The baseline manifest checks `trajectory.tum`, `point_cloud.ply`, `metrics.json`, `run.log`, and `renders/`, then writes artifact sizes, SHA-256 hashes, pose count, PLY vertex count, and render count to `baseline_manifest.json`.

Current readiness gate:

```text
./scripts/baseline_readiness.py --dataset-root /home/frank/data/fast_livo --sequence CBD_Building_01
```

See [docs/BASELINE_DATA.md](docs/BASELINE_DATA.md), [docs/RELEASE_MILESTONES.md](docs/RELEASE_MILESTONES.md), and [docs/ROADMAP.md](docs/ROADMAP.md).

## Docker

Non-torch Jazzy smoke-test container:

```bash
docker compose -f docker/compose.yaml run --rm smoke
```

See [docs/DOCKER.md](docs/DOCKER.md).

## Upstream Sources

Fetch upstream sources for porting work:

```bash
./scripts/fetch_upstreams.sh
```

This clones:

```text
external/Gaussian-LIC
```

`external/Gaussian-LIC` is the primary upstream and now carries the Gaussian-LIC2 release path. To also fetch the historical Coco-LIC ROS1 reference, run:

```bash
./scripts/fetch_upstreams.sh --with-legacy-cocolic
```

That adds:

```text
external/Coco-LIC
```

Those directories are ignored by git. The upstream Gaussian-LIC/Gaussian-LIC2 project is GPL-3.0; Coco-LIC is treated as an optional legacy GPL-3.0 reference.

## License

This repository is GPL-3.0-or-later compatible because Gaussian-LIC/Gaussian-LIC2 is a GPL-3.0 upstream project. Do not copy upstream source into this repository under a different license.
