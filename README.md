# Gaussian-LIC ROS2

Native ROS2 engineering port for [Gaussian-LIC/Gaussian-LIC2](https://github.com/APRIL-ZJU/Gaussian-LIC).

The long-term target is a paper-level ROS2 reproduction of LiDAR-Inertial-Camera Gaussian Splatting SLAM: native ROS2 tracking, native Gaussian mapping, rosbag2 replay, offline reproducibility tooling, and strict comparison against the ROS1 upstream baseline.

This repository is **not a ROS1 bridge wrapper**. It is a clean ROS2 workspace that keeps algorithm code isolated from middleware glue so the tracking and mapping backends can be ported incrementally without locking the project into ROS1-era APIs.

## Current Release State

This repository is now an executable ROS2 porting checkpoint for the public Gaussian-LIC/Gaussian-LIC2 code path. It has native ROS2 message, launch, adapter, mapper, CUDA/Torch Gaussian backend plumbing, initial native tracking factors, artifact extraction, strict replay/readiness tooling, and an official FAST-LIVO2 Bright substitute proof chain.

It is **not yet** the strict full-paper Gaussian-LIC2 ROS2 port: the native tracker still needs full Coco-LIC2-grade sliding-window joint optimization, TensorRT/SPNet needs a runtime engine artifact, and the strict `CBD_Building_01` current report is still failing the paper-quality gate against the locally archived ROS1 upstream baseline.

Available now:

- Native ROS2 packages for messages, launch/config, mapping scaffold, and test tools.
- `gaussian_lic_frontend/lic2_contract_adapter` for routing raw ROS2 camera/LiDAR/IMU/pose topics into the mapper contract.
- Optional Livox `CustomMsg` to `PointCloud2` bridge for raw Livox driver packets.
- ROS2 synchronization/conversion for the Gaussian-LIC mapper input contract:
  `/points_for_gs`, `/pose_for_gs`, `/image_for_gs`, `/camera_info_for_gs`, `/depth_for_gs`, `/imu_for_gs`.
- Odometry, path, TF, debug map cloud, rendered debug preview, status, GaussianArray, and SaveMap interfaces.
- Optional libtorch/CUDA path for keyframe-gated Gaussian tensor initialization, skybox seeding, incremental foreground insertion, CUDA rasterizer preview/loss, fused SSIM, SparseGaussianAdam, gradient-aware densification, multi-criteria pruning, opacity reset, and optional TensorRT/SPNet depth-completion wrapper.
- Native tracking package with signed-nanosecond trajectory and IMU primitives, LiDAR residual correction, visual residual comparison against mapper renders, GaussianMap snapshot subscription, odometry/path/optional TF publication, and deterministic probes.
- Dataset profile YAMLs derived from upstream Gaussian-LIC: FAST-LIVO, FAST-LIVO2, M2DGR, MCD, and R3LIVE.
- `gaussian_lic_offline` CLI for rosbag2 artifact extraction without launching ROS nodes.
- FAST-LIVO2 ROS1-to-ROS2 raw frontend conversion, ROS1 mapper-contract conversion, upstream ROS1 baseline runner, strict rosbag2 replay wrapper, baseline manifest/readiness gates, and combined reproduction reports.
- Current executable Bright substitute report with `metrics`, `trajectory`, `point_cloud`, and dedicated Torch Gaussian `gaussian_color` gates passing.

Still pending:

- Full Coco-LIC2-grade frontend/tracking algorithm: deskew, LIO/VIO factors, and sliding-window joint optimization.
- Runtime SPNet TensorRT `.engine` artifact and real depth-completion benchmark.
- Strict FAST-LIVO2 `CBD_Building_01` paper gate after the native ROS2 current run reaches PSNR/SSIM/LPIPS/Chamfer parity with the archived ROS1 upstream baseline.

## Progress Ledger

| Scope | Current status |
| --- | --- |
| ROS2 workspace, messages, launch, composable node, profiles, bag checks, artifact gates | Complete for the current porting surface |
| FAST-LIVO2 Bright substitute evidence chain | Complete and executable with `metrics`, `trajectory`, `point_cloud`, and `gaussian_color` passing |
| Strict `CBD_Building_01` paper-data gate | Official bag is local; ROS1 baseline is archived; ROS2 current collection/report is executable but still failing quality/Chamfer coverage gates |
| Paper-level Gaussian-LIC/Gaussian-LIC2 algorithm migration | Mapper CUDA/Torch backend is ported; full Coco-LIC2 tracking BA, SPNet engine, and strict metric parity remain |

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

- The native tracker has timestamp-safe trajectory/IMU primitives plus initial LiDAR/visual/Gaussian snapshot factors, but it is not yet the full continuous-time Coco-LIC2 sliding-window optimizer.
- LiDAR deskew, VIO residual Jacobians, IMU bias optimization, and joint BA remain to be ported.
- TensorRT/SPNet depth completion has a native optional wrapper, but no SPNet `.engine` artifact is checked in or benchmarked locally.
- Strict paper reproduction now has the local `CBD_Building_01` bag, ROS1 upstream baseline artifacts, strict CUDA current collection, and final-map render-pair extraction. The ROS2 current path remains below the strict paper gate until the native tracker and Gaussian optimization reach PSNR/SSIM/LPIPS/Chamfer parity.

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

`/gaussian_lic/rendered_image` currently defaults to:

```text
render_mode:=debug_cpu
```

That mode is a CPU projected-map preview for normal lightweight launches. The strict CBD reproduction entrypoint overrides this to `render_mode:=rasterizer`, `torch_gaussian_device:=cuda`, Torch pruning/densification, and final-map render evaluation. `rendered_image_mode` is kept only as a deprecated compatibility alias.

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

These profiles currently cover the native mapper surface: topic names, QoS, synchronization, upstream camera intrinsics, image size, depth-completion toggles, Gaussian initialization/map-growth parameters, upstream optimizer/loss/exposure parameter names, output topics, and active profile labels. Full real-dataset replay still needs the native Gaussian-LIC2 frontend/tracking port, real bag/topic adapters, and the CUDA rasterizer/optimizer port.

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
  --current-torch-max-foreground 800000
```

The strict script defaults to CUDA rasterizer mode, `torch_gaussian_device=cuda`,
`--current-torch-optimization-steps 100`, a slowed current playback rate of
`0.25`, and a 60 second post-playback settle window. In this path the step count
means up to 100 accumulated train-frame optimizer samples per keyframe, matching
the upstream Gaussian-LIC `optimize()`
scheduler instead of repeating 100 steps on only the newest frame. The slowed
playback is deliberate: the strict CUDA path is heavier than the live preview
path, so 1x rosbag2 replay can underfeed the final-map metric gate by dropping
unprocessed frames. The settle window lets the mapper consume queued frames
before `SaveMap` writes final-map render pairs.

That command converts the official ROS1 bag to a sqlite-backed ROS2
`frontend_raw` bag, audits replay timing, writes the ROS1 mapper-contract bag,
runs the upstream ROS1 baseline container, collects ROS2 current artifacts, and
emits strict readiness and reproduction reports. During report generation it also
fills `metrics.json::quality` from same-name render/GT image pairs. If
`${HOME}/.cache/gaussian_lic_ros2/quality-venv/bin/python` exists, it is used for
CPU LPIPS evaluation; otherwise set `QUALITY_PYTHON` to a Python environment with
`torch`, `torchvision`, `numpy`, and `Pillow`.

Latest local strict run, 2026-05-05:

- ROS1 baseline visual dump: 1186 render/GT pairs, 237 train and 949 novel frames.
- ROS2 current strict path: CUDA rasterizer, final-map evaluation, IMU-fallback
  world-frame point-cloud rotation enabled, 950 render/GT pairs, 190 train and
  760 novel frames, trajectory coverage 80.10%. The trajectory gate now passes.
- Current vs ROS1 quality remains below the paper gate: ROS2 novel PSNR 9.64 dB
  vs ROS1 12.70 dB, ROS2 novel SSIM 0.0418 vs ROS1 0.3644, and ROS2 novel
  LPIPS 0.714. The 64-pair strict render summary reports mean PSNR 10.03 dB and
  mean SSIM 0.281.
- Chamfer/point-cloud parity still fails, but the ROS1/ROS2 point-cloud frame
  semantics fix reduced the drift: centroid drift is now 2.43 m, bidirectional
  nearest RMSE is 0.378 m, and unmatched ratio is 45.69%.

So the strict chain now produces full-frame, same-cadence numbers and passes the
strict trajectory coverage gate, but the remaining blocker is algorithmic parity
in the native tracking/mapping path, not missing render pairs or a disabled CUDA
runtime.

## Release Roadmap

The public plan is release-driven:

```text
v0.1.0  Baseline & Infrastructure
v0.2.0  Gaussian-LIC2 ROS2 Native Frontend
v0.3.0  Gaussian Mapping Full Backend
v0.4.0  Strict FAST-LIVO2 Reproduction
```

Strict paper-data action: run `scripts/run_strict_cbd_pipeline.sh` from the local
`CBD_Building_01` bag, keep the ROS1 baseline archive fixed, and fix ROS2 current
report failures until the 5% gate passes.

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
