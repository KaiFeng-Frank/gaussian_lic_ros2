# Changelog

## Unreleased

- Pivoted the upstream porting plan to the now-public Gaussian-LIC2 code path in `APRIL-ZJU/Gaussian-LIC`; Coco-LIC is now documented as an optional legacy reference.
- Documented the released Gaussian-LIC2 upstream surface and confirmed the current public tree exposes mapper/rasterizer/optimizer/depth-completion code but no standalone native LIC2 frontend executable.
- Added `gaussian_lic_frontend/lic2_contract_adapter` to route raw ROS2 camera/LiDAR/IMU/pose topics into the Gaussian mapper contract.
- Extended the LIC2 contract adapter with frontend odometry/path outputs and optional TF.
- Added an optional ROS2 Livox CustomMsg-to-PointCloud2 bridge for raw driver packets before the LIC2 contract adapter.
- Added a `frontend_raw` bag contract and synthetic raw frontend bag recording mode for LIC2 adapter input validation.
- Added a `frontend_sensor_raw` bag contract for true LIC2 camera/LiDAR/IMU input without requiring a pose source.
- Extended CI replay smoke to replay raw frontend bags through the LIC2 adapter and report the `frontend_raw` contract.
- Added synthetic raw odometry input coverage for the LIC2 adapter and `frontend_raw` bag contract.
- Added IMU gyro orientation fallback in the LIC2 contract adapter so pose fallback can publish non-identity orientation from raw IMU when odometry is unavailable.
- Added a FAST-LIVO2 camera-LiDAR static transform profile in the LIC2 adapter so raw LiDAR points can be projected in the camera frame for image color and photometric Gaussian supervision.
- Added keyframe-gated Torch Gaussian initialization and incremental foreground insertion for the ROS2 mapping node.
- Added optional skybox tensor seeding, Gaussian map republishing after extension, and mapping latency/mean iteration status fields.
- Added a Torch GaussianMap CPU splat preview for `render_mode:=rasterizer` and smoke coverage for the rasterizer status mode.
- Added an optional Torch photometric Gaussian tensor optimizer that backpropagates keyframe image supervision into DC color and opacity tensors, with launch/profile/script controls and probe coverage.
- Added optional Torch Gaussian pruning by opacity and foreground count cap to keep ROS2 Gaussian maps bounded during long current runs.
- Added `scripts/collect_current_results.sh` to record ROS2 mapper outputs and write current reproduction artifacts for baseline-vs-current reports.
- Made `scripts/collect_current_results.sh` start mapper output recording before rosbag playback for deterministic full-sequence current artifacts.
- Confirmed the ROS1 upstream `gs_mapping` baseline target builds in Docker with OpenCV 4.10 CUDA fallback and TensorRT 8.6.1.6.
- Added `scripts/frontend_raw_to_ros1_mapper_contract.py` to convert official FAST-LIVO2 ROS2 frontend raw bags into a ROS1 mapper-contract bag for shared upstream/current replay.
- Extended the ROS1 mapper-contract converter with the same FAST-LIVO2 camera-LiDAR transform profile used by the ROS2 adapter.
- Extended the ROS1 mapper-contract converter with IMU gyro orientation pose fallback so curated baselines can mirror the ROS2 LIC2 adapter pose fallback.
- Added optional image-projected RGB fields in ROS1 mapper-contract point clouds so upstream `PointXYZRGB` baselines can share the ROS2 colorization surface.
- Added `scripts/run_upstream_baseline.sh` and `scripts/patch_upstream_baseline.py` to run the upstream ROS1 Gaussian-LIC/Gaussian-LIC2 mapper in Docker, archive baseline artifacts, and guard local TensorRT/libtorch runtime issues.
- Extended baseline and point-cloud artifact gates to read binary little-endian Gaussian PLY files from upstream.
- Extended point-cloud drift reports with an explicit Gaussian PLY `f_dc_0..2` RGB derivation option, enabling opt-in baseline-vs-current color checks for Gaussian artifacts.
- Produced a curated official FAST-LIVO2 `Bright_Screen_Wall_curated_8s` baseline-vs-current report with metrics, trajectory, and point-cloud gates passing while strict `CBD_Building_01` data remains quota-blocked.
- Added `scripts/run_curated_fastlivo2_report.sh` as a one-command curated FAST-LIVO2 reproduction chain for conversion, ROS2 current collection, ROS1 upstream baseline execution, and report generation.
- Extended the curated FAST-LIVO2 runner with a validated transformed Bright preset and opt-in Gaussian PLY color audit forwarding.
- Added a dedicated `gaussian_color` reproduction-report gate so the current Bright substitute proof chain combines full-sequence metrics, trajectory, geometry, and Torch Gaussian `f_dc_0..2` RGB drift checks in one report.
- Clarified README and roadmap progress accounting to distinguish the completed Bright substitute evidence chain from the still-incomplete paper-level algorithm port.
- Added P0 strict-porting groundwork: upstream/toolchain lock docs, LIC2 surface inventory, sm_120 CUDA/libtorch probe, and a strict CUDA/Torch build wrapper.
- Ported the upstream simple-knn CUDA `distCUDA2` operator into the strict CUDA build profile with a brute-force CPU reference probe.
- Ported the upstream SparseGaussianAdam visibility-masked CUDA update into the strict CUDA build profile with a CPU reference/performance probe.
- Ported the upstream fused-ssim CUDA forward/backward operator into the strict CUDA build profile with scalar and gradient reference probes.
- Ported the upstream CUDA Gaussian rasterizer forward/backward operator into the strict CUDA build profile with a nonblank render/depth and finite-gradient probe.
- Added an enforced ROS2 semantic contract for timestamp math, sensor QoS, executor choice, tf2 usage, and rosbag2 replay before the continuous-time frontend port proceeds.
- Switched mapper frame synchronization and LIC2 adapter IMU fallback from double-second stamp deltas to signed int64 nanosecond arithmetic.
- Made composable strict replay use the single-threaded ROS2 component container by default to preserve ROS1 callback ordering assumptions.
- Wired the strict CUDA rasterizer into the Torch Gaussian backend with L1 + fused-SSIM + optional depth supervision and visibility-masked SparseGaussianAdam updates for xyz, SH features, opacity, scaling, and rotation.
- Added `render_gaussian_map_from_camera` and wired `render_mode:=rasterizer` to publish CUDA rasterizer RGB output from the Torch Gaussian map in strict CUDA builds.
- Added `torch_cuda_backend_probe` to verify the public optimization/render path produces supervised rasterizer loss, initializes all SparseAdam state tensors, renders a nonblank preview, and updates all Gaussian parameter groups on CUDA.
- Added gradient-aware Torch Gaussian densification with clone/split topology updates, optimizer-state extension, multi-criteria pruning, and opacity reset scheduling.
- Extended `torch_cuda_backend_probe` to verify densification split growth, bounded pruning, and opacity reset execution on CUDA.
- Ported the upstream TensorRT/SPNet depth completion wrapper as an optional ROS2 mapping backend with lazy engine loading, `depth_completion_engine_path`, TensorRT build plumbing, and profile schema coverage.
- Documented the public LIC2-vs-v1 surface audit, including the absence of a released 2D Gaussian primitive or skybox patch in `cd4c122`.
- Added the `gaussian_lic_tracking` ROS2 package with a signed-nanosecond cubic B-spline trajectory manager foundation and constant-velocity/negative-time probe.
- Added a signed-nanosecond IMU propagation foundation with deterministic gyro/acceleration probe coverage.
- Added a native ROS2 `tracking_node` surface that forwards raw camera/LiDAR/depth topics to mapper inputs and publishes IMU-propagated odometry, path, pose, and optional TF with explicit QoS.
- Added a single-node ROS2 launch entrypoint for the native tracking surface.
- Added a Coco-LIC frontend inventory for the remaining native tracker port.
- Added upstream Gaussian-LIC backend parameter coverage for image size, depth completion, optimizer, loss, exposure, Gaussian extension, and skybox control.
- Implemented the dependency-gated ROS1 `.bag` to rosbag2 converter backend using `rosbags`.
- Added `gaussian_lic_bag_check` to validate rosbag2 mapper-contract topics before replay.
- Extended `gaussian_lic_bag_check` to inspect ROS1 `.bag` metadata before conversion.
- Added image-projection color fallback for PointCloud2 mapper inputs that do not carry RGB fields.
- Added intensity/reflectivity grayscale fallback after image projection misses so uncolored LiDAR points do not seed white Gaussian DC values.
- Added synthetic smoke coverage for image-projected point coloring from uncolored PointCloud2 input.
- Added `require_depth_topic` so the mapper can run with sparse point-projected depth when depth images are unavailable.
- Added a `mapper_minimal` bag contract mode for point/pose/image replay bags.
- Added `smoke_test.sh --minimal-inputs` for replaying point/pose/image-only mapper bags.
- Preserved PointCloud2 RGB fields in `gaussian_lic_offline` debug PLY artifacts.
- Added offline artifact metrics for topic rates, trajectory path length, point-cloud bounds, and color coverage.
- Added direct ROS1 `.bag` support to `gaussian_lic_offline` for dependency-gated baseline artifact extraction.
- Added `scripts/trajectory_compare.py` for timestamp-associated TUM trajectory drift gates.
- Added `scripts/pointcloud_compare.py` for ASCII PLY map drift gates.
- Added `scripts/baseline_manifest.py` for ROS1 baseline artifact validation and fingerprinting.
- Added `scripts/fetch_fastlivo2_sequence.py` to discover and download official FAST-LIVO2 Google Drive sequence bags.
- Added `scripts/fastlivo2_ros1_to_frontend_raw.py` to convert official FAST-LIVO2 ROS1 bags into ROS2 camera/LiDAR/IMU frontend input bags.
- Added `scripts/baseline_readiness.py` to gate FAST-LIVO2 data, ROS1 baseline artifacts, and ROS2 current artifacts before reproduction comparison.
- Added `scripts/upstream_baseline_build_attempt.sh` to reproduce the ROS1 Noetic Gaussian-LIC/Gaussian-LIC2 baseline build attempt, support an explicit OpenCV build directory, and capture local environment blockers.
- Added `scripts/reproduction_report.py` to aggregate baseline manifest, metrics, trajectory, and PLY drift gates.
- Added a Python-only GitHub Actions artifact-gates job using `scripts/verify_artifact_gates.sh`.
- Uploaded artifact-gate JSON/Markdown reports from CI as `artifact-gate-reports`.
- Added a Jazzy CI synthetic rosbag2 replay smoke step with uploaded smoke logs.
- Made synthetic bag, smoke, and workspace verification helpers respect `ROS_DISTRO` with a Jazzy default.
- Added `scripts/build_ros2.sh` and kept `scripts/build_jazzy.sh` as a compatibility wrapper.
- Added `scripts/ci_replay_smoke.sh` so CI and local generated-bag replay smoke share one entrypoint.
- Added CI replay smoke artifact export for bag contracts and offline extraction outputs.
- Uploaded per-distro CI `log/` directories for colcon and smoke-test diagnostics.
- Added Markdown job summaries for artifact gates and CI replay smoke reports.

## v0.1.0-m1-infra - 2026-05-03

M1 infrastructure checkpoint for the Gaussian-LIC ROS2 port.

Completed:

- Added release-driven roadmap for M1-M4.
- Added `/gaussian_lic/status` schema documentation and extended `MappingStatus.msg`.
- Added canonical `render_mode` parameter with deprecated `rendered_image_mode` compatibility.
- Added ROS2 native mapper smoke path for synthetic live input and rosbag2 replay.
- Added dataset profile YAMLs for FAST-LIVO, FAST-LIVO2, M2DGR, MCD, and R3LIVE.
- Added `gaussian_lic_offline` rosbag2 artifact extraction CLI.
- Added `scripts/perf_regression.py` for metrics regression checks.
- Added `scripts/convert_ros1_bag_to_rosbag2.py` as the dependency-gated ROS1 bag conversion entrypoint.
- Added Jazzy/Humble build-only GitHub Actions skeleton.
- Added calibration schema, common errors, release milestones, and status schema docs.

Validated locally:

```bash
./scripts/build_jazzy.sh
./scripts/verify_workspace.sh --skip-build --full-profiles
GAUSSIAN_LIC_ENABLE_TORCH=ON ./scripts/build_jazzy.sh --packages-select gaussian_lic_mapping
ros2 run gaussian_lic_mapping torch_backend_probe
./scripts/smoke_test.sh --bag bags/synthetic_gs_demo --torch --tf
./scripts/smoke_test.sh --bag bags/synthetic_gs_demo --composition --tf
```

Known limitations:

- Native Gaussian-LIC2 frontend/tracking is not ported yet.
- Full Gaussian rasterizer/optimizer/densification/pruning are not ported yet.
- `/gaussian_lic/rendered_image` currently uses `render_mode:=debug_cpu`, not the paper rasterizer.
- `/home/frank/data/fast_livo/` is empty on the current machine; strict FAST-LIVO2 baseline reproduction is blocked on data.
- TensorRT is not installed locally; depth completion remains optional.

Next concrete action:

```bash
docker pull ros:noetic-ros-base-focal
```

Then run the ROS1 upstream baseline on FAST-LIVO2 and archive outputs under:

```text
baseline/fastlivo2/<sequence>/
```
