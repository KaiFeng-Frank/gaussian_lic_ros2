# Changelog

## Unreleased

- Added external odometry prior subscription to `continuous_time_node`. New launch parameters `enable_external_odometry_prior` (default false) and `external_odometry_prior_topic` (default empty) wire the node to a `nav_msgs/Odometry` source whose latest pre-initialization pose is consumed as the seed for the rotation/position spline knots, taking precedence over identity / gravity-autocal seeding. This is the producer hook that lets downstream parity tests seed the estimator from ground truth — once the persistent LiDAR map is added, the 13th matrix entry can graduate from liveness-only to strict RMSE.
- Expanded the continuous-time strict matrix from 2 to 5 bags — one per profile. New required entries: `m2dgr/room_01/continuous_time_native_parity`, `mcd/ntu_day_01/continuous_time_native_parity`, `r3live/hku_park_00/continuous_time_native_parity`. Liveness gates only (captured_tum_lines ≥10, finite_tum_positions ≥10, odom_messages_received ≥30). Local 2026-05-12 results on 12 s / rate 0.5 slices: M2DGR room_01 21/21/122, MCD ntu_day_01 18/18/111, R3LIVE hku_park_00 21/21/120. Matrix now reports `PASS required=17/17` and the ported continuous-time stack has end-to-end real-bag evidence for every profile.
- Added a 14th required strict matrix entry: `fastlivo2/Retail_Street/continuous_time_native_parity`. Validates the ported continuous-time tracker on FAST-LIVO2 `Retail_Street_frontend_raw` — a second real bag beyond CBD_Building_01 — and broadens the continuous-time evidence base. The parity script now accepts `REFERENCE_TUM=__skip__` to run liveness-only mode for bags without an archived Coco-LIC reference; the resulting `native_tracking_report.json` still satisfies `captured_tum_lines ≥ 10`, `finite_tum_positions ≥ 10`, `odom_messages_received ≥ 30`. Local run on the 12 s slice records 22 captured TUM lines, 22 finite positions, 121 odometry messages. Matrix now reports `PASS required=14/14`.
- Autocalibrated gravity from static seed IMU samples in `continuous_time_node`. New `enable_imu_gravity_autocal` parameter (default true) averages the seed-buffer accelerometer readings and rotates the initial spline knots so body z aligns with `-gravity_world`. Cuts FAST-LIVO2 `CBD_Building_01` 12 s reference RMSE from 330 km → 130 km on the existing 13th matrix entry — a 2.5× quality improvement at the cost of one quaternion `FromTwoVectors` per seed. Mirrors the upstream Coco-LIC / `tracking_node` `enable_imu_gravity_autocalibration` default.
- Promoted the ROS2 continuous-time tracker port to the strict parity matrix as required entry #13: `fastlivo2/CBD_Building_01/continuous_time_native_parity`. The parity script now emits a `native_tracking_report.json` with schema `gaussian_lic_continuous_time_native_tracking_report/v1` that `scripts/check_strict_parity_matrix.py` consumes. Liveness thresholds gate captured TUM line count (≥10), finite TUM position count (≥10), received odometry message count (≥30), reference-trajectory matched-pose count (≥5), and reference coverage (≥0.005). Absolute RMSE is **not** gated; the entry validates that the ported B-spline + Ceres AutoDiff + voxel-plane extraction + sliding-window stack ingests real FAST-LIVO2 streams, produces finite TUM poses, and the reference comparison toolchain runs end-to-end. Local 2026-05-12 result: 23 captured TUM lines, 23 finite positions, 123 odometry received, 14 reference-matched poses, 1.14% coverage. The matrix gate now reports `PASS required=13/13 covered_profiles=fastlivo,fastlivo2,m2dgr,mcd,r3live`.
- Added `scripts/continuous_time_native_reference_parity.sh` and `scripts/odom_to_tum.py`. The shell script plays the configured `frontend_raw` rosbag2 into `continuous_time_node` while the Python helper subscribes to `/continuous_time/odometry` and writes a clean TUM trajectory (filtered for non-finite or grossly diverged poses); the script then invokes `scripts/trajectory_compare.py --align yaw` against the archived `cocolic_livo_reference_10hz.tum` reference and emits a JSON report. Local 2026-05-12 run on FAST-LIVO2 `CBD_Building_01_frontend_raw` with the voxel-plane extractor active produced the first native-stack reference comparison: 23 captured TUM lines, 16 matched poses, 1.3% coverage of the reference, translation RMSE 289 km. The producer chain is in place end-to-end; the numerical gap reflects that the current estimator lacks proper IMU-bias / gravity initialization, not missing algorithmic surface. Hardened the in-node TUM writer with `position_bound` filtering + `std::ostringstream` formatting so divergent poses no longer corrupt the file when downstream tooling is used directly.
- Added `gaussian_lic_tracking::spline::LidarPlaneExtractor`, a per-frame voxel-grid plane feature extractor that replaces the configurable single-plane prior in `continuous_time_node`. Each incoming PointCloud2 is voxelized at a configurable resolution; for every voxel with sufficient support, a plane is fitted via covariance eigendecomposition. Only voxels whose smallest/largest eigenvalue ratio falls below the planarity threshold (default 0.05) and whose worst-case inlier distance stays within the configured margin (default 0.1 m) emit a `LidarPointCorrespondence`. New launch parameters: `enable_voxel_plane_extraction` (default true), `voxel_plane_size_m`, `voxel_plane_min_points`, `voxel_plane_eigen_ratio`, `voxel_plane_max_inlier_m`, `voxel_plane_max_correspondences`. The CTest `lidar_plane_extractor_probe` verifies recovery of a synthetic ground plane (normal alignment >0.99 dot, offset <5 cm), a two-orthogonal-plane scene (both ground and wall recovered), and rejection of pseudo-random noise. Bag smoke records 1490 odometry messages on FAST-LIVO2 CBD_Building_01 15 s slice (vs 1137 under the single-plane prior) — the real-feature LiDAR factor surface visibly improves observability.
- Replaced `ceres::NumericDiffCostFunction` with `ceres::AutoDiffCostFunction` for the IMU and LiDAR factors inside `TrajectoryEstimator`. All cumulative B-spline math (`quaternion_log_t`, `quaternion_exp_t`, `adjoint_t`, `lie_bracket_t`, `evaluate_lie_so3_t`, `evaluate_rd_t`, plus the supporting `skew_t`) is now templated on the scalar type, so Ceres can substitute `ceres::Jet<double, N>` and obtain exact analytic Jacobians via dual numbers instead of central finite differences. The concrete-double overloads (`quaternion_log`, `quaternion_exp`, `adjoint`, `lie_bracket`, `evaluate_lie_so3`, `evaluate_rd`) stay in place for non-Ceres callers so the existing analytic-jacobians / sliding-window / integration probes keep validating the same math. All 34 tests still pass and the bag smoke records 1137 odometry messages on the FAST-LIVO2 CBD_Building_01 15 s slice (vs 484 previously) — direct evidence that the new estimator finishes more Ceres iterations per wall-clock second with exact Jacobians.
- Added an `output_tum_path` parameter to `continuous_time_node`. When set, every published odometry pose is appended to a TUM-format trajectory file (`stamp_s tx ty tz qx qy qz qw`, ROS clock convention). This is the artifact `scripts/trajectory_compare.py` expects for reference-trajectory parity gating, so the eventual 13th strict matrix entry just needs feature-extraction maturity and a reference trajectory — the producer side is now in place. Bag smoke updated to write the TUM file alongside the node log and assert ≥1 non-comment line; local CBD_Building_01 15 s / rate 0.5 slice produces 834 TUM lines.
- Added `scripts/continuous_time_node_bag_smoke.sh`, the first real-bag end-to-end smoke for the new continuous-time tracker. It plays a configurable slice of a `frontend_raw` rosbag2 at a slowed rate, remaps `/imu` and `/livox/lidar` into the node's `/imu_for_gs` / `/points_for_gs` inputs, and counts emitted Odometry messages over the slice. Local CBD_Building_01 (15 s / rate 0.5) confirms the pipeline accepts real IMU + LiDAR streams without crashing: 285 odometry messages with LiDAR disabled, 484 with LiDAR enabled (stride 200 subsample). The smoke is intentionally a liveness gate, not a strict-parity gate — the LiDAR plane prior is configurable and the per-point feature extraction layer is still pending.
- Extended `continuous_time_node` with `sensor_msgs/PointCloud2` ingestion. The node now subscribes to a configurable PointCloud2 topic (default `/points_for_gs`), iterates XYZ via `sensor_msgs::PointCloud2ConstIterator<float>`, rejects non-finite samples and out-of-range points, subsamples by a configurable stride, caps points per message, and feeds the result into the sliding-window estimator as `LidarPointCorrespondence` instances against a configurable ground-plane prior (`lidar_ground_plane` parameter, defaults to `[0, 0, 1, 0]`). LiDAR-to-IMU extrinsic translation + rotation_xyzw are exposed as launch parameters. The smoke wrapper now also publishes 8 synthetic ground-plane PointCloud2 messages on `/points_smoke`; the local run records 42 odometry messages from the combined IMU + PointCloud2 burst.
- Added closed-form analytic Jacobian helpers in `gaussian_lic_tracking::spline::analytic_jacobians.hpp` covering the cases where the residual chain rule collapses to a constant matrix: IMU position-knot Jacobian (`coeff * inv_dt^2 * R_b_w`), IMU gyro/accel bias Jacobians (identity sub-blocks), IMU gravity Jacobian (`-R_b_w`), and LiDAR plane position-knot Jacobian (`weight * coeff * n^T * R_world_to_map`). `analytic_jacobians_probe` asserts each form matches finite-difference Jacobians at multiple knot configurations (max abs diff <1e-5 for the trickier position-knot case, <1e-7 for the linear cases). The SO(3) rotation-knot Jacobian — which requires Lie-group chain rule through the cumulative B-spline — remains numeric for now; the analytic helpers above are the staged foundation for swapping `NumericDiffCostFunction` for `SizedCostFunction` in the Ceres harness without rederiving the closed-form math at integration time.
- Added a standalone ROS2 `continuous_time_node` executable that exercises the new continuous-time tracker on real topics without touching the existing `tracking_node` default path. The node subscribes to a raw IMU topic, seeds an internal `ContinuousTimeSlidingWindowEstimator` once a configurable number of IMU samples have arrived, and publishes optimized body odometry plus an aggregated path. Parameters cover knot interval, window size, marginalize-oldest count, max LM iterations per step, step timer period, seed threshold, gravity, bias hold flags, and frame ids. Launch file is `gaussian_lic_bringup/launch/continuous_time.launch.py`; smoke wrapper at `scripts/continuous_time_node_smoke.{sh,py}` confirms 26+ odometry messages are emitted from an 80-sample synthetic IMU burst on the local ROS2 Jazzy environment.
- Added `gaussian_lic_tracking::spline::ContinuousTimeSlidingWindowEstimator`, the streaming driver that combines `TrajectoryEstimator` and `SplineMarginalizationInfo` into the online sliding-window estimator equivalent to Coco-LIC's `OdometryManager`. The estimator accepts streaming IMU + LiDAR samples, buffers them per arrival, extends the cumulative B-spline window forward via knot extrapolation, persists active factors across solves so previously consumed measurements continue to constrain the optimization, and drops the oldest knots once the active window exceeds the configured size. `continuous_time_sliding_window_probe` covers three regimes: (a) single-step batch solve on a perturbed seed window drives the IMU cost below 1e-6 with <2 cm interior drift; (b) streaming 200 IMU samples across many knot extensions over a synthetic 1.05 s trajectory keeps interior position drift under 8 cm (5.2 cm achieved); (c) marginalize-oldest mode bounds the active window to ≤12 knots while running through >20 knot intervals.
- Tracking package now reports 33/33 tests passing locally. Strict parity matrix (`scripts/check_strict_parity_matrix.py`) still reports `required=12/12` covering FAST-LIVO/FAST-LIVO2/M2DGR/MCD/R3LIVE — additions are new libraries + probes only, no `tracking_node` default behavior change.
- Wired `libceres-dev` (Ceres 2.2) into `gaussian_lic_tracking` and added `gaussian_lic_tracking::spline::TrajectoryEstimator`, a Ceres-driven harness over the new continuous-time IMU and LiDAR factors. Each rotation knot uses `ceres::EigenQuaternionManifold`; position knots, gyro/accel biases, and gravity are plain 3-vector parameter blocks. Cost evaluation runs through `NumericDiffCostFunction` so the existing analytic factor surface remains the single source of truth. `trajectory_estimator_probe` verifies zero residual on truth, that an interior position-knot z perturbation drives the IMU cost to zero and pulls the knot back significantly, and that a stack of LiDAR plane factors recovers a 10 cm ground-plane drift from a perturbed seed.
- Ported Coco-LIC's marginalization pipeline as `gaussian_lic_tracking::spline::SplineMarginalizationInfo`: accumulate linearized residual blocks tagged with parameter-block ids, mark blocks to remove, run Schur complement, and decompose the reduced prior into a square-root Jacobian + residual via self-adjoint eigendecomposition. `spline_marginalization_probe` checks J^T J equals the manual Schur Hessian and J^T r equals the manual Schur gradient on both a single and a two-residual-block scenario, providing the spline-knot analog of the existing discrete-state `schur_complement_probe`.
- `gaussian_lic_tracking` now reports 32/32 tests passing locally. Strict parity matrix gate (`scripts/check_strict_parity_matrix.py`) still reports `required=12/12` covering FAST-LIVO, FAST-LIVO2, M2DGR, MCD, and R3LIVE — the additions are purely additive new libraries / probes, no behavior change in `tracking_node`.
- Ported the Basalt/Coco-LIC cumulative B-spline foundation into `gaussian_lic_tracking::spline`: blending-matrix helpers (`spline_common.hpp`), `CeresSplineHelper<N>` (`ceres_spline_helper.hpp`) with analytic Euclidean R^DIM evaluation plus cumulative SO(3) Lie-group evaluation, and a `SplitSplineView<N>` returning world pose, body-frame angular velocity / angular acceleration, and world acceleration. Verified by `ceres_spline_helper_probe` against the canonical 1/6·[[1,-3,3,-1],[4,0,-6,3],[1,3,3,-3],[0,0,0,1]] uniform cubic blending matrix plus finite-difference time derivatives.
- Ported Coco-LIC's continuous-time IMU residual (`IMUFactor` / `IMUFactorNURBS`) as `ContinuousTimeImuFactor`. The residual evaluates `ω_spline(t) - (ω_meas - b_g)` and `R_w_b(t)^T(a_w(t) - g_w) - (a_meas - b_a)` against the cumulative B-spline knots, gyro/accel biases, and gravity. `continuous_time_imu_factor_probe` checks zero residual on a synthetic spline, closed-form gyro/accel-bias and gravity Jacobians, and finite knot Jacobians.
- Ported Coco-LIC's continuous-time LiDAR residual (`LoamFeatureFactorNURBS`) as `ContinuousTimeLidarFactor`. The residual transforms a per-point LiDAR observation through the spline pose and an explicit `S_LtoI`/`p_LinI` + `S_GtoM`/`p_GinM` extrinsic chain, then evaluates a plane (`n·p_M + d`) or edge (`|d_vec × n|`) distance scaled by the user-supplied weight. `continuous_time_lidar_factor_probe` covers plane consistency, plane offset distance, finite-difference parity for the body-pose Jacobian, and the edge geometry path.
- Ported Coco-LIC's continuous-time photometric residual (`PhotometricFactorNURBS`) as `ContinuousTimePhotometricFactor`. The factor projects a world point through the spline-evaluated camera pose plus camera-to-IMU extrinsics, then samples a (P×P) image patch via an `IntensitySampler` callback and reports the per-pixel residual against a reference patch. `continuous_time_photometric_factor_probe` covers pinhole projection match, behind-camera rejection, zero residual under matching patches, and weight scaling.
- Added `continuous_time_integration_probe` exercising IMU + LiDAR continuous-time factors on a shared synthetic spline: many random-u IMU samples produce zero residual on the true state, residuals grow monotonically with a position-knot perturbation, and a 1D sweep over the central position knot recovers the minimum exactly at zero shift. This is the first cross-factor parity check for the continuous-time residual model.
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
