# Gaussian-LIC ROS2

Native ROS2 engineering port for [Gaussian-LIC/Gaussian-LIC2](https://github.com/APRIL-ZJU/Gaussian-LIC).

The long-term target is a paper-level ROS2 reproduction of LiDAR-Inertial-Camera Gaussian Splatting SLAM: native ROS2 tracking, native Gaussian mapping, rosbag2 replay, offline reproducibility tooling, and strict comparison against the ROS1 upstream baseline.

This repository is **not a ROS1 bridge wrapper**. It is a clean ROS2 workspace that keeps algorithm code isolated from middleware glue so the tracking and mapping backends can be ported incrementally without locking the project into ROS1-era APIs.

## Current Release State

This repository is now an executable ROS2 porting checkpoint for the public Gaussian-LIC/Gaussian-LIC2 code path. It has native ROS2 message, launch, adapter, mapper, CUDA/Torch Gaussian backend plumbing, native tracking factors, artifact extraction, strict replay/readiness tooling, an official FAST-LIVO2 Bright substitute proof chain, local strict FAST-LIVO2 `CBD_Building_01`, MCD `ntu_day_01`, and R3LIVE `hku_park_00` reproduction chains against archived ROS1 upstream baselines, plus trusted upstream/native reference trajectory gates for the FAST-LIVO2 `CBD_Building_01` frontend and the new continuous-time tracker producer chain. The archived strict mapper-contract/CUDA reports are green for trajectory, PSNR/SSIM/LPIPS, GT-associated render-pair, and point-cloud gates; the full local strict evidence matrix now reports `required=17/17`.

It is **not a claim of universal super-paper performance on every possible sequence**: the five continuous-time matrix entries are currently real-bag liveness/producer-chain gates, not RMSE-gated paper parity. The current release claim is narrower and executable: the required local strict parity matrix passes, including mapper-contract/CUDA parity, the FAST-LIVO2 `CBD_Building_01` native frontend comparison against an upstream Coco-LIC-generated reference, and continuous-time tracker evidence on FAST-LIVO2, M2DGR, MCD, and R3LIVE bags.

Available now:

- Native ROS2 packages for messages, launch/config, mapping scaffold, and test tools.
- `gaussian_lic_frontend/lic2_contract_adapter` for routing raw ROS2 camera/LiDAR/IMU/pose topics into the mapper contract.
- Optional Livox `CustomMsg` to `PointCloud2` bridge for raw Livox driver packets.
- ROS2 synchronization/conversion for the Gaussian-LIC mapper input contract:
  `/points_for_gs`, `/pose_for_gs`, `/image_for_gs`, `/camera_info_for_gs`, `/depth_for_gs`, `/imu_for_gs`.
- Odometry, path, TF, debug map cloud, rendered debug preview, status, GaussianArray, and SaveMap interfaces.
- Optional libtorch/CUDA path for keyframe-gated Gaussian tensor initialization, skybox seeding, incremental foreground insertion, CUDA rasterizer preview/loss, fused SSIM, SparseGaussianAdam, gradient-aware densification, multi-criteria pruning, opacity reset, and optional TensorRT/SPNet depth-completion wrapper.
- Native tracking package with signed-nanosecond trajectory and IMU primitives, invalid ROS2 builtin-stamp nanosecond rejection at mapper/frontend boundaries, non-monotonic tracking stream stamp rejection with status counters, non-finite IMU measurement rejection before propagation/preintegration, LiDAR malformed-frame/invalid-point/invalid-point-time/out-of-range point-time status counters, CameraInfo/image/depth/rendered invalid-frame counters, mapper per-input QoS overrides, frontend-adapter per-stream input/output QoS controls, and native tracking per-stream raw-input/mapper-output QoS controls for image/point cloud/pose/CameraInfo/depth/IMU/odometry streams, finite CameraInfo intrinsic gating before mapper/tracker state updates, finite LiDAR-to-IMU/camera-to-IMU extrinsic, IMU gravity, LiDAR keyframe-threshold, tracking QoS depth, visual/SE3 photometric gate, LiDAR-factor, cache-size, and sliding-window BA-bound validation before tracker startup, signed-nanosecond image/LiDAR/IMU status gates, bounded best-effort tracking sensor QoS exposed through launch/status gates, executor callback serialization for ROS1-style estimator mutation order, ROS2-configurable LiDAR-to-IMU and camera-to-IMU extrinsics, signed-nanosecond visual-factor freshness gating with bounded nearest-stamp rendered/depth frame caches, IMU history interpolation, raw-sample IMU preintegration/reintegration with finite-input validation, SO(3) log-map rotation residuals, AutoDiff start-bias reintegration sensitivity, optional full 9x9 IMU residual sqrt-information whitening, per-block rotation/velocity/position residual weights, optimized pose/velocity/bias feedback into odometry, the continuous-time trajectory-control cache, safe IMU re-anchoring, and finite optimized-state/control-pose rejection before odometry/IMU/B-spline feedback, IMU bias continuity and marginalization-prior anchoring inside the optional sliding window, bias observability status/probes, reusable normal-equation linearization, factor-agnostic BA solve triggering when non-IMU LiDAR/visual factors are available, bounded LM state increments, Schur-complement retained-window dense priors with rank/singular-value health, timestamp-safe cubic B-spline position/velocity plus SO(3) cubic orientation trajectory queries for deskew fallback, default-enabled three-state continuous-time trajectory smoothness factors plus pose/state/dense/SE3 priors with SO(3) log-map residuals and closed-form left-Jacobian inverse rotation blocks, per-point LiDAR deskew with invalid-pose and scan-time-bound rejection, bounded 6-DoF LiDAR residual correction with finite-sample/pose validation and Huber-weighted Kabsch centroids/covariance, corrected-pose LiDAR/Gaussian correspondence construction before BA ingestion, spatial-indexed robust direct LiDAR/Gaussian-map point-to-point window factors with `(0, 1]` correspondence-weight validation, LiDAR point-to-plane window factors with residual and local-planarity confidence weighting, LiDAR correspondence confidence and spatial-index health status, sliding-window BA optimization timing and numeric-Jacobian fallback status, Huber-robust visual-alignment and SE3 photometric window factors, analytic geometric Jacobians for point-to-point, point-to-plane, visual-alignment, and full current IMU preintegration factors, visual residual/direct subpixel image-alignment comparison against mapper renders, runtime-gated 2-DoF photometric translation linearization, analytic SE3 camera photometric pixel Jacobians, multi-sample normal equations, sparse LiDAR-projected depth dilation plus valid-depth SE3 sampling for mapper-rendered feedback, optional visual-alignment window factors, chunk-complete GaussianMap snapshot caching, odometry/path/status/optional TF publication, and deterministic probes.
- Dataset profile YAMLs derived from upstream Gaussian-LIC: FAST-LIVO, FAST-LIVO2, M2DGR, MCD, and R3LIVE.
- `gaussian_lic_offline` CLI for rosbag2 artifact extraction without launching ROS nodes.
- Profile-aware ROS1-to-ROS2 raw frontend conversion for FAST-LIVO, FAST-LIVO2, M2DGR, MCD, and R3LIVE; ROS1 mapper-contract conversion, upstream ROS1 baseline runner, strict rosbag2 replay wrapper, baseline manifest/readiness gates, and combined reproduction reports.
- Current executable Bright substitute report with `metrics`, `trajectory`, `point_cloud`, and dedicated Torch Gaussian `gaussian_color` gates passing.
- Strict FAST-LIVO2 `CBD_Building_01` artifact/readiness pipeline with trajectory, PSNR/SSIM/LPIPS, render-pair, and point-cloud gates passing for the mapper-contract/CUDA path.
- `scripts/run_strict_parity_queue.sh` turns the remaining data/evidence backlog into a resumable full-profile queue: it reuses or creates frontend-raw bags, emits ROS1 mapper-contract bags, runs the upstream baseline with the matching profile config, collects ROS2 CUDA current artifacts, and writes strict readiness/reproduction reports for FAST-LIVO, FAST-LIVO2, M2DGR, MCD, and R3LIVE targets.
- `scripts/check_strict_parity_matrix.py` for the final full-dataset release gate. It now reports `required=17/17`: FAST-LIVO2 mapper strict parity, FAST-LIVO hku1/hku2/Visual_Challenge/LiDAR_Degenerate mapper strict parity, M2DGR room_01/room_02/room_03 mapper strict parity, MCD `ntu_day_01` mapper strict parity, R3LIVE `hku_park_00` mapper strict parity, CBD native BA runtime health, CBD native reference trajectory parity, and five continuous-time native tracker producer-chain gates are all archived and passing.
- `scripts/audit_strict_data_inputs.py` for the data-side gate. The current local audit is archived at `docs/strict_data_status.md` / `docs/strict_data_status.json`; it now separates raw/frontend data, ROS1 baseline artifacts, ROS2 current artifacts, and native reference trajectories. As of 2026-05-12 all required profiles pass those input checks and the strict evidence matrix is `17/17`.
- R3LIVE `hku_park_00` can be converted to ROS2 frontend-raw, passes the native sensor-only tracking health gate, and now has a passing full-sequence mapper-contract/CUDA strict parity report.
- FAST-LIVO2 `Retail_Street` is now fetched from the official Google Drive index, converted to ROS2 frontend-raw, has an archived ROS1 upstream mapper baseline with 1,354 poses, 704,826 PLY vertices, and 1,354 renders, and passes a 60s native scan-order deskew tracking health gate; this remains additional runtime/data coverage, not an RMSE-gated native tracker parity claim.
- Generic Google Drive file fetching is available through `scripts/fetch_google_drive_file.py`; MCD `ntu_day_01` now has local `d435i`, `mid70`, `vn100`, ground-truth TUM, frontend-raw conversion, archived ROS1 baseline artifacts, ROS2 current artifacts, and a passing strict mapper-contract/CUDA report.
- SPNet TensorRT engine generation for the local `sm_120` GPU via TensorRT 10.9, with the generated FP16 engine kept outside git.
- Native tracking probes are registered with CTest, so `colcon test --packages-select gaussian_lic_tracking` runs trajectory, IMU, LiDAR, sliding-window, bias observability, geometric Jacobian, Gaussian snapshot, trajectory smoothness, SE3 photometric Jacobian/factor, and visual checks automatically. `scripts/tracking_smoke_test.sh` also verifies the launch path through `/gaussian_lic/frontend/status`, including signed-nanosecond time status, tracking QoS, executor callback serialization, auto-start IMU preintegration re-integration semantics, cumulative IMU-factor/preintegration and visual/SE3 factor evidence that survives window marginalization, dense-prior health with stamp/reference validation, Schur/fallback marginalization health, same-stamp prior plus same-source IMU/LiDAR/visual/SE3/smoothness replacement counters, orphan-factor health, active-window state-cadence health, bias observability, accepted BA feedback stamp/delta/limit health, optimized IMU re-anchors, last-consumed IMU preintegration sample/dt/span health with time-gap skip gating, B-spline trajectory control poses, LiDAR correspondence confidence and spatial-index health, sliding-window optimization timing, zero global numeric-Jacobian fallback, nonzero trajectory smoothness factors, accepted/rejected/limited BA step health, linearization/linear-solve failure counters, factor skip counters, visual alignment, bounded photometric sample weights, photometric linearization status, rendered/depth cache diagnostics, and SE3 photometric sample quality/rejection counters.
- CI semantic checks keep the GitHub Actions build matrix Jazzy-only; adding ROS2 Humble to `.github/workflows/ci.yaml` is treated as a contract violation.

Still pending:

- RMSE-gated continuous-time native tracker parity on the full long-window datasets; the data and reference-trajectory archive gate itself is now complete.
- Latest 2026-05-13 Coco-LIC2 tracker work: LiDAR residuals now carry Coco-LIC-style feature scale through both residuals and weighted Huber gates, limited sliding-window updates feed accepted IMU bias/gravity state instead of freezing it, scan-to-scan velocity/angle clamps now bound the stored target pose as well as the prior, a Coco-LIC `fixed_control_point_index` gauge-anchor mode is implemented as an ablation parameter, and the continuous-time estimator now has native 3D LiDAR point-to-point map residuals for persistent/Gaussian map anchors. The mapper-to-tracker reverse channel lets `continuous_time_node` subscribe to chunked `GaussianArray` snapshots, build voxel-indexed nearest-neighbor Gaussian anchors, and inject robust LiDAR-to-Gaussian point factors into the online window through parity-script controls; `run_native_tracking_bag_report.sh --enable-gaussian-map-feedback --require-gaussian-snapshot` starts mapper Torch Gaussian init/extend and gates complete snapshot delivery. The Gaussian map QoS depth is parameterized instead of hard-coded to 1, `GaussianSnapshot` double-buffers in-flight chunk sequences, strict feedback replay now slows to `--rate 0.5` unless explicitly overridden, and the current 75s CBD visual+Gaussian feedback run passes first-align reference tracking gates with 685 matches, 55.65% coverage, 1.396 m RMSE, 1.033 m mean error, 3.278 m max error, and 15.5% path drift. This closes the local CBD native reference checkpoint; cross-dataset long-window RMSE-gated continuous-time parity remains the next hardening target.
- Continuous-time tracker quality hardening: the producer chain is now cross-profile and matrix-gated, persistent world-frame plane constraints plus online solve-step rejection are in tree, but the newest continuous-time entries are still liveness-gated while RMSE/paper-grade parity remains the next numerical target.
- Continued production hardening of the native tracker under faster-than-strict replay; strict reference parity still uses controlled rosbag2 replay to preserve ROS1-style sequential timing semantics.

## Progress Ledger

| Scope | Current status |
| --- | --- |
| ROS2 workspace, messages, launch, composable node, profiles, bag checks, artifact gates | Complete for the current porting surface |
| FAST-LIVO2 Bright substitute evidence chain | Complete and executable with `metrics`, `trajectory`, `point_cloud`, and `gaussian_color` passing |
| Strict `CBD_Building_01` paper-data gate | Official bag is local; ROS1 baseline is archived; latest ROS2 mapper-contract/CUDA strict report passes `reproduction_report.py --strict` |
| Paper-level Gaussian-LIC/Gaussian-LIC2 algorithm migration | Mapper CUDA/Torch backend, executable strict mapper-contract chain, local SPNet TensorRT engine generation, continuous-time spline factors, persistent world-frame LiDAR plane map constraints, and solve-step rejection are in place; full RMSE-gated native Coco-LIC2 tracking BA remains |
| Full-dataset strict parity matrix | PASS: `scripts/check_strict_parity_matrix.py` reports `required=17/17` with FAST-LIVO, FAST-LIVO2, M2DGR, MCD, and R3LIVE covered by required mapper/native/continuous-time evidence |
| Full raw/frontend/baseline/reference data inputs | PASS: `scripts/audit_strict_data_inputs.py` reports raw, frontend, ROS1 baseline, ROS2 current, native-reference, and disk-headroom checks passing for FAST-LIVO, FAST-LIVO2, M2DGR, MCD, and R3LIVE |
| R3LIVE coverage | `hku_park_00` frontend-raw conversion, 60s sensor-only native tracking health gate, and full-sequence mapper-contract/CUDA strict parity report pass |

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

- The native tracker has timestamp-safe trajectory/IMU primitives, ROS2-configurable LiDAR-to-IMU and camera-to-IMU extrinsics, IMU history interpolation, raw-sample IMU preintegration/reintegration, optimized pose/velocity/bias feedback into odometry, continuous-time trajectory controls, safe IMU re-anchoring, SO(3) log-map IMU rotation residuals with AutoDiff start-bias reintegration sensitivity, optional full 9x9 IMU residual sqrt-information whitening, IMU bias continuity residuals, per-block rotation/velocity/position residual weights, dense marginalization-prior anchoring, bias observability metrics/probes, reusable normal-equation linearization, Ceres-based cumulative B-spline trajectory estimation, continuous-time IMU/LiDAR/photometric residual surfaces, timestamped position-prior and SO(3) orientation-prior residuals for optional external/frontend odometry fusion, voxel-plane extraction, delayed point-cloud factor queuing until a same-stamp B-spline pose exists, persistent world-frame LiDAR plane map constraints, online Ceres solve-step rejection for non-finite or implausibly large knot updates, TUM prior publisher tooling, and real-bag continuous-time smoke/parity reports across every required profile.
- Full joint BA remains to be numerically hardened beyond current liveness-gated continuous-time evidence: the next paper-grade step is RMSE-gated continuous-time parity with stable IMU/gravity/bias initialization, persistent world-frame LiDAR map constraints, and Gaussian photometric feedback coupled into the spline window.
- Latest CBD continuous-time hardening, 2026-05-12: FAST-LIVO2 normalized-g IMU scaling (`9.80665`), one online Ceres iteration, lower IMU information weights, and rejected-solve position-extrapolation damping keep the frontend finite. The damped 60 s prior-seeded smoke drops the previous RMSE `59.498 m` blow-up to `1.803 m` at `results/fastlivo2/CBD_Building_01_continuous_time_damped0_prior_smoke_60s/trajectory_compare_report.json`, but that is a zero-path safety fallback. The parity script now records `runtime_diagnostics` from the continuous-time node; a 4 s diagnostic run at `results/fastlivo2/CBD_Building_01_continuous_time_diagnostics_probe_4s/native_tracking_report.json` shows 916 IMU messages, 45 pointcloud frames, 2,124 pointcloud correspondences, and 41 rejected solve steps, confirming the data enters the window while rotation updates remain the bottleneck. Real position-prior and SO(3) orientation-prior residuals were added for optional external/frontend odometry fusion; the 12 s position-prior diagnostic with weight `0.2` reaches RMSE `0.139 m`, but the 60 s diagnostic at `results/fastlivo2/CBD_Building_01_continuous_time_position_prior_w0p2_apply_diag_60s/trajectory_compare_report.json` still has RMSE `1.802 m`, 48.33% coverage, path length `39.18 m` vs `10.73 m`, and 381 rotation-limited steps. A 12 s pose-prior diagnostic with position weight `0.2` and orientation weight `0.5` reaches only RMSE `0.307 m` with `7.26x` path drift, so the pose-prior interface is not a paper-parity substitute. Delayed point-cloud queuing now holds plane/point-map LiDAR factors until the corresponding continuous-time pose can be queried; on the 12 s CBD diagnostic it releases 118/119 queued clouds, raises plane matches/updates to 2,459/3,554, and records zero queue drops. The 60 s queue diagnostic still reports RMSE `1.802 m` with 592 rejected Ceres steps, so the current blocker is inconsistent long-window rotation/LIO/VIO coupling, not missing point-cloud data. Per-factor Ceres cost diagnostics are now in the native report; the 4 s cost probe shows the last step is IMU-dominated (`405046 -> 227.6`), LiDAR cost increases (`0.0033 -> 0.116`), and all 41 rejections are rotation-update rejections with `0.685 rad` proposed, pointing at rotation coupling rather than data starvation. Ceres trust-region radii are exposed for online-window tuning: a `0.05` radius suppresses 4 s rotation rejections, but the 12 s CBD probe remains near the zero-path fallback (`0.145 m` RMSE, `0.016 m` path vs `0.917 m` baseline), so this is a diagnostic control rather than the parity fix. LiDAR plane-normal, independently weighted LiDAR scan-to-map SE(3) pose priors, bias-hold sweep controls, and default-off second-difference position smoothness factors are now available for diagnostics; local 12 s sweeps show they can reduce rejections, add 140k+ scan-map matches, or tune path scale. Limited-rotation updates now optionally scale position by the same ratio, fixing the earlier 17m path blow-up and giving a finite `0.957/1.002 m` 12 s path-scale probe, but ATE still does not close. The update gate also exposes a default-off edge-knot margin for guard-knot diagnostics; 12 s CBD margin-2 probes did not improve ATE (`0.203 m` without limited rotation, `0.205 m` with limited rotation), so it remains a tuning hook rather than a parity default. Default-off SO(3) second-difference smoothness and LiDAR scan-to-map velocity priors now give Ceres real rotation-shape and motion-scale residuals. The best 12 s CBD probe so far, `results/fastlivo2/CBD_Building_01_continuous_time_rot_smooth_0p01_lidar_pose_pos5_vel1_12s/trajectory_compare_report.json`, reaches RMSE `0.198 m`, zero rejected steps, and path `0.990/0.988 m`. The matching 60 s run keeps path scale finite at `10.089/10.603 m` with zero rejected steps, but RMSE remains `1.806 m`; a dense scan-to-map sweep worsens path scale, so full-sequence RMSE-gated parity now points at long-window trajectory shape/visual-global coupling rather than map capacity. A default-off image-to-image visual rotation-prior diagnostic now subscribes to raw camera frames in the continuous-time parity path with bounded image subsampling and frame stride. The 12 s stride-3 probe produces 36-40 visual priors with zero point-cloud drops, but worsens path scale (`4.58-10.12 m` vs `~0.98 m` baseline depending on sign/scale), so it is archived as evidence that naive monocular rotation coupling is insufficient. The standalone continuous-time node now also has a default-off sparse-depth visual SE3 prior that projects LiDAR depth into camera frames, applies SE3 photometric normal equations, transforms camera deltas through the camera-to-IMU extrinsic, and reports depth misses, valid batches, coverage tiles, mean residual, Hessian rank/condition, and prior counts. The best short 12 s CBD sweep so far (`visual_se3_max_dt_ns=250000000`, position/orientation weights `0.02/0.02`) reaches RMSE `0.19695 m`, slightly better than the no-visual `0.19797 m` checkpoint, but the matching 60 s run records RMSE `1.811 m` despite 581 SE3 batches and 298 valid priors, so it remains an experimental diagnostic rather than a paper-parity default. These are hardening checkpoints, not full-sequence paper parity.
- Latest CBD continuous-time hardening, 2026-05-13: LiDAR residual Huber gates now preserve physical units when residuals are preweighted, and native LiDAR correspondences carry Coco-LIC-style `use_lidar_scale` feature confidence. The parity script archives `pointcloud_factor_weight`, `pointcloud_use_lidar_scale`, and `pointcloud_min_lidar_scale`. The 12 s stable-weight probe at `results/fastlivo2/CBD_Building_01_lidar_scale_huber_weighted_default_w0p1_12s/trajectory_compare_report.json` records RMSE `0.116 m` and path `0.994 m` vs `0.811 m`; the 60 s run at `results/fastlivo2/CBD_Building_01_lidar_scale_huber_weighted_default_w0p1_60s/trajectory_compare_report.json` remains at RMSE `1.798 m`. Upstream-style `lidar_weight=500` no longer rejects every solve but overdrives the online window (`2.35-2.38 m` short-window path), so high LiDAR weight is not a parity default yet.
- The experimental persistent point-map path behind `enable_persistent_point_map` now feeds a coupled 3D continuous-time point-to-point residual instead of three independent axis-plane residuals, batches scan-local map insertions until after all associations are built, and only allows mature cross-stamp map entries to become association targets. The parity script archives `persistent_point_map_merge_distance_m`, `persistent_point_map_min_match_age_s`, and `persistent_point_map_min_observations_for_match` so point-anchor sweeps are reproducible. This fixes the residual/Jacobian/update semantics needed by Gaussian-map anchors, but it remains disabled as a parity default: CBD 12 s probes with mature 3D anchors still sit around RMSE `0.144-0.145 m` and shorten path length to `0.23-0.48 m` against a `0.92 m` reference.
- Sparse-depth visual SE3 velocity priors are also wired as a default-off diagnostic. Local 12 s CBD sweeps with velocity-only and mixed position/velocity/orientation weights show the factor is observable in Ceres, but it currently shortens path scale and does not improve ATE, so it stays off by default.
- LiDAR scan-to-map angular-velocity priors are wired on the continuous-time SO(3) derivative. The 10 Hz output trajectory stats exposed excessive long-window angular speed, and a 60 s CBD sweep with `lidar_pose_prior_angular_velocity_weight=0.1` reduces path drift from `264.75%` to `184.79%` while lowering p95 speed and angular speed. Full-data CBD is the stricter gate, though: `results/fastlivo2/CBD_Building_01_ct_stamp_driven_full_angvel01_output10hz/native_tracking_report.json` keeps RMSE at `3.664 m` and worsens path drift to `130.86%`, so angular velocity remains opt-in instrumentation rather than a default parity setting.
- Two-stamp scan-to-scan relative pose priors are now implemented in the continuous-time Ceres backend as default-off relative translation and SO(3) residuals between B-spline query times. Batch and streaming-window probes verify that the factors constrain relative motion without requiring a global anchor. Real CBD 12 s ablations prove the path is live but not yet a parity setting: relative position+orientation priors at weight `0.1` record RMSE `0.144711 m` with path `0.0355/0.9159 m`, and orientation-only records RMSE `0.144454 m` with path `0.0548/0.9159 m`. The remaining blocker is still long-window trajectory-shape and visual/Gaussian-map coupling, not missing relative-pose plumbing.
- Scan-to-scan target-quality gates now record raw target/prediction ratios and can either use the predicted relative translation or skip translation priors when ICP collapses the target motion. CBD 12 s probes show the gate is useful instrumentation but not the parity fix: ratio `0.5` prediction fallback fires 39 times and raises cumulative target path to `7.92 m`, yet the optimized path remains `0.045/0.914 m`; stronger weights pull path to `0.338 m` and then overdrive it to `2.41 m` without improving RMSE. This keeps the new knobs default-off and points the next sprint at global visual/Gaussian-map constraints rather than scan-to-scan scalar tuning.
- Gaussian/visual feedback replay now defaults the mapper feedback sync tolerance to `0.05 s` instead of `0.01 s`. A full120 CBD attempt with the old tolerance received points/poses/images but aligned zero mapper frames, so no Gaussian snapshot or visual/SE3 factors entered BA. The default 30 s probe at `/tmp/gaussian_feedback_default_sync_30s/native_tracking_report.json` now records a complete 16,653-point Gaussian snapshot, 222 visual and SE3 photometric BA factors, RMSE `0.328 m`, and `40.82%` path drift.
- Full120 Gaussian/visual feedback now has an archived yaw-gated health pass at `results/fastlivo2/CBD_Building_01_gaussian_visual_feedback_full120_yaw_gate_fixed/native_tracking_report.json`. The run records `1143` odometry poses, `1144` mapper point frames, `92.20%` reference coverage, `1135` yaw-aligned matches, RMSE `2.513 m`, mean `2.338 m`, max `4.591 m`, and `24.48%` path drift, with a complete `34,827`-point Gaussian snapshot, `814` visual factors, `814` SE3 photometric factors, `285` feedback updates, non-degenerate BA, and `sliding_window_orphan_factors=0`. This proves the production Gaussian/visual BA chain can run full-length, but it is still a meter-level tracker-quality checkpoint, not strict paper parity.
- Tracking step-guard diagnostics now report clamp counts plus the last raw/allowed step, dt, and reference speed. Adaptive step-guard knobs are exposed through `tracking.launch.py` and `run_native_tracking_bag_report.sh`, but remain opt-in: a 30 s CBD probe with velocity/acceleration scaling proved the parameters are live by raising current step p95 to `0.073 m`, but it overdrives path length to `10.30 m` vs `2.82 m` reference and worsens RMSE to `0.674 m`. The Gaussian feedback preset therefore keeps the conservative fixed `0.025 m` guard until a confidence-gated speed model is proven on full windows.
- Native tracking bag reports now merge `trajectory_compare` into `native_tracking_report.json` even when a required reference gate fails. That preserves failing RMSE/max-error/path-drift diagnostics in the main artifact instead of exiting after printing the comparison.
- The continuous-time point-cloud wait queue now covers every pose-dependent factor path, including scan-to-scan priors, Gaussian snapshot anchors, plane-normal factors, voxel-plane extraction, and persistent map constraints. That prevents ROS2 replay/callback timing from silently degrading those constraints into same-scan or stale-pose factors before the B-spline can answer the scan stamp. A 12 s pose-ready probe released all delayed point clouds with zero drops. Scan-to-scan velocity/angular-velocity priors are implemented and reproducibly exposed, but they stay opt-in because their best short-window probe (`0.096 m` RMSE) regressed the 60 s default run to `2.062 m` RMSE and `1361%` path drift. The default parity preset therefore keeps LiDAR scan-to-map pose/velocity priors plus SO(3) smoothness active while leaving scan-to-scan as a diagnostic until long-window trajectory shape is stable.
- Continuous-time parity replay now has a stamp-driven optimization mode. The ROS2 node still keeps its wall timer path for normal runtime, but the parity script enables `use_stamp_driven_steps` so Ceres solve cadence follows message stamps instead of wall-clock replay speed. A 4 s CBD check at playback rates `0.5` and `1.0` now produces the same 12 estimator steps instead of doubling the solve count under slow replay. The script also waits for `/continuous_time_node` before playing the bag, uses a fixed rosbag2 playback duration, records the capture wall duration, and defaults this parity path to playback rate `1.0` unless explicitly overridden. `PLAYBACK_DURATION=full` now resolves the bag duration from `metadata.yaml`, and parity output defaults to 10 Hz via `pose_output_period_seconds=0.1` so reports do not pass on sparse 5 Hz coverage; the native report also archives path drift, max translation error, best time-offset sweep diagnostics, and matched trajectory dt/step/speed/angular-speed statistics.
- Full-data CBD continuous-time evidence is now explicit: `results/fastlivo2/CBD_Building_01_ct_stamp_driven_full_default_stats_output10hz/native_tracking_report.json` replays the local 118.709 s frontend-raw bag with `PLAYBACK_DURATION=full`, captures 1184 odometry poses, matches 1173 reference poses (`95.29%` coverage), and records RMSE `3.665 m`, mean `3.303 m`, max `8.128 m`, path drift `64.78%`, current speed p95 `1.225 m/s` vs reference `0.613 m/s`, and current angular-speed p95 `4.141 rad/s`. This resolves the "data first" blocker for this gate and confirms the remaining strict blocker is long-window BA trajectory shape, not missing CBD input data.
- TensorRT/SPNet depth completion has a native optional wrapper and a local TensorRT 10.9 FP16 engine benchmark; the generated `.engine` is hardware/runtime-specific and intentionally not checked in.
- Strict paper reproduction now has the local `CBD_Building_01` bag, ROS1 upstream baseline artifacts, strict CUDA current collection, final-map render-pair extraction, a green mapper-contract/CUDA report, a green CBD native reference trajectory report, and continuous-time tracker producer-chain evidence across every required profile; the latest archived strict `CBD_Building_01` report is green. RMSE-gated continuous-time Coco-LIC2 tracking parity is still the next numerical target.

## Platform

Primary tested platform:

```text
Ubuntu 24.04
ROS2 Jazzy
CUDA 12.8
libtorch 2.7.0
```

Jazzy is the first-class development and CI target. Runtime helper scripts default to `ROS_DISTRO=jazzy`; other installed distros can be tried locally by exporting `ROS_DISTRO`, but Humble is not part of the required GitHub Actions matrix for this port.

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

Native C++ packages default to `RelWithDebInfo` when `CMAKE_BUILD_TYPE` is not
set, so local smoke and BA timing runs exercise optimized production code while
still preserving debug symbols. Explicit `colcon build --cmake-args
-DCMAKE_BUILD_TYPE=Debug` remains honored.
Native tracking smoke status and log artifacts are isolated by ROS domain and
process id, so concurrent local/CI runs cannot race on the same `/tmp` status
file; the latest successful status is still mirrored to
`/tmp/gaussian_lic_tracking_smoke_status.txt` for inspection.
`scripts/verify_workspace.sh` also runs a strict rosbag2 timing audit on the
frontend visual MCAP bag; when ROS2 or `rosbags` readers are available it checks
per-topic message header stamp ordering, not only rosbag metadata.

Run the native tracking probe suite:

```bash
colcon test --packages-select gaussian_lic_tracking --event-handlers console_direct+
colcon test-result --verbose
```

Expected local result is `35 tests, 0 errors, 0 failures, 0 skipped`.

Run a real frontend-raw native tracking evidence pass:

```bash
./scripts/run_native_tracking_bag_report.sh \
  --bag /home/frank/data/fast_livo/Bright_Screen_Wall_frontend_raw \
  --output results/fastlivo2/Bright_Screen_Wall_native_tracking_12s
```

This records native tracking odometry/status and mapper-contract outputs, then
gates signed-time, IMU, LiDAR, sliding-window, and numeric-Jacobian health in
`native_tracking_report.json`.

Add reference trajectory gating when the frontend-raw bag carries a trusted
odometry stream:

```bash
./scripts/run_native_tracking_bag_report.sh \
  --bag bags/synthetic_frontend_raw_visual_demo \
  --output /tmp/gaussian_lic_native_tracking_reference_prior_probe \
  --playback-duration 4 \
  --enable-external-odometry-prior \
  --require-reference-trajectory \
  --min-reference-poses 5
```

The default native path stays sensor-only; `--enable-external-odometry-prior`
must be set explicitly before the `/gaussian_lic/frontend/input_odometry`
stream is admitted as a sliding-window pose prior. The report then records both
`trajectory.tum` and `reference_trajectory.tum`, runs
`scripts/trajectory_compare.py`, and stores the result in
`native_tracking_trajectory_compare.json`.

For pure sensor-only native tracking, require the last BA solve to be
non-degenerate and tune the LiDAR factor budget explicitly:

```bash
./scripts/run_native_tracking_bag_report.sh \
  --bag /home/frank/data/fast_livo/Bright_Screen_Wall_frontend_raw \
  --output results/fastlivo2/Bright_Screen_Wall_native_tracking_nondegenerate_12s \
  --playback-duration 12 \
  --timeout 45 \
  --min-poses 30 \
  --min-point-frames 20 \
  --lidar-max-frame-points 1200 \
  --lidar-max-map-points 16000 \
  --sliding-window-max-iterations 2 \
  --sliding-window-max-state-gap-s 1.5 \
  --require-nondegenerate-ba
```

For the real-bag visual/SE3 photometric feedback path, launch mapper feedback
beside native tracking:

```bash
./scripts/run_native_tracking_bag_report.sh \
  --bag /home/frank/data/fast_livo/Bright_Screen_Wall_frontend_raw \
  --output results/fastlivo2/Bright_Screen_Wall_native_tracking_visual_sparse_depth_extrinsic_12s \
  --playback-duration 12 \
  --timeout 45 \
  --min-poses 30 \
  --min-point-frames 20 \
  --lidar-max-frame-points 1200 \
  --lidar-max-map-points 16000 \
  --sliding-window-max-iterations 2 \
  --sliding-window-max-state-gap-s 1.5 \
  --require-nondegenerate-ba \
  --enable-mapper-feedback
```

For a longer full-sequence CBD production-BA health run, use the frontend-raw
bag and loosen mapper feedback frame synchronization enough for asynchronous
rosbag2 replay:

```bash
./scripts/run_native_tracking_bag_report.sh \
  --bag /home/frank/data/fast_livo/CBD_Building_01_frontend_raw \
  --output results/fastlivo2/CBD_Building_01_native_tracking_visual_ba_sync50ms_120s \
  --playback-duration 120 \
  --timeout 240 \
  --min-poses 100 \
  --min-point-frames 100 \
  --enable-scan-order-deskew \
  --require-deskew \
  --enable-mapper-feedback \
  --mapper-feedback-sync-tolerance-sec 0.05 \
  --require-nondegenerate-ba
```

Latest local real-bag check:

```text
results/fastlivo2/Bright_Screen_Wall_native_tracking_8s/native_tracking_report.json
ok=true, poses=20, /points_for_gs=21, status_samples=20, imu_factors=3,
normal_equation_rows=87779, numeric_jacobian_blocks=0

results/fastlivo2/CBD_Building_01_native_tracking_8s/native_tracking_report.json
ok=true, poses=22, /points_for_gs=23, status_samples=22, imu_factors=3,
normal_equation_rows=65611, numeric_jacobian_blocks=0

/tmp/gaussian_lic_native_tracking_reference_prior_probe/native_tracking_report.json
ok=true, poses=32, reference_poses=32, external_prior_matches=25,
trajectory_rmse=0.101496 m, coverage=100.00%

results/fastlivo2/Bright_Screen_Wall_native_tracking_truncated_imu_gate_12s/native_tracking_report.json
ok=true, poses=38, /points_for_gs=38, status_samples=38, imu_factors=36,
state=tracking_with_sliding_window, normal_equation_degenerate=false,
state_gap_degenerate=false, accepted_steps=2, feedback_updates=36,
imu_factor_skip_count=0, imu_time_gap_skip_count=0,
pointcloud_imu_wait_deferred=7, pointcloud_imu_wait_released=7,
pointcloud_imu_wait_dropped=0, normal_equation_rows=30465,
condition=5.773e5, numeric_jacobian_blocks=0

results/fastlivo2/Bright_Screen_Wall_native_tracking_visual_sparse_depth_extrinsic_12s/native_tracking_report.json
ok=true, poses=36, /points_for_gs=36, status_samples=36, imu_factors=35,
visual_factors=2, se3_photometric_factors=2, se3_samples=68,
visual_depth_cache_size=4, visual_depth_match_delta_ns=95735550,
imu_factor_skip_count=0, imu_time_gap_skip_count=0,
normal_equation_rows=33855, condition=8.363e5, numeric_jacobian_blocks=0

results/fastlivo2/Bright_Screen_Wall_native_tracking_visual_sparse_depth_extrinsic_windowed_visual_30s/native_tracking_report.json
ok=true, poses=122, /points_for_gs=122, status_samples=122, imu_factors=121,
visual_factors=64, se3_photometric_factors=64, se3_samples=85,
visual_depth_cache_size=8, visual_depth_match_delta_ns=6003618,
visual_rendered_cache_size=64, visual_rendered_match_delta_ns=6003618,
visual_pending_stale_drops=0, se3_pending_stale_drops=0,
invalid_optimized_states=0, feedback_updates=121,
normal_equation_rows=22588, condition=6.532e6, rank_ratio=1.0,
numeric_jacobian_blocks=0

results/fastlivo2/Bright_Screen_Wall_native_tracking_scan_order_deskew_hits_12s/native_tracking_report.json
ok=true, poses=36, /points_for_gs=36, status_samples=36, imu_factors=35,
scan_order_deskew=true, max_abs_point_time_offset_s=0.05,
trajectory_deskew_queries=864000, trajectory_deskew_hits=762866,
lidar_invalid_point_times=0, lidar_out_of_range_point_times=0,
imu_factor_skip_count=0, imu_time_gap_skip_count=0,
invalid_optimized_states=0, normal_equation_rows=31137,
condition=1.035e6, numeric_jacobian_blocks=0

results/fastlivo2/Bright_Screen_Wall_native_tracking_visual_scan_order_deskew_gate_30s/native_tracking_report.json
ok=true, poses=112, /points_for_gs=112, status_samples=112, imu_factors=111,
visual_factors=59, se3_photometric_factors=9,
trajectory_deskew_queries=2688000, trajectory_deskew_hits=2616215,
max_abs_point_time_offset_s=0.05, lidar_invalid_point_times=0,
lidar_out_of_range_point_times=0, imu_factor_skip_count=0,
imu_time_gap_skip_count=0, invalid_optimized_states=0,
normal_equation_rows=17908, condition=5.044e6, rank_ratio=1.0,
numeric_jacobian_blocks=0

results/fastlivo2/Bright_Screen_Wall_native_tracking_visual_scan_order_deskew_queue_imu_order_30s/native_tracking_report.json
ok=true, poses=109, /points_for_gs=109, status_samples=109, imu_factors=107,
visual_factors=64, se3_photometric_factors=16,
trajectory_deskew_queries=2616000, trajectory_deskew_hits=2552484,
max_abs_point_time_offset_s=0.05, lidar_invalid_point_times=0,
lidar_out_of_range_point_times=0, imu_factor_skip_count=0,
imu_time_gap_skip_count=0, invalid_optimized_states=0,
visual_pending_stale_drops=0, se3_pending_stale_drops=0,
normal_equation_rows=14730, condition=8.467e6, rank_ratio=1.0,
numeric_jacobian_blocks=0

results/fastlivo2/Bright_Screen_Wall_native_tracking_visual_scan_order_deskew_source_id_30s/native_tracking_report.json
ok=true, poses=110, /points_for_gs=110, status_samples=110, imu_factors=108,
visual_factors=56, se3_photometric_factors=11,
visual_factor_replacements=0, se3_photometric_replacements=0,
visual_pending_queue_size=0, se3_pending_queue_size=0,
trajectory_deskew_queries=2640000, trajectory_deskew_hits=2562222,
imu_factor_skip_count=0, imu_time_gap_skip_count=0,
invalid_optimized_states=0, normal_equation_rows=18064,
condition=9.500e6, rank_ratio=1.0, numeric_jacobian_blocks=0

results/fastlivo2/Bright_Screen_Wall_native_tracking_visual_scan_order_deskew_source_id_queue_gate_60s/native_tracking_report.json
ok=true, poses=272, /points_for_gs=272, status_samples=270, imu_factors=270,
visual_factors=155, se3_photometric_factors=10,
visual_factor_replacements=0, se3_photometric_replacements=0,
visual_pending_queue_size=0, se3_pending_queue_size=0,
visual_pending_stale_drops=0, se3_pending_stale_drops=0,
trajectory_deskew_queries=6528000, trajectory_deskew_hits=6470281,
imu_factor_skip_count=0, imu_time_gap_skip_count=0,
invalid_optimized_states=0, feedback_updates=271,
normal_equation_rows=7679, condition=1.709e7, rank_ratio=1.0,
numeric_jacobian_blocks=0

results/fastlivo2/Bright_Screen_Wall_native_tracking_visual_valid_depth_sampling_default_60s/native_tracking_report.json
ok=true, poses=195, /points_for_gs=195, status_samples=195, imu_factors=194,
visual_factors=99, se3_photometric_factors=12,
se3_total_batches=99, se3_valid_batches=12, se3_total_samples=21627,
visual_depth_dilation_px=5, visual_factor_max_dt_ns=300000000,
visual_pending_queue_size=0, se3_pending_queue_size=0,
visual_pending_stale_drops=0, se3_pending_stale_drops=0,
trajectory_deskew_queries=4680000, trajectory_deskew_hits=4668986,
imu_factor_skip_count=0, imu_time_gap_skip_count=0,
invalid_optimized_states=0, feedback_updates=194,
normal_equation_rows=20489, condition=1.391e7, rank_ratio=1.0,
numeric_jacobian_blocks=0

results/fastlivo2/Bright_Screen_Wall_native_tracking_visual_depth_window_split_default_30s/native_tracking_report.json
ok=true, visual_factor_max_dt_ns=300000000, visual_depth_max_dt_ns=0,
visual_factors=45, se3_photometric_factors=10,
se3_total_batches=45, se3_valid_batches=10, se3_total_samples=19176,
visual_pending_stale_drops=0, se3_pending_stale_drops=0,
imu_factor_skip_count=0, imu_time_gap_skip_count=0,
normal_equation_rows=44808, condition=7.019e6,
numeric_jacobian_blocks=0

results/fastlivo2/Bright_Screen_Wall_native_tracking_se3_hessian_gate_30s/native_tracking_report.json
ok=true, poses=87, /points_for_gs=88, status_samples=87, imu_factors=85,
visual_factors=46, se3_photometric_factors=10,
se3_total_batches=46, se3_valid_batches=10, se3_degenerate_batches=0,
se3_total_samples=19176, se3_min_hessian_rank_gate=3,
last_accepted_se3_hessian_rank=6, last_accepted_se3_hessian_condition=7.240e3,
trajectory_deskew_queries=2088000, trajectory_deskew_hits=2031012,
imu_factor_skip_count=0, imu_time_gap_skip_count=0, feedback_updates=85,
normal_equation_rows=40555, condition=9.543e6, rank_ratio=1.0,
numeric_jacobian_blocks=0

results/fastlivo2/Bright_Screen_Wall_native_tracking_se3_sample_quality_gate_30s/native_tracking_report.json
ok=true, poses=82, /points_for_gs=83, status_samples=82, imu_factors=80,
visual_factors=73, se3_photometric_factors=22,
se3_total_batches=73, se3_valid_batches=22, se3_degenerate_batches=0,
se3_quality_rejected_batches=0, se3_total_samples=39321,
se3_min_sample_inlier_ratio_gate=0.25,
last_accepted_se3_sampled_depth=446, last_accepted_se3_samples=422,
last_accepted_se3_sample_inlier_ratio=0.946,
last_accepted_se3_hessian_rank=6, last_accepted_se3_hessian_condition=2.439e7,
trajectory_deskew_queries=1968000, trajectory_deskew_hits=1906016,
imu_factor_skip_count=0, imu_time_gap_skip_count=0, feedback_updates=80,
normal_equation_rows=40801, condition=1.877e7, rank_ratio=1.0,
numeric_jacobian_blocks=0

results/fastlivo2/Bright_Screen_Wall_native_tracking_se3_coverage_gate_30s/native_tracking_report.json
ok=true, poses=82, /points_for_gs=82, status_samples=82, imu_factors=81,
visual_factors=43, se3_photometric_factors=12,
se3_total_batches=43, se3_valid_batches=12, se3_degenerate_batches=0,
se3_quality_rejected_batches=1, se3_total_samples=24210,
se3_coverage_grid=4x4, se3_min_coverage_tiles_gate=4,
last_accepted_se3_sampled_depth=1962, last_accepted_se3_samples=1837,
last_accepted_se3_sample_inlier_ratio=0.936,
last_accepted_se3_coverage_tiles=4/16,
last_accepted_se3_hessian_rank=6, last_accepted_se3_hessian_condition=9.787e3,
trajectory_deskew_queries=1968000, trajectory_deskew_hits=1924727,
imu_factor_skip_count=0, imu_time_gap_skip_count=0, feedback_updates=81,
normal_equation_rows=39122, condition=4.754e6, rank_ratio=1.0,
numeric_jacobian_blocks=0

results/fastlivo2/Bright_Screen_Wall_native_tracking_se3_balanced_sampling_30s/native_tracking_report.json
ok=true, poses=83, /points_for_gs=83, status_samples=83, imu_factors=82,
visual_factors=56, se3_photometric_factors=15,
se3_total_batches=56, se3_valid_batches=15, se3_degenerate_batches=0,
se3_quality_rejected_batches=4, se3_total_samples=35004,
se3_coverage_grid=4x4, se3_min_coverage_tiles_gate=4,
last_accepted_se3_sampled_depth=2000, last_accepted_se3_samples=1899,
last_accepted_se3_sample_inlier_ratio=0.950,
last_accepted_se3_coverage_tiles=5/16,
last_accepted_se3_hessian_rank=6, last_accepted_se3_hessian_condition=7.467e3,
imu_factor_skip_count=0, imu_time_gap_skip_count=0, feedback_updates=82,
normal_equation_rows=49171, condition=1.143e7, rank_ratio=1.0,
numeric_jacobian_blocks=0

results/fastlivo2/CBD_Building_01_native_tracking_visual_ba_sync50ms_120s/native_tracking_report.json
ok=true, poses=478, /points_for_gs=478, status_samples=478, imu_factors=477,
visual_factors=238, se3_photometric_factors=27,
se3_total_batches=238, se3_valid_batches=27, se3_degenerate_batches=0,
se3_quality_rejected_batches=21, se3_total_samples=76250,
visual_rendered_miss_count=2, visual_rendered_cache_size=64,
visual_depth_cache_size=64, mapper_feedback_sync_tolerance_sec=0.05,
last_accepted_se3_sample_inlier_ratio=0.935,
last_accepted_se3_coverage_tiles=6/16,
last_accepted_se3_hessian_rank=6, last_accepted_se3_hessian_condition=6.187e3,
trajectory_deskew_queries=11472000, trajectory_deskew_hits=11430279,
imu_factor_skip_count=0, imu_time_gap_skip_count=0, feedback_updates=477,
normal_equation_rows=6258, condition=5.818e6, rank_ratio=1.0,
numeric_jacobian_blocks=0

results/r3live/hku_park_00_native_tracking_scan_order_60s/native_tracking_report.json
ok=true, poses=554, /points_for_gs=554, status_samples=554, imu_factors=553,
total_window_point_correspondences=18004,
total_window_plane_correspondences=7474, smoothness_factors=10,
trajectory_deskew_queries=13296000, trajectory_deskew_hits=8542971,
lidar_map_points=16000, pointcloud_imu_wait_deferred=395,
pointcloud_imu_wait_released=395, pointcloud_imu_wait_dropped=0,
imu_factor_skip_count=0, imu_time_gap_skip_count=0, feedback_updates=553,
normal_equation_rows=567, condition=3.317e4, rank_ratio=1.0,
numeric_jacobian_blocks=0
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

Convert an official ROS1 dataset bag into that ROS2 raw-sensor contract. The
profile selects source-topic fallbacks, camera intrinsics, and the LiDAR frame;
`fastlivo2_ros1_to_frontend_raw.py` remains as a compatibility entrypoint, but
new runs should use `ros1_to_frontend_raw.py`:

```bash
PYTHONPATH=/home/frank/.cache/gaussian_lic_ros2/rosbags-venv/lib/python3.12/site-packages \
  /usr/bin/python3 scripts/ros1_to_frontend_raw.py \
  --profile fastlivo2 \
  --input /home/frank/data/fast_livo/Bright_Screen_Wall.bag \
  --output /home/frank/data/fast_livo/Bright_Screen_Wall_frontend_raw \
  --overwrite
```

Valid profiles are `fastlivo`, `fastlivo2`, `m2dgr`, `mcd`, and `r3live`.
For rotating-LiDAR datasets that already publish `sensor_msgs/PointCloud2`, the
converter preserves the original fields and only normalizes the ROS2 bag
contract topics. For split datasets such as MCD, pass multiple ROS1 bags as a
comma-separated `--input` list so the converter merges camera, LiDAR, and IMU
streams into one timestamp-sorted ROS2 `frontend_raw` bag.

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
SPNET_PYTHON=/home/frank/.cache/gaussian_lic_ros2/quality-cuda-venv/bin/python \
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

These profiles currently cover the native mapper surface: topic names, global and per-input QoS, synchronization, upstream camera intrinsics, image size, depth-completion toggles, Gaussian initialization/map-growth parameters, upstream optimizer/loss/exposure parameter names, output topics, and active profile labels. The contract adapter and native tracking launch also expose per-stream raw-input and mapper-output QoS so selected streams can be promoted without making every camera/LiDAR/IMU/depth/pose topic reliable. The native tracking launch keeps high-rate tracking inputs at bounded `best_effort` by default, while `scripts/run_native_tracking_bag_report.sh` promotes the strict-replay IMU and point-cloud inputs to reliable QoS with deep queues; image topics remain best-effort. Tracking now auto-calibrates initial IMU gravity from the startup accelerometer window, preserves a deeper IMU history for delayed point-cloud callbacks, and queries the propagated IMU state at each point-cloud stamp instead of using a future latest state. The launch exposes per-point LiDAR deskew controls for common `offset_time`, `time`, `timestamp`, and `t` fields while preserving signed-nanosecond estimator math, rejects invalid builtin stamp nanosecond fields, non-monotonic tracking stream stamps, non-finite IMU measurements, non-finite CameraInfo intrinsics, non-finite extrinsic parameters, non-finite IMU gravity, invalid LiDAR keyframe thresholds, invalid cache sizes, invalid visual/SE3 photometric gates, invalid LiDAR thresholds, and invalid sliding-window BA bounds before mapper/frontend arithmetic, rejects invalid deskew poses before they can create NaN points, publishes latest signed-nanosecond image/LiDAR/IMU stamps, stamp-regression counters, and invalid-IMU counters for smoke gating, and serializes tracking callbacks by default so a future multi-threaded executor cannot concurrently mutate IMU/LiDAR/image estimator state. The sliding-window optimizer is now default-enabled for production BA runtime coverage, with reusable normal-equation linearization, rank/condition health diagnostics and solve guards, factor-agnostic solve triggering when LiDAR/visual/smoothness factors arrive without a valid IMU factor, bounded LM state increments plus accepted/rejected step status, dense prior anchoring, Schur-complement retained-window priors with rank/singular-value health, optimized pose/velocity/bias feedback into odometry, trajectory controls, and safe IMU re-anchoring, timestamp-safe cubic B-spline position/velocity and SO(3) cubic orientation trajectory queries for deskew fallback, default-enabled three-state trajectory smoothness factors with closed-form SO(3) rotation blocks, bias observability status, Huber-weighted LiDAR 6-DoF pose correction, corrected-pose LiDAR/Gaussian correspondence construction, spatial-indexed per-correspondence robust LiDAR/Gaussian-map weights bounded to `(0, 1]`, LiDAR point-to-plane factors with residual/local-planarity confidence weighting, LiDAR confidence/spatial-index status, sliding-window optimization timing plus numeric-Jacobian fallback status, Huber-robust visual-alignment and SE3 photometric window factors, analytic geometric and full current IMU preintegration Jacobians, robust analytic SE3 camera photometric normal equations/window factors from rendered/current/depth images selected through bounded nearest-stamp render/depth caches with body-frame sqrt-information, runtime status for 2-DoF and SE3 photometric linearization quality, and subpixel visual alignment in place. The local CBD native reference checkpoint now passes under controlled strict replay; cross-dataset long-window RMSE-gated production BA remains the next tracker quality target.

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

The trajectory gate associates poses by timestamp, reports translation RMSE/mean/max plus path-length drift, and exits non-zero when any threshold fails. Use `--align first` only for datasets whose baseline and current trajectories differ by a known initial translation offset. Use `--align yaw` for local-world frontend trajectories, such as M2DGR native tracking, where the estimator origin/yaw is arbitrary but scale is fixed.

M2DGR `room_01` native tracking now has a 60s slow-replay reference pass at `results/m2dgr/room_01_tracking_sweep_yaw_guarded008_60s/all_frames_default_60s/native_tracking_report.json`: 106 odometry poses, 105 yaw-aligned GT matches, RMSE 0.4479 m, mean 0.4342 m, max 0.7597 m, and zero report errors. This is a frontend trajectory parity checkpoint, not a replacement for the remaining full-dataset ROS1 mapper/render strict artifacts.

Run the remaining full-profile strict artifact backlog from one queue:

```bash
./scripts/run_strict_parity_queue.sh --continue-on-error
```

Use `--dry-run` first to see the exact per-profile commands. The queue uses the existing raw/frontend data under `/home/frank/data`, writes baselines under `baseline/<profile>/<sequence>`, writes current artifacts under `results/<profile>/<sequence>_strict_current`, and finishes by refreshing `docs/strict_data_status.*` plus `scripts/check_strict_parity_matrix.py --allow-incomplete`.

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

If `--current-results-dir` is omitted and `results/fastlivo2/current` does not
exist, the readiness tool now auto-selects a sequence-specific archived current
artifact with the required `trajectory.tum`, `point_cloud.ply`, and
`metrics.json` files. On the local strict archive it resolves to
`results/fastlivo2/CBD_Building_01_current_round_no_opacity_prune_probe/` and
recomputes the strict report instead of trusting stale JSON.

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
slowed current playback rate of `0.25`, `--current-record-sec 0` so collection
runs until rosbag2 playback plus the 60 second post-playback settle window
finish, a 1.5M foreground Gaussian cap, uniform foreground count-cap policy with
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
Strict quality extraction defaults LPIPS to `cuda` on the target workstation; set
`--quality-lpips-device cpu` or `QUALITY_LPIPS_DEVICE=cpu` only for CPU-only
report refreshes. On this workstation the CUDA report environment is
`${HOME}/.cache/gaussian_lic_ros2/quality-cuda-venv/bin/python` with
Torch `2.7.0+cu128`; if a requested CUDA LPIPS device is unavailable, the
evaluator records the requested device and falls back to CPU instead of leaving
LPIPS null.

That command converts the official ROS1 bag to a sqlite-backed ROS2
`frontend_raw` bag, audits replay timing, writes the ROS1 mapper-contract bag,
runs the upstream ROS1 baseline container, collects ROS2 current artifacts, and
emits strict readiness and reproduction reports. During report generation it also
fills `metrics.json::quality`: the baseline uses upstream GT, while the current
run uses its saved `gt/` frames so dropped/reordered replay frames do not get
scored against the wrong source image. Render-pair comparison uses decoded GT
hashes to associate ROS1 and ROS2 frame identities before sampling pairs. The
strict report keeps every per-pair outlier in JSON, but gates the supplemental
baseline-vs-current render-pair check by bounded outlier ratio plus mean
PSNR/SSIM; the primary paper-quality gate remains the full matched-frame
PSNR/SSIM/LPIPS regression against GT. Set `QUALITY_PYTHON` to a Python
environment with `torch`, `torchvision`, `numpy`, and `Pillow` when refreshing
those metrics outside the default CUDA venv.

Latest archived local strict run, 2026-05-05:

- ROS1 baseline visual dump: 1186 render/GT pairs, 237 train and 949 novel frames.
- ROS2 current strict path: CUDA rasterizer, final-map evaluation, IMU-fallback
  world-frame point-cloud rotation enabled, ROS1-compatible projected-depth
  nearest-pixel rounding, high-rate GaussianArray publication disabled, 1.5M
  foreground cap with no opacity pruning and uniform count-cap policy,
  alpha-hole extension filtering
  enabled at threshold `0.99`, ROS2 gradient split/clone densification disabled,
  opacity reset disabled, deterministic upstream-random optimization-frame
  sampling enabled with seed `20260505`,
  `PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True`, and
  `results/fastlivo2/CBD_Building_01_current_round_no_opacity_prune_probe/` as
  the current archived strict artifact. The trajectory gate passes with 1063
  matched poses, 89.63% coverage, zero translation RMSE, and zero path drift.
- Current vs ROS1 quality passes the archived strict metric gate: ROS2 novel
  PSNR is 12.6542 dB vs ROS1 12.7036 dB (0.39% regression), ROS2 novel SSIM is
  0.368961 vs ROS1 0.3644, and ROS2 novel LPIPS is 0.741661 vs ROS1 0.75136.
  Future quality extraction uses Gaussian-LIC's 11x11 Gaussian-window SSIM and
  keeps the previous global SSIM as a diagnostic field.
- The GT-associated ROS1-vs-ROS2 render-pair smoke check passes: 1063 pairs
  match by decoded GT hash, 64 pairs are evaluated, mean PSNR is 21.600 dB,
  mean SSIM is 0.785417, min PSNR is 17.448 dB, and min SSIM is 0.477513.
- Chamfer/point-cloud parity passes: count ratio 0.978, centroid drift
  0.147855 m, bidirectional nearest RMSE 0.145491 m, and bidirectional nearest
  mean 0.099418 m against the 0.100000 m strict threshold.

Visual review artifacts from the same strict run:

```text
docs/assets/strict_cbd_montage.jpg
docs/assets/strict_cbd_render_demo.gif
```

So the strict chain now passes the local `CBD_Building_01` reproduction gate for
the mapper-contract/CUDA current path. The remaining paper-port work is full
native Coco-LIC2 tracking parity and dataset-validated production joint BA beyond
the mapper-contract path.

The final full-dataset parity gate is explicit and now passes for the required
local evidence matrix:

```bash
./scripts/check_strict_parity_matrix.py
```

To write the same report while allowing future optional items to remain incomplete:

```bash
./scripts/check_strict_parity_matrix.py \
  --allow-incomplete \
  --output results/strict_parity_matrix.json \
  --markdown results/strict_parity_matrix.md
```

Current local matrix status: `PASS`, `required=17/17`, `covered_profiles=fastlivo,fastlivo2,m2dgr,mcd,r3live`.
Passing required items are FAST-LIVO2 `CBD_Building_01` mapper-contract/CUDA
strict parity, FAST-LIVO hku1/hku2/Visual_Challenge/LiDAR_Degenerate
mapper-contract/CUDA strict parity, M2DGR room_01/room_02/room_03 mapper-contract/CUDA strict
parity, MCD `ntu_day_01` mapper-contract/CUDA strict parity, R3LIVE
`hku_park_00` mapper-contract/CUDA strict parity, and the 120s CBD native
visual/SE3 BA health report. The final required item, CBD native reference
trajectory parity, now passes against an upstream Coco-LIC-generated trusted TUM
reference decimated to the ROS2 native output cadence. The five added
continuous-time native tracker entries cover FAST-LIVO2 CBD, FAST-LIVO2 Retail,
M2DGR room_01, MCD ntu_day_01, and R3LIVE hku_park_00 as liveness/producer-chain
gates.

The full-profile strict queue uses `--current-record-sec 0` and a slower default
`0.15x` replay rate than the focused CBD script so CUDA final-render runs do not
silently truncate or drop synchronized point/pose frames on longer FAST-LIVO
sequences.
For offline strict replay it also overrides every mapper and adapter stream QoS
to reliable with keep-last depth `50`; live profiles keep the YAML defaults
(`best_effort`, depth `5`) unless the launch caller overrides them.

The matching data audit is:

```bash
./scripts/audit_strict_data_inputs.py \
  --output docs/strict_data_status.json \
  --markdown docs/strict_data_status.md \
  --cleanup-candidates 12 \
  --min-free-gb 100
```

As of 2026-05-12 it reports `109.39 GiB` free on `/home/frank/data`, above the
100 GiB release threshold. Raw bags, converted frontend-raw bags, ROS1 baseline
artifacts, ROS2 current artifacts, and native reference trajectories are local
for every required profile: FAST-LIVO, FAST-LIVO2, M2DGR, MCD, and R3LIVE.
FAST-LIVO2 `Retail_Street` now also has an archived ROS1 upstream mapper
baseline and native-reference TUM. The script also lists the largest non-matrix
`results/` directories that can be reviewed before reclaiming space; it
deliberately does not list archived baseline directories as cleanup candidates.

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

Latest native-tracker status (2026-05-13): data and reference trajectories are
available, and the remaining blocker is long-window continuous-time BA quality.
The best recent CBD 60 s native run is still about `1.8 m` RMSE against the
trusted Coco-LIC reference. New retained-knot prior hooks and scan-to-scan
position-prior ablations are implemented and tested, but the 12 s probes showed
they are diagnostic only: direct scan-to-scan position priors over-scale path
length, retained-knot soft anchors also over-scale, and unconstrained velocity
priors do not beat the stable configuration. The current parity script therefore
uses a conservative default instead of the old all-off smoke preset: LiDAR
pose-prior factors are enabled, LiDAR pose velocity weight remains active,
IMU information defaults to `0.3/0.03`, and rotation smoothness defaults to
`0.01`. Scan-to-scan priors remain opt-in diagnostics; enabling damped
scan-to-scan velocity/angular priors reached `0.096 m` RMSE on a 12 s probe but
regressed the 60 s default to `2.062 m` RMSE with `1361%` path drift. Full
marginalization/global visual-map coupling is still needed to close the strict
parity gap.

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
