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
- Strict FAST-LIVO2 `CBD_Building_01` local mapper-contract/CUDA reproduction pipeline with trajectory, PSNR/SSIM/LPIPS, GT-associated render-pair, and point-cloud gates passing against the archived ROS1 upstream baseline.

Incomplete paper-algorithm scope:

- `lic2_contract_adapter` is not the native continuous-time Gaussian-LIC/Coco-LIC tracker.
- TensorRT/SPNet depth completion has a generated local TensorRT 10.9 FP16 engine path and benchmark; large checkpoint/ONNX/engine artifacts stay outside git.
- The public LIC2 upstream surface does not currently expose separate 2D Gaussian primitive or skybox patches beyond the checked mapper backend.
- Full native Coco-LIC2 frontend/tracking parity remains outside the current mapper-contract checkpoint.

## Current Execution Gate

Strict baseline data gate:

```text
./scripts/baseline_readiness.py --dataset-root /home/frank/data/fast_livo --sequence CBD_Building_01
```

If the raw bag is missing, fetch it with:

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

As of 2026-05-05, `/home/frank/data/fast_livo/CBD_Building_01.bag` is present
locally, the upstream ROS1 `gs_mapping` target builds in Docker with the local
OpenCV 4.10 CUDA fallback and TensorRT 8.6.1.6, and
`baseline/fastlivo2/CBD_Building_01/` contains the archived ROS1 upstream
baseline (`trajectory.tum`, `point_cloud.ply`, `renders/`, `metrics.json`, and
`run.log`). The latest archived ROS2 strict current report at
`results/fastlivo2/CBD_Building_01_current_round_no_opacity_prune_probe/`
passes `reproduction_report.py --strict`: trajectory coverage is 89.63%, novel
PSNR regresses 0.39%, novel SSIM/LPIPS improve relative to the archived ROS1
baseline, the decoded-GT render-pair gate passes, point-cloud centroid drift is
0.147855 m, and bidirectional nearest mean distance is 0.099418 m against the
0.100000 m strict threshold.

The official FAST-LIVO2 `Bright_Screen_Wall` bag remains the quick executable
substitute for short regression coverage. The latest colorized combined proof
chain has:

```text
baseline/fastlivo2/Bright_Screen_Wall_fastlivo2_color_8s/
results/fastlivo2/Bright_Screen_Wall_current_extrinsic_color_fullseq/reproduction_report.json
```

That Bright substitute passing report is a fast regression companion to the
strict `CBD_Building_01` paper gate, not a replacement for the full-sequence
strict report.

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
- [x] Add per-stream input/output QoS controls to the frontend contract adapter so mapper QoS overrides can be matched without forcing all streams reliable.
- [x] Add a `frontend_raw` bag contract for raw camera/LiDAR/IMU/pose validation before adapter replay.
- [x] Add a `frontend_sensor_raw` contract and FAST-LIVO2 ROS1-to-ROS2 converter for real camera/LiDAR/IMU bags without pose.
- [x] Add IMU gyro orientation fallback for raw frontend bags without odometry.
- [x] Add FAST-LIVO2 camera-LiDAR static transform profile for raw pointcloud projection.
- [x] Add real-dataset topic remaps/adapters for the current native Gaussian-LIC2 frontend/tracking surface.

## Milestone 2: Gaussian-LIC Mapping Port

- [x] Inventory ROS1 APIs used by upstream Gaussian-LIC.
- [x] Replace the four-topic ROS1 input synchronization surface with native ROS2.
- [x] Convert aligned ROS2 frames into mapper-ready image/depth/pose/point data.
- [x] Consume CameraInfo intrinsics with parameter fallback.
- [x] Reject non-finite or non-positive CameraInfo intrinsics before mapper/tracker state updates.
- [x] Accumulate converted frames into a non-torch `MapperDataset`.
- [x] Publish ROS2 odometry, path, and accumulated map-point topics from the native mapper.
- [x] Add optional `map -> camera` TF broadcasting.
- [x] Port upstream Gaussian-LIC backend parameter names into ROS2 profiles.
- [x] Replace remaining ROS1 node handles, publishers, and launch files in the ROS2 source tree.
- [x] Isolate CUDA/Gaussian map code from ROS middleware code.
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
- [x] Add mapper per-input QoS controls so high-rate image/LiDAR and lower-rate pose/CameraInfo streams are not forced into one reliability policy.
- [x] Fill `MappingStatus` tracking/mapping rate fields from live frame counters.
- [x] Make TensorRT depth completion optional at the ROS2 mapper input boundary.
- [x] Port the native TensorRT/SPNet depth completion wrapper and mapper lazy-loading path.
- [x] Add SPNet checkpoint download, ONNX export, TensorRT 10.x engine generation, and sm_120 compatibility guard.
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
- [x] Publish signed-nanosecond image/LiDAR/IMU tracking stamps and gate them in native tracking smoke.
- [x] Reject non-monotonic native tracking image/depth/rendered/LiDAR/IMU stream stamps before estimator mutation and publish regression counters.
- [x] Reject invalid ROS2 builtin stamp nanosecond fields in mapper/frontend timestamp helpers before estimator arithmetic.
- [x] Expose tracking sensor QoS through launch/status and gate bounded `best_effort` defaults in native tracking smoke.
- [x] Add per-stream native tracking raw-input and mapper-output QoS controls so image/LiDAR/IMU/depth/pose/odometry streams no longer share one fixed policy.
- [x] Add default callback serialization in native tracking so MultiThreadedExecutor use cannot concurrently mutate IMU/LiDAR/image estimator state.
- [x] Add a native timestamp-safe cubic B-spline trajectory manager foundation with position/velocity and SO(3) cubic orientation constant-rate probes.
- [x] Add a native signed-nanosecond IMU propagation foundation with a deterministic gyro/accel probe.
- [x] Reject non-finite IMU angular velocity or acceleration at the native tracking input boundary before propagation/preintegration.
- [x] Publish CameraInfo/image/depth/rendered-image invalid-frame counters so visual/SE3 photometric factor loss is directly observable.
- [x] Add signed-nanosecond IMU history interpolation and an IMU preintegration residual foundation with deterministic probes.
- [x] Add per-block IMU preintegration residual weighting for rotation, velocity, and position constraints with analytic Jacobian coverage.
- [x] Add a native ROS2 tracking node surface that publishes mapper input topics plus odometry, path, and optional TF.
- [x] Add a native LiDAR nearest-neighbor residual factor with bounded local map, PointCloud2 parsing, bounded 6-DoF pose correction, and deterministic correction probes.
- [x] Publish LiDAR invalid xyz and per-point timestamp rejection counters so degraded point/factor counts are visible in native tracking status.
- [x] Publish malformed PointCloud2 frame rejection counters for unsupported layout/endian/field contracts.
- [x] Gate out-of-range LiDAR per-point timestamp offsets before deskew and publish rejection/max-offset diagnostics.
- [x] Add ROS2-configurable LiDAR-to-IMU extrinsics before LIO, deskew, and mapper point publication.
- [x] Reject malformed or non-finite LiDAR-to-IMU and camera-to-IMU extrinsics before tracker startup.
- [x] Reject non-finite IMU gravity and LiDAR keyframe threshold parameters before propagation/keyframe logic.
- [x] Hard-validate native tracking QoS depths, visual/SE3 photometric gates, LiDAR factor thresholds, cache sizes, and sliding-window BA bounds before runtime clamps can hide bad configs.
- [x] Add per-point LiDAR deskew from PointCloud2 time fields before mapper publication and LIO correction.
- [x] Reject invalid reference/per-point deskew poses so zero quaternions cannot inject NaNs into LiDAR points before LIO/BA.
- [x] Add an optional native sliding-window optimizer foundation with IMU preintegration factors, raw-sample bias reintegration, bias continuity, bias observability metrics, pose/state priors, dense marginalization-prior anchoring, window trimming, and deterministic convergence probes.
- [x] Add Schur-complement normal-equation math coverage for the tracking marginalization path, with deterministic equivalence against the full normal-equation solve.
- [x] Expose reusable sliding-window residual/Jacobian/Hessian/RHS normal-equation linearization and cover it with CTest.
- [x] Add analytic Jacobians for full current IMU preintegration factors, pose-prior, state-prior, retained dense-prior, IMU bias-continuity, and SE3 photometric window rows, with deterministic CTest coverage.
- [x] Upgrade IMU preintegration rotation residuals to SO(3) log-map semantics, including AutoDiff start-bias sensitivity and deterministic residual/Jacobian probes.
- [x] Add optional full 9x9 sqrt-information whitening for IMU preintegration residuals, preserving scalar block weights as the default path and covering correlated residual whitening with finite-difference CTest.
- [x] Feed Schur-complement marginalization back into the live sliding window as retained-state dense priors, with fallback to diagonal anchoring when the marginalized block has no information.
- [x] Publish dense-prior marginalization rank and singular-value health, and gate it in native tracking smoke.
- [x] Add direct LiDAR point-to-point correspondence factors to the optional tracking window.
- [x] Add per-correspondence robust weighting for LiDAR and Gaussian-map point-to-point tracking-window factors.
- [x] Apply Huber weights inside bounded LiDAR 6-DoF pose-correction Kabsch centroids/covariance so outlier correspondences cannot dominate pre-BA pose feedback.
- [x] Build LiDAR, plane, and Gaussian-map window correspondences from the bounded corrected pose instead of stale IMU-only prediction.
- [x] Add LiDAR point-to-plane tracking-window factors from local plane fits with deterministic CTest coverage.
- [x] Add a native visual photometric residual/alignment factor and subscribe tracking to mapper rendered-image output.
- [x] Add ROS2-configurable camera-to-IMU extrinsics and apply their SE3 adjoint to photometric BA deltas.
- [x] Add subpixel refinement for visual alignment factors before they enter the optional tracking window.
- [x] Gate pending visual/SE3 photometric BA factors by signed-nanosecond image-to-LiDAR and image-to-depth timestamp freshness before consumption.
- [x] Cache recent depth frames and select the nearest fresh stamp for SE3 photometric sampling instead of relying on latest-frame arrival order.
- [x] Cache mapper rendered frames and select the nearest fresh stamp for visual residual/alignment/SE3 photometric factors.
- [x] Add a configurable Huber kernel to visual-alignment sliding-window factors so outlier image shifts are downweighted inside BA.
- [x] Add a configurable Huber kernel to SE3 photometric window-factor whitened residuals so outlier render/depth deltas are downweighted inside BA.
- [x] Add configurable LM step limits for rotation, translation, velocity, and biases to prevent BA state jumps from transient outliers.
- [x] Add configurable final BA feedback limits for translation, rotation, and velocity before optimized states can update odometry, IMU re-anchors, or B-spline controls.
- [x] Publish sliding-window accepted/rejected LM steps, last step norm/scale, and damping through `TrackingStatus`.
- [x] Explicitly reject non-finite sliding-window LM candidate states and publish invalid-candidate step counts.
- [x] Publish sliding-window limited-step counts so real-bag BA saturation is visible without parsing debug logs.
- [x] Publish sliding-window normal-equation row/column/rank, singular-value, and condition-number diagnostics through `TrackingStatus`.
- [x] Add configurable sliding-window normal-equation rank/condition guards so degenerate BA solves are rejected before state feedback.
- [x] Trigger sliding-window BA from any accepted LiDAR, plane, visual, SE3 photometric, smoothness, or IMU factor instead of silently waiting for a valid IMU factor.
- [x] Publish per-factor skip counters for LiDAR point/plane, visual, SE3 photometric, smoothness, IMU, and optimizer failures.
- [x] Publish the last consumed IMU preintegration block sample count, dt, and signed-nanosecond span through `TrackingStatus`.
- [x] Publish cumulative sliding-window IMU-factor/preintegration totals so optimized runs still prove IMU usage after active-window marginalization.
- [x] Make synthetic rosbag IMU samples monotonic inside the previous/current frame interval so timer jitter cannot create false IMU stamp regressions.
- [x] Extend rosbag2 timing audit to inspect MCAP/header stamp order through ROS2 readers and gate frontend visual bags in workspace verification.
- [x] Reject IMU factors whose preintegration span does not cover the BA state interval, with bounded final-sample extrapolation and time-gap skip status.
- [x] Report and gate orphan sliding-window factors whose referenced states are absent instead of silently skipping residual blocks.
- [x] Publish active-window min/max state spacing and gate oversized state gaps before solving BA.
- [x] Preserve auto-start IMU samples when re-integrating preintegration blocks with updated bias.
- [x] Publish Schur marginalization vs fallback prior counters for sliding-window BA.
- [x] Reject and publish linearization/linear-solve failures instead of silently breaking the BA loop.
- [x] Bound SE3 photometric sample weights to `(0, 1]` before accumulating Hessians.
- [x] Publish accepted sliding-window feedback count, stamp, and correction magnitudes.
- [x] Validate dense marginalization-prior stamp/reference ordering and reject duplicate or mismatched retained states.
- [x] Replace same-stamp pose/state priors instead of accumulating duplicate residual weight.
- [x] Replace duplicate IMU spans and smoothness triplets while preserving legitimate same-frame LiDAR/visual residual blocks.
- [x] Publish duplicate IMU span and smoothness triplet replacement counters through `/gaussian_lic/frontend/status`.
- [x] Replace same-source LiDAR point/plane, visual-alignment, and SE3 photometric window factors by `(stamp, source_id)` so repeated rosbag frames or relinearized runtime observations do not silently double-weight BA residuals.
- [x] Publish same-source LiDAR/visual/SE3 factor replacement counters through `/gaussian_lic/frontend/status` and offline metrics.
- [x] Reject non-finite LiDAR point/plane correspondences, zero plane normals, and invalid robust weights before they enter the BA normal equation.
- [x] Bound LiDAR/Gaussian robust correspondence weights to `(0, 1]` so bad external factors cannot amplify outliers.
- [x] Filter non-finite LiDAR keyframe/factor samples and reject invalid-pose LiDAR factors before they contaminate the map or BA window.
- [x] Reject non-finite LiDAR factor configuration values before runtime correction/factor construction.
- [x] Reject non-finite IMU preintegration samples, non-finite residual states, and zero-norm IMU residual quaternions before BA consumes them.
- [x] Reject non-finite sliding-window states, priors, dense priors, factor weights, smoothness weights, and invalid quaternions before normal-equation construction.
- [x] Publish rendered/depth cache hit diagnostics, pending visual-factor stale drops, and SE3 photometric depth/gradient/residual rejection counters through `TrackingStatus`.
- [x] Skip non-finite visual residual pixels, non-finite SE3 photometric sample weights, invalid intrinsics, and invalid camera-to-body adjoints before photometric BA.
- [x] Add analytic SE3 camera photometric pixel Jacobians and multi-sample normal equations with finite-difference CTest coverage.
- [x] Transform SE3 photometric camera-frame Hessians through the camera-to-IMU adjoint into body-frame sqrt-information before adding sliding-window BA factors.
- [x] Default-enable native sliding-window BA plus visual-alignment and SE3 photometric window factors in `tracking.launch.py`, while preserving bounded best-effort QoS and serialized callbacks.
- [x] Feed optimized sliding-window pose/velocity/bias back into odometry, the continuous-time trajectory-control cache, and safe IMU propagation re-anchoring, and expose cubic B-spline pose/velocity trajectory queries for deskew fallback.
- [x] Reject invalid optimized sliding-window states before odometry/IMU feedback and reject invalid trajectory-control poses before B-spline deskew use.
- [x] Publish and smoke-gate optimized IMU re-anchor and B-spline trajectory-control runtime status fields.
- [x] Add optional SE3 photometric pose factors to the sliding-window optimizer with deterministic CTest coverage.
- [x] Use SO(3) log-map rotation residuals and left-Jacobian inverse blocks for pose/state/dense/SE3 priors, with nonzero-residual finite-difference CTest coverage.
- [x] Use SO(3) log-map residuals and left-Jacobian inverse blocks for IMU preintegration rotation rows, with finite-difference CTest coverage for nonzero IMU errors and bias sensitivity.
- [x] Extract runtime SE3 photometric window factors from rendered/current/depth images and gate them in native tracking smoke.
- [x] Add robust runtime SE3 photometric sampling gates for depth range, image gradient, residual outliers, Huber weighting, and status-reported sample quality.
- [x] Publish cumulative visual-alignment and SE3 photometric window-factor totals so optimized runs still prove visual BA usage after active-window trimming.
- [x] Add optional visual-alignment factors to the tracking window with deterministic CTest coverage.
- [x] Add default-enabled three-state continuous-time trajectory smoothness factors to the tracking window, with analytic linear Jacobian blocks, deterministic CTest, and native tracking smoke coverage.
- [x] Publish and smoke-gate sliding-window numeric-Jacobian fallback block/column counts, and keep smoothness rotation rows off the global residual-rebuild path.
- [x] Replace trajectory smoothness rotation local finite-difference blocks with closed-form SO(3) left-Jacobian inverse blocks and deterministic finite-difference CTest coverage.
- [x] Tighten native tracking smoke's default sliding-window optimization budget to 1000 ms under the optimized `RelWithDebInfo` build path.
- [x] Isolate native tracking smoke status/log artifacts per ROS domain and process so parallel BA smoke runs cannot race on `/tmp` state files.
- [x] Subscribe native tracking to mapper `GaussianArray` snapshot chunks and cache chunk-complete Gaussian-map snapshots for the reverse channel.
- [x] Build optional Gaussian-map point-to-point tracking-window factors from cached `GaussianArray` snapshots.
- [x] Register native tracking probes with CTest so `colcon test` runs trajectory, IMU, LiDAR, sliding-window, bias-observability, Gaussian snapshot, and visual checks.
- [x] Replace dynamic reconfigure/global parameters with ROS2 parameters.
- [x] Publish odometry, path, TF, and Gaussian mapper input topics natively.
- [x] Keep any ROS1 bridge mode clearly marked as temporary.

## Milestone 4: Usability

- [x] Docker image with NVIDIA runtime support for the non-torch ROS2 smoke-test path.
- [x] Add RViz2 launch option and default visualization config.
- [x] One-command synthetic smoke-test script.
- [x] One-command synthetic rosbag2 generation and replay smoke test.
- [x] Screenshots and short demo video.
- [x] Common errors page.
- [x] Status schema page.
- [x] Offline rosbag2 artifact extraction CLI skeleton.
- [x] Performance regression comparison script.
- [x] Jazzy-only GitHub Actions CI matrix.
- [x] Python-only CI artifact gates for baseline manifests and reproduction reports.
- [x] FAST-LIVO2 data fetch/readiness scripts for executable baseline status.
- [x] Reproducible ROS1 upstream baseline build-attempt script for the local Noetic Docker environment.
- [x] Add a ROS2 current-results collector that records mapper outputs and writes `trajectory.tum`, `point_cloud.ply`, and `metrics.json`.
- [x] Add `--strict` reproduction readiness that fails on missing PSNR/SSIM/LPIPS and enforces the 5% paper metric gate.
- [x] Jazzy CI smoke tests with a generated mini rosbag2 sequence.
- [x] ROS1 upstream baseline archive for the official curated FAST-LIVO2 `Bright_Screen_Wall_curated_8s` substitute.
- [x] ROS1 `.bag` to rosbag2 `.mcap` converter backend.
- [x] PR comparison report on real or curated mini-sequence artifacts.
- [x] Resumable strict `CBD_Building_01` pipeline entrypoint from official bag to strict report.
- [x] Strict `CBD_Building_01` ROS1 upstream baseline archive from the official local bag.
- [x] Strict report quality extraction fills PSNR/SSIM/LPIPS from render/GT pairs, with lpipsPyTorch fallback for CPU report environments.
- [x] Strict rasterizer current collection forwards CUDA optimization and pruning controls so runtime does not silently run zero optimizer steps.
- [x] Strict ROS1 baseline visual dump continues after LPIPS runtime failures, so render/GT pairs are not capped at the first frame.
- [x] Strict ROS2 current collection saves final-map train/test re-renders from `SaveMap` instead of judging the live preview topic.
- [x] Strict `CBD_Building_01` current collection uses the FAST-LIVO2 profile cadence (`select_every_k_frame=5`) so ROS2 train/test render names align with the ROS1 upstream baseline.
- [x] Strict frontend adapter matches the ROS1 mapper-contract fallback by rotating FAST-LIVO2 point clouds into the IMU-pose world frame before the mapper consumes them.
- [x] Strict current collection disables high-rate `GaussianArray` visualization publication and uses a 1.5M foreground cap, reducing the local output bag from 38.7 GiB to 3.7 GiB while preserving metric artifacts.
- [x] Strict current collection supports foreground count-cap policy selection and defaults the strict CUDA run to uniform pruning, preserving insertion-order spatial coverage better than opacity-only top-k pruning.
- [x] Strict CUDA extension filters pending points to current-view alpha holes before appending, matching the upstream Gaussian-LIC `extend()` visibility semantics instead of adding every synchronized LiDAR sample.
- [x] Strict CUDA current collection records the PyTorch allocator policy in `metrics.json` and defaults CUDA runs to `PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True` to reduce late-run fragmentation OOMs around million-Gaussian maps.
- [x] Strict CUDA current collection disables the ROS2-only periodic opacity reset by default, matching released Gaussian-LIC2 optimizer behavior and preventing final-map renders from being saved right after all foreground opacity is reset to 0.01.
- [x] Strict CUDA current collection disables ROS2 gradient split/clone densification by default, matching released Gaussian-LIC2 append-only `extend()` and avoiding the large blurred splats seen when the diagnostic split/clone path is forced on.
- [x] Strict report treats unavailable generic runtime performance keys as skipped when quality/trajectory/point-cloud paper gates are present, so `tracking_hz`/`mapping_hz` absence does not mask the real paper gate failures.
- [x] Strict report samples render pairs evenly across the full sequence instead of only the first sorted frames, and the point-cloud count gate now uses declared PLY vertex counts rather than the downsampled load sample.
- [x] Packaged dataset profiles and launch defaults are CUDA/rasterizer-first, while `collect_current_results.sh` still explicitly disables Torch when the caller omits `--torch`.
- [x] Strict CUDA optimization uses upstream-style random train-camera sampling plus shuffle by default, with a fixed seed for reproducible ROS2 reports and `seed=0` available for upstream `random_device` behavior.
- [x] ROS2 fallback projected-depth generation now matches the ROS1 mapper-contract converter's nearest-pixel rounding semantics; RGB projection remains floor-based because the ROS1 converter uses floor for color.
- [x] Strict report uses current saved GT frames for ROS2 quality extraction and decoded-GT hash association for ROS1-vs-ROS2 render-pair comparison, avoiding false failures when rosbag2 replay drops or reorders frame names relative to the ROS1 baseline.
- [x] Strict CUDA current defaults to no opacity pruning for released LIC2 append-only parity; foreground count limiting remains available through the uniform count-cap policy.
- [x] Strict `CBD_Building_01` ROS2 current report with trajectory coverage, render pairs, PSNR/SSIM/LPIPS, and Chamfer within the paper gate.
  - 2026-05-05 archived strict run at `results/fastlivo2/CBD_Building_01_current_round_no_opacity_prune_probe/` passes `reproduction_report.py --strict`.
  - Trajectory gate passes with 1063 matched poses, 89.63% coverage, zero translation RMSE, and zero path drift.
  - Novel-view quality passes: ROS2 PSNR 12.6542 dB vs ROS1 12.7036 dB passes at 0.39% regression, ROS2 SSIM 0.368961 vs ROS1 0.3644 passes, and ROS2 LPIPS 0.741661 vs ROS1 0.75136 passes.
  - GT-associated render-pair smoke gate passes with 1063 matched frames, 64 sampled pairs, mean PSNR 21.600 dB, mean SSIM 0.785417, min PSNR 17.448 dB, and min SSIM 0.477513.
  - Point-cloud parity passes: count ratio 0.978, centroid drift 0.147855 m, bidirectional nearest RMSE 0.145491 m, and bidirectional nearest mean 0.099418 m against the 0.100000 m strict threshold.
- [x] Strict visual artifacts generated for release review.
  - `docs/assets/strict_cbd_montage.jpg` shows sampled ROS2 current, GT, and ROS1 baseline frames.
  - `docs/assets/strict_cbd_render_demo.gif` is a compact current-vs-GT render demo from the strict run.
- [x] Add a real frontend-raw native tracking report script that records odometry/status/mapper-contract outputs and gates signed-time, IMU, LiDAR, sliding-window, and numeric-Jacobian health.
  - 2026-05-06 local Bright 8s run at `results/fastlivo2/Bright_Screen_Wall_native_tracking_8s/native_tracking_report.json` passes with 20 odometry poses, 21 mapper point frames, 20 status samples, 3 IMU factors, 87,779 normal-equation rows, and zero numeric-Jacobian fallback.
  - 2026-05-06 local CBD 8s run at `results/fastlivo2/CBD_Building_01_native_tracking_8s/native_tracking_report.json` passes with 22 odometry poses, 23 mapper point frames, 22 status samples, 3 IMU factors, 65,611 normal-equation rows, and zero numeric-Jacobian fallback.
- [x] Add optional reference-trajectory native tracking evidence with explicit external-odometry pose priors.
  - The native report now records `reference_trajectory.tum`, can require `trajectory_compare.py` against `/gaussian_lic/frontend/input_odometry`, and keeps the odometry prior off by default so sensor-only tracking is not silently assisted.
  - 2026-05-06 synthetic frontend-raw visual run with `--enable-external-odometry-prior --require-reference-trajectory` passes with 32 native poses, 32 reference poses, 25 external-prior matches, 100.00% trajectory coverage, and 0.101496 m RMSE.
- [x] Add a native tracking report non-degenerate BA gate and a real sensor-only Bright pass.
  - `run_native_tracking_bag_report.sh` now exposes LiDAR factor budget, BA iteration, max state-gap, condition-number, and `--require-nondegenerate-ba` controls.
  - 2026-05-06 local Bright 12s sensor-only run at `results/fastlivo2/Bright_Screen_Wall_native_tracking_nondegenerate_12s/native_tracking_report.json` passes with 39 odometry poses, 39 mapper point frames, 6 IMU factors, `tracking_with_sliding_window`, non-degenerate state cadence, non-degenerate normal equations, 1 accepted BA step, 37 feedback updates, 7 pointcloud/IMU wait-queue deferrals, 7 releases, zero wait drops, 29,565 normal-equation rows, condition number 2.116e9, and zero numeric-Jacobian fallback.
- [x] Truncate overrun IMU preintegrations at point-cloud stamps instead of dropping valid late-arriving factors.
  - `ImuPreintegrator::truncated()` reconstructs the segment with an interpolated boundary sample, and `run_native_tracking_bag_report.sh --require-nondegenerate-ba` now fails on IMU factor/time-gap skips.
  - 2026-05-06 local Bright 12s sensor-only run at `results/fastlivo2/Bright_Screen_Wall_native_tracking_truncated_imu_gate_12s/native_tracking_report.json` passes with 38 odometry poses, 38 mapper point frames, 36 IMU factors, zero IMU factor skips, zero IMU time-gap skips, 2 accepted BA steps, 36 feedback updates, 7 pointcloud/IMU wait-queue deferrals, 7 releases, zero wait drops, 30,465 normal-equation rows, condition number 5.773e5, and zero numeric-Jacobian fallback.
- [x] Add mapper-rendered feedback and sparse LiDAR-projected depth evidence for real-bag visual/SE3 photometric tracking factors.
  - `run_native_tracking_bag_report.sh --enable-mapper-feedback` launches `mapping_node` beside tracking, enables visual gates, uses FAST-LIVO2 LiDAR/camera extrinsics by default, and requires nonzero visual plus SE3 photometric factors when visual factors are enabled.
  - 2026-05-06 local Bright 12s run at `results/fastlivo2/Bright_Screen_Wall_native_tracking_visual_sparse_depth_extrinsic_12s/native_tracking_report.json` passes with 36 odometry poses, 36 mapper point frames, 35 IMU factors, 2 visual-alignment factors, 2 SE3 photometric factors, 68 SE3 samples, 4 sparse projected-depth frames in cache, zero IMU factor/time-gap skips, 33,855 normal-equation rows, condition number 8.363e5, and zero numeric-Jacobian fallback.
- [x] Keep mapper-rendered visual feedback alive under real asynchronous callback latency.
  - Tracking now caches observed camera frames, processes visual pairs from either image arrival order, and attaches delayed visual/SE3 photometric factors to the closest active sliding-window state instead of dropping them against the current point-cloud stamp.
  - 2026-05-06 local Bright 30s run at `results/fastlivo2/Bright_Screen_Wall_native_tracking_visual_sparse_depth_extrinsic_windowed_visual_30s/native_tracking_report.json` passes with 122 odometry poses, 122 mapper point frames, 121 IMU factors, zero IMU factor/time-gap skips, zero invalid optimized states, 64 visual-alignment factors, 64 SE3 photometric factors, 85 SE3 samples, zero visual/SE3 pending stale drops, 22,588 normal-equation rows, condition number 6.532e6, rank ratio 1.0, and zero numeric-Jacobian fallback.
- [x] Add explicit scan-order LiDAR deskew diagnostics for legacy bags without per-point time fields.
  - Default tracking still refuses to fabricate point timestamps. `run_native_tracking_bag_report.sh --enable-scan-order-deskew --require-deskew` opts into centered scan-order offsets and gates nonzero deskew lookup queries/hits.
  - 2026-05-06 local Bright 12s run at `results/fastlivo2/Bright_Screen_Wall_native_tracking_scan_order_deskew_hits_12s/native_tracking_report.json` passes with 36 odometry poses, 36 mapper point frames, 35 IMU factors, max synthetic point-time offset 0.05 s, 864,000 deskew queries, 762,866 deskew hits, zero invalid/out-of-range point times, zero IMU factor/time-gap skips, 31,137 normal-equation rows, condition number 1.035e6, and zero numeric-Jacobian fallback.
  - 2026-05-06 local Bright 30s combined visual+deskew run at `results/fastlivo2/Bright_Screen_Wall_native_tracking_visual_scan_order_deskew_gate_30s/native_tracking_report.json` passes with 112 odometry poses, 112 mapper point frames, 111 IMU factors, 59 visual-alignment factors, 9 SE3 photometric factors, 2,688,000 deskew queries, 2,616,215 deskew hits, zero invalid/out-of-range point times, zero IMU factor/time-gap skips, zero invalid optimized states, 17,908 normal-equation rows, condition number 5.044e6, rank ratio 1.0, and zero numeric-Jacobian fallback.
- [x] Queue delayed visual/SE3 BA factors and preserve IMU-before-pointcloud causal order under asynchronous ROS2 replay.
  - Tracking now bounds delayed visual-alignment and SE3 photometric factor queues with `visual_pending_factor_queue_size`, consumes every still-fresh queued factor against the closest active window state, and only releases point clouds waiting for IMU after the current IMU sample has been propagated and added to preintegration.
  - 2026-05-06 local Bright 30s visual+scan-order deskew run at `results/fastlivo2/Bright_Screen_Wall_native_tracking_visual_scan_order_deskew_queue_imu_order_30s/native_tracking_report.json` passes with 109 odometry poses, 109 mapper point frames, 107 IMU factors, 64 visual-alignment factors, 16 SE3 photometric factors, 2,616,000 deskew queries, 2,552,484 deskew hits, zero invalid/out-of-range point times, zero IMU factor/time-gap skips, zero visual/SE3 pending stale drops, zero invalid optimized states, 14,730 normal-equation rows, condition number 8.467e6, rank ratio 1.0, and zero numeric-Jacobian fallback.
- [x] Preserve distinct delayed visual observations when several image pairs attach to the same active BA state.
  - Visual-alignment and SE3 photometric window factors now use a deterministic source id derived from the observed/rendered stamp pair instead of a fixed source id, so duplicate processing of the same pair still replaces while distinct image pairs do not overwrite each other after active-window stamp snapping.
  - 2026-05-06 local Bright 30s visual+scan-order deskew run at `results/fastlivo2/Bright_Screen_Wall_native_tracking_visual_scan_order_deskew_source_id_30s/native_tracking_report.json` passes with 110 odometry poses, 110 mapper point frames, 108 IMU factors, 56 visual-alignment factors, 11 SE3 photometric factors, zero visual/SE3 factor replacements, zero pending visual/SE3 queue backlog, 2,640,000 deskew queries, 2,562,222 deskew hits, zero IMU factor/time-gap skips, zero invalid optimized states, 18,064 normal-equation rows, condition number 9.500e6, rank ratio 1.0, and zero numeric-Jacobian fallback.
- [x] Gate CI as Jazzy-only so Humble cannot re-enter the build matrix accidentally.
  - `check_ros2_semantics_contract.py` now fails if `.github/workflows/ci.yaml` contains `humble` or if the ROS build matrix is not exactly `ros_distro: [jazzy]`.
- [x] Gate native tracking real-bag reports on visual pending-factor queue health.
  - `run_native_tracking_bag_report.sh --enable-mapper-feedback` now requires pending visual/SE3 queue-size fields, bounds them by the configured report queue budget, and fails on pending stale drops.
  - 2026-05-06 local Bright 12s visual+scan-order deskew run at `results/fastlivo2/Bright_Screen_Wall_native_tracking_visual_queue_gate_12s/native_tracking_report.json` passes with 8 visual-alignment factors, 7 SE3 photometric factors, zero visual/SE3 replacements, zero pending queue backlog, zero pending stale drops, 840,000 deskew queries, 778,367 deskew hits, zero IMU factor/time-gap skips, 30,503 normal-equation rows, condition number 1.183e6, and zero numeric-Jacobian fallback.
  - 2026-05-06 local Bright 60s visual+scan-order deskew run at `results/fastlivo2/Bright_Screen_Wall_native_tracking_visual_scan_order_deskew_source_id_queue_gate_60s/native_tracking_report.json` passes with 272 odometry poses, 272 mapper point frames, 270 IMU factors, 155 visual-alignment factors, 10 SE3 photometric factors, zero visual/SE3 replacements, zero pending visual/SE3 backlog or stale drops, 6,528,000 deskew queries, 6,470,281 deskew hits, zero IMU factor/time-gap skips, zero invalid optimized states, 271 feedback updates, 7,679 normal-equation rows, condition number 1.709e7, rank ratio 1.0, and zero numeric-Jacobian fallback.
- [x] Sample SE3 photometric BA from valid sparse-depth pixels instead of uniform image strides.
  - Tracking now optionally dilates sparse LiDAR-projected depth before SE3 photometric sampling, samples from finite in-range depth pixels, exposes a depth-specific freshness window that defaults to the visual pairing window, publishes cumulative SE3 photometric batch/sample diagnostics, and makes the real-bag visual report fail unless it sees valid SE3 photometric batches and cumulative samples.
  - 2026-05-06 local Bright 30s visual+scan-order deskew run at `results/fastlivo2/Bright_Screen_Wall_native_tracking_visual_valid_depth_sampling_default_30s/native_tracking_report.json` passes with 84 odometry poses, 85 mapper point frames, 83 IMU factors, 45 visual-alignment factors, 11 SE3 photometric factors, 45 SE3 photometric batches, 11 valid SE3 batches, 18,092 cumulative SE3 samples, sparse depth dilation radius 5 px, zero visual/SE3 pending backlog or stale drops, zero IMU factor/time-gap skips, 40,817 normal-equation rows, condition number 2.478e7, and zero numeric-Jacobian fallback.
  - 2026-05-06 local Bright 60s visual+scan-order deskew run at `results/fastlivo2/Bright_Screen_Wall_native_tracking_visual_valid_depth_sampling_default_60s/native_tracking_report.json` passes with 195 odometry poses, 195 mapper point frames, 194 IMU factors, 99 visual-alignment factors, 12 SE3 photometric factors, 99 SE3 photometric batches, 12 valid SE3 batches, 21,627 cumulative SE3 samples, sparse depth dilation radius 5 px, zero visual/SE3 pending backlog or stale drops, 4,680,000 deskew queries, 4,668,986 deskew hits, zero IMU factor/time-gap skips, zero invalid optimized states, 194 feedback updates, 20,489 normal-equation rows, condition number 1.391e7, rank ratio 1.0, and zero numeric-Jacobian fallback.
  - 2026-05-06 local Bright 30s split-window run at `results/fastlivo2/Bright_Screen_Wall_native_tracking_visual_depth_window_split_default_30s/native_tracking_report.json` passes with `visual_factor_max_dt_ns=300000000`, `visual_depth_max_dt_ns=0`, 45 visual-alignment factors, 10 SE3 photometric factors, 45 SE3 batches, 10 valid SE3 batches, 19,176 cumulative SE3 samples, zero visual/SE3 pending stale drops, zero IMU factor/time-gap skips, 44,808 normal-equation rows, condition number 7.019e6, and zero numeric-Jacobian fallback.
- [x] Gate SE3 photometric BA factors by Hessian rank and condition number.
  - Tracking now publishes current and last-accepted SE3 photometric Hessian rank, singular values, and condition number; only sample-sufficient, Hessian-healthy SE3 factors enter the sliding window.
  - 2026-05-06 local Bright 30s visual+scan-order deskew run at `results/fastlivo2/Bright_Screen_Wall_native_tracking_se3_hessian_gate_30s/native_tracking_report.json` passes with 87 odometry poses, 88 mapper point frames, 85 IMU factors, 46 visual-alignment factors, 10 SE3 photometric factors, 46 SE3 batches, 10 valid SE3 batches, 0 degenerate SE3 batches, 19,176 cumulative SE3 samples, last accepted SE3 Hessian rank 6, last accepted SE3 Hessian condition 7.240e3, 2,088,000 deskew queries, 2,031,012 deskew hits, zero IMU factor/time-gap skips, 85 feedback updates, 40,555 normal-equation rows, condition number 9.543e6, rank ratio 1.0, and zero numeric-Jacobian fallback.
- [x] Gate SE3 photometric BA factors by sparse-depth sample inlier quality.
  - Tracking now publishes current and last-accepted sampled-depth counts, accepted-sample counts, sample inlier ratio, mean absolute residual, step norm, and quality-rejected batch counters; sample-sufficient and Hessian-healthy SE3 factors are still rejected if the accepted/sampled sparse-depth ratio falls below the configured gate.
  - 2026-05-06 local Bright 30s visual+scan-order deskew run at `results/fastlivo2/Bright_Screen_Wall_native_tracking_se3_sample_quality_gate_30s/native_tracking_report.json` passes with 82 odometry poses, 83 mapper point frames, 80 IMU factors, 73 visual-alignment factors, 22 SE3 photometric factors, 73 SE3 batches, 22 valid SE3 batches, 0 degenerate or quality-rejected SE3 batches, 39,321 cumulative SE3 samples, last accepted sampled-depth count 446, last accepted accepted-sample count 422, last accepted sample inlier ratio 0.946, last accepted SE3 Hessian rank 6, last accepted SE3 Hessian condition 2.439e7, 1,968,000 deskew queries, 1,906,016 deskew hits, zero IMU factor/time-gap skips, 80 feedback updates, 40,801 normal-equation rows, condition number 1.877e7, rank ratio 1.0, and zero numeric-Jacobian fallback.
- [x] Gate SE3 photometric BA factors by spatial sample coverage.
  - Tracking now partitions accepted SE3 photometric sparse-depth samples across a configurable image grid and rejects otherwise sample-sufficient, Hessian-healthy batches when accepted samples occupy too few tiles; current and last-accepted occupied/total coverage tiles are published in `TrackingStatus` and gated by the native tracking report.
  - 2026-05-06 local Bright 30s visual+scan-order deskew run at `results/fastlivo2/Bright_Screen_Wall_native_tracking_se3_coverage_gate_30s/native_tracking_report.json` passes with 82 odometry poses, 82 mapper point frames, 81 IMU factors, 43 visual-alignment factors, 12 SE3 photometric factors, 43 SE3 batches, 12 valid SE3 batches, 0 degenerate batches, 1 quality-rejected batch, 24,210 cumulative SE3 samples, 4x4 coverage grid with a 4-tile minimum, last accepted sampled-depth count 1,962, last accepted accepted-sample count 1,837, last accepted sample inlier ratio 0.936, last accepted coverage 4/16 tiles, last accepted SE3 Hessian rank 6, last accepted SE3 Hessian condition 9.787e3, 1,968,000 deskew queries, 1,924,727 deskew hits, zero IMU factor/time-gap skips, 81 feedback updates, 39,122 normal-equation rows, condition number 4.754e6, rank ratio 1.0, and zero numeric-Jacobian fallback.
- [x] Use tile-balanced SE3 photometric sampling before the spatial coverage gate.
  - Accepted sparse-depth candidates are now bucketed by the configured coverage grid, assigned a balanced per-tile quota, and sampled evenly inside each occupied tile before gradient/residual/Hessian gates are evaluated; this avoids row-major stride sampling starving smaller image regions.
  - 2026-05-06 local Bright 30s visual+scan-order deskew run at `results/fastlivo2/Bright_Screen_Wall_native_tracking_se3_balanced_sampling_30s/native_tracking_report.json` passes with 83 odometry poses, 83 mapper point frames, 82 IMU factors, 56 visual-alignment factors, 15 SE3 photometric factors, 56 SE3 batches, 15 valid SE3 batches, 0 degenerate batches, 4 quality-rejected batches, 35,004 cumulative SE3 samples, 4x4 coverage grid with a 4-tile minimum, last accepted sampled-depth count 2,000, last accepted accepted-sample count 1,899, last accepted sample inlier ratio 0.950, last accepted coverage 5/16 tiles, last accepted SE3 Hessian rank 6, last accepted SE3 Hessian condition 7.467e3, zero IMU factor/time-gap skips, 82 feedback updates, 49,171 normal-equation rows, condition number 1.143e7, rank ratio 1.0, and zero numeric-Jacobian fallback.
- [x] Add an executable full-dataset strict parity evidence matrix.
  - `scripts/check_strict_parity_matrix.py` validates required reproduction and native-tracking evidence from `docs/strict_parity_matrix.json`; the default release gate exits nonzero until every required dataset/profile item is backed by archived strict artifacts.
  - Current local matrix status is `required=2/7`, `covered_profiles=fastlivo2`: the FAST-LIVO2 `CBD_Building_01` mapper-contract/CUDA strict report and CBD native visual/SE3 BA health report pass, while native reference trajectory parity plus FAST-LIVO/M2DGR/MCD/R3LIVE full-sequence strict artifacts are still missing.
- [x] Add a full-sequence CBD native visual/SE3 production-BA health report.
  - `run_native_tracking_bag_report.sh` now exposes `--mapper-feedback-sync-tolerance-sec` for mapper feedback under asynchronous replay, accepts `--reference-tum` for external reference-trajectory comparison, and avoids recorder shutdown noise when ROS2 tears down the context.
  - 2026-05-06 local CBD 120s visual+scan-order deskew run at `results/fastlivo2/CBD_Building_01_native_tracking_visual_ba_sync50ms_120s/native_tracking_report.json` passes with 478 odometry poses, 478 mapper point frames, 477 IMU factors, 238 visual-alignment factors, 27 SE3 photometric factors, 238 SE3 batches, 27 valid SE3 batches, 0 degenerate batches, 21 quality-rejected batches, 76,250 cumulative SE3 samples, 64 rendered/depth cache slots, 2 rendered misses, last accepted sample inlier ratio 0.935, last accepted coverage 6/16 tiles, last accepted SE3 Hessian rank 6, last accepted SE3 Hessian condition 6.187e3, 11,472,000 deskew queries, 11,430,279 deskew hits, zero IMU factor/time-gap skips, 477 feedback updates, 6,258 normal-equation rows, condition number 5.818e6, rank ratio 1.0, and zero numeric-Jacobian fallback.
- [x] Add R3LIVE real-bag native tracking runtime coverage.
  - `hku_park_00.bag` converts to ROS2 frontend-raw with 3,428 images, 2,285 LiDAR frames, and 46,599 IMU samples; the native report gate now treats cumulative LiDAR point/plane correspondences as valid long-run evidence when the final active window has already marginalized current LiDAR factors.
  - 2026-05-06 local R3LIVE 60s sensor-only scan-order deskew run at `results/r3live/hku_park_00_native_tracking_scan_order_60s/native_tracking_report.json` passes with 554 odometry poses, 554 mapper point frames, 553 IMU factors, 18,004 cumulative point correspondences, 7,474 cumulative plane correspondences, 10 smoothness factors in the last window, 13,296,000 deskew queries, 8,542,971 deskew hits, zero IMU factor/time-gap skips, 553 feedback updates, 567 normal-equation rows, condition number 3.317e4, rank ratio 1.0, and zero numeric-Jacobian fallback.
- [x] Add FAST-LIVO2 Retail_Street real-bag native tracking runtime coverage.
  - `Retail_Street.bag` downloads from the official FAST-LIVO2 Google Drive index with 1,961,140,106 bytes, converts to ROS2 frontend-raw with 1,355 images, 1,355 LiDAR frames, and 27,447 IMU samples, and passes the `frontend_sensor_raw` bag contract.
  - 2026-05-06 local Retail 60s sensor-only scan-order deskew run at `results/fastlivo2/Retail_Street_native_tracking_scan_order_60s/native_tracking_report.json` passes with 197 odometry poses, 197 mapper point frames, 195 IMU factors, 103,677 cumulative point correspondences, 73,962 cumulative plane correspondences, 4,728,000 deskew queries, normal-equation rank ratio 1.0, and zero numeric-Jacobian fallback.
- [x] Add profile-aware ROS1 frontend-raw conversion and a data-side strict audit.
  - `scripts/ros1_to_frontend_raw.py` wraps the profile-aware converter for FAST-LIVO, FAST-LIVO2, M2DGR, MCD, and R3LIVE; the older `fastlivo2_ros1_to_frontend_raw.py` entrypoint remains compatible. The converter chooses per-profile topic fallbacks and intrinsics, supports comma-separated multi-bag inputs for split datasets, supports both Livox `CustomMsg` and source `sensor_msgs/PointCloud2` LiDAR bags, and normalizes the output to the ROS2 `frontend_sensor_raw` contract.
  - 2026-05-06 local 1s conversion probes pass the `frontend_sensor_raw` bag contract for FAST-LIVO2 `Bright_Screen_Wall` and R3LIVE `hku_park_00`.
  - `scripts/audit_strict_data_inputs.py` writes `docs/strict_data_status.json` / `.md`, checks raw bags, converted frontend bags, ROS1 baselines, ROS2 current artifacts, trusted native reference trajectories, disk headroom, and the largest non-matrix `results/` cleanup candidates. After pruning obsolete non-matrix `results/` artifacts, fetching `Retail_Street`, and adding partial MCD components, the current audit shows `/home/frank/data` has 58.82 GiB free; FAST-LIVO2 `Retail_Street` now has raw/frontend/current runtime artifacts, while FAST-LIVO/M2DGR and the complete MCD camera+LiDAR+IMU raw input set are still not local.
- [x] Add generic Google Drive single-file fetcher and partial MCD data acquisition.
  - `scripts/fetch_google_drive_file.py` reuses the repository Google Drive warning-page handling, byte-count checks, manifests, and free-space gate for arbitrary public file IDs.
  - 2026-05-06 local MCD `ntu_day_01` acquisition now has `mid70` LiDAR (`568,196,474` bytes) and `vn100` IMU (`33,957,020` bytes) under `/home/frank/data/mcd/ntu_day_01`; the data audit deliberately keeps MCD failing until the `d435i` camera component, frontend conversion, baseline, current result, and trusted reference trajectory are present.
- [ ] Collect and pass full native reference trajectory parity for CBD and every required non-FAST-LIVO2 dataset/profile.
  - The attempted CBD comparison against `baseline/fastlivo2/CBD_Building_01/trajectory.tum` is not accepted as proof because that archived mapper-contract TUM is not a valid native frontend reference for the frontend-raw bag; the full-dataset matrix keeps this item failing until a trusted reference odometry/TUM source is available and passes `trajectory_compare.py`.
- [x] Local SPNet TensorRT engine benchmark passes the runtime target.
  - TensorRT 10.9 CUDA 12.8 builds `/home/frank/Software/TensorRT-engines/spnet_512_640_fp16.engine` for the RTX 5070 Ti `sm_120` GPU.
  - Mean `trtexec` latency is 26.4492 ms at `512x640`, below the 30 ms/frame target.
