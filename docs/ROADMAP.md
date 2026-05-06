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
- [x] Extract runtime SE3 photometric window factors from rendered/current/depth images and gate them in native tracking smoke.
- [x] Add robust runtime SE3 photometric sampling gates for depth range, image gradient, residual outliers, Huber weighting, and status-reported sample quality.
- [x] Publish cumulative visual-alignment and SE3 photometric window-factor totals so optimized runs still prove visual BA usage after active-window trimming.
- [x] Add optional visual-alignment factors to the tracking window with deterministic CTest coverage.
- [x] Add default-enabled three-state continuous-time trajectory smoothness factors to the tracking window, with analytic linear Jacobian blocks, deterministic CTest, and native tracking smoke coverage.
- [x] Publish and smoke-gate sliding-window numeric-Jacobian fallback block/column counts, and keep smoothness rotation rows off the global residual-rebuild path.
- [x] Tighten native tracking smoke's default sliding-window optimization budget to 1000 ms under the optimized `RelWithDebInfo` build path.
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
- [x] Local SPNet TensorRT engine benchmark passes the runtime target.
  - TensorRT 10.9 CUDA 12.8 builds `/home/frank/Software/TensorRT-engines/spnet_512_640_fp16.engine` for the RTX 5070 Ti `sm_120` GPU.
  - Mean `trtexec` latency is 26.4492 ms at `512x640`, below the 30 ms/frame target.
