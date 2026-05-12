# Roadmap

The public release plan is milestone-based:

- `v0.1.0` Baseline & Infrastructure
- `v0.2.0` Gaussian-LIC2 ROS2 Native Frontend
- `v0.3.0` Gaussian Mapping Full Backend
- `v0.4.0` Strict FAST-LIVO2 Reproduction

See `docs/RELEASE_MILESTONES.md` for release criteria and artifacts.

## Current Truth

The current repository is a strong ROS2 plumbing, mapping, native-tracking, continuous-time tracker, and reproducibility checkpoint with the required local strict parity matrix passing. It is still deliberately evidence-driven: claims beyond the archived matrix should be backed by additional dataset-specific reports.

Completed proof/evidence scope:

- ROS2 workspace, messages, launch, composable mapper node, dataset profiles, bag contracts, artifact extraction, and CI artifact gates.
- Native mapper input contract and `lic2_contract_adapter` boundary for raw camera/LiDAR/IMU plus pose/odometry.
- Optional Torch Gaussian tensor path with keyframe initialization, skybox seeding, foreground append, gradient-aware densification, multi-criteria pruning, opacity reset scheduling, CUDA rasterizer preview, CUDA rasterizer/fused-SSIM/depth loss, visibility-masked SparseGaussianAdam updates for the full Gaussian parameter set, and optional TensorRT/SPNet depth completion.
- Continuous-time ROS2 tracker producer chain with Ceres cumulative B-spline trajectory estimation, continuous-time IMU/LiDAR/photometric residual surfaces, voxel-plane extraction, TUM prior publishing, and required liveness gates across FAST-LIVO2, M2DGR, MCD, and R3LIVE.
- Official FAST-LIVO2 `Bright_Screen_Wall` substitute report with `metrics`, `trajectory`, `point_cloud`, and `gaussian_color` gates passing.
- Strict FAST-LIVO2 `CBD_Building_01` local mapper-contract/CUDA reproduction pipeline with trajectory, PSNR/SSIM/LPIPS, GT-associated render-pair, and point-cloud gates passing against the archived ROS1 upstream baseline.

Incomplete paper-algorithm scope:

- Continuous-time tracker quality is not yet paper-grade RMSE parity: current new matrix entries validate real-bag producer-chain liveness, while estimator drift remains open until persistent world-frame LiDAR map constraints, stronger IMU/gravity/bias initialization, and Gaussian photometric feedback are coupled into the spline window.
- TensorRT/SPNet depth completion has a generated local TensorRT 10.9 FP16 engine path and benchmark; large checkpoint/ONNX/engine artifacts stay outside git.
- The public LIC2 upstream surface does not currently expose separate 2D Gaussian primitive or skybox patches beyond the checked mapper backend.
- Additional native Coco-LIC2 frontend/tracking parity evidence outside the required matrix should be archived as trusted reference trajectories become available.

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
  - Current local matrix status is `PASS`, `required=17/17`, `covered_profiles=fastlivo,fastlivo2,m2dgr,mcd,r3live`: the FAST-LIVO2 `CBD_Building_01` mapper-contract/CUDA strict report, FAST-LIVO hku1/hku2/Visual_Challenge/LiDAR_Degenerate mapper-contract/CUDA strict reports, M2DGR room_01/room_02/room_03 mapper-contract/CUDA strict reports, MCD `ntu_day_01` mapper-contract/CUDA strict report, R3LIVE `hku_park_00` mapper-contract/CUDA strict report, CBD native visual/SE3 BA health report, CBD native reference trajectory parity, and five continuous-time native tracker liveness gates all pass.
  - 2026-05-07: `scripts/run_strict_parity_queue.sh` turns strict-data production into an executable queue across FAST-LIVO, FAST-LIVO2, M2DGR, MCD, and R3LIVE. It performs profile-aware frontend conversion, ROS1 mapper-contract export, profile-matched upstream baseline runs, ROS2 CUDA current collection, strict reports, and data/matrix audits.
  - 2026-05-07: FAST-LIVO hku1/hku2 strict mapper-contract reports pass. hku1 records 1,276/1,277 trajectory coverage, point-cloud count ratio 1.005, full-frame PSNR/SSIM/LPIPS within the 5% gate, and a bounded supplemental render-pair outlier ratio of 5/64. hku2 passes with 1,051 matched poses, zero render-pair outliers, and LPIPS improving relative to the ROS1 baseline.
  - 2026-05-08: FAST-LIVO Visual_Challenge strict mapper-contract/CUDA report passes after the ROS2 adapter switched strict image/camera_info sync to ROS1-style `latest_before` sequencing and the strict queue defaulted to no densification with an 80k foreground cap. The run records 1,602/1,602 trajectory matches, novel PSNR 14.8194 dB vs ROS1 14.9798 dB, novel SSIM 0.602463 vs 0.603409, novel LPIPS 0.563578 vs 0.566402, 64 sampled render pairs with mean PSNR 27.4057 dB and mean SSIM 0.784817, and point-cloud nearest mean 0.022905 m.
  - 2026-05-08: FAST-LIVO LiDAR_Degenerate strict mapper-contract/CUDA report passes with 779/779 trajectory matches, novel PSNR 9.5179 dB vs ROS1 9.5170 dB, novel SSIM 0.009163 vs 0.009109, novel LPIPS 0.646615 vs 0.646225, zero sampled render-pair outliers, and point-cloud nearest mean 0.000006 m.
  - 2026-05-08: M2DGR room_01 strict mapper-contract/CUDA report passes with 700/701 trajectory matches, novel PSNR 5.2965 dB vs ROS1 5.2962 dB, novel SSIM 0.001795 vs 0.001792, novel LPIPS 0.734130 vs 0.733806, zero sampled render-pair outliers, and point-cloud nearest mean 0.004092 m.
  - 2026-05-08: M2DGR room_02 strict mapper-contract/CUDA report passes with 752/752 trajectory matches, novel PSNR 5.3113 dB exactly matching ROS1, novel SSIM 0.001565 exactly matching ROS1, novel LPIPS 0.721426 exactly matching ROS1, zero sampled render-pair outliers, and point-cloud nearest mean 0.001254 m.
  - 2026-05-08: M2DGR room_03 strict mapper-contract/CUDA report passes with 1,277/1,278 trajectory matches, novel PSNR 5.3253 dB vs ROS1 5.3259 dB, novel SSIM 0.001693 vs 0.001694, novel LPIPS 0.692587 vs 0.692454, zero sampled render-pair outliers, and point-cloud nearest mean 0.004084 m.
  - 2026-05-08: MCD `ntu_day_01` strict mapper-contract/CUDA report passes with 6,024/6,024 trajectory matches, novel PSNR 10.9764 dB vs ROS1 10.9636 dB, novel SSIM 0.432230 vs 0.429721, novel LPIPS 0.659652 vs 0.660955, 1,804 GT-matched render pairs evaluated with mean PSNR 28.1503 dB and mean SSIM 0.713952 under the MCD full-sequence sanity thresholds, and point-cloud nearest mean 0.000874 m.
  - 2026-05-08: R3LIVE `hku_park_00` strict mapper-contract/CUDA report passes with 2,285/2,285 trajectory matches, novel PSNR 11.9323 dB vs ROS1 11.8051 dB, novel SSIM 0.347540 vs 0.343424, novel LPIPS 0.727250 vs 0.726637, 2,280 GT-matched render pairs evaluated with mean PSNR 23.6587 dB and mean SSIM 0.742624 under the R3LIVE full-sequence sanity thresholds, and point-cloud nearest mean 0.000196 m.
  - 2026-05-08: FAST-LIVO2 `CBD_Building_01` native reference parity passes against a trusted upstream Coco-LIC trajectory generated from the official ROS1 bag with `external/Coco-LIC/config/ct_odometry_fastlivo2.yaml`. The archived ROS2 native report at `results/fastlivo2/CBD_Building_01_native_tracking_cocolic_reference_imu_scale_full_120s/native_tracking_report.json` records 1,186 odometry poses, 1,186 mapper point frames, 1,185 IMU factors, 44,927 normal-equation rows, rank ratio 1.0, 28,464,000 deskew queries, normalized-g IMU scaling (`9.80665`), and yaw-aligned comparison to the 10 Hz decimated Coco-LIC reference with 1,173 matches, 95.29% coverage, 0.203 m RMSE, 0.181 m mean error, 0.474 m max error, and 17.18% path drift.
  - 2026-05-12 continuous-time Ceres tuning: FAST-LIVO2 frontend_raw IMU samples were verified as normalized-g (`|a| ~= 0.991`) and the standalone continuous-time tracker now exposes `imu_linear_acceleration_scale`, `imu_info_gyro`, `imu_info_accel`, `position_extrapolation_damping`, runtime diagnostics, and real timestamped position-prior/SO(3) orientation-prior residuals for optional external/frontend odometry fusion. The parity script defaults to `9.80665`, one online Ceres iteration, `10/1` IMU information weights, and no stale-velocity position extrapolation after rejected solves. This moves the CBD prior-seeded smoke from the previous 12 s RMSE `10.541 m` to `0.559 m` before damping and `0.145 m` with damping at `results/fastlivo2/CBD_Building_01_continuous_time_damped0_prior_smoke_12s/trajectory_compare_report.json`; the 60 s safety run drops from RMSE `59.498 m` to `1.803 m` at `results/fastlivo2/CBD_Building_01_continuous_time_damped0_prior_smoke_60s/trajectory_compare_report.json`. A 4 s diagnostic report at `results/fastlivo2/CBD_Building_01_continuous_time_diagnostics_probe_4s/native_tracking_report.json` now records 916 IMU messages, 45 pointcloud frames, 2,124 pointcloud correspondences, and 41 rejected solve steps. The 60 s position-prior diagnostic at `results/fastlivo2/CBD_Building_01_continuous_time_position_prior_w0p2_apply_diag_60s/trajectory_compare_report.json` reaches 48.33% coverage and remains finite, but RMSE is `1.802 m`, path drift is `264.99%`, and 381 rotation-limited steps show the remaining blocker is production-grade rotation/LIO/VIO coupling, not missing topic data. A 12 s pose-prior diagnostic with position weight `0.2` and orientation weight `0.5` records 270 orientation prior factors, but RMSE is only `0.307 m` with `7.26x` path drift, so the pose-prior interface is useful plumbing rather than parity evidence.
  - 2026-05-12 point-cloud timing hardening: the standalone continuous-time node now queues incoming point clouds until a same-stamp B-spline pose is queryable before extracting persistent world-frame plane/point-map factors. The 12 s CBD diagnostic at `results/fastlivo2/CBD_Building_01_continuous_time_pc_wait_queue_diag_12s/native_tracking_report.json` defers 119 point clouds, releases 118, drops 0, and raises plane matches/updates to 2,459/3,554. The matching 60 s queue diagnostic releases 598/599 queued point clouds and records 22,472 LiDAR factors, but still has RMSE `1.802 m` and 592 rejected solve steps. This closes the ROS2 replay/callback timing data gap and moves the remaining blocker to the numerical consistency of the long-window rotation/LIO/VIO constraints. A diagnostic-only limited-rotation update option was added; it removes rejected steps on a 12 s CBD run but worsens RMSE to `0.179 m` and path drift to `2.45x`, so it stays default-off.
  - 2026-05-12 cost-source diagnostics: `TrajectoryEstimator` now reports initial/final Ceres cost by IMU, LiDAR, position prior, and orientation prior residual blocks, and the continuous-time native report parses scientific-notation metrics. The 4 s CBD cost probe at `results/fastlivo2/CBD_Building_01_continuous_time_cost_diagnostics_probe_4s/native_tracking_report.json` records last-step `initial_imu_cost=405046.294`, `final_imu_cost=227.592`, `initial_lidar_cost=0.003325`, `final_lidar_cost=0.116361`, `max_rotation_update_rad=0.684529`, and 41/41 rejections as rotation rejections. This gives the next production-BA sprint a concrete target: IMU-dominated rotation proposals are overpowering the current LiDAR geometry instead of data failing to enter the window.
  - 2026-05-12 Ceres trust-region controls: `TrajectoryEstimatorOptions`, the continuous-time node, launch file, and parity script now expose optional `ceres_initial_trust_region_radius` and `ceres_max_trust_region_radius` knobs. A 4 s CBD sweep shows radius `0.05` can suppress rotation rejections by keeping the last-step rotation update near `0.048 rad`, while `0.10` and `0.20` still reject. The 12 s `0.05` probe at `results/fastlivo2/CBD_Building_01_continuous_time_trust_0p05_probe_12s/native_tracking_report.json` still has RMSE `0.145 m` and path length `0.016 m` vs `0.917 m` baseline, so trust-region control is useful instrumentation but not the production BA fix.
  - 2026-05-12 persistent-map update gate: the continuous-time node can now require the last solver step to be fully accepted before mutating persistent world-frame plane/point maps, and reports accepted-step plus map-update-skip counters. The 12 s CBD gated probes reduce plane updates but collapse path length, confirming the current tracker still depends on continuous map growth and that the remaining blocker is not just rejected-pose map pollution.
  - 2026-05-12 LiDAR plane-normal factors: matched persistent planes can now add a direct normal-alignment residual, complementing point-to-plane distance with an explicit SO(3) observation. In 12 s CBD probes the factor is active and produces thousands of normal factors. Alone it does not improve ATE; combined with higher point-to-plane weight it can reduce rotation rejections sharply (`w_dist=0.5`, `w_normal=0.1`: 10 rejections, 117 accepted steps), but currently over-scales translation (`2.02 m` path vs `0.92 m` baseline). This confirms the next blocker is joint IMU/LiDAR scale and motion-prior balance, not missing LiDAR rotation observability.
  - 2026-05-12 position smoothness diagnostic: the continuous-time Ceres path now exposes a default-off second-difference position smoothness factor through `position_smoothness_weight` and `position_smoothness_huber_delta_m`, with factor counts archived by the parity script. In 12 s CBD sweeps combined with point-to-plane and plane-normal factors, smoothness can tune path scale but worsens ATE (`w_smooth=0.1`: RMSE `0.347 m`, path `0.909 m`; `w_smooth=0.5`: RMSE `0.175 m`, path `0.374 m`; `w_smooth=1.0`: RMSE `0.656 m`, path `1.825 m`; `w_smooth=2.0`: RMSE `0.405 m`, path `1.089 m`). It stays off by default and narrows the remaining blocker to IMU/LiDAR pose coupling rather than a missing scalar motion prior.
  - 2026-05-12 LiDAR scan-to-map pose priors: the standalone continuous-time node can now reuse the native `LidarFactor` weighted-Kabsch scan matcher as a default-off SE(3) pose-prior source. LiDAR points are transformed through the configured LiDAR-to-IMU extrinsic before matching, the corrected scan pose is fed as timestamped position and/or SO(3) orientation priors, and the parity script archives `lidar_pose_priors`, `lidar_pose_matches`, `lidar_pose_keyframes`, and rejection counts. A 4 s CBD probe at `results/fastlivo2/CBD_Building_01_continuous_time_lidar_pose_prior_probe_4s/native_tracking_report.json` records 48 pose priors, 50,118 matches, and 10 scan-map keyframes. The 12 s probe at `results/fastlivo2/CBD_Building_01_continuous_time_lidar_pose_prior_probe_12s/native_tracking_report.json` records 124 pose priors and 140,670 matches, but still has RMSE `0.204 m`, path `0.043 m` vs `1.002 m`, and 57 rotation rejections. Follow-up sweeps show the best current 12 s path-scale result is lower IMU weighting (`imu_info=0.3/0.03`) with RMSE `0.202 m` and path `0.768 m` vs `0.987 m`; holding gyro/accel bias or enabling position-only/orientation-only pose priors does not close ATE. The limited-rotation trust-region path now optionally scales position updates by the same SO(3) ratio; this prevents the old `17.55 m` path blow-up and gives a finite 12 s diagnostic at `results/fastlivo2/CBD_Building_01_continuous_time_lidar_pose_prior_limited_scaled_12s/native_tracking_report.json` with path `0.957 m` vs `1.002 m` and zero rejected steps, although RMSE remains `0.205 m`. This closes the "SE(3) scan matcher not connected to continuous-time BA" gap; the remaining blocker is trajectory shape/rotation accuracy, not missing pose-prior data.
  - 2026-05-12 LiDAR pose-prior ICP iteration diagnostic: `LidarFactor` now has a default-preserving `lidar_pose_factor_iterations` knob for bounded multi-iteration scan-to-map pose correction, with a residual monotonic gate after the first iteration and script/report archival. The deterministic probe shows the extra iterations are mathematically live (`single_error=0.0563`, `multi_error=0.0122` on a low-gain synthetic scan). Real CBD 12 s sweeps show why this stays default `1`: iter=2/4 overfit local correspondences and inflate path length, and even the monotonic-gated iter=4 run at `results/fastlivo2/CBD_Building_01_continuous_time_lidar_pose_iter4_monotonic_12s/trajectory_compare_report.json` records RMSE `0.428 m` with path `5.497 m` vs `0.916 m` reference. The latest iter=1 guard run at `results/fastlivo2/CBD_Building_01_continuous_time_lidar_pose_iter1_after_iter_patch_12s/trajectory_compare_report.json` remains finite (RMSE `0.104 m`) but still has path drift `272.37%`; multi-step nearest-neighbor ICP is therefore implemented and bounded, but not a long-window parity fix.
  - 2026-05-12 update-gate edge-knot margin: the sliding-window acceptance gate now has a default-off `update_gate_edge_knot_margin` knob so guard knots can be excluded from max update magnitude diagnostics while still being finite-checked. The 12 s CBD margin-2 probes at `results/fastlivo2/CBD_Building_01_continuous_time_lidar_pose_prior_edge_margin2_12s/trajectory_compare_report.json` and `results/fastlivo2/CBD_Building_01_continuous_time_lidar_pose_prior_edge_margin2_limited_scaled_12s/trajectory_compare_report.json` reached RMSE `0.203 m` and `0.205 m`, respectively, and did not beat the default margin-0 runs. It stays a diagnostic control, not a paper-parity default.
  - 2026-05-12 SO(3) smoothness factor: `TrajectoryEstimator` now supports a default-off second-difference rotation smoothness residual, the sliding-window diagnostics report rotation smoothness factor counts and smoothness cost, and the parity script archives the weights. The 12 s CBD sweep shows the factor is active and can remove rotation rejections; paired with LiDAR scan-to-map position prior weight `2.0`, `results/fastlivo2/CBD_Building_01_continuous_time_rot_smooth_0p01_lidar_pose_pos2_12s/trajectory_compare_report.json` improves RMSE to `0.199 m` from the previous `0.202 m` pose-prior run, but path scale is still short at `0.633 m` vs `0.987 m`. This is a real BA residual and a measurable improvement, but not yet full Coco-LIC2 trajectory parity.
  - 2026-05-12 LiDAR velocity priors: the continuous-time estimator now supports timestamped velocity priors in Ceres, and the native node can derive them from consecutive LiDAR scan-to-map corrected poses. This adds a motion-increment constraint without anchoring absolute position. The 12 s CBD probe at `results/fastlivo2/CBD_Building_01_continuous_time_rot_smooth_0p01_lidar_pose_pos5_vel1_12s/trajectory_compare_report.json` combines rotation smoothness `0.01`, LiDAR pose position weight `5.0`, velocity weight `1.0`, and orientation weight `0.5`; it records 65 Ceres velocity prior factors, 131 LiDAR velocity-prior events, zero rejected Ceres steps, RMSE `0.198 m`, and path length `0.990 m` vs `0.988 m` reference. The matching 60 s run at `results/fastlivo2/CBD_Building_01_continuous_time_rot_smooth_0p01_lidar_pose_pos5_vel1_60s/trajectory_compare_report.json` keeps path scale finite (`10.089 m` vs `10.603 m`) and accepts all 302 solves, but RMSE remains `1.806 m`; a dense scan-to-map map sweep (`stride=1`, 100k map points) increases matches to 1.84M but worsens path scale to `2.040 m`, so the remaining long-window blocker is trajectory shape/visual-global coupling rather than map capacity. This is the current best short-window production-BA checkpoint; full-sequence strict parity is still pending.
  - 2026-05-12 visual rotation-prior diagnostic: the standalone continuous-time node can now subscribe to raw camera frames and CameraInfo, run bounded grayscale frame-to-frame alignment, and feed an approximate monocular rotation prior into the spline orientation residual path. The parity script archives the visual subsampling, frame stride, sign, scale, and runtime counters. The first CBD 12 s probes confirm the data path works (`36-40` visual priors, zero point-cloud drops after stride/max-pixel throttling), but the factor worsens path scale (`4.58-10.12 m` current vs `~0.98 m` reference). It stays default-off and documents that naive image-shift priors are not a Coco-LIC2-grade substitute for depth/Gaussian photometric BA coupled directly to the spline.
  - 2026-05-12 sparse-depth visual SE3 prior diagnostic: the standalone continuous-time node now projects LiDAR points into the camera frame using ROS2-configured camera-to-IMU extrinsics, builds sparse depth frames, selects the nearest fresh depth stamp, solves SE3 photometric normal equations, transforms camera deltas back to the body frame, and feeds bounded position/SO(3) orientation priors into the spline window. The parity script exposes all visual-SE3 weights, depth freshness/cache, coverage-grid, Hessian, residual, and step-limit gates, and archives runtime counters. The initial 12 s CBD stride-fix probe proves the path is live with one valid batch and RMSE `0.2007 m`; widening depth freshness to `500 ms` yields 114 batches and 35 valid priors but worsens RMSE to `0.2035 m`. A lighter `250 ms`, `0.02/0.02` position/orientation-weight sweep reaches RMSE `0.19695 m`, slightly better than the no-visual `0.19797 m` checkpoint, while the corresponding 60 s run records 581 batches, 298 valid priors, zero point-cloud drops, finite path scale (`9.226/10.607 m`), and RMSE `1.811 m`. Coverage-grid gating is implemented and observable (`visual_se_last_coverage_tiles`, `visual_se_last_mean_abs_residual`), but the 12 s coverage-gated probe did not improve ATE. This closes the "no direct sparse-depth visual SE3 feedback in continuous-time parity path" implementation gap; it is still default-off because long-window RMSE parity is not solved.
  - 2026-05-12 sparse-depth visual velocity-prior diagnostic: the same SE3 photometric delta can now feed a timestamped world-frame velocity prior through the existing Ceres velocity-prior residual path. The parity script archives `visual_se3_velocity_weight`, `visual_se3_huber_delta_mps`, and `visual_se_velocity_priors`. CBD 12 s sweeps show the path is active (`59-60` velocity priors), but velocity-only and mixed velocity/position/orientation configurations shorten path scale and do not beat the no-velocity checkpoint, so the velocity branch remains default-off instrumentation.
  - 2026-05-12 LiDAR angular-velocity priors: `TrajectoryEstimator` and the streaming continuous-time window now accept timestamped body-frame angular-velocity priors, and the standalone node can derive them from consecutive LiDAR scan-to-map corrected orientations. The parity script archives `lidar_pose_prior_angular_velocity_weight`, `lidar_pose_angular_velocity_priors`, and total Ceres angular-velocity prior factors. CBD 12 s probes confirm the factor is active (`119-120` LiDAR angular-velocity prior events and `59-60` Ceres factors), but tested weights `0.05/0.10/0.20` shorten path scale and worsen RMSE (`0.205-0.222 m`), so this remains default-off instrumentation rather than a production parity setting.
  - 2026-05-12 LiDAR scan-to-scan relative-motion priors: the continuous-time node can now build a consecutive-scan ICP constraint in the previous IMU frame, convert it into timestamped world velocity and body angular-velocity priors, and report `lidar_scan_to_scan_*` counters in native parity artifacts. The 12 s CBD sweep at `results/fastlivo2/CBD_Building_01_continuous_time_scan_to_scan_v1p0_w0p1_12s/trajectory_compare_report.json` improves the latest comparable short-window RMSE from `0.104 m` to `0.074 m` and confirms 99 velocity/99 angular priors entered Ceres, but the matching 60 s run at `results/fastlivo2/CBD_Building_01_continuous_time_scan_to_scan_v1p0_w0p1_60s/trajectory_compare_report.json` regresses to RMSE `1.982 m` with `1763.95%` path drift. This is useful production-BA plumbing and a short-window stabilizer, but it stays default-off until a global visual/map constraint prevents long-window shape drift.
  - 2026-05-12 output-cadence hardening: `continuous_time_node` now supports `pose_output_period_seconds`, and `continuous_time_native_reference_parity.sh` exposes both `STEP_PERIOD_SECONDS` and `POSE_OUTPUT_PERIOD_SECONDS` so strict reports can separate solver cadence from sampled odometry cadence. A CBD 12 s 10 Hz output probe at `results/fastlivo2/CBD_Building_01_continuous_time_pose_output_10hz_12s/trajectory_compare_report.json` writes 122 poses and reaches RMSE `0.136 m`. The matching 60 s 10 Hz output probe writes 603 poses and raises reference coverage to `48.66%`, but RMSE remains `1.812 m` and path drift grows to `165.7%`; forcing 10 Hz solver cadence and adding position smoothness worsened 12 s RMSE, while holding both IMU biases improves 12 s RMSE to `0.060 m` but worsens 60 s RMSE to `1.892 m`. This closes the hidden low-cadence reporting gap and confirms the remaining blocker is not output sampling, simple bias locking, or missing data; it is long-window trajectory-shape coupling.
  - 2026-05-12 IMU bias soft-prior/random-walk: `TrajectoryEstimator` and `ContinuousTimeSlidingWindow` now add default-off gyro/accel bias priors against the previous accepted bias, expose bias-prior counts and costs in runtime diagnostics, and let `continuous_time_native_reference_parity.sh` sweep `GYRO_BIAS_PRIOR_WEIGHT` / `ACCEL_BIAS_PRIOR_WEIGHT` plus Huber deltas. CTest covers bias-prior regularization in the estimator probe. A CBD 12 s `g1/a1` probe at `results/fastlivo2/CBD_Building_01_continuous_time_biasprior_g1_a1_12s/trajectory_compare_report.json` improves RMSE to `0.047 m` with path drift `14.5%`, but the matching 60 s run at `results/fastlivo2/CBD_Building_01_continuous_time_biasprior_g1_a1_60s/trajectory_compare_report.json` remains at RMSE `1.894 m` with path drift `946.38%`. This proves bias random-walk control is useful short-window production-BA plumbing while confirming the long-window blocker is still global trajectory-shape/visual-map coupling, not a missing bias prior alone.
- [x] Add a full-sequence CBD native visual/SE3 production-BA health report.
  - `run_native_tracking_bag_report.sh` now exposes `--mapper-feedback-sync-tolerance-sec` for mapper feedback under asynchronous replay, accepts `--reference-tum` for external reference-trajectory comparison, and avoids recorder shutdown noise when ROS2 tears down the context.
  - 2026-05-06 local CBD 120s visual+scan-order deskew run at `results/fastlivo2/CBD_Building_01_native_tracking_visual_ba_sync50ms_120s/native_tracking_report.json` passes with 478 odometry poses, 478 mapper point frames, 477 IMU factors, 238 visual-alignment factors, 27 SE3 photometric factors, 238 SE3 batches, 27 valid SE3 batches, 0 degenerate batches, 21 quality-rejected batches, 76,250 cumulative SE3 samples, 64 rendered/depth cache slots, 2 rendered misses, last accepted sample inlier ratio 0.935, last accepted coverage 6/16 tiles, last accepted SE3 Hessian rank 6, last accepted SE3 Hessian condition 6.187e3, 11,472,000 deskew queries, 11,430,279 deskew hits, zero IMU factor/time-gap skips, 477 feedback updates, 6,258 normal-equation rows, condition number 5.818e6, rank ratio 1.0, and zero numeric-Jacobian fallback.
- [x] Add R3LIVE real-bag native tracking runtime coverage.
  - `hku_park_00.bag` converts to ROS2 frontend-raw with 3,428 images, 2,285 LiDAR frames, and 46,599 IMU samples; the native report gate now treats cumulative LiDAR point/plane correspondences as valid long-run evidence when the final active window has already marginalized current LiDAR factors.
  - 2026-05-06 local R3LIVE 60s sensor-only scan-order deskew run at `results/r3live/hku_park_00_native_tracking_scan_order_60s/native_tracking_report.json` passes with 554 odometry poses, 554 mapper point frames, 553 IMU factors, 18,004 cumulative point correspondences, 7,474 cumulative plane correspondences, 10 smoothness factors in the last window, 13,296,000 deskew queries, 8,542,971 deskew hits, zero IMU factor/time-gap skips, 553 feedback updates, 567 normal-equation rows, condition number 3.317e4, rank ratio 1.0, and zero numeric-Jacobian fallback.
- [x] Add FAST-LIVO2 Retail_Street real-bag native tracking runtime coverage.
  - `Retail_Street.bag` downloads from the official FAST-LIVO2 Google Drive index with 1,961,140,106 bytes, converts to ROS2 frontend-raw with 1,355 images, 1,355 LiDAR frames, and 27,447 IMU samples, and passes the `frontend_sensor_raw` bag contract.
  - 2026-05-06 local Retail 60s sensor-only scan-order deskew run at `results/fastlivo2/Retail_Street_native_tracking_scan_order_60s/native_tracking_report.json` passes with 197 odometry poses, 197 mapper point frames, 195 IMU factors, 103,677 cumulative point correspondences, 73,962 cumulative plane correspondences, 4,728,000 deskew queries, normal-equation rank ratio 1.0, and zero numeric-Jacobian fallback.
  - 2026-05-12 local Retail upstream mapper baseline is archived at `baseline/fastlivo2/Retail_Street`: the ROS1 manifest records 1,354 trajectory poses, 704,826 PLY vertices, 1,354 rendered frames, and a native-reference TUM at `baseline/fastlivo2/Retail_Street/native_reference/ros1_mapper_baseline_full.tum`. This closes the Retail data/reference side of the audit while leaving native tracker RMSE parity as a separate quality gate.
- [x] Fix M2DGR native tracking time/QoS replay blockers before full strict parity.
  - Tracking now defaults to nonzero world gravity, auto-calibrates initial roll/pitch from the startup IMU accelerometer mean, seeds the propagator with the first IMU sample, and tests that a stationary accelerometer sample remains fixed under gravity compensation.
  - Point-cloud processing now queries `ImuPropagator` at the point-cloud stamp instead of silently using a future latest IMU state when ROS2 callback order differs from ROS1 rosbag order. The native report script promotes strict-replay raw IMU and point-cloud inputs to reliable QoS while keeping images best-effort, deepens the point-cloud wait queue and IMU history, and exposes a post-playback drain period for offline strict replay.
  - 2026-05-07 M2DGR `room_01` evidence improved from the previous 60s zero-gravity path length of 17,980.16 m to 1,226.63 m with gravity autocalibration, 419.54 m with reliable IMU, 5.86 m on a 20s slow strict replay, and 2.24 m / 1.18 m RMSE on a 20s strict replay with both IMU and point-cloud reliable QoS at `results/m2dgr/room_01_native_tracking_time_query_reliable_imu_pc_rate02_20s/native_tracking_report.json`.
  - The corresponding 60s reliable IMU+point-cloud strict replay at `results/m2dgr/room_01_native_tracking_time_query_reliable_imu_pc_rate02_60s/native_tracking_report.json` is still not strict, but reduces the current trajectory path from 1,146.12 m to 33.80 m and RMSE from 171.99 m to 16.12 m. Full-sequence ±5% parity remains open and now points at long-window LIO/BA drift rather than missing data or ROS2 replay loss.
  - `scripts/run_native_tracking_bag_report.sh` now exposes the LiDAR correspondence/correction/keyframe and sliding-window IMU/pose/smoothness weights used by the launch file, fixes slow rosbag2 replay timeout accounting to divide by playback rate, and `scripts/run_m2dgr_tracking_sweep.sh` ranks local M2DGR runs against `room_01_gt.tum`. A 2026-05-07 20s sweep at `results/m2dgr/room_01_tracking_sweep_20s/sweep_summary.json` currently ranks all-frame strict replay best by RMSE (`0.91 m`) while stronger IMU/pose/smoothness weighting lowers path drift but does not beat RMSE; the matching 30s all-frame run at `results/m2dgr/room_01_tracking_sweep_30s/sweep_summary.json` reaches `2.11 m` RMSE, so drift starts growing materially between 20s and 30s.
  - 2026-05-07 follow-up fixes add delayed-IMU rebase/replay after sliding-window feedback, refill the current preintegrator from IMU history for queued point clouds, gate IMU feedback on non-IMU LiDAR/visual/external factors, clamp velocity/bias feedback, skip keyframes without tracking support, and add a profile-level native pose-step guard. The M2DGR `room_01` 60s sweep at `results/m2dgr/room_01_tracking_sweep_yaw_guarded008_60s/sweep_summary.json` now passes the local native reference gate with yaw-aligned ATE: 106 poses, 105 matches, RMSE 0.4479 m, mean 0.4342 m, max 0.7597 m, and path drift 31.44%. The trajectory comparator now supports `--align yaw` for no-scale local-world-to-reference ATE, while full-dataset strict parity remains open until ROS1 mapper/render baselines and all required profiles are archived.
- [x] Add profile-aware ROS1 frontend-raw conversion and a data-side strict audit.
  - `scripts/ros1_to_frontend_raw.py` wraps the profile-aware converter for FAST-LIVO, FAST-LIVO2, M2DGR, MCD, and R3LIVE; the older `fastlivo2_ros1_to_frontend_raw.py` entrypoint remains compatible. The converter chooses per-profile topic fallbacks and intrinsics, supports comma-separated multi-bag inputs for split datasets, supports both Livox `CustomMsg` and source `sensor_msgs/PointCloud2` LiDAR bags, and normalizes the output to the ROS2 `frontend_sensor_raw` contract.
  - 2026-05-06 local 1s conversion probes pass the `frontend_sensor_raw` bag contract for FAST-LIVO2 `Bright_Screen_Wall` and R3LIVE `hku_park_00`.
  - `scripts/audit_strict_data_inputs.py` writes `docs/strict_data_status.json` / `.md`, checks raw bags, converted frontend bags, ROS1 baselines, ROS2 current artifacts, trusted native reference trajectories, disk headroom, and the largest non-matrix `results/` cleanup candidates. The 2026-05-12 audit now passes all data-side inputs: raw, frontend, ROS1 baseline, ROS2 current, native-reference, and disk-headroom checks are green for FAST-LIVO, FAST-LIVO2, M2DGR, MCD, and R3LIVE, with `109.39 GiB` free on `/home/frank/data` and strict evidence materialized at `17/17`.
- [x] Add generic Google Drive single-file fetcher and partial MCD data acquisition.
  - `scripts/fetch_google_drive_file.py` reuses the repository Google Drive warning-page handling, byte-count checks, manifests, and free-space gate for arbitrary public file IDs.
  - 2026-05-08 local MCD `ntu_day_01` acquisition now has `d435i` camera (`14,264,815,921` bytes), `mid70` LiDAR (`568,196,474` bytes), `vn100` IMU (`33,957,020` bytes), `groundtruth/pose_inW.tum`, frontend conversion, archived ROS1 baseline artifacts, ROS2 current artifacts, and a passing strict mapper-contract/CUDA report.
- [x] Collect and pass the required CBD native reference trajectory parity gate.
  - The archived mapper-contract TUM remains intentionally rejected as a native frontend reference. The accepted reference is generated by upstream Coco-LIC on the official ROS1 `CBD_Building_01.bag`, archived as `baseline/fastlivo2/CBD_Building_01/native_reference/cocolic_livo_reference_full.tum`, decimated to `baseline/fastlivo2/CBD_Building_01/native_reference/cocolic_livo_reference_10hz.tum` for ROS2 native-output cadence matching, and gated by `trajectory_compare.py`.
  - Native reference trajectory evidence is now archived for every required profile in the data audit. The required strict matrix is green; the remaining quality work is RMSE-gated long-window continuous-time native tracker parity, not missing dataset/reference inputs.
- [x] Local SPNet TensorRT engine benchmark passes the runtime target.
  - TensorRT 10.9 CUDA 12.8 builds `/home/frank/Software/TensorRT-engines/spnet_512_640_fp16.engine` for the RTX 5070 Ti `sm_120` GPU.
  - Mean `trtexec` latency is 26.4492 ms at `512x640`, below the 30 ms/frame target.
