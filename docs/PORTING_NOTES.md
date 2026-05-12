# Porting Notes

## Project Direction

The port should be native ROS2, not only a ROS1 bridge wrapper. Bridge support can exist as a compatibility path, but the main README and launch flow should target native ROS2 nodes.

## Initial ROS2 Topic Contract

Inputs:

```text
/camera/image          sensor_msgs/msg/Image
/camera/camera_info    sensor_msgs/msg/CameraInfo
/livox/lidar           sensor_msgs/msg/PointCloud2
/imu                   sensor_msgs/msg/Imu
/tf                    tf2_msgs/msg/TFMessage
/tf_static             tf2_msgs/msg/TFMessage
```

Outputs:

```text
/gaussian_lic/odometry        nav_msgs/msg/Odometry
/gaussian_lic/path            nav_msgs/msg/Path
/gaussian_lic/map_points      sensor_msgs/msg/PointCloud2
/gaussian_lic/rendered_image  sensor_msgs/msg/Image
/gaussian_lic/gaussian_map    gaussian_lic_msgs/msg/GaussianArray
/gaussian_lic/status          gaussian_lic_msgs/msg/MappingStatus
/gaussian_lic/frontend/status gaussian_lic_msgs/msg/TrackingStatus
```

## ROS2 Design Rules

- Use configurable sensor-data QoS for image, point cloud, pose, camera info, depth, and IMU subscriptions. Defaults are `best_effort`, `keep_last`, depth `5`; `sensor_qos_reliability:=reliable` is available for reliable rosbag2 or driver outputs.
- Keep estimator timestamp math in signed `int64_t` nanoseconds. Do not replace ROS1 `ros::Time` math with `rclcpp::Time` or double seconds inside B-spline, IMU, LiDAR deskew, or frame-sync code.
- Strict replay and component launch must use a single-threaded executor until the frontend has deterministic stamp-ordered queues around every estimator update.
- Use lifecycle nodes for long-running mapping/tracking components.
- Keep GPU/CUDA code isolated from middleware glue.
- `mapping_node` is available as both a standalone executable and an `rclcpp_components` plugin; launch with `use_composition:=true` to load it in the single-threaded `component_container`.
- Keep launch files dataset-agnostic; put dataset paths and remaps in config.
- See `docs/ROS2_SEMANTICS.md` for the enforced time, QoS, executor, tf2, and rosbag replay contract.

## Known Hard Part

Gaussian-LIC2 is now public in the primary `APRIL-ZJU/Gaussian-LIC` upstream, so the native frontend/tracking port should be based on that code path first. Coco-LIC remains useful as legacy ROS1 reference material for the original mapper-topic contract, but it is no longer the main implementation target. A bridge-only solution should not be marketed as the main ROS2 port.

## Current Native Slice

`gaussian_lic_frontend/lic2_contract_adapter` is the first native ROS2 frontend boundary. It subscribes to the raw sensor/pose surface:

```text
/camera/image
/camera/camera_info
/camera/depth
/livox/lidar
/imu
/gaussian_lic/frontend/pose
/gaussian_lic/frontend/input_odometry
```

and republishes into the mapper contract:

```text
/image_for_gs
/camera_info_for_gs
/depth_for_gs
/points_for_gs
/pose_for_gs
/imu_for_gs
```

Odometry messages are converted into `PoseStamped` for `/pose_for_gs`. The adapter also publishes `/gaussian_lic/frontend/odometry`, `/gaussian_lic/frontend/path`, and optional TF from any incoming pose or odometry so downstream visualization and regression tools can use the same frontend surface before the full continuous-time odometry algorithm is ported. For raw bags without odometry, `imu_pose_fallback:=true` integrates IMU angular velocity into a non-identity orientation fallback at point-cloud timestamps; this is an executable fallback for current artifact generation, not a replacement for the full Gaussian-LIC2 continuous-time frontend.

For FAST-LIVO2 raw bags, `pointcloud_transform_profile:=fastlivo2` applies the released camera-LiDAR extrinsic profile before publishing `/points_for_gs`. This turns `/livox/lidar` points into the camera frame using the Coco-LIC/FAST-LIVO2 calibration (`R_cl`, `P_cl`) and lets image-projection color and the Torch photometric optimizer receive visible camera-frame supervision even when the strict frontend odometry is not available.

`gaussian_lic_mapping/mapping_node` now ports the ROS-facing frame synchronization and frame-conversion surface from upstream Gaussian-LIC. It buffers:

```text
/points_for_gs
/pose_for_gs
/image_for_gs
/camera_info_for_gs
/depth_for_gs
```

and aligns frames using `/points_for_gs` as the reference timestamp with `sync_tolerance_sec` defaulting to `0.01`, matching upstream's 10 ms tolerance. Internally the public seconds parameter is converted once to signed nanoseconds, and all queue trimming/comparison uses integer nanosecond stamps to avoid silent ROS1-to-ROS2 epoch and floating-point drift. `CameraInfo` is not part of the synchronized set; the latest valid message updates the active `fx/fy/cx/cy` intrinsics, and parameter values remain the fallback.

After alignment, the node converts ROS2 messages into `MapperFrameData`:

```text
Image       -> RGB float32 OpenCV matrix in [0, 1]
CameraInfo  -> active fx/fy/cx/cy intrinsics for torch camera/Gaussian initialization
Depth       -> metric float32 OpenCV matrix
PoseStamped -> Eigen q_wc, t_wc, R_wc
PointCloud2 -> world xyz, RGB color in [0, 1], camera-frame depth
```

PointCloud2 color uses packed `rgb/rgba` or scalar `r/g/b` fields when present. For uncolored clouds, the ROS2 port now projects each positive-depth point into the synchronized image with the active pinhole intrinsics and samples the image color, keeping a white fallback for points outside the image. This preserves the LiDAR-camera color boundary needed by Gaussian initialization without requiring a pre-colored cloud topic.

Depth images are required by default. When `require_depth_topic:=false`, the mapper aligns only point cloud, pose, and image messages, then builds a sparse `CV_32FC1` metric depth image by projecting valid point-cloud points into the camera. This keeps replay usable for intermediate upstream bags when TensorRT/SPNet depth completion is disabled or no engine path is configured.

Converted frames are then accumulated in `MapperDataset`, mirroring the non-torch state of upstream `Dataset`:

```text
train/test camera frame records
R_wc / t_wc pose records
pending world points
pending RGB colors
pending camera-frame depths
```

The native mapper also republishes converted tracking/map-point surfaces:

```text
PoseStamped -> nav_msgs/msg/Odometry on /gaussian_lic/odometry
PoseStamped -> nav_msgs/msg/Path on /gaussian_lic/path
Accumulated MapperPoint world cloud -> sensor_msgs/msg/PointCloud2 on /gaussian_lic/map_points
render_mode:=debug_cpu CPU projected accumulated map preview -> sensor_msgs/msg/Image on /gaussian_lic/rendered_image
Imu -> native ROS2 subscription/counting for the LIC input contract
PoseStamped -> optional TF map -> camera when publish_tf:=true
```

The CUDA rasterizer, fused SSIM loss, visibility-masked SparseGaussianAdam, densification/pruning, and strict CUDA rendered preview are ported behind the Torch/CUDA build profile. The native tracking package now provides timestamp-safe trajectory/IMU primitives, ROS2-configurable LiDAR-to-IMU extrinsics before LIO/deskew/mapper point publication, ROS2-configurable camera-to-IMU extrinsics for photometric BA delta conversion, signed-nanosecond visual/render/depth freshness gates with nearest-stamp cached render/depth selection, signed-nanosecond IMU history interpolation, raw-sample IMU preintegration/reintegration, delayed-IMU rebase/replay after optimized feedback, current-window preintegration refill from IMU history for queued point clouds, point-cloud factor queuing until a matching continuous-time pose is queryable for persistent world-frame plane/point maps, optional accepted-solve gating before persistent map mutation, direct LiDAR plane-normal alignment factors for persistent-plane matches, default-on Coco-LIC-style LiDAR feature-confidence scaling with physically scaled weighted-Huber gates, default-off LiDAR scan-to-map SE(3) pose priors from the native weighted-Kabsch scan matcher with independently disableable position/orientation components, optimized pose/velocity/bias feedback into odometry, the continuous-time trajectory-control cache, and safe IMU re-anchoring with status gates, IMU bias continuity residuals and marginalization-prior anchoring in the default-enabled sliding window, bias observability status, B-spline position/velocity plus SO(3) cubic orientation trajectory queries for deskew fallback, bounded LM state increments, default-enabled three-state trajectory smoothness factors with analytic linear Jacobian blocks, default-off second-difference position and SO(3) rotation smoothness factors for motion-prior diagnostics, per-point LiDAR deskew, native odometry/path/TF publication, bounded 6-DoF LiDAR residual correction, spatial-indexed direct LiDAR/Gaussian-map point-to-point window factors with robust correspondence confidence, LiDAR point-to-plane factors with residual/local-planarity confidence weighting, LiDAR confidence/spatial-index status, sliding-window BA optimization timing and numeric-Jacobian fallback status, Huber-robust visual-alignment and SE3 photometric window factors, visual residual/direct image-alignment paths that compare mapper rendered images with incoming camera frames, analytic full-current IMU preintegration Jacobians, analytic SE3 camera photometric pixel Jacobians with multi-sample normal-equation solving, default-enabled robust SE3 photometric window factors extracted from rendered/current/depth images when available, and chunk-complete `GaussianArray` snapshot caching for the mapper-to-tracker reverse channel. The standalone continuous-time tracker now also exposes IMU linear-acceleration scaling, IMU information weights, optional bias hold flags in the parity script, optional Ceres trust-region radii, bounded position-extrapolation damping, runtime diagnostics in the parity report, timestamped position-prior, velocity-prior, and SO(3) orientation-prior residuals for optional external/frontend/LiDAR scan-to-map odometry fusion, a default-off update-gate edge-knot margin for guard-knot diagnostics, and a default-off limited-rotation diagnostic guard that scales oversized SO(3) updates instead of accepting them wholesale; when that guard is enabled, its companion position update can be scaled by the same trust-region ratio to avoid accepting full translation with partial rotation. FAST-LIVO/FAST-LIVO2 normalized-g frontend_raw bags should use `imu_linear_acceleration_scale:=9.80665`, and the evidence-backed online Ceres setting is one iteration per step to avoid over-solving a streaming window. Damping rejected-solve extrapolation is a safety guard against stale-velocity blow-up, pose priors are diagnostic/fusion hooks rather than a substitute for LIO/VIO motion constraints, trust-region radii, bias holds, position/rotation smoothness, LiDAR scan-to-map pose and velocity priors, update-gate edge margins, and accepted-solve map mutation gates are diagnostic controls rather than parity defaults, and limited rotation is not a parity default because it can reduce rejection counters while leaving ATE above the paper gate. The real-bag report tooling also supports yaw-aligned no-scale local-world trajectory ATE for datasets whose motion-capture/world frame does not share the estimator's arbitrary yaw. Full Coco-LIC2-grade production joint BA is still separate from this mapper backend work.

2026-05-13 tracker parity ablations added three upstream-alignment hooks without marking the long-window gate complete: limited-update branches now feed accepted IMU bias/gravity deltas, scan-to-scan physical clamps now bound the stored target pose as well as velocity/angular priors, and `fixed_control_point_index` can reproduce Coco-LIC's `SetFixedIndex(3)` gauge anchor for experiments. The fixed-index default remains disabled until the matching marginalization prior is fully active, because the standalone anchor worsened the current CBD 12s ablation.

2026-05-13 continuous-time map anchoring added a native 3D LiDAR
point-to-point residual to `TrajectoryEstimator` and the online
`ContinuousTimeSlidingWindowEstimator`. Persistent point-map correspondences now
enter Ceres as one coupled SE(3) residual with one robust loss instead of three
independent axis-aligned plane residuals. This is required for future
Gaussian-map/global-anchor BA, but it is still a default-off parity path until
the next long-window CBD run proves RMSE improvement.

The same update batches persistent point-map insertions until the end of each
scan, so correspondences are built only against historical map points and not
against earlier points from the same scan. CBD 12 s validation confirms the new
3D residual is active (`~12.5k` LiDAR point factors), but path scale remains too
short (`0.47 m` current vs `0.92 m` reference, RMSE `0.144 m`), so Gaussian-map
point anchors still need covariance/visibility filtering before they can become
parity defaults.

2026-05-13 follow-up: persistent point-map entries now store first/last
signed-nanosecond stamps, per-stamp observation counts, and same-stamp sample
means. Matches are gated on minimum age and minimum cross-stamp observations,
with `persistent_point_map_merge_distance_m`,
`persistent_point_map_min_match_age_s`, and
`persistent_point_map_min_observations_for_match` forwarded by the parity
script. This removes the remaining same-scan target ambiguity, but CBD 12 s
evidence still does not improve ATE (`0.143944-0.145313 m` RMSE and shortened
path scale), so the open strict-parity blocker remains global trajectory-shape
and visual/Gaussian-map coupling rather than point-map update timing.

2026-05-13 Gaussian snapshot anchoring follow-up: the standalone
`continuous_time_node` can now subscribe directly to chunked
`gaussian_lic_msgs/GaussianArray` map snapshots on `gaussian_map_topic`,
cache complete mapper snapshots, build a voxel nearest-neighbor index over
opacity-gated Gaussians, and add robust LiDAR-to-Gaussian point-to-point
residuals to the same continuous-time Ceres window used by IMU/LiDAR/visual
factors. `continuous_time_native_reference_parity.sh` forwards the default-off
controls (`enable_gaussian_snapshot_lidar_factor`,
`gaussian_snapshot_lidar_factor_weight`,
`gaussian_snapshot_lidar_nearest_distance_m`,
`gaussian_snapshot_lidar_min_opacity`, frame/map subsampling, and max
correspondences) and records them in the native report config. This closes the
missing mapper-to-tracker Gaussian global-anchor plumbing, but it remains
default-off until a full mapper-feedback run proves long-window RMSE parity.
`run_native_tracking_bag_report.sh` now has a matching
`--enable-gaussian-map-feedback` mode that starts `mapping_node` with Torch
Gaussian initialization/extension and `GaussianArray` publication enabled, plus
`--require-gaussian-snapshot` to fail reports when tracking did not receive a
complete map snapshot. This turns mapper feedback from a rendered-image-only
diagnostic into a reproducible Gaussian-map anchor experiment.

The first CBD probe exposed a ROS2 QoS bug specific to chunked map transport:
`GaussianArray` publisher/subscribers used transient-local reliable QoS with
depth 1, so a five-chunk map could arrive as only the last chunk. The mapper
now exposes `gaussian_map_qos_depth`, tracking and continuous-time nodes expose
`gaussian_snapshot_qos_depth`, and the report scripts use depth 128 for
feedback runs. `results/fastlivo2/CBD_Building_01_gaussian_map_feedback_qos128_probe_6s/native_tracking_report.json`
passes with `gaussian_snapshot_chunks_received=5/5`,
`gaussian_snapshot_points=17716`, and 18 point-factor batches in the tracking
window.

Longer feedback runs can receive a new map sequence while the previous one is
still the active tracker anchor. `GaussianSnapshot` therefore now stages chunks
in a pending buffer and only swaps the committed map after the pending sequence
is complete. The 20 s CBD probe at
`results/fastlivo2/CBD_Building_01_gaussian_map_feedback_double_buffer_probe_20s/native_tracking_report.json`
keeps `gaussian_snapshot_complete=true` despite a later in-flight map update,
with `20861` committed Gaussians, `6/6` committed chunks, and 20 point-factor
batches.
`tracking_node` also exposes `gaussian_snapshot_lidar_factor_weight` for
mapper-feedback sweeps. CBD 20 s probes at `0.05` and `0.5` reduced the point
factor cost but worsened short-window RMSE relative to the original `1.0`
weight, so the default remains `1.0` while long-window work focuses on replay
coverage, correspondence quality, and visual/global coupling rather than a
blind scalar downweight.
`mapping_node` now exposes `gaussian_map_publish_min_interval_sec` and
`gaussian_map_publish_on_empty_extend`; mapper-feedback report runs use a
0.5 s simulated-time full-map publish interval and skip keyframes that insert
zero new Gaussians, while normal launch defaults preserve immediate publishing.
This is a ROS2 runtime timing guard: it reduces repeated full-map chunk bursts
without changing Gaussian math or QoS semantics.
The matching tracker guard is `sliding_window_optimize_every_n_frames`:
normal launch defaults keep every-frame BA, but Gaussian-map feedback evidence
runs default to optimizing every 4 point-cloud frames because full Gaussian
anchor windows take seconds per solve on the current setup. Factors are still
accumulated every frame; only the synchronous Ceres solve cadence is decimated
to preserve rosbag replay coverage.
CBD/FAST-LIVO2 native report runs also default `imu_linear_acceleration_scale`
to `9.80665` because the frontend-raw bag stores accelerometer samples in
normalized-g units. The 20 s Gaussian-feedback probe with scale `1.0` recorded
RMSE `1.4935 m`; the same run with `9.80665` recorded RMSE `0.1031 m`, proving
this is a timestamp-preserving scale semantic rather than a tuning preference.
For Gaussian-map feedback evidence, the report preset now also uses
`lidar_max_frame_points=500`, `tracking_max_pose_step_m=0.025`,
`sliding_window_max_feedback_translation_m=0.05`, and
`sliding_window_max_feedback_velocity_mps=0.5`. On the 70 s CBD probe this keeps
first-aligned path drift low but still misses first-align RMSE/mean; the same
artifact passes the yaw-gauge comparison (`coverage=44.5%`, `RMSE=1.65 m`,
`mean=1.43 m`, `max=3.07 m`). The remaining non-paper-grade gap is therefore
native yaw observability/initial heading, not dataset availability or IMU scale.
The script also exposes `visual_alignment_window_weight` and
`se3_photometric_window_weight` for the native visual BA path. Enabling visual
plus Gaussian feedback on the 75 s CBD probe activates both factor families
(`98` visual and `98` SE3 photometric factors in the 70 s diagnostic; the 75 s
artifact records the same path active) and nearly closes first-align tracking:
coverage `42.65%`, RMSE `1.83 m`, max `3.49 m`, path drift `7.8%`, with mean
`1.5208 m` just over the `1.5 m` gate. The yaw-gauge comparison for the same
75 s artifact passes (`RMSE=1.461 m`, `mean=1.373 m`, `max=2.499 m`).

2026-05-13 scan-to-scan relative pose follow-up: `TrajectoryEstimator` now
supports two-timestamp relative position and SO(3) priors, and
`ContinuousTimeSlidingWindow` promotes them with the same signed-nanosecond
window semantics as other factors. `continuous_time_node` can route
consecutive-scan ICP targets into those residuals through
`lidar_scan_to_scan_use_relative_pose_factor`, with the flag forwarded by the
native reference parity script. The implementation is covered by both batch and
streaming probes, but remains default-off: CBD 12 s relative pose runs consumed
the expected Ceres factors while shortening the optimized path to
`0.0355-0.0548 m` against a `0.9159 m` reference and leaving RMSE near
`0.144 m`. The missing piece is now target quality/global coupling, not the
relative-pose residual surface itself.

The same path now has a target-quality gate for degenerate scan-to-scan ICP
translation estimates. `lidar_scan_to_scan_min_target_prediction_ratio` and
`lidar_scan_to_scan_min_target_translation_m` detect when the corrected target
motion collapses relative to the predicted motion; the node can then either use
the predicted relative translation or skip translation priors while still
reporting raw target length, prediction ratio, fallback counts, and skipped
prior counts. CBD 12 s probes show why this remains diagnostic: prediction
fallback fires and prevents silent target collapse, but low weights still freeze
the optimized path, while high weights overscale it without improving RMSE. The
next parity work is therefore a global visual/Gaussian-map trajectory-shape
constraint, not another scan-to-scan scalar clamp.

## Dataset Profiles

`gaussian_lic_bringup/config` includes ROS2 mapping profiles derived from the upstream Gaussian-LIC YAML files:

```text
fastlivo.yaml
fastlivo2.yaml
m2dgr.yaml
mcd.yaml
r3live.yaml
```

These files currently cover the ROS2 native mapper surface: topic names, synchronization/QoS settings, upstream camera intrinsics, image size, `select_every_k_frame`, depth-completion toggles, SH degree, Gaussian scaling seed, upstream optimizer/loss/exposure parameter names, output topics, and save-map service name. They are not complete tracking configs; upstream dataset rosbag paths, LiDAR/IMU/camera extrinsics, and LIC optimization weights still need a native Gaussian-LIC2 ROS2 frontend/tracking adapter before real bags can be launched as a full end-to-end system.

For parameter-file smoke testing with the synthetic `1x1` demo bag, skip the red-pixel preview assertion:

```bash
./scripts/smoke_test.sh \
  --bag bags/synthetic_gs_demo \
  --config src/gaussian_lic_bringup/config/fastlivo2.yaml \
  --render-mode debug_cpu \
  --skip-rendered-data-check \
  --tf
```

## Local Dependency Probe

Current machine state:

```text
libtorch:  /home/frank/Software/libtorch
CUDA:      /usr/local/cuda-12.8
TensorRT:  not found
```

The ROS2 parameter contract accepts `depth_completion`, `depth_completion_engine_path`, `patch_size`, `max_depth`, and `require_depth_topic`. The native TensorRT/SPNet wrapper is available when built with `GAUSSIAN_LIC_ENABLE_TENSORRT=ON`; without a configured SPNet `.engine` file, the mapper keeps using the provided depth topic or sparse point-projected depth.

The optional torch backend can be built with:

```bash
GAUSSIAN_LIC_ENABLE_TORCH=ON ./scripts/build_ros2.sh --packages-select gaussian_lic_mapping
```

2026-05-13 continuous-time BA note: direct control-point position/orientation
priors are now available in `TrajectoryEstimator`, and
`ContinuousTimeSlidingWindow` can inject default-off retained-knot soft anchors
through `retained_knot_*` parameters. This makes the Coco-LIC fixed-prefix /
marginalization behavior executable as an ablation instead of a hard-coded
assumption. CBD 12 s probes show the hook is not yet a parity setting by itself:
retained-knot soft anchors (`retained_knot_prior_count=4`, weight `10`) record
RMSE `0.174 m` and path `8.53 m` against a `0.91 m` reference path; direct
scan-to-scan position priors record RMSE `0.170 m` and path `5.77 m`; velocity
only records RMSE `0.184 m`. The stable short-window configuration remains the
safer default while full Schur marginalization/global visual-map coupling is
completed.
`lidar_scan_to_scan_relative_translation_gain` now scales the relative
translation before both position and velocity priors are formed, so future sweeps
can damp ICP translation scale without forking the node. A CBD 12 s gain `0.15`
probe reduced cumulative scan-target path to `1.24 m` but still produced a
`5.97 m` optimized path and RMSE `0.185 m`, so the blocker is the coupled
optimization surface rather than only raw ICP translation magnitude.
The newer `lidar_scan_to_scan_use_relative_pose_factor` branch feeds the same
ICP target into two-timestamp relative position/SO(3) factors instead of
velocity/angular-velocity priors. CTest proves the math is active, but CBD 12 s
evidence freezes path scale, so it is an ablation hook rather than a production
default.
`lidar_scan_to_scan_use_prediction_on_small_target` and
`lidar_scan_to_scan_skip_translation_priors_on_small_target` make that ablation
safe to sweep when the raw ICP target is much smaller than the predicted B-spline
motion.

It currently provides:

```text
MapperDataset CameraFrameRecord -> TorchCamera
OpenCV RGB/depth float matrices -> torch tensors
Eigen pose/intrinsics -> world-view/projection/full-projection tensors
MapperDataset pending points/colors/depth -> TorchGaussianMap initialization tensors
```

When `mapping_node` is built with `GAUSSIAN_LIC_ENABLE_TORCH=ON`, launch can enable live conversion with:

```bash
enable_torch_camera_conversion:=true
enable_torch_gaussian_init:=true
enable_torch_gaussian_extend:=true
enable_torch_gaussian_optimization:=true
torch_gaussian_optimization_steps:=100
torch_gaussian_optimization_sampling:=upstream_random
torch_gaussian_optimization_seed:=20260505
enable_torch_gaussian_pruning:=true
torch_gaussian_max_foreground:=1500000
torch_gaussian_device:=cuda
```

The Gaussian initialization mirrors the tensor boundary of upstream `GaussianModel::initialize()` and is gated on keyframes to match the upstream mapping loop:

```text
xyz:           [N, 3]
features_dc:   [N, 1, 3]
features_rest: [N, (sh_degree + 1)^2 - 1, 3]
scaling:       [N, 3]
rotation:      [N, 4]
opacity:       [N, 1]
```

The torch backend now supports optional upstream-style skybox seeding, incremental foreground insertion from pending keyframe points, a dependency-gated photometric tensor update, and bounded-map pruning. When `enable_torch_gaussian_optimization:=true` and `torch_gaussian_optimization_steps` is positive, visible foreground Gaussians are projected into the keyframe image and a small Torch backward pass updates DC color and opacity logits from image supervision. When `enable_torch_gaussian_pruning:=true`, low-opacity foreground Gaussians can be removed and `torch_gaussian_max_foreground` can cap map growth. `render_mode:=rasterizer` publishes a CPU Gaussian splat preview from the live `TorchGaussianMap` using Gaussian centers, DC color, scale, opacity, and the current camera pose. It is still not the full mapper: the upstream CUDA rasterizer, densification, and full multi-term loss schedule remain separate porting steps. This keeps the heavy torch dependency compile-time optional while allowing the live ROS2 data path to exercise the same tensor boundaries, map-growth lifecycle, gradient-update hook, pruning hook, and rasterizer output topic that the full Gaussian core will need.

After initialization and each keyframe extension, the node publishes the current map as chunked `gaussian_lic_msgs/msg/GaussianArray` on `/gaussian_lic/gaussian_map` with reliable transient-local QoS. The message uses public transport values:

```text
rotation_xyzw: internal [w, x, y, z] tensor converted to [x, y, z, w]
scale:         exp(internal log-scale tensor)
opacity:       sigmoid(internal opacity-logit tensor)
SH order:      DC RGB triplet followed by remaining coefficient RGB triplets
```

The `/gaussian_lic/save_map` service writes the initialized Gaussian map to `point_cloud.ply` when given a directory path, or directly to the requested `.ply` path. The Gaussian PLY properties follow upstream Gaussian-LIC naming: `f_dc_*`, `f_rest_*`, `opacity`, `scale_*`, and `rot_*`. These are raw optimization tensor values, not sigmoid/exp activated transport values.

When the torch Gaussian map is not initialized, the same service falls back to the accumulated debug map points and writes a plain XYZRGB PLY with `red/green/blue` uchar fields. This keeps native ROS2 smoke tests useful without requiring libtorch.

## Full-Profile Strict Queue

`scripts/run_strict_parity_queue.sh` is the current execution path for the
remaining dataset evidence. It is intentionally built on the same conversion,
mapper-contract, ROS1 baseline, ROS2 current, and strict-report tools used by the
green FAST-LIVO2 CBD run, but parameterizes the upstream config and result paths
by profile:

```bash
./scripts/run_strict_parity_queue.sh --dry-run
./scripts/run_strict_parity_queue.sh --continue-on-error
```

This addresses the data-side blocker by making FAST-LIVO, FAST-LIVO2, M2DGR,
MCD, and R3LIVE strict artifact generation restartable. It does not change the
release gate: `scripts/check_strict_parity_matrix.py` must pass without
`--allow-incomplete` before the port can be called full paper parity.
