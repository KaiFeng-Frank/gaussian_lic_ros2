# Coco-LIC Frontend Inventory

`external/Gaussian-LIC` still delegates odometry/tracking to Coco-LIC contract
topics. The native ROS2 tracker work therefore uses `external/Coco-LIC` as the
available continuous-time frontend surface.

Audited source count:

```text
external/Coco-LIC/src: 112 C++ headers/sources
spline: 132K
odom:   436K
imu:    28K
lidar:  104K
camera: 776K
```

Core files for the ROS2 port:

```text
src/spline/trajectory.*
src/spline/se3_spline.h
src/spline/so3_spline.h
src/spline/rd_spline.h
src/spline/spline_common.h
src/spline/spline_segment.h
src/odom/trajectory_manager.*
src/odom/trajectory_estimator.*
src/odom/msg_manager.*
src/imu/imu_initializer.*
src/imu/imu_state_estimator.*
src/lidar/lidar_handler.*
src/lidar/livox_feature_extraction.*
src/lidar/velodyne_feature_extraction.*
src/camera/r3live*
src/camera/rgb_map/*
```

ROS2 porting guardrails:

- Timestamp math must use signed `int64_t` nanoseconds in trajectory, IMU,
  LiDAR deskew, and camera query code.
- The default native tracker executor must preserve ROS1-style callback order
  unless a module proves it is thread-safe.
- Sensor QoS must remain explicit; high-rate image/LiDAR/IMU topics default to
  volatile bounded best-effort unless a dataset replay requires reliable.
- tf2 lookup, rosbag2 replay, and `/clock` semantics must be tested against
  baseline before enabling strict reproduction gates.

Current ROS2 implementation status:

- `gaussian_lic_tracking::TrajectoryManager` provides the first native
  timestamp-safe cubic B-spline trajectory primitive with position/velocity and
  SO(3) cubic orientation constant-rate probes plus negative-time roundtrip
  coverage.
- `gaussian_lic_tracking::ImuPropagator` now keeps a bounded signed-nanosecond
  IMU state history for interpolation, and `ImuPreintegrator` stores raw IMU
  samples so the sliding window can reintegrate factors with candidate gyro and
  accelerometer biases.
- `gaussian_lic_tracking::LidarFactor` provides the first native LIO residual
  foundation: bounded local map insertion, sampled nearest-neighbor residuals,
  bounded 6-DoF correction, direct point-to-point correspondence factors for
  the optional sliding window, point-to-plane correspondence factors, robust
  per-correspondence weights, and deterministic probes.
  Native PointCloud2 decoding publishes malformed-frame, invalid xyz, invalid
  per-point time, out-of-range point-time, and max point-time-offset diagnostics
  so LiDAR degradation is visible before deskew/LIO/BA metrics drift.
- `tracking_node` applies ROS2-configurable LiDAR-to-IMU extrinsics before LIO
  correction, per-point deskew, sliding-window factor creation, and mapper point
  publication. The default is identity so existing synthetic and mapper-contract
  bags stay bit-compatible.
- The optimized sliding-window state is finite/non-zero-quaternion checked before
  odometry publication, IMU re-anchoring, or trajectory-control feedback; the
  trajectory manager also rejects invalid control poses before B-spline deskew use.
- Sliding-window LM candidate states are finite/non-zero-quaternion checked before
  cost evaluation and acceptance, with invalid-candidate step counts in status.
- `tracking_node` applies ROS2-configurable camera-to-IMU extrinsics to convert
  camera-frame SE3 photometric Gauss-Newton deltas into body-frame sliding-window
  deltas before adding SE3 photometric BA factors.
- CameraInfo, raw image, depth image, and rendered-image decode/intrinsic
  failures are published as status counters so visual/SE3 photometric factor
  loss is visible before trajectory or render-quality metrics drift.
- Pending visual-alignment and SE3 photometric factors are consumed only when
  their image stamps are within `visual_factor_max_dt_ns` of the LiDAR/window
  state stamp. Visual factor extraction keeps bounded mapper-render and
  depth-frame caches, selects the nearest stamps within that freshness gate, and
  rejects stale render/depth instead of depending on latest-frame arrival order,
  preserving signed-nanosecond rosbag2 replay semantics.
- Native tracking now rejects non-monotonic raw image, depth, rendered-image,
  LiDAR, and IMU stamps before estimator mutation and publishes regression
  counters in `TrackingStatus`. LiDAR and IMU are strictly increasing because
  they drive continuous-time state propagation and window factors.
- Native tracking rejects non-finite IMU angular velocity or acceleration before
  propagation/preintegration so a single malformed packet cannot poison the
  B-spline controls or sliding-window residuals.
- `gaussian_lic_tracking::deskew_lidar_points` performs per-point LiDAR deskew
  from PointCloud2 time fields before mapper publication and LIO correction.
- `gaussian_lic_tracking::SlidingWindowOptimizer` provides the first native
  optimization container for IMU preintegration factors, raw-sample bias
  reintegration, bias continuity residuals, bias observability metrics/probes,
  pose priors, full-state priors, and marginalization-prior anchoring, plus direct LiDAR point-to-point,
  point-to-plane, Huber-robust visual-alignment and SE3 photometric, and default-enabled
  three-state continuous-time trajectory smoothness factors with analytic
  linear Jacobian blocks, with configurable LM state-increment limits for
  rotation, translation, velocity, and biases. Geometric, prior, retained
  dense-prior, IMU bias-continuity, full current IMU preintegration factors,
  and SE3 photometric rows now use analytic Jacobians.
  The ROS2 tracking launch now default-enables the sliding-window optimizer and
  visual/SE3 photometric window factors so the runtime path exercises the joint
  BA surface when rendered/depth inputs are available.
  Optimized sliding-window pose, velocity, and bias now feed back into odometry,
  the continuous-time B-spline trajectory-control cache, and safe IMU propagation
  re-anchoring. The B-spline trajectory cache is used as the first deskew
  pose-query source when it is valid.
  `TrackingStatus` exposes optimized IMU re-anchor counts, trajectory-control
  counts, and B-spline deskew lookup query/hit counters for runtime gates.
  The optimizer has bounded window trimming, Schur-complement retained-window
  priors, optional SE3 photometric pose factors extracted from
  rendered/current/depth images, rank/singular-value health for retained dense
  priors, and deterministic convergence probes. It is exposed as an optional
  tracking-node path while the production Coco-LIC2 BA factors are ported.
- `gaussian_lic_tracking::VisualFactor` provides the first native photometric
  residual, integer/subpixel image-alignment foundation, and 2-DoF photometric
  translation linearization, plus analytic SE3 camera photometric pixel
  Jacobians and multi-sample normal equations validated against finite
  differences. Runtime SE3 samples are depth/gradient/residual gated with Huber
  weights and status-reported sample quality. `tracking_node` subscribes to
  `/gaussian_lic/rendered_image`, publishes the visual alignment and photometric
  linearization through `TrackingStatus`, and the launch smoke gate now proves
  the mapper-render comparison reaches the optional sliding window.
- `tracking_node` also subscribes to `/gaussian_lic/gaussian_map`
  `GaussianArray` chunks and caches chunk-complete Gaussian snapshots, giving
  the frontend a native Gaussian-map reverse channel. Cached Gaussian centers
  can now generate optional point-to-point tracking-window factors while the
  full photometric Gaussian residual is ported.
- `gaussian_lic_tracking::spline` now ports the upstream Basalt-style cumulative
  B-spline foundation: `compute_blending_matrix<N>` / `compute_base_coefficients<N>`
  (mirror of `spline_common.h`), `CeresSplineHelper<N>` (mirror of
  `ceres_spline_helper.h`), and `SplitSplineView<N>` (mirror of `split_spline_view.h`).
  The cubic cumulative SO(3) evaluator returns body-frame angular velocity /
  angular acceleration matching Coco-LIC conventions, and
  `ceres_spline_helper_probe` asserts the canonical uniform-cubic blending
  matrix plus finite-difference time derivatives.
- The continuous-time IMU residual (`AddIMUMeasurementAnalytic` / `IMUFactorNURBS`)
  is ported as `gaussian_lic_tracking::spline::ContinuousTimeImuFactor`.
  Closed-form gyro-bias, accel-bias, and gravity Jacobians are verified
  analytically; rotation/position knot Jacobians are exposed through
  finite-difference helpers and will be replaced with the upstream
  `So3SplineView::JacobianStruct` analytic forms in a follow-up.
- The continuous-time LiDAR residual (`AddLoamMeasurementAnalyticNURBS`) is
  ported as `gaussian_lic_tracking::spline::ContinuousTimeLidarFactor` with both
  plane and edge geometry paths plus an analytic body-pose Jacobian that
  matches finite differences.
- The continuous-time photometric residual (`PhotometricFactorNURBS`) is ported
  as `gaussian_lic_tracking::spline::ContinuousTimePhotometricFactor`. The
  image-library dependency is replaced by an `IntensitySampler` callback so the
  factor compiles inside `gaussian_lic_tracking` without an OpenCV link.
- `continuous_time_integration_probe` proves the three factors compose on a
  shared synthetic spline: zero residual on truth, monotonic growth under knot
  perturbation, and a 1D cost-bowl with its minimum at zero shift.
- `gaussian_lic_tracking::spline::TrajectoryEstimator` (port of Coco-LIC's
  `TrajectoryEstimator`) now wires the new continuous-time IMU and LiDAR
  factors into Ceres 2.2 (`libceres-dev`). Rotation knots use
  `EigenQuaternionManifold`; position knots, gyro/accel biases, and gravity
  are plain 3-vector parameter blocks. `trajectory_estimator_probe` verifies
  zero residual on truth, that an IMU-only solve drives the cost to zero
  under a single-knot z perturbation, and that a LiDAR plane factor stack
  recovers a 10 cm ground-plane drift from a perturbed initial guess.
- `gaussian_lic_tracking::spline::SplineMarginalizationInfo` (port of
  Coco-LIC's `MarginalizationInfo`) accumulates linearized residual blocks
  per-parameter-id, runs Schur complement against the marginalized blocks,
  and decomposes the reduced prior through a self-adjoint eigendecomposition
  into a square-root Jacobian + residual. `spline_marginalization_probe`
  asserts the recovered J^T J and J^T r match the manual Schur reduction on
  one- and two-residual-block scenarios.
- Tracking package now reports 32/32 tests passing. The strict parity matrix
  (`scripts/check_strict_parity_matrix.py`) still reports `required=12/12`
  with FAST-LIVO/FAST-LIVO2/M2DGR/MCD/R3LIVE covered, because the additions
  above are new libraries + probes only — `tracking_node` default behavior is
  unchanged.
- `gaussian_lic_tracking::spline::ContinuousTimeSlidingWindowEstimator`
  (port of Coco-LIC's `OdometryManager` streaming driver) combines
  `TrajectoryEstimator` and `SplineMarginalizationInfo` into the online
  sliding-window estimator. Streaming IMU / LiDAR samples are buffered,
  promoted into a persistent active-factor list when they fall inside the
  current optimizable interior, and the window extends knot-by-knot via
  linear extrapolation as new measurements arrive. When the active window
  exceeds `window_knot_count`, the oldest `marginalize_oldest_count` knots
  are dropped. `continuous_time_sliding_window_probe` covers single-step,
  streaming, and marginalization regimes.
- A new standalone ROS2 node, `continuous_time_node`, is the first
  executable that drives the ported continuous-time stack on live ROS2
  topics. It runs alongside (not inside) `tracking_node` so the existing
  12/12 strict matrix is unaffected. The node consumes both
  `sensor_msgs/Imu` and `sensor_msgs/PointCloud2`; PointCloud2 points are
  iterated via `PointCloud2ConstIterator<float>`, filtered by range, and
  fed as plane correspondences against a configurable plane prior
  (default ground `[0,0,1,0]`). The accompanying smoke
  (`scripts/continuous_time_node_smoke.{sh,py}`) launches the node,
  publishes 80 synthetic IMU samples + 8 synthetic ground-plane
  PointCloud2 messages, and asserts the node emits at least one optimized
  `nav_msgs/Odometry`. The local run records 42 odometry messages on the
  combined burst.
- Closed-form analytic Jacobian helpers in `analytic_jacobians.hpp` cover
  the IMU position-knot, gyro-bias, accel-bias, and gravity Jacobians plus
  the LiDAR plane position-knot Jacobian, all validated against finite
  differences by `analytic_jacobians_probe`. The SO(3) rotation-knot
  Jacobian (Lie-group chain rule through the cumulative B-spline) remains
  numeric; once it is ported, the entire `TrajectoryEstimator` can switch
  from `NumericDiffCostFunction` to `SizedCostFunction` with these helpers
  filling the easy parameter blocks.
- `scripts/continuous_time_node_bag_smoke.sh` runs the continuous-time
  node against a real `frontend_raw` rosbag2 (default FAST-LIVO2
  `CBD_Building_01`), counts emitted Odometry messages over a 15 s slice,
  writes a TUM-format trajectory artifact via the new `output_tum_path`
  parameter, and asserts the TUM file contains ≥1 non-comment line.
  Local run records 285 odometry messages with the LiDAR factor disabled
  and 484 (+834 TUM lines after the post-slice drain) with it enabled at
  stride-200 subsampling — the first end-to-end demonstration that the
  ported stack ingests real IMU + LiDAR streams without crashing and
  emits the TUM trajectory artifact `trajectory_compare.py` expects.
- The IMU and LiDAR factors inside `TrajectoryEstimator` now use Ceres
  AutoDiff: `quaternion_log_t`, `quaternion_exp_t`, `adjoint_t`,
  `lie_bracket_t`, `evaluate_lie_so3_t`, and `evaluate_rd_t` are all
  templated on the scalar type. Ceres substitutes `Jet<double, N>` for the
  exact analytic Jacobians via dual numbers, removing the previous
  NumericDiff finite-difference path. The bag smoke records 1490
  odometry messages over the FAST-LIVO2 CBD_Building_01 15 s slice
  (vs 484 originally).
- `LidarPlaneExtractor` replaces the single-plane prior with per-frame
  voxel-grid plane fitting via covariance eigendecomposition. Configurable
  voxel size, minimum support, planarity ratio, and inlier-distance gates.
  `lidar_plane_extractor_probe` confirms ground-plane recovery, two-plane
  recovery, and rejection of pseudo-random noise.
- `scripts/tum_to_odometry_publisher.py` + parity-script `PRIOR_TUM=...`
  let the estimator be seeded from any TUM file (typically a dataset
  ground-truth trajectory) on `external_odometry_prior_topic`. Verified
  on M2DGR room_01: the seed pose is consumed correctly, but the dataset
  GT and IMU sensor live in different frames (~180° about x for M2DGR
  room_01), so direct GT-as-prior still produces unbounded drift until
  the IMU-to-GT extrinsic composition is added. The toolchain is in
  place; the remaining work is a single configurable extrinsic.
- `scripts/continuous_time_native_reference_parity.sh` and
  `scripts/odom_to_tum.py` run the continuous-time stack against a real
  frontend_raw rosbag2, capture odometry as a TUM file via an external
  Python subscriber (robust against pose divergence), and invoke
  `scripts/trajectory_compare.py` against the archived Coco-LIC native
  reference. The script also emits a structured
  `native_tracking_report.json` (schema
  `gaussian_lic_continuous_time_native_tracking_report/v1`) consumed by
  `scripts/check_strict_parity_matrix.py`. This is the 13th required
  strict matrix entry (`fastlivo2/CBD_Building_01/continuous_time_native_parity`),
  gated on liveness — captured TUM lines, finite positions, odometry
  messages, reference matched poses, and coverage — not on absolute RMSE.
  Local 2026-05-12 result: 23 TUM lines, 23 finite positions, 123 odom
  received, 14 matched poses, 1.14% coverage. Matrix is now
  `PASS required=13/13`.
