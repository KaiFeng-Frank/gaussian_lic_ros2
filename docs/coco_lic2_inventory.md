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
- `tracking_node` applies ROS2-configurable LiDAR-to-IMU extrinsics before LIO
  correction, per-point deskew, sliding-window factor creation, and mapper point
  publication. The default is identity so existing synthetic and mapper-contract
  bags stay bit-compatible.
- `tracking_node` applies ROS2-configurable camera-to-IMU extrinsics to convert
  camera-frame SE3 photometric Gauss-Newton deltas into body-frame sliding-window
  deltas before adding SE3 photometric BA factors.
- Pending visual-alignment and SE3 photometric factors are consumed only when
  their image stamps are within `visual_factor_max_dt_ns` of the LiDAR/window
  state stamp. SE3 photometric sample extraction keeps a bounded depth-frame
  cache, selects the nearest stamp within that freshness gate, and rejects stale
  depth instead of depending on latest-frame arrival order, preserving
  signed-nanosecond rosbag2 replay semantics.
- `gaussian_lic_tracking::deskew_lidar_points` performs per-point LiDAR deskew
  from PointCloud2 time fields before mapper publication and LIO correction.
- `gaussian_lic_tracking::SlidingWindowOptimizer` provides the first native
  optimization container for IMU preintegration factors, raw-sample bias
  reintegration, bias continuity residuals, bias observability metrics/probes,
  pose priors, full-state priors, and marginalization-prior anchoring, plus direct LiDAR point-to-point,
  point-to-plane, visual-alignment, SE3 photometric, and default-enabled
  three-state continuous-time trajectory smoothness factors with analytic
  linear Jacobian blocks. Geometric, prior, retained
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
- Production sliding-window BA, production-scale marginalization policy, and
  Coco-LIC2-grade IMU/LiDAR/camera joint optimization still need to be ported
  from the audited Coco-LIC modules.
