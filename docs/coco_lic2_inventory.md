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
  timestamp-safe cubic B-spline trajectory primitive with a constant-velocity
  probe and negative-time roundtrip coverage.
- `gaussian_lic_tracking::ImuPropagator` now keeps a bounded signed-nanosecond
  IMU state history for interpolation, and `ImuPreintegrator` stores raw IMU
  samples so the sliding window can reintegrate factors with candidate gyro and
  accelerometer biases.
- `gaussian_lic_tracking::LidarFactor` provides the first native LIO residual
  foundation: bounded local map insertion, sampled nearest-neighbor residuals,
  bounded 6-DoF correction, direct point-to-point correspondence factors for
  the optional sliding window, point-to-plane correspondence factors, robust
  per-correspondence weights, and deterministic probes.
- `gaussian_lic_tracking::deskew_lidar_points` performs per-point LiDAR deskew
  from PointCloud2 time fields before mapper publication and LIO correction.
- `gaussian_lic_tracking::SlidingWindowOptimizer` provides the first native
  optimization container for IMU preintegration factors, raw-sample bias
  reintegration, bias continuity residuals, bias observability metrics/probes,
  pose priors, full-state priors, and marginalization-prior anchoring, plus direct LiDAR point-to-point,
  point-to-plane, and visual-alignment factors. Geometric, prior, retained
  dense-prior, IMU bias-continuity, full current IMU preintegration factors,
  and SE3 photometric rows now use analytic Jacobians.
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
