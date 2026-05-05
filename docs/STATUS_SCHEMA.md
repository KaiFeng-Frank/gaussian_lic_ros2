# Status Schema

`/gaussian_lic/status` uses `gaussian_lic_msgs/msg/MappingStatus`.
`/gaussian_lic/frontend/status` uses `gaussian_lic_msgs/msg/TrackingStatus`
when the native tracking frontend is launched.

## State Fields

```text
state            Aggregate node state.
tracking_state  Gaussian-LIC2 frontend state, or STATE_UNCONFIGURED when absent.
mapping_state   Gaussian-LIC backend state.
render_mode     One of RENDER_MODE_OFF, DEBUG_CPU, RASTERIZER, DEBUG_INPUT.
status_text     Human-readable diagnostic summary.
active_profile  Dataset/config profile name.
```

Valid component states:

```text
STATE_UNCONFIGURED
STATE_INACTIVE
STATE_ACTIVE
STATE_OPTIMIZING
STATE_ERROR
```

## Counters And Rates

```text
num_tracking_frames   Frames aligned or accepted by the tracking/frontend path.
num_mapping_frames    Frames converted or consumed by the mapping/backend path.
num_keyframes         Current mapper keyframe count.
num_gaussians         Current Gaussian count, zero when the Gaussian backend is not initialized.
num_dropped_messages  Dropped input messages caused by queues or sync trimming.
num_errors            Conversion, render, or backend initialization errors.
tracking_hz           Recent tracking/alignment rate.
mapping_hz            Recent mapping/conversion rate.
```

## Performance Fields

```text
tracking_latency_ms
mapping_latency_ms
mean_iteration_ms
gpu_memory_mb
```

The current mapping slice fills rates, counters, mapping latency, and mean iteration time. Full tracking/mapping ports must fill tracking latency and GPU memory fields before v0.4 strict reproduction.

## Native Tracking Status

`TrackingStatus` is the native frontend gate. It is published with transient-local
QoS on `/gaussian_lic/frontend/status` so smoke tests and rosbag runs can verify
the estimator did more than publish pass-through topics.

```text
state/status_text
num_raw_images
num_raw_pointclouds
num_raw_imus
num_published_poses
num_lidar_keyframes
lidar_map_points
last_lidar_points
last_lidar_matches
last_window_point_correspondences
last_window_plane_correspondences
total_window_point_correspondences
total_window_plane_correspondences
last_lidar_mean_residual_m
sliding_window_enabled
sliding_window_states
sliding_window_imu_factors
sliding_window_pose_priors
sliding_window_dense_priors
sliding_window_point_factors
sliding_window_plane_factors
sliding_window_visual_factors
sliding_window_se3_photometric_factors
sliding_window_marginalized_states
sliding_window_iterations
sliding_window_initial_cost
sliding_window_final_cost
sliding_window_gyro_bias_norm
sliding_window_accel_bias_norm
sliding_window_gyro_bias_observability
sliding_window_accel_bias_observability
sliding_window_converged
gaussian_snapshot_points
gaussian_snapshot_expected_total
gaussian_snapshot_chunks_received
gaussian_snapshot_expected_chunks
gaussian_snapshot_complete
visual_factor_enabled
visual_alignment_valid
visual_rmse
visual_subpixel_dx
visual_subpixel_dy
visual_photometric_valid
visual_photometric_pixels
visual_photometric_cost
visual_photometric_step_dx
visual_photometric_step_dy
```

`scripts/tracking_smoke_test.sh` asserts that the synthetic frontend bag reaches
`STATE_TRACKING`, publishes frontend odometry/path, and exercises the sliding
window with nonzero IMU, LiDAR point, bias-observability, and visual factors.
The visual gate replays a 32x32 Gaussian-pattern image bag while a
transient-local rendered-image publisher supplies the mapper-render reference,
then checks both subpixel alignment and photometric Gauss-Newton linearization
status, plus nonzero SE3 photometric window factors extracted from
rendered/current/depth images. The synthetic bag intentionally lowers the LiDAR
point threshold to one point; dataset profiles keep production thresholds.

## Render Mode Policy

`render_mode` is the canonical parameter.

```text
debug_cpu    CPU projected map preview diagnostic.
debug_input  Input image passthrough diagnostic.
rasterizer   Default CUDA Gaussian rasterizer preview/loss path.
off          Do not publish rendered preview images.
```

`rendered_image_mode` is a deprecated compatibility alias. CPU debug rendering is planned to be removed after the v0.3 rasterizer release stabilizes.
