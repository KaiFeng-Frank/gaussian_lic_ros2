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
executor_callback_serialization_enabled
sensor_qos_reliability
sensor_qos_history
sensor_qos_depth
signed_nanosecond_time_math_enabled
last_image_stamp_ns
last_pointcloud_stamp_ns
last_imu_stamp_ns
image_stamp_regressions
depth_stamp_regressions
rendered_stamp_regressions
pointcloud_stamp_regressions
imu_stamp_regressions
imu_invalid_measurements
camera_info_invalid_intrinsics
image_invalid_frames
depth_invalid_frames
rendered_invalid_frames
num_raw_images
num_raw_pointclouds
num_raw_imus
num_published_poses
num_lidar_keyframes
lidar_map_points
last_lidar_points
lidar_invalid_points
lidar_invalid_point_times
lidar_out_of_range_point_times
last_lidar_max_abs_point_time_offset_s
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
sliding_window_smoothness_factors
sliding_window_point_factor_skip_count
sliding_window_plane_factor_skip_count
sliding_window_visual_factor_skip_count
sliding_window_se3_photometric_factor_skip_count
sliding_window_smoothness_factor_skip_count
sliding_window_imu_factor_skip_count
sliding_window_optimization_skip_count
sliding_window_invalid_optimized_states
sliding_window_marginalized_states
sliding_window_dense_prior_rows
sliding_window_dense_prior_cols
sliding_window_dense_prior_rank
sliding_window_normal_equation_rows
sliding_window_normal_equation_cols
sliding_window_normal_equation_rank
sliding_window_iterations
sliding_window_accepted_steps
sliding_window_rejected_steps
sliding_window_limited_steps
sliding_window_initial_cost
sliding_window_final_cost
sliding_window_last_step_norm
sliding_window_last_step_scale
sliding_window_last_damping
sliding_window_dense_prior_min_singular_value
sliding_window_dense_prior_max_singular_value
sliding_window_normal_equation_min_singular_value
sliding_window_normal_equation_max_singular_value
sliding_window_normal_equation_condition_number
sliding_window_gyro_bias_norm
sliding_window_accel_bias_norm
sliding_window_gyro_bias_observability
sliding_window_accel_bias_observability
sliding_window_converged
sliding_window_normal_equation_degenerate
sliding_window_imu_reanchors
trajectory_control_poses
trajectory_deskew_queries
trajectory_deskew_hits
trajectory_control_pose_skip_count
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
visual_se3_photometric_valid
visual_se3_photometric_candidates
visual_se3_photometric_samples
visual_se3_photometric_inlier_ratio
visual_se3_photometric_mean_abs_residual
visual_se3_photometric_cost
visual_se3_photometric_step_norm
```

`scripts/tracking_smoke_test.sh` asserts that the synthetic frontend bag reaches
`STATE_TRACKING`, publishes frontend odometry/path, and exercises the sliding
window with callback serialization enabled, nonzero signed-nanosecond image,
LiDAR, and IMU stamps, bounded best-effort sensor QoS, bias-observability, and
visual factors.
The sliding-window gate also requires nonzero dense-prior rank and singular
value coverage after marginalization so retained-state priors are numerically
observable rather than only present by count; the optimizer itself bounds LM
rotation, translation, velocity, and bias increments before applying a candidate
state update and reports accepted/rejected step counts, final step norm/scale,
and damping for runtime diagnosis.
It also gates nonzero trajectory smoothness factors, which constrain adjacent
three-state rotation-rate, position-rate, velocity-acceleration, and bias-rate
continuity in the native joint BA window.
The visual gate replays a 32x32 Gaussian-pattern image bag while a
transient-local rendered-image publisher supplies the mapper-render reference,
then checks both subpixel alignment, Huber-robust visual-alignment factors, and
photometric Gauss-Newton linearization status, plus nonzero Huber-robust SE3 photometric window factors extracted from
nearest-fresh-render/current/depth images. The SE3 status fields expose candidate pixels,
accepted robust samples, inlier ratio, mean absolute residual, cost, and
Gauss-Newton step norm so bad depth, flat gradients, or large photometric
outliers are visible in CI logs. The synthetic bag intentionally lowers the
LiDAR point threshold to one point; dataset profiles keep production thresholds.

## Render Mode Policy

`render_mode` is the canonical parameter.

```text
debug_cpu    CPU projected map preview diagnostic.
debug_input  Input image passthrough diagnostic.
rasterizer   Default CUDA Gaussian rasterizer preview/loss path.
off          Do not publish rendered preview images.
```

`rendered_image_mode` is a deprecated compatibility alias. CPU debug rendering is planned to be removed after the v0.3 rasterizer release stabilizes.
