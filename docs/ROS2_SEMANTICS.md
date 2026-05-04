# ROS2 Semantic Contract

Gaussian-LIC and Coco-LIC use message time as part of the estimator state, not just
as metadata. This file records the ROS2 behavior that must stay fixed while the
paper-level CUDA mapper and continuous-time frontend are being ported.

## Time Math

- Algorithmic time deltas and sync windows use signed `int64_t` nanoseconds.
- `rclcpp::Time` and double seconds are allowed only at ROS API or log boundaries.
- Zero and negative builtin stamps are preserved as numeric nanoseconds. They are
  not normalized through ROS clocks before B-spline, IMU, LiDAR deskew, or frame
  synchronization math.
- `mapping_node` keeps `sync_tolerance_sec` as the public parameter for profile
  compatibility, then converts it once to `sync_tolerance_nsec_`.
- `lic2_contract_adapter` IMU fallback integrates with nanosecond stamp deltas
  and rejects non-positive or over-limit intervals.

## QoS

- Sensor streams default to `best_effort`, `keep_last`, depth `5`.
- High-rate image, LiDAR, IMU, and depth topics should stay volatile and bounded.
- Reliable QoS is opt-in for controlled local rosbag2 replay or drivers known to
  publish reliable sensor streams.
- State and visualization outputs that are naturally latched, such as path and
  Gaussian map chunks, can use reliable transient-local QoS.

## Executor

- Strict replay and composition use the single-threaded `component_container`.
- The continuous-time frontend must not rely on callback interleaving from a
  `MultiThreadedExecutor`. If parallel callbacks are added later, callbacks must
  push messages into deterministic stamp-ordered queues before estimator updates.

## tf2

- Strict estimator math must not depend on implicit `tf2` lookup interpolation,
  timeout, or cache behavior.
- Current nodes only broadcast TF from already selected poses. A future tracker
  may consume static extrinsics from configuration, but any time-varying transform
  used for math needs an explicit stamped buffer and an upstream-parity test.

## Rosbag Replay

- Launch replay uses `ros2 bag play ... --clock --read-ahead-queue-size 1`
  and `use_sim_time:=true`.
- Strict baseline comparison still needs the same topic stamps and start order as
  the ROS1 run. If rosbag2 playback ordering is not enough for a target sequence,
  add a deterministic sequential replay helper instead of tuning estimator code.
- `scripts/rosbag2_timing_audit.py` checks `metadata.yaml` plus sqlite3 `.db3`
  message timestamp ordering and required topic counts before a replay is used
  as baseline evidence. MCAP bags are reported as metadata-only unless an MCAP
  parser is available in the local environment.
- `scripts/fastlivo2_ros1_to_frontend_raw.py` writes converted rosbag2 messages
  through a stable header-stamp sort buffer (`--sort-buffer-sec`, default 5s) so
  ROS1 bag connection ordering does not leak timestamp regressions into rosbag2
  playback.
- `scripts/strict_rosbag2_play.sh` wraps audit plus the fixed replay command for
  current-result collection scripts, keeping strict replay options in one place.

## Regression Check

Run:

```bash
./scripts/check_ros2_semantics_contract.py
```

The check fails if timestamp helpers regress to double-second math, dataset
profiles stop using bounded best-effort sensor QoS, composition defaults back to
the multi-threaded component container, launch replay drops `/clock`, or tf2
lookup consumption is introduced without a semantic wrapper.
