# Gaussian-LIC ROS2

Native ROS2 engineering port for [Gaussian-LIC](https://github.com/APRIL-ZJU/Gaussian-LIC) and its Coco-LIC tracking dependency.

The long-term target is a paper-level ROS2 reproduction of LiDAR-Inertial-Camera Gaussian Splatting SLAM: native ROS2 tracking, native Gaussian mapping, rosbag2 replay, offline reproducibility tooling, and strict comparison against the ROS1 upstream baseline.

This repository is **not a ROS1 bridge wrapper**. It is a clean ROS2 workspace that keeps algorithm code isolated from middleware glue so the tracking and mapping backends can be ported incrementally without locking the project into ROS1-era APIs.

## Current Release State

This checkpoint is the M1 infrastructure slice. It is useful today for ROS2 interface validation, synthetic rosbag2 smoke tests, profile/schema work, and offline artifact extraction. It is **not yet** the full Gaussian-LIC/Coco-LIC algorithm port.

Available now:

- Native ROS2 packages for messages, launch/config, mapping scaffold, and test tools.
- ROS2 synchronization/conversion for the Gaussian-LIC mapper input contract:
  `/points_for_gs`, `/pose_for_gs`, `/image_for_gs`, `/camera_info_for_gs`, `/depth_for_gs`, `/imu_for_gs`.
- Odometry, path, TF, debug map cloud, rendered debug preview, status, GaussianArray, and SaveMap interfaces.
- Optional libtorch/CUDA path for keyframe-gated Gaussian tensor initialization, skybox seeding, and incremental foreground insertion.
- Dataset profile YAMLs derived from upstream Gaussian-LIC: FAST-LIVO, FAST-LIVO2, M2DGR, MCD, and R3LIVE.
- `gaussian_lic_offline` CLI for rosbag2 artifact extraction without launching ROS nodes.
- M1 release docs, status schema, performance regression script, and Jazzy/Humble build-only CI skeleton.

Still pending:

- Native Coco-LIC tracking port.
- Full Gaussian rasterizer, optimizer, densification, pruning, and real rendered output.
- TensorRT depth completion as an optional backend.
- Strict FAST-LIVO2 reproduction against archived ROS1 upstream baseline artifacts.

## Platform

Primary tested platform:

```text
Ubuntu 24.04
ROS2 Jazzy
CUDA 12.8
libtorch 2.7.0
```

Humble is represented in CI as build-only scaffolding; Jazzy is the first-class development target.

On this machine, use the ASCII workspace path:

```bash
cd /home/frank/gaussian_lic_ros2
```

`/home/frank/桌面/gaussian_lic_ros2` is only a symlink. Some ROS2 interface-generation tools are fragile under non-ASCII parent paths.

## Quick Start

Build and run the default synthetic smoke test:

```bash
source /opt/ros/jazzy/setup.bash
./scripts/build_jazzy.sh
source install/setup.bash
./scripts/smoke_test.sh --tf
```

Run the full local verification wrapper:

```bash
./scripts/verify_workspace.sh --full-profiles
```

The build wrapper pins ROS2 interface generation to `/usr/bin/python3` so conda Python does not get selected by CMake.

## Synthetic Rosbag2 Demo

Generate a small synthetic bag:

```bash
./scripts/create_synthetic_bag.sh --output bags/synthetic_gs_demo
```

Replay it through the native mapper:

```bash
./scripts/smoke_test.sh --bag bags/synthetic_gs_demo --tf
```

Run the same bag with a dataset profile:

```bash
./scripts/smoke_test.sh \
  --bag bags/synthetic_gs_demo \
  --config src/gaussian_lic_bringup/config/fastlivo2.yaml \
  --render-mode debug_cpu \
  --skip-rendered-data-check \
  --tf
```

The synthetic demo image is `1x1`; real dataset intrinsics can project the synthetic point outside the image, so profile smoke tests skip the red-pixel assertion.

## Offline Mode

Extract reproducibility artifacts from a rosbag2 directory without starting a ROS launch:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run gaussian_lic_tools gaussian_lic_offline \
  --bag bags/synthetic_gs_demo \
  --output /tmp/gaussian_lic_offline_demo
```

Current outputs:

```text
trajectory.tum
point_cloud_debug.ply
metrics.json
```

This is the seed for the future `gaussian_lic_offline` reproduction binary. The current implementation reads mapper contract topics and writes debug artifacts; it does not yet run the full Coco-LIC/Gaussian-LIC algorithm offline.

## Torch Backend Probe

Build the optional libtorch/CUDA path:

```bash
GAUSSIAN_LIC_ENABLE_TORCH=ON ./scripts/build_jazzy.sh --packages-select gaussian_lic_mapping
source install/setup.bash
ros2 run gaussian_lic_mapping torch_backend_probe
```

Expected shape output includes:

```text
image_sizes=[3, 1, 1]
depth_sizes=[1, 1]
gaussian_xyz_sizes=[2, 3]
gaussian_features_dc_sizes=[2, 1, 3]
gaussian_features_rest_sizes=[2, 15, 3]
gaussian_count_after_init=1
appended_count=1
gaussian_count=2
```

On the tested machine, CUDA is available. TensorRT is not installed and must remain optional.

## ROS2 Interfaces

Default logical sensor contract:

```text
/camera/image
/camera/camera_info
/livox/lidar
/imu
/tf
/tf_static
```

Current mapper contract:

```text
/points_for_gs       sensor_msgs/msg/PointCloud2
/pose_for_gs         geometry_msgs/msg/PoseStamped
/image_for_gs        sensor_msgs/msg/Image
/camera_info_for_gs  sensor_msgs/msg/CameraInfo
/depth_for_gs        sensor_msgs/msg/Image
/imu_for_gs          sensor_msgs/msg/Imu
```

Current outputs:

```text
/gaussian_lic/odometry
/gaussian_lic/path
/gaussian_lic/map_points
/gaussian_lic/rendered_image
/gaussian_lic/gaussian_map
/gaussian_lic/status
/gaussian_lic/save_map
```

`/gaussian_lic/rendered_image` currently defaults to:

```text
render_mode:=debug_cpu
```

That mode is a CPU projected-map preview for debugging. The v0.3 milestone will switch the default to `render_mode:=rasterizer` after the CUDA rasterizer is ported. `rendered_image_mode` is kept only as a deprecated compatibility alias.

See [docs/STATUS_SCHEMA.md](docs/STATUS_SCHEMA.md) for the status topic schema.

## Dataset Profiles

Packaged profiles:

```text
src/gaussian_lic_bringup/config/fastlivo.yaml
src/gaussian_lic_bringup/config/fastlivo2.yaml
src/gaussian_lic_bringup/config/m2dgr.yaml
src/gaussian_lic_bringup/config/mcd.yaml
src/gaussian_lic_bringup/config/r3live.yaml
```

Validate profile schema:

```bash
./scripts/check_profiles.py
```

These profiles currently cover the native mapper surface: topic names, QoS, synchronization, upstream camera intrinsics, Gaussian initialization parameters, output topics, and active profile labels. Full real-dataset replay still needs the native Coco-LIC tracking port and real bag/topic adapters.

## Performance Regression

Compare current metrics against a baseline:

```bash
./scripts/perf_regression.py \
  --metrics results/fastlivo2/current/metrics.json \
  --baseline baseline/fastlivo2/CBD_Building_01/metrics.json
```

Default gates check:

```text
tracking_hz
mapping_hz
mean_iteration_ms
```

A regression greater than 15% fails by default.

## Release Roadmap

The public plan is release-driven:

```text
v0.1.0  Baseline & Infrastructure
v0.2.0  Coco-LIC ROS2 Native Tracking
v0.3.0  Gaussian Mapping Full Backend
v0.4.0  Strict FAST-LIVO2 Reproduction
```

Immediate next action:

```text
Prepare ROS1 upstream baseline environment:
docker pull ros:noetic-ros-base-focal
```

Then run the original Gaussian-LIC/Coco-LIC on FAST-LIVO2 and archive artifacts under:

```text
baseline/fastlivo2/<sequence>/
```

Current blocker:

```text
FAST-LIVO2 bag data is required at /home/frank/data/fast_livo/
```

See [docs/RELEASE_MILESTONES.md](docs/RELEASE_MILESTONES.md) and [docs/ROADMAP.md](docs/ROADMAP.md).

## Docker

Non-torch Jazzy smoke-test container:

```bash
docker compose -f docker/compose.yaml run --rm smoke
```

See [docs/DOCKER.md](docs/DOCKER.md).

## Upstream Sources

Fetch upstream sources for porting work:

```bash
./scripts/fetch_upstreams.sh
```

This clones:

```text
external/Gaussian-LIC
external/Coco-LIC
```

Those directories are ignored by git. Both upstream projects are GPL-3.0.

## License

This repository is GPL-3.0-or-later compatible because Gaussian-LIC and Coco-LIC are GPL-3.0 upstream projects. Do not copy upstream source into this repository under a different license.
