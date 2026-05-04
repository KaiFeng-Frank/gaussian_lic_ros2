# Smoke Tests

## Build

```bash
cd /home/frank/gaussian_lic_ros2
./scripts/build_jazzy.sh
```

Expected result:

```text
Summary: 4 packages finished
```

Validate packaged launch profiles:

```bash
./scripts/check_profiles.py
```

Expected result includes:

```text
[profile] default.yaml: OK
[profile] fastlivo2.yaml: OK
```

Run the local verification wrapper:

```bash
./scripts/verify_workspace.sh --full-profiles
```

Add `--torch` to include the optional libtorch build, backend probe, Gaussian map topic, and Gaussian PLY save checks.

## Native Mapping Synchronization

The shortest local check is:

```bash
cd /home/frank/gaussian_lic_ros2
./scripts/build_jazzy.sh
./scripts/smoke_test.sh --tf
```

Run the native ROS2 mapping scaffold with synthetic synchronized input:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch gaussian_lic_bringup run_bag.launch.py \
  stub_mode:=false \
  synthetic_input:=true \
  use_sim_time:=false \
  use_composition:=true \
  rviz:=true
```

Expected mapping-node log:

```text
received points=... pose=... image=... depth=... imu=... | aligned=... | converted=... train=... test=... dataset_frames=... | conversion_errors=0
camera_info=... intrinsics=camera_info fx=... fy=... cx=... cy=...
```

`aligned` and `converted` should increase at approximately `synthetic_gs_frame_pub.publish_rate_hz`.
With the default synthetic publisher:

```text
last_points=1 pending_points=... total_points=...
```

confirms the PointCloud2 `x/y/z/rgb` fields were converted and accumulated into `MapperDataset`.
When `rviz:=true`, RViz2 opens the packaged `gaussian_lic.rviz` view with odometry, path, accumulated map points, rendered-image preview, and TF displays.

Verify the ROS2 tracking/map-point outputs from another shell:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 topic echo --once --timeout 5 /gaussian_lic/odometry nav_msgs/msg/Odometry
ros2 topic echo --once --timeout 5 /camera_info_for_gs sensor_msgs/msg/CameraInfo
ros2 topic echo --once --timeout 5 --no-arr /imu_for_gs sensor_msgs/msg/Imu
ros2 topic echo --once --timeout 5 \
  --qos-profile default \
  --qos-reliability reliable \
  --qos-durability transient_local \
  --qos-depth 1 \
  --field poses \
  /gaussian_lic/path \
  nav_msgs/msg/Path
ros2 topic echo --once --timeout 5 --no-arr \
  /gaussian_lic/map_points \
  sensor_msgs/msg/PointCloud2
ros2 topic echo --once --timeout 5 --no-arr \
  --qos-profile default \
  --qos-reliability reliable \
  --qos-durability transient_local \
  --qos-depth 1 \
  /gaussian_lic/rendered_image \
  sensor_msgs/msg/Image
ros2 topic echo --once --timeout 5 \
  /gaussian_lic/status \
  gaussian_lic_msgs/msg/MappingStatus
```

Expected output shape:

```text
/gaussian_lic/odometry: child_frame_id=camera, pose in frame_id=map
/camera_info_for_gs: k=[1.0, 0.0, 0.5, 0.0, 1.0, 0.5, 0.0, 0.0, 1.0]
/imu_for_gs: frame_id=imu, stationary synthetic angular velocity, gravity-like z acceleration
/gaussian_lic/path: poses length grows while synthetic input is running
/gaussian_lic/map_points: accumulated height=1 cloud; width grows up to max_map_points; fields=x/y/z/rgb point_step=16
/gaussian_lic/rendered_image: RGB8 CPU projected-map preview; synthetic data contains the projected red map point
/gaussian_lic/status: tracking_hz and mapping_hz are nonzero while frames are being processed
```

Optional TF smoke test:

```bash
ros2 launch gaussian_lic_bringup run_bag.launch.py \
  stub_mode:=false \
  synthetic_input:=true \
  use_sim_time:=false \
  publish_tf:=true
ros2 topic echo --once --timeout 5 /tf tf2_msgs/msg/TFMessage
```

Expected transform:

```text
frame_id: map
child_frame_id: camera
```

## Synthetic Rosbag2 Replay

Create a small sample bag from the synthetic mapper input publisher:

```bash
./scripts/create_synthetic_bag.sh --output bags/synthetic_gs_demo --duration 4
```

Expected `ros2 bag info` summary:

```text
Topic: /points_for_gs
Topic: /pose_for_gs
Topic: /image_for_gs
Topic: /camera_info_for_gs
Topic: /depth_for_gs
Topic: /imu_for_gs
```

Check the mapper contract directly:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run gaussian_lic_tools gaussian_lic_bag_check \
  --bag bags/synthetic_gs_demo \
  --json
```

The same command accepts `--bag-format ros1` for a ROS1 `.bag` when the optional `rosbags` package is available in that Python environment. This is useful before converting archived upstream bags to rosbag2.

Run the same mapper checks from rosbag2 playback:

```bash
./scripts/smoke_test.sh --bag bags/synthetic_gs_demo --tf
```

The smoke script launches rosbag2 with `loop_bag:=true` so volatile input-topic checks do not race short bag playback.

The non-torch smoke path also calls `/gaussian_lic/save_map` and checks that a debug XYZRGB PLY is written.

Reliable input-QoS rosbag2 check:

```bash
./scripts/smoke_test.sh --bag bags/synthetic_gs_demo --sensor-qos reliable --tf
```

Dataset-profile launch check:

```bash
./scripts/smoke_test.sh \
  --bag bags/synthetic_gs_demo \
  --config src/gaussian_lic_bringup/config/fastlivo2.yaml \
  --render-mode debug_cpu \
  --skip-rendered-data-check \
  --tf
```

The synthetic bag is `1x1`, while the upstream-derived dataset profiles use real camera intrinsics such as `fx ~= 646` and `cx ~= 313`. `--skip-rendered-data-check` keeps this check focused on parameter-file loading, topic synchronization, status, odometry, path, map-point, TF, and save-map behavior without requiring the synthetic point to project into a real-resolution image plane.

Available ROS2 profile files:

```text
src/gaussian_lic_bringup/config/fastlivo.yaml
src/gaussian_lic_bringup/config/fastlivo2.yaml
src/gaussian_lic_bringup/config/m2dgr.yaml
src/gaussian_lic_bringup/config/mcd.yaml
src/gaussian_lic_bringup/config/r3live.yaml
```

Composable-node rosbag2 check:

```bash
./scripts/smoke_test.sh --bag bags/synthetic_gs_demo --composition --tf
```

Torch-enabled rosbag2 check:

```bash
GAUSSIAN_LIC_ENABLE_TORCH=ON ./scripts/build_jazzy.sh --packages-select gaussian_lic_mapping
./scripts/build_jazzy.sh --packages-select gaussian_lic_bringup
./scripts/smoke_test.sh --bag bags/synthetic_gs_demo --torch --tf
```

## Optional Torch Backend Probe

```bash
GAUSSIAN_LIC_ENABLE_TORCH=ON ./scripts/build_jazzy.sh --packages-select gaussian_lic_mapping
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run gaussian_lic_mapping torch_backend_probe
```

Expected output on a working libtorch install:

```text
torch_version=...
cuda_available=true
image_sizes=[3, 1, 1]
depth_sizes=[1, 1]
world_view_sizes=[4, 4]
projection_sizes=[4, 4]
gaussian_xyz_sizes=[2, 3]
gaussian_features_dc_sizes=[2, 1, 3]
gaussian_features_rest_sizes=[2, 15, 3]
gaussian_count_after_init=1
appended_count=1
gaussian_count=2
```

## Live Mapping Node With TorchCamera And Incremental Gaussian Map

The shortest local torch-enabled check is:

```bash
GAUSSIAN_LIC_ENABLE_TORCH=ON ./scripts/build_jazzy.sh --packages-select gaussian_lic_mapping
./scripts/build_jazzy.sh --packages-select gaussian_lic_bringup
./scripts/smoke_test.sh --torch --tf
```

Build mapping with torch enabled:

```bash
GAUSSIAN_LIC_ENABLE_TORCH=ON ./scripts/build_jazzy.sh --packages-select gaussian_lic_mapping
./scripts/build_jazzy.sh --packages-select gaussian_lic_bringup
```

Run the live node with synthetic input and torch conversion enabled:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch gaussian_lic_bringup run_bag.launch.py \
  stub_mode:=false \
  synthetic_input:=true \
  use_sim_time:=false \
  enable_torch_camera_conversion:=true \
  enable_torch_gaussian_init:=true \
  enable_torch_gaussian_extend:=true \
  torch_gaussian_device:=cpu
```

Expected log:

```text
Torch camera conversion enabled
torch_cameras=... torch_errors=0 torch_image=[3, 1, 1] torch_depth=[1, 1]
Initialized Torch Gaussian map: foreground=... skybox=0 xyz=[..., 3] features_dc=[..., 1, 3] device=cpu
Extended Torch Gaussian map: inserted=... foreground=... skybox=0 xyz=[..., 3] device=cpu
torch_gaussians=... gaussian_inits=1 gaussian_extends=... gaussian_init_errors=0 gaussian_extend_errors=0
```

Verify the transient-local Gaussian map output from another shell:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 topic echo --once --timeout 5 \
  --qos-profile default \
  --qos-reliability reliable \
  --qos-durability transient_local \
  --qos-depth 1 \
  /gaussian_lic/gaussian_map \
  gaussian_lic_msgs/msg/GaussianArray
```

Expected fields:

```text
total_count: ...
chunk_index: 0
chunk_count: 1
gaussians:
- id: 0
  xyz: [0.0, 0.0, 1.0]
  opacity: 0.10000000149011612
```

Verify the save-map service:

```bash
ros2 service call /gaussian_lic/save_map \
  gaussian_lic_msgs/srv/SaveMap \
  "{path: /tmp/gaussian_lic_save_test, include_skybox: false}"
```

Expected response:

```text
success=True
message='saved Gaussian map to /tmp/gaussian_lic_save_test/point_cloud.ply'
```

The saved PLY should contain upstream-compatible Gaussian fields:

```bash
sed -n '1,12p' /tmp/gaussian_lic_save_test/point_cloud.ply
tail -n 1 /tmp/gaussian_lic_save_test/point_cloud.ply
```

This initializes foreground Gaussian tensors from keyframe-gated `MapperDataset` pending points, then appends later pending keyframe points into the same tensor map. The behavior matches the upstream initialize/extend lifecycle at the tensor boundary. Rasterization, optimization, pruning, and gradient-aware densification are still later porting slices.
