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

The CUDA rasterizer, fused SSIM loss, visibility-masked SparseGaussianAdam, densification/pruning, and strict CUDA rendered preview are ported behind the Torch/CUDA build profile. The native tracking package now provides timestamp-safe trajectory/IMU primitives, signed-nanosecond IMU history interpolation, an IMU preintegration residual foundation, per-point LiDAR deskew, an optional sliding-window optimizer foundation, native odometry/path/TF publication, a bounded 6-DoF LiDAR residual correction factor, a visual residual path that compares mapper rendered images with incoming camera frames, and a `GaussianArray` snapshot subscription for the mapper-to-tracker reverse channel. Full Coco-LIC2-grade production joint BA is still separate from this mapper backend work.

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
