# Upstream Inventory

Fetched upstream revisions:

- Gaussian-LIC/Gaussian-LIC2 primary upstream: `cd4c122`
- Coco-LIC legacy reference, only fetched with `scripts/fetch_upstreams.sh --with-legacy-cocolic`: `4ead7e4`

## Gaussian-LIC2 Status

Gaussian-LIC2 is now public in the same upstream repository used by Gaussian-LIC:

```text
https://github.com/APRIL-ZJU/Gaussian-LIC
```

The upstream project page is:

```text
https://xingxingzuo.github.io/gaussian_lic2/
```

The upstream README announces the Gaussian-LIC2 release on 2026-02-21 and includes the Gaussian-LIC2 citation. The public repository currently exposes only the `master` branch at `cd4c122`, with no separate LIC2 tag or branch. The checked tree includes LIC2-labeled depth-completion sources and the updated Gaussian backend surface, while its run instructions still reference Coco-LIC for odometry input. This ROS2 port therefore treats `external/Gaussian-LIC` as the primary upstream for all released Gaussian-LIC/Gaussian-LIC2 code, and treats Coco-LIC only as historical ROS1 reference material unless a specific compatibility issue requires it.

## Gaussian-LIC ROS Surface

Gaussian-LIC has a relatively thin ROS1 surface. The core mapping and CUDA code is mostly behind `mapping.cpp`, `mapping.h`, and `gaussian.cpp`.

ROS-facing files:

```text
external/Gaussian-LIC/CMakeLists.txt
external/Gaussian-LIC/src/mapping.cpp
external/Gaussian-LIC/src/mapping.h
external/Gaussian-LIC/src/gaussian.cpp
```

Current ROS1 runtime contract:

```text
Subscribed:
  /points_for_gs     sensor_msgs/PointCloud2
  /pose_for_gs       geometry_msgs/PoseStamped
  /image_for_gs      sensor_msgs/Image
  /depth_for_gs      sensor_msgs/Image

Private parameters:
  config_path
  result_path
  lpips_path
```

Direct ROS1 APIs to replace:

```text
ros::init
ros::NodeHandle
ros::Subscriber
ros::Rate
ros::Time
ros::spin
ros::package::getPath
image_transport::ImageTransport
image_transport::Subscriber
cv_bridge
tf / tf_conversions / eigen_conversions
```

The first native mapping port should copy only this surface into a new ROS2 package and keep CUDA/Gaussian internals close to upstream.

## Legacy Coco-LIC ROS Surface

Coco-LIC used to be the larger porting target because the original Gaussian-LIC runtime depended on its ROS1 tracking outputs. With Gaussian-LIC2 now released in `APRIL-ZJU/Gaussian-LIC`, the main porting target moves to the Gaussian-LIC2 frontend/tracking surface in that primary upstream. Coco-LIC still documents useful legacy behavior for bag reading, sensor synchronization, Livox parsing, feature extraction, odometry publication, and the mapper topics consumed by the Gaussian backend.

ROS-facing files include:

```text
external/Coco-LIC/CMakeLists.txt
external/Coco-LIC/msg/*.msg
external/Coco-LIC/src/odometry_node.cpp
external/Coco-LIC/src/odom/msg_manager.cpp
external/Coco-LIC/src/odom/msg_manager.h
external/Coco-LIC/src/odom/odometry_manager.cpp
external/Coco-LIC/src/odom/odometry_manager.h
external/Coco-LIC/src/odom/odometry_viewer.h
external/Coco-LIC/src/lidar/*feature_extraction*
external/Coco-LIC/src/camera/r3live*
external/Coco-LIC/src/camera/loam/*
```

Current Gaussian-LIC bridge topics are published by `odometry_viewer.h`:

```text
/image_for_gs   sensor_msgs/Image
/depth_for_gs   sensor_msgs/Image
/pose_for_gs    geometry_msgs/PoseStamped
/points_for_gs  sensor_msgs/PointCloud2
```

Important porting issues:

- `MsgManager` reads ROS1 bags directly via `rosbag::Bag`; native ROS2 needs either `rosbag2_cpp`/`rosbag2_py` or live subscriptions.
- Livox handling depends on `livox_ros_driver::CustomMsg`; ROS2 should support `livox_ros_driver2` and a PointCloud2 fallback.
- Coco-LIC custom messages need ROS2 versions only if a legacy compatibility mode exposes them externally.
- Many visualization publishers can be deferred; the minimal port only needs odometry/path plus the four `*_for_gs` mapper inputs.

## Proposed Port Order

1. Create `gaussian_lic_mapping` as a ROS2 C++ package with a placeholder executable and CMake dependency skeleton. Done.
2. Port Gaussian-LIC `mapping.cpp/.h` middleware synchronization surface from ROS1 to `rclcpp`. Done for the four mapper input topics.
3. Keep input topics compatible with `/image_for_gs`, `/depth_for_gs`, `/pose_for_gs`, `/points_for_gs` first. Done.
4. Inventory the released Gaussian-LIC2 code surface in `external/Gaussian-LIC`, prioritizing depth completion, rasterization, optimization, and any frontend/tracking code that appears in future upstream commits.
5. Replace upstream ROS1 bag reading or launch assumptions with rosbag2/live subscriptions instead of relying on ROS1 bridge.

## Current Native Mapping Behavior

`gaussian_lic_mapping/mapping_node` now buffers and aligns the four upstream mapper inputs using `/points_for_gs` as the reference timestamp. The default tolerance is 10 ms, matching upstream `getAlignedData()`.

The current node also converts the aligned messages into `MapperFrameData`, mirroring the non-torch part of upstream `Dataset::addFrame()`:

- RGB/depth OpenCV matrices
- Eigen pose
- colored world points
- per-point camera-frame depth

`MapperDataset` now accumulates those converted frames into train/test records and pending point/color/depth arrays.

The current node intentionally stops before upstream `Camera` tensor creation, Gaussian initialization, extension, and optimization. Those require bringing in the upstream CUDA/libtorch build surface. TensorRT should be optional because the local machine does not currently have TensorRT installed, and upstream only needs it for depth completion.

An optional torch backend probe now converts `CameraFrameRecord` into a torch-backed camera representation. This is not yet wired into the live mapping node by default.
