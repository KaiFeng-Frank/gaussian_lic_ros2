# Calibration Schema

Gaussian-LIC ROS2 currently consumes camera intrinsics through `sensor_msgs/msg/CameraInfo` and falls back to the mapper parameters `fx`, `fy`, `cx`, and `cy`. The schema below is the repository convention for real robot and dataset calibration files while the native Gaussian-LIC2 frontend/tracking port is being completed.

## File Layout

```yaml
calibration:
  frames:
    world: map
    body: base_link
    camera: camera
    lidar: lidar
    imu: imu

  camera:
    model: pinhole
    image_width: 640
    image_height: 512
    intrinsics:
      fx: 646.78472
      fy: 646.65775
      cx: 313.456795
      cy: 261.399612
    distortion:
      model: plumb_bob
      coefficients: [0.0, 0.0, 0.0, 0.0, 0.0]

  extrinsics:
    T_body_camera:
      translation: [0.0, 0.0, 0.0]
      rotation_xyzw: [0.0, 0.0, 0.0, 1.0]
    T_body_lidar:
      translation: [0.0, 0.0, 0.0]
      rotation_xyzw: [0.0, 0.0, 0.0, 1.0]
    T_body_imu:
      translation: [0.0, 0.0, 0.0]
      rotation_xyzw: [0.0, 0.0, 0.0, 1.0]

  time_offsets_sec:
    camera_to_imu: 0.0
    lidar_to_imu: 0.0

  imu:
    gyroscope_noise_density: 0.00345557556
    gyroscope_random_walk: 0.0000002617993833
    accelerometer_noise_density: 0.02425376259
    accelerometer_random_walk: 0.00000196
```

## Conventions

- `T_parent_child` maps a point from the child frame into the parent frame.
- Quaternions are stored as `rotation_xyzw`, matching ROS message order.
- Intrinsics use pixels and should match the published `CameraInfo.k`.
- Time offsets use the same sign as the legacy upstream Coco-LIC comments: `camera_to_imu` is camera timestamp minus IMU timestamp.
- Dataset-specific ROS2 mapper profiles live in `src/gaussian_lic_bringup/config/*.yaml`; full tracking calibration should use this schema or a converter into this schema.

## Current Runtime Use

The native mapper currently uses:

```text
CameraInfo.k -> fx, fy, cx, cy
PoseStamped  -> world-to-camera pose already estimated by the tracking frontend
PointCloud2  -> world points already prepared for Gaussian initialization
```

That means extrinsics and IMU noise parameters are documented now but will be consumed by the native Gaussian-LIC2 frontend/tracking port, not by the current mapping-only slice.
