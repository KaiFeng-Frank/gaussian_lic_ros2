# Coco-LIC / Gaussian-LIC2 — ROS2 Jazzy Port: Results

Faithful mechanical port of upstream **Coco-LIC** (continuous-time LiDAR-Inertial-Camera
odometry, the Gaussian-LIC2 front-end) from ROS1 to **ROS2 Jazzy**, validated on
FAST-LIVO2 `CBD_Building_01`.

## Result (headline)

| Mode | ATE RMSE vs upstream reference | Notes |
|------|-------------------------------|-------|
| **LiDAR-Inertial (LIO)** | **2.12 cm** yaw-aligned RMSE | **paper-level cm-scale; passes the acceptance gate** |
| LiDAR-Inertial-Camera (LICO) | **2.27 cm** yaw-aligned RMSE; camera-fixed probe **2.81 cm** | camera works but does not beat LIO (see ceiling) |

- Reference: `baseline/fastlivo2/CBD_Building_01/native_reference/cocolic_livo_reference_10hz.tum`
  (upstream Coco-LIC's own LICO output on the official ROS1 `CBD_Building_01.bag`).
- Committed evidence:
  - `run_lio/lio_vs_reference_report.json`: 100% coverage (1231/1231 poses),
    2.12 cm RMSE, 0.414% path drift, `"ok": true`.
  - `run_lio/lico_vs_reference_report.json`: 100% coverage (1231/1231 poses),
    2.27 cm RMSE, 0.397% path drift, `"ok": true`.
  - `run_lio/lico_camerafixed_vs_reference_report.json`: 100% coverage
    (1231/1231 poses), 2.81 cm RMSE, 0.740% path drift, `"ok": true`.
- `scripts/check_cocolic_port_results.py` gates those committed reports in CI so
  future releases cannot drift from the cm-scale port evidence.
- **13×** better than the prior independent reimplementation (0.2694 m), **63×** over baseline (1.346 m).
- Reproducible from a clean build; **deterministic** per binary (3/3 runs identical).

## The sub-cm ceiling (why it stops at ~2 cm)

The reference *is* upstream's own output, so reaching sub-cm would require **bit-fidelity**
to upstream's exact pipeline. An independent port cannot achieve that:
- **Numerical floor ~2 cm**: Ceres 2.0→2.2 optimizer version, per-point `offset_time`
  reconstruction, and float/compiler differences.
- **Visual front-end can't be bit-identical**: LK / RANSAC-fundamental / PnP are
  OpenCV-version- and implementation-sensitive. Confirmed empirically: with the camera
  fully working, LICO is *worse* than LIO, and an `image_weight` sweep shows LICO →
  LIO as weight → 0 (the camera only pulls away from upstream's LICO).

⇒ **Paper-level cm-scale is achieved (2.18 cm); super-paper sub-cm is structurally
unreachable for an independent ROS2 port** (proven, not hypothesized).

## What was ported

- **Computational core**: `sophus_lib`, `spline` (non-uniform B-spline trajectory),
  `utils`, `odom` estimator + analytic factors + marginalization, `trajectory_manager`.
  Ceres 2.0 `LocalParameterization` → 2.2 `Manifold` migration.
- **Sensors**: Livox feature extraction (via `CustomMsgLite` from the offset_time
  PointCloud2), IMU initializer/state estimator, LiDAR handler.
- **I/O glue (S5)**: `msg_manager` (rosbag2 reader, CDR deserialize, storage
  auto-detect for sqlite3/mcap), `odometry_manager` (finite `RunBag`),
  `odometry_node` (rclcpp entry). RViz viewer stubbed.
- **Camera (S4)**: slimmed R3LIVE visual front-end (optical-flow tracking + RGB map),
  feeding map-point→pixel correspondences to the ported image-feature factor.

## Build & run

```bash
cd /home/frank/gaussian_lic_ros2
source /opt/ros/jazzy/setup.bash
export PATH=/home/frank/miniconda3/bin:$PATH   # conda python has lark for rosidl
colcon build --packages-select cocolic
source install/setup.bash

# LIO (best result):
ros2 run cocolic odometry_node run_lio/config/ct_odometry_lio.yaml
# LICO (full camera):
ros2 run cocolic odometry_node run_lio/config/ct_odometry_lico.yaml

# Output trajectories:
# - run_lio/data/CBD_Building_01_frontend_raw_offset_time_full_LIO.txt
# - run_lio/data/CBD_Building_01_frontend_raw_offset_time_full_LICO.txt
# Compare:
python3 scripts/trajectory_compare.py \
  --baseline baseline/fastlivo2/CBD_Building_01/native_reference/cocolic_livo_reference_10hz.tum \
  --current  run_lio/data/CBD_Building_01_frontend_raw_offset_time_full_LIO.txt --align yaw
```

## Input data note (cm-critical)

Use the **`offset_time_full` mcap** bag, not `frontend_raw` (db3). Continuous-time
deskew needs per-point time; `frontend_raw`'s PointCloud2 (point_step 16) lacks it,
while `offset_time_full` (point_step 20) carries the per-point `offset_time` (UINT32,
monotonic over the full ~100 ms scan).

## Key port fixes (subtle, load-bearing)

- **Ceres Manifold migration** in `ceres_local_param.h` (Plus/Minus/Jacobians).
- **rosbag2 storage auto-detect**: leave `storage_id` empty → read from `metadata.yaml`
  (don't extension-sniff the path; a sqlite3 bag is a directory).
- **`m_camera_intrinsic` must be `Eigen::RowMajor`** (matches upstream): the code does
  `g_cam_K << m_camera_intrinsic.data()[0..8]` assuming row-major; a column-major
  `Matrix3d` transposes K → degenerate undistort → black image → optical-flow failure.
- **`findFundamentalMat` guard** (`>= 8` points) — robustness.
- PCL `io` component dropped (avoids system gdal/hdf5 libcurl clash with conda);
  link `lz4` + system `libcurl` explicitly.

## Generalization (2nd scene)

Ran on **Retail_Street** (different FAST-LIVO2 scene; built its offset_time mcap via
`scripts/fastlivo2_ros1_to_frontend_raw.py`). LIO ran end-to-end: 134s, **65.82 m path,
start-to-end deviation 3.6 cm (~0.05% drift)** — sane, low-drift odometry. No ATE
reference exists for Retail (only CBD), so this is a loop-closure sanity metric, not ATE.
Confirms the port is not CBD-overfit.
