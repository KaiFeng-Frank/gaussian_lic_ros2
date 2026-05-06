# Strict Data Input Audit

Generated: `2026-05-06T12:27:24+00:00`
Data root: `/home/frank/data`
Disk free: `190.26 GiB` (minimum requested `100.00 GiB`) - PASS
Strict evidence currently materialized: `2/7`

| Profile | Dataset | Status | Missing Inputs |
| --- | --- | --- | --- |
| `fastlivo` | FAST-LIVO | MISSING | frontend_raw_rosbag2, native_reference_trajectory, raw_ros1_bag, ros1_baseline_artifacts, ros2_current_artifacts |
| `fastlivo2` | FAST-LIVO2 | MISSING | native_reference_trajectory, ros1_baseline_artifacts |
| `m2dgr` | M2DGR | MISSING | frontend_raw_rosbag2, raw_ros1_bag, ros1_baseline_artifacts, ros2_current_artifacts |
| `mcd` | MCD | MISSING | frontend_raw_rosbag2, raw_component_camera_d435i, ros1_baseline_artifacts, ros2_current_artifacts |
| `r3live` | R3LIVE | MISSING | native_reference_trajectory, ros1_baseline_artifacts |

## Sequence Detail

### fastlivo
- `LiDAR_Degenerate`: MISSING; missing: raw_ros1_bag, frontend_raw_rosbag2, ros1_baseline_artifacts, ros2_current_artifacts, native_reference_trajectory
- `Visual_Challenge`: MISSING; missing: raw_ros1_bag, frontend_raw_rosbag2, ros1_baseline_artifacts, ros2_current_artifacts, native_reference_trajectory
- `hku1`: MISSING; missing: ros1_baseline_artifacts, native_reference_trajectory
  - raw_ros1_bag: `/home/frank/data/fast_livo_mcap/FAST-LIVO/hku1/hku1_0.mcap`
  - frontend_raw_rosbag2: `/home/frank/data/fast_livo_mcap/hku1_frontend_raw`
  - ros2_current_artifacts: `/home/frank/gaussian_lic_ros2/results/fastlivo/hku1_native_tracking_scan_order_60s`
- `hku2`: MISSING; missing: ros1_baseline_artifacts, native_reference_trajectory
  - raw_ros1_bag: `/home/frank/data/fast_livo_mcap/FAST-LIVO/hku2/hku2_0.mcap`
  - frontend_raw_rosbag2: `/home/frank/data/fast_livo_mcap/hku2_frontend_raw`
  - ros2_current_artifacts: `/home/frank/gaussian_lic_ros2/results/fastlivo/hku2_native_tracking_scan_order_60s`

### fastlivo2
- `CBD_Building_01`: MISSING; missing: native_reference_trajectory
  - raw_ros1_bag: `/home/frank/data/fast_livo/CBD_Building_01.bag`
  - frontend_raw_rosbag2: `/home/frank/data/fast_livo/CBD_Building_01_frontend_raw`
  - ros1_baseline_artifacts: `/home/frank/gaussian_lic_ros2/baseline/fastlivo2/CBD_Building_01`
  - ros2_current_artifacts: `/home/frank/gaussian_lic_ros2/results/fastlivo2/CBD_Building_01_current_rasterizer_opt_probe`
- `Retail_Street`: MISSING; missing: ros1_baseline_artifacts, native_reference_trajectory
  - raw_ros1_bag: `/home/frank/data/fast_livo/Retail_Street.bag`
  - frontend_raw_rosbag2: `/home/frank/data/fast_livo/Retail_Street_frontend_raw`
  - ros2_current_artifacts: `/home/frank/gaussian_lic_ros2/results/fastlivo2/Retail_Street_native_tracking_scan_order_60s`

### m2dgr
- `room_01`: MISSING; missing: raw_ros1_bag, frontend_raw_rosbag2, ros1_baseline_artifacts, ros2_current_artifacts
  - native_reference_trajectory: `/home/frank/data/m2dgr/room_01/room_01_gt.tum`
- `room_02`: MISSING; missing: raw_ros1_bag, frontend_raw_rosbag2, ros1_baseline_artifacts, ros2_current_artifacts
  - native_reference_trajectory: `/home/frank/data/m2dgr/room_02/room_02_gt.tum`
- `room_03`: MISSING; missing: raw_ros1_bag, frontend_raw_rosbag2, ros1_baseline_artifacts, ros2_current_artifacts
  - native_reference_trajectory: `/home/frank/data/m2dgr/room_03/room_03_gt.tum`

### mcd
- `ntu_day_01`: MISSING; missing: raw_component_camera_d435i, frontend_raw_rosbag2, ros1_baseline_artifacts, ros2_current_artifacts
  - raw_ros1_bag: `/home/frank/data/mcd/ntu_day_01/ntu_day_01_mid70.bag.bz2`
  - native_reference_trajectory: `/home/frank/data/mcd/ntu_day_01/groundtruth/pose_inW.tum`
  - raw_component_camera_d435i: `missing`
  - raw_component_lidar_mid70: `/home/frank/data/mcd/ntu_day_01/ntu_day_01_mid70.bag.bz2`
  - raw_component_imu_vn100_or_vn200: `/home/frank/data/mcd/ntu_day_01/ntu_day_01_vn100.bag.bz2`

### r3live
- `hku_park_00`: MISSING; missing: ros1_baseline_artifacts, native_reference_trajectory
  - raw_ros1_bag: `/home/frank/data/r3live/hku_park_00.bag`
  - frontend_raw_rosbag2: `/home/frank/data/r3live/hku_park_00_frontend_raw`
  - ros2_current_artifacts: `/home/frank/gaussian_lic_ros2/results/r3live/hku_park_00_native_tracking_scan_order_60s`

## Largest Non-Matrix Artifact Candidates

- `/home/frank/gaussian_lic_ros2/results/fastlivo2/Bright_Screen_Wall_current_extrinsic_color_torch` - 0.44 GiB
- `/home/frank/gaussian_lic_ros2/results/fastlivo2/Bright_Screen_Wall_current_extrinsic_torch` - 0.31 GiB
- `/home/frank/gaussian_lic_ros2/results/fastlivo2/CBD_Building_01_current_rasterizer_opt_probe` - 0.28 GiB
- `/home/frank/gaussian_lic_ros2/results/fastlivo2/CBD_Building_01_current_rasterizer_probe` - 0.28 GiB
- `/home/frank/gaussian_lic_ros2/results/fastlivo2/Bright_Screen_Wall_current_torch_opt` - 0.25 GiB
- `/home/frank/gaussian_lic_ros2/results/fastlivo2/Bright_Screen_Wall_current_extrinsic_torch_intensity` - 0.24 GiB
- `/home/frank/gaussian_lic_ros2/results/fastlivo2/Bright_Screen_Wall_current_extrinsic` - 0.18 GiB
- `/home/frank/gaussian_lic_ros2/results/fastlivo2/Bright_Screen_Wall_current_extrinsic_fullseq` - 0.17 GiB
- `/home/frank/gaussian_lic_ros2/results/fastlivo2/Bright_Screen_Wall_current` - 0.16 GiB
- `/home/frank/gaussian_lic_ros2/results/fastlivo2/Bright_Screen_Wall_current_imu_pose` - 0.15 GiB
- `/home/frank/gaussian_lic_ros2/results/smoke_torch_optimization/ros2_output_bag` - 0.00 GiB
- `/home/frank/gaussian_lic_ros2/results/smoke_torch_pruning/ros2_output_bag` - 0.00 GiB

## Source URLs

- `fastlivo`: https://github.com/hku-mars/FAST-LIVO, https://connecthkuhk-my.sharepoint.com/personal/zhengcr_connect_hku_hk/_layouts/15/onedrive.aspx?id=%2Fpersonal%2Fzhengcr%5Fconnect%5Fhku%5Fhk%2FDocuments%2FFAST%2DLIVO%2DDatasets&ga=1, https://huggingface.co/datasets/DapengFeng/MCAP/tree/main/FAST-LIVO
- `fastlivo2`: https://github.com/hku-mars/FAST-LIVO2, https://drive.google.com/drive/folders/1bf5LQ8iSxw-fD8BObZmouw7lRxNacfrA?usp=drive_link
- `m2dgr`: https://github.com/SJTU-ViSYS/M2DGR
- `mcd`: https://mcdviral.github.io/Download.html
- `r3live`: https://github.com/ziv-lin/r3live_dataset, https://github.com/hku-mars/r3live
