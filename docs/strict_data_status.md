# Strict Data Input Audit

Generated: `2026-05-08T00:33:38+00:00`
Data root: `/home/frank/data`
Disk free: `117.88 GiB` (minimum requested `100.00 GiB`) - PASS
Strict evidence currently materialized: `5/10`
Raw/frontend inputs local: `PASS`
ROS1 baseline artifacts local: `INCOMPLETE`
Native reference trajectories local: `INCOMPLETE`

| Profile | Dataset | Raw | Frontend | Baseline | Current | Reference | Status | Missing Inputs |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `fastlivo` | FAST-LIVO | PASS | PASS | MISS | PASS | MISS | MISSING | native_reference_trajectory, ros1_baseline_artifacts |
| `fastlivo2` | FAST-LIVO2 | PASS | PASS | MISS | PASS | MISS | MISSING | native_reference_trajectory, ros1_baseline_artifacts |
| `m2dgr` | M2DGR | PASS | PASS | MISS | PASS | PASS | MISSING | ros1_baseline_artifacts |
| `mcd` | MCD | PASS | PASS | MISS | PASS | PASS | MISSING | ros1_baseline_artifacts |
| `r3live` | R3LIVE | PASS | PASS | MISS | PASS | MISS | MISSING | native_reference_trajectory, ros1_baseline_artifacts |

## Sequence Detail

### fastlivo
- `LiDAR_Degenerate`: MISSING (raw=PASS, frontend=PASS, baseline=MISS, current=PASS, reference=MISS); missing: ros1_baseline_artifacts, native_reference_trajectory
  - raw_ros1_bag: `/home/frank/data/fast_livo_mcap/FAST-LIVO/LiDAR_Degenerate/LiDAR_Degenerate_0.mcap`
  - frontend_raw_rosbag2: `/home/frank/data/fast_livo_mcap/LiDAR_Degenerate_frontend_raw`
  - ros2_current_artifacts: `/home/frank/gaussian_lic_ros2/results/fastlivo/LiDAR_Degenerate_native_tracking_scan_order_60s`
- `Visual_Challenge`: MISSING (raw=PASS, frontend=PASS, baseline=PASS, current=PASS, reference=MISS); missing: native_reference_trajectory
  - raw_ros1_bag: `/home/frank/data/fast_livo_mcap/FAST-LIVO/Visual_Challenge/Visual_Challenge_0.mcap`
  - frontend_raw_rosbag2: `/home/frank/data/fast_livo_mcap/Visual_Challenge_frontend_raw`
  - ros1_baseline_artifacts: `/home/frank/gaussian_lic_ros2/baseline/fastlivo/Visual_Challenge`
  - ros2_current_artifacts: `/home/frank/gaussian_lic_ros2/results/fastlivo/Visual_Challenge_strict_current`
- `hku1`: MISSING (raw=PASS, frontend=PASS, baseline=PASS, current=PASS, reference=MISS); missing: native_reference_trajectory
  - raw_ros1_bag: `/home/frank/data/fast_livo_mcap/FAST-LIVO/hku1/hku1_0.mcap`
  - frontend_raw_rosbag2: `/home/frank/data/fast_livo_mcap/hku1_frontend_raw`
  - ros1_baseline_artifacts: `/home/frank/gaussian_lic_ros2/baseline/fastlivo/hku1`
  - ros2_current_artifacts: `/home/frank/gaussian_lic_ros2/results/fastlivo/hku1_strict_current`
- `hku2`: MISSING (raw=PASS, frontend=PASS, baseline=PASS, current=PASS, reference=MISS); missing: native_reference_trajectory
  - raw_ros1_bag: `/home/frank/data/fast_livo_mcap/FAST-LIVO/hku2/hku2_0.mcap`
  - frontend_raw_rosbag2: `/home/frank/data/fast_livo_mcap/hku2_frontend_raw`
  - ros1_baseline_artifacts: `/home/frank/gaussian_lic_ros2/baseline/fastlivo/hku2`
  - ros2_current_artifacts: `/home/frank/gaussian_lic_ros2/results/fastlivo/hku2_strict_current`

### fastlivo2
- `CBD_Building_01`: MISSING (raw=PASS, frontend=PASS, baseline=PASS, current=PASS, reference=MISS); missing: native_reference_trajectory
  - raw_ros1_bag: `/home/frank/data/fast_livo/CBD_Building_01.bag`
  - frontend_raw_rosbag2: `/home/frank/data/fast_livo/CBD_Building_01_frontend_raw`
  - ros1_baseline_artifacts: `/home/frank/gaussian_lic_ros2/baseline/fastlivo2/CBD_Building_01`
  - ros2_current_artifacts: `/home/frank/gaussian_lic_ros2/results/fastlivo2/CBD_Building_01_current_round_no_opacity_prune_probe`
- `Retail_Street`: MISSING (raw=PASS, frontend=PASS, baseline=MISS, current=PASS, reference=MISS); missing: ros1_baseline_artifacts, native_reference_trajectory
  - raw_ros1_bag: `/home/frank/data/fast_livo/Retail_Street.bag`
  - frontend_raw_rosbag2: `/home/frank/data/fast_livo/Retail_Street_frontend_raw`
  - ros2_current_artifacts: `/home/frank/gaussian_lic_ros2/results/fastlivo2/Retail_Street_native_tracking_scan_order_60s`

### m2dgr
- `room_01`: MISSING (raw=PASS, frontend=PASS, baseline=MISS, current=PASS, reference=PASS); missing: ros1_baseline_artifacts
  - raw_ros1_bag: `/home/frank/data/m2dgr/room_01/room_01.bag`
  - frontend_raw_rosbag2: `/home/frank/data/m2dgr/room_01_frontend_raw`
  - ros2_current_artifacts: `/home/frank/gaussian_lic_ros2/results/m2dgr/room_01_tracking_sweep_20s`
  - native_reference_trajectory: `/home/frank/data/m2dgr/room_01/room_01_gt.tum`
- `room_02`: MISSING (raw=PASS, frontend=PASS, baseline=MISS, current=PASS, reference=PASS); missing: ros1_baseline_artifacts
  - raw_ros1_bag: `/home/frank/data/m2dgr/room_02/room_02.bag`
  - frontend_raw_rosbag2: `/home/frank/data/m2dgr/room_02_frontend_raw`
  - ros2_current_artifacts: `/home/frank/gaussian_lic_ros2/results/m2dgr/room_02_native_tracking_scan_order_60s`
  - native_reference_trajectory: `/home/frank/data/m2dgr/room_02/room_02_gt.tum`
- `room_03`: MISSING (raw=PASS, frontend=PASS, baseline=MISS, current=PASS, reference=PASS); missing: ros1_baseline_artifacts
  - raw_ros1_bag: `/home/frank/data/m2dgr/room_03/room_03.bag`
  - frontend_raw_rosbag2: `/home/frank/data/m2dgr/room_03_frontend_raw`
  - ros2_current_artifacts: `/home/frank/gaussian_lic_ros2/results/m2dgr/room_03_native_tracking_scan_order_60s`
  - native_reference_trajectory: `/home/frank/data/m2dgr/room_03/room_03_gt.tum`

### mcd
- `ntu_day_01`: MISSING (raw=PASS, frontend=PASS, baseline=MISS, current=PASS, reference=PASS); missing: ros1_baseline_artifacts
  - raw_ros1_bag: `/home/frank/data/mcd/ntu_day_01/ntu_day_01_d435i.bag`
  - frontend_raw_rosbag2: `/home/frank/data/mcd/ntu_day_01_frontend_raw`
  - ros2_current_artifacts: `/home/frank/gaussian_lic_ros2/results/mcd/ntu_day_01_native_tracking_scan_order_60s`
  - native_reference_trajectory: `/home/frank/data/mcd/ntu_day_01/groundtruth/pose_inW.tum`
  - raw_component_camera_d435i: `/home/frank/data/mcd/ntu_day_01/ntu_day_01_d435i.bag`
  - raw_component_lidar_mid70: `/home/frank/data/mcd/ntu_day_01/ntu_day_01_mid70.bag`
  - raw_component_imu_vn100_or_vn200: `/home/frank/data/mcd/ntu_day_01/ntu_day_01_vn100.bag`

### r3live
- `hku_park_00`: MISSING (raw=PASS, frontend=PASS, baseline=MISS, current=PASS, reference=MISS); missing: ros1_baseline_artifacts, native_reference_trajectory
  - raw_ros1_bag: `/home/frank/data/r3live/hku_park_00.bag`
  - frontend_raw_rosbag2: `/home/frank/data/r3live/hku_park_00_frontend_raw`
  - ros2_current_artifacts: `/home/frank/gaussian_lic_ros2/results/r3live/hku_park_00_native_tracking_scan_order_60s`

## Largest Non-Matrix Artifact Candidates

- `/home/frank/gaussian_lic_ros2/results/smoke_torch_optimization/ros2_output_bag` - 0.00 GiB
- `/home/frank/gaussian_lic_ros2/results/smoke_torch_pruning/ros2_output_bag` - 0.00 GiB
- `/home/frank/gaussian_lic_ros2/results/m2dgr/room_01_tracking_sweep_20s` - 0.00 GiB
- `/home/frank/gaussian_lic_ros2/results/fastlivo2/CBD_Building_01_native_tracking_balanced_sampling_120s` - 0.00 GiB
- `/home/frank/gaussian_lic_ros2/results/fastlivo2/CBD_Building_01_native_tracking_reference_tum_sync50ms_120s` - 0.00 GiB
- `/home/frank/gaussian_lic_ros2/results/m2dgr/room_01_tracking_sweep_rebase_60s` - 0.00 GiB
- `/home/frank/gaussian_lic_ros2/results/fastlivo/Visual_Challenge_upstream_baseline_failure` - 0.00 GiB
- `/home/frank/gaussian_lic_ros2/results/fastlivo2/Bright_Screen_Wall_native_tracking_visual_sparse_depth_extrinsic_windowed_visual_60s` - 0.00 GiB
- `/home/frank/gaussian_lic_ros2/results/m2dgr/room_01_tracking_sweep_rebase_gated_60s` - 0.00 GiB
- `/home/frank/gaussian_lic_ros2/results/fastlivo2/Bright_Screen_Wall_native_tracking_visual_scan_order_deskew_source_id_queue_gate_60s` - 0.00 GiB
- `/home/frank/gaussian_lic_ros2/results/fastlivo2/Bright_Screen_Wall_native_tracking_visual_depth_dt1s_valid_sampling_60s` - 0.00 GiB
- `/home/frank/gaussian_lic_ros2/results/fastlivo2/Bright_Screen_Wall_native_tracking_visual_valid_depth_sampling_default_60s` - 0.00 GiB

## Source URLs

- `fastlivo`: https://github.com/hku-mars/FAST-LIVO, https://connecthkuhk-my.sharepoint.com/personal/zhengcr_connect_hku_hk/_layouts/15/onedrive.aspx?id=%2Fpersonal%2Fzhengcr%5Fconnect%5Fhku%5Fhk%2FDocuments%2FFAST%2DLIVO%2DDatasets&ga=1, https://huggingface.co/datasets/DapengFeng/MCAP/tree/main/FAST-LIVO
- `fastlivo2`: https://github.com/hku-mars/FAST-LIVO2, https://drive.google.com/drive/folders/1bf5LQ8iSxw-fD8BObZmouw7lRxNacfrA?usp=drive_link
- `m2dgr`: https://github.com/SJTU-ViSYS/M2DGR
- `mcd`: https://mcdviral.github.io/Download.html
- `r3live`: https://github.com/ziv-lin/r3live_dataset, https://github.com/hku-mars/r3live
