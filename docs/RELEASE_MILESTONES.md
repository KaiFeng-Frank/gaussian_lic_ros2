# Release Milestones

## v0.1.0 - Baseline & Infrastructure

Goal: make strict reproduction measurable before the native algorithm port grows.

- Run and archive upstream Gaussian-LIC/Gaussian-LIC2 baseline artifacts for the selected FAST-LIVO2 sequence.
- Add rosbag conversion tooling for ROS1 `.bag` to rosbag2 `.mcap`.
- Keep Jazzy as the primary target and add Humble build-only CI scaffolding.
- Keep `gaussian_lic_offline` available for non-launch rosbag2 artifact extraction.
- Freeze `/gaussian_lic/status` and `metrics.json` schemas for later CI comparison.
- Add performance regression gates for tracking FPS, mapping FPS, and iteration time.
- Add a trajectory regression gate for timestamp-associated TUM trajectory drift.
- Add a point-cloud regression gate for ASCII PLY map drift.
- Add a baseline manifest generator for archived ROS1 artifact fingerprinting.
- Add a combined reproduction report that aggregates manifest, metrics, trajectory, and map gates.

Release artifacts:

```text
baseline/fastlivo2/<sequence>/trajectory.tum
baseline/fastlivo2/<sequence>/point_cloud.ply
baseline/fastlivo2/<sequence>/renders/
baseline/fastlivo2/<sequence>/metrics.json
baseline/fastlivo2/<sequence>/run.log
baseline/fastlivo2/<sequence>/baseline_manifest.json
```

## v0.2.0 - Gaussian-LIC2 ROS2 Native Frontend

Goal: the Gaussian-LIC2 frontend/tracking path runs natively in ROS2 without depending on a ROS1 runtime.

- Refactor timestamp math to `int64_t` nanoseconds before ROS2 API replacement.
- Keep `gaussian_lic_frontend/lic2_contract_adapter` as the raw-topic to mapper-contract boundary while the full odometry frontend is being ported.
- Port PointCloud2/Livox LiDAR input, IMU input, camera input, continuous-time trajectory, and odometry output against the released Gaussian-LIC2 upstream surface; if upstream still delegates odometry to legacy code, keep that boundary explicit and native in ROS2.
- Publish `/gaussian_lic/frontend/odometry`, `/gaussian_lic/frontend/path`, TF, and the mapper input topics:
  `/image_for_gs`, `/depth_for_gs`, `/pose_for_gs`, `/points_for_gs`.
- Compare trajectory against the ROS1 baseline with `scripts/trajectory_compare.py` and fail CI when drift exceeds the accepted threshold.
- Compare saved PLY maps against the ROS1 baseline with `scripts/pointcloud_compare.py` once the native map backend is active.

## v0.3.0 - Gaussian Mapping Full Backend

Goal: replace the debug mapping slice with the real Gaussian-LIC backend.

- Port CUDA rasterizer, simple-knn, fused-ssim, optimizer, densification, pruning, and upstream-compatible PLY persistence.
- Change the default render path to `render_mode:=rasterizer`.
- Keep `render_mode:=debug_cpu` only as a temporary diagnostic path and remove it after v0.3 stabilization.
- Keep TensorRT/depth completion optional; `depth_completion:=false` must run without TensorRT installed.

## v0.4.0 - Strict FAST-LIVO2 Reproduction

Goal: one-command paper-level reproduction on the selected FAST-LIVO2 sequence.

- Run full Gaussian-LIC2 frontend plus Gaussian mapping end to end from rosbag2.
- Generate `metrics.json`, trajectory, Gaussian PLY, rendered images, and comparison report.
- Compare ROS2 outputs against archived ROS1 baseline artifacts with `scripts/reproduction_report.py`.
- Publish release artifacts and expected screenshots/video for README and GitHub release notes.
