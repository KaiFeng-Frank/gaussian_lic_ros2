# Changelog

## Unreleased

- Pivoted the upstream porting plan to the now-public Gaussian-LIC2 code path in `APRIL-ZJU/Gaussian-LIC`; Coco-LIC is now documented as an optional legacy reference.
- Added `gaussian_lic_frontend/lic2_contract_adapter` to route raw ROS2 camera/LiDAR/IMU/pose topics into the Gaussian mapper contract.
- Added keyframe-gated Torch Gaussian initialization and incremental foreground insertion for the ROS2 mapping node.
- Added optional skybox tensor seeding, Gaussian map republishing after extension, and mapping latency/mean iteration status fields.
- Added upstream Gaussian-LIC backend parameter coverage for image size, depth completion, optimizer, loss, exposure, Gaussian extension, and skybox control.
- Implemented the dependency-gated ROS1 `.bag` to rosbag2 converter backend using `rosbags`.
- Added `gaussian_lic_bag_check` to validate rosbag2 mapper-contract topics before replay.
- Extended `gaussian_lic_bag_check` to inspect ROS1 `.bag` metadata before conversion.
- Added image-projection color fallback for PointCloud2 mapper inputs that do not carry RGB fields.
- Added synthetic smoke coverage for image-projected point coloring from uncolored PointCloud2 input.
- Added `require_depth_topic` so the mapper can run with sparse point-projected depth when depth images are unavailable.
- Added a `mapper_minimal` bag contract mode for point/pose/image replay bags.
- Added `smoke_test.sh --minimal-inputs` for replaying point/pose/image-only mapper bags.
- Preserved PointCloud2 RGB fields in `gaussian_lic_offline` debug PLY artifacts.
- Added offline artifact metrics for topic rates, trajectory path length, point-cloud bounds, and color coverage.
- Added direct ROS1 `.bag` support to `gaussian_lic_offline` for dependency-gated baseline artifact extraction.
- Added `scripts/trajectory_compare.py` for timestamp-associated TUM trajectory drift gates.
- Added `scripts/pointcloud_compare.py` for ASCII PLY map drift gates.
- Added `scripts/baseline_manifest.py` for ROS1 baseline artifact validation and fingerprinting.
- Added `scripts/reproduction_report.py` to aggregate baseline manifest, metrics, trajectory, and PLY drift gates.
- Added a Python-only GitHub Actions artifact-gates job using `scripts/verify_artifact_gates.sh`.
- Uploaded artifact-gate JSON/Markdown reports from CI as `artifact-gate-reports`.
- Added a Jazzy CI synthetic rosbag2 replay smoke step with uploaded smoke logs.
- Made synthetic bag, smoke, and workspace verification helpers respect `ROS_DISTRO` with a Jazzy default.
- Added `scripts/build_ros2.sh` and kept `scripts/build_jazzy.sh` as a compatibility wrapper.
- Added `scripts/ci_replay_smoke.sh` so CI and local generated-bag replay smoke share one entrypoint.
- Added CI replay smoke artifact export for bag contracts and offline extraction outputs.
- Uploaded per-distro CI `log/` directories for colcon and smoke-test diagnostics.
- Added Markdown job summaries for artifact gates and CI replay smoke reports.

## v0.1.0-m1-infra - 2026-05-03

M1 infrastructure checkpoint for the Gaussian-LIC ROS2 port.

Completed:

- Added release-driven roadmap for M1-M4.
- Added `/gaussian_lic/status` schema documentation and extended `MappingStatus.msg`.
- Added canonical `render_mode` parameter with deprecated `rendered_image_mode` compatibility.
- Added ROS2 native mapper smoke path for synthetic live input and rosbag2 replay.
- Added dataset profile YAMLs for FAST-LIVO, FAST-LIVO2, M2DGR, MCD, and R3LIVE.
- Added `gaussian_lic_offline` rosbag2 artifact extraction CLI.
- Added `scripts/perf_regression.py` for metrics regression checks.
- Added `scripts/convert_ros1_bag_to_rosbag2.py` as the dependency-gated ROS1 bag conversion entrypoint.
- Added Jazzy/Humble build-only GitHub Actions skeleton.
- Added calibration schema, common errors, release milestones, and status schema docs.

Validated locally:

```bash
./scripts/build_jazzy.sh
./scripts/verify_workspace.sh --skip-build --full-profiles
GAUSSIAN_LIC_ENABLE_TORCH=ON ./scripts/build_jazzy.sh --packages-select gaussian_lic_mapping
ros2 run gaussian_lic_mapping torch_backend_probe
./scripts/smoke_test.sh --bag bags/synthetic_gs_demo --torch --tf
./scripts/smoke_test.sh --bag bags/synthetic_gs_demo --composition --tf
```

Known limitations:

- Native Gaussian-LIC2 frontend/tracking is not ported yet.
- Full Gaussian rasterizer/optimizer/densification/pruning are not ported yet.
- `/gaussian_lic/rendered_image` currently uses `render_mode:=debug_cpu`, not the paper rasterizer.
- `/home/frank/data/fast_livo/` is empty on the current machine; strict FAST-LIVO2 baseline reproduction is blocked on data.
- TensorRT is not installed locally; depth completion remains optional.

Next concrete action:

```bash
docker pull ros:noetic-ros-base-focal
```

Then run the ROS1 upstream baseline on FAST-LIVO2 and archive outputs under:

```text
baseline/fastlivo2/<sequence>/
```
