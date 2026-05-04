# Changelog

## Unreleased

- Added keyframe-gated Torch Gaussian initialization and incremental foreground insertion for the ROS2 mapping node.
- Added optional skybox tensor seeding, Gaussian map republishing after extension, and mapping latency/mean iteration status fields.
- Added upstream Gaussian-LIC backend parameter coverage for image size, depth completion, optimizer, loss, exposure, Gaussian extension, and skybox control.

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

- Native Coco-LIC tracking is not ported yet.
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
