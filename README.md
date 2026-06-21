# Gaussian-LIC ROS2

Native ROS2 Jazzy engineering port of
[Gaussian-LIC / Gaussian-LIC2](https://github.com/APRIL-ZJU/Gaussian-LIC) for
LiDAR-Inertial-Camera Gaussian Splatting SLAM.

This repository is not a ROS1 bridge wrapper. It keeps the ROS2 middleware,
tracking frontend, mapper contract, CUDA/Torch Gaussian mapping path, offline
artifact tooling, and validation scripts in one workspace so each part can be
ported and tested independently.

## Current Status

The public tree is an executable ROS2 porting checkpoint, not a packaged
one-command dataset release. Source, configs, scripts, CI checks, and validation
reports are versioned in git. Large datasets, generated rosbag2 artifacts,
render outputs, point clouds, TensorRT engines, and long-form experiment bundles
must be supplied separately.

Highlights:

- ROS2 Jazzy workspace with native message, launch, frontend, tracking, mapping,
  and offline tooling packages.
- Mapper input contract for `/points_for_gs`, `/pose_for_gs`,
  `/image_for_gs`, `/camera_info_for_gs`, `/depth_for_gs`, and `/imu_for_gs`.
- Optional CUDA/libtorch Gaussian mapping path with rendered feedback for
  closed-loop tracking experiments.
- Continuous-time LIC tracking surface with LiDAR, IMU, visual, and
  render-photometric factor plumbing.
- Dataset profiles and conversion scripts for FAST-LIVO, FAST-LIVO2, M2DGR,
  MCD, and R3LIVE style inputs.
- Validation reports kept under `docs/` for strict parity, paper-completion
  audit, GL2 closed-loop evidence, and known upstream/reference limitations.

## Quick Start

```bash
git clone https://github.com/KaiFeng-Frank/gaussian_lic_ros2.git
cd gaussian_lic_ros2

source /opt/ros/jazzy/setup.bash
./scripts/build_ros2.sh
source install/setup.bash
./scripts/smoke_test.sh --tf
```

Run the native tracking probe suite:

```bash
colcon test --packages-select gaussian_lic_tracking --event-handlers console_direct+
colcon test-result --verbose
```

For real datasets, place bags and generated artifacts outside git and point the
provided scripts/configs at those local paths.

## Evidence Snapshot

The validation method is evidence-driven: compare against valid upstream
references or ground truth where available, and explicitly mark datasets whose
archived upstream references are defective.

| Evidence surface | Public report |
|---|---|
| Strict multi-dataset parity matrix | [`docs/strict_parity_matrix_report.md`](docs/strict_parity_matrix_report.md) |
| Paper-completion audit | [`docs/paper_completion_report.md`](docs/paper_completion_report.md) |
| GL2 closed-loop tracking and 3DGS mapping results | [`docs/GL2_RESULTS.md`](docs/GL2_RESULTS.md) |
| Input data audit contract | [`docs/strict_data_status.md`](docs/strict_data_status.md) |
| Historical rejected diagnostics | [`docs/HISTORICAL_DIAGNOSTICS.md`](docs/HISTORICAL_DIAGNOSTICS.md) |

Current archived reports record a passing strict matrix for FAST-LIVO,
FAST-LIVO2, M2DGR, MCD, and R3LIVE coverage. The strongest closed-loop GL2
result is on FAST-LIVO2 `CBD_Building_01`: cm-class baseline tracking, full
frame live mapper feedback, and measured PSNR/ATE ablations documented in
`docs/GL2_RESULTS.md`.

## Repository Layout

```text
src/                         ROS2 packages for messages, tracking, mapping, and tools
config/                      Dataset and runtime configuration examples
scripts/                     Build, replay, conversion, validation, and audit utilities
docs/                        Public validation reports and engineering notes
docker/                      Jazzy container environment
external/                    Upstream source inventory
```

## Reproducibility Boundary

This repository intentionally does not commit:

- raw datasets or rosbag2/MCAP conversions
- generated trajectories, rendered images, point clouds, or maps
- local TensorRT engines or GPU-specific binary artifacts
- large upstream baseline bundles

The public contract is therefore source plus validation metadata. To reproduce a
dataset-level report, obtain the relevant dataset/baseline artifacts, configure
the local paths, and run the scripts referenced in the matching `docs/` report.

## Platform

Primary development target:

```text
Ubuntu 24.04
ROS2 Jazzy
CUDA 12.x
libtorch 2.x
```

Other ROS2 distributions are not part of the required CI or public support
surface for this port.

## Scope and Limitations

- This is an unofficial research/engineering port, not an APRIL-ZJU release.
- Dataset-level claims depend on valid references; broken zero-trajectory
  upstream references are treated as evidence defects, not silently accepted.
- Closed-loop render-photometric coupling has been measured as stable and useful
  under degraded tracking settings, but larger gains need loop/revisit datasets
  rather than more scalar tuning on non-revisit sequences.

## Detailed Notes

The previous long-form README has been retained as
[`docs/DETAILED_STATUS_ARCHIVE.md`](docs/DETAILED_STATUS_ARCHIVE.md) for audit
history. New readers should start with this README and the validation reports
linked above.
