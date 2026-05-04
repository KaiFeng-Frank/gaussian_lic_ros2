# Baseline Data

This repository treats the FAST-LIVO2 baseline as an executable gate, not a manual note.

The default strict sequence is `CBD_Building_01`, matching the Gaussian-LIC/Gaussian-LIC2 quick start. The expected local raw-data path is:

```bash
/home/frank/data/fast_livo/CBD_Building_01.bag
```

Discover and fetch that sequence from the official FAST-LIVO2 Google Drive mirror:

```bash
./scripts/fetch_fastlivo2_sequence.py \
  --sequence CBD_Building_01 \
  --output-dir /home/frank/data/fast_livo
```

Check whether the raw data, archived ROS1 baseline, and ROS2 current artifacts are ready:

```bash
./scripts/baseline_readiness.py \
  --dataset-root /home/frank/data/fast_livo \
  --baseline-dir baseline/fastlivo2/CBD_Building_01 \
  --current-results-dir results/fastlivo2/current \
  --sequence CBD_Building_01 \
  --probe-sources \
  --markdown baseline_readiness.md
```

Collect ROS2 current artifacts from a rosbag2 replay after the mapper is built:

```bash
./scripts/collect_current_results.sh \
  --bag /home/frank/data/fast_livo/<sequence>_frontend_raw \
  --frontend-adapter \
  --imu-pose-fallback \
  --sync-image-to-pointcloud \
  --fastlivo2-camera-lidar-transform \
  --optional-depth \
  --sensor-qos reliable \
  --play-rate 0.2 \
  --post-play-settle 30 \
  --record-sec 650 \
  --output results/fastlivo2/current
```

For the current Torch Gaussian preview path:

```bash
./scripts/collect_current_results.sh \
  --bag /home/frank/data/fast_livo/<sequence>_frontend_raw \
  --frontend-adapter \
  --imu-pose-fallback \
  --sync-image-to-pointcloud \
  --fastlivo2-camera-lidar-transform \
  --optional-depth \
  --sensor-qos reliable \
  --play-rate 0.2 \
  --post-play-settle 30 \
  --record-sec 650 \
  --torch \
  --torch-device cuda \
  --render-mode rasterizer \
  --output results/fastlivo2/current
```

The output directory is intentionally shaped for `scripts/reproduction_report.py` and `scripts/baseline_readiness.py`: `trajectory.tum`, `point_cloud.ply`, `metrics.json`, and strict render-pair images under `renders/` are produced when `/gaussian_lic/rendered_image` is available.

Convert a downloaded official FAST-LIVO2 ROS1 bag into the ROS2 raw-sensor frontend contract:

```bash
PYTHONPATH=/home/frank/.cache/gaussian_lic_ros2/rosbags-venv/lib/python3.12/site-packages \
  /usr/bin/python3 scripts/fastlivo2_ros1_to_frontend_raw.py \
  --input /home/frank/data/fast_livo/Bright_Screen_Wall.bag \
  --output /home/frank/data/fast_livo/Bright_Screen_Wall_frontend_raw \
  --overwrite
```

Validate the converted bag:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run gaussian_lic_tools gaussian_lic_bag_check \
  --bag /home/frank/data/fast_livo/Bright_Screen_Wall_frontend_raw \
  --contract frontend_sensor_raw \
  --json
```

`frontend_sensor_raw` is the true LIC2 sensor-input contract. It requires camera image, CameraInfo, LiDAR PointCloud2, and IMU topics, but does not require pose or odometry. `frontend_raw` remains the temporary adapter contract for smoke tests that inject a pose source.

Create the curated ROS1 upstream mapper-contract substitute from the official `Bright_Screen_Wall_frontend_raw` bag:

```bash
PYTHONPATH=/home/frank/.cache/gaussian_lic_ros2/rosbags-venv/lib/python3.12/site-packages \
  /usr/bin/python3 scripts/frontend_raw_to_ros1_mapper_contract.py \
  --input /home/frank/data/fast_livo/Bright_Screen_Wall_frontend_raw \
  --output /home/frank/data/fast_livo/Bright_Screen_Wall_mapper_contract_8s.bag \
  --max-duration-sec 8 \
  --overwrite
```

For a camera-frame mapper-contract bag using the released FAST-LIVO2 camera-LiDAR extrinsics:

```bash
PYTHONPATH=/home/frank/.cache/gaussian_lic_ros2/rosbags-venv/lib/python3.12/site-packages \
  /usr/bin/python3 scripts/frontend_raw_to_ros1_mapper_contract.py \
  --input /home/frank/data/fast_livo/Bright_Screen_Wall_frontend_raw \
  --output /home/frank/data/fast_livo/Bright_Screen_Wall_mapper_contract_fastlivo2_extrinsic_8s.bag \
  --max-duration-sec 8 \
  --pointcloud-transform-profile fastlivo2 \
  --overwrite
```

To make the ROS1 upstream baseline consume the same image-projected color
surface as the ROS2 mapper, add packed RGB fields to the mapper-contract point
cloud:

```bash
PYTHONPATH=/home/frank/.cache/gaussian_lic_ros2/rosbags-venv/lib/python3.12/site-packages \
  /usr/bin/python3 scripts/frontend_raw_to_ros1_mapper_contract.py \
  --input /home/frank/data/fast_livo/Bright_Screen_Wall_frontend_raw \
  --output /home/frank/data/fast_livo/Bright_Screen_Wall_mapper_contract_fastlivo2_color_8s.bag \
  --max-duration-sec 8 \
  --pointcloud-transform-profile fastlivo2 \
  --colorize-pointcloud \
  --overwrite
```

Run the ROS1 upstream baseline on that shared mapper-contract input:

```bash
./scripts/run_upstream_baseline.sh \
  --bag /home/frank/data/fast_livo/Bright_Screen_Wall_mapper_contract_8s.bag \
  --sequence Bright_Screen_Wall_curated_8s \
  --output baseline/fastlivo2/Bright_Screen_Wall_curated_8s \
  --rebuild \
  --runtime-sec 180
```

Run the curated baseline-vs-current report:

```bash
./scripts/reproduction_report.py \
  --baseline-dir baseline/fastlivo2/Bright_Screen_Wall_curated_8s \
  --current-dir results/fastlivo2/Bright_Screen_Wall_current \
  --sequence Bright_Screen_Wall_curated_8s \
  --metric-key debug_points \
  --trajectory-align first \
  --min-trajectory-coverage 0.4 \
  --max-association-dt 0.2 \
  --pointcloud-align centroid \
  --max-pointcloud-points 50000 \
  --max-nearest-m 0.5 \
  --max-chamfer-rmse-m 0.5 \
  --max-chamfer-mean-m 0.5 \
  --max-chamfer-max-m 0.5 \
  --max-unmatched-ratio 0.25 \
  --min-point-count-ratio 0.5 \
  --max-point-count-ratio 5.0 \
  --max-centroid-drift-m 0.5 \
  --output results/fastlivo2/Bright_Screen_Wall_current/reproduction_report.json \
  --markdown results/fastlivo2/Bright_Screen_Wall_current/reproduction_report.md
```

For the paper gate, use `--strict` on the real `CBD_Building_01` baseline/current
pair. Strict mode requires finite novel-view PSNR, SSIM, and LPIPS metrics in
both `metrics.json` files, enforces the default 5% regression limit, and checks
matched rendered images under `renders/`:

```bash
./scripts/baseline_readiness.py \
  --dataset-root /home/frank/data/fast_livo \
  --baseline-dir baseline/fastlivo2/CBD_Building_01 \
  --current-results-dir results/fastlivo2/current \
  --sequence CBD_Building_01 \
  --strict
```

After `/home/frank/data/fast_livo/CBD_Building_01.bag` is present, the full
local strict chain can be run from one resumable entrypoint:

```bash
./scripts/run_strict_cbd_pipeline.sh \
  --overwrite \
  --render-mode rasterizer \
  --current-torch-device cuda \
  --current-torch-optimization-steps 1 \
  --current-torch-max-foreground 20000
```

The script converts the official ROS1 bag into a sqlite-backed ROS2
`frontend_raw` bag using header-stamp-sorted writes, audits replay timing, writes
the ROS1 mapper-contract bag, runs the upstream ROS1 baseline container,
collects ROS2 current artifacts, and emits strict readiness/report JSON and
Markdown files under the current-results directory.

Before strict reporting, the script runs `scripts/eval_render_quality.py` on the
baseline and current render directories when upstream GT images are available.
That step fills finite PSNR, SSIM, and LPIPS fields under
`metrics.json::quality`. The upstream TorchScript LPIPS file is traced with CUDA
tensors, so CPU-only report environments should use the lpipsPyTorch fallback via
a Python environment containing `torch`, `torchvision`, `numpy`, and `Pillow`.
If `${HOME}/.cache/gaussian_lic_ros2/quality-venv/bin/python` exists it is
selected automatically; otherwise set `QUALITY_PYTHON=/path/to/python`.

The validated curated chain can also be re-run from one entrypoint:

```bash
./scripts/run_curated_fastlivo2_report.sh --skip-build
```

The transformed Bright substitute uses the released FAST-LIVO2 camera-LiDAR
extrinsics for both the ROS1 mapper-contract baseline and the ROS2 LIC2 adapter
current run. The transform flag selects the validated sequence name, mapper bag,
baseline directory, current directory, and 10 second current recording window
unless those paths are explicitly overridden:

```bash
./scripts/run_curated_fastlivo2_report.sh \
  --fastlivo2-camera-lidar-transform \
  --skip-build
```

For a report-only refresh using the already archived ROS1 baseline and ROS2 current artifacts:

```bash
./scripts/run_curated_fastlivo2_report.sh \
  --skip-convert \
  --skip-current \
  --skip-baseline \
  --skip-build
```

For the transformed Bright report-only refresh:

```bash
./scripts/run_curated_fastlivo2_report.sh \
  --fastlivo2-camera-lidar-transform \
  --skip-convert \
  --skip-current \
  --skip-baseline \
  --skip-build
```

Add `--derive-gaussian-rgb` to the primary point-cloud report command only when
the goal is to audit color drift from upstream Gaussian PLY `f_dc_0..2`
coefficients. For the current executable Bright substitute, pass
`--gaussian-color-current-point-cloud` to add the dedicated Torch Gaussian color
gate while keeping the full-sequence non-Torch trajectory and geometry gates.

## Execution Status: 2026-05-04

Strict target sequence:

```text
CBD_Building_01.bag
```

Status:

- Official FAST-LIVO2 Google Drive discovery works and resolves `CBD_Building_01.bag` to a 5,218,555,727 byte file; `/home/frank/data/fast_livo/CBD_Building_01.bag` is now present locally and has produced a ROS1 upstream baseline archive.
- The official SharePoint source currently returns HTTP 404 from the readiness probe.
- `Bright_Screen_Wall.bag` has been downloaded from the same official FAST-LIVO2 Google Drive mirror and converted to `/home/frank/data/fast_livo/Bright_Screen_Wall_frontend_raw`.
- The converted `Bright_Screen_Wall_frontend_raw` bag passes `frontend_sensor_raw` and has replayed through the ROS2 LIC2 contract adapter, forwarding camera images, CameraInfo, LiDAR PointCloud2, and IMU data.
- The upstream ROS1 Gaussian-LIC/Gaussian-LIC2 Noetic build is build-complete in Docker with the local OpenCV 4.10 CUDA fallback and TensorRT 8.6.1.6 SDK.
- `Bright_Screen_Wall_frontend_raw` has produced a ROS2 current artifact directory at `results/fastlivo2/Bright_Screen_Wall_current`.
- The curated official substitute `Bright_Screen_Wall_curated_8s` has produced a ROS1 upstream baseline archive at `baseline/fastlivo2/Bright_Screen_Wall_curated_8s`.
- The curated report at `results/fastlivo2/Bright_Screen_Wall_current/reproduction_report.json` passes metrics, trajectory, and point-cloud gates. The metrics gate is limited to the shared `debug_points` artifact count because this substitute uses identity-pose fallback and is not the strict `CBD_Building_01` paper gate.
- `scripts/run_curated_fastlivo2_report.sh` now replays that executable chain end to end, including report-only refresh mode for already archived artifacts.
- The transformed FAST-LIVO2 extrinsic path has produced `/home/frank/data/fast_livo/Bright_Screen_Wall_mapper_contract_fastlivo2_extrinsic_8s.bag`, a ROS1 upstream baseline at `baseline/fastlivo2/Bright_Screen_Wall_fastlivo2_extrinsic_8s`, and a deterministic ROS2 current report at `results/fastlivo2/Bright_Screen_Wall_current_extrinsic_fullseq/reproduction_report.json` with metrics, trajectory, and point-cloud gates passing. This validates camera-frame color/photometric supervision mechanics on official data, but it remains a Bright substitute rather than the strict `CBD_Building_01` paper gate while that sequence is quota-blocked and the adapter still uses pose fallback.
- The colorized transformed mapper-contract path has produced `/home/frank/data/fast_livo/Bright_Screen_Wall_mapper_contract_fastlivo2_color_8s.bag`, a ROS1 upstream baseline at `baseline/fastlivo2/Bright_Screen_Wall_fastlivo2_color_8s`, and a combined ROS2 current report at `results/fastlivo2/Bright_Screen_Wall_current_extrinsic_color_fullseq/reproduction_report.json` with metrics, trajectory, point-cloud, and dedicated Torch Gaussian color gates passing. The `gaussian_color` gate audits `results/fastlivo2/Bright_Screen_Wall_current_extrinsic_color_torch/point_cloud.ply` with Gaussian `f_dc_0..2` derived RGB drift `11.657 < 40.0`. This is a complete executable Bright substitute proof chain for current ROS2 plumbing and Torch tensor boundaries, not a completed paper-level Gaussian-LIC2 algorithm port.

Current ROS1 upstream build status:

- TensorRT SDK is installed at `/home/frank/Software/TensorRT-8.6.1.6`; `NvInfer.h`, `libnvinfer.so`, `libnvonnxparser.so`, `libnvparsers.so`, and `libnvinfer_plugin.so` are present.
- The strict local OpenCV 4.7.0 build under `/home/frank/Software/opencv/opencv-4.7.0/build` is incomplete for upstream linking; the generated OpenCV config references `libopencv_gapi.so.4.7.0`, but that library is absent. A direct rebuild attempt of the 4.7 targets currently fails in the host Conda/CMake linker environment on `/lib64/libm.so.6` and `/lib64/libmvec.so.1`.
- The existing OpenCV 4.10 CUDA build under `/home/frank/Software/opencv/opencv-4.10.0/build` removes the OpenCV link blocker when passed with `--opencv-dir`; with TensorRT installed, the upstream `gs_mapping` target builds successfully.
- Current CUDA/compiler pairing needs a compatibility include for `FLT_MAX` in copied upstream CUDA files. `scripts/upstream_baseline_build_attempt.sh` applies that include only inside `/tmp/catkin_gaussian`, leaving `external/Gaussian-LIC` unchanged.
- `scripts/run_upstream_baseline.sh` applies runtime-only patches to the copied upstream workspace: lazy depth-completer construction when depth completion is disabled, compatible `libstdc++` preloading for libtorch, LPIPS/cuDNN failure isolation, and a fallback Gaussian XYZ PLY writer if upstream binary PLY writing fails.

Re-run the upstream build attempt:

```bash
./scripts/upstream_baseline_build_attempt.sh
```

Re-run the build attempt with the local OpenCV 4.10 CUDA fallback:

```bash
./scripts/upstream_baseline_build_attempt.sh \
  --opencv-dir /home/frank/Software/opencv/opencv-4.10.0/build \
  --log log/noetic_gaussian_lic_build_attempt_opencv410_tensorrt.log
```

The successful OpenCV 4.10 + TensorRT build log is written to:

```text
log/noetic_gaussian_lic_build_attempt_opencv410_tensorrt.log
```

No `baseline/fastlivo2/CBD_Building_01` archive should be created until the upstream run actually emits `trajectory.tum`, `point_cloud.ply`, `metrics.json`, `run.log`, and renders. The comparison gate must remain blocked until both ROS1 baseline artifacts and ROS2 current artifacts exist.

`scripts/collect_current_results.sh` creates the ROS2 current archive from actual `/gaussian_lic/*` mapper outputs. It also stores `run.log`, the recorded `ros2_output_bag/`, the service-saved `saved_map/`, and offline debug extraction under `offline/` for auditability.

Required ROS1 baseline archive:

```text
baseline/fastlivo2/CBD_Building_01/
├── trajectory.tum
├── point_cloud.ply
├── metrics.json
├── run.log
├── renders/
└── baseline_manifest.json
```

After the ROS2 result directory has the matching `trajectory.tum`, `point_cloud.ply`, and `metrics.json`, run:

```bash
./scripts/reproduction_report.py \
  --baseline-dir baseline/fastlivo2/CBD_Building_01 \
  --current-dir results/fastlivo2/current \
  --sequence CBD_Building_01 \
  --output results/fastlivo2/current/reproduction_report.json \
  --markdown results/fastlivo2/current/reproduction_report.md
```

Official source anchors:

- Gaussian-LIC/Gaussian-LIC2 upstream: <https://github.com/APRIL-ZJU/Gaussian-LIC>
- Gaussian-LIC2 project page: <https://xingxingzuo.github.io/gaussian_lic2/>
- FAST-LIVO2 upstream: <https://github.com/hku-mars/FAST-LIVO2>
- FAST-LIVO2 Google Drive mirror: <https://drive.google.com/drive/folders/1bf5LQ8iSxw-fD8BObZmouw7lRxNacfrA?usp=drive_link>
