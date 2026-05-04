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
  --output results/fastlivo2/current
```

For the current Torch Gaussian preview path:

```bash
./scripts/collect_current_results.sh \
  --bag /home/frank/data/fast_livo/<sequence>_frontend_raw \
  --frontend-adapter \
  --torch \
  --render-mode rasterizer \
  --output results/fastlivo2/current
```

The output directory is intentionally shaped for `scripts/reproduction_report.py` and `scripts/baseline_readiness.py`: `trajectory.tum`, `point_cloud.ply`, and `metrics.json` are required.

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

## Execution Status: 2026-05-04

Strict target sequence:

```text
CBD_Building_01.bag
```

Status:

- Official FAST-LIVO2 Google Drive discovery works and resolves `CBD_Building_01.bag` to a 5,218,555,727 byte file, but the current download attempt is blocked by Google Drive quota.
- The official SharePoint source currently returns HTTP 404 from the readiness probe.
- `Bright_Screen_Wall.bag` has been downloaded from the same official FAST-LIVO2 Google Drive mirror and converted to `/home/frank/data/fast_livo/Bright_Screen_Wall_frontend_raw`.
- The converted `Bright_Screen_Wall_frontend_raw` bag passes `frontend_sensor_raw` and has replayed through the ROS2 LIC2 contract adapter, forwarding camera images, CameraInfo, LiDAR PointCloud2, and IMU data.
- The upstream ROS1 Gaussian-LIC/Gaussian-LIC2 Noetic build has been executed inside Docker and reaches CUDA/C++ compilation, but the local baseline environment is not yet build-complete.

Current ROS1 upstream build status:

- Missing TensorRT SDK at `~/Software/TensorRT-8.6.1.6`: `NvInfer.h`, `libnvinfer.so`, `libnvonnxparser.so`, `libnvparsers.so`, and `libnvinfer_plugin.so` are required by upstream CMake.
- The strict local OpenCV 4.7.0 build under `/home/frank/Software/opencv/opencv-4.7.0/build` is incomplete for upstream linking; the generated OpenCV config references `libopencv_gapi.so.4.7.0`, but that library is absent. A direct rebuild attempt of the 4.7 targets currently fails in the host Conda/CMake linker environment on `/lib64/libm.so.6` and `/lib64/libmvec.so.1`.
- The existing OpenCV 4.10 CUDA build under `/home/frank/Software/opencv/opencv-4.10.0/build` removes the OpenCV link blocker when passed with `--opencv-dir`; the build then fails only on missing TensorRT headers/libraries.
- Current CUDA/compiler pairing needs a compatibility include for `FLT_MAX` in copied upstream CUDA files. `scripts/upstream_baseline_build_attempt.sh` applies that include only inside `/tmp/catkin_gaussian`, leaving `external/Gaussian-LIC` unchanged.

Re-run the upstream build attempt:

```bash
./scripts/upstream_baseline_build_attempt.sh
```

Re-run the build attempt with the local OpenCV 4.10 CUDA fallback:

```bash
./scripts/upstream_baseline_build_attempt.sh \
  --opencv-dir /home/frank/Software/opencv/opencv-4.10.0/build \
  --log log/noetic_gaussian_lic_build_attempt_opencv410.log
```

The build log is written to:

```text
log/noetic_gaussian_lic_build_attempt.log
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
