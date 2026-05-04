# Common Errors

## Build Uses Conda Python

Symptom:

```text
ModuleNotFoundError: No module named 'rosidl_adapter'
```

Use the build wrapper:

```bash
./scripts/build_jazzy.sh
```

It pins ROS2 interface generation to `/usr/bin/python3`.

## Non-ASCII Workspace Path

Symptom:

```text
rosidl_generate_interfaces or CMake fails under /home/frank/桌面/...
```

Build from the ASCII path:

```bash
cd /home/frank/gaussian_lic_ros2
```

`/home/frank/桌面/gaussian_lic_ros2` is only a symlink.

## No Messages From Rosbag2

Symptom:

```text
ros2 topic echo --once ... timed out
```

For short bags, keep playback looping during verification:

```bash
./scripts/smoke_test.sh --bag bags/synthetic_gs_demo --tf
```

The smoke script launches `ros2 bag play --loop` and avoids races with volatile sensor topics.

## Reliable Driver Or Bag Does Not Match Best-Effort Subscriber

Symptom:

```text
mapping_node starts, but point/image/depth counters stay at zero
```

Try reliable input QoS:

```bash
ros2 launch gaussian_lic_bringup run_bag.launch.py \
  stub_mode:=false \
  bag:=bags/synthetic_gs_demo \
  play_bag:=true \
  loop_bag:=true \
  sensor_qos_reliability:=reliable
```

The same path is available in smoke tests:

```bash
./scripts/smoke_test.sh --bag bags/synthetic_gs_demo --sensor-qos reliable --tf
```

## Dataset Profile Preview Looks Black

The synthetic demo image is `1x1`, but dataset profiles use real intrinsics such as `cx ~= 313`. The projected synthetic point can therefore fall outside the tiny image.

For profile smoke tests, verify the input preview path:

```bash
./scripts/smoke_test.sh \
  --bag bags/synthetic_gs_demo \
  --config src/gaussian_lic_bringup/config/fastlivo2.yaml \
  --render-mode debug_cpu \
  --skip-rendered-data-check \
  --tf
```

## Libtorch CMake Warns About NVRTC Or NVTX

Symptom:

```text
Failed to compute shorthash for libnvrtc.so
Cannot find NVTX3, find old NVTX instead
```

On the tested local machine these are warnings from the libtorch CMake package and do not block the torch backend. Verify with:

```bash
GAUSSIAN_LIC_ENABLE_TORCH=ON ./scripts/build_jazzy.sh --packages-select gaussian_lic_mapping
source install/setup.bash
ros2 run gaussian_lic_mapping torch_backend_probe
```

This warning is known for the `v0.1.0-m1-infra` checkpoint and should not be treated as a new regression unless the torch probe or torch smoke test fails.

## TensorRT Not Found

TensorRT is not installed on the tested machine. Depth completion must stay optional until the full Gaussian-LIC backend is ported.

Current mapper smoke tests do not require TensorRT:

```bash
./scripts/smoke_test.sh --tf
```

## ROS1 Bag Converter Missing `rosbags`

Symptom:

```text
ROS1 bag conversion requires the optional Python package 'rosbags'.
```

Install it only in the conversion environment:

```bash
/usr/bin/python3 -m pip install --user rosbags
```

Then rerun:

```bash
./scripts/convert_ros1_bag_to_rosbag2.py \
  --input /path/to/input.bag \
  --output bags/input_ros2 \
  --storage mcap \
  --dst-typestore ros2_jazzy
```

The Jazzy build and smoke-test path intentionally does not depend on ROS1 or `rosbags`.

The bag contract checker uses the same optional dependency when inspecting ROS1 `.bag` files:

```bash
gaussian_lic_bag_check --bag /path/to/input.bag --bag-format ros1 --json
```
