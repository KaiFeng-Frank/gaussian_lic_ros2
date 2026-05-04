# Docker

The Docker path is intended for the current ROS2-native smoke-test slice. It builds the non-torch workspace on ROS2 Jazzy and can run the synthetic mapping checks.

## Build

Requires Docker and the Docker Compose v2 plugin for the compose commands below.

```bash
cd /home/frank/gaussian_lic_ros2
docker build -f docker/Dockerfile.jazzy -t gaussian_lic_ros2:jazzy .
```

## Shell

```bash
docker compose -f docker/compose.yaml run --rm gaussian_lic_ros2
```

If the Compose plugin is not installed, use Docker directly:

```bash
docker run --rm -it \
  --network host \
  --ipc host \
  --gpus all \
  -e DISPLAY="${DISPLAY}" \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=compute,graphics,utility \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$PWD":/ws \
  gaussian_lic_ros2:jazzy \
  bash
```

The compose service mounts the repository at `/ws`, uses host networking, and requests NVIDIA GPUs for RViz/CUDA-friendly runs. The default image does not install libtorch, so torch-enabled Gaussian initialization is still a native-machine path for now.

## Smoke Test

```bash
docker compose -f docker/compose.yaml run --rm smoke
```

Without Compose:

```bash
docker run --rm -it \
  --network host \
  --ipc host \
  --gpus all \
  -v "$PWD":/ws \
  gaussian_lic_ros2:jazzy \
  bash -lc "./scripts/build_ros2.sh && ./scripts/smoke_test.sh --tf"
```

This runs:

```bash
./scripts/build_ros2.sh
./scripts/smoke_test.sh --tf
```

The smoke test starts synthetic synchronized inputs, verifies `/gaussian_lic/odometry`, `/gaussian_lic/path`, `/gaussian_lic/map_points`, and checks optional `map -> camera` TF output.

To generate and replay a synthetic rosbag2 inside the container:

```bash
./scripts/create_synthetic_bag.sh --output bags/synthetic_gs_demo
./scripts/smoke_test.sh --bag bags/synthetic_gs_demo --tf
```

## Native Torch Smoke Test

On the local machine with libtorch installed:

```bash
GAUSSIAN_LIC_ENABLE_TORCH=ON ./scripts/build_ros2.sh --packages-select gaussian_lic_mapping
./scripts/build_ros2.sh --packages-select gaussian_lic_bringup
./scripts/smoke_test.sh --torch --tf
```
