#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
BAG_PATH="${ROOT_DIR}/bags/synthetic_frontend_raw_demo"
TIMEOUT_SEC=20
ENABLE_SLIDING_WINDOW=true
ENABLE_LIDAR_PLANE_FACTOR=true

usage() {
  cat <<'EOF'
Usage: scripts/tracking_smoke_test.sh [OPTIONS]

Launch the native ROS2 tracking frontend against a frontend-raw synthetic bag
and verify that mapper-contract and frontend odometry/path topics are produced.

Options:
  --bag DIR                         Frontend-raw rosbag2 directory.
                                    Default: bags/synthetic_frontend_raw_demo
  --timeout SEC                     Topic wait timeout. Default: 20
  --no-sliding-window               Disable optional sliding-window optimizer.
  --no-lidar-plane-factor           Disable point-to-plane LiDAR window factors.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --bag)
      BAG_PATH="$2"
      shift 2
      ;;
    --timeout)
      TIMEOUT_SEC="$2"
      shift 2
      ;;
    --no-sliding-window)
      ENABLE_SLIDING_WINDOW=false
      shift
      ;;
    --no-lidar-plane-factor)
      ENABLE_LIDAR_PLANE_FACTOR=false
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

cd "${ROOT_DIR}"
if [[ ! -f "${BAG_PATH}/metadata.yaml" ]]; then
  ./scripts/create_synthetic_bag.sh \
    --frontend-raw \
    --frontend-raw-odometry \
    --output "${BAG_PATH}" \
    --duration 4
fi

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source install/setup.bash
set -u

launch_log="${ROOT_DIR}/log/tracking_smoke_launch.log"
play_log="${ROOT_DIR}/log/tracking_smoke_play.log"
mkdir -p "${ROOT_DIR}/log"
rm -f "${launch_log}" "${play_log}"

cleanup() {
  if [[ -n "${play_pid:-}" ]]; then
    kill "${play_pid}" 2>/dev/null || true
    wait "${play_pid}" 2>/dev/null || true
  fi
  if [[ -n "${launch_pid:-}" ]]; then
    kill -- -"${launch_pid}" 2>/dev/null || true
    wait "${launch_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

setsid ros2 launch gaussian_lic_bringup tracking.launch.py \
  enable_sliding_window_optimizer:="${ENABLE_SLIDING_WINDOW}" \
  enable_lidar_plane_factor:="${ENABLE_LIDAR_PLANE_FACTOR}" \
  lidar_min_points:=1 \
  lidar_nearest_distance_m:=2.0 \
  lidar_keyframe_translation_m:=0.0 \
  >"${launch_log}" 2>&1 &
launch_pid=$!

sleep 2
ros2 bag play "${BAG_PATH}" --clock --rate 1.0 >"${play_log}" 2>&1 &
play_pid=$!

for topic in /pose_for_gs /points_for_gs /gaussian_lic/frontend/odometry /gaussian_lic/frontend/path; do
  timeout "${TIMEOUT_SEC}" ros2 topic echo --once "${topic}" >/dev/null
done
timeout "${TIMEOUT_SEC}" ros2 topic echo --once \
  /gaussian_lic/frontend/status gaussian_lic_msgs/msg/TrackingStatus \
  >/tmp/gaussian_lic_tracking_smoke_status.txt
rg -q "state: 2" /tmp/gaussian_lic_tracking_smoke_status.txt
rg -q "sliding_window_enabled: true" /tmp/gaussian_lic_tracking_smoke_status.txt
rg -q "sliding_window_imu_factors: [1-9]" /tmp/gaussian_lic_tracking_smoke_status.txt
rg -q "sliding_window_point_factors: [1-9]" /tmp/gaussian_lic_tracking_smoke_status.txt
rg -q "total_window_point_correspondences: [1-9]" /tmp/gaussian_lic_tracking_smoke_status.txt
rg -q "num_lidar_keyframes: [1-9]" /tmp/gaussian_lic_tracking_smoke_status.txt

echo "[tracking-smoke] passed"
echo "[tracking-smoke] launch log: ${launch_log}"
