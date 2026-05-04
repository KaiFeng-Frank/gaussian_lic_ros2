#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
OUTPUT="${ROOT_DIR}/bags/synthetic_gs_demo"
DURATION_SEC=6
FRONTEND_RAW=false

usage() {
  cat <<'EOF'
Usage: scripts/create_synthetic_bag.sh [--output DIR] [--duration SEC] [--frontend-raw]

Records a small rosbag2 from synthetic Gaussian-LIC mapper inputs.

Options:
  --frontend-raw  Record raw LIC2 frontend adapter inputs instead of mapper topics.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --output)
      OUTPUT="$2"
      shift 2
      ;;
    --duration)
      DURATION_SEC="$2"
      shift 2
      ;;
    --frontend-raw)
      FRONTEND_RAW=true
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

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${ROOT_DIR}/install/setup.bash"
set -u
cd "${ROOT_DIR}"

mkdir -p "$(dirname "${OUTPUT}")" "${ROOT_DIR}/log"
rm -rf "${OUTPUT}"

if [[ "${FRONTEND_RAW}" == "true" ]]; then
  topics=(
    /camera/image
    /camera/camera_info
    /camera/depth
    /livox/lidar
    /imu
    /gaussian_lic/frontend/pose
  )
  publisher_args=(
    --ros-args
    --params-file "${ROOT_DIR}/src/gaussian_lic_bringup/config/default.yaml"
    -p pointcloud_topic:=/livox/lidar
    -p pose_topic:=/gaussian_lic/frontend/pose
    -p image_topic:=/camera/image
    -p camera_info_topic:=/camera/camera_info
    -p depth_topic:=/camera/depth
    -p imu_topic:=/imu
  )
else
  topics=(
    /points_for_gs
    /pose_for_gs
    /image_for_gs
    /camera_info_for_gs
    /depth_for_gs
    /imu_for_gs
  )
  publisher_args=(
    --ros-args
    --params-file "${ROOT_DIR}/src/gaussian_lic_bringup/config/default.yaml"
  )
fi

REC_LOG="${ROOT_DIR}/log/create_synthetic_bag_record.log"
PUB_LOG="${ROOT_DIR}/log/create_synthetic_bag_publish.log"
rm -f "${REC_LOG}" "${PUB_LOG}"

setsid ros2 bag record -o "${OUTPUT}" "${topics[@]}" >"${REC_LOG}" 2>&1 &
REC_PID=$!

setsid ros2 run gaussian_lic_tools synthetic_gs_frame_pub \
  "${publisher_args[@]}" \
  >"${PUB_LOG}" 2>&1 &
PUB_PID=$!

cleanup() {
  kill -INT "-${PUB_PID}" >/dev/null 2>&1 || true
  kill -INT "-${REC_PID}" >/dev/null 2>&1 || true
  sleep 1
  kill -TERM "-${PUB_PID}" "-${REC_PID}" >/dev/null 2>&1 || true
  sleep 1
  kill -KILL "-${PUB_PID}" "-${REC_PID}" >/dev/null 2>&1 || true
  wait "${PUB_PID}" >/dev/null 2>&1 || true
  wait "${REC_PID}" >/dev/null 2>&1 || true
}
trap cleanup EXIT

sleep "${DURATION_SEC}"
cleanup
trap - EXIT

if [[ ! -f "${OUTPUT}/metadata.yaml" ]]; then
  echo "Failed to create rosbag2 at ${OUTPUT}" >&2
  echo "record log: ${REC_LOG}" >&2
  echo "publisher log: ${PUB_LOG}" >&2
  exit 1
fi

echo "[bag] wrote ${OUTPUT}"
ros2 bag info "${OUTPUT}"
