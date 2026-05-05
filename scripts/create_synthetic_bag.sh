#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
OUTPUT="${ROOT_DIR}/bags/synthetic_gs_demo"
DURATION_SEC=6
FRONTEND_RAW=false
FRONTEND_RAW_ODOMETRY=false
IMAGE_WIDTH=1
IMAGE_HEIGHT=1
IMAGE_PATTERN=solid
IMAGE_SHIFT_X_PX=0.0
IMAGE_SHIFT_Y_PX=0.0

usage() {
  cat <<'EOF'
Usage: scripts/create_synthetic_bag.sh [--output DIR] [--duration SEC] [--frontend-raw] [--frontend-raw-odometry] [--image-width PX] [--image-height PX] [--image-pattern solid|gradient|gaussian] [--image-shift-x PX] [--image-shift-y PX]

Records a small rosbag2 from synthetic Gaussian-LIC mapper inputs.

Options:
  --frontend-raw  Record raw LIC2 frontend adapter inputs instead of mapper topics.
  --frontend-raw-odometry
                  In frontend raw mode, record raw Odometry instead of PoseStamped.
  --image-width PX
                  Synthetic image width. Default: 1.
  --image-height PX
                  Synthetic image height. Default: 1.
  --image-pattern solid|gradient|gaussian
                  Synthetic image pattern. Default: solid.
  --image-shift-x PX
                  Subpixel horizontal shift applied to the recorded image.
  --image-shift-y PX
                  Subpixel vertical shift applied to the recorded image.
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
    --frontend-raw-odometry)
      FRONTEND_RAW=true
      FRONTEND_RAW_ODOMETRY=true
      shift
      ;;
    --image-width)
      IMAGE_WIDTH="$2"
      shift 2
      ;;
    --image-height)
      IMAGE_HEIGHT="$2"
      shift 2
      ;;
    --image-pattern)
      IMAGE_PATTERN="$2"
      shift 2
      ;;
    --image-shift-x)
      IMAGE_SHIFT_X_PX="$2"
      shift 2
      ;;
    --image-shift-y)
      IMAGE_SHIFT_Y_PX="$2"
      shift 2
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

if [[ -n "${GAUSSIAN_LIC_BAG_ROS_DOMAIN_ID:-}" ]]; then
  export ROS_DOMAIN_ID="${GAUSSIAN_LIC_BAG_ROS_DOMAIN_ID}"
elif [[ -z "${ROS_DOMAIN_ID:-}" ]]; then
  export ROS_DOMAIN_ID="$((120 + (BASHPID % 100)))"
fi

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
  )
  if [[ "${FRONTEND_RAW_ODOMETRY}" == "true" ]]; then
    topics+=(
      /gaussian_lic/frontend/input_odometry
    )
    pose_output_args=(
      -p pose_output_mode:=odometry
      -p odometry_topic:=/gaussian_lic/frontend/input_odometry
    )
  else
    topics+=(
      /gaussian_lic/frontend/pose
    )
    pose_output_args=(
      -p pose_output_mode:=pose_stamped
      -p pose_topic:=/gaussian_lic/frontend/pose
    )
  fi
  publisher_args=(
    --ros-args
    --params-file "${ROOT_DIR}/src/gaussian_lic_bringup/config/default.yaml"
    -p pointcloud_topic:=/livox/lidar
    -p image_topic:=/camera/image
    -p camera_info_topic:=/camera/camera_info
    -p depth_topic:=/camera/depth
    -p imu_topic:=/imu
    -p image_width:="${IMAGE_WIDTH}"
    -p image_height:="${IMAGE_HEIGHT}"
    -p image_pattern:="${IMAGE_PATTERN}"
    -p image_shift_x_px:="${IMAGE_SHIFT_X_PX}"
    -p image_shift_y_px:="${IMAGE_SHIFT_Y_PX}"
    "${pose_output_args[@]}"
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
    -p image_width:="${IMAGE_WIDTH}"
    -p image_height:="${IMAGE_HEIGHT}"
    -p image_pattern:="${IMAGE_PATTERN}"
    -p image_shift_x_px:="${IMAGE_SHIFT_X_PX}"
    -p image_shift_y_px:="${IMAGE_SHIFT_Y_PX}"
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
