#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
ENABLE_TORCH=false
PUBLISH_TF=false
USE_COMPOSITION=false
FRONTEND_ADAPTER=false
SENSOR_QOS_RELIABILITY=""
TIMEOUT_SEC=8
SAVE_DIR="/tmp/gaussian_lic_smoke_map"
BAG_PATH=""
CONFIG_PATH=""
RENDER_MODE="debug_cpu"
CHECK_RENDERED_DATA=true
IMAGE_COLOR_FALLBACK_CHECK=false
OPTIONAL_DEPTH_CHECK=false
MINIMAL_INPUTS=false

usage() {
  cat <<'EOF'
Usage: scripts/smoke_test.sh [--torch] [--tf] [--composition] [--frontend-adapter] [--sensor-qos RELIABILITY] [--config FILE] [--render-mode MODE] [--skip-rendered-data-check] [--image-color-fallback-check] [--optional-depth-check] [--minimal-inputs] [--bag DIR] [--timeout SEC] [--save-dir DIR]

Runs the synthetic ROS2 mapping smoke test and verifies published outputs.

Options:
  --torch         Also verify Torch Gaussian map topic and SaveMap service.
  --tf            Enable and verify map -> camera TF output.
  --composition   Load mapping_node as a composable rclcpp component.
  --frontend-adapter
                  Publish synthetic raw sensor topics and route them through lic2_contract_adapter.
  --sensor-qos    Override input sensor QoS reliability: best_effort or reliable.
  --config FILE   Override the bringup parameter YAML.
  --render-mode MODE
                  Override rendered output mode: debug_cpu, debug_input, rasterizer, or off.
  --rendered-image-mode MODE
                  Deprecated alias: projected_map, input, or auto.
  --skip-rendered-data-check
                  Only verify rendered-image metadata, not synthetic red pixel data.
  --image-color-fallback-check
                  Publish an uncolored synthetic cloud and verify image-projected RGB in SaveMap output.
  --optional-depth-check
                  Disable synthetic depth and verify require_depth_topic:=false projected-depth fallback.
  --minimal-inputs
                  Replay/check bags with only points, pose, and image mapper inputs.
  --bag DIR       Replay a rosbag2 directory instead of live synthetic input.
  --timeout SEC   Per-topic wait timeout. Default: 8.
  --save-dir DIR  SaveMap target directory for --torch. Default: /tmp/gaussian_lic_smoke_map.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --torch)
      ENABLE_TORCH=true
      shift
      ;;
    --tf)
      PUBLISH_TF=true
      shift
      ;;
    --composition)
      USE_COMPOSITION=true
      shift
      ;;
    --frontend-adapter)
      FRONTEND_ADAPTER=true
      shift
      ;;
    --sensor-qos)
      SENSOR_QOS_RELIABILITY="$2"
      shift 2
      ;;
    --config)
      CONFIG_PATH="$2"
      shift 2
      ;;
    --render-mode)
      RENDER_MODE="$2"
      shift 2
      ;;
    --rendered-image-mode)
      case "$2" in
        projected_map|auto)
          RENDER_MODE="debug_cpu"
          ;;
        input)
          RENDER_MODE="debug_input"
          ;;
        *)
          RENDER_MODE="$2"
          ;;
      esac
      shift 2
      ;;
    --skip-rendered-data-check)
      CHECK_RENDERED_DATA=false
      shift
      ;;
    --image-color-fallback-check)
      IMAGE_COLOR_FALLBACK_CHECK=true
      shift
      ;;
    --optional-depth-check)
      OPTIONAL_DEPTH_CHECK=true
      shift
      ;;
    --minimal-inputs)
      MINIMAL_INPUTS=true
      shift
      ;;
    --bag)
      BAG_PATH="$2"
      shift 2
      ;;
    --timeout)
      TIMEOUT_SEC="$2"
      shift 2
      ;;
    --save-dir)
      SAVE_DIR="$2"
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

case "${RENDER_MODE}" in
  debug_cpu|debug_input|rasterizer|off)
    ;;
  *)
    echo "Invalid --render-mode: ${RENDER_MODE}" >&2
    usage >&2
    exit 2
    ;;
esac

if [[ "${RENDER_MODE}" != "debug_cpu" ]]; then
  CHECK_RENDERED_DATA=false
fi

if [[ "${MINIMAL_INPUTS}" == "true" ]]; then
  CHECK_RENDERED_DATA=false
fi

if [[ "${IMAGE_COLOR_FALLBACK_CHECK}" == "true" && -n "${BAG_PATH}" ]]; then
  echo "--image-color-fallback-check requires live synthetic input, not --bag" >&2
  exit 2
fi

if [[ "${OPTIONAL_DEPTH_CHECK}" == "true" && -n "${BAG_PATH}" ]]; then
  echo "--optional-depth-check requires live synthetic input, not --bag" >&2
  exit 2
fi

if [[ "${IMAGE_COLOR_FALLBACK_CHECK}" == "true" && "${ENABLE_TORCH}" == "true" ]]; then
  echo "--image-color-fallback-check verifies the non-torch debug PLY SaveMap path" >&2
  exit 2
fi

if [[ "${MINIMAL_INPUTS}" == "true" && "${ENABLE_TORCH}" == "true" ]]; then
  echo "--minimal-inputs verifies the non-torch mapper replay path" >&2
  exit 2
fi

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${ROOT_DIR}/install/setup.bash"
set -u
cd "${ROOT_DIR}"

mkdir -p "${ROOT_DIR}/log"
LAUNCH_LOG="${ROOT_DIR}/log/smoke_test_launch.log"
rm -f "${LAUNCH_LOG}"

launch_args=(
  stub_mode:=false
  frontend_adapter:="${FRONTEND_ADAPTER}"
  publish_tf:="${PUBLISH_TF}"
  use_composition:="${USE_COMPOSITION}"
  render_mode:="${RENDER_MODE}"
)

if [[ -n "${CONFIG_PATH}" ]]; then
  launch_args+=(
    config:="${CONFIG_PATH}"
  )
fi

if [[ -n "${BAG_PATH}" ]]; then
  launch_args+=(
    bag:="${BAG_PATH}"
    play_bag:=true
    loop_bag:=true
    synthetic_input:=false
    use_sim_time:=true
  )
  if [[ "${MINIMAL_INPUTS}" == "true" ]]; then
    launch_args+=(
      require_depth_topic:=false
    )
  fi
else
  launch_args+=(
    synthetic_input:=true
    use_sim_time:=false
  )
  if [[ "${IMAGE_COLOR_FALLBACK_CHECK}" == "true" ]]; then
    launch_args+=(
      synthetic_pointcloud_color_mode:=none
      synthetic_image_color_rgb:=255,32,16
    )
  fi
  if [[ "${OPTIONAL_DEPTH_CHECK}" == "true" ]]; then
    launch_args+=(
      require_depth_topic:=false
      synthetic_publish_depth:=false
    )
  fi
fi

if [[ "${ENABLE_TORCH}" == "true" ]]; then
  launch_args+=(
    enable_torch_camera_conversion:=true
    enable_torch_gaussian_init:=true
    torch_gaussian_device:=cpu
  )
fi

if [[ -n "${SENSOR_QOS_RELIABILITY}" ]]; then
  launch_args+=(
    sensor_qos_reliability:="${SENSOR_QOS_RELIABILITY}"
  )
fi

setsid ros2 launch gaussian_lic_bringup run_bag.launch.py "${launch_args[@]}" \
  >"${LAUNCH_LOG}" 2>&1 &
LAUNCH_PID=$!

cleanup() {
  local pgid="-${LAUNCH_PID}"
  kill -INT "${pgid}" >/dev/null 2>&1 || true
  sleep 1
  kill -TERM "${pgid}" >/dev/null 2>&1 || true
  sleep 1
  kill -KILL "${pgid}" >/dev/null 2>&1 || true
  wait "${LAUNCH_PID}" >/dev/null 2>&1 || true
}
trap cleanup EXIT

run_check() {
  local name="$1"
  shift
  echo "[smoke] checking ${name}"
  "$@"
}

sleep 1

run_check "odometry" \
  ros2 topic echo --once --timeout "${TIMEOUT_SEC}" \
    /gaussian_lic/odometry nav_msgs/msg/Odometry >/tmp/gaussian_lic_smoke_odom.txt

if [[ "${MINIMAL_INPUTS}" != "true" ]]; then
  run_check "camera_info" \
    ros2 topic echo --once --timeout "${TIMEOUT_SEC}" \
      /camera_info_for_gs sensor_msgs/msg/CameraInfo >/tmp/gaussian_lic_smoke_camera_info.txt

  run_check "imu" \
    ros2 topic echo --once --timeout "${TIMEOUT_SEC}" --no-arr \
      /imu_for_gs sensor_msgs/msg/Imu >/tmp/gaussian_lic_smoke_imu.txt
  rg -q "linear_acceleration:" /tmp/gaussian_lic_smoke_imu.txt
fi

run_check "path" \
  ros2 topic echo --once --timeout "${TIMEOUT_SEC}" \
    --qos-profile default \
    --qos-reliability reliable \
    --qos-durability transient_local \
    --qos-depth 1 \
    --field poses \
    /gaussian_lic/path nav_msgs/msg/Path >/tmp/gaussian_lic_smoke_path.txt

run_check "map_points" \
  ros2 topic echo --once --timeout "${TIMEOUT_SEC}" --no-arr \
    /gaussian_lic/map_points sensor_msgs/msg/PointCloud2 >/tmp/gaussian_lic_smoke_points.txt

run_check "rendered_image" \
  ros2 topic echo --once --timeout "${TIMEOUT_SEC}" --no-arr \
    --qos-profile default \
    --qos-reliability reliable \
    --qos-durability transient_local \
    --qos-depth 1 \
    /gaussian_lic/rendered_image sensor_msgs/msg/Image >/tmp/gaussian_lic_smoke_rendered_image.txt
rg -q "encoding: rgb8" /tmp/gaussian_lic_smoke_rendered_image.txt
if [[ "${CHECK_RENDERED_DATA}" == "true" ]]; then
  ros2 topic echo --once --timeout "${TIMEOUT_SEC}" \
    --qos-profile default \
    --qos-reliability reliable \
    --qos-durability transient_local \
    --qos-depth 1 \
    --field data \
    /gaussian_lic/rendered_image sensor_msgs/msg/Image >/tmp/gaussian_lic_smoke_rendered_image_data.txt
  rg -q "255" /tmp/gaussian_lic_smoke_rendered_image_data.txt
fi

run_check "status" \
  ros2 topic echo --once --timeout "${TIMEOUT_SEC}" \
    /gaussian_lic/status gaussian_lic_msgs/msg/MappingStatus >/tmp/gaussian_lic_smoke_status.txt
rg -q "tracking_hz: [1-9]" /tmp/gaussian_lic_smoke_status.txt
rg -q "mapping_hz: [1-9]" /tmp/gaussian_lic_smoke_status.txt
rg -q "tracking_state: 2" /tmp/gaussian_lic_smoke_status.txt
rg -q "mapping_state: 2" /tmp/gaussian_lic_smoke_status.txt
rg -q "render_mode: 1" /tmp/gaussian_lic_smoke_status.txt
rg -q "num_mapping_frames: [1-9]" /tmp/gaussian_lic_smoke_status.txt
rg -q "num_errors: 0" /tmp/gaussian_lic_smoke_status.txt

if [[ "${FRONTEND_ADAPTER}" == "true" ]]; then
  run_check "frontend_odometry" \
    ros2 topic echo --once --timeout "${TIMEOUT_SEC}" \
      /gaussian_lic/frontend/odometry nav_msgs/msg/Odometry \
      >/tmp/gaussian_lic_smoke_frontend_odom.txt

  run_check "frontend_path" \
    ros2 topic echo --once --timeout "${TIMEOUT_SEC}" \
      --qos-profile default \
      --qos-reliability reliable \
      --qos-durability transient_local \
      --qos-depth 1 \
      --field poses \
      /gaussian_lic/frontend/path nav_msgs/msg/Path \
      >/tmp/gaussian_lic_smoke_frontend_path.txt
fi

if [[ "${PUBLISH_TF}" == "true" ]]; then
  run_check "tf" \
    ros2 topic echo --once --timeout "${TIMEOUT_SEC}" \
      /tf tf2_msgs/msg/TFMessage >/tmp/gaussian_lic_smoke_tf.txt
fi

if [[ "${ENABLE_TORCH}" == "true" ]]; then
  run_check "gaussian_map" \
    ros2 topic echo --once --timeout "${TIMEOUT_SEC}" \
      --qos-profile default \
      --qos-reliability reliable \
      --qos-durability transient_local \
      --qos-depth 1 \
      /gaussian_lic/gaussian_map gaussian_lic_msgs/msg/GaussianArray \
      >/tmp/gaussian_lic_smoke_gaussian_map.txt

  rm -rf "${SAVE_DIR}"
  run_check "save_map" \
    ros2 service call /gaussian_lic/save_map gaussian_lic_msgs/srv/SaveMap \
      "{path: ${SAVE_DIR}, include_skybox: false}" \
      >/tmp/gaussian_lic_smoke_save_map.txt

  test -f "${SAVE_DIR}/point_cloud.ply"
  rg -q "property float f_dc_" "${SAVE_DIR}/point_cloud.ply"
else
  rm -rf "${SAVE_DIR}"
  run_check "save_debug_map" \
    ros2 service call /gaussian_lic/save_map gaussian_lic_msgs/srv/SaveMap \
      "{path: ${SAVE_DIR}, include_skybox: false}" \
      >/tmp/gaussian_lic_smoke_save_map.txt

  test -f "${SAVE_DIR}/point_cloud.ply"
  rg -q "property uchar red" "${SAVE_DIR}/point_cloud.ply"
  if [[ "${IMAGE_COLOR_FALLBACK_CHECK}" == "true" ]]; then
    rg -q "^[-+0-9.eE]+ [-+0-9.eE]+ [-+0-9.eE]+ 255 32 16$" \
      "${SAVE_DIR}/point_cloud.ply"
  fi
fi

echo "[smoke] passed"
echo "[smoke] launch log: ${LAUNCH_LOG}"
