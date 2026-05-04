#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
BAG_PATH=""
OUTPUT_DIR="${ROOT_DIR}/results/fastlivo2/current"
CONFIG_PATH=""
ENABLE_TORCH=false
TORCH_OPTIMIZATION_STEPS=0
TORCH_MAX_FOREGROUND=0
TORCH_PRUNE_MIN_OPACITY=0.005
FRONTEND_ADAPTER=false
IDENTITY_POSE_FALLBACK=false
IMU_POSE_FALLBACK=false
POINTCLOUD_TRANSFORM_PROFILE="identity"
LIVOX_CUSTOM_BRIDGE=false
LIVOX_CUSTOM_TOPIC="/livox/lidar"
LIVOX_POINTCLOUD_TOPIC="/livox/lidar/points"
PUBLISH_TF=false
REQUIRE_DEPTH_TOPIC=true
RENDER_MODE="debug_cpu"
RECORD_SEC=12
TIMEOUT_SEC=20
SENSOR_QOS_RELIABILITY=""

usage() {
  cat <<'EOF'
Usage: scripts/collect_current_results.sh [options]

Run the ROS2 Gaussian-LIC mapper on a rosbag2 input or live synthetic input and
write current reproduction artifacts:

  trajectory.tum
  point_cloud.ply
  metrics.json
  run.log
  ros2_output_bag/
  saved_map/

Options:
  --bag DIR                    Replay a rosbag2 directory. If omitted, live synthetic input is used.
  --output DIR                 Output directory. Default: results/fastlivo2/current.
  --config FILE                Override bringup parameter YAML.
  --torch                      Enable Torch Gaussian map init/extend and save Gaussian PLY output.
  --torch-optimization-steps N Enable Torch photometric Gaussian tensor updates with N steps per keyframe.
  --torch-max-foreground N     Enable Torch pruning and retain at most N foreground Gaussians.
  --torch-prune-min-opacity X  Enable Torch pruning and drop foreground Gaussians below opacity X.
  --frontend-adapter           Route raw frontend topics through lic2_contract_adapter.
  --identity-pose-fallback     Let the frontend adapter publish identity poses from point-cloud stamps.
  --imu-pose-fallback          Let the frontend adapter integrate IMU gyro orientation for pose fallback.
  --fastlivo2-camera-lidar-transform
                               Transform raw FAST-LIVO2 LiDAR points into camera frame in the adapter.
  --livox-custom-bridge        Bridge Livox CustomMsg input to PointCloud2 before the adapter.
  --optional-depth             Allow mapper replay without a depth image topic.
  --tf                         Enable TF publication.
  --render-mode MODE           debug_cpu, debug_input, rasterizer, or off. Default: debug_cpu.
  --sensor-qos RELIABILITY     Override input sensor QoS reliability: best_effort or reliable.
  --record-sec SEC             Output recording duration. Default: 12.
  --timeout SEC                Topic/service wait timeout. Default: 20.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --bag)
      BAG_PATH="$2"
      shift 2
      ;;
    --output)
      OUTPUT_DIR="$2"
      shift 2
      ;;
    --config)
      CONFIG_PATH="$2"
      shift 2
      ;;
    --torch)
      ENABLE_TORCH=true
      shift
      ;;
    --torch-optimization-steps)
      ENABLE_TORCH=true
      TORCH_OPTIMIZATION_STEPS="$2"
      shift 2
      ;;
    --torch-max-foreground)
      ENABLE_TORCH=true
      TORCH_MAX_FOREGROUND="$2"
      shift 2
      ;;
    --torch-prune-min-opacity)
      ENABLE_TORCH=true
      TORCH_PRUNE_MIN_OPACITY="$2"
      shift 2
      ;;
    --frontend-adapter)
      FRONTEND_ADAPTER=true
      shift
      ;;
    --identity-pose-fallback)
      IDENTITY_POSE_FALLBACK=true
      shift
      ;;
    --imu-pose-fallback)
      IMU_POSE_FALLBACK=true
      shift
      ;;
    --fastlivo2-camera-lidar-transform)
      POINTCLOUD_TRANSFORM_PROFILE="fastlivo2"
      shift
      ;;
    --livox-custom-bridge)
      LIVOX_CUSTOM_BRIDGE=true
      shift
      ;;
    --optional-depth)
      REQUIRE_DEPTH_TOPIC=false
      shift
      ;;
    --tf)
      PUBLISH_TF=true
      shift
      ;;
    --render-mode)
      RENDER_MODE="$2"
      shift 2
      ;;
    --sensor-qos)
      SENSOR_QOS_RELIABILITY="$2"
      shift 2
      ;;
    --record-sec)
      RECORD_SEC="$2"
      shift 2
      ;;
    --timeout)
      TIMEOUT_SEC="$2"
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

if [[ "${RENDER_MODE}" == "rasterizer" && "${ENABLE_TORCH}" != "true" ]]; then
  echo "--render-mode rasterizer requires --torch in the current ROS2 port" >&2
  exit 2
fi

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${ROOT_DIR}/install/setup.bash"
set -u
cd "${ROOT_DIR}"

rm -rf "${OUTPUT_DIR}"
mkdir -p "${OUTPUT_DIR}"
RUN_LOG="${OUTPUT_DIR}/run.log"
RECORDED_BAG="${OUTPUT_DIR}/ros2_output_bag"
SAVED_MAP_DIR="${OUTPUT_DIR}/saved_map"

launch_args=(
  stub_mode:=false
  frontend_adapter:="${FRONTEND_ADAPTER}"
  adapter_identity_pose_fallback:="${IDENTITY_POSE_FALLBACK}"
  adapter_imu_pose_fallback:="${IMU_POSE_FALLBACK}"
  adapter_pointcloud_transform_profile:="${POINTCLOUD_TRANSFORM_PROFILE}"
  livox_custom_bridge:="${LIVOX_CUSTOM_BRIDGE}"
  livox_custom_topic:="${LIVOX_CUSTOM_TOPIC}"
  livox_pointcloud_topic:="${LIVOX_POINTCLOUD_TOPIC}"
  publish_tf:="${PUBLISH_TF}"
  require_depth_topic:="${REQUIRE_DEPTH_TOPIC}"
  render_mode:="${RENDER_MODE}"
)
if [[ "${LIVOX_CUSTOM_BRIDGE}" == "true" ]]; then
  launch_args+=(adapter_raw_pointcloud_topic:="${LIVOX_POINTCLOUD_TOPIC}")
fi

if [[ -n "${CONFIG_PATH}" ]]; then
  launch_args+=(config:="${CONFIG_PATH}")
fi

if [[ -n "${BAG_PATH}" ]]; then
  launch_args+=(
    bag:="${BAG_PATH}"
    play_bag:=false
    loop_bag:=false
    synthetic_input:=false
    use_sim_time:=true
  )
else
  launch_args+=(
    synthetic_input:=true
    use_sim_time:=false
  )
fi

if [[ "${ENABLE_TORCH}" == "true" ]]; then
  torch_opt_enabled=false
  if [[ "${TORCH_OPTIMIZATION_STEPS}" -gt 0 ]]; then
    torch_opt_enabled=true
  fi
  torch_prune_enabled=false
  if [[ "${TORCH_MAX_FOREGROUND}" -gt 0 || "${TORCH_PRUNE_MIN_OPACITY}" != "0.005" ]]; then
    torch_prune_enabled=true
  fi
  launch_args+=(
    enable_torch_camera_conversion:=true
    enable_torch_gaussian_init:=true
    enable_torch_gaussian_optimization:="${torch_opt_enabled}"
    torch_gaussian_optimization_steps:="${TORCH_OPTIMIZATION_STEPS}"
    enable_torch_gaussian_pruning:="${torch_prune_enabled}"
    torch_gaussian_prune_min_opacity:="${TORCH_PRUNE_MIN_OPACITY}"
    torch_gaussian_max_foreground:="${TORCH_MAX_FOREGROUND}"
    torch_gaussian_device:=cpu
  )
fi

if [[ -n "${SENSOR_QOS_RELIABILITY}" ]]; then
  launch_args+=(sensor_qos_reliability:="${SENSOR_QOS_RELIABILITY}")
fi

setsid ros2 launch gaussian_lic_bringup run_bag.launch.py "${launch_args[@]}" \
  >"${RUN_LOG}" 2>&1 &
LAUNCH_PID=$!
PLAY_PID=""
RECORD_PID=""

cleanup() {
  if [[ -n "${RECORD_PID}" ]]; then
    kill -INT "${RECORD_PID}" >/dev/null 2>&1 || true
  fi
  if [[ -n "${PLAY_PID}" ]]; then
    local play_pgid="-${PLAY_PID}"
    kill -INT "${play_pgid}" >/dev/null 2>&1 || true
    sleep 1
    kill -TERM "${play_pgid}" >/dev/null 2>&1 || true
    sleep 1
    kill -KILL "${play_pgid}" >/dev/null 2>&1 || true
  fi
  local pgid="-${LAUNCH_PID}"
  kill -INT "${pgid}" >/dev/null 2>&1 || true
  sleep 1
  kill -TERM "${pgid}" >/dev/null 2>&1 || true
  sleep 1
  kill -KILL "${pgid}" >/dev/null 2>&1 || true
  wait "${LAUNCH_PID}" >/dev/null 2>&1 || true
}
trap cleanup EXIT

record_topics=(
  /gaussian_lic/odometry
  /gaussian_lic/path
  /gaussian_lic/map_points
  /gaussian_lic/rendered_image
  /gaussian_lic/status
)
if [[ "${ENABLE_TORCH}" == "true" ]]; then
  record_topics+=(/gaussian_lic/gaussian_map)
fi
if [[ "${PUBLISH_TF}" == "true" ]]; then
  record_topics+=(/tf)
fi

wait_for_node() {
  local node_name="$1"
  local deadline=$((SECONDS + TIMEOUT_SEC))
  while (( SECONDS < deadline )); do
    if ros2 node list 2>/dev/null | grep -Fxq "${node_name}"; then
      return 0
    fi
    sleep 0.2
  done
  echo "Timed out waiting for ROS2 node ${node_name}" >&2
  return 1
}

start_recording() {
  echo "[current] recording ROS2 mapper outputs: ${RECORDED_BAG}"
  timeout --signal=INT --kill-after=5 "${RECORD_SEC}" \
    ros2 bag record \
      -o "${RECORDED_BAG}" \
      "${record_topics[@]}" \
      >"${OUTPUT_DIR}/record.log" 2>&1 &
  RECORD_PID=$!
}

wait_for_recording() {
  local record_status=0
  set +e
  wait "${RECORD_PID}"
  record_status=$?
  set -e
  RECORD_PID=""
  if [[ ${record_status} -ne 0 && ${record_status} -ne 124 && ${record_status} -ne 130 ]]; then
    echo "ros2 bag record failed with status ${record_status}; see ${OUTPUT_DIR}/record.log" >&2
    exit "${record_status}"
  fi
}

if [[ -n "${BAG_PATH}" ]]; then
  echo "[current] waiting for mapper nodes"
  wait_for_node "/mapping_node"
  if [[ "${FRONTEND_ADAPTER}" == "true" ]]; then
    wait_for_node "/lic2_contract_adapter"
  fi
  start_recording
  sleep 0.5
  echo "[current] replaying input bag: ${BAG_PATH}"
  setsid ros2 bag play "${BAG_PATH}" --clock --loop >"${OUTPUT_DIR}/play.log" 2>&1 &
  PLAY_PID=$!
  echo "[current] waiting for mapper status"
  ros2 topic echo --once --timeout "${TIMEOUT_SEC}" \
    /gaussian_lic/status gaussian_lic_msgs/msg/MappingStatus \
    >"${OUTPUT_DIR}/status.txt"
  wait_for_recording
else
  echo "[current] waiting for mapper status"
  ros2 topic echo --once --timeout "${TIMEOUT_SEC}" \
    /gaussian_lic/status gaussian_lic_msgs/msg/MappingStatus \
    >"${OUTPUT_DIR}/status.txt"
  start_recording
  wait_for_recording
fi

echo "[current] saving mapper point cloud"
rm -rf "${SAVED_MAP_DIR}"
timeout "${TIMEOUT_SEC}" \
  ros2 service call /gaussian_lic/save_map gaussian_lic_msgs/srv/SaveMap \
    "{path: ${SAVED_MAP_DIR}, include_skybox: false}" \
    >"${OUTPUT_DIR}/save_map.log"
test -f "${SAVED_MAP_DIR}/point_cloud.ply"

cleanup
trap - EXIT

echo "[current] extracting trajectory and debug cloud metrics"
ros2 run gaussian_lic_tools gaussian_lic_offline \
  --bag "${RECORDED_BAG}" \
  --output "${OUTPUT_DIR}/offline" \
  --pose-topic /gaussian_lic/odometry \
  --pointcloud-topic /gaussian_lic/map_points \
  >"${OUTPUT_DIR}/offline_stdout.json"

cp "${OUTPUT_DIR}/offline/trajectory.tum" "${OUTPUT_DIR}/trajectory.tum"
cp "${SAVED_MAP_DIR}/point_cloud.ply" "${OUTPUT_DIR}/point_cloud.ply"

python3 - "${OUTPUT_DIR}" "${BAG_PATH}" "${RENDER_MODE}" "${ENABLE_TORCH}" "${FRONTEND_ADAPTER}" "${RECORD_SEC}" "${TORCH_OPTIMIZATION_STEPS}" "${IMU_POSE_FALLBACK}" "${TORCH_MAX_FOREGROUND}" "${TORCH_PRUNE_MIN_OPACITY}" "${POINTCLOUD_TRANSFORM_PROFILE}" <<'PY'
import json
import sys
from pathlib import Path

output = Path(sys.argv[1]).resolve()
bag = sys.argv[2]
metrics_path = output / "offline" / "metrics.json"
metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
metrics.update(
    {
        "schema": "gaussian_lic_current_results/v1",
        "input_bag": bag,
        "render_mode": sys.argv[3],
        "torch_enabled": sys.argv[4] == "true",
        "frontend_adapter": sys.argv[5] == "true",
        "record_sec": float(sys.argv[6]),
        "torch_optimization_steps": int(sys.argv[7]),
        "imu_pose_fallback": sys.argv[8] == "true",
        "torch_max_foreground": int(sys.argv[9]),
        "torch_prune_min_opacity": float(sys.argv[10]),
        "pointcloud_transform_profile": sys.argv[11],
        "saved_map": str((output / "saved_map" / "point_cloud.ply").resolve()),
        "outputs": {
            **metrics.get("outputs", {}),
            "trajectory_tum": str((output / "trajectory.tum").resolve()),
            "point_cloud_ply": str((output / "point_cloud.ply").resolve()),
            "ros2_output_bag": str((output / "ros2_output_bag").resolve()),
            "run_log": str((output / "run.log").resolve()),
        },
    }
)
(output / "metrics.json").write_text(json.dumps(metrics, indent=2, sort_keys=True) + "\n", encoding="utf-8")
PY

test -s "${OUTPUT_DIR}/trajectory.tum"
test -s "${OUTPUT_DIR}/point_cloud.ply"
test -s "${OUTPUT_DIR}/metrics.json"

echo "[current] artifacts: ${OUTPUT_DIR}"
echo "[current] passed"
