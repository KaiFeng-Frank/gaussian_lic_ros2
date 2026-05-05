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
TORCH_PRUNE_COUNT_POLICY=opacity
TORCH_EXTEND_VISIBILITY_FILTER=true
TORCH_EXTEND_ALPHA_THRESHOLD=0.99
TORCH_DEVICE="cpu"
ENABLE_TORCH_DENSIFICATION=false
FINAL_RENDER_EVAL=false
PUBLISH_GAUSSIAN_MAP=true
FRONTEND_ADAPTER=false
IDENTITY_POSE_FALLBACK=false
IMU_POSE_FALLBACK=false
ROTATE_POINTCLOUD_WITH_IMU_POSE=true
SYNC_IMAGE_TO_POINTCLOUD=false
POINTCLOUD_TRANSFORM_PROFILE="identity"
LIVOX_CUSTOM_BRIDGE=false
LIVOX_CUSTOM_TOPIC="/livox/lidar"
LIVOX_POINTCLOUD_TOPIC="/livox/lidar/points"
PUBLISH_TF=false
REQUIRE_DEPTH_TOPIC=true
RENDER_MODE="debug_cpu"
RECORD_SEC=12
TIMEOUT_SEC=20
SAVE_TIMEOUT_SEC=600
SENSOR_QOS_RELIABILITY=""
PLAY_RATE="1"
LOOP_PLAYBACK=false
POST_PLAY_SETTLE_SEC=8

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
  --torch-optimization-steps N Enable Torch photometric updates with up to N accumulated train-frame samples per keyframe.
  --torch-max-foreground N     Enable Torch pruning and retain at most N foreground Gaussians.
  --torch-prune-min-opacity X  Enable Torch pruning and drop foreground Gaussians below opacity X.
  --torch-prune-count-policy P Count-cap policy: opacity or uniform. Default: opacity.
  --no-torch-extend-visibility-filter
                               Append all pending points instead of filtering to current-view alpha holes.
  --torch-extend-alpha-threshold X
                               Alpha threshold for Torch extension visibility filtering. Default: 0.99.
  --torch-device DEVICE        Torch Gaussian device: cpu, cuda, or auto. Default: cpu.
  --torch-densification        Enable gradient-aware Gaussian densification in the Torch backend.
  --final-render-eval          Save final-map train/test renders during SaveMap instead of using live preview frames.
  --no-publish-gaussian-map    Disable high-rate GaussianArray visualization publication/recording.
  --frontend-adapter           Route raw frontend topics through lic2_contract_adapter.
  --identity-pose-fallback     Let the frontend adapter publish identity poses from point-cloud stamps.
  --imu-pose-fallback          Let the frontend adapter integrate IMU gyro orientation for pose fallback.
  --no-rotate-pointcloud-with-imu-pose
                               Keep adapter point clouds in the camera frame when IMU pose fallback is enabled.
  --sync-image-to-pointcloud   Re-stamp latest raw image/camera_info to each point-cloud stamp in the adapter.
  --fastlivo2-camera-lidar-transform
                               Transform raw FAST-LIVO2 LiDAR points into camera frame in the adapter.
  --livox-custom-bridge        Bridge Livox CustomMsg input to PointCloud2 before the adapter.
  --optional-depth             Allow mapper replay without a depth image topic.
  --tf                         Enable TF publication.
  --render-mode MODE           debug_cpu, debug_input, rasterizer, or off. Default: debug_cpu.
  --sensor-qos RELIABILITY     Override input sensor QoS reliability: best_effort or reliable.
  --play-rate RATE             rosbag2 playback rate when --bag is used. Default: 1.
  --loop-playback              Loop rosbag2 playback during recording.
  --post-play-settle SEC       Seconds to keep mapper alive after finite playback. Default: 8.
  --record-sec SEC             Output recording duration. Default: 12.
  --timeout SEC                Topic/service wait timeout. Default: 20.
  --save-timeout SEC           SaveMap/final-render timeout. Default: 600.
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
    --torch-prune-count-policy)
      ENABLE_TORCH=true
      TORCH_PRUNE_COUNT_POLICY="$2"
      shift 2
      ;;
    --no-torch-extend-visibility-filter)
      ENABLE_TORCH=true
      TORCH_EXTEND_VISIBILITY_FILTER=false
      shift
      ;;
    --torch-extend-alpha-threshold)
      ENABLE_TORCH=true
      TORCH_EXTEND_ALPHA_THRESHOLD="$2"
      shift 2
      ;;
    --torch-device)
      TORCH_DEVICE="$2"
      shift 2
      ;;
    --torch-densification)
      ENABLE_TORCH=true
      ENABLE_TORCH_DENSIFICATION=true
      shift
      ;;
    --final-render-eval)
      FINAL_RENDER_EVAL=true
      shift
      ;;
    --no-publish-gaussian-map)
      PUBLISH_GAUSSIAN_MAP=false
      shift
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
    --no-rotate-pointcloud-with-imu-pose)
      ROTATE_POINTCLOUD_WITH_IMU_POSE=false
      shift
      ;;
    --sync-image-to-pointcloud)
      SYNC_IMAGE_TO_POINTCLOUD=true
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
    --play-rate)
      PLAY_RATE="$2"
      shift 2
      ;;
    --loop-playback)
      LOOP_PLAYBACK=true
      shift
      ;;
    --post-play-settle)
      POST_PLAY_SETTLE_SEC="$2"
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
    --save-timeout)
      SAVE_TIMEOUT_SEC="$2"
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
if [[ "${RENDER_MODE}" == "rasterizer" && "${TORCH_DEVICE}" == "cpu" ]]; then
  echo "--render-mode rasterizer requires --torch-device cuda or auto with the strict CUDA build" >&2
  exit 2
fi
if [[ "${ENABLE_TORCH}" == "true" && "${TORCH_DEVICE}" != "cpu" ]]; then
  export PYTORCH_CUDA_ALLOC_CONF="${PYTORCH_CUDA_ALLOC_CONF:-expandable_segments:True}"
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
  adapter_rotate_pointcloud_with_imu_pose:="${ROTATE_POINTCLOUD_WITH_IMU_POSE}"
  adapter_sync_image_to_pointcloud:="${SYNC_IMAGE_TO_POINTCLOUD}"
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
    enable_torch_gaussian_densification:="${ENABLE_TORCH_DENSIFICATION}"
    torch_gaussian_prune_min_opacity:="${TORCH_PRUNE_MIN_OPACITY}"
    torch_gaussian_max_foreground:="${TORCH_MAX_FOREGROUND}"
    torch_gaussian_prune_count_policy:="${TORCH_PRUNE_COUNT_POLICY}"
    enable_torch_gaussian_extend_visibility_filter:="${TORCH_EXTEND_VISIBILITY_FILTER}"
    torch_gaussian_extend_alpha_threshold:="${TORCH_EXTEND_ALPHA_THRESHOLD}"
    torch_gaussian_device:="${TORCH_DEVICE}"
    publish_gaussian_map:="${PUBLISH_GAUSSIAN_MAP}"
    save_map_render_evaluation:="${FINAL_RENDER_EVAL}"
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
if [[ "${ENABLE_TORCH}" == "true" && "${PUBLISH_GAUSSIAN_MAP}" == "true" ]]; then
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

stop_recording() {
  if [[ -n "${RECORD_PID}" ]]; then
    kill -INT "${RECORD_PID}" >/dev/null 2>&1 || true
  fi
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

wait_for_playback() {
  if [[ -z "${PLAY_PID}" ]]; then
    return 0
  fi

  local play_status=0
  set +e
  wait "${PLAY_PID}"
  play_status=$?
  set -e
  PLAY_PID=""
  if [[ ${play_status} -ne 0 && ${play_status} -ne 130 ]]; then
    echo "rosbag2 playback failed with status ${play_status}; see ${OUTPUT_DIR}/play.log" >&2
    exit "${play_status}"
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
  play_args=(--rate "${PLAY_RATE}")
  if [[ "${LOOP_PLAYBACK}" == "true" ]]; then
    play_args+=(--loop)
  fi
  setsid ./scripts/strict_rosbag2_play.sh "${play_args[@]}" "${BAG_PATH}" >"${OUTPUT_DIR}/play.log" 2>&1 &
  PLAY_PID=$!
  echo "[current] waiting for mapper status"
  ros2 topic echo --once --timeout "${TIMEOUT_SEC}" \
    /gaussian_lic/status gaussian_lic_msgs/msg/MappingStatus \
    >"${OUTPUT_DIR}/status.txt"
  if [[ "${LOOP_PLAYBACK}" == "true" ]]; then
    wait_for_recording
  else
    wait_for_playback
    if [[ "${POST_PLAY_SETTLE_SEC}" != "0" && "${POST_PLAY_SETTLE_SEC}" != "0.0" ]]; then
      echo "[current] settling mapper after playback for ${POST_PLAY_SETTLE_SEC}s"
      sleep "${POST_PLAY_SETTLE_SEC}"
    fi
    stop_recording
    wait_for_recording
  fi
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
timeout "${SAVE_TIMEOUT_SEC}" \
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

echo "[current] extracting rendered image pairs"
if [[ -d "${SAVED_MAP_DIR}/renders" ]]; then
  rm -rf "${OUTPUT_DIR}/renders" "${OUTPUT_DIR}/gt" "${OUTPUT_DIR}/render_depth"
  cp -a "${SAVED_MAP_DIR}/renders" "${OUTPUT_DIR}/renders"
  if [[ -d "${SAVED_MAP_DIR}/gt" ]]; then
    cp -a "${SAVED_MAP_DIR}/gt" "${OUTPUT_DIR}/gt"
  fi
  if [[ -d "${SAVED_MAP_DIR}/render_depth" ]]; then
    cp -a "${SAVED_MAP_DIR}/render_depth" "${OUTPUT_DIR}/render_depth"
  fi
  if [[ -f "${SAVED_MAP_DIR}/render_manifest.json" ]]; then
    cp "${SAVED_MAP_DIR}/render_manifest.json" "${OUTPUT_DIR}/render_extract.json"
  else
    printf '{"schema":"gaussian_lic_ros2_final_render/v1","source":"save_map","written":0}\n' \
      >"${OUTPUT_DIR}/render_extract.json"
  fi
else
  /usr/bin/python3 "${ROOT_DIR}/scripts/extract_ros2_rendered_images.py" \
    --bag "${RECORDED_BAG}" \
    --output "${OUTPUT_DIR}/renders" \
    --topic /gaussian_lic/rendered_image \
    --first-name train_0004.jpg \
    --max-images 1 \
    >"${OUTPUT_DIR}/render_extract.json"
fi

cp "${OUTPUT_DIR}/offline/trajectory.tum" "${OUTPUT_DIR}/trajectory.tum"
cp "${SAVED_MAP_DIR}/point_cloud.ply" "${OUTPUT_DIR}/point_cloud.ply"

python3 - "${OUTPUT_DIR}" "${BAG_PATH}" "${RENDER_MODE}" "${ENABLE_TORCH}" "${FRONTEND_ADAPTER}" "${RECORD_SEC}" "${TORCH_OPTIMIZATION_STEPS}" "${IMU_POSE_FALLBACK}" "${TORCH_MAX_FOREGROUND}" "${TORCH_PRUNE_MIN_OPACITY}" "${POINTCLOUD_TRANSFORM_PROFILE}" "${SYNC_IMAGE_TO_POINTCLOUD}" "${PLAY_RATE}" "${LOOP_PLAYBACK}" "${POST_PLAY_SETTLE_SEC}" "${TORCH_DEVICE}" "${FINAL_RENDER_EVAL}" "${ENABLE_TORCH_DENSIFICATION}" "${ROTATE_POINTCLOUD_WITH_IMU_POSE}" "${PUBLISH_GAUSSIAN_MAP}" "${TORCH_PRUNE_COUNT_POLICY}" "${TORCH_EXTEND_VISIBILITY_FILTER}" "${TORCH_EXTEND_ALPHA_THRESHOLD}" "${PYTORCH_CUDA_ALLOC_CONF:-}" <<'PY'
import json
import sys
from pathlib import Path

output = Path(sys.argv[1]).resolve()
bag = sys.argv[2]
metrics_path = output / "offline" / "metrics.json"
metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
render_extract_path = output / "render_extract.json"
render_extract = {}
if render_extract_path.is_file():
    render_extract = json.loads(render_extract_path.read_text(encoding="utf-8"))
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
        "sync_image_to_pointcloud": sys.argv[12] == "true",
        "play_rate": float(sys.argv[13]),
        "loop_playback": sys.argv[14] == "true",
        "post_play_settle_sec": float(sys.argv[15]),
        "torch_device": sys.argv[16],
        "final_render_eval": sys.argv[17] == "true",
        "torch_densification": sys.argv[18] == "true",
        "rotate_pointcloud_with_imu_pose": sys.argv[19] == "true",
        "publish_gaussian_map": sys.argv[20] == "true",
        "torch_prune_count_policy": sys.argv[21],
        "torch_extend_visibility_filter": sys.argv[22] == "true",
        "torch_extend_alpha_threshold": float(sys.argv[23]),
        "pytorch_cuda_alloc_conf": sys.argv[24],
        "render_extract": render_extract,
        "saved_map": str((output / "saved_map" / "point_cloud.ply").resolve()),
        "outputs": {
            **metrics.get("outputs", {}),
            "trajectory_tum": str((output / "trajectory.tum").resolve()),
            "point_cloud_ply": str((output / "point_cloud.ply").resolve()),
            "ros2_output_bag": str((output / "ros2_output_bag").resolve()),
            "renders": str((output / "renders").resolve()),
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
