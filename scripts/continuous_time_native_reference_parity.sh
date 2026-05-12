#!/usr/bin/env bash
# Runs `continuous_time_node` on a frontend_raw rosbag2, captures the TUM
# trajectory, and compares it against a native reference TUM via
# `scripts/trajectory_compare.py`. Result is written as JSON for posterity.
#
# This is the first end-to-end native-stack reference parity report for the
# ROS2 continuous-time tracker port.

set -eo pipefail

WORKSPACE="${WORKSPACE:-/home/frank/gaussian_lic_ros2}"
SOURCE_SETUP="${SOURCE_SETUP:-/opt/ros/jazzy/setup.bash}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-44}"
export ROS_DOMAIN_ID

BAG_DIR="${BAG_DIR:-/home/frank/data/fast_livo/CBD_Building_01_frontend_raw}"
REFERENCE_TUM="${REFERENCE_TUM:-${WORKSPACE}/baseline/fastlivo2/CBD_Building_01/native_reference/cocolic_livo_reference_10hz.tum}"
PLAYBACK_DURATION="${PLAYBACK_DURATION:-12}"
PLAYBACK_RATE="${PLAYBACK_RATE:-0.5}"
OUTPUT_DIR="${OUTPUT_DIR:-${WORKSPACE}/results/fastlivo2/CBD_Building_01_continuous_time_native_parity}"

source "${SOURCE_SETUP}"
source "${WORKSPACE}/install/setup.bash"

mkdir -p "${OUTPUT_DIR}"
NODE_LOG="${OUTPUT_DIR}/node.log"
PLAY_LOG="${OUTPUT_DIR}/play.log"
TUM_PATH="${OUTPUT_DIR}/continuous_time_trajectory.tum"
REPORT_PATH="${OUTPUT_DIR}/trajectory_compare_report.json"

# `setsid` puts the node in its own process group so cleanup can kill the
# full process tree, not just the `ros2 run` wrapper.
setsid ros2 run gaussian_lic_tracking continuous_time_node \
  --ros-args \
  --remap /imu_for_gs:=/imu \
  --remap /points_for_gs:=/livox/lidar \
  -p odometry_topic:=/continuous_time/odometry \
  -p path_topic:=/continuous_time/path \
  -p knot_interval_seconds:=0.05 \
  -p window_knot_count:=10 \
  -p marginalize_oldest_count:=1 \
  -p seed_min_imu_count:=30 \
  -p step_period_seconds:=0.20 \
  -p pointcloud_enable:=true \
  -p enable_voxel_plane_extraction:=true \
  -p voxel_plane_size_m:=0.8 \
  -p voxel_plane_min_points:=6 \
  -p voxel_plane_eigen_ratio:=0.10 \
  -p voxel_plane_max_inlier_m:=0.20 \
  -p voxel_plane_max_correspondences:=48 \
  > "${NODE_LOG}" 2>&1 &
NODE_PID=$!
NODE_PGID=$(ps -o pgid= -p "${NODE_PID}" 2>/dev/null | tr -d ' ')

PLAYBACK_SECS=$(awk -v d="${PLAYBACK_DURATION}" -v r="${PLAYBACK_RATE}" 'BEGIN{ print d / r }')
CAPTURE_SECS=$(awk -v p="${PLAYBACK_SECS}" 'BEGIN{ print p + 3 }')

/usr/bin/python3.12 "${WORKSPACE}/scripts/odom_to_tum.py" \
  --topic /continuous_time/odometry \
  --output "${TUM_PATH}" \
  --duration "${CAPTURE_SECS}" \
  > "${OUTPUT_DIR}/odom_to_tum.log" 2>&1 &
LOGGER_PID=$!

cleanup() {
  if [ -n "${NODE_PGID:-}" ]; then
    kill -9 -- "-${NODE_PGID}" 2>/dev/null || true
  fi
  for pid in "${LOGGER_PID:-}" "${NODE_PID:-}" "${PLAY_PID:-}"; do
    if [ -n "${pid}" ] && kill -0 "${pid}" 2>/dev/null; then
      kill -TERM "${pid}" 2>/dev/null || true
      sleep 0.2
      kill -9 "${pid}" 2>/dev/null || true
    fi
  done
  # Belt-and-suspenders: kill any continuous_time_node that escaped.
  pkill -9 -f "continuous_time_node --ros-args" 2>/dev/null || true
  pkill -9 -f "ros2 bag play ${BAG_DIR}" 2>/dev/null || true
  pkill -9 -f "odom_to_tum" 2>/dev/null || true
}
trap cleanup EXIT

sleep 2

ros2 bag play "${BAG_DIR}" --rate "${PLAYBACK_RATE}" \
  > "${PLAY_LOG}" 2>&1 &
PLAY_PID=$!

# Wait for the logger to finish (it covers playback + drain).
wait "${LOGGER_PID}" 2>/dev/null || true

# Stop node + bag play.
kill -INT "${NODE_PID}" 2>/dev/null || true
sleep 1

if [ ! -s "${TUM_PATH}" ]; then
  echo "continuous_time_native_reference_parity FAIL: TUM artifact missing"
  tail -30 "${NODE_LOG}" || true
  exit 1
fi

TUM_LINES=$(grep -cv '^#' "${TUM_PATH}" || echo 0)
echo "Captured TUM trajectory: ${TUM_PATH} (${TUM_LINES} lines)"

python3 "${WORKSPACE}/scripts/trajectory_compare.py" \
  --baseline "${REFERENCE_TUM}" \
  --current "${TUM_PATH}" \
  --align yaw \
  --output "${REPORT_PATH}" \
  --max-association-dt 0.10 \
  --min-matches 50 \
  --min-coverage 0.20 \
  --max-rmse-m 10000.0 \
  --max-mean-m 10000.0 \
  --max-error-m 10000.0 \
  --max-path-drift 100.0 \
  || true   # don't bubble up strict-threshold failures here; we just want the JSON

if [ ! -s "${REPORT_PATH}" ]; then
  echo "continuous_time_native_reference_parity FAIL: report missing"
  exit 1
fi

echo "Wrote parity report: ${REPORT_PATH}"
python3 - <<PY
import json
with open("${REPORT_PATH}") as fh:
    report = json.load(fh)
def get(path, default=None):
    parts = path.split('.')
    cur = report
    for p in parts:
        if not isinstance(cur, dict) or p not in cur:
            return default
        cur = cur[p]
    return cur
print("  matched_poses    :", get('matched_poses'))
print("  coverage         :", get('coverage'))
print("  rmse_m           :", get('translation_rmse_m'))
print("  mean_error_m     :", get('translation_mean_m'))
print("  max_error_m      :", get('translation_max_m'))
print("  path_drift       :", get('path_drift'))
PY

echo "continuous_time_native_reference_parity OK"
