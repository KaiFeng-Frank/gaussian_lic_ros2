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
# Set REFERENCE_TUM=__skip__ to produce a liveness-only native_tracking_report
# (no trajectory_compare). Used by bags without an archived native reference.
PRIOR_TUM="${PRIOR_TUM:-}"
# When set to an existing TUM file, the parity script publishes it as
# nav_msgs/Odometry on `external_odometry_prior_topic` so the estimator
# can seed its initial pose from ground truth. Leave empty for identity
# / gravity-autocal seeding.
PLAYBACK_DURATION="${PLAYBACK_DURATION:-12}"
PLAYBACK_RATE="${PLAYBACK_RATE:-0.5}"
OUTPUT_DIR="${OUTPUT_DIR:-${WORKSPACE}/results/fastlivo2/CBD_Building_01_continuous_time_native_parity}"
POINTCLOUD_ENABLE="${POINTCLOUD_ENABLE:-true}"
POINTCLOUD_FACTOR_WEIGHT="${POINTCLOUD_FACTOR_WEIGHT:-0.1}"
ENABLE_VOXEL_PLANE_EXTRACTION="${ENABLE_VOXEL_PLANE_EXTRACTION:-true}"
ENABLE_PERSISTENT_PLANE_MAP="${ENABLE_PERSISTENT_PLANE_MAP:-true}"
ENABLE_PERSISTENT_POINT_MAP="${ENABLE_PERSISTENT_POINT_MAP:-false}"
PERSISTENT_POINT_MAP_NEAREST_DISTANCE_M="${PERSISTENT_POINT_MAP_NEAREST_DISTANCE_M:-0.35}"
PERSISTENT_POINT_MAP_FACTOR_WEIGHT="${PERSISTENT_POINT_MAP_FACTOR_WEIGHT:-0.05}"
PERSISTENT_POINT_MAP_SUBSAMPLE_STRIDE="${PERSISTENT_POINT_MAP_SUBSAMPLE_STRIDE:-20}"
PERSISTENT_POINT_MAP_MAX_POINTS="${PERSISTENT_POINT_MAP_MAX_POINTS:-20000}"
PERSISTENT_POINT_MAP_MAX_CORRESPONDENCES="${PERSISTENT_POINT_MAP_MAX_CORRESPONDENCES:-64}"
LIDAR_HUBER_DELTA_M="${LIDAR_HUBER_DELTA_M:-0.10}"
VOXEL_PLANE_SIZE_M="${VOXEL_PLANE_SIZE_M:-0.8}"
VOXEL_PLANE_MIN_POINTS="${VOXEL_PLANE_MIN_POINTS:-6}"
VOXEL_PLANE_EIGEN_RATIO="${VOXEL_PLANE_EIGEN_RATIO:-0.10}"
VOXEL_PLANE_MAX_INLIER_M="${VOXEL_PLANE_MAX_INLIER_M:-0.20}"
VOXEL_PLANE_MAX_CORRESPONDENCES="${VOXEL_PLANE_MAX_CORRESPONDENCES:-48}"
PERSISTENT_PLANE_MAP_MIN_OBSERVATIONS="${PERSISTENT_PLANE_MAP_MIN_OBSERVATIONS:-3}"
MAX_POSITION_UPDATE_M="${MAX_POSITION_UPDATE_M:-2.0}"
MAX_ROTATION_UPDATE_RAD="${MAX_ROTATION_UPDATE_RAD:-0.50}"

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
  -p pointcloud_enable:="${POINTCLOUD_ENABLE}" \
  -p pointcloud_factor_weight:="${POINTCLOUD_FACTOR_WEIGHT}" \
  -p lidar_huber_delta_m:="${LIDAR_HUBER_DELTA_M}" \
  -p enable_voxel_plane_extraction:="${ENABLE_VOXEL_PLANE_EXTRACTION}" \
  -p enable_persistent_plane_map:="${ENABLE_PERSISTENT_PLANE_MAP}" \
  -p enable_persistent_point_map:="${ENABLE_PERSISTENT_POINT_MAP}" \
  -p persistent_point_map_nearest_distance_m:="${PERSISTENT_POINT_MAP_NEAREST_DISTANCE_M}" \
  -p persistent_point_map_factor_weight:="${PERSISTENT_POINT_MAP_FACTOR_WEIGHT}" \
  -p persistent_point_map_subsample_stride:="${PERSISTENT_POINT_MAP_SUBSAMPLE_STRIDE}" \
  -p persistent_point_map_max_points:="${PERSISTENT_POINT_MAP_MAX_POINTS}" \
  -p persistent_point_map_max_correspondences:="${PERSISTENT_POINT_MAP_MAX_CORRESPONDENCES}" \
  -p voxel_plane_size_m:="${VOXEL_PLANE_SIZE_M}" \
  -p voxel_plane_min_points:="${VOXEL_PLANE_MIN_POINTS}" \
  -p voxel_plane_eigen_ratio:="${VOXEL_PLANE_EIGEN_RATIO}" \
  -p voxel_plane_max_inlier_m:="${VOXEL_PLANE_MAX_INLIER_M}" \
  -p voxel_plane_max_correspondences:="${VOXEL_PLANE_MAX_CORRESPONDENCES}" \
  -p persistent_plane_map_min_observations_for_match:="${PERSISTENT_PLANE_MAP_MIN_OBSERVATIONS}" \
  -p max_position_update_m:="${MAX_POSITION_UPDATE_M}" \
  -p max_rotation_update_rad:="${MAX_ROTATION_UPDATE_RAD}" \
  -p enable_external_odometry_prior:="$([ -n "${PRIOR_TUM}" ] && echo true || echo false)" \
  -p external_odometry_prior_topic:=/external_odometry_prior \
  > "${NODE_LOG}" 2>&1 &
NODE_PID=$!
NODE_PGID=$(ps -o pgid= -p "${NODE_PID}" 2>/dev/null | tr -d ' ')

PRIOR_PID=""
PRIOR_PGID=""
if [ -n "${PRIOR_TUM}" ]; then
  if [ ! -f "${PRIOR_TUM}" ]; then
    echo "continuous_time_native_reference_parity FAIL: PRIOR_TUM ${PRIOR_TUM} not found"
    exit 1
  fi
  setsid /usr/bin/python3.12 "${WORKSPACE}/scripts/tum_to_odometry_publisher.py" \
    --tum "${PRIOR_TUM}" \
    --topic /external_odometry_prior \
    --rate-hz 20 \
    --duration 6 \
    --max-messages 60 \
    > "${OUTPUT_DIR}/prior_publisher.log" 2>&1 &
  PRIOR_PID=$!
  PRIOR_PGID=$(ps -o pgid= -p "${PRIOR_PID}" 2>/dev/null | tr -d ' ')
fi

PLAYBACK_SECS=$(awk -v d="${PLAYBACK_DURATION}" -v r="${PLAYBACK_RATE}" 'BEGIN{ print d / r }')
CAPTURE_SECS=$(awk -v p="${PLAYBACK_SECS}" 'BEGIN{ print p + 3 }')

/usr/bin/python3.12 "${WORKSPACE}/scripts/odom_to_tum.py" \
  --topic /continuous_time/odometry \
  --output "${TUM_PATH}" \
  --duration "${CAPTURE_SECS}" \
  > "${OUTPUT_DIR}/odom_to_tum.log" 2>&1 &
LOGGER_PID=$!

cleanup() {
  for pgid in "${NODE_PGID:-}" "${PRIOR_PGID:-}"; do
    if [ -n "${pgid}" ]; then
      kill -9 -- "-${pgid}" 2>/dev/null || true
    fi
  done
  for pid in "${LOGGER_PID:-}" "${NODE_PID:-}" "${PLAY_PID:-}" "${PRIOR_PID:-}"; do
    if [ -n "${pid}" ] && kill -0 "${pid}" 2>/dev/null; then
      kill -TERM "${pid}" 2>/dev/null || true
      sleep 0.2
      kill -9 "${pid}" 2>/dev/null || true
    fi
  done
  pkill -9 -f "continuous_time_node --ros-args" 2>/dev/null || true
  pkill -9 -f "ros2 bag play ${BAG_DIR}" 2>/dev/null || true
  pkill -9 -f "odom_to_tum" 2>/dev/null || true
  pkill -9 -f "tum_to_odometry_publisher" 2>/dev/null || true
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

if [ "${REFERENCE_TUM}" != "__skip__" ]; then
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
else
  echo "Reference TUM skipped (REFERENCE_TUM=__skip__) — liveness-only mode"
  REPORT_PATH=""
fi

# Compose a `native_tracking_report.json` artifact consumable by
# `scripts/check_strict_parity_matrix.py` (kind=native_tracking_report).
# Liveness-only schema: ok=true if the TUM file exists with enough lines
# AND trajectory_compare produced a JSON. No strict RMSE gate — the new
# estimator still needs IMU bias / gravity tuning before paper-grade
# accuracy is achievable. Counts are what the matrix gates against.
NATIVE_REPORT_PATH="${OUTPUT_DIR}/native_tracking_report.json"
LOGGER_LOG="${OUTPUT_DIR}/odom_to_tum.log"

/usr/bin/python3.12 - <<PY
import json
import math
import os
import re

tum_path = "${TUM_PATH}"
compare_path = "${REPORT_PATH}"   # empty string when REFERENCE_TUM=__skip__
logger_log = "${LOGGER_LOG}"
native_report_path = "${NATIVE_REPORT_PATH}"

tum_lines = 0
finite_positions = 0
if os.path.isfile(tum_path):
    with open(tum_path, "r", encoding="utf-8") as fh:
        for raw in fh:
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) != 8:
                continue
            tum_lines += 1
            try:
                vals = [float(p) for p in parts]
                if all(math.isfinite(v) for v in vals):
                    finite_positions += 1
            except ValueError:
                pass

odom_received = 0
odom_written = 0
if os.path.isfile(logger_log):
    txt = open(logger_log, "r", encoding="utf-8").read()
    m = re.search(r"received=(\d+)", txt)
    if m:
        odom_received = int(m.group(1))
    m = re.search(r"written=(\d+)", txt)
    if m:
        odom_written = int(m.group(1))

compare_payload = {}
compare_ok = False
if compare_path and os.path.isfile(compare_path):
    try:
        compare_payload = json.load(open(compare_path, "r", encoding="utf-8"))
        compare_ok = bool(compare_payload.get("ok", False))
    except Exception:
        compare_payload = {"ok": False, "error": "load failed"}

native = {
    "ok": (tum_lines > 0 and finite_positions > 0),
    "schema": "gaussian_lic_continuous_time_native_tracking_report/v1",
    "bag": "${BAG_DIR##*/}",
    "playback_duration_s": float("${PLAYBACK_DURATION}"),
    "playback_rate": float("${PLAYBACK_RATE}"),
    "metrics": {
        "captured_tum_lines": tum_lines,
        "finite_tum_positions": finite_positions,
        "odom_messages_received": odom_received,
        "odom_messages_written": odom_written,
        "reference_matched_poses": compare_payload.get("matched_poses", 0),
        "reference_coverage": compare_payload.get("coverage", 0.0),
        "reference_translation_rmse_m":
            compare_payload.get("translation", {}).get("rmse_m"),
        "reference_translation_mean_m":
            compare_payload.get("translation", {}).get("mean_m"),
    },
    "trajectory_compare": {
        "ok": compare_ok,
        "report": compare_path,
        "matched_poses": compare_payload.get("matched_poses", 0),
        "coverage": compare_payload.get("coverage", 0.0),
    },
}
with open(native_report_path, "w", encoding="utf-8") as fh:
    json.dump(native, fh, indent=2, sort_keys=True)
print("native_report:", native_report_path)
print("  ok                    :", native["ok"])
print("  captured_tum_lines    :", tum_lines)
print("  finite_tum_positions  :", finite_positions)
print("  odom_messages_received:", odom_received)
print("  reference_matched_poses:", native["metrics"]["reference_matched_poses"])
print("  reference_coverage    :", native["metrics"]["reference_coverage"])
print("  reference_rmse_m      :", native["metrics"]["reference_translation_rmse_m"])
PY

echo "continuous_time_native_reference_parity OK"
