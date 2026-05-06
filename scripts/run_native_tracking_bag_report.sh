#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
BAG_PATH="/home/frank/data/fast_livo/Bright_Screen_Wall_frontend_raw"
OUTPUT_DIR="${ROOT_DIR}/results/fastlivo2/Bright_Screen_Wall_native_tracking_12s"
PLAYBACK_DURATION=12
PLAYBACK_RATE=1.0
TIMEOUT_SEC=30
MIN_POSES=20
MIN_STATUS_SAMPLES=1
MIN_POINT_FRAMES=10
LIDAR_MIN_POINTS=32
LIDAR_MAX_FRAME_POINTS=4000
LIDAR_MAX_MAP_POINTS=40000
LIDAR_TIME_MODE=auto
LIDAR_SCAN_ORDER_DURATION_S=0.1
LIDAR_TO_IMU_TRANSLATION_M="[0.04165, 0.02326, -0.0284]"
LIDAR_TO_IMU_RPY_RAD="[0.0, 0.0, 0.0]"
CAMERA_TO_IMU_TRANSLATION_M="[0.0673699, 0.0412418, 0.0764217]"
CAMERA_TO_IMU_RPY_RAD="[-1.5768568829, 0.0154178108, -1.5646936365]"
SLIDING_WINDOW_MAX_ITERATIONS=3
SLIDING_WINDOW_MAX_STATE_GAP_S=1.0
SLIDING_WINDOW_MAX_NORMAL_EQUATION_CONDITION=10000000000000.0
REQUIRE_BA_FEEDBACK=false
REQUIRE_NONDEGENERATE_BA=false
REQUIRE_DESKEW=false
ENABLE_VISUAL_FACTORS=false
ENABLE_MAPPER_FEEDBACK=false
VISUAL_DEPTH_FRAME_CACHE_SIZE=64
VISUAL_PENDING_FACTOR_QUEUE_SIZE=128
ENABLE_EXTERNAL_ODOMETRY_PRIOR=false
REFERENCE_ODOMETRY_TOPIC="/gaussian_lic/frontend/input_odometry"
REFERENCE_POSE_TOPIC=""
REQUIRE_REFERENCE_TRAJECTORY=false
MIN_REFERENCE_POSES=10
REFERENCE_TRAJECTORY_ALIGN=first
REFERENCE_MAX_ASSOCIATION_DT=0.2
REFERENCE_MIN_COVERAGE=0.4
REFERENCE_MAX_RMSE_M=2.0
REFERENCE_MAX_MEAN_M=1.5
REFERENCE_MAX_ERROR_M=5.0
REFERENCE_MAX_PATH_DRIFT=0.75
EXTERNAL_ODOMETRY_PRIOR_MAX_DT_NS=100000000
EXTERNAL_ODOMETRY_PRIOR_TRANSLATION_WEIGHT=4.0
EXTERNAL_ODOMETRY_PRIOR_ROTATION_WEIGHT=4.0

usage() {
  cat <<'EOF'
Usage: scripts/run_native_tracking_bag_report.sh [OPTIONS]

Run the native ROS2 tracking frontend against a frontend-raw rosbag2 dataset,
record tracking outputs, extract reproducibility artifacts, and gate tracking
health counters. This is a real-bag native tracking evidence path; it does not
replace the strict mapper-contract/CUDA paper gate.

Options:
  --bag DIR                    Frontend-raw rosbag2 directory.
  --output DIR                 Output report directory.
  --playback-duration SEC      rosbag2 playback duration. Default: 12.
  --rate RATE                  rosbag2 playback rate. Default: 1.0.
  --timeout SEC                Topic/report timeout. Default: 30.
  --min-poses N                Minimum recorded frontend odometry poses. Default: 20.
  --min-status-samples N       Minimum TrackingStatus samples. Default: 1.
  --min-point-frames N         Minimum recorded /points_for_gs frames. Default: 10.
  --lidar-min-points N         Tracking LiDAR frame minimum. Default: 32.
  --lidar-max-frame-points N   Max LiDAR points used per frame factor. Default: 4000.
  --lidar-max-map-points N     Max LiDAR map points retained by the native factor. Default: 40000.
  --enable-scan-order-deskew   Use explicit scan-order point timestamps when the bag has no per-point time field.
  --lidar-scan-order-duration-s SEC
                               Scan duration for --enable-scan-order-deskew. Default: 0.1.
  --require-deskew             Require nonzero trajectory deskew queries and hits in the report.
  --lidar-to-imu-translation V  YAML vector for LiDAR->IMU translation. Default: FAST-LIVO2.
  --lidar-to-imu-rpy V          YAML vector for LiDAR->IMU RPY radians. Default: FAST-LIVO2 identity.
  --camera-to-imu-translation V YAML vector for camera->IMU translation. Default: FAST-LIVO2.
  --camera-to-imu-rpy V         YAML vector for camera->IMU RPY radians. Default: FAST-LIVO2.
  --sliding-window-max-iterations N
                               Max BA iterations per solve. Default: 3.
  --sliding-window-max-state-gap-s SEC
                               Max active-window state gap before BA is marked degenerate. Default: 1.0.
  --sliding-window-max-condition C
                               Max normal-equation condition before BA is marked degenerate. Default: 1e13.
  --require-ba-feedback        Require accepted sliding-window feedback.
  --require-nondegenerate-ba   Require the last reported BA normal equation and state cadence to be non-degenerate.
  --enable-visual-factors      Require mapper-rendered-image visual factors to be present externally.
  --enable-mapper-feedback     Launch mapping_node so native tracking can consume mapper rendered-image feedback.
  --enable-external-odometry-prior
                               Feed the reference odometry topic into tracking BA as an optional pose prior.
  --reference-odometry-topic T Topic to record as reference TUM trajectory. Default: /gaussian_lic/frontend/input_odometry.
  --reference-pose-topic T     Optional PoseStamped reference trajectory topic.
  --require-reference-trajectory
                               Require reference trajectory samples and trajectory_compare.py gate.
  --min-reference-poses N      Minimum reference poses when required. Default: 10.
  --reference-trajectory-align none|first
                               Alignment mode passed to trajectory_compare.py. Default: first.
  --reference-max-association-dt SEC
                               Max timestamp association delta for reference comparison. Default: 0.2.
  --reference-min-coverage R   Minimum reference trajectory coverage. Default: 0.4.
  --reference-max-rmse-m M     Max native-vs-reference translation RMSE. Default: 2.0.
  --reference-max-mean-m M     Max native-vs-reference translation mean error. Default: 1.5.
  --reference-max-error-m M    Max native-vs-reference translation max error. Default: 5.0.
  --reference-max-path-drift R Max relative path-length drift. Default: 0.75.
  --external-odometry-prior-max-dt-ns NS
                               Freshness window for optional external odometry BA prior. Default: 100000000.
  --external-odometry-prior-translation-weight W
                               Optional external odometry prior translation weight. Default: 4.0.
  --external-odometry-prior-rotation-weight W
                               Optional external odometry prior rotation weight. Default: 4.0.
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
    --playback-duration)
      PLAYBACK_DURATION="$2"
      shift 2
      ;;
    --rate)
      PLAYBACK_RATE="$2"
      shift 2
      ;;
    --timeout)
      TIMEOUT_SEC="$2"
      shift 2
      ;;
    --min-poses)
      MIN_POSES="$2"
      shift 2
      ;;
    --min-status-samples)
      MIN_STATUS_SAMPLES="$2"
      shift 2
      ;;
    --min-point-frames)
      MIN_POINT_FRAMES="$2"
      shift 2
      ;;
    --lidar-min-points)
      LIDAR_MIN_POINTS="$2"
      shift 2
      ;;
    --lidar-max-frame-points)
      LIDAR_MAX_FRAME_POINTS="$2"
      shift 2
      ;;
    --lidar-max-map-points)
      LIDAR_MAX_MAP_POINTS="$2"
      shift 2
      ;;
    --enable-scan-order-deskew)
      LIDAR_TIME_MODE=scan_order
      shift
      ;;
    --lidar-scan-order-duration-s)
      LIDAR_SCAN_ORDER_DURATION_S="$2"
      shift 2
      ;;
    --require-deskew)
      REQUIRE_DESKEW=true
      shift
      ;;
    --lidar-to-imu-translation)
      LIDAR_TO_IMU_TRANSLATION_M="$2"
      shift 2
      ;;
    --lidar-to-imu-rpy)
      LIDAR_TO_IMU_RPY_RAD="$2"
      shift 2
      ;;
    --camera-to-imu-translation)
      CAMERA_TO_IMU_TRANSLATION_M="$2"
      shift 2
      ;;
    --camera-to-imu-rpy)
      CAMERA_TO_IMU_RPY_RAD="$2"
      shift 2
      ;;
    --sliding-window-max-iterations)
      SLIDING_WINDOW_MAX_ITERATIONS="$2"
      shift 2
      ;;
    --sliding-window-max-state-gap-s)
      SLIDING_WINDOW_MAX_STATE_GAP_S="$2"
      shift 2
      ;;
    --sliding-window-max-condition)
      SLIDING_WINDOW_MAX_NORMAL_EQUATION_CONDITION="$2"
      shift 2
      ;;
    --require-ba-feedback)
      REQUIRE_BA_FEEDBACK=true
      shift
      ;;
    --require-nondegenerate-ba)
      REQUIRE_NONDEGENERATE_BA=true
      shift
      ;;
    --enable-visual-factors)
      ENABLE_VISUAL_FACTORS=true
      shift
      ;;
    --enable-mapper-feedback)
      ENABLE_MAPPER_FEEDBACK=true
      ENABLE_VISUAL_FACTORS=true
      shift
      ;;
    --enable-external-odometry-prior)
      ENABLE_EXTERNAL_ODOMETRY_PRIOR=true
      shift
      ;;
    --reference-odometry-topic)
      REFERENCE_ODOMETRY_TOPIC="$2"
      shift 2
      ;;
    --reference-pose-topic)
      REFERENCE_POSE_TOPIC="$2"
      shift 2
      ;;
    --require-reference-trajectory)
      REQUIRE_REFERENCE_TRAJECTORY=true
      shift
      ;;
    --min-reference-poses)
      MIN_REFERENCE_POSES="$2"
      shift 2
      ;;
    --reference-trajectory-align)
      REFERENCE_TRAJECTORY_ALIGN="$2"
      shift 2
      ;;
    --reference-max-association-dt)
      REFERENCE_MAX_ASSOCIATION_DT="$2"
      shift 2
      ;;
    --reference-min-coverage)
      REFERENCE_MIN_COVERAGE="$2"
      shift 2
      ;;
    --reference-max-rmse-m)
      REFERENCE_MAX_RMSE_M="$2"
      shift 2
      ;;
    --reference-max-mean-m)
      REFERENCE_MAX_MEAN_M="$2"
      shift 2
      ;;
    --reference-max-error-m)
      REFERENCE_MAX_ERROR_M="$2"
      shift 2
      ;;
    --reference-max-path-drift)
      REFERENCE_MAX_PATH_DRIFT="$2"
      shift 2
      ;;
    --external-odometry-prior-max-dt-ns)
      EXTERNAL_ODOMETRY_PRIOR_MAX_DT_NS="$2"
      shift 2
      ;;
    --external-odometry-prior-translation-weight)
      EXTERNAL_ODOMETRY_PRIOR_TRANSLATION_WEIGHT="$2"
      shift 2
      ;;
    --external-odometry-prior-rotation-weight)
      EXTERNAL_ODOMETRY_PRIOR_ROTATION_WEIGHT="$2"
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

if [[ ! -f "${BAG_PATH}/metadata.yaml" ]]; then
  echo "frontend-raw bag metadata not found: ${BAG_PATH}" >&2
  exit 2
fi

if [[ -n "${GAUSSIAN_LIC_NATIVE_TRACKING_ROS_DOMAIN_ID:-}" ]]; then
  export ROS_DOMAIN_ID="${GAUSSIAN_LIC_NATIVE_TRACKING_ROS_DOMAIN_ID}"
elif [[ -z "${ROS_DOMAIN_ID:-}" ]]; then
  export ROS_DOMAIN_ID="$((170 + (BASHPID % 40)))"
fi

cd "${ROOT_DIR}"
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source install/setup.bash
set -u

OUTPUT_DIR="$(realpath -m "${OUTPUT_DIR}")"
ARTIFACT_DIR="${OUTPUT_DIR}/artifacts"
REPORT_JSON="${OUTPUT_DIR}/native_tracking_report.json"
LOG_DIR="${OUTPUT_DIR}/logs"
rm -rf "${OUTPUT_DIR}"
mkdir -p "${LOG_DIR}" "${ARTIFACT_DIR}"

launch_log="${LOG_DIR}/tracking_launch.log"
mapper_log="${LOG_DIR}/mapping_feedback.log"
play_log="${LOG_DIR}/rosbag_play.log"
recorder_log="${LOG_DIR}/native_tracking_recorder.log"

stop_process_group() {
  local pid="$1"
  local signal_name="$2"
  if [[ -z "${pid}" ]]; then
    return
  fi
  kill "-${signal_name}" "${pid}" 2>/dev/null || true
  kill "-${signal_name}" -- -"${pid}" 2>/dev/null || true
  for _ in $(seq 1 200); do
    if ! kill -0 "${pid}" 2>/dev/null; then
      wait "${pid}" 2>/dev/null || true
      return
    fi
    sleep 0.1
  done
  kill -KILL -- -"${pid}" 2>/dev/null || kill -KILL "${pid}" 2>/dev/null || true
  wait "${pid}" 2>/dev/null || true
}

cleanup() {
  if [[ -n "${play_pid:-}" ]]; then
    kill "${play_pid}" 2>/dev/null || true
    wait "${play_pid}" 2>/dev/null || true
  fi
  if [[ -n "${record_pid:-}" ]]; then
    stop_process_group "${record_pid}" TERM
  fi
  if [[ -n "${mapper_pid:-}" ]]; then
    stop_process_group "${mapper_pid}" TERM
  fi
  if [[ -n "${launch_pid:-}" ]]; then
    stop_process_group "${launch_pid}" TERM
  fi
}
trap cleanup EXIT

setsid ros2 launch gaussian_lic_bringup tracking.launch.py \
  enable_sliding_window_optimizer:=true \
  enable_lidar_plane_factor:=true \
  enable_visual_factor:="${ENABLE_VISUAL_FACTORS}" \
  enable_visual_alignment_window_factor:="${ENABLE_VISUAL_FACTORS}" \
  enable_se3_photometric_window_factor:="${ENABLE_VISUAL_FACTORS}" \
  depth_frame_cache_size:="${VISUAL_DEPTH_FRAME_CACHE_SIZE}" \
  rendered_frame_cache_size:=64 \
  observed_frame_cache_size:=128 \
  visual_pending_factor_queue_size:="${VISUAL_PENDING_FACTOR_QUEUE_SIZE}" \
  enable_external_odometry_prior:="${ENABLE_EXTERNAL_ODOMETRY_PRIOR}" \
  external_odometry_prior_topic:="${REFERENCE_ODOMETRY_TOPIC}" \
  external_odometry_prior_max_dt_ns:="${EXTERNAL_ODOMETRY_PRIOR_MAX_DT_NS}" \
  external_odometry_prior_translation_weight:="${EXTERNAL_ODOMETRY_PRIOR_TRANSLATION_WEIGHT}" \
  external_odometry_prior_rotation_weight:="${EXTERNAL_ODOMETRY_PRIOR_ROTATION_WEIGHT}" \
  lidar_min_points:="${LIDAR_MIN_POINTS}" \
  lidar_keyframe_translation_m:=0.0 \
  lidar_to_imu_translation_m:="${LIDAR_TO_IMU_TRANSLATION_M}" \
  lidar_to_imu_rpy_rad:="${LIDAR_TO_IMU_RPY_RAD}" \
  camera_to_imu_translation_m:="${CAMERA_TO_IMU_TRANSLATION_M}" \
  camera_to_imu_rpy_rad:="${CAMERA_TO_IMU_RPY_RAD}" \
  lidar_time_mode:="${LIDAR_TIME_MODE}" \
  lidar_scan_order_duration_s:="${LIDAR_SCAN_ORDER_DURATION_S}" \
  lidar_nearest_distance_m:=1.0 \
  lidar_max_frame_points:="${LIDAR_MAX_FRAME_POINTS}" \
  lidar_max_map_points:="${LIDAR_MAX_MAP_POINTS}" \
  sliding_window_max_iterations:="${SLIDING_WINDOW_MAX_ITERATIONS}" \
  sliding_window_max_state_gap_s:="${SLIDING_WINDOW_MAX_STATE_GAP_S}" \
  sliding_window_max_normal_equation_condition:="${SLIDING_WINDOW_MAX_NORMAL_EQUATION_CONDITION}" \
  >"${launch_log}" 2>&1 &
launch_pid=$!

if [[ "${ENABLE_MAPPER_FEEDBACK}" == "true" ]]; then
  setsid ros2 run gaussian_lic_mapping mapping_node \
    --ros-args \
    --params-file "${ROOT_DIR}/src/gaussian_lic_bringup/config/default.yaml" \
    -p use_sim_time:=true \
    -p render_mode:=debug_input \
    -p require_depth_topic:=false \
    -p publish_gaussian_map:=false \
    -p enable_torch_camera_conversion:=false \
    -p enable_torch_gaussian_init:=false \
    -p enable_torch_gaussian_extend:=false \
    -p enable_torch_gaussian_optimization:=false \
    -p enable_torch_gaussian_pruning:=false \
    -p enable_torch_gaussian_densification:=false \
    -p torch_gaussian_device:=cpu \
    >"${mapper_log}" 2>&1 &
  mapper_pid=$!
fi

sleep 2

recorder_args=(
  --ros-args
  -p output_dir:="${ARTIFACT_DIR}"
  -p odometry_topic:=/gaussian_lic/frontend/odometry
  -p pointcloud_topic:=/points_for_gs
  -p status_topic:=/gaussian_lic/frontend/status
)
if [[ -n "${REFERENCE_ODOMETRY_TOPIC}" ]]; then
  recorder_args+=(-p reference_odometry_topic:="${REFERENCE_ODOMETRY_TOPIC}")
fi
if [[ -n "${REFERENCE_POSE_TOPIC}" ]]; then
  recorder_args+=(-p reference_pose_topic:="${REFERENCE_POSE_TOPIC}")
fi

setsid ros2 run gaussian_lic_tools native_tracking_recorder \
  "${recorder_args[@]}" \
  >"${recorder_log}" 2>&1 &
record_pid=$!

sleep 2

ros2 bag play "${BAG_PATH}" \
  --clock \
  --rate "${PLAYBACK_RATE}" \
  --read-ahead-queue-size 100 \
  --playback-duration "${PLAYBACK_DURATION}" \
  --disable-keyboard-controls \
  >"${play_log}" 2>&1 &
play_pid=$!

play_timeout="$(
  python3 - "${PLAYBACK_DURATION}" "${TIMEOUT_SEC}" <<'PY'
import math
import sys
print(int(math.ceil(float(sys.argv[1]) + float(sys.argv[2]))))
PY
)"
if ! timeout "${play_timeout}" tail --pid="${play_pid}" -f /dev/null; then
  echo "rosbag2 playback did not finish before timeout" >&2
  exit 1
fi
wait "${play_pid}"
unset play_pid

sleep 2
stop_process_group "${record_pid}" TERM
unset record_pid

stop_process_group "${launch_pid}" TERM
unset launch_pid

python3 - "${ARTIFACT_DIR}/metrics.json" "${REPORT_JSON}" \
  "${MIN_POSES}" "${MIN_STATUS_SAMPLES}" "${MIN_POINT_FRAMES}" "${REQUIRE_BA_FEEDBACK}" \
  "${REQUIRE_REFERENCE_TRAJECTORY}" "${MIN_REFERENCE_POSES}" "${REQUIRE_NONDEGENERATE_BA}" \
  "${ENABLE_VISUAL_FACTORS}" "${REQUIRE_DESKEW}" "${VISUAL_PENDING_FACTOR_QUEUE_SIZE}" <<'PY'
import json
import sys
from pathlib import Path

metrics_path = Path(sys.argv[1])
report_path = Path(sys.argv[2])
min_poses = int(sys.argv[3])
min_status = int(sys.argv[4])
min_point_frames = int(sys.argv[5])
require_ba_feedback = sys.argv[6].lower() == "true"
require_reference_trajectory = sys.argv[7].lower() == "true"
min_reference_poses = int(sys.argv[8])
require_nondegenerate_ba = sys.argv[9].lower() == "true"
enable_visual_factors = sys.argv[10].lower() == "true"
require_deskew = sys.argv[11].lower() == "true"
visual_pending_factor_queue_size = int(sys.argv[12])

metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
topic_counts = metrics.get("topic_counts", {})
status = metrics.get("tracking_status", {})
last = status.get("last") or {}
errors = []

if metrics.get("trajectory_poses", 0) < min_poses:
    errors.append(f"trajectory poses {metrics.get('trajectory_poses', 0)} < {min_poses}")
if status.get("samples", 0) < min_status:
    errors.append(f"tracking status samples {status.get('samples', 0)} < {min_status}")
if topic_counts.get("/points_for_gs", 0) < min_point_frames:
    errors.append(f"/points_for_gs frames {topic_counts.get('/points_for_gs', 0)} < {min_point_frames}")
if require_reference_trajectory and metrics.get("reference_trajectory_poses", 0) < min_reference_poses:
    errors.append(
        f"reference trajectory poses {metrics.get('reference_trajectory_poses', 0)} < {min_reference_poses}")

for key in (
    "image_stamp_regressions",
    "pointcloud_stamp_regressions",
    "imu_stamp_regressions",
    "external_odometry_prior_stamp_regressions",
    "imu_invalid_measurements",
    "external_odometry_prior_invalid_messages",
    "lidar_invalid_frames",
    "lidar_invalid_points",
    "lidar_invalid_point_times",
    "lidar_out_of_range_point_times",
    "pointcloud_imu_wait_dropped",
    "sliding_window_invalid_candidate_steps",
    "sliding_window_linearization_failure_count",
    "sliding_window_linear_solve_failure_count",
    "sliding_window_invalid_optimized_states",
    "sliding_window_numeric_jacobian_blocks",
    "sliding_window_numeric_jacobian_columns",
    "sliding_window_orphan_factors",
):
    if int(last.get(key, 0)) != 0:
        errors.append(f"{key} is {last.get(key)}")

if int(last.get("sliding_window_normal_equation_rows", 0)) <= 0:
    errors.append("sliding_window_normal_equation_rows is zero")
if int(last.get("sliding_window_total_imu_factors", 0)) <= 0:
    errors.append("sliding_window_total_imu_factors is zero")
if int(last.get("sliding_window_point_factors", 0)) <= 0:
    errors.append("sliding_window_point_factors is zero")
if int(last.get("sliding_window_smoothness_factors", 0)) <= 0:
    errors.append("sliding_window_smoothness_factors is zero")
if require_ba_feedback and int(last.get("sliding_window_feedback_updates", 0)) <= 0:
    errors.append("sliding_window_feedback_updates is zero")
if require_ba_feedback and int(last.get("sliding_window_accepted_steps", 0)) <= 0:
    errors.append("sliding_window_accepted_steps is zero")
if require_nondegenerate_ba and bool(last.get("sliding_window_normal_equation_degenerate", False)):
    errors.append("sliding_window_normal_equation_degenerate is true")
if require_nondegenerate_ba and bool(last.get("sliding_window_state_gap_degenerate", False)):
    errors.append("sliding_window_state_gap_degenerate is true")
if require_nondegenerate_ba and int(last.get("sliding_window_accepted_steps", 0)) <= 0:
    errors.append("sliding_window_accepted_steps is zero")
if require_nondegenerate_ba and int(last.get("sliding_window_imu_factor_skip_count", 0)) != 0:
    errors.append(f"sliding_window_imu_factor_skip_count is {last.get('sliding_window_imu_factor_skip_count')}")
if require_nondegenerate_ba and int(last.get("sliding_window_imu_time_gap_skip_count", 0)) != 0:
    errors.append(
        f"sliding_window_imu_time_gap_skip_count is {last.get('sliding_window_imu_time_gap_skip_count')}")
if enable_visual_factors and int(last.get("sliding_window_total_visual_factors", 0)) <= 0:
    errors.append("sliding_window_total_visual_factors is zero")
if enable_visual_factors and int(last.get("sliding_window_total_se3_photometric_factors", 0)) <= 0:
    errors.append("sliding_window_total_se3_photometric_factors is zero")
if enable_visual_factors:
    for key in (
        "visual_alignment_pending_queue_size",
        "visual_se3_photometric_pending_queue_size",
        "visual_alignment_pending_stale_drops",
        "visual_se3_photometric_pending_stale_drops",
    ):
        if key not in last:
            errors.append(f"{key} is missing")
    for key in ("visual_alignment_pending_queue_size", "visual_se3_photometric_pending_queue_size"):
        if int(last.get(key, 0)) > visual_pending_factor_queue_size:
            errors.append(f"{key} exceeds report queue budget: {last.get(key)}")
    for key in ("visual_alignment_pending_stale_drops", "visual_se3_photometric_pending_stale_drops"):
        if int(last.get(key, 0)) != 0:
            errors.append(f"{key} is {last.get(key)}")
if (
    enable_visual_factors
    and int(last.get("sliding_window_total_visual_factors", 0)) <= 0
    and not bool(last.get("visual_photometric_valid", False))
):
    errors.append("visual_photometric_valid is false")
if (
    enable_visual_factors
    and int(last.get("sliding_window_total_se3_photometric_factors", 0)) <= 0
    and not bool(last.get("visual_se3_photometric_valid", False))
):
    errors.append("visual_se3_photometric_valid is false")
if require_deskew and int(last.get("trajectory_deskew_queries", 0)) <= 0:
    errors.append("trajectory_deskew_queries is zero")
if require_deskew and int(last.get("trajectory_deskew_hits", 0)) <= 0:
    errors.append("trajectory_deskew_hits is zero")

report = {
    "ok": not errors,
    "errors": errors,
    "metrics": metrics,
}
report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
if errors:
    print(json.dumps(report, indent=2, sort_keys=True), file=sys.stderr)
    raise SystemExit(1)
print(
    "native tracking report OK: "
    f"poses={metrics.get('trajectory_poses', 0)} "
    f"points_frames={topic_counts.get('/points_for_gs', 0)} "
    f"status_samples={status.get('samples', 0)} "
    f"reference_poses={metrics.get('reference_trajectory_poses', 0)} "
    f"imu_factors={last.get('sliding_window_total_imu_factors', 0)}"
)
PY

COMPARE_JSON="${OUTPUT_DIR}/native_tracking_trajectory_compare.json"
REFERENCE_TUM="${ARTIFACT_DIR}/reference_trajectory.tum"
CURRENT_TUM="${ARTIFACT_DIR}/trajectory.tum"
if [[ -s "${REFERENCE_TUM}" && -s "${CURRENT_TUM}" ]]; then
  compare_cmd=(
    "${ROOT_DIR}/scripts/trajectory_compare.py"
    --baseline "${REFERENCE_TUM}"
    --current "${CURRENT_TUM}"
    --output "${COMPARE_JSON}"
    --align "${REFERENCE_TRAJECTORY_ALIGN}"
    --max-association-dt "${REFERENCE_MAX_ASSOCIATION_DT}"
    --min-matches "${MIN_REFERENCE_POSES}"
    --min-coverage "${REFERENCE_MIN_COVERAGE}"
    --max-rmse-m "${REFERENCE_MAX_RMSE_M}"
    --max-mean-m "${REFERENCE_MAX_MEAN_M}"
    --max-error-m "${REFERENCE_MAX_ERROR_M}"
    --max-path-drift "${REFERENCE_MAX_PATH_DRIFT}"
  )
  if [[ "${REQUIRE_REFERENCE_TRAJECTORY}" == "true" ]]; then
    "${compare_cmd[@]}"
  else
    set +e
    "${compare_cmd[@]}"
    compare_status=$?
    set -e
    if [[ "${compare_status}" -ne 0 ]]; then
      echo "[native-tracking] reference trajectory comparison did not meet thresholds; report kept non-gating at ${COMPARE_JSON}" >&2
    fi
  fi

  python3 - "${REPORT_JSON}" "${COMPARE_JSON}" <<'PY'
import json
import sys
from pathlib import Path

report_path = Path(sys.argv[1])
compare_path = Path(sys.argv[2])
report = json.loads(report_path.read_text(encoding="utf-8"))
if compare_path.exists():
    report["trajectory_compare"] = json.loads(compare_path.read_text(encoding="utf-8"))
report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
PY
elif [[ "${REQUIRE_REFERENCE_TRAJECTORY}" == "true" ]]; then
  echo "required reference/current trajectory file is empty or missing" >&2
  exit 1
fi

echo "[native-tracking] report: ${REPORT_JSON}"
