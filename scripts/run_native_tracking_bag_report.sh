#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
BAG_PATH="/home/frank/data/fast_livo/Bright_Screen_Wall_frontend_raw"
OUTPUT_DIR="${ROOT_DIR}/results/fastlivo2/Bright_Screen_Wall_native_tracking_12s"
PLAYBACK_DURATION=12
PLAYBACK_RATE=1.0
PLAYBACK_RATE_EXPLICIT=false
PLAYBACK_START_OFFSET=0.0
PLAYBACK_CLOCK_TOPICS_ALL=false
TIMEOUT_SEC=30
POST_PLAY_SETTLE_SEC=8
POST_PLAY_DRAIN_TARGET_POSES=0
POST_PLAY_DRAIN_TARGET_VISUAL_FACTORS=0
POST_PLAY_DRAIN_TARGET_SE3_FACTORS=0
POST_PLAY_DRAIN_TIMEOUT_SEC=0
MIN_POSES=20
MIN_STATUS_SAMPLES=1
MIN_STATUS_BIN_SAMPLE_COUNT=0
MIN_VISUAL_FACTOR_DELTA_PER_STATUS_BIN=0
MIN_SE3_PHOTOMETRIC_FACTOR_DELTA_PER_STATUS_BIN=0
MIN_MOTION_TARGET_DELTA_PER_STATUS_BIN=0
MIN_POINT_FRAMES=10
LIDAR_MIN_POINTS=32
LIDAR_MAX_FRAME_POINTS=4000
LIDAR_MAX_MAP_POINTS=40000
LIDAR_NEAREST_DISTANCE_M=1.0
LIDAR_CORRECTION_GAIN=0.7
LIDAR_MAX_CORRECTION_M=0.25
LIDAR_MAX_ROTATION_RAD=0.08
LIDAR_ROBUST_KERNEL_M=0.15
LIDAR_POSE_FACTOR_ITERATIONS=1
LIDAR_WINDOW_POINT_FACTOR_WEIGHT=1.0
LIDAR_WINDOW_PLANE_FACTOR_WEIGHT=1.0
LIDAR_WINDOW_CONFIDENCE_POWER=1.0
LIDAR_KEYFRAME_TRANSLATION_M=0.0
MAX_LIDAR_INVALID_FRAMES=0
LIDAR_TIME_MODE=auto
LIDAR_SCAN_ORDER_DURATION_S=0.1
RAW_IMU_QOS_RELIABILITY=reliable
RAW_IMU_QOS_DEPTH=2000
RAW_POINTCLOUD_QOS_RELIABILITY=reliable
RAW_POINTCLOUD_QOS_DEPTH=256
POINTCLOUD_IMU_WAIT_QUEUE_SIZE=512
IMU_HISTORY_SIZE=12000
IMU_LINEAR_ACCELERATION_SCALE=9.80665
TRACKING_MAX_POSE_STEP_M=0.25
ENABLE_PRE_LIO_TRACKING_STEP_GUARD=true
ENABLE_POST_BA_TRACKING_STEP_GUARD=true
PRE_LIO_TRACKING_MAX_POSE_STEP_M=0.0
POST_BA_TRACKING_MAX_POSE_STEP_M=0.0
POST_BA_STEP_GUARD_CONFIDENCE_MAX_POSE_STEP_M=0.0
POST_BA_STEP_GUARD_CONFIDENCE_WARMUP_MARGINALIZATIONS=0
POST_BA_STEP_GUARD_MIN_LIDAR_CONFIDENCE=0.6
POST_BA_STEP_GUARD_MIN_VISUAL_INLIER_RATIO=0.85
POST_BA_STEP_GUARD_MAX_VISUAL_RESIDUAL=0.3
POST_BA_STEP_GUARD_MIN_VISUAL_COVERAGE_TILES=8
POST_BA_STEP_GUARD_REJECT_TO_PRE_BA_OVER_M=0.0
POST_BA_STEP_GUARD_PRE_BA_AGREEMENT_MAX_POSE_STEP_M=0.0
POST_BA_STEP_GUARD_PRE_BA_AGREEMENT_LATE_START_MARGINALIZATIONS=0
POST_BA_STEP_GUARD_PRE_BA_AGREEMENT_LATE_MAX_POSE_STEP_M=0.0
POST_BA_STEP_GUARD_PRE_BA_AGREEMENT_MIN_COSINE=0.85
POST_BA_STEP_GUARD_PRE_BA_AGREEMENT_MAX_DELTA_M=0.05
POST_BA_STEP_GUARD_PRE_BA_AGREEMENT_MARGIN_M=0.0
POST_BA_STEP_GUARD_PRE_BA_BLEND_ON_CLAMP=0.0
TRACKING_STEP_GUARD_VELOCITY_SCALE=0.0
PRE_LIO_TRACKING_STEP_GUARD_VELOCITY_SCALE=0.0
POST_BA_TRACKING_STEP_GUARD_VELOCITY_SCALE=0.0
TRACKING_STEP_GUARD_ACCELERATION_MPS2=0.0
TRACKING_STEP_GUARD_MAX_VELOCITY_MPS=0.0
TRACKING_STEP_GUARD_MARGIN_M=0.0
LIDAR_TO_IMU_TRANSLATION_M="[0.04165, 0.02326, -0.0284]"
LIDAR_TO_IMU_RPY_RAD="[0.0, 0.0, 0.0]"
CAMERA_TO_IMU_TRANSLATION_M="[0.0673699, 0.0412418, 0.0764217]"
CAMERA_TO_IMU_RPY_RAD="[-1.5768568829, 0.0154178108, -1.5646936365]"
SLIDING_WINDOW_OPTIMIZE_EVERY_N_FRAMES=1
SLIDING_WINDOW_MAX_STATES=12
SLIDING_WINDOW_MAX_ITERATIONS=3
SLIDING_WINDOW_MAX_STATE_GAP_S=1.0
SLIDING_WINDOW_MARGINALIZATION_PRIOR_WEIGHT=1.0
SLIDING_WINDOW_MAX_NORMAL_EQUATION_CONDITION=10000000000000.0
SLIDING_WINDOW_MAX_TRANSLATION_STEP_M=1.0
SLIDING_WINDOW_MAX_FEEDBACK_TRANSLATION_M=1.0
SLIDING_WINDOW_MAX_FEEDBACK_ROTATION_RAD=0.5
SLIDING_WINDOW_MAX_FEEDBACK_VELOCITY_MPS=5.0
SLIDING_WINDOW_MAX_FEEDBACK_GYRO_BIAS_STEP=0.0
SLIDING_WINDOW_MAX_FEEDBACK_ACCEL_BIAS_STEP=0.0
SLIDING_WINDOW_MAX_FEEDBACK_GYRO_BIAS_STEP_EXPLICIT=false
SLIDING_WINDOW_MAX_FEEDBACK_ACCEL_BIAS_STEP_EXPLICIT=false
SLIDING_WINDOW_MIN_BIAS_FEEDBACK_VISUAL_FACTORS=0
SLIDING_WINDOW_BIAS_FEEDBACK_OWNERSHIP=optimized
SLIDING_WINDOW_SYNC_GUARDED_POSE_STATE=false
SLIDING_WINDOW_GUARDED_POSE_PRIOR_TRANSLATION_WEIGHT=0.0
SLIDING_WINDOW_GUARDED_POSE_PRIOR_ROTATION_WEIGHT=0.0
SLIDING_WINDOW_IMU_WEIGHT=1.0
SLIDING_WINDOW_IMU_ROTATION_WEIGHT=1.0
SLIDING_WINDOW_IMU_VELOCITY_WEIGHT=1.0
SLIDING_WINDOW_IMU_POSITION_WEIGHT=1.0
SLIDING_WINDOW_IMU_VELOCITY_PRIOR_WEIGHT=0.0
SLIDING_WINDOW_GYRO_BIAS_PRIOR_WEIGHT=0.0
SLIDING_WINDOW_ACCEL_BIAS_PRIOR_WEIGHT=0.0
SLIDING_WINDOW_BIAS_WEIGHT=1.0
SLIDING_WINDOW_GYRO_BIAS_WEIGHT=1.0
SLIDING_WINDOW_ACCEL_BIAS_WEIGHT=1.0
SLIDING_WINDOW_BIAS_RANDOM_WALK_REFERENCE_DT_S=0.0
SLIDING_WINDOW_GYRO_BIAS_RANDOM_WALK_SIGMA=0.0
SLIDING_WINDOW_ACCEL_BIAS_RANDOM_WALK_SIGMA=0.0
SLIDING_WINDOW_POSE_TRANSLATION_WEIGHT=2.0
SLIDING_WINDOW_POSE_ROTATION_WEIGHT=2.0
SLIDING_WINDOW_SMOOTHNESS_ROTATION_WEIGHT=0.1
SLIDING_WINDOW_SMOOTHNESS_POSITION_WEIGHT=0.1
SLIDING_WINDOW_SMOOTHNESS_VELOCITY_WEIGHT=0.1
SLIDING_WINDOW_SMOOTHNESS_POSITION_VELOCITY_WEIGHT=0.0
SLIDING_WINDOW_SMOOTHNESS_BIAS_WEIGHT=0.1
SLIDING_WINDOW_SMOOTHNESS_USE_MOTION_TARGETS=false
SLIDING_WINDOW_SMOOTHNESS_MOTION_TARGET_MIN_VISUAL_FACTORS=0
SLIDING_WINDOW_SMOOTHNESS_MOTION_TARGET_MIN_SE3_PHOTOMETRIC_FACTORS=0
SLIDING_WINDOW_SMOOTHNESS_MOTION_TARGET_RECENT_WINDOW=0
SLIDING_WINDOW_SMOOTHNESS_MOTION_TARGET_MIN_RECENT_VISUAL_FACTORS=0
SLIDING_WINDOW_SMOOTHNESS_MOTION_TARGET_MIN_RECENT_SE3_PHOTOMETRIC_FACTORS=0
SLIDING_WINDOW_SMOOTHNESS_MOTION_TARGET_START_AFTER_S=0.0
SLIDING_WINDOW_SMOOTHNESS_MOTION_TARGET_MAX_ROTATION_RATE_DELTA_RADPS=0.25
SLIDING_WINDOW_SMOOTHNESS_MOTION_TARGET_MAX_POSITION_RATE_DELTA_MPS=0.5
SLIDING_WINDOW_SMOOTHNESS_MOTION_TARGET_MAX_VELOCITY_ACCELERATION_DELTA_MPS2=2.0
ENABLE_SLIDING_WINDOW_RELATIVE_TRANSLATION_FACTOR=false
SLIDING_WINDOW_RELATIVE_TRANSLATION_WEIGHT=0.0
SLIDING_WINDOW_RELATIVE_TRANSLATION_HUBER_DELTA_M=0.1
SLIDING_WINDOW_RELATIVE_TRANSLATION_IN_FROM_FRAME=false
SLIDING_WINDOW_RELATIVE_ROTATION_WEIGHT=0.0
SLIDING_WINDOW_RELATIVE_ROTATION_HUBER_DELTA_RAD=0.05
ENABLE_SLIDING_WINDOW_RELATIVE_DISTANCE_FACTOR=false
SLIDING_WINDOW_RELATIVE_DISTANCE_WEIGHT=0.0
SLIDING_WINDOW_RELATIVE_DISTANCE_HUBER_DELTA_M=0.1
ENABLE_SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_FACTOR=false
SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_WEIGHT=0.0
SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_HUBER_DELTA_M=0.15
SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_IN_FROM_FRAME=false
SLIDING_WINDOW_MULTIHOP_RELATIVE_ROTATION_WEIGHT=0.0
SLIDING_WINDOW_MULTIHOP_RELATIVE_ROTATION_HUBER_DELTA_RAD=0.08
ENABLE_SLIDING_WINDOW_MULTIHOP_RELATIVE_DISTANCE_FACTOR=false
SLIDING_WINDOW_MULTIHOP_RELATIVE_DISTANCE_WEIGHT=0.0
SLIDING_WINDOW_MULTIHOP_RELATIVE_DISTANCE_HUBER_DELTA_M=0.15
SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_MIN_DT_S=0.45
SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_MAX_DT_S=1.05
SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_MAX_FACTORS=1
ENABLE_SLIDING_WINDOW_DELAYED_PUBLISHED_MULTIHOP_RELATIVE_TRANSLATION_FACTOR=false
SLIDING_WINDOW_DELAYED_PUBLISHED_MULTIHOP_START_AFTER_S=0.0
SLIDING_WINDOW_DELAYED_PUBLISHED_MULTIHOP_MAX_FACTORS=1
SLIDING_WINDOW_RELATIVE_MOTION_HISTORY_SOURCE=pre_ba
SLIDING_WINDOW_RELATIVE_MOTION_HISTORY_PUBLISHED_AFTER_S=0.0
REQUIRE_BA_FEEDBACK=false
REQUIRE_NONDEGENERATE_BA=false
REQUIRE_DESKEW=false
ENABLE_VISUAL_FACTORS=false
ENABLE_VISUAL_FACTOR_TIME_INTERPOLATION=false
ENABLE_VISUAL_CACHE_RECONCILIATION=false
ENABLE_VISUAL_CACHE_RECONCILIATION_MONOTONIC_UNIQUE=false
ENABLE_VISUAL_PAIR_MONOTONIC_UNIQUE=false
ENABLE_VISUAL_WATERMARK_PAIR_SCHEDULER=false
VISUAL_WATERMARK_PAIR_SCHEDULER_MAX_PAIRS_PER_POINTCLOUD=2
ENABLE_VISUAL_CALLBACK_FACTOR_INGEST=false
ENABLE_RENDERED_FEEDBACK_WATERMARK_QUEUE=false
DEFER_FUTURE_VISUAL_FACTORS_UNTIL_ACTIVE=false
ENABLE_VISUAL_ADAPTIVE_STATE_RETENTION=false
VISUAL_ADAPTIVE_STATE_RETENTION_MARGIN_STATES=4
VISUAL_ADAPTIVE_STATE_RETENTION_MAX_STATES=64
ENABLE_VISUAL_EXPIRED_FACTOR_PROJECTION=false
ENABLE_VISUAL_MARGINALIZATION_PRIOR=false
ENABLE_VISUAL_MARGINALIZATION_PRIOR_BATCHING=false
ENABLE_VISUAL_MARGINALIZATION_PRIOR_SATURATION_GATE=false
VISUAL_MARGINALIZATION_PRIOR_ZERO_BIAS_COLUMNS=false
ENABLE_VISUAL_FACTOR_REFERENCE_SNAPSHOT=false
ENABLE_RENDERED_FEEDBACK_SOURCE_POSE_REFERENCE=false
ENABLE_RENDERED_FEEDBACK_SOURCE_MOTION_FACTOR=false
ENABLE_RENDERED_FEEDBACK_SOURCE_MOTION_MARGINALIZED_PRIOR=false
RENDERED_FEEDBACK_SOURCE_MOTION_TRANSLATION_WEIGHT=0.0
RENDERED_FEEDBACK_SOURCE_MOTION_ROTATION_WEIGHT=0.0
RENDERED_FEEDBACK_SOURCE_MOTION_HUBER_DELTA_M=0.1
RENDERED_FEEDBACK_SOURCE_MOTION_ROTATION_HUBER_DELTA_RAD=0.05
RENDERED_FEEDBACK_SOURCE_MOTION_MIN_DT_S=0.0
RENDERED_FEEDBACK_SOURCE_MOTION_MAX_DT_S=1.0
RENDERED_FEEDBACK_SOURCE_MOTION_IN_FROM_FRAME=false
VISUAL_EXPIRED_FACTOR_PROJECTION_MAX_AGE_S=5.0
ENABLE_VISUAL_CACHE_RECONCILIATION_DEFER_TO_POINTCLOUD=false
ENABLE_VISUAL_PAIR_PROCESSING_DEFER_TO_POINTCLOUD=false
ENABLE_MAPPER_FEEDBACK=false
MAPPER_FEEDBACK_SYNC_TOLERANCE_SEC=0.05
MAPPER_FEEDBACK_SYNC_ANCHOR_STREAM=pointcloud
ENABLE_GAUSSIAN_MAP_FEEDBACK=false
REQUIRE_GAUSSIAN_SNAPSHOT=false
MAPPER_FEEDBACK_RENDER_MODE="${MAPPER_FEEDBACK_RENDER_MODE:-debug_input}"
MAPPER_FEEDBACK_RENDER_MODE_EXPLICIT=false
MAPPER_FEEDBACK_POINTCLOUD_COORDINATES=world
MAPPER_FEEDBACK_POINTCLOUD_COORDINATES_EXPLICIT=false
MAPPER_FEEDBACK_MAX_DEPTH=20.0
MAPPER_FEEDBACK_MAX_DEPTH_EXPLICIT=false
MAPPER_FEEDBACK_IMAGE_QOS_RELIABILITY=best_effort
MAPPER_FEEDBACK_IMAGE_QOS_RELIABILITY_EXPLICIT=false
MAPPER_FEEDBACK_IMAGE_QOS_DEPTH=5
MAPPER_FEEDBACK_IMAGE_QOS_DEPTH_EXPLICIT=false
MAPPER_FEEDBACK_REQUIRE_PROJECTED_POINT_COLOR=true
MAPPER_FEEDBACK_ZBUFFER_PROJECTED_POINTS=false
MAPPER_FEEDBACK_PUBLISH_GAUSSIAN_MAP=false
MAPPER_FEEDBACK_GAUSSIAN_MAP_CHUNK_SIZE=4096
MAPPER_FEEDBACK_GAUSSIAN_MAP_QOS_DEPTH=128
MAPPER_FEEDBACK_GAUSSIAN_MAP_PUBLISH_MIN_INTERVAL_SEC=0.5
MAPPER_FEEDBACK_GAUSSIAN_MAP_PUBLISH_ON_EMPTY_EXTEND=false
GAUSSIAN_SNAPSHOT_QOS_DEPTH=128
GAUSSIAN_SNAPSHOT_LIDAR_FACTOR_WEIGHT=1.0
GAUSSIAN_SNAPSHOT_LIDAR_NEAREST_DISTANCE_M=0.0
GAUSSIAN_SNAPSHOT_LIDAR_RESIDUAL_PREWEIGHT=true
ENABLE_GAUSSIAN_SNAPSHOT_LIDAR_POSE_CORRECTION=false
GAUSSIAN_SNAPSHOT_LIDAR_POSE_CORRECTION_GAIN=0.3
GAUSSIAN_SNAPSHOT_LIDAR_POSE_CORRECTION_MAX_TRANSLATION_M=0.05
GAUSSIAN_SNAPSHOT_LIDAR_POSE_CORRECTION_MAX_ROTATION_RAD=0.02
GAUSSIAN_SNAPSHOT_LIDAR_POSE_CORRECTION_MIN_MATCH_RATIO=0.0
GAUSSIAN_SNAPSHOT_LIDAR_POSE_CORRECTION_MAX_MEAN_RESIDUAL_M=0.0
GAUSSIAN_SNAPSHOT_LIDAR_POSE_CORRECTION_COVERAGE_GRID_COLS=1
GAUSSIAN_SNAPSHOT_LIDAR_POSE_CORRECTION_COVERAGE_GRID_ROWS=1
GAUSSIAN_SNAPSHOT_LIDAR_POSE_CORRECTION_MIN_COVERAGE_TILES=0
GAUSSIAN_SNAPSHOT_LIDAR_POSE_CORRECTION_BIDIRECTIONAL_MAX_DISTANCE_M=0.0
ENABLE_GAUSSIAN_SNAPSHOT_LIDAR_PLANE_FACTOR=false
GAUSSIAN_SNAPSHOT_LIDAR_PLANE_FACTOR_WEIGHT=1.0
GAUSSIAN_SNAPSHOT_LIDAR_PLANE_MIN_ANISOTROPY=0.25
WRITE_STATUS_HISTORY=false
MAPPER_FEEDBACK_SELECT_EVERY_K_FRAME=8
MAPPER_FEEDBACK_SELECT_EVERY_K_FRAME_EXPLICIT=false
MAPPER_FEEDBACK_ENABLE_TORCH_CAMERA_CONVERSION=false
MAPPER_FEEDBACK_ENABLE_TORCH_GAUSSIAN_INIT=false
MAPPER_FEEDBACK_ENABLE_TORCH_GAUSSIAN_EXTEND=false
MAPPER_FEEDBACK_ENABLE_TORCH_GAUSSIAN_EXTEND_VISIBILITY_FILTER=true
MAPPER_FEEDBACK_ENABLE_TORCH_GAUSSIAN_OPTIMIZATION=false
MAPPER_FEEDBACK_ENABLE_TORCH_GAUSSIAN_PRUNING=false
MAPPER_FEEDBACK_ENABLE_TORCH_GAUSSIAN_DENSIFICATION=false
MAPPER_FEEDBACK_TORCH_DEVICE=cpu
MAPPER_FEEDBACK_TORCH_OPTIMIZATION_STEPS=0
MAPPER_FEEDBACK_TORCH_OPTIMIZATION_EVERY_N_KEYFRAMES=1
MAPPER_FEEDBACK_TORCH_OPTIMIZATION_EVERY_EXPLICIT=false
MAPPER_FEEDBACK_TORCH_OPTIMIZATION_SAMPLING=upstream_random
MAPPER_FEEDBACK_TORCH_OPTIMIZATION_SAMPLING_EXPLICIT=false
MAPPER_FEEDBACK_POSITION_LR=0.00016
MAPPER_FEEDBACK_FEATURE_LR=0.005
MAPPER_FEEDBACK_OPACITY_LR=0.05
MAPPER_FEEDBACK_SCALING_LR=0.005
MAPPER_FEEDBACK_ROTATION_LR=0.001
MAPPER_FEEDBACK_LR_EXPLICIT=false
MAPPER_FEEDBACK_TORCH_MAX_FOREGROUND=400000
MAPPER_FEEDBACK_TORCH_PRUNE_COUNT_POLICY=uniform
MAPPER_FEEDBACK_PUBLISH_RENDERED_BEFORE_UPDATE=false
MAPPER_FEEDBACK_RENDERED_FEEDBACK_SOURCE_STREAM=aligned_frame
MAPPER_FEEDBACK_RENDERED_FEEDBACK_IMAGE_TOPIC=__inherit__
MAPPER_FEEDBACK_RENDERED_FEEDBACK_POSE_TOPIC=__inherit__
MAPPER_FEEDBACK_RENDERED_FEEDBACK_IMAGE_QOS_RELIABILITY=best_effort
MAPPER_FEEDBACK_RENDERED_FEEDBACK_IMAGE_QOS_HISTORY=keep_last
MAPPER_FEEDBACK_RENDERED_FEEDBACK_IMAGE_QOS_DEPTH=64
MAPPER_FEEDBACK_RENDERED_FEEDBACK_POSE_QOS_RELIABILITY=best_effort
MAPPER_FEEDBACK_RENDERED_FEEDBACK_POSE_QOS_HISTORY=keep_last
MAPPER_FEEDBACK_RENDERED_FEEDBACK_POSE_QOS_DEPTH=64
RENDERED_IMAGE_QOS_RELIABILITY=reliable
RENDERED_IMAGE_QOS_DURABILITY=transient_local
RENDERED_IMAGE_QOS_DEPTH=1
ENABLE_RENDERED_FEEDBACK_CONTRACT=false
RENDERED_FEEDBACK_TOPIC=/gaussian_lic/rendered_feedback
RENDERED_FEEDBACK_QOS_RELIABILITY=reliable
RENDERED_FEEDBACK_QOS_DURABILITY=volatile
RENDERED_FEEDBACK_QOS_DEPTH=128
ENABLE_RENDERED_FEEDBACK_INGRESS_QUEUE=true
RENDERED_FEEDBACK_INGRESS_QUEUE_SIZE=512
RENDERED_FEEDBACK_INGRESS_DRAIN_MAX_PER_CYCLE=64
RENDERED_FEEDBACK_INGRESS_DRAIN_PERIOD_MS=5
VISUAL_FACTOR_MAX_DT_NS=300000000
VISUAL_DEPTH_MAX_DT_NS=0
VISUAL_DEPTH_FRAME_CACHE_SIZE=64
VISUAL_DEPTH_DILATION_PX=5
RENDERED_FRAME_CACHE_SIZE=64
OBSERVED_FRAME_CACHE_SIZE=128
VISUAL_PENDING_FACTOR_QUEUE_SIZE=128
ENABLE_VISUAL_FACTOR_QUALITY_WEIGHTING=false
VISUAL_FACTOR_QUALITY_MIN_WEIGHT_SCALE=0.25
ENABLE_VISUAL_FACTOR_QUALITY_SELECTION=false
ENABLE_VISUAL_FACTOR_QUALITY_REFERENCE_CAP=true
VISUAL_FACTOR_QUALITY_SELECTION_MAX_PER_REFERENCE=2
VISUAL_FACTOR_QUALITY_SELECTION_START_AFTER_S=0.0
VISUAL_ALIGNMENT_MAX_SHIFT_PX=8
VISUAL_ALIGNMENT_SCORE_MODE=rmse
VISUAL_ALIGNMENT_FACTOR_SOURCE=search
VISUAL_FACTOR_SOURCE_ID_MODE=legacy_8bit
VISUAL_FACTOR_REFERENCE_STAMP_MODE=observed
VISUAL_ALIGNMENT_WINDOW_WEIGHT=1.0
VISUAL_ALIGNMENT_SATURATION_MARGIN_PX=0.0
VISUAL_ALIGNMENT_SATURATED_WEIGHT_SCALE=1.0
SE3_PHOTOMETRIC_WINDOW_WEIGHT=0.5
SE3_PHOTOMETRIC_MAX_SAMPLES=2000
SE3_PHOTOMETRIC_MIN_SAMPLES=8
SE3_PHOTOMETRIC_MIN_HESSIAN_RANK=3
SE3_PHOTOMETRIC_MAX_HESSIAN_CONDITION=1000000000000.0
SE3_PHOTOMETRIC_MIN_SAMPLE_INLIER_RATIO=0.25
SE3_PHOTOMETRIC_MAX_MEAN_ABS_RESIDUAL_FOR_FACTOR=0.0
SE3_PHOTOMETRIC_COVERAGE_GRID_COLS=4
SE3_PHOTOMETRIC_COVERAGE_GRID_ROWS=4
SE3_PHOTOMETRIC_MIN_COVERAGE_TILES=4
SE3_PHOTOMETRIC_RANK_SAMPLES_BY_GRADIENT=false
SE3_PHOTOMETRIC_USE_RENDERED_GRADIENT=false
ENABLE_SE3_PHOTOMETRIC_POSE_CORRECTION=false
SE3_PHOTOMETRIC_POSE_CORRECTION_GAIN=0.1
SE3_PHOTOMETRIC_POSE_CORRECTION_MAX_TRANSLATION_M=0.02
SE3_PHOTOMETRIC_POSE_CORRECTION_MAX_ROTATION_RAD=0.01
SE3_PHOTOMETRIC_POSE_CORRECTION_MAX_DT_NS=0
ENABLE_EXTERNAL_ODOMETRY_PRIOR=false
REFERENCE_ODOMETRY_TOPIC="/gaussian_lic/frontend/input_odometry"
REFERENCE_POSE_TOPIC=""
REFERENCE_TUM_PATH=""
REQUIRE_REFERENCE_TRAJECTORY=false
MIN_REFERENCE_POSES=10
REFERENCE_TRAJECTORY_ALIGN=first
REFERENCE_MAX_ASSOCIATION_DT=0.2
REFERENCE_MIN_COVERAGE=0.4
REFERENCE_COVERAGE_MODE=all
REFERENCE_MAX_RMSE_M=2.0
REFERENCE_MAX_MEAN_M=1.5
REFERENCE_MAX_ERROR_M=5.0
REFERENCE_MAX_PATH_DRIFT=0.75
REFERENCE_MIN_CURRENT_PATH_RATIO=0.0
REFERENCE_MAX_CURRENT_PATH_RATIO=0.0
REFERENCE_ERROR_BIN_COUNT=8
REFERENCE_MIN_ERROR_BIN_MATCHES=0
REFERENCE_MAX_ERROR_BIN_RMSE_M=0.0
REFERENCE_MAX_ERROR_BIN_MEAN_M=0.0
REFERENCE_MAX_ERROR_BIN_BIAS_NORM_M=0.0
REFERENCE_TIME_OFFSET_SWEEP_MIN=-0.5
REFERENCE_TIME_OFFSET_SWEEP_MAX=0.5
REFERENCE_TIME_OFFSET_SWEEP_STEP=0.1
EXTERNAL_ODOMETRY_PRIOR_MAX_DT_NS=100000000
EXTERNAL_ODOMETRY_PRIOR_TRANSLATION_WEIGHT=4.0
EXTERNAL_ODOMETRY_PRIOR_ROTATION_WEIGHT=4.0
MAX_RENDERED_DELIVERY_LAG=0
MIN_RENDERED_DELIVERY_RECEIVED_RATIO=0.0
MAX_TRACKING_STEP_GUARD_CLAMP_RATIO=0.0

canonical_float() {
  python3 - "$1" <<'PY'
import math
import sys

value = float(sys.argv[1])
if not math.isfinite(value):
    raise SystemExit(f"expected finite float, got {sys.argv[1]!r}")
print(str(value))
PY
}

read_recorder_drain_value() {
  python3 - "$1" "$2" <<'PY'
import json
import sys
from pathlib import Path

metrics_path = Path(sys.argv[1])
metric_name = sys.argv[2]
if not metrics_path.is_file():
    print(0)
    raise SystemExit(0)
try:
    metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
except (OSError, json.JSONDecodeError):
    print(0)
    raise SystemExit(0)
if metric_name == "trajectory_poses":
    print(int(metrics.get("trajectory_poses", 0) or 0))
    raise SystemExit(0)
last = metrics.get("tracking_status", {}).get("last", {})
print(int(last.get(metric_name, 0) or 0))
PY
}

wait_for_recorder_drain() {
  local metrics_path="$1"
  local target_poses="$2"
  local target_visual="$3"
  local target_se3="$4"
  local timeout_sec="$5"
  if (( target_poses <= 0 && target_visual <= 0 && target_se3 <= 0 )); then
    return
  fi
  if (( timeout_sec <= 0 )); then
    return
  fi
  local deadline=$(( $(date +%s) + timeout_sec ))
  local poses=0
  local visual=0
  local se3=0
  while true; do
    poses="$(read_recorder_drain_value "${metrics_path}" trajectory_poses)"
    visual="$(read_recorder_drain_value "${metrics_path}" sliding_window_total_visual_factors)"
    se3="$(read_recorder_drain_value "${metrics_path}" sliding_window_total_se3_photometric_factors)"
    if (( (target_poses <= 0 || poses >= target_poses) &&
          (target_visual <= 0 || visual >= target_visual) &&
          (target_se3 <= 0 || se3 >= target_se3) )); then
      echo "[native-tracking] post-play drain reached poses=${poses}/${target_poses} visual=${visual}/${target_visual} se3=${se3}/${target_se3}" >&2
      return
    fi
    if (( $(date +%s) >= deadline )); then
      echo "[native-tracking] post-play drain timed out at poses=${poses}/${target_poses} visual=${visual}/${target_visual} se3=${se3}/${target_se3}" >&2
      return
    fi
    sleep 1
  done
}

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
  --playback-start-offset SEC   Start this many seconds into the bag. Default: 0.0.
  --rate RATE                  rosbag2 playback rate. Default: 1.0; Gaussian-map feedback uses 0.25 unless explicitly overridden.
  --clock-topics-all           Publish /clock immediately before each replayed message instead of using periodic --clock.
  --timeout SEC                Topic/report timeout. Default: 30.
  --post-play-settle SEC       Time to drain tracking callbacks after rosbag play exits. Default: 8.
  --post-play-drain-target-poses N
                               After post-play settle, keep recorder/tracker alive until trajectory_poses reaches N or the drain timeout expires. Default: 0 disabled.
  --post-play-drain-target-visual-factors N
                               Also wait until sliding_window_total_visual_factors reaches N. Default: 0 disabled.
  --post-play-drain-target-se3-factors N
                               Also wait until sliding_window_total_se3_photometric_factors reaches N. Default: 0 disabled.
  --post-play-drain-timeout SEC
                               Maximum additional drain wait. Default: 0 disabled.
  --min-poses N                Minimum recorded frontend odometry poses. Default: 20.
  --min-status-samples N       Minimum TrackingStatus samples. Default: 1.
  --write-status-history       Write per-sample TrackingStatus JSONL for drift diagnostics.
  --min-status-bin-sample-count N
                               Minimum samples required in each TrackingStatus time bin. Default: 0 disabled.
  --min-visual-factor-delta-per-status-bin N
                               Minimum cumulative visual-factor increase required in each TrackingStatus bin when visual factors are enabled. Default: 0 disabled.
  --min-se3-photometric-factor-delta-per-status-bin N
                               Minimum cumulative SE3 photometric-factor increase required in each TrackingStatus bin when visual factors are enabled. Default: 0 disabled.
  --min-motion-target-delta-per-status-bin N
                               Minimum cumulative smoothness motion-target activations required in each TrackingStatus bin when motion targets are enabled. Default: 0 disabled.
  --max-rendered-delivery-lag N
                               Fail if mapper rendered previews exceed tracking received rendered images by more than N. Default: 0 disabled.
  --min-rendered-delivery-received-ratio R
                               Fail if received/rendered preview ratio falls below R when mapper feedback renders images. Default: 0.0 disabled.
  --max-tracking-step-guard-clamp-ratio R
                               Fail if total step-guard clamps exceed R times recorded trajectory poses. Default: 0.0 disabled.
  --min-point-frames N         Minimum recorded /points_for_gs frames. Default: 10.
  --lidar-min-points N         Tracking LiDAR frame minimum. Default: 32.
  --lidar-max-frame-points N   Max LiDAR points used per frame factor. Default: 4000.
  --lidar-max-map-points N     Max LiDAR map points retained by the native factor. Default: 40000.
  --lidar-nearest-distance-m M  LiDAR correspondence gate. Default: 1.0.
  --lidar-correction-gain G     Bounded LiDAR correction gain. Default: 0.7.
  --lidar-max-correction-m M    Max LiDAR translation correction per frame. Default: 0.25.
  --lidar-max-rotation-rad R    Max LiDAR rotation correction per frame. Default: 0.08.
  --lidar-robust-kernel-m M     LiDAR robust kernel delta. Default: 0.15.
  --lidar-pose-factor-iterations N
                               LiDAR point-to-plane pose correction iterations. Default: 1.
  --lidar-window-point-factor-weight W
                               Weight multiplier for native LiDAR point-to-point BA factors. Default: 1.0.
  --lidar-window-plane-factor-weight W
                               Weight multiplier for native LiDAR point-to-plane BA factors. Default: 1.0.
  --lidar-window-confidence-power P
                               Exponent applied to LiDAR/Gaussian correspondence confidence weights before BA insertion. Default: 1.0.
  --enable-sliding-window-multihop-relative-translation-factor
                               Add one longer-baseline pre-BA relative translation factor inside the active BA window.
  --sliding-window-multihop-relative-translation-weight W
                               Weight for the multi-hop relative translation factor. Default: 0.0.
  --sliding-window-multihop-relative-translation-huber-delta-m M
                               Huber delta for the multi-hop relative translation factor. Default: 0.15.
  --sliding-window-relative-translation-in-from-frame
                               Compare adjacent relative translation in the source IMU frame instead of the world frame. Default: disabled.
  --sliding-window-multihop-relative-translation-in-from-frame
                               Compare multi-hop relative translation in the source IMU frame instead of the world frame. Default: disabled.
  --sliding-window-relative-rotation-weight W
                               Optional adjacent relative rotation weight attached to the relative translation factor. Default: 0.0.
  --sliding-window-relative-rotation-huber-delta-rad R
                               Huber delta for adjacent relative rotation. Default: 0.05.
  --sliding-window-multihop-relative-rotation-weight W
                               Optional multi-hop relative rotation weight. Default: 0.0.
  --sliding-window-multihop-relative-rotation-huber-delta-rad R
                               Huber delta for multi-hop relative rotation. Default: 0.08.
  --enable-sliding-window-relative-distance-factor
                               Add an adjacent distance-only motion-length factor. Default: disabled.
  --sliding-window-relative-distance-weight W
                               Weight for the adjacent distance-only factor. Default: 0.0.
  --sliding-window-relative-distance-huber-delta-m M
                               Huber delta for the adjacent distance-only factor. Default: 0.1.
  --enable-sliding-window-multihop-relative-distance-factor
                               Add multi-hop distance-only motion-length factors. Default: disabled.
  --sliding-window-multihop-relative-distance-weight W
                               Weight for the multi-hop distance-only factors. Default: 0.0.
  --sliding-window-multihop-relative-distance-huber-delta-m M
                               Huber delta for the multi-hop distance-only factors. Default: 0.15.
  --sliding-window-multihop-relative-translation-min-dt-s SEC
                               Minimum span for the multi-hop relative translation factor. Default: 0.45.
  --sliding-window-multihop-relative-translation-max-dt-s SEC
                               Maximum span for the multi-hop relative translation factor. Default: 1.05.
  --sliding-window-multihop-relative-translation-max-factors N
                               Max multi-hop relative translation factors inserted per frame. Default: 1.
  --enable-sliding-window-delayed-published-multihop-relative-translation-factor
                               Add same-stream published-to-published multi-hop relative factors after
                               post-BA/guard output, so the next solve/marginalization can test endpoint
                               consistency without replacing the production pre-BA path. Default: disabled.
  --sliding-window-delayed-published-multihop-start-after-s SEC
                               Warmup before adding delayed published multi-hop factors. Default: 0.
  --sliding-window-delayed-published-multihop-max-factors N
                               Max delayed published multi-hop factors inserted per frame. Default: 1.
  --sliding-window-relative-motion-history-source pre_ba|published|published_after_warmup
                               Source used to build relative/multihop motion-history factors.
                               pre_ba preserves the production default; published tests whether
                               final post-BA poses reduce long-horizon shape drift.
  --sliding-window-relative-motion-history-published-after-s SEC
                               With published_after_warmup, switch from pre-BA to published pose
                               history after SEC from the first sliding-window state. Default: 0.
  --lidar-keyframe-translation-m M
                               Keyframe insertion threshold. Default: 0.0 for strict replay sweeps.
  --max-lidar-invalid-frames N  Maximum invalid LiDAR frames tolerated by report gate. Default: 0.
  --enable-scan-order-deskew   Use explicit scan-order point timestamps when the bag has no per-point time field.
  --lidar-scan-order-duration-s SEC
                               Scan duration for --enable-scan-order-deskew. Default: 0.1.
  --raw-imu-qos-reliability R  IMU subscription reliability. Default: reliable.
  --raw-imu-qos-depth N        IMU subscription keep_last depth. Default: 2000.
  --raw-pointcloud-qos-reliability R
                               Point-cloud subscription reliability. Default: reliable.
  --raw-pointcloud-qos-depth N Point-cloud subscription keep_last depth. Default: 256.
  --pointcloud-imu-wait-queue-size N
                               Point-cloud queue while waiting for IMU catch-up. Default: 512.
  --imu-history-size N         IMU propagation history for delayed point-cloud timestamp queries. Default: 12000.
  --imu-linear-acceleration-scale S
                               Scale applied to incoming IMU linear_acceleration before propagation. Default: 9.80665 for FAST-LIVO/FAST-LIVO2 normalized-g bags; use 1.0 for SI m/s^2 bags.
  --tracking-max-pose-step-m M Max accepted native tracking pose step per point-cloud frame. Default: 0.25.
  --disable-pre-lio-step-guard
                               Skip the pre-LIO pose-step clamp while keeping the post-BA guard available.
  --disable-post-ba-step-guard
                               Skip the final post-BA pose-step clamp. Use only for diagnostics.
  --pre-lio-tracking-max-pose-step-m M
                               Override only the pre-LIO step guard limit. Default: 0.0, use --tracking-max-pose-step-m.
  --post-ba-tracking-max-pose-step-m M
                               Override only the post-BA step guard limit. Default: 0.0, use --tracking-max-pose-step-m.
  --post-ba-step-guard-confidence-max-pose-step-m M
                               Let high-confidence post-BA frames interpolate up to this step limit. Default: 0.0 disabled.
  --post-ba-step-guard-confidence-warmup-marginalizations N
                               Require at least N Schur marginalizations before confidence-gated post-BA release. Default: 0.
  --post-ba-step-guard-min-lidar-confidence C
                               LiDAR confidence needed for full confidence-gated post-BA allowance. Default: 0.6.
  --post-ba-step-guard-min-visual-inlier-ratio R
                               SE3 photometric inlier ratio needed for full confidence-gated post-BA allowance. Default: 0.85.
  --post-ba-step-guard-max-visual-residual R
                               Mean SE3 photometric residual for full confidence-gated post-BA allowance. Default: 0.3.
  --post-ba-step-guard-min-visual-coverage-tiles N
                               SE3 photometric coverage tiles needed for full confidence-gated post-BA allowance. Default: 8.
  --post-ba-step-guard-reject-to-pre-ba-over-m M
                               Reject over-large post-BA candidates to the pre-BA LiDAR/IMU pose instead of clamping their direction. Default: 0.0 disabled.
  --post-ba-step-guard-pre-ba-agreement-max-pose-step-m M
                               Allow a larger post-BA step up to M only when it agrees with the pre-BA LiDAR/IMU step. Default: 0.0 disabled.
  --post-ba-step-guard-pre-ba-agreement-late-start-marginalizations N
                               After N Schur marginalizations, use the late pre-BA agreement cap when larger. Default: 0 disabled.
  --post-ba-step-guard-pre-ba-agreement-late-max-pose-step-m M
                               Late-window pre-BA agreement pose-step cap. Default: 0.0 disabled.
  --post-ba-step-guard-pre-ba-agreement-min-cosine C
                               Direction cosine required for pre-BA agreement. Default: 0.85.
  --post-ba-step-guard-pre-ba-agreement-max-delta-m M
                               Max post-BA to pre-BA pose distance for agreement allowance. Default: 0.05.
  --post-ba-step-guard-pre-ba-agreement-margin-m M
                               Extra allowance added to the pre-BA step before the agreement cap. Default: 0.0.
  --post-ba-step-guard-pre-ba-blend-on-clamp W
                               Blend clamped post-BA step direction toward the pre-BA LiDAR/IMU direction. 0 keeps BA direction, 1 uses pre-BA direction. Default: 0.0.
  --tracking-step-guard-velocity-scale S
                               Optional speed-scaled adaptive pose-step allowance. Default: 0.0 disabled.
  --pre-lio-tracking-step-guard-velocity-scale S
                               Optional speed-scaled adaptive pose-step allowance for the pre-LIO guard only. Default: 0.0 disabled.
  --post-ba-tracking-step-guard-velocity-scale S
                               Optional speed-scaled adaptive pose-step allowance for the post-BA guard only. Default: 0.0 disabled.
  --tracking-step-guard-acceleration-mps2 A
                               Optional acceleration ramp for adaptive pose-step allowance. Default: 0.0 disabled.
  --tracking-step-guard-max-velocity-mps V
                               Optional hard velocity cap for adaptive pose-step allowance. Default: 0.0 disabled.
  --tracking-step-guard-margin-m M
                               Extra adaptive pose-step allowance margin. Default: 0.0.
  --require-deskew             Require nonzero trajectory deskew queries and hits in the report.
  --lidar-to-imu-translation V  YAML vector for LiDAR->IMU translation. Default: FAST-LIVO2.
  --lidar-to-imu-rpy V          YAML vector for LiDAR->IMU RPY radians. Default: FAST-LIVO2 identity.
  --camera-to-imu-translation V YAML vector for camera->IMU translation. Default: FAST-LIVO2.
  --camera-to-imu-rpy V         YAML vector for camera->IMU RPY radians. Default: FAST-LIVO2.
  --sliding-window-max-iterations N
                               Max BA iterations per solve. Default: 3.
  --sliding-window-optimize-every-n-frames N
                               Optimize the accumulated sliding window every N point-cloud frames. Default: 1; Gaussian-map feedback sets 4.
  --sliding-window-max-states N
                               Maximum states retained by the native sliding-window BA. Default: 12.
  --sliding-window-max-state-gap-s SEC
                               Max active-window state gap before BA is marked degenerate. Default: 1.0.
  --sliding-window-marginalization-prior-weight W
                               Weight for the Schur marginalization prior that anchors dropped sliding-window states. Default: 1.0.
  --sliding-window-max-condition C
                               Max normal-equation condition before BA is marked degenerate. Default: 1e13.
  --sliding-window-max-translation-step-m M
                               Max LM translation increment per state. Default: 1.0.
  --sliding-window-max-feedback-translation-m M
                               Max optimized-state translation feedback applied back to online tracking. Default: 1.0.
  --sliding-window-max-feedback-rotation-rad R
                               Max optimized-state rotation feedback applied back to online tracking. Default: 0.5.
  --sliding-window-max-feedback-velocity-mps V
                               Max optimized-state velocity feedback applied back to online tracking. Default: 5.0.
  --sliding-window-max-feedback-gyro-bias-step B
                               Max per-feedback gyro-bias delta applied back to online IMU propagation. Default: 0.0 disabled.
  --sliding-window-max-feedback-accel-bias-step B
                               Max per-feedback accel-bias delta applied back to online IMU propagation. Default: 0.0 disabled.
  --sliding-window-min-bias-feedback-visual-factors N
                               Hold optimized bias feedback unless the current BA update added at least N visual/SE3 factors. Default: 0 disabled.
  --sliding-window-bias-feedback-ownership MODE
                               Bias feedback ownership mode: optimized or pose_only_for_visual_feedback. Default: optimized.
  --sliding-window-sync-guarded-pose-state
                               Experimental: after post-BA step guard clamps/rejects a pose, synchronize the same sliding-window state to the final published pose. Default: disabled.
  --sliding-window-guarded-pose-prior-weights TRANS ROT
                               Experimental: after post-BA guard clamps/rejects a pose, add a soft pose prior at the guarded output pose. Default: 0 0 disabled.
  --sliding-window-imu-weight W
                               IMU residual weight. Default: 1.0.
  --sliding-window-imu-rotation-weight W
                               IMU rotation residual weight. Default: 1.0.
  --sliding-window-imu-velocity-weight W
                               IMU velocity residual weight. Default: 1.0.
  --sliding-window-imu-position-weight W
                               IMU position residual weight. Default: 1.0.
  --sliding-window-imu-velocity-prior-weight W
                               Optional state velocity prior toward propagated IMU velocity. Default: 0.0 disabled.
  --sliding-window-gyro-bias-prior-weight W
                               Optional insertion-time gyro-bias anchor prior. Default: 0.0 disabled.
  --sliding-window-accel-bias-prior-weight W
                               Optional insertion-time accel-bias anchor prior. Default: 0.0 disabled.
  --sliding-window-bias-weight W
                               IMU bias random-walk residual weight. Default: 1.0.
  --sliding-window-gyro-bias-weight W
                               Multiplier for gyro-bias random-walk residuals. Default: 1.0.
  --sliding-window-accel-bias-weight W
                               Multiplier for accel-bias random-walk residuals. Default: 1.0.
  --sliding-window-bias-random-walk-reference-dt-s SEC
                               Scale bias random-walk residuals by sqrt(SEC / dt). Default: 0.0 keeps legacy per-step weighting.
  --sliding-window-gyro-bias-random-walk-sigma SIGMA
                               Coco-LIC style gyro-bias random-walk sigma used to form sqrt information from IMU sample intervals. Default: 0.0 keeps legacy weighting.
  --sliding-window-accel-bias-random-walk-sigma SIGMA
                               Coco-LIC style accel-bias random-walk sigma used to form sqrt information from IMU sample intervals. Default: 0.0 keeps legacy weighting.
  --sliding-window-pose-translation-weight W
                               Pose prior translation weight. Default: 2.0.
  --sliding-window-pose-rotation-weight W
                               Pose prior rotation weight. Default: 2.0.
  --sliding-window-smoothness-rotation-weight W
                               Trajectory smoothness rotation weight. Default: 0.1.
  --sliding-window-smoothness-position-weight W
                               Trajectory smoothness position weight. Default: 0.1.
  --sliding-window-smoothness-velocity-weight W
                               Trajectory smoothness velocity weight. Default: 0.1.
  --sliding-window-smoothness-position-velocity-weight W
                               Position-rate to state-velocity consistency weight. Default: 0.0 disabled.
  --sliding-window-smoothness-bias-weight W
                               Bias smoothness weight. Default: 0.1.
  --sliding-window-smoothness-use-motion-targets
                               Use pre-BA local motion curvature as the smoothness target instead of forcing zero curvature. Default: disabled.
  --sliding-window-smoothness-motion-target-min-visual-factors N
                               Only apply motion targets when this update has at least N visual factors. Default: 0.
  --sliding-window-smoothness-motion-target-min-se3-photometric-factors N
                               Only apply motion targets when this update has at least N SE3 photometric factors. Default: 0.
  --sliding-window-smoothness-motion-target-recent-window N
                               Require a recent support window of N BA updates before applying measured motion targets. Default: 0 disabled.
  --sliding-window-smoothness-motion-target-min-recent-visual-factors N
                               Minimum visual factors across the recent support window. Default: 0.
  --sliding-window-smoothness-motion-target-min-recent-se3-photometric-factors N
                               Minimum SE3 photometric factors across the recent support window. Default: 0.
  --sliding-window-smoothness-motion-target-start-after-s SEC
                               Delay motion-target smoothness until SEC after the first sliding-window state. Default: 0.0.
  --sliding-window-smoothness-motion-target-max-rotation-rate-delta-radps W
                               Clamp motion-target rotation-rate delta norm. 0 disables clamping. Default: 0.25.
  --sliding-window-smoothness-motion-target-max-position-rate-delta-mps W
                               Clamp motion-target position-rate delta norm. 0 disables clamping. Default: 0.5.
  --sliding-window-smoothness-motion-target-max-velocity-acceleration-delta-mps2 W
                               Clamp motion-target velocity-acceleration delta norm. 0 disables clamping. Default: 2.0.
  --enable-sliding-window-relative-translation-factor
                               Add an internal adjacent-pose relative translation prior from raw IMU/pre-LIO propagation. Default: disabled.
  --sliding-window-relative-translation-weight W
                               Relative translation prior weight. Default: 0.0 disabled.
  --sliding-window-relative-translation-huber-delta-m M
                               Relative translation prior Huber delta. Default: 0.1.
  --require-ba-feedback        Require accepted sliding-window feedback.
  --require-nondegenerate-ba   Require the last reported BA normal equation and state cadence to be non-degenerate.
  --enable-visual-factors      Require mapper-rendered-image visual factors to be present externally.
  --enable-visual-factor-time-interpolation
                               Attach visual/SE3 factors to bracketing continuous-time states instead of one snapped state.
  --disable-visual-factor-time-interpolation
                               Keep the legacy single-reference visual/SE3 factor behavior.
  --enable-visual-cache-reconciliation
                               Re-scan observed/rendered caches for unprocessed nearest-stamp visual pairs.
  --disable-visual-cache-reconciliation
                               Keep the legacy one-pair-per-arrival cache behavior.
  --enable-visual-cache-reconciliation-monotonic-unique
                               In reconciliation mode, consume each observed and rendered stamp at most once.
  --disable-visual-cache-reconciliation-monotonic-unique
                               Allow multiple observed/rendered stamps to share the opposite image stamp.
  --enable-visual-pair-monotonic-unique
                               Consume each observed and rendered stamp at most once across direct and reconciled visual pairs.
  --disable-visual-pair-monotonic-unique
                               Allow direct visual pairs to reuse observed/rendered stamps when exact pair keys differ.
  --enable-visual-watermark-pair-scheduler
                               Cache image/render callbacks and process deterministic observed/rendered pairs up to each point-cloud stamp.
  --disable-visual-watermark-pair-scheduler
                               Keep legacy callback-time visual pair processing.
  --visual-watermark-pair-scheduler-max-pairs-per-pointcloud N
                               Maximum watermark-scheduled visual pairs consumed per point-cloud callback. Default: 2.
  --enable-visual-cache-reconciliation-defer-to-pointcloud
                               Run cache reconciliation from point-cloud callbacks instead of image/render callbacks.
  --disable-visual-cache-reconciliation-defer-to-pointcloud
                               Run cache reconciliation immediately from image/render callbacks.
  --enable-visual-pair-processing-defer-to-pointcloud
                               Cache image/render callbacks and process visual pairs from point-cloud callbacks.
  --disable-visual-pair-processing-defer-to-pointcloud
                               Process direct visual pairs inside image/render callbacks.
  --enable-visual-callback-factor-ingest
                               Ingest visual/SE3 factors from image/render callbacks when their sliding-window reference is still active.
  --disable-visual-callback-factor-ingest
                               Queue visual/SE3 factors until point-cloud callbacks ingest them.
  --enable-rendered-feedback-watermark-queue
                               Queue typed rendered-feedback pairs and consume them from point-cloud watermarks. Default: disabled.
  --disable-rendered-feedback-watermark-queue
                               Disable typed rendered-feedback watermark queueing.
  --defer-future-visual-factors-until-active
                               Keep visual/SE3 factors pending until their reference stamp is no longer newer than the current tracking state.
  --no-defer-future-visual-factors-until-active
                               Restore legacy immediate future-reference visual factor ingestion.
  --enable-visual-adaptive-state-retention
                               Increase sliding-window state retention from rendered-feedback backlog so late visual factors can still bind active states.
  --disable-visual-adaptive-state-retention
                               Keep the fixed configured sliding-window max-state count.
  --visual-adaptive-state-retention-margin-states N
                               Extra state margin added above measured rendered-feedback backlog. Default: 4.
  --visual-adaptive-state-retention-max-states N
                               Upper bound for adaptive visual state retention. Default: 64.
  --enable-visual-expired-factor-projection
                               Project late rendered-feedback visual/SE3 factors onto the current active state instead of dropping them after their original state expires.
  --disable-visual-expired-factor-projection
                               Drop expired visual/SE3 factors when their original active-window state is gone.
  --enable-visual-marginalization-prior
                               Convert late visual/SE3 factors on marginalized source states into Schur back-substitution dense priors on retained states.
  --disable-visual-marginalization-prior
                               Do not convert late visual/SE3 factors into marginalized-state priors.
  --enable-visual-marginalization-prior-batching
                               Batch late visual/SE3 marginalized priors into one retained dense prior per drain cycle. Default: disabled.
  --disable-visual-marginalization-prior-batching
                               Add late visual/SE3 marginalized priors one factor at a time.
  --enable-visual-marginalization-prior-saturation-gate
                               Drop late visual/SE3 marginalized priors whose visual pair saturated the alignment search window.
  --disable-visual-marginalization-prior-saturation-gate
                               Keep saturated visual pairs eligible for late visual/SE3 marginalized priors.
  --visual-marginalization-prior-zero-bias-columns
                               Prevent late visual/SE3 marginalized priors from writing IMU gyro/accel bias columns.
  --no-visual-marginalization-prior-zero-bias-columns
                               Keep full Schur back-substitution coupling, including IMU bias columns.
  --enable-visual-factor-reference-snapshot
                               Snapshot visual/SE3 linearization reference poses when factors are produced. Default: disabled until full-window promotion.
  --disable-visual-factor-reference-snapshot
                               Use the active/marginalized support pose as the visual/SE3 factor reference.
  --enable-rendered-feedback-source-pose-reference
                               Use the mapper-provided rendered source pose as the visual/SE3 factor reference for typed RenderedFeedback. Default: disabled.
  --disable-rendered-feedback-source-pose-reference
                               Ignore RenderedFeedback.source_pose and use the active/marginalized support pose.
  --enable-rendered-feedback-source-motion-factor
                               Queue cross-frame relative motion factors from consecutive typed RenderedFeedback.source_pose samples. Default: disabled.
  --disable-rendered-feedback-source-motion-factor
                               Disable RenderedFeedback.source_pose relative motion factors.
  --enable-rendered-feedback-source-motion-marginalized-prior
                               Convert late source-motion factors into Schur back-substitution dense priors. Default: disabled until full-window promotion.
  --disable-rendered-feedback-source-motion-marginalized-prior
                               Drop late source-motion factors instead of adding marginalized dense priors.
  --rendered-feedback-source-motion-translation-weight W
                               Translation weight for source-pose relative motion factors. Default: 0.0.
  --rendered-feedback-source-motion-rotation-weight W
                               Rotation weight for source-pose relative motion factors. Default: 0.0.
  --rendered-feedback-source-motion-max-dt-s SEC
                               Maximum source-pose pair interval accepted for relative motion factors. Default: 1.0.
  --visual-expired-factor-projection-max-age-s SEC
                               Maximum age for projected expired visual/SE3 factors. Use <=0 for unlimited. Default: 5.0.
  --enable-mapper-feedback     Launch mapping_node so native tracking can consume mapper rendered-image feedback.
  --enable-gaussian-map-feedback
                               Launch mapping_node with Torch Gaussian init/extend, rasterizer rendered-image feedback, and GaussianArray publication so tracking can consume map anchors and real Gaussian photometric BA.
  --require-gaussian-snapshot  Require a complete GaussianArray snapshot in the tracking status report.
  --gaussian-snapshot-lidar-factor-weight W
                               Weight multiplier for LiDAR-to-Gaussian map anchors. Default: 1.0.
  --gaussian-snapshot-lidar-nearest-distance-m M
                               Dedicated nearest-neighbor gate for Gaussian snapshot anchors. Default: 0.0 inherits --lidar-nearest-distance-m.
  --disable-gaussian-snapshot-lidar-residual-preweight
                               Do not pre-weight Gaussian snapshot anchors by residual before the BA Huber loss. Diagnostic for double-robust weighting.
  --enable-gaussian-snapshot-lidar-pose-correction
                               Apply a bounded pre-BA SE3 scan-to-Gaussian snapshot correction. Default: disabled.
  --gaussian-snapshot-lidar-pose-correction-gain G
                               Gain for the bounded Gaussian snapshot pose correction. Default: 0.3.
  --gaussian-snapshot-lidar-pose-correction-max-translation-m M
                               Max translation step from the Gaussian snapshot pose correction. Default: 0.05.
  --gaussian-snapshot-lidar-pose-correction-max-rotation-rad R
                               Max rotation step from the Gaussian snapshot pose correction. Default: 0.02.
  --gaussian-snapshot-lidar-pose-correction-min-match-ratio R
                               Minimum finite sampled-point match ratio before applying Gaussian snapshot pose correction. Default: 0.0 disabled.
  --gaussian-snapshot-lidar-pose-correction-max-mean-residual-m M
                               Maximum weighted mean residual before applying Gaussian snapshot pose correction. Default: 0.0 disabled.
  --gaussian-snapshot-lidar-pose-correction-coverage-grid-cols N
                               XY coverage grid columns for Gaussian snapshot pose correction matches. Default: 1.
  --gaussian-snapshot-lidar-pose-correction-coverage-grid-rows N
                               XY coverage grid rows for Gaussian snapshot pose correction matches. Default: 1.
  --gaussian-snapshot-lidar-pose-correction-min-coverage-tiles N
                               Minimum occupied XY coverage tiles before applying Gaussian snapshot pose correction. Default: 0 disabled.
  --gaussian-snapshot-lidar-pose-correction-bidirectional-max-distance-m M
                               Require target Gaussian's nearest matched source point to be the originating source within this distance. Default: 0.0 disabled.
  --enable-gaussian-snapshot-lidar-plane-factor
                               Add LiDAR-to-Gaussian point-to-plane BA anchors from anisotropic Gaussian rotation/scale.
  --gaussian-snapshot-lidar-plane-factor-weight W
                               Weight multiplier for LiDAR-to-Gaussian plane anchors. Default: 1.0.
  --gaussian-snapshot-lidar-plane-min-anisotropy A
                               Minimum 1-min(scale)/max(scale) before a Gaussian contributes a plane normal. Default: 0.25.
  --mapper-feedback-sync-tolerance-sec SEC
                               mapping_node frame sync tolerance for mapper feedback. Default: 0.05.
  --mapper-feedback-sync-anchor STREAM
                               mapping_node synchronization anchor stream for mapper feedback: pointcloud or image. Default: pointcloud.
  --mapper-feedback-image-qos-reliability MODE
                               mapping_node image input QoS reliability for mapper feedback. Default: best_effort. Use only with a compatible rosbag/player QoS override.
  --mapper-feedback-image-qos-depth N
                               mapping_node image input QoS depth for mapper feedback. Default: 5.
  --mapper-feedback-render-mode MODE
                               mapping_node rendered image mode for feedback. Default: debug_input; --enable-gaussian-map-feedback promotes this to rasterizer unless explicitly set.
  --mapper-feedback-pointcloud-coordinates MODE
                               mapping_node pointcloud coordinate semantics: world or sensor. Native Gaussian feedback defaults to sensor.
  --mapper-feedback-max-depth M
                               mapping_node input/projection max depth in meters. Native Gaussian feedback defaults to 20 to match mapper profiles and bound Gaussian growth.
  --mapper-feedback-allow-unprojected-color-fallback
                               Allow non-RGB LiDAR points that cannot project into the image to enter the Gaussian map with intensity/grayscale fallback color. Default: disabled.
  --mapper-feedback-zbuffer-projected-points
                               Keep only the nearest projected non-RGB LiDAR point per image pixel before Gaussian insertion. Default: disabled.
  --mapper-feedback-gaussian-map-publish-min-interval-sec SEC
                               Minimum simulated-time interval between full GaussianArray feedback publications. Default: 0.5.
  --mapper-feedback-gaussian-map-publish-on-empty-extend
                               Publish GaussianArray after keyframes that insert no new Gaussians. Disabled by default for feedback runs.
  --mapper-feedback-select-every-k-frame N
                               Mapping keyframe decimation for mapper feedback. Gaussian-map feedback defaults to 1 unless explicitly set.
  --mapper-feedback-torch-device DEV
                               Torch device for mapper feedback Gaussian snapshots. Default: cpu; --enable-gaussian-map-feedback sets auto.
  --mapper-feedback-torch-optimization-steps N
                               Per-frame Torch optimization steps for mapper feedback Gaussian snapshots. Default: 0.
  --mapper-feedback-torch-optimization-every-n-keyframes N
                               Run mapper feedback optimization once per N keyframes. Online Gaussian feedback with optimization defaults to 16.
  --mapper-feedback-torch-optimization-sampling MODE
                               Training-frame sampler for mapper feedback optimization. Default: upstream_random; online Gaussian feedback with optimization steps uses latest_even unless explicitly set.
  --mapper-feedback-enable-torch-optimization
                               Enable mapper feedback Torch photometric optimization.
  --mapper-feedback-gaussian-lr POS FEAT OPACITY SCALE ROT
                               Override mapper feedback Gaussian optimizer learning rates.
  --mapper-feedback-torch-max-foreground N
                               Cap foreground Gaussians for mapper feedback snapshots. Default: 400000 in the Gaussian feedback preset.
  --mapper-feedback-torch-prune-count-policy POLICY
                               Foreground cap pruning policy: opacity or uniform. Default: uniform for feedback coverage.
  --mapper-feedback-enable-extend-visibility-filter
                               Keep mapper feedback Gaussian extension alpha-hole filtering enabled. The production Gaussian feedback preset disables it so the map can keep growing under skybox/rasterizer feedback.
  --mapper-feedback-disable-extend-visibility-filter
                               Disable mapper feedback Gaussian extension alpha-hole filtering.
  --mapper-feedback-publish-rendered-before-update
                               Publish mapper rendered feedback before Torch/Gaussian update. Diagnostic for source-time visual/SE3 factor expiry; default disabled.
  --rendered-image-qos-reliability MODE
                               QoS reliability for /gaussian_lic/rendered_image feedback: reliable or best_effort. Default: reliable.
  --rendered-image-qos-durability MODE
                               QoS durability for /gaussian_lic/rendered_image feedback: transient_local or volatile. Default: transient_local.
  --rendered-image-qos-depth N
                               QoS keep-last depth for /gaussian_lic/rendered_image feedback. Default: 1.
  --enable-rendered-feedback-contract
                               Subscribe tracking to typed /gaussian_lic/rendered_feedback with mapper/source stamps instead of the legacy rendered Image topic.
  --rendered-feedback-topic TOPIC
                               Typed rendered-feedback topic. Default: /gaussian_lic/rendered_feedback.
  --rendered-feedback-qos-reliability MODE
                               QoS reliability for typed /gaussian_lic/rendered_feedback: reliable or best_effort. Default: reliable.
  --rendered-feedback-qos-durability MODE
                               QoS durability for typed /gaussian_lic/rendered_feedback: transient_local or volatile. Default: volatile.
  --rendered-feedback-qos-depth N
                               QoS keep-last depth for typed /gaussian_lic/rendered_feedback. Default: 128.
  --enable-rendered-feedback-ingress-queue
                               Keep typed rendered-feedback ROS callbacks lightweight by enqueueing images before serialized estimator processing. Default enabled.
  --disable-rendered-feedback-ingress-queue
                               Process typed rendered-feedback directly in the serialized callback path.
  --rendered-feedback-ingress-queue-size N
                               Internal typed rendered-feedback ingress queue depth. Default: 512.
  --rendered-feedback-ingress-drain-max-per-cycle N
                               Max queued typed rendered-feedback messages drained per serialized cycle. Default: 64.
  --rendered-feedback-ingress-drain-period-ms N
                               Wall-timer period for draining queued typed rendered-feedback messages. Default: 5.
  --visual-factor-max-dt-ns NS Max nearest-stamp delta for rendered/observed visual BA pairing. Default: 300000000.
  --visual-depth-max-dt-ns NS  Max nearest-stamp delta for sparse LiDAR depth selected by SE3 visual BA. Default: 0, follow --visual-factor-max-dt-ns.
  --visual-depth-dilation-px N Sparse LiDAR depth projection dilation radius for SE3 visual BA. Default: 5.
  --rendered-frame-cache-size N
                               tracking_node rendered-image cache size for delayed mapper feedback. Default: 64.
  --observed-frame-cache-size N
                               tracking_node observed-image cache size for delayed mapper feedback. Default: 128.
  --visual-pending-factor-queue-size N
                               tracking_node pending visual/SE3 factor queue size before BA ingestion. Default: 128.
  --enable-visual-factor-quality-weighting
                               Preserve visual/SE3 factor continuity but downweight stale, saturated, or low-coverage observations. Default: disabled.
  --disable-visual-factor-quality-weighting
                               Disable visual/SE3 quality weight scaling.
  --visual-factor-quality-min-weight-scale S
                               Minimum multiplicative weight scale for quality-weighted visual/SE3 factors. Default: 0.25.
  --enable-visual-factor-quality-selection
                               Enable quality-ranked pending visual/SE3 selection before BA ingestion. Default: disabled.
  --disable-visual-factor-quality-selection
                               Disable quality-ranked pending visual/SE3 selection and restore FIFO queue trimming.
  --enable-visual-factor-quality-reference-cap
                               Limit quality-selected visual/SE3 factors retained per reference state. Default: enabled.
  --disable-visual-factor-quality-reference-cap
                               Keep quality-ranked queue trimming active without capping factors per reference state.
  --visual-factor-quality-selection-max-per-reference N
                               Maximum quality-ranked visual/SE3 factors retained per reference state. Default: 2.
  --visual-factor-quality-selection-start-after-s SEC
                               Delay quality-ranked visual/SE3 selection until SEC after the first sliding-window state. Default: 0.
  --visual-alignment-max-shift-px N
                               Exhaustive 2D visual alignment search radius. Default: 8.
  --visual-alignment-score-mode rmse|zncc
                               2D visual alignment objective. rmse preserves the legacy raw-intensity cost; zncc uses zero-mean normalized cross-correlation. Default: rmse.
  --visual-alignment-factor-source search|photometric_step|saturated_photometric_step
                               Measurement source for the sliding-window 2D visual factor. Default: search.
  --visual-factor-source-id-mode legacy_8bit|full_64bit
                               Source-id hash width for rendered/observed visual factors. Default: legacy_8bit.
  --visual-factor-reference-stamp-mode observed|rendered
                               Stamp used to own rendered-feedback visual/SE3 factors in the sliding window. Default: observed.
  --visual-alignment-window-weight W
                               Sliding-window 2D visual alignment factor weight. Default: 1.0.
  --visual-alignment-saturation-margin-px PX
                               Treat visual alignment shifts within PX of the search edge as saturated. Default: 0.0.
  --visual-alignment-saturated-weight-scale S
                               Multiply saturated visual alignment factor weight by S while keeping factor continuity. Default: 1.0.
  --se3-photometric-window-weight W
                               Sliding-window SE3 photometric factor weight. Default: 0.5.
  --se3-photometric-max-samples N
                               Maximum sparse-depth samples used per SE3 photometric BA batch. Default: 2000.
  --se3-photometric-min-samples N
                               Minimum valid sparse-depth samples needed before adding an SE3 photometric BA factor. Default: 8.
  --se3-photometric-min-hessian-rank N
                               Minimum 6x6 SE3 photometric Hessian rank before accepting a BA factor. Default: 3.
  --se3-photometric-max-hessian-condition C
                               Maximum SE3 photometric Hessian condition before rejecting a BA factor. Default: 1e12.
  --se3-photometric-min-sample-inlier-ratio R
                               Minimum accepted/sampled sparse-depth ratio before accepting an SE3 photometric BA factor. Default: 0.25.
  --se3-photometric-max-mean-abs-residual R
                               Optional max mean absolute residual for accepted SE3 photometric BA factors. Default: 0.0 disabled.
  --se3-photometric-use-rendered-gradient
                               Use rendered/map-image gradients for SE3 photometric BA linearization. Default: observed-image gradients.
  --se3-photometric-rank-samples-by-gradient
                               Rank candidate sparse-depth samples by image-gradient magnitude inside each coverage tile before applying the per-tile quota. Default: false.
  --se3-photometric-coverage-grid-cols N
                               SE3 photometric image coverage grid columns. Default: 4.
  --se3-photometric-coverage-grid-rows N
                               SE3 photometric image coverage grid rows. Default: 4.
  --se3-photometric-min-coverage-tiles N
                               Minimum occupied image tiles before accepting an SE3 photometric BA factor. Default: 4.
  --enable-se3-photometric-pose-correction
                               Apply gated SE3 photometric Gauss-Newton steps directly to the tracking pose. Default: false.
  --se3-photometric-pose-correction-gain G
                               Gain for direct SE3 photometric pose correction. Default: 0.1.
  --se3-photometric-pose-correction-max-translation-m M
                               Clamp direct SE3 photometric translation correction. Default: 0.02.
  --se3-photometric-pose-correction-max-rotation-rad R
                               Clamp direct SE3 photometric rotation correction. Default: 0.01.
  --se3-photometric-pose-correction-max-dt-ns NS
                               Max stamp delta for direct SE3 photometric correction. Default: 0, follow --visual-factor-max-dt-ns.
  --enable-external-odometry-prior
                               Feed the reference odometry topic into tracking BA as an optional pose prior.
  --reference-odometry-topic T Topic to record as reference TUM trajectory. Default: /gaussian_lic/frontend/input_odometry.
  --reference-pose-topic T     Optional PoseStamped reference trajectory topic.
  --reference-tum FILE         Optional external TUM reference trajectory for report comparison.
  --require-reference-trajectory
                               Require reference trajectory samples and trajectory_compare.py gate.
  --min-reference-poses N      Minimum reference poses when required. Default: 10.
  --reference-trajectory-align none|first|yaw
                               Alignment mode passed to trajectory_compare.py. Default: first.
  --reference-max-association-dt SEC
                               Max timestamp association delta for reference comparison. Default: 0.2.
  --reference-min-coverage R   Minimum reference trajectory coverage. Default: 0.4.
  --reference-coverage-mode all|overlap
                               Coverage denominator for trajectory_compare.py.
                               Default all keeps full-sequence strict gates; overlap is for clipped bag probes.
  --reference-max-rmse-m M     Max native-vs-reference translation RMSE. Default: 2.0.
  --reference-max-mean-m M     Max native-vs-reference translation mean error. Default: 1.5.
  --reference-max-error-m M    Max native-vs-reference translation max error. Default: 5.0.
  --reference-max-path-drift R Max relative path-length drift. Default: 0.75.
  --reference-min-current-path-ratio R
                               Minimum current/reference path-length ratio for detecting guard-induced compression. Default: 0.0 disabled.
  --reference-max-current-path-ratio R
                               Maximum current/reference path-length ratio for detecting over-release. Default: 0.0 disabled.
  --reference-error-bin-count N
                               Number of time-ordered trajectory error bins archived in the reference comparison. Default: 8.
  --reference-min-error-bin-matches N
                               Minimum matched poses required in every trajectory error bin. Default: 0 disabled.
  --reference-max-error-bin-rmse-m M
                               Maximum RMSE allowed in every trajectory error bin. Default: 0.0 disabled.
  --reference-max-error-bin-mean-m M
                               Maximum mean translation error allowed in every trajectory error bin. Default: 0.0 disabled.
  --reference-max-error-bin-bias-norm-m M
                               Maximum norm of each error-bin XYZ bias vector. Default: 0.0 disabled.
  --reference-time-offset-sweep MIN MAX STEP
                               Sweep current-trajectory timestamp offsets for diagnostics. Default: -0.5 0.5 0.1; use 0 0 0 to disable.
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
    --playback-start-offset)
      PLAYBACK_START_OFFSET="$2"
      shift 2
      ;;
    --rate)
      PLAYBACK_RATE="$2"
      PLAYBACK_RATE_EXPLICIT=true
      shift 2
      ;;
    --clock-topics-all)
      PLAYBACK_CLOCK_TOPICS_ALL=true
      shift
      ;;
    --timeout)
      TIMEOUT_SEC="$2"
      shift 2
      ;;
    --post-play-settle)
      POST_PLAY_SETTLE_SEC="$2"
      shift 2
      ;;
    --post-play-drain-target-poses)
      POST_PLAY_DRAIN_TARGET_POSES="$2"
      shift 2
      ;;
    --post-play-drain-target-visual-factors)
      POST_PLAY_DRAIN_TARGET_VISUAL_FACTORS="$2"
      shift 2
      ;;
    --post-play-drain-target-se3-factors)
      POST_PLAY_DRAIN_TARGET_SE3_FACTORS="$2"
      shift 2
      ;;
    --post-play-drain-timeout)
      POST_PLAY_DRAIN_TIMEOUT_SEC="$2"
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
    --min-status-bin-sample-count)
      MIN_STATUS_BIN_SAMPLE_COUNT="$2"
      shift 2
      ;;
    --min-visual-factor-delta-per-status-bin)
      MIN_VISUAL_FACTOR_DELTA_PER_STATUS_BIN="$2"
      shift 2
      ;;
    --min-se3-photometric-factor-delta-per-status-bin)
      MIN_SE3_PHOTOMETRIC_FACTOR_DELTA_PER_STATUS_BIN="$2"
      shift 2
      ;;
    --min-motion-target-delta-per-status-bin)
      MIN_MOTION_TARGET_DELTA_PER_STATUS_BIN="$2"
      shift 2
      ;;
    --max-rendered-delivery-lag)
      MAX_RENDERED_DELIVERY_LAG="$2"
      shift 2
      ;;
    --min-rendered-delivery-received-ratio)
      MIN_RENDERED_DELIVERY_RECEIVED_RATIO="$2"
      shift 2
      ;;
    --max-tracking-step-guard-clamp-ratio)
      MAX_TRACKING_STEP_GUARD_CLAMP_RATIO="$2"
      shift 2
      ;;
    --write-status-history)
      WRITE_STATUS_HISTORY=true
      shift
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
    --lidar-nearest-distance-m)
      LIDAR_NEAREST_DISTANCE_M="$2"
      shift 2
      ;;
    --lidar-correction-gain)
      LIDAR_CORRECTION_GAIN="$2"
      shift 2
      ;;
    --lidar-max-correction-m)
      LIDAR_MAX_CORRECTION_M="$2"
      shift 2
      ;;
    --lidar-max-rotation-rad)
      LIDAR_MAX_ROTATION_RAD="$2"
      shift 2
      ;;
    --lidar-robust-kernel-m)
      LIDAR_ROBUST_KERNEL_M="$2"
      shift 2
      ;;
    --lidar-pose-factor-iterations)
      LIDAR_POSE_FACTOR_ITERATIONS="$2"
      shift 2
      ;;
    --lidar-window-point-factor-weight)
      LIDAR_WINDOW_POINT_FACTOR_WEIGHT="$2"
      shift 2
      ;;
    --lidar-window-plane-factor-weight)
      LIDAR_WINDOW_PLANE_FACTOR_WEIGHT="$2"
      shift 2
      ;;
    --lidar-window-confidence-power)
      LIDAR_WINDOW_CONFIDENCE_POWER="$2"
      shift 2
      ;;
    --lidar-keyframe-translation-m)
      LIDAR_KEYFRAME_TRANSLATION_M="$2"
      shift 2
      ;;
    --max-lidar-invalid-frames)
      MAX_LIDAR_INVALID_FRAMES="$2"
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
    --raw-imu-qos-reliability)
      RAW_IMU_QOS_RELIABILITY="$2"
      shift 2
      ;;
    --raw-imu-qos-depth)
      RAW_IMU_QOS_DEPTH="$2"
      shift 2
      ;;
    --raw-pointcloud-qos-reliability)
      RAW_POINTCLOUD_QOS_RELIABILITY="$2"
      shift 2
      ;;
    --raw-pointcloud-qos-depth)
      RAW_POINTCLOUD_QOS_DEPTH="$2"
      shift 2
      ;;
    --pointcloud-imu-wait-queue-size)
      POINTCLOUD_IMU_WAIT_QUEUE_SIZE="$2"
      shift 2
      ;;
    --imu-history-size)
      IMU_HISTORY_SIZE="$2"
      shift 2
      ;;
    --imu-linear-acceleration-scale)
      IMU_LINEAR_ACCELERATION_SCALE="$2"
      shift 2
      ;;
    --tracking-max-pose-step-m)
      TRACKING_MAX_POSE_STEP_M="$2"
      shift 2
      ;;
    --tracking-step-guard-velocity-scale)
      TRACKING_STEP_GUARD_VELOCITY_SCALE="$2"
      shift 2
      ;;
    --pre-lio-tracking-step-guard-velocity-scale)
      PRE_LIO_TRACKING_STEP_GUARD_VELOCITY_SCALE="$2"
      shift 2
      ;;
    --post-ba-tracking-step-guard-velocity-scale)
      POST_BA_TRACKING_STEP_GUARD_VELOCITY_SCALE="$2"
      shift 2
      ;;
    --tracking-step-guard-acceleration-mps2)
      TRACKING_STEP_GUARD_ACCELERATION_MPS2="$2"
      shift 2
      ;;
    --tracking-step-guard-max-velocity-mps)
      TRACKING_STEP_GUARD_MAX_VELOCITY_MPS="$2"
      shift 2
      ;;
    --tracking-step-guard-margin-m)
      TRACKING_STEP_GUARD_MARGIN_M="$2"
      shift 2
      ;;
    --disable-pre-lio-step-guard)
      ENABLE_PRE_LIO_TRACKING_STEP_GUARD=false
      shift
      ;;
    --disable-post-ba-step-guard)
      ENABLE_POST_BA_TRACKING_STEP_GUARD=false
      shift
      ;;
    --pre-lio-tracking-max-pose-step-m)
      PRE_LIO_TRACKING_MAX_POSE_STEP_M="$2"
      shift 2
      ;;
    --post-ba-tracking-max-pose-step-m)
      POST_BA_TRACKING_MAX_POSE_STEP_M="$2"
      shift 2
      ;;
    --post-ba-step-guard-confidence-max-pose-step-m)
      POST_BA_STEP_GUARD_CONFIDENCE_MAX_POSE_STEP_M="$2"
      shift 2
      ;;
    --post-ba-step-guard-confidence-warmup-marginalizations)
      POST_BA_STEP_GUARD_CONFIDENCE_WARMUP_MARGINALIZATIONS="$2"
      shift 2
      ;;
    --post-ba-step-guard-min-lidar-confidence)
      POST_BA_STEP_GUARD_MIN_LIDAR_CONFIDENCE="$2"
      shift 2
      ;;
    --post-ba-step-guard-min-visual-inlier-ratio)
      POST_BA_STEP_GUARD_MIN_VISUAL_INLIER_RATIO="$2"
      shift 2
      ;;
    --post-ba-step-guard-max-visual-residual)
      POST_BA_STEP_GUARD_MAX_VISUAL_RESIDUAL="$2"
      shift 2
      ;;
    --post-ba-step-guard-min-visual-coverage-tiles)
      POST_BA_STEP_GUARD_MIN_VISUAL_COVERAGE_TILES="$2"
      shift 2
      ;;
    --post-ba-step-guard-reject-to-pre-ba-over-m)
      POST_BA_STEP_GUARD_REJECT_TO_PRE_BA_OVER_M="$2"
      shift 2
      ;;
    --post-ba-step-guard-pre-ba-agreement-max-pose-step-m)
      POST_BA_STEP_GUARD_PRE_BA_AGREEMENT_MAX_POSE_STEP_M="$2"
      shift 2
      ;;
    --post-ba-step-guard-pre-ba-agreement-late-start-marginalizations)
      POST_BA_STEP_GUARD_PRE_BA_AGREEMENT_LATE_START_MARGINALIZATIONS="$2"
      shift 2
      ;;
    --post-ba-step-guard-pre-ba-agreement-late-max-pose-step-m)
      POST_BA_STEP_GUARD_PRE_BA_AGREEMENT_LATE_MAX_POSE_STEP_M="$2"
      shift 2
      ;;
    --post-ba-step-guard-pre-ba-agreement-min-cosine)
      POST_BA_STEP_GUARD_PRE_BA_AGREEMENT_MIN_COSINE="$2"
      shift 2
      ;;
    --post-ba-step-guard-pre-ba-agreement-max-delta-m)
      POST_BA_STEP_GUARD_PRE_BA_AGREEMENT_MAX_DELTA_M="$2"
      shift 2
      ;;
    --post-ba-step-guard-pre-ba-agreement-margin-m)
      POST_BA_STEP_GUARD_PRE_BA_AGREEMENT_MARGIN_M="$2"
      shift 2
      ;;
    --post-ba-step-guard-pre-ba-blend-on-clamp)
      POST_BA_STEP_GUARD_PRE_BA_BLEND_ON_CLAMP="$2"
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
    --sliding-window-optimize-every-n-frames)
      SLIDING_WINDOW_OPTIMIZE_EVERY_N_FRAMES="$2"
      shift 2
      ;;
    --sliding-window-max-states)
      SLIDING_WINDOW_MAX_STATES="$2"
      shift 2
      ;;
    --sliding-window-max-state-gap-s)
      SLIDING_WINDOW_MAX_STATE_GAP_S="$2"
      shift 2
      ;;
    --sliding-window-marginalization-prior-weight)
      SLIDING_WINDOW_MARGINALIZATION_PRIOR_WEIGHT="$2"
      shift 2
      ;;
    --sliding-window-max-condition)
      SLIDING_WINDOW_MAX_NORMAL_EQUATION_CONDITION="$2"
      shift 2
      ;;
    --sliding-window-max-translation-step-m)
      SLIDING_WINDOW_MAX_TRANSLATION_STEP_M="$2"
      shift 2
      ;;
    --sliding-window-max-feedback-translation-m)
      SLIDING_WINDOW_MAX_FEEDBACK_TRANSLATION_M="$2"
      shift 2
      ;;
    --sliding-window-max-feedback-rotation-rad)
      SLIDING_WINDOW_MAX_FEEDBACK_ROTATION_RAD="$2"
      shift 2
      ;;
    --sliding-window-max-feedback-velocity-mps)
      SLIDING_WINDOW_MAX_FEEDBACK_VELOCITY_MPS="$2"
      shift 2
      ;;
    --sliding-window-max-feedback-gyro-bias-step)
      SLIDING_WINDOW_MAX_FEEDBACK_GYRO_BIAS_STEP="$2"
      SLIDING_WINDOW_MAX_FEEDBACK_GYRO_BIAS_STEP_EXPLICIT=true
      shift 2
      ;;
    --sliding-window-max-feedback-accel-bias-step)
      SLIDING_WINDOW_MAX_FEEDBACK_ACCEL_BIAS_STEP="$2"
      SLIDING_WINDOW_MAX_FEEDBACK_ACCEL_BIAS_STEP_EXPLICIT=true
      shift 2
      ;;
    --sliding-window-min-bias-feedback-visual-factors)
      SLIDING_WINDOW_MIN_BIAS_FEEDBACK_VISUAL_FACTORS="$2"
      shift 2
      ;;
    --sliding-window-bias-feedback-ownership)
      SLIDING_WINDOW_BIAS_FEEDBACK_OWNERSHIP="$2"
      shift 2
      ;;
    --sliding-window-sync-guarded-pose-state)
      SLIDING_WINDOW_SYNC_GUARDED_POSE_STATE=true
      shift
      ;;
    --sliding-window-guarded-pose-prior-weights)
      SLIDING_WINDOW_GUARDED_POSE_PRIOR_TRANSLATION_WEIGHT="$2"
      SLIDING_WINDOW_GUARDED_POSE_PRIOR_ROTATION_WEIGHT="$3"
      shift 3
      ;;
    --sliding-window-imu-weight)
      SLIDING_WINDOW_IMU_WEIGHT="$2"
      shift 2
      ;;
    --sliding-window-imu-rotation-weight)
      SLIDING_WINDOW_IMU_ROTATION_WEIGHT="$2"
      shift 2
      ;;
    --sliding-window-imu-velocity-weight)
      SLIDING_WINDOW_IMU_VELOCITY_WEIGHT="$2"
      shift 2
      ;;
    --sliding-window-imu-position-weight)
      SLIDING_WINDOW_IMU_POSITION_WEIGHT="$2"
      shift 2
      ;;
    --sliding-window-imu-velocity-prior-weight)
      SLIDING_WINDOW_IMU_VELOCITY_PRIOR_WEIGHT="$2"
      shift 2
      ;;
    --sliding-window-gyro-bias-prior-weight)
      SLIDING_WINDOW_GYRO_BIAS_PRIOR_WEIGHT="$2"
      shift 2
      ;;
    --sliding-window-accel-bias-prior-weight)
      SLIDING_WINDOW_ACCEL_BIAS_PRIOR_WEIGHT="$2"
      shift 2
      ;;
    --sliding-window-bias-weight)
      SLIDING_WINDOW_BIAS_WEIGHT="$2"
      shift 2
      ;;
    --sliding-window-gyro-bias-weight)
      SLIDING_WINDOW_GYRO_BIAS_WEIGHT="$2"
      shift 2
      ;;
    --sliding-window-accel-bias-weight)
      SLIDING_WINDOW_ACCEL_BIAS_WEIGHT="$2"
      shift 2
      ;;
    --sliding-window-bias-random-walk-reference-dt-s)
      SLIDING_WINDOW_BIAS_RANDOM_WALK_REFERENCE_DT_S="$2"
      shift 2
      ;;
    --sliding-window-gyro-bias-random-walk-sigma)
      SLIDING_WINDOW_GYRO_BIAS_RANDOM_WALK_SIGMA="$2"
      shift 2
      ;;
    --sliding-window-accel-bias-random-walk-sigma)
      SLIDING_WINDOW_ACCEL_BIAS_RANDOM_WALK_SIGMA="$2"
      shift 2
      ;;
    --sliding-window-pose-translation-weight)
      SLIDING_WINDOW_POSE_TRANSLATION_WEIGHT="$2"
      shift 2
      ;;
    --sliding-window-pose-rotation-weight)
      SLIDING_WINDOW_POSE_ROTATION_WEIGHT="$2"
      shift 2
      ;;
    --sliding-window-smoothness-rotation-weight)
      SLIDING_WINDOW_SMOOTHNESS_ROTATION_WEIGHT="$2"
      shift 2
      ;;
    --sliding-window-smoothness-position-weight)
      SLIDING_WINDOW_SMOOTHNESS_POSITION_WEIGHT="$2"
      shift 2
      ;;
    --sliding-window-smoothness-velocity-weight)
      SLIDING_WINDOW_SMOOTHNESS_VELOCITY_WEIGHT="$2"
      shift 2
      ;;
    --sliding-window-smoothness-position-velocity-weight)
      SLIDING_WINDOW_SMOOTHNESS_POSITION_VELOCITY_WEIGHT="$2"
      shift 2
      ;;
    --sliding-window-smoothness-bias-weight)
      SLIDING_WINDOW_SMOOTHNESS_BIAS_WEIGHT="$2"
      shift 2
      ;;
    --sliding-window-smoothness-use-motion-targets)
      SLIDING_WINDOW_SMOOTHNESS_USE_MOTION_TARGETS=true
      shift
      ;;
    --sliding-window-smoothness-motion-target-min-visual-factors)
      SLIDING_WINDOW_SMOOTHNESS_MOTION_TARGET_MIN_VISUAL_FACTORS="$2"
      shift 2
      ;;
    --sliding-window-smoothness-motion-target-min-se3-photometric-factors)
      SLIDING_WINDOW_SMOOTHNESS_MOTION_TARGET_MIN_SE3_PHOTOMETRIC_FACTORS="$2"
      shift 2
      ;;
    --sliding-window-smoothness-motion-target-recent-window)
      SLIDING_WINDOW_SMOOTHNESS_MOTION_TARGET_RECENT_WINDOW="$2"
      shift 2
      ;;
    --sliding-window-smoothness-motion-target-min-recent-visual-factors)
      SLIDING_WINDOW_SMOOTHNESS_MOTION_TARGET_MIN_RECENT_VISUAL_FACTORS="$2"
      shift 2
      ;;
    --sliding-window-smoothness-motion-target-min-recent-se3-photometric-factors)
      SLIDING_WINDOW_SMOOTHNESS_MOTION_TARGET_MIN_RECENT_SE3_PHOTOMETRIC_FACTORS="$2"
      shift 2
      ;;
    --sliding-window-smoothness-motion-target-start-after-s)
      SLIDING_WINDOW_SMOOTHNESS_MOTION_TARGET_START_AFTER_S="$2"
      shift 2
      ;;
    --sliding-window-smoothness-motion-target-max-rotation-rate-delta-radps)
      SLIDING_WINDOW_SMOOTHNESS_MOTION_TARGET_MAX_ROTATION_RATE_DELTA_RADPS="$2"
      shift 2
      ;;
    --sliding-window-smoothness-motion-target-max-position-rate-delta-mps)
      SLIDING_WINDOW_SMOOTHNESS_MOTION_TARGET_MAX_POSITION_RATE_DELTA_MPS="$2"
      shift 2
      ;;
    --sliding-window-smoothness-motion-target-max-velocity-acceleration-delta-mps2)
      SLIDING_WINDOW_SMOOTHNESS_MOTION_TARGET_MAX_VELOCITY_ACCELERATION_DELTA_MPS2="$2"
      shift 2
      ;;
    --enable-sliding-window-relative-translation-factor)
      ENABLE_SLIDING_WINDOW_RELATIVE_TRANSLATION_FACTOR=true
      shift
      ;;
    --sliding-window-relative-translation-weight)
      SLIDING_WINDOW_RELATIVE_TRANSLATION_WEIGHT="$2"
      shift 2
      ;;
    --sliding-window-relative-translation-huber-delta-m)
      SLIDING_WINDOW_RELATIVE_TRANSLATION_HUBER_DELTA_M="$2"
      shift 2
      ;;
    --sliding-window-relative-translation-in-from-frame)
      SLIDING_WINDOW_RELATIVE_TRANSLATION_IN_FROM_FRAME=true
      shift
      ;;
    --sliding-window-relative-rotation-weight)
      SLIDING_WINDOW_RELATIVE_ROTATION_WEIGHT="$2"
      shift 2
      ;;
    --sliding-window-relative-rotation-huber-delta-rad)
      SLIDING_WINDOW_RELATIVE_ROTATION_HUBER_DELTA_RAD="$2"
      shift 2
      ;;
    --enable-sliding-window-relative-distance-factor)
      ENABLE_SLIDING_WINDOW_RELATIVE_DISTANCE_FACTOR=true
      shift
      ;;
    --sliding-window-relative-distance-weight)
      SLIDING_WINDOW_RELATIVE_DISTANCE_WEIGHT="$2"
      shift 2
      ;;
    --sliding-window-relative-distance-huber-delta-m)
      SLIDING_WINDOW_RELATIVE_DISTANCE_HUBER_DELTA_M="$2"
      shift 2
      ;;
    --enable-sliding-window-multihop-relative-translation-factor)
      ENABLE_SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_FACTOR=true
      shift
      ;;
    --sliding-window-multihop-relative-translation-weight)
      SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_WEIGHT="$2"
      shift 2
      ;;
    --sliding-window-multihop-relative-translation-huber-delta-m)
      SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_HUBER_DELTA_M="$2"
      shift 2
      ;;
    --sliding-window-multihop-relative-translation-in-from-frame)
      SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_IN_FROM_FRAME=true
      shift
      ;;
    --sliding-window-multihop-relative-rotation-weight)
      SLIDING_WINDOW_MULTIHOP_RELATIVE_ROTATION_WEIGHT="$2"
      shift 2
      ;;
    --sliding-window-multihop-relative-rotation-huber-delta-rad)
      SLIDING_WINDOW_MULTIHOP_RELATIVE_ROTATION_HUBER_DELTA_RAD="$2"
      shift 2
      ;;
    --enable-sliding-window-multihop-relative-distance-factor)
      ENABLE_SLIDING_WINDOW_MULTIHOP_RELATIVE_DISTANCE_FACTOR=true
      shift
      ;;
    --sliding-window-multihop-relative-distance-weight)
      SLIDING_WINDOW_MULTIHOP_RELATIVE_DISTANCE_WEIGHT="$2"
      shift 2
      ;;
    --sliding-window-multihop-relative-distance-huber-delta-m)
      SLIDING_WINDOW_MULTIHOP_RELATIVE_DISTANCE_HUBER_DELTA_M="$2"
      shift 2
      ;;
    --sliding-window-multihop-relative-translation-min-dt-s)
      SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_MIN_DT_S="$2"
      shift 2
      ;;
    --sliding-window-multihop-relative-translation-max-dt-s)
      SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_MAX_DT_S="$2"
      shift 2
      ;;
    --sliding-window-multihop-relative-translation-max-factors)
      SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_MAX_FACTORS="$2"
      shift 2
      ;;
    --enable-sliding-window-delayed-published-multihop-relative-translation-factor)
      ENABLE_SLIDING_WINDOW_DELAYED_PUBLISHED_MULTIHOP_RELATIVE_TRANSLATION_FACTOR=true
      shift
      ;;
    --sliding-window-delayed-published-multihop-start-after-s)
      SLIDING_WINDOW_DELAYED_PUBLISHED_MULTIHOP_START_AFTER_S="$2"
      shift 2
      ;;
    --sliding-window-delayed-published-multihop-max-factors)
      SLIDING_WINDOW_DELAYED_PUBLISHED_MULTIHOP_MAX_FACTORS="$2"
      shift 2
      ;;
    --sliding-window-relative-motion-history-source)
      SLIDING_WINDOW_RELATIVE_MOTION_HISTORY_SOURCE="$2"
      shift 2
      ;;
    --sliding-window-relative-motion-history-published-after-s)
      SLIDING_WINDOW_RELATIVE_MOTION_HISTORY_PUBLISHED_AFTER_S="$2"
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
    --enable-visual-factor-time-interpolation)
      ENABLE_VISUAL_FACTOR_TIME_INTERPOLATION=true
      ENABLE_VISUAL_FACTORS=true
      shift
      ;;
    --disable-visual-factor-time-interpolation)
      ENABLE_VISUAL_FACTOR_TIME_INTERPOLATION=false
      shift
      ;;
    --enable-visual-cache-reconciliation)
      ENABLE_VISUAL_CACHE_RECONCILIATION=true
      ENABLE_VISUAL_FACTORS=true
      shift
      ;;
    --disable-visual-cache-reconciliation)
      ENABLE_VISUAL_CACHE_RECONCILIATION=false
      shift
      ;;
    --enable-visual-cache-reconciliation-monotonic-unique)
      ENABLE_VISUAL_CACHE_RECONCILIATION=true
      ENABLE_VISUAL_CACHE_RECONCILIATION_MONOTONIC_UNIQUE=true
      ENABLE_VISUAL_FACTORS=true
      shift
      ;;
    --disable-visual-cache-reconciliation-monotonic-unique)
      ENABLE_VISUAL_CACHE_RECONCILIATION_MONOTONIC_UNIQUE=false
      shift
      ;;
    --enable-visual-pair-monotonic-unique)
      ENABLE_VISUAL_PAIR_MONOTONIC_UNIQUE=true
      ENABLE_VISUAL_FACTORS=true
      shift
      ;;
    --disable-visual-pair-monotonic-unique)
      ENABLE_VISUAL_PAIR_MONOTONIC_UNIQUE=false
      shift
      ;;
    --enable-visual-watermark-pair-scheduler)
      ENABLE_VISUAL_WATERMARK_PAIR_SCHEDULER=true
      ENABLE_VISUAL_FACTORS=true
      shift
      ;;
    --disable-visual-watermark-pair-scheduler)
      ENABLE_VISUAL_WATERMARK_PAIR_SCHEDULER=false
      shift
      ;;
    --visual-watermark-pair-scheduler-max-pairs-per-pointcloud)
      VISUAL_WATERMARK_PAIR_SCHEDULER_MAX_PAIRS_PER_POINTCLOUD="$2"
      shift 2
      ;;
    --enable-visual-callback-factor-ingest)
      ENABLE_VISUAL_CALLBACK_FACTOR_INGEST=true
      ENABLE_VISUAL_FACTORS=true
      shift
      ;;
    --disable-visual-callback-factor-ingest)
      ENABLE_VISUAL_CALLBACK_FACTOR_INGEST=false
      shift
      ;;
    --enable-rendered-feedback-watermark-queue)
      ENABLE_RENDERED_FEEDBACK_WATERMARK_QUEUE=true
      ENABLE_RENDERED_FEEDBACK_CONTRACT=true
      ENABLE_VISUAL_FACTORS=true
      shift
      ;;
    --disable-rendered-feedback-watermark-queue)
      ENABLE_RENDERED_FEEDBACK_WATERMARK_QUEUE=false
      shift
      ;;
    --defer-future-visual-factors-until-active)
      DEFER_FUTURE_VISUAL_FACTORS_UNTIL_ACTIVE=true
      ENABLE_VISUAL_FACTORS=true
      shift
      ;;
    --no-defer-future-visual-factors-until-active)
      DEFER_FUTURE_VISUAL_FACTORS_UNTIL_ACTIVE=false
      shift
      ;;
    --enable-visual-adaptive-state-retention)
      ENABLE_VISUAL_ADAPTIVE_STATE_RETENTION=true
      ENABLE_VISUAL_FACTORS=true
      shift
      ;;
    --disable-visual-adaptive-state-retention)
      ENABLE_VISUAL_ADAPTIVE_STATE_RETENTION=false
      shift
      ;;
    --visual-adaptive-state-retention-margin-states)
      VISUAL_ADAPTIVE_STATE_RETENTION_MARGIN_STATES="$2"
      shift 2
      ;;
    --visual-adaptive-state-retention-max-states)
      VISUAL_ADAPTIVE_STATE_RETENTION_MAX_STATES="$2"
      shift 2
      ;;
    --enable-visual-expired-factor-projection)
      ENABLE_VISUAL_EXPIRED_FACTOR_PROJECTION=true
      ENABLE_VISUAL_FACTORS=true
      shift
      ;;
    --disable-visual-expired-factor-projection)
      ENABLE_VISUAL_EXPIRED_FACTOR_PROJECTION=false
      shift
      ;;
    --enable-visual-marginalization-prior)
      ENABLE_VISUAL_MARGINALIZATION_PRIOR=true
      ENABLE_VISUAL_FACTORS=true
      shift
      ;;
    --disable-visual-marginalization-prior)
      ENABLE_VISUAL_MARGINALIZATION_PRIOR=false
      shift
      ;;
    --enable-visual-marginalization-prior-batching)
      ENABLE_VISUAL_MARGINALIZATION_PRIOR_BATCHING=true
      ENABLE_VISUAL_MARGINALIZATION_PRIOR=true
      ENABLE_VISUAL_FACTORS=true
      shift
      ;;
    --disable-visual-marginalization-prior-batching)
      ENABLE_VISUAL_MARGINALIZATION_PRIOR_BATCHING=false
      shift
      ;;
    --enable-visual-marginalization-prior-saturation-gate)
      ENABLE_VISUAL_MARGINALIZATION_PRIOR_SATURATION_GATE=true
      ENABLE_VISUAL_MARGINALIZATION_PRIOR=true
      ENABLE_VISUAL_FACTORS=true
      shift
      ;;
    --disable-visual-marginalization-prior-saturation-gate)
      ENABLE_VISUAL_MARGINALIZATION_PRIOR_SATURATION_GATE=false
      shift
      ;;
    --visual-marginalization-prior-zero-bias-columns)
      VISUAL_MARGINALIZATION_PRIOR_ZERO_BIAS_COLUMNS=true
      ENABLE_VISUAL_MARGINALIZATION_PRIOR=true
      ENABLE_VISUAL_FACTORS=true
      shift
      ;;
    --no-visual-marginalization-prior-zero-bias-columns)
      VISUAL_MARGINALIZATION_PRIOR_ZERO_BIAS_COLUMNS=false
      shift
      ;;
    --enable-visual-factor-reference-snapshot)
      ENABLE_VISUAL_FACTOR_REFERENCE_SNAPSHOT=true
      ENABLE_VISUAL_FACTORS=true
      shift
      ;;
    --disable-visual-factor-reference-snapshot)
      ENABLE_VISUAL_FACTOR_REFERENCE_SNAPSHOT=false
      shift
      ;;
    --enable-rendered-feedback-source-pose-reference)
      ENABLE_RENDERED_FEEDBACK_SOURCE_POSE_REFERENCE=true
      ENABLE_RENDERED_FEEDBACK_CONTRACT=true
      ENABLE_VISUAL_FACTORS=true
      shift
      ;;
    --disable-rendered-feedback-source-pose-reference)
      ENABLE_RENDERED_FEEDBACK_SOURCE_POSE_REFERENCE=false
      shift
      ;;
    --enable-rendered-feedback-source-motion-factor)
      ENABLE_RENDERED_FEEDBACK_SOURCE_MOTION_FACTOR=true
      ENABLE_RENDERED_FEEDBACK_CONTRACT=true
      ENABLE_VISUAL_FACTORS=true
      shift
      ;;
    --disable-rendered-feedback-source-motion-factor)
      ENABLE_RENDERED_FEEDBACK_SOURCE_MOTION_FACTOR=false
      shift
      ;;
    --enable-rendered-feedback-source-motion-marginalized-prior)
      ENABLE_RENDERED_FEEDBACK_SOURCE_MOTION_MARGINALIZED_PRIOR=true
      ENABLE_RENDERED_FEEDBACK_SOURCE_MOTION_FACTOR=true
      ENABLE_RENDERED_FEEDBACK_CONTRACT=true
      ENABLE_VISUAL_FACTORS=true
      shift
      ;;
    --disable-rendered-feedback-source-motion-marginalized-prior)
      ENABLE_RENDERED_FEEDBACK_SOURCE_MOTION_MARGINALIZED_PRIOR=false
      shift
      ;;
    --rendered-feedback-source-motion-translation-weight)
      RENDERED_FEEDBACK_SOURCE_MOTION_TRANSLATION_WEIGHT="$2"
      shift 2
      ;;
    --rendered-feedback-source-motion-rotation-weight)
      RENDERED_FEEDBACK_SOURCE_MOTION_ROTATION_WEIGHT="$2"
      shift 2
      ;;
    --rendered-feedback-source-motion-huber-delta-m)
      RENDERED_FEEDBACK_SOURCE_MOTION_HUBER_DELTA_M="$2"
      shift 2
      ;;
    --rendered-feedback-source-motion-rotation-huber-delta-rad)
      RENDERED_FEEDBACK_SOURCE_MOTION_ROTATION_HUBER_DELTA_RAD="$2"
      shift 2
      ;;
    --rendered-feedback-source-motion-min-dt-s)
      RENDERED_FEEDBACK_SOURCE_MOTION_MIN_DT_S="$2"
      shift 2
      ;;
    --rendered-feedback-source-motion-max-dt-s)
      RENDERED_FEEDBACK_SOURCE_MOTION_MAX_DT_S="$2"
      shift 2
      ;;
    --rendered-feedback-source-motion-in-from-frame)
      RENDERED_FEEDBACK_SOURCE_MOTION_IN_FROM_FRAME=true
      shift
      ;;
    --no-rendered-feedback-source-motion-in-from-frame)
      RENDERED_FEEDBACK_SOURCE_MOTION_IN_FROM_FRAME=false
      shift
      ;;
    --visual-expired-factor-projection-max-age-s)
      VISUAL_EXPIRED_FACTOR_PROJECTION_MAX_AGE_S="$2"
      shift 2
      ;;
    --enable-visual-cache-reconciliation-defer-to-pointcloud)
      ENABLE_VISUAL_CACHE_RECONCILIATION=true
      ENABLE_VISUAL_CACHE_RECONCILIATION_DEFER_TO_POINTCLOUD=true
      ENABLE_VISUAL_FACTORS=true
      shift
      ;;
    --disable-visual-cache-reconciliation-defer-to-pointcloud)
      ENABLE_VISUAL_CACHE_RECONCILIATION_DEFER_TO_POINTCLOUD=false
      shift
      ;;
    --enable-visual-pair-processing-defer-to-pointcloud)
      ENABLE_VISUAL_CACHE_RECONCILIATION=true
      ENABLE_VISUAL_PAIR_PROCESSING_DEFER_TO_POINTCLOUD=true
      ENABLE_VISUAL_FACTORS=true
      shift
      ;;
    --disable-visual-pair-processing-defer-to-pointcloud)
      ENABLE_VISUAL_PAIR_PROCESSING_DEFER_TO_POINTCLOUD=false
      shift
      ;;
    --enable-mapper-feedback)
      ENABLE_MAPPER_FEEDBACK=true
      ENABLE_VISUAL_FACTORS=true
      shift
      ;;
    --enable-gaussian-map-feedback)
      ENABLE_MAPPER_FEEDBACK=true
      ENABLE_GAUSSIAN_MAP_FEEDBACK=true
      ENABLE_VISUAL_FACTORS=true
      MAPPER_FEEDBACK_PUBLISH_GAUSSIAN_MAP=true
      MAPPER_FEEDBACK_ENABLE_TORCH_CAMERA_CONVERSION=true
      MAPPER_FEEDBACK_ENABLE_TORCH_GAUSSIAN_INIT=true
      MAPPER_FEEDBACK_ENABLE_TORCH_GAUSSIAN_EXTEND=true
      MAPPER_FEEDBACK_ENABLE_TORCH_GAUSSIAN_EXTEND_VISIBILITY_FILTER=false
      MAPPER_FEEDBACK_ENABLE_TORCH_GAUSSIAN_PRUNING=true
      MAPPER_FEEDBACK_TORCH_DEVICE=auto
      if [[ "${MAPPER_FEEDBACK_SELECT_EVERY_K_FRAME_EXPLICIT}" != "true" ]]; then
        MAPPER_FEEDBACK_SELECT_EVERY_K_FRAME=1
      fi
      if [[ "${MAPPER_FEEDBACK_RENDER_MODE_EXPLICIT}" != "true" ]]; then
        MAPPER_FEEDBACK_RENDER_MODE=rasterizer
      fi
      if [[ "${MAPPER_FEEDBACK_POINTCLOUD_COORDINATES_EXPLICIT}" != "true" ]]; then
        MAPPER_FEEDBACK_POINTCLOUD_COORDINATES=sensor
      fi
      if [[ "${MAPPER_FEEDBACK_MAX_DEPTH_EXPLICIT}" != "true" ]]; then
        MAPPER_FEEDBACK_MAX_DEPTH=20.0
      fi
      if [[ "${PLAYBACK_RATE_EXPLICIT}" != "true" ]]; then
        PLAYBACK_RATE=0.25
      fi
      LIDAR_MAX_FRAME_POINTS=500
      TRACKING_MAX_POSE_STEP_M=0.0265
      SLIDING_WINDOW_OPTIMIZE_EVERY_N_FRAMES=4
      SLIDING_WINDOW_MAX_FEEDBACK_TRANSLATION_M=0.05
      SLIDING_WINDOW_MAX_FEEDBACK_VELOCITY_MPS=0.5
      if [[ "${SLIDING_WINDOW_MAX_FEEDBACK_GYRO_BIAS_STEP_EXPLICIT}" != "true" ]]; then
        SLIDING_WINDOW_MAX_FEEDBACK_GYRO_BIAS_STEP=0.02
      fi
      if [[ "${SLIDING_WINDOW_MAX_FEEDBACK_ACCEL_BIAS_STEP_EXPLICIT}" != "true" ]]; then
        SLIDING_WINDOW_MAX_FEEDBACK_ACCEL_BIAS_STEP=0.12
      fi
      LIDAR_WINDOW_PLANE_FACTOR_WEIGHT=4.0
      POST_BA_STEP_GUARD_PRE_BA_AGREEMENT_MAX_POSE_STEP_M=0.045
      POST_BA_STEP_GUARD_PRE_BA_AGREEMENT_MIN_COSINE=0.95
      POST_BA_STEP_GUARD_PRE_BA_AGREEMENT_MAX_DELTA_M=0.02
      ENABLE_SLIDING_WINDOW_RELATIVE_TRANSLATION_FACTOR=true
      SLIDING_WINDOW_RELATIVE_TRANSLATION_WEIGHT=0.1
      SLIDING_WINDOW_RELATIVE_TRANSLATION_HUBER_DELTA_M=0.1
      SLIDING_WINDOW_RELATIVE_ROTATION_WEIGHT=0.01
      ENABLE_SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_FACTOR=true
      SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_WEIGHT=0.03
      SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_MAX_FACTORS=6
      SLIDING_WINDOW_MULTIHOP_RELATIVE_ROTATION_WEIGHT=0.005
      VISUAL_ALIGNMENT_WINDOW_WEIGHT=0.25
      SE3_PHOTOMETRIC_WINDOW_WEIGHT=0.5
      REQUIRE_GAUSSIAN_SNAPSHOT=true
      shift
      ;;
    --require-gaussian-snapshot)
      REQUIRE_GAUSSIAN_SNAPSHOT=true
      shift
      ;;
    --gaussian-snapshot-lidar-factor-weight)
      GAUSSIAN_SNAPSHOT_LIDAR_FACTOR_WEIGHT="$2"
      shift 2
      ;;
    --gaussian-snapshot-lidar-nearest-distance-m)
      GAUSSIAN_SNAPSHOT_LIDAR_NEAREST_DISTANCE_M="$2"
      shift 2
      ;;
    --disable-gaussian-snapshot-lidar-residual-preweight)
      GAUSSIAN_SNAPSHOT_LIDAR_RESIDUAL_PREWEIGHT=false
      shift
      ;;
    --enable-gaussian-snapshot-lidar-pose-correction)
      ENABLE_GAUSSIAN_SNAPSHOT_LIDAR_POSE_CORRECTION=true
      shift
      ;;
    --gaussian-snapshot-lidar-pose-correction-gain)
      GAUSSIAN_SNAPSHOT_LIDAR_POSE_CORRECTION_GAIN="$2"
      shift 2
      ;;
    --gaussian-snapshot-lidar-pose-correction-max-translation-m)
      GAUSSIAN_SNAPSHOT_LIDAR_POSE_CORRECTION_MAX_TRANSLATION_M="$2"
      shift 2
      ;;
    --gaussian-snapshot-lidar-pose-correction-max-rotation-rad)
      GAUSSIAN_SNAPSHOT_LIDAR_POSE_CORRECTION_MAX_ROTATION_RAD="$2"
      shift 2
      ;;
    --gaussian-snapshot-lidar-pose-correction-min-match-ratio)
      GAUSSIAN_SNAPSHOT_LIDAR_POSE_CORRECTION_MIN_MATCH_RATIO="$2"
      shift 2
      ;;
    --gaussian-snapshot-lidar-pose-correction-max-mean-residual-m)
      GAUSSIAN_SNAPSHOT_LIDAR_POSE_CORRECTION_MAX_MEAN_RESIDUAL_M="$2"
      shift 2
      ;;
    --gaussian-snapshot-lidar-pose-correction-coverage-grid-cols)
      GAUSSIAN_SNAPSHOT_LIDAR_POSE_CORRECTION_COVERAGE_GRID_COLS="$2"
      shift 2
      ;;
    --gaussian-snapshot-lidar-pose-correction-coverage-grid-rows)
      GAUSSIAN_SNAPSHOT_LIDAR_POSE_CORRECTION_COVERAGE_GRID_ROWS="$2"
      shift 2
      ;;
    --gaussian-snapshot-lidar-pose-correction-min-coverage-tiles)
      GAUSSIAN_SNAPSHOT_LIDAR_POSE_CORRECTION_MIN_COVERAGE_TILES="$2"
      shift 2
      ;;
    --gaussian-snapshot-lidar-pose-correction-bidirectional-max-distance-m)
      GAUSSIAN_SNAPSHOT_LIDAR_POSE_CORRECTION_BIDIRECTIONAL_MAX_DISTANCE_M="$2"
      shift 2
      ;;
    --enable-gaussian-snapshot-lidar-plane-factor)
      ENABLE_GAUSSIAN_SNAPSHOT_LIDAR_PLANE_FACTOR=true
      shift
      ;;
    --gaussian-snapshot-lidar-plane-factor-weight)
      GAUSSIAN_SNAPSHOT_LIDAR_PLANE_FACTOR_WEIGHT="$2"
      shift 2
      ;;
    --gaussian-snapshot-lidar-plane-min-anisotropy)
      GAUSSIAN_SNAPSHOT_LIDAR_PLANE_MIN_ANISOTROPY="$2"
      shift 2
      ;;
    --mapper-feedback-sync-tolerance-sec)
      MAPPER_FEEDBACK_SYNC_TOLERANCE_SEC="$2"
      shift 2
      ;;
    --mapper-feedback-sync-anchor)
      MAPPER_FEEDBACK_SYNC_ANCHOR_STREAM="$2"
      shift 2
      ;;
    --mapper-feedback-image-qos-reliability)
      MAPPER_FEEDBACK_IMAGE_QOS_RELIABILITY="$2"
      MAPPER_FEEDBACK_IMAGE_QOS_RELIABILITY_EXPLICIT=true
      shift 2
      ;;
    --mapper-feedback-image-qos-depth)
      MAPPER_FEEDBACK_IMAGE_QOS_DEPTH="$2"
      MAPPER_FEEDBACK_IMAGE_QOS_DEPTH_EXPLICIT=true
      shift 2
      ;;
    --mapper-feedback-render-mode)
      MAPPER_FEEDBACK_RENDER_MODE="$2"
      MAPPER_FEEDBACK_RENDER_MODE_EXPLICIT=true
      shift 2
      ;;
    --mapper-feedback-pointcloud-coordinates)
      MAPPER_FEEDBACK_POINTCLOUD_COORDINATES="$2"
      MAPPER_FEEDBACK_POINTCLOUD_COORDINATES_EXPLICIT=true
      shift 2
      ;;
    --mapper-feedback-max-depth)
      MAPPER_FEEDBACK_MAX_DEPTH="$2"
      MAPPER_FEEDBACK_MAX_DEPTH_EXPLICIT=true
      shift 2
      ;;
    --mapper-feedback-allow-unprojected-color-fallback)
      MAPPER_FEEDBACK_REQUIRE_PROJECTED_POINT_COLOR=false
      shift
      ;;
    --mapper-feedback-zbuffer-projected-points)
      MAPPER_FEEDBACK_ZBUFFER_PROJECTED_POINTS=true
      shift
      ;;
    --mapper-feedback-gaussian-map-publish-min-interval-sec)
      MAPPER_FEEDBACK_GAUSSIAN_MAP_PUBLISH_MIN_INTERVAL_SEC="$2"
      shift 2
      ;;
    --mapper-feedback-gaussian-map-publish-on-empty-extend)
      MAPPER_FEEDBACK_GAUSSIAN_MAP_PUBLISH_ON_EMPTY_EXTEND=true
      shift
      ;;
    --mapper-feedback-select-every-k-frame)
      MAPPER_FEEDBACK_SELECT_EVERY_K_FRAME="$2"
      MAPPER_FEEDBACK_SELECT_EVERY_K_FRAME_EXPLICIT=true
      shift 2
      ;;
    --mapper-feedback-torch-device)
      MAPPER_FEEDBACK_TORCH_DEVICE="$2"
      shift 2
      ;;
    --mapper-feedback-torch-optimization-steps)
      MAPPER_FEEDBACK_TORCH_OPTIMIZATION_STEPS="$2"
      shift 2
      ;;
    --mapper-feedback-torch-optimization-every-n-keyframes)
      MAPPER_FEEDBACK_TORCH_OPTIMIZATION_EVERY_N_KEYFRAMES="$2"
      MAPPER_FEEDBACK_TORCH_OPTIMIZATION_EVERY_EXPLICIT=true
      shift 2
      ;;
    --mapper-feedback-torch-optimization-sampling)
      MAPPER_FEEDBACK_TORCH_OPTIMIZATION_SAMPLING="$2"
      MAPPER_FEEDBACK_TORCH_OPTIMIZATION_SAMPLING_EXPLICIT=true
      shift 2
      ;;
    --mapper-feedback-enable-torch-optimization)
      MAPPER_FEEDBACK_ENABLE_TORCH_GAUSSIAN_OPTIMIZATION=true
      shift
      ;;
    --mapper-feedback-gaussian-lr)
      MAPPER_FEEDBACK_POSITION_LR="$2"
      MAPPER_FEEDBACK_FEATURE_LR="$3"
      MAPPER_FEEDBACK_OPACITY_LR="$4"
      MAPPER_FEEDBACK_SCALING_LR="$5"
      MAPPER_FEEDBACK_ROTATION_LR="$6"
      MAPPER_FEEDBACK_LR_EXPLICIT=true
      shift 6
      ;;
    --mapper-feedback-torch-max-foreground)
      MAPPER_FEEDBACK_TORCH_MAX_FOREGROUND="$2"
      shift 2
      ;;
    --mapper-feedback-torch-prune-count-policy)
      MAPPER_FEEDBACK_TORCH_PRUNE_COUNT_POLICY="$2"
      shift 2
      ;;
    --mapper-feedback-enable-extend-visibility-filter)
      MAPPER_FEEDBACK_ENABLE_TORCH_GAUSSIAN_EXTEND_VISIBILITY_FILTER=true
      shift
      ;;
    --mapper-feedback-disable-extend-visibility-filter)
      MAPPER_FEEDBACK_ENABLE_TORCH_GAUSSIAN_EXTEND_VISIBILITY_FILTER=false
      shift
      ;;
    --mapper-feedback-publish-rendered-before-update)
      MAPPER_FEEDBACK_PUBLISH_RENDERED_BEFORE_UPDATE=true
      shift
      ;;
    --mapper-feedback-rendered-feedback-source-stream)
      MAPPER_FEEDBACK_RENDERED_FEEDBACK_SOURCE_STREAM="$2"
      shift 2
      ;;
    --mapper-feedback-rendered-feedback-image-topic)
      MAPPER_FEEDBACK_RENDERED_FEEDBACK_IMAGE_TOPIC="$2"
      shift 2
      ;;
    --mapper-feedback-rendered-feedback-pose-topic)
      MAPPER_FEEDBACK_RENDERED_FEEDBACK_POSE_TOPIC="$2"
      shift 2
      ;;
    --mapper-feedback-rendered-feedback-image-qos-reliability)
      MAPPER_FEEDBACK_RENDERED_FEEDBACK_IMAGE_QOS_RELIABILITY="$2"
      shift 2
      ;;
    --mapper-feedback-rendered-feedback-image-qos-history)
      MAPPER_FEEDBACK_RENDERED_FEEDBACK_IMAGE_QOS_HISTORY="$2"
      shift 2
      ;;
    --mapper-feedback-rendered-feedback-image-qos-depth)
      MAPPER_FEEDBACK_RENDERED_FEEDBACK_IMAGE_QOS_DEPTH="$2"
      shift 2
      ;;
    --mapper-feedback-rendered-feedback-pose-qos-reliability)
      MAPPER_FEEDBACK_RENDERED_FEEDBACK_POSE_QOS_RELIABILITY="$2"
      shift 2
      ;;
    --mapper-feedback-rendered-feedback-pose-qos-history)
      MAPPER_FEEDBACK_RENDERED_FEEDBACK_POSE_QOS_HISTORY="$2"
      shift 2
      ;;
    --mapper-feedback-rendered-feedback-pose-qos-depth)
      MAPPER_FEEDBACK_RENDERED_FEEDBACK_POSE_QOS_DEPTH="$2"
      shift 2
      ;;
    --rendered-image-qos-reliability)
      RENDERED_IMAGE_QOS_RELIABILITY="$2"
      shift 2
      ;;
    --rendered-image-qos-durability)
      RENDERED_IMAGE_QOS_DURABILITY="$2"
      shift 2
      ;;
    --rendered-image-qos-depth)
      RENDERED_IMAGE_QOS_DEPTH="$2"
      shift 2
      ;;
    --enable-rendered-feedback-contract)
      ENABLE_RENDERED_FEEDBACK_CONTRACT=true
      ENABLE_VISUAL_FACTORS=true
      shift
      ;;
    --disable-rendered-feedback-contract)
      ENABLE_RENDERED_FEEDBACK_CONTRACT=false
      shift
      ;;
    --rendered-feedback-topic)
      RENDERED_FEEDBACK_TOPIC="$2"
      shift 2
      ;;
    --rendered-feedback-qos-reliability)
      RENDERED_FEEDBACK_QOS_RELIABILITY="$2"
      shift 2
      ;;
    --rendered-feedback-qos-durability)
      RENDERED_FEEDBACK_QOS_DURABILITY="$2"
      shift 2
      ;;
    --rendered-feedback-qos-depth)
      RENDERED_FEEDBACK_QOS_DEPTH="$2"
      shift 2
      ;;
    --enable-rendered-feedback-ingress-queue)
      ENABLE_RENDERED_FEEDBACK_INGRESS_QUEUE=true
      shift
      ;;
    --disable-rendered-feedback-ingress-queue)
      ENABLE_RENDERED_FEEDBACK_INGRESS_QUEUE=false
      shift
      ;;
    --rendered-feedback-ingress-queue-size)
      RENDERED_FEEDBACK_INGRESS_QUEUE_SIZE="$2"
      shift 2
      ;;
    --rendered-feedback-ingress-drain-max-per-cycle)
      RENDERED_FEEDBACK_INGRESS_DRAIN_MAX_PER_CYCLE="$2"
      shift 2
      ;;
    --rendered-feedback-ingress-drain-period-ms)
      RENDERED_FEEDBACK_INGRESS_DRAIN_PERIOD_MS="$2"
      shift 2
      ;;
    --visual-factor-max-dt-ns)
      VISUAL_FACTOR_MAX_DT_NS="$2"
      shift 2
      ;;
    --visual-depth-max-dt-ns)
      VISUAL_DEPTH_MAX_DT_NS="$2"
      shift 2
      ;;
    --visual-depth-dilation-px)
      VISUAL_DEPTH_DILATION_PX="$2"
      shift 2
      ;;
    --rendered-frame-cache-size)
      RENDERED_FRAME_CACHE_SIZE="$2"
      shift 2
      ;;
    --observed-frame-cache-size)
      OBSERVED_FRAME_CACHE_SIZE="$2"
      shift 2
      ;;
    --visual-pending-factor-queue-size)
      VISUAL_PENDING_FACTOR_QUEUE_SIZE="$2"
      shift 2
      ;;
    --enable-visual-factor-quality-weighting)
      ENABLE_VISUAL_FACTOR_QUALITY_WEIGHTING=true
      shift
      ;;
    --disable-visual-factor-quality-weighting)
      ENABLE_VISUAL_FACTOR_QUALITY_WEIGHTING=false
      shift
      ;;
    --visual-factor-quality-min-weight-scale)
      VISUAL_FACTOR_QUALITY_MIN_WEIGHT_SCALE="$2"
      shift 2
      ;;
    --enable-visual-factor-quality-selection)
      ENABLE_VISUAL_FACTOR_QUALITY_SELECTION=true
      shift
      ;;
    --disable-visual-factor-quality-selection)
      ENABLE_VISUAL_FACTOR_QUALITY_SELECTION=false
      shift
      ;;
    --enable-visual-factor-quality-reference-cap)
      ENABLE_VISUAL_FACTOR_QUALITY_REFERENCE_CAP=true
      shift
      ;;
    --disable-visual-factor-quality-reference-cap)
      ENABLE_VISUAL_FACTOR_QUALITY_REFERENCE_CAP=false
      shift
      ;;
    --visual-factor-quality-selection-max-per-reference)
      VISUAL_FACTOR_QUALITY_SELECTION_MAX_PER_REFERENCE="$2"
      shift 2
      ;;
    --visual-factor-quality-selection-start-after-s)
      VISUAL_FACTOR_QUALITY_SELECTION_START_AFTER_S="$2"
      shift 2
      ;;
    --visual-alignment-max-shift-px)
      VISUAL_ALIGNMENT_MAX_SHIFT_PX="$2"
      shift 2
      ;;
    --visual-alignment-score-mode)
      VISUAL_ALIGNMENT_SCORE_MODE="$2"
      shift 2
      ;;
    --visual-alignment-factor-source)
      VISUAL_ALIGNMENT_FACTOR_SOURCE="$2"
      shift 2
      ;;
    --visual-factor-source-id-mode)
      VISUAL_FACTOR_SOURCE_ID_MODE="$2"
      shift 2
      ;;
    --visual-factor-reference-stamp-mode)
      VISUAL_FACTOR_REFERENCE_STAMP_MODE="$2"
      shift 2
      ;;
    --visual-alignment-window-weight)
      VISUAL_ALIGNMENT_WINDOW_WEIGHT="$2"
      shift 2
      ;;
    --visual-alignment-saturation-margin-px)
      VISUAL_ALIGNMENT_SATURATION_MARGIN_PX="$2"
      shift 2
      ;;
    --visual-alignment-saturated-weight-scale)
      VISUAL_ALIGNMENT_SATURATED_WEIGHT_SCALE="$2"
      shift 2
      ;;
    --se3-photometric-window-weight)
      SE3_PHOTOMETRIC_WINDOW_WEIGHT="$2"
      shift 2
      ;;
    --se3-photometric-max-samples)
      SE3_PHOTOMETRIC_MAX_SAMPLES="$2"
      shift 2
      ;;
    --se3-photometric-min-samples)
      SE3_PHOTOMETRIC_MIN_SAMPLES="$2"
      shift 2
      ;;
    --se3-photometric-min-hessian-rank)
      SE3_PHOTOMETRIC_MIN_HESSIAN_RANK="$2"
      shift 2
      ;;
    --se3-photometric-max-hessian-condition)
      SE3_PHOTOMETRIC_MAX_HESSIAN_CONDITION="$2"
      shift 2
      ;;
    --se3-photometric-min-sample-inlier-ratio)
      SE3_PHOTOMETRIC_MIN_SAMPLE_INLIER_RATIO="$2"
      shift 2
      ;;
    --se3-photometric-max-mean-abs-residual)
      SE3_PHOTOMETRIC_MAX_MEAN_ABS_RESIDUAL_FOR_FACTOR="$2"
      shift 2
      ;;
    --se3-photometric-use-rendered-gradient)
      SE3_PHOTOMETRIC_USE_RENDERED_GRADIENT=true
      shift
      ;;
    --se3-photometric-rank-samples-by-gradient)
      SE3_PHOTOMETRIC_RANK_SAMPLES_BY_GRADIENT=true
      shift
      ;;
    --se3-photometric-coverage-grid-cols)
      SE3_PHOTOMETRIC_COVERAGE_GRID_COLS="$2"
      shift 2
      ;;
    --se3-photometric-coverage-grid-rows)
      SE3_PHOTOMETRIC_COVERAGE_GRID_ROWS="$2"
      shift 2
      ;;
    --se3-photometric-min-coverage-tiles)
      SE3_PHOTOMETRIC_MIN_COVERAGE_TILES="$2"
      shift 2
      ;;
    --enable-se3-photometric-pose-correction)
      ENABLE_SE3_PHOTOMETRIC_POSE_CORRECTION=true
      shift
      ;;
    --se3-photometric-pose-correction-gain)
      SE3_PHOTOMETRIC_POSE_CORRECTION_GAIN="$2"
      shift 2
      ;;
    --se3-photometric-pose-correction-max-translation-m)
      SE3_PHOTOMETRIC_POSE_CORRECTION_MAX_TRANSLATION_M="$2"
      shift 2
      ;;
    --se3-photometric-pose-correction-max-rotation-rad)
      SE3_PHOTOMETRIC_POSE_CORRECTION_MAX_ROTATION_RAD="$2"
      shift 2
      ;;
    --se3-photometric-pose-correction-max-dt-ns)
      SE3_PHOTOMETRIC_POSE_CORRECTION_MAX_DT_NS="$2"
      shift 2
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
    --reference-tum)
      REFERENCE_TUM_PATH="$(realpath -m "$2")"
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
    --reference-coverage-mode)
      REFERENCE_COVERAGE_MODE="$2"
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
    --reference-min-current-path-ratio)
      REFERENCE_MIN_CURRENT_PATH_RATIO="$2"
      shift 2
      ;;
    --reference-max-current-path-ratio)
      REFERENCE_MAX_CURRENT_PATH_RATIO="$2"
      shift 2
      ;;
    --reference-error-bin-count)
      REFERENCE_ERROR_BIN_COUNT="$2"
      shift 2
      ;;
    --reference-min-error-bin-matches)
      REFERENCE_MIN_ERROR_BIN_MATCHES="$2"
      shift 2
      ;;
    --reference-max-error-bin-rmse-m)
      REFERENCE_MAX_ERROR_BIN_RMSE_M="$2"
      shift 2
      ;;
    --reference-max-error-bin-mean-m)
      REFERENCE_MAX_ERROR_BIN_MEAN_M="$2"
      shift 2
      ;;
    --reference-max-error-bin-bias-norm-m)
      REFERENCE_MAX_ERROR_BIN_BIAS_NORM_M="$2"
      shift 2
      ;;
    --reference-time-offset-sweep)
      REFERENCE_TIME_OFFSET_SWEEP_MIN="$2"
      REFERENCE_TIME_OFFSET_SWEEP_MAX="$3"
      REFERENCE_TIME_OFFSET_SWEEP_STEP="$4"
      shift 4
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

if (( MAPPER_FEEDBACK_TORCH_OPTIMIZATION_STEPS > 0 )); then
  MAPPER_FEEDBACK_ENABLE_TORCH_GAUSSIAN_OPTIMIZATION=true
  if [[
    "${ENABLE_GAUSSIAN_MAP_FEEDBACK}" == "true" &&
    "${MAPPER_FEEDBACK_TORCH_OPTIMIZATION_SAMPLING_EXPLICIT}" != "true"
  ]]; then
    MAPPER_FEEDBACK_TORCH_OPTIMIZATION_SAMPLING=latest_even
  fi
  if [[ "${ENABLE_GAUSSIAN_MAP_FEEDBACK}" == "true" && "${MAPPER_FEEDBACK_LR_EXPLICIT}" != "true" ]]; then
    MAPPER_FEEDBACK_POSITION_LR=0.00003
    MAPPER_FEEDBACK_FEATURE_LR=0.001
    MAPPER_FEEDBACK_OPACITY_LR=0.01
    MAPPER_FEEDBACK_SCALING_LR=0.001
    MAPPER_FEEDBACK_ROTATION_LR=0.0002
  fi
  if [[
    "${ENABLE_GAUSSIAN_MAP_FEEDBACK}" == "true" &&
    "${MAPPER_FEEDBACK_TORCH_OPTIMIZATION_EVERY_EXPLICIT}" != "true"
  ]]; then
    MAPPER_FEEDBACK_TORCH_OPTIMIZATION_EVERY_N_KEYFRAMES=16
  fi
fi

case "${MAPPER_FEEDBACK_SYNC_ANCHOR_STREAM}" in
  pointcloud|image)
    ;;
  *)
    echo "--mapper-feedback-sync-anchor must be pointcloud or image, got ${MAPPER_FEEDBACK_SYNC_ANCHOR_STREAM}" >&2
    exit 2
    ;;
esac
case "${MAPPER_FEEDBACK_IMAGE_QOS_RELIABILITY}" in
  reliable|best_effort|besteffort)
    ;;
  *)
    echo "--mapper-feedback-image-qos-reliability must be reliable or best_effort, got ${MAPPER_FEEDBACK_IMAGE_QOS_RELIABILITY}" >&2
    exit 2
    ;;
esac

case "${SLIDING_WINDOW_RELATIVE_MOTION_HISTORY_SOURCE}" in
  pre_ba|published|published_after_warmup)
    ;;
  *)
    echo "--sliding-window-relative-motion-history-source must be pre_ba, published, or published_after_warmup, got ${SLIDING_WINDOW_RELATIVE_MOTION_HISTORY_SOURCE}" >&2
    exit 2
    ;;
esac

for float_var in \
  MAPPER_FEEDBACK_MAX_DEPTH \
  MAPPER_FEEDBACK_SYNC_TOLERANCE_SEC \
  MAPPER_FEEDBACK_GAUSSIAN_MAP_PUBLISH_MIN_INTERVAL_SEC \
  MAPPER_FEEDBACK_POSITION_LR \
  MAPPER_FEEDBACK_FEATURE_LR \
  MAPPER_FEEDBACK_OPACITY_LR \
  MAPPER_FEEDBACK_SCALING_LR \
  MAPPER_FEEDBACK_ROTATION_LR \
  SLIDING_WINDOW_GUARDED_POSE_PRIOR_TRANSLATION_WEIGHT \
  SLIDING_WINDOW_GUARDED_POSE_PRIOR_ROTATION_WEIGHT \
  SLIDING_WINDOW_DELAYED_PUBLISHED_MULTIHOP_START_AFTER_S \
  SLIDING_WINDOW_RELATIVE_MOTION_HISTORY_PUBLISHED_AFTER_S; do
  printf -v "${float_var}" '%s' "$(canonical_float "${!float_var}")"
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
    stop_process_group "${record_pid}" INT
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
  enable_rendered_feedback_contract:="${ENABLE_RENDERED_FEEDBACK_CONTRACT}" \
  rendered_feedback_topic:="${RENDERED_FEEDBACK_TOPIC}" \
  rendered_feedback_qos_reliability:="${RENDERED_FEEDBACK_QOS_RELIABILITY}" \
  rendered_feedback_qos_durability:="${RENDERED_FEEDBACK_QOS_DURABILITY}" \
  rendered_feedback_qos_depth:="${RENDERED_FEEDBACK_QOS_DEPTH}" \
  enable_rendered_feedback_ingress_queue:="${ENABLE_RENDERED_FEEDBACK_INGRESS_QUEUE}" \
  rendered_feedback_ingress_queue_size:="${RENDERED_FEEDBACK_INGRESS_QUEUE_SIZE}" \
  rendered_feedback_ingress_drain_max_per_cycle:="${RENDERED_FEEDBACK_INGRESS_DRAIN_MAX_PER_CYCLE}" \
  rendered_feedback_ingress_drain_period_ms:="${RENDERED_FEEDBACK_INGRESS_DRAIN_PERIOD_MS}" \
  rendered_feedback_source_stream:="${MAPPER_FEEDBACK_RENDERED_FEEDBACK_SOURCE_STREAM}" \
  rendered_feedback_image_topic:="${MAPPER_FEEDBACK_RENDERED_FEEDBACK_IMAGE_TOPIC}" \
  rendered_feedback_pose_topic:="${MAPPER_FEEDBACK_RENDERED_FEEDBACK_POSE_TOPIC}" \
  rendered_feedback_image_qos_reliability:="${MAPPER_FEEDBACK_RENDERED_FEEDBACK_IMAGE_QOS_RELIABILITY}" \
  rendered_feedback_image_qos_history:="${MAPPER_FEEDBACK_RENDERED_FEEDBACK_IMAGE_QOS_HISTORY}" \
  rendered_feedback_image_qos_depth:="${MAPPER_FEEDBACK_RENDERED_FEEDBACK_IMAGE_QOS_DEPTH}" \
  rendered_feedback_pose_qos_reliability:="${MAPPER_FEEDBACK_RENDERED_FEEDBACK_POSE_QOS_RELIABILITY}" \
  rendered_feedback_pose_qos_history:="${MAPPER_FEEDBACK_RENDERED_FEEDBACK_POSE_QOS_HISTORY}" \
  rendered_feedback_pose_qos_depth:="${MAPPER_FEEDBACK_RENDERED_FEEDBACK_POSE_QOS_DEPTH}" \
  rendered_image_qos_reliability:="${RENDERED_IMAGE_QOS_RELIABILITY}" \
  rendered_image_qos_durability:="${RENDERED_IMAGE_QOS_DURABILITY}" \
  rendered_image_qos_depth:="${RENDERED_IMAGE_QOS_DEPTH}" \
  image_qos_reliability:="${MAPPER_FEEDBACK_IMAGE_QOS_RELIABILITY}" \
  image_qos_depth:="${MAPPER_FEEDBACK_IMAGE_QOS_DEPTH}" \
  visual_factor_max_dt_ns:="${VISUAL_FACTOR_MAX_DT_NS}" \
  enable_visual_factor_time_interpolation:="${ENABLE_VISUAL_FACTOR_TIME_INTERPOLATION}" \
  enable_visual_cache_reconciliation:="${ENABLE_VISUAL_CACHE_RECONCILIATION}" \
  visual_cache_reconciliation_monotonic_unique:="${ENABLE_VISUAL_CACHE_RECONCILIATION_MONOTONIC_UNIQUE}" \
  visual_pair_monotonic_unique:="${ENABLE_VISUAL_PAIR_MONOTONIC_UNIQUE}" \
  enable_visual_watermark_pair_scheduler:="${ENABLE_VISUAL_WATERMARK_PAIR_SCHEDULER}" \
  visual_watermark_pair_scheduler_max_pairs_per_pointcloud:="${VISUAL_WATERMARK_PAIR_SCHEDULER_MAX_PAIRS_PER_POINTCLOUD}" \
  enable_visual_callback_factor_ingest:="${ENABLE_VISUAL_CALLBACK_FACTOR_INGEST}" \
  enable_rendered_feedback_watermark_queue:="${ENABLE_RENDERED_FEEDBACK_WATERMARK_QUEUE}" \
  defer_future_visual_factors_until_active:="${DEFER_FUTURE_VISUAL_FACTORS_UNTIL_ACTIVE}" \
  enable_visual_adaptive_state_retention:="${ENABLE_VISUAL_ADAPTIVE_STATE_RETENTION}" \
  visual_adaptive_state_retention_margin_states:="${VISUAL_ADAPTIVE_STATE_RETENTION_MARGIN_STATES}" \
  visual_adaptive_state_retention_max_states:="${VISUAL_ADAPTIVE_STATE_RETENTION_MAX_STATES}" \
  enable_visual_expired_factor_projection:="${ENABLE_VISUAL_EXPIRED_FACTOR_PROJECTION}" \
  enable_visual_marginalization_prior:="${ENABLE_VISUAL_MARGINALIZATION_PRIOR}" \
  enable_visual_marginalization_prior_batching:="${ENABLE_VISUAL_MARGINALIZATION_PRIOR_BATCHING}" \
  enable_visual_marginalization_prior_saturation_gate:="${ENABLE_VISUAL_MARGINALIZATION_PRIOR_SATURATION_GATE}" \
  visual_marginalization_prior_zero_bias_columns:="${VISUAL_MARGINALIZATION_PRIOR_ZERO_BIAS_COLUMNS}" \
  enable_visual_factor_reference_snapshot:="${ENABLE_VISUAL_FACTOR_REFERENCE_SNAPSHOT}" \
  enable_rendered_feedback_source_pose_reference:="${ENABLE_RENDERED_FEEDBACK_SOURCE_POSE_REFERENCE}" \
  enable_rendered_feedback_source_motion_factor:="${ENABLE_RENDERED_FEEDBACK_SOURCE_MOTION_FACTOR}" \
  enable_rendered_feedback_source_motion_marginalized_prior:="${ENABLE_RENDERED_FEEDBACK_SOURCE_MOTION_MARGINALIZED_PRIOR}" \
  rendered_feedback_source_motion_translation_weight:="${RENDERED_FEEDBACK_SOURCE_MOTION_TRANSLATION_WEIGHT}" \
  rendered_feedback_source_motion_rotation_weight:="${RENDERED_FEEDBACK_SOURCE_MOTION_ROTATION_WEIGHT}" \
  rendered_feedback_source_motion_huber_delta_m:="${RENDERED_FEEDBACK_SOURCE_MOTION_HUBER_DELTA_M}" \
  rendered_feedback_source_motion_rotation_huber_delta_rad:="${RENDERED_FEEDBACK_SOURCE_MOTION_ROTATION_HUBER_DELTA_RAD}" \
  rendered_feedback_source_motion_min_dt_s:="${RENDERED_FEEDBACK_SOURCE_MOTION_MIN_DT_S}" \
  rendered_feedback_source_motion_max_dt_s:="${RENDERED_FEEDBACK_SOURCE_MOTION_MAX_DT_S}" \
  rendered_feedback_source_motion_in_from_frame:="${RENDERED_FEEDBACK_SOURCE_MOTION_IN_FROM_FRAME}" \
  visual_expired_factor_projection_max_age_s:="${VISUAL_EXPIRED_FACTOR_PROJECTION_MAX_AGE_S}" \
  visual_cache_reconciliation_defer_to_pointcloud:="${ENABLE_VISUAL_CACHE_RECONCILIATION_DEFER_TO_POINTCLOUD}" \
  visual_pair_processing_defer_to_pointcloud:="${ENABLE_VISUAL_PAIR_PROCESSING_DEFER_TO_POINTCLOUD}" \
  visual_depth_max_dt_ns:="${VISUAL_DEPTH_MAX_DT_NS}" \
  depth_frame_cache_size:="${VISUAL_DEPTH_FRAME_CACHE_SIZE}" \
  sparse_lidar_depth_dilation_px:="${VISUAL_DEPTH_DILATION_PX}" \
  visual_alignment_max_shift_px:="${VISUAL_ALIGNMENT_MAX_SHIFT_PX}" \
  visual_alignment_score_mode:="${VISUAL_ALIGNMENT_SCORE_MODE}" \
  visual_alignment_factor_source:="${VISUAL_ALIGNMENT_FACTOR_SOURCE}" \
  visual_factor_source_id_mode:="${VISUAL_FACTOR_SOURCE_ID_MODE}" \
  visual_factor_reference_stamp_mode:="${VISUAL_FACTOR_REFERENCE_STAMP_MODE}" \
  visual_alignment_window_weight:="${VISUAL_ALIGNMENT_WINDOW_WEIGHT}" \
  visual_alignment_saturation_margin_px:="${VISUAL_ALIGNMENT_SATURATION_MARGIN_PX}" \
  visual_alignment_saturated_weight_scale:="${VISUAL_ALIGNMENT_SATURATED_WEIGHT_SCALE}" \
  se3_photometric_window_weight:="${SE3_PHOTOMETRIC_WINDOW_WEIGHT}" \
  rendered_frame_cache_size:="${RENDERED_FRAME_CACHE_SIZE}" \
  observed_frame_cache_size:="${OBSERVED_FRAME_CACHE_SIZE}" \
  visual_pending_factor_queue_size:="${VISUAL_PENDING_FACTOR_QUEUE_SIZE}" \
  enable_visual_factor_quality_weighting:="${ENABLE_VISUAL_FACTOR_QUALITY_WEIGHTING}" \
  visual_factor_quality_min_weight_scale:="${VISUAL_FACTOR_QUALITY_MIN_WEIGHT_SCALE}" \
  enable_visual_factor_quality_selection:="${ENABLE_VISUAL_FACTOR_QUALITY_SELECTION}" \
  enable_visual_factor_quality_reference_cap:="${ENABLE_VISUAL_FACTOR_QUALITY_REFERENCE_CAP}" \
  visual_factor_quality_selection_max_per_reference:="${VISUAL_FACTOR_QUALITY_SELECTION_MAX_PER_REFERENCE}" \
  visual_factor_quality_selection_start_after_s:="${VISUAL_FACTOR_QUALITY_SELECTION_START_AFTER_S}" \
  se3_photometric_max_samples:="${SE3_PHOTOMETRIC_MAX_SAMPLES}" \
  se3_photometric_min_samples:="${SE3_PHOTOMETRIC_MIN_SAMPLES}" \
  se3_photometric_min_hessian_rank:="${SE3_PHOTOMETRIC_MIN_HESSIAN_RANK}" \
  se3_photometric_max_hessian_condition:="${SE3_PHOTOMETRIC_MAX_HESSIAN_CONDITION}" \
  se3_photometric_min_sample_inlier_ratio:="${SE3_PHOTOMETRIC_MIN_SAMPLE_INLIER_RATIO}" \
  se3_photometric_max_mean_abs_residual_for_factor:="${SE3_PHOTOMETRIC_MAX_MEAN_ABS_RESIDUAL_FOR_FACTOR}" \
  se3_photometric_rank_samples_by_gradient:="${SE3_PHOTOMETRIC_RANK_SAMPLES_BY_GRADIENT}" \
  se3_photometric_use_rendered_gradient:="${SE3_PHOTOMETRIC_USE_RENDERED_GRADIENT}" \
  se3_photometric_coverage_grid_cols:="${SE3_PHOTOMETRIC_COVERAGE_GRID_COLS}" \
  se3_photometric_coverage_grid_rows:="${SE3_PHOTOMETRIC_COVERAGE_GRID_ROWS}" \
  se3_photometric_min_coverage_tiles:="${SE3_PHOTOMETRIC_MIN_COVERAGE_TILES}" \
  enable_se3_photometric_pose_correction:="${ENABLE_SE3_PHOTOMETRIC_POSE_CORRECTION}" \
  se3_photometric_pose_correction_gain:="${SE3_PHOTOMETRIC_POSE_CORRECTION_GAIN}" \
  se3_photometric_pose_correction_max_translation_m:="${SE3_PHOTOMETRIC_POSE_CORRECTION_MAX_TRANSLATION_M}" \
  se3_photometric_pose_correction_max_rotation_rad:="${SE3_PHOTOMETRIC_POSE_CORRECTION_MAX_ROTATION_RAD}" \
  se3_photometric_pose_correction_max_dt_ns:="${SE3_PHOTOMETRIC_POSE_CORRECTION_MAX_DT_NS}" \
  enable_external_odometry_prior:="${ENABLE_EXTERNAL_ODOMETRY_PRIOR}" \
  external_odometry_prior_topic:="${REFERENCE_ODOMETRY_TOPIC:-/__unused_reference_odometry}" \
  external_odometry_prior_max_dt_ns:="${EXTERNAL_ODOMETRY_PRIOR_MAX_DT_NS}" \
  external_odometry_prior_translation_weight:="${EXTERNAL_ODOMETRY_PRIOR_TRANSLATION_WEIGHT}" \
  external_odometry_prior_rotation_weight:="${EXTERNAL_ODOMETRY_PRIOR_ROTATION_WEIGHT}" \
  raw_imu_qos_reliability:="${RAW_IMU_QOS_RELIABILITY}" \
  raw_imu_qos_depth:="${RAW_IMU_QOS_DEPTH}" \
  raw_pointcloud_qos_reliability:="${RAW_POINTCLOUD_QOS_RELIABILITY}" \
  raw_pointcloud_qos_depth:="${RAW_POINTCLOUD_QOS_DEPTH}" \
  pointcloud_imu_wait_queue_size:="${POINTCLOUD_IMU_WAIT_QUEUE_SIZE}" \
  imu_history_size:="${IMU_HISTORY_SIZE}" \
  imu_linear_acceleration_scale:="${IMU_LINEAR_ACCELERATION_SCALE}" \
  enable_gaussian_snapshot_lidar_factor:="${ENABLE_GAUSSIAN_MAP_FEEDBACK}" \
  gaussian_snapshot_qos_depth:="${GAUSSIAN_SNAPSHOT_QOS_DEPTH}" \
  gaussian_snapshot_lidar_factor_weight:="${GAUSSIAN_SNAPSHOT_LIDAR_FACTOR_WEIGHT}" \
  gaussian_snapshot_lidar_nearest_distance_m:="${GAUSSIAN_SNAPSHOT_LIDAR_NEAREST_DISTANCE_M}" \
  gaussian_snapshot_lidar_residual_preweight:="${GAUSSIAN_SNAPSHOT_LIDAR_RESIDUAL_PREWEIGHT}" \
  enable_gaussian_snapshot_lidar_pose_correction:="${ENABLE_GAUSSIAN_SNAPSHOT_LIDAR_POSE_CORRECTION}" \
  gaussian_snapshot_lidar_pose_correction_gain:="${GAUSSIAN_SNAPSHOT_LIDAR_POSE_CORRECTION_GAIN}" \
  gaussian_snapshot_lidar_pose_correction_max_translation_m:="${GAUSSIAN_SNAPSHOT_LIDAR_POSE_CORRECTION_MAX_TRANSLATION_M}" \
  gaussian_snapshot_lidar_pose_correction_max_rotation_rad:="${GAUSSIAN_SNAPSHOT_LIDAR_POSE_CORRECTION_MAX_ROTATION_RAD}" \
  gaussian_snapshot_lidar_pose_correction_min_match_ratio:="${GAUSSIAN_SNAPSHOT_LIDAR_POSE_CORRECTION_MIN_MATCH_RATIO}" \
  gaussian_snapshot_lidar_pose_correction_max_mean_residual_m:="${GAUSSIAN_SNAPSHOT_LIDAR_POSE_CORRECTION_MAX_MEAN_RESIDUAL_M}" \
  gaussian_snapshot_lidar_pose_correction_coverage_grid_cols:="${GAUSSIAN_SNAPSHOT_LIDAR_POSE_CORRECTION_COVERAGE_GRID_COLS}" \
  gaussian_snapshot_lidar_pose_correction_coverage_grid_rows:="${GAUSSIAN_SNAPSHOT_LIDAR_POSE_CORRECTION_COVERAGE_GRID_ROWS}" \
  gaussian_snapshot_lidar_pose_correction_min_coverage_tiles:="${GAUSSIAN_SNAPSHOT_LIDAR_POSE_CORRECTION_MIN_COVERAGE_TILES}" \
  gaussian_snapshot_lidar_pose_correction_bidirectional_max_distance_m:="${GAUSSIAN_SNAPSHOT_LIDAR_POSE_CORRECTION_BIDIRECTIONAL_MAX_DISTANCE_M}" \
  enable_gaussian_snapshot_lidar_plane_factor:="${ENABLE_GAUSSIAN_SNAPSHOT_LIDAR_PLANE_FACTOR}" \
  gaussian_snapshot_lidar_plane_factor_weight:="${GAUSSIAN_SNAPSHOT_LIDAR_PLANE_FACTOR_WEIGHT}" \
  gaussian_snapshot_lidar_plane_min_anisotropy:="${GAUSSIAN_SNAPSHOT_LIDAR_PLANE_MIN_ANISOTROPY}" \
  tracking_max_pose_step_m:="${TRACKING_MAX_POSE_STEP_M}" \
  enable_pre_lio_tracking_step_guard:="${ENABLE_PRE_LIO_TRACKING_STEP_GUARD}" \
  enable_post_ba_tracking_step_guard:="${ENABLE_POST_BA_TRACKING_STEP_GUARD}" \
  pre_lio_tracking_max_pose_step_m:="${PRE_LIO_TRACKING_MAX_POSE_STEP_M}" \
  post_ba_tracking_max_pose_step_m:="${POST_BA_TRACKING_MAX_POSE_STEP_M}" \
  post_ba_step_guard_confidence_max_pose_step_m:="${POST_BA_STEP_GUARD_CONFIDENCE_MAX_POSE_STEP_M}" \
  post_ba_step_guard_confidence_warmup_marginalizations:="${POST_BA_STEP_GUARD_CONFIDENCE_WARMUP_MARGINALIZATIONS}" \
  post_ba_step_guard_min_lidar_confidence:="${POST_BA_STEP_GUARD_MIN_LIDAR_CONFIDENCE}" \
  post_ba_step_guard_min_visual_inlier_ratio:="${POST_BA_STEP_GUARD_MIN_VISUAL_INLIER_RATIO}" \
  post_ba_step_guard_max_visual_residual:="${POST_BA_STEP_GUARD_MAX_VISUAL_RESIDUAL}" \
  post_ba_step_guard_min_visual_coverage_tiles:="${POST_BA_STEP_GUARD_MIN_VISUAL_COVERAGE_TILES}" \
  post_ba_step_guard_reject_to_pre_ba_over_m:="${POST_BA_STEP_GUARD_REJECT_TO_PRE_BA_OVER_M}" \
  post_ba_step_guard_pre_ba_agreement_max_pose_step_m:="${POST_BA_STEP_GUARD_PRE_BA_AGREEMENT_MAX_POSE_STEP_M}" \
  post_ba_step_guard_pre_ba_agreement_late_start_marginalizations:="${POST_BA_STEP_GUARD_PRE_BA_AGREEMENT_LATE_START_MARGINALIZATIONS}" \
  post_ba_step_guard_pre_ba_agreement_late_max_pose_step_m:="${POST_BA_STEP_GUARD_PRE_BA_AGREEMENT_LATE_MAX_POSE_STEP_M}" \
  post_ba_step_guard_pre_ba_agreement_min_cosine:="${POST_BA_STEP_GUARD_PRE_BA_AGREEMENT_MIN_COSINE}" \
  post_ba_step_guard_pre_ba_agreement_max_delta_m:="${POST_BA_STEP_GUARD_PRE_BA_AGREEMENT_MAX_DELTA_M}" \
  post_ba_step_guard_pre_ba_agreement_margin_m:="${POST_BA_STEP_GUARD_PRE_BA_AGREEMENT_MARGIN_M}" \
  post_ba_step_guard_pre_ba_blend_on_clamp:="${POST_BA_STEP_GUARD_PRE_BA_BLEND_ON_CLAMP}" \
  tracking_step_guard_velocity_scale:="${TRACKING_STEP_GUARD_VELOCITY_SCALE}" \
  pre_lio_tracking_step_guard_velocity_scale:="${PRE_LIO_TRACKING_STEP_GUARD_VELOCITY_SCALE}" \
  post_ba_tracking_step_guard_velocity_scale:="${POST_BA_TRACKING_STEP_GUARD_VELOCITY_SCALE}" \
  tracking_step_guard_acceleration_mps2:="${TRACKING_STEP_GUARD_ACCELERATION_MPS2}" \
  tracking_step_guard_max_velocity_mps:="${TRACKING_STEP_GUARD_MAX_VELOCITY_MPS}" \
  tracking_step_guard_margin_m:="${TRACKING_STEP_GUARD_MARGIN_M}" \
  lidar_min_points:="${LIDAR_MIN_POINTS}" \
  lidar_keyframe_translation_m:="${LIDAR_KEYFRAME_TRANSLATION_M}" \
  lidar_to_imu_translation_m:="${LIDAR_TO_IMU_TRANSLATION_M}" \
  lidar_to_imu_rpy_rad:="${LIDAR_TO_IMU_RPY_RAD}" \
  camera_to_imu_translation_m:="${CAMERA_TO_IMU_TRANSLATION_M}" \
  camera_to_imu_rpy_rad:="${CAMERA_TO_IMU_RPY_RAD}" \
  lidar_time_mode:="${LIDAR_TIME_MODE}" \
  lidar_scan_order_duration_s:="${LIDAR_SCAN_ORDER_DURATION_S}" \
  lidar_nearest_distance_m:="${LIDAR_NEAREST_DISTANCE_M}" \
  lidar_correction_gain:="${LIDAR_CORRECTION_GAIN}" \
  lidar_max_correction_m:="${LIDAR_MAX_CORRECTION_M}" \
  lidar_max_rotation_rad:="${LIDAR_MAX_ROTATION_RAD}" \
  lidar_robust_kernel_m:="${LIDAR_ROBUST_KERNEL_M}" \
  lidar_pose_factor_iterations:="${LIDAR_POSE_FACTOR_ITERATIONS}" \
  lidar_window_point_factor_weight:="${LIDAR_WINDOW_POINT_FACTOR_WEIGHT}" \
  lidar_window_plane_factor_weight:="${LIDAR_WINDOW_PLANE_FACTOR_WEIGHT}" \
  lidar_window_confidence_power:="${LIDAR_WINDOW_CONFIDENCE_POWER}" \
  lidar_max_frame_points:="${LIDAR_MAX_FRAME_POINTS}" \
  lidar_max_map_points:="${LIDAR_MAX_MAP_POINTS}" \
  sliding_window_optimize_every_n_frames:="${SLIDING_WINDOW_OPTIMIZE_EVERY_N_FRAMES}" \
  sliding_window_max_states:="${SLIDING_WINDOW_MAX_STATES}" \
  sliding_window_max_iterations:="${SLIDING_WINDOW_MAX_ITERATIONS}" \
  sliding_window_max_state_gap_s:="${SLIDING_WINDOW_MAX_STATE_GAP_S}" \
  sliding_window_marginalization_prior_weight:="${SLIDING_WINDOW_MARGINALIZATION_PRIOR_WEIGHT}" \
  sliding_window_max_normal_equation_condition:="${SLIDING_WINDOW_MAX_NORMAL_EQUATION_CONDITION}" \
  sliding_window_max_translation_step_m:="${SLIDING_WINDOW_MAX_TRANSLATION_STEP_M}" \
  sliding_window_max_feedback_translation_m:="${SLIDING_WINDOW_MAX_FEEDBACK_TRANSLATION_M}" \
  sliding_window_max_feedback_rotation_rad:="${SLIDING_WINDOW_MAX_FEEDBACK_ROTATION_RAD}" \
  sliding_window_max_feedback_velocity_mps:="${SLIDING_WINDOW_MAX_FEEDBACK_VELOCITY_MPS}" \
  sliding_window_max_feedback_gyro_bias_step:="${SLIDING_WINDOW_MAX_FEEDBACK_GYRO_BIAS_STEP}" \
  sliding_window_max_feedback_accel_bias_step:="${SLIDING_WINDOW_MAX_FEEDBACK_ACCEL_BIAS_STEP}" \
  sliding_window_min_bias_feedback_visual_factors:="${SLIDING_WINDOW_MIN_BIAS_FEEDBACK_VISUAL_FACTORS}" \
  sliding_window_bias_feedback_ownership:="${SLIDING_WINDOW_BIAS_FEEDBACK_OWNERSHIP}" \
  sliding_window_sync_guarded_pose_state:="${SLIDING_WINDOW_SYNC_GUARDED_POSE_STATE}" \
  sliding_window_guarded_pose_prior_translation_weight:="${SLIDING_WINDOW_GUARDED_POSE_PRIOR_TRANSLATION_WEIGHT}" \
  sliding_window_guarded_pose_prior_rotation_weight:="${SLIDING_WINDOW_GUARDED_POSE_PRIOR_ROTATION_WEIGHT}" \
  sliding_window_imu_weight:="${SLIDING_WINDOW_IMU_WEIGHT}" \
  sliding_window_imu_rotation_weight:="${SLIDING_WINDOW_IMU_ROTATION_WEIGHT}" \
  sliding_window_imu_velocity_weight:="${SLIDING_WINDOW_IMU_VELOCITY_WEIGHT}" \
  sliding_window_imu_position_weight:="${SLIDING_WINDOW_IMU_POSITION_WEIGHT}" \
  sliding_window_imu_velocity_prior_weight:="${SLIDING_WINDOW_IMU_VELOCITY_PRIOR_WEIGHT}" \
  sliding_window_gyro_bias_prior_weight:="${SLIDING_WINDOW_GYRO_BIAS_PRIOR_WEIGHT}" \
  sliding_window_accel_bias_prior_weight:="${SLIDING_WINDOW_ACCEL_BIAS_PRIOR_WEIGHT}" \
  sliding_window_bias_weight:="${SLIDING_WINDOW_BIAS_WEIGHT}" \
  sliding_window_gyro_bias_weight:="${SLIDING_WINDOW_GYRO_BIAS_WEIGHT}" \
  sliding_window_accel_bias_weight:="${SLIDING_WINDOW_ACCEL_BIAS_WEIGHT}" \
  sliding_window_bias_random_walk_reference_dt_s:="${SLIDING_WINDOW_BIAS_RANDOM_WALK_REFERENCE_DT_S}" \
  sliding_window_gyro_bias_random_walk_sigma:="${SLIDING_WINDOW_GYRO_BIAS_RANDOM_WALK_SIGMA}" \
  sliding_window_accel_bias_random_walk_sigma:="${SLIDING_WINDOW_ACCEL_BIAS_RANDOM_WALK_SIGMA}" \
  sliding_window_pose_translation_weight:="${SLIDING_WINDOW_POSE_TRANSLATION_WEIGHT}" \
  sliding_window_pose_rotation_weight:="${SLIDING_WINDOW_POSE_ROTATION_WEIGHT}" \
  sliding_window_smoothness_rotation_weight:="${SLIDING_WINDOW_SMOOTHNESS_ROTATION_WEIGHT}" \
  sliding_window_smoothness_position_weight:="${SLIDING_WINDOW_SMOOTHNESS_POSITION_WEIGHT}" \
  sliding_window_smoothness_velocity_weight:="${SLIDING_WINDOW_SMOOTHNESS_VELOCITY_WEIGHT}" \
  sliding_window_smoothness_position_velocity_weight:="${SLIDING_WINDOW_SMOOTHNESS_POSITION_VELOCITY_WEIGHT}" \
  sliding_window_smoothness_bias_weight:="${SLIDING_WINDOW_SMOOTHNESS_BIAS_WEIGHT}" \
  sliding_window_smoothness_use_motion_targets:="${SLIDING_WINDOW_SMOOTHNESS_USE_MOTION_TARGETS}" \
  sliding_window_smoothness_motion_target_min_visual_factors:="${SLIDING_WINDOW_SMOOTHNESS_MOTION_TARGET_MIN_VISUAL_FACTORS}" \
  sliding_window_smoothness_motion_target_min_se3_photometric_factors:="${SLIDING_WINDOW_SMOOTHNESS_MOTION_TARGET_MIN_SE3_PHOTOMETRIC_FACTORS}" \
  sliding_window_smoothness_motion_target_recent_window:="${SLIDING_WINDOW_SMOOTHNESS_MOTION_TARGET_RECENT_WINDOW}" \
  sliding_window_smoothness_motion_target_min_recent_visual_factors:="${SLIDING_WINDOW_SMOOTHNESS_MOTION_TARGET_MIN_RECENT_VISUAL_FACTORS}" \
  sliding_window_smoothness_motion_target_min_recent_se3_photometric_factors:="${SLIDING_WINDOW_SMOOTHNESS_MOTION_TARGET_MIN_RECENT_SE3_PHOTOMETRIC_FACTORS}" \
  sliding_window_smoothness_motion_target_start_after_s:="${SLIDING_WINDOW_SMOOTHNESS_MOTION_TARGET_START_AFTER_S}" \
  sliding_window_smoothness_motion_target_max_rotation_rate_delta_radps:="${SLIDING_WINDOW_SMOOTHNESS_MOTION_TARGET_MAX_ROTATION_RATE_DELTA_RADPS}" \
  sliding_window_smoothness_motion_target_max_position_rate_delta_mps:="${SLIDING_WINDOW_SMOOTHNESS_MOTION_TARGET_MAX_POSITION_RATE_DELTA_MPS}" \
  sliding_window_smoothness_motion_target_max_velocity_acceleration_delta_mps2:="${SLIDING_WINDOW_SMOOTHNESS_MOTION_TARGET_MAX_VELOCITY_ACCELERATION_DELTA_MPS2}" \
  enable_sliding_window_relative_translation_factor:="${ENABLE_SLIDING_WINDOW_RELATIVE_TRANSLATION_FACTOR}" \
  sliding_window_relative_translation_weight:="${SLIDING_WINDOW_RELATIVE_TRANSLATION_WEIGHT}" \
  sliding_window_relative_translation_huber_delta_m:="${SLIDING_WINDOW_RELATIVE_TRANSLATION_HUBER_DELTA_M}" \
  sliding_window_relative_translation_in_from_frame:="${SLIDING_WINDOW_RELATIVE_TRANSLATION_IN_FROM_FRAME}" \
  sliding_window_relative_rotation_weight:="${SLIDING_WINDOW_RELATIVE_ROTATION_WEIGHT}" \
  sliding_window_relative_rotation_huber_delta_rad:="${SLIDING_WINDOW_RELATIVE_ROTATION_HUBER_DELTA_RAD}" \
  enable_sliding_window_relative_distance_factor:="${ENABLE_SLIDING_WINDOW_RELATIVE_DISTANCE_FACTOR}" \
  sliding_window_relative_distance_weight:="${SLIDING_WINDOW_RELATIVE_DISTANCE_WEIGHT}" \
  sliding_window_relative_distance_huber_delta_m:="${SLIDING_WINDOW_RELATIVE_DISTANCE_HUBER_DELTA_M}" \
  enable_sliding_window_multihop_relative_translation_factor:="${ENABLE_SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_FACTOR}" \
  sliding_window_multihop_relative_translation_weight:="${SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_WEIGHT}" \
  sliding_window_multihop_relative_translation_huber_delta_m:="${SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_HUBER_DELTA_M}" \
  sliding_window_multihop_relative_translation_in_from_frame:="${SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_IN_FROM_FRAME}" \
  sliding_window_multihop_relative_rotation_weight:="${SLIDING_WINDOW_MULTIHOP_RELATIVE_ROTATION_WEIGHT}" \
  sliding_window_multihop_relative_rotation_huber_delta_rad:="${SLIDING_WINDOW_MULTIHOP_RELATIVE_ROTATION_HUBER_DELTA_RAD}" \
  enable_sliding_window_multihop_relative_distance_factor:="${ENABLE_SLIDING_WINDOW_MULTIHOP_RELATIVE_DISTANCE_FACTOR}" \
  sliding_window_multihop_relative_distance_weight:="${SLIDING_WINDOW_MULTIHOP_RELATIVE_DISTANCE_WEIGHT}" \
  sliding_window_multihop_relative_distance_huber_delta_m:="${SLIDING_WINDOW_MULTIHOP_RELATIVE_DISTANCE_HUBER_DELTA_M}" \
  sliding_window_multihop_relative_translation_min_dt_s:="${SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_MIN_DT_S}" \
  sliding_window_multihop_relative_translation_max_dt_s:="${SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_MAX_DT_S}" \
  sliding_window_multihop_relative_translation_max_factors:="${SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_MAX_FACTORS}" \
  enable_sliding_window_delayed_published_multihop_relative_translation_factor:="${ENABLE_SLIDING_WINDOW_DELAYED_PUBLISHED_MULTIHOP_RELATIVE_TRANSLATION_FACTOR}" \
  sliding_window_delayed_published_multihop_start_after_s:="${SLIDING_WINDOW_DELAYED_PUBLISHED_MULTIHOP_START_AFTER_S}" \
  sliding_window_delayed_published_multihop_max_factors:="${SLIDING_WINDOW_DELAYED_PUBLISHED_MULTIHOP_MAX_FACTORS}" \
  sliding_window_relative_motion_history_source:="${SLIDING_WINDOW_RELATIVE_MOTION_HISTORY_SOURCE}" \
  sliding_window_relative_motion_history_published_after_s:="${SLIDING_WINDOW_RELATIVE_MOTION_HISTORY_PUBLISHED_AFTER_S}" \
  >"${launch_log}" 2>&1 &
launch_pid=$!

if [[ "${ENABLE_MAPPER_FEEDBACK}" == "true" ]]; then
  setsid ros2 run gaussian_lic_mapping mapping_node \
    --ros-args \
    --params-file "${ROOT_DIR}/src/gaussian_lic_bringup/config/default.yaml" \
    -p use_sim_time:=true \
    -p pointcloud_coordinates:="${MAPPER_FEEDBACK_POINTCLOUD_COORDINATES}" \
    -p camera_to_pose_translation_m:="${CAMERA_TO_IMU_TRANSLATION_M}" \
    -p camera_to_pose_rpy_rad:="${CAMERA_TO_IMU_RPY_RAD}" \
    -p max_depth:="${MAPPER_FEEDBACK_MAX_DEPTH}" \
    -p require_projected_point_color:="${MAPPER_FEEDBACK_REQUIRE_PROJECTED_POINT_COLOR}" \
    -p zbuffer_projected_points:="${MAPPER_FEEDBACK_ZBUFFER_PROJECTED_POINTS}" \
    -p render_mode:="${MAPPER_FEEDBACK_RENDER_MODE}" \
    -p rendered_feedback_topic:="${RENDERED_FEEDBACK_TOPIC}" \
    -p rendered_feedback_qos_reliability:="${RENDERED_FEEDBACK_QOS_RELIABILITY}" \
    -p rendered_feedback_qos_durability:="${RENDERED_FEEDBACK_QOS_DURABILITY}" \
    -p rendered_feedback_qos_depth:="${RENDERED_FEEDBACK_QOS_DEPTH}" \
    -p rendered_feedback_source_stream:="${MAPPER_FEEDBACK_RENDERED_FEEDBACK_SOURCE_STREAM}" \
    -p rendered_feedback_image_topic:="${MAPPER_FEEDBACK_RENDERED_FEEDBACK_IMAGE_TOPIC}" \
    -p rendered_feedback_pose_topic:="${MAPPER_FEEDBACK_RENDERED_FEEDBACK_POSE_TOPIC}" \
    -p rendered_feedback_image_qos_reliability:="${MAPPER_FEEDBACK_RENDERED_FEEDBACK_IMAGE_QOS_RELIABILITY}" \
    -p rendered_feedback_image_qos_history:="${MAPPER_FEEDBACK_RENDERED_FEEDBACK_IMAGE_QOS_HISTORY}" \
    -p rendered_feedback_image_qos_depth:="${MAPPER_FEEDBACK_RENDERED_FEEDBACK_IMAGE_QOS_DEPTH}" \
    -p rendered_feedback_pose_qos_reliability:="${MAPPER_FEEDBACK_RENDERED_FEEDBACK_POSE_QOS_RELIABILITY}" \
    -p rendered_feedback_pose_qos_history:="${MAPPER_FEEDBACK_RENDERED_FEEDBACK_POSE_QOS_HISTORY}" \
    -p rendered_feedback_pose_qos_depth:="${MAPPER_FEEDBACK_RENDERED_FEEDBACK_POSE_QOS_DEPTH}" \
    -p publish_rendered_feedback_before_update:="${MAPPER_FEEDBACK_PUBLISH_RENDERED_BEFORE_UPDATE}" \
    -p rendered_image_qos_reliability:="${RENDERED_IMAGE_QOS_RELIABILITY}" \
    -p rendered_image_qos_durability:="${RENDERED_IMAGE_QOS_DURABILITY}" \
    -p rendered_image_qos_depth:="${RENDERED_IMAGE_QOS_DEPTH}" \
    -p image_qos_reliability:="${MAPPER_FEEDBACK_IMAGE_QOS_RELIABILITY}" \
    -p image_qos_depth:="${MAPPER_FEEDBACK_IMAGE_QOS_DEPTH}" \
    -p sync_tolerance_sec:="${MAPPER_FEEDBACK_SYNC_TOLERANCE_SEC}" \
    -p sync_anchor_stream:="${MAPPER_FEEDBACK_SYNC_ANCHOR_STREAM}" \
    -p select_every_k_frame:="${MAPPER_FEEDBACK_SELECT_EVERY_K_FRAME}" \
    -p require_depth_topic:=false \
    -p publish_gaussian_map:="${MAPPER_FEEDBACK_PUBLISH_GAUSSIAN_MAP}" \
    -p gaussian_map_chunk_size:="${MAPPER_FEEDBACK_GAUSSIAN_MAP_CHUNK_SIZE}" \
    -p gaussian_map_qos_depth:="${MAPPER_FEEDBACK_GAUSSIAN_MAP_QOS_DEPTH}" \
    -p gaussian_map_publish_min_interval_sec:="${MAPPER_FEEDBACK_GAUSSIAN_MAP_PUBLISH_MIN_INTERVAL_SEC}" \
    -p gaussian_map_publish_on_empty_extend:="${MAPPER_FEEDBACK_GAUSSIAN_MAP_PUBLISH_ON_EMPTY_EXTEND}" \
    -p enable_torch_camera_conversion:="${MAPPER_FEEDBACK_ENABLE_TORCH_CAMERA_CONVERSION}" \
    -p enable_torch_gaussian_init:="${MAPPER_FEEDBACK_ENABLE_TORCH_GAUSSIAN_INIT}" \
    -p enable_torch_gaussian_extend:="${MAPPER_FEEDBACK_ENABLE_TORCH_GAUSSIAN_EXTEND}" \
    -p enable_torch_gaussian_extend_visibility_filter:="${MAPPER_FEEDBACK_ENABLE_TORCH_GAUSSIAN_EXTEND_VISIBILITY_FILTER}" \
    -p enable_torch_gaussian_optimization:="${MAPPER_FEEDBACK_ENABLE_TORCH_GAUSSIAN_OPTIMIZATION}" \
    -p enable_torch_gaussian_pruning:="${MAPPER_FEEDBACK_ENABLE_TORCH_GAUSSIAN_PRUNING}" \
    -p enable_torch_gaussian_densification:="${MAPPER_FEEDBACK_ENABLE_TORCH_GAUSSIAN_DENSIFICATION}" \
    -p torch_gaussian_device:="${MAPPER_FEEDBACK_TORCH_DEVICE}" \
    -p torch_gaussian_optimization_steps:="${MAPPER_FEEDBACK_TORCH_OPTIMIZATION_STEPS}" \
    -p torch_gaussian_optimization_every_n_keyframes:="${MAPPER_FEEDBACK_TORCH_OPTIMIZATION_EVERY_N_KEYFRAMES}" \
    -p torch_gaussian_optimization_sampling:="${MAPPER_FEEDBACK_TORCH_OPTIMIZATION_SAMPLING}" \
    -p position_lr:="${MAPPER_FEEDBACK_POSITION_LR}" \
    -p feature_lr:="${MAPPER_FEEDBACK_FEATURE_LR}" \
    -p opacity_lr:="${MAPPER_FEEDBACK_OPACITY_LR}" \
    -p scaling_lr:="${MAPPER_FEEDBACK_SCALING_LR}" \
    -p rotation_lr:="${MAPPER_FEEDBACK_ROTATION_LR}" \
    -p torch_gaussian_max_foreground:="${MAPPER_FEEDBACK_TORCH_MAX_FOREGROUND}" \
    -p torch_gaussian_prune_count_policy:="${MAPPER_FEEDBACK_TORCH_PRUNE_COUNT_POLICY}" \
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
  -p mapping_status_topic:=/gaussian_lic/status
  -p write_status_history:="${WRITE_STATUS_HISTORY}"
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

playback_args=(
  "${BAG_PATH}"
  --rate "${PLAYBACK_RATE}"
  --read-ahead-queue-size 100
  --playback-duration "${PLAYBACK_DURATION}"
  --disable-keyboard-controls
)
if [[ "${PLAYBACK_CLOCK_TOPICS_ALL}" == "true" ]]; then
  playback_args+=(--clock-topics-all)
else
  playback_args+=(--clock)
fi
if [[ "${PLAYBACK_START_OFFSET}" != "0" && "${PLAYBACK_START_OFFSET}" != "0.0" ]]; then
  playback_args+=(--start-offset "${PLAYBACK_START_OFFSET}")
fi
ros2 bag play "${playback_args[@]}" \
  >"${play_log}" 2>&1 &
play_pid=$!

play_timeout="$(
  python3 - "${PLAYBACK_DURATION}" "${TIMEOUT_SEC}" "${PLAYBACK_RATE}" <<'PY'
import math
import sys
duration_sec = float(sys.argv[1])
timeout_sec = float(sys.argv[2])
rate = float(sys.argv[3])
if rate <= 0.0:
    raise SystemExit("playback rate must be positive")
print(int(math.ceil(duration_sec / rate + timeout_sec)))
PY
)"
if ! timeout "${play_timeout}" tail --pid="${play_pid}" -f /dev/null; then
  echo "rosbag2 playback did not finish before timeout" >&2
  exit 1
fi
wait "${play_pid}"
unset play_pid

sleep "${POST_PLAY_SETTLE_SEC}"
wait_for_recorder_drain \
  "${ARTIFACT_DIR}/metrics.json" \
  "${POST_PLAY_DRAIN_TARGET_POSES}" \
  "${POST_PLAY_DRAIN_TARGET_VISUAL_FACTORS}" \
  "${POST_PLAY_DRAIN_TARGET_SE3_FACTORS}" \
  "${POST_PLAY_DRAIN_TIMEOUT_SEC}"
stop_process_group "${record_pid}" INT
unset record_pid

stop_process_group "${launch_pid}" TERM
unset launch_pid

set +e
IMU_LINEAR_ACCELERATION_SCALE_REPORT="${IMU_LINEAR_ACCELERATION_SCALE}" \
PLAYBACK_DURATION_REPORT="${PLAYBACK_DURATION}" \
TIMEOUT_SEC_REPORT="${TIMEOUT_SEC}" \
POST_PLAY_SETTLE_SEC_REPORT="${POST_PLAY_SETTLE_SEC}" \
POST_PLAY_DRAIN_TARGET_POSES_REPORT="${POST_PLAY_DRAIN_TARGET_POSES}" \
POST_PLAY_DRAIN_TARGET_VISUAL_FACTORS_REPORT="${POST_PLAY_DRAIN_TARGET_VISUAL_FACTORS}" \
POST_PLAY_DRAIN_TARGET_SE3_FACTORS_REPORT="${POST_PLAY_DRAIN_TARGET_SE3_FACTORS}" \
POST_PLAY_DRAIN_TIMEOUT_SEC_REPORT="${POST_PLAY_DRAIN_TIMEOUT_SEC}" \
PLAYBACK_RATE_REPORT="${PLAYBACK_RATE}" \
PLAYBACK_START_OFFSET_REPORT="${PLAYBACK_START_OFFSET}" \
PLAYBACK_CLOCK_TOPICS_ALL_REPORT="${PLAYBACK_CLOCK_TOPICS_ALL}" \
MAX_LIDAR_INVALID_FRAMES_REPORT="${MAX_LIDAR_INVALID_FRAMES}" \
MIN_STATUS_BIN_SAMPLE_COUNT_REPORT="${MIN_STATUS_BIN_SAMPLE_COUNT}" \
MIN_VISUAL_FACTOR_DELTA_PER_STATUS_BIN_REPORT="${MIN_VISUAL_FACTOR_DELTA_PER_STATUS_BIN}" \
MIN_SE3_PHOTOMETRIC_FACTOR_DELTA_PER_STATUS_BIN_REPORT="${MIN_SE3_PHOTOMETRIC_FACTOR_DELTA_PER_STATUS_BIN}" \
MIN_MOTION_TARGET_DELTA_PER_STATUS_BIN_REPORT="${MIN_MOTION_TARGET_DELTA_PER_STATUS_BIN}" \
MAPPER_FEEDBACK_POINTCLOUD_COORDINATES_REPORT="${MAPPER_FEEDBACK_POINTCLOUD_COORDINATES}" \
MAPPER_FEEDBACK_MAX_DEPTH_REPORT="${MAPPER_FEEDBACK_MAX_DEPTH}" \
MAPPER_FEEDBACK_REQUIRE_PROJECTED_POINT_COLOR_REPORT="${MAPPER_FEEDBACK_REQUIRE_PROJECTED_POINT_COLOR}" \
MAPPER_FEEDBACK_ZBUFFER_PROJECTED_POINTS_REPORT="${MAPPER_FEEDBACK_ZBUFFER_PROJECTED_POINTS}" \
MAPPER_FEEDBACK_LR_REPORT="${MAPPER_FEEDBACK_POSITION_LR},${MAPPER_FEEDBACK_FEATURE_LR},${MAPPER_FEEDBACK_OPACITY_LR},${MAPPER_FEEDBACK_SCALING_LR},${MAPPER_FEEDBACK_ROTATION_LR}" \
MAPPER_FEEDBACK_OPTIMIZATION_EVERY_REPORT="${MAPPER_FEEDBACK_TORCH_OPTIMIZATION_EVERY_N_KEYFRAMES}" \
MAPPER_FEEDBACK_SELECT_EVERY_REPORT="${MAPPER_FEEDBACK_SELECT_EVERY_K_FRAME}" \
MAPPER_FEEDBACK_IMAGE_QOS_RELIABILITY_REPORT="${MAPPER_FEEDBACK_IMAGE_QOS_RELIABILITY}" \
MAPPER_FEEDBACK_IMAGE_QOS_DEPTH_REPORT="${MAPPER_FEEDBACK_IMAGE_QOS_DEPTH}" \
GAUSSIAN_SNAPSHOT_LIDAR_FACTOR_WEIGHT_REPORT="${GAUSSIAN_SNAPSHOT_LIDAR_FACTOR_WEIGHT}" \
GAUSSIAN_SNAPSHOT_LIDAR_NEAREST_DISTANCE_M_REPORT="${GAUSSIAN_SNAPSHOT_LIDAR_NEAREST_DISTANCE_M}" \
GAUSSIAN_SNAPSHOT_LIDAR_RESIDUAL_PREWEIGHT_REPORT="${GAUSSIAN_SNAPSHOT_LIDAR_RESIDUAL_PREWEIGHT}" \
GAUSSIAN_SNAPSHOT_LIDAR_POSE_CORRECTION_REPORT="${ENABLE_GAUSSIAN_SNAPSHOT_LIDAR_POSE_CORRECTION}" \
GAUSSIAN_SNAPSHOT_LIDAR_POSE_CORRECTION_GAIN_REPORT="${GAUSSIAN_SNAPSHOT_LIDAR_POSE_CORRECTION_GAIN}" \
GAUSSIAN_SNAPSHOT_LIDAR_POSE_CORRECTION_MAX_TRANSLATION_M_REPORT="${GAUSSIAN_SNAPSHOT_LIDAR_POSE_CORRECTION_MAX_TRANSLATION_M}" \
GAUSSIAN_SNAPSHOT_LIDAR_POSE_CORRECTION_MAX_ROTATION_RAD_REPORT="${GAUSSIAN_SNAPSHOT_LIDAR_POSE_CORRECTION_MAX_ROTATION_RAD}" \
GAUSSIAN_SNAPSHOT_LIDAR_POSE_CORRECTION_MIN_MATCH_RATIO_REPORT="${GAUSSIAN_SNAPSHOT_LIDAR_POSE_CORRECTION_MIN_MATCH_RATIO}" \
GAUSSIAN_SNAPSHOT_LIDAR_POSE_CORRECTION_MAX_MEAN_RESIDUAL_M_REPORT="${GAUSSIAN_SNAPSHOT_LIDAR_POSE_CORRECTION_MAX_MEAN_RESIDUAL_M}" \
GAUSSIAN_SNAPSHOT_LIDAR_POSE_CORRECTION_COVERAGE_GRID_COLS_REPORT="${GAUSSIAN_SNAPSHOT_LIDAR_POSE_CORRECTION_COVERAGE_GRID_COLS}" \
GAUSSIAN_SNAPSHOT_LIDAR_POSE_CORRECTION_COVERAGE_GRID_ROWS_REPORT="${GAUSSIAN_SNAPSHOT_LIDAR_POSE_CORRECTION_COVERAGE_GRID_ROWS}" \
GAUSSIAN_SNAPSHOT_LIDAR_POSE_CORRECTION_MIN_COVERAGE_TILES_REPORT="${GAUSSIAN_SNAPSHOT_LIDAR_POSE_CORRECTION_MIN_COVERAGE_TILES}" \
GAUSSIAN_SNAPSHOT_LIDAR_POSE_CORRECTION_BIDIRECTIONAL_MAX_DISTANCE_M_REPORT="${GAUSSIAN_SNAPSHOT_LIDAR_POSE_CORRECTION_BIDIRECTIONAL_MAX_DISTANCE_M}" \
GAUSSIAN_SNAPSHOT_LIDAR_PLANE_FACTOR_REPORT="${ENABLE_GAUSSIAN_SNAPSHOT_LIDAR_PLANE_FACTOR}" \
GAUSSIAN_SNAPSHOT_LIDAR_PLANE_FACTOR_WEIGHT_REPORT="${GAUSSIAN_SNAPSHOT_LIDAR_PLANE_FACTOR_WEIGHT}" \
GAUSSIAN_SNAPSHOT_LIDAR_PLANE_MIN_ANISOTROPY_REPORT="${GAUSSIAN_SNAPSHOT_LIDAR_PLANE_MIN_ANISOTROPY}" \
LIDAR_POSE_FACTOR_ITERATIONS_REPORT="${LIDAR_POSE_FACTOR_ITERATIONS}" \
LIDAR_WINDOW_POINT_FACTOR_WEIGHT_REPORT="${LIDAR_WINDOW_POINT_FACTOR_WEIGHT}" \
LIDAR_WINDOW_PLANE_FACTOR_WEIGHT_REPORT="${LIDAR_WINDOW_PLANE_FACTOR_WEIGHT}" \
LIDAR_WINDOW_CONFIDENCE_POWER_REPORT="${LIDAR_WINDOW_CONFIDENCE_POWER}" \
SLIDING_WINDOW_MAX_STATES_REPORT="${SLIDING_WINDOW_MAX_STATES}" \
SLIDING_WINDOW_MAX_ITERATIONS_REPORT="${SLIDING_WINDOW_MAX_ITERATIONS}" \
SLIDING_WINDOW_MARGINALIZATION_PRIOR_WEIGHT_REPORT="${SLIDING_WINDOW_MARGINALIZATION_PRIOR_WEIGHT}" \
ENABLE_VISUAL_MARGINALIZATION_PRIOR_BATCHING_REPORT="${ENABLE_VISUAL_MARGINALIZATION_PRIOR_BATCHING}" \
ENABLE_VISUAL_MARGINALIZATION_PRIOR_SATURATION_GATE_REPORT="${ENABLE_VISUAL_MARGINALIZATION_PRIOR_SATURATION_GATE}" \
VISUAL_MARGINALIZATION_PRIOR_ZERO_BIAS_COLUMNS_REPORT="${VISUAL_MARGINALIZATION_PRIOR_ZERO_BIAS_COLUMNS}" \
ENABLE_VISUAL_FACTOR_REFERENCE_SNAPSHOT_REPORT="${ENABLE_VISUAL_FACTOR_REFERENCE_SNAPSHOT}" \
ENABLE_RENDERED_FEEDBACK_SOURCE_POSE_REFERENCE_REPORT="${ENABLE_RENDERED_FEEDBACK_SOURCE_POSE_REFERENCE}" \
ENABLE_RENDERED_FEEDBACK_SOURCE_MOTION_FACTOR_REPORT="${ENABLE_RENDERED_FEEDBACK_SOURCE_MOTION_FACTOR}" \
ENABLE_RENDERED_FEEDBACK_SOURCE_MOTION_MARGINALIZED_PRIOR_REPORT="${ENABLE_RENDERED_FEEDBACK_SOURCE_MOTION_MARGINALIZED_PRIOR}" \
RENDERED_FEEDBACK_SOURCE_MOTION_TRANSLATION_WEIGHT_REPORT="${RENDERED_FEEDBACK_SOURCE_MOTION_TRANSLATION_WEIGHT}" \
RENDERED_FEEDBACK_SOURCE_MOTION_ROTATION_WEIGHT_REPORT="${RENDERED_FEEDBACK_SOURCE_MOTION_ROTATION_WEIGHT}" \
RENDERED_FEEDBACK_SOURCE_MOTION_HUBER_DELTA_M_REPORT="${RENDERED_FEEDBACK_SOURCE_MOTION_HUBER_DELTA_M}" \
RENDERED_FEEDBACK_SOURCE_MOTION_ROTATION_HUBER_DELTA_RAD_REPORT="${RENDERED_FEEDBACK_SOURCE_MOTION_ROTATION_HUBER_DELTA_RAD}" \
RENDERED_FEEDBACK_SOURCE_MOTION_MIN_DT_S_REPORT="${RENDERED_FEEDBACK_SOURCE_MOTION_MIN_DT_S}" \
RENDERED_FEEDBACK_SOURCE_MOTION_MAX_DT_S_REPORT="${RENDERED_FEEDBACK_SOURCE_MOTION_MAX_DT_S}" \
RENDERED_FEEDBACK_SOURCE_MOTION_IN_FROM_FRAME_REPORT="${RENDERED_FEEDBACK_SOURCE_MOTION_IN_FROM_FRAME}" \
ENABLE_PRE_LIO_TRACKING_STEP_GUARD_REPORT="${ENABLE_PRE_LIO_TRACKING_STEP_GUARD}" \
ENABLE_POST_BA_TRACKING_STEP_GUARD_REPORT="${ENABLE_POST_BA_TRACKING_STEP_GUARD}" \
PRE_LIO_TRACKING_MAX_POSE_STEP_M_REPORT="${PRE_LIO_TRACKING_MAX_POSE_STEP_M}" \
POST_BA_TRACKING_MAX_POSE_STEP_M_REPORT="${POST_BA_TRACKING_MAX_POSE_STEP_M}" \
POST_BA_STEP_GUARD_CONFIDENCE_MAX_POSE_STEP_M_REPORT="${POST_BA_STEP_GUARD_CONFIDENCE_MAX_POSE_STEP_M}" \
POST_BA_STEP_GUARD_CONFIDENCE_WARMUP_MARGINALIZATIONS_REPORT="${POST_BA_STEP_GUARD_CONFIDENCE_WARMUP_MARGINALIZATIONS}" \
POST_BA_STEP_GUARD_MIN_LIDAR_CONFIDENCE_REPORT="${POST_BA_STEP_GUARD_MIN_LIDAR_CONFIDENCE}" \
POST_BA_STEP_GUARD_MIN_VISUAL_INLIER_RATIO_REPORT="${POST_BA_STEP_GUARD_MIN_VISUAL_INLIER_RATIO}" \
POST_BA_STEP_GUARD_MAX_VISUAL_RESIDUAL_REPORT="${POST_BA_STEP_GUARD_MAX_VISUAL_RESIDUAL}" \
POST_BA_STEP_GUARD_MIN_VISUAL_COVERAGE_TILES_REPORT="${POST_BA_STEP_GUARD_MIN_VISUAL_COVERAGE_TILES}" \
POST_BA_STEP_GUARD_REJECT_TO_PRE_BA_OVER_M_REPORT="${POST_BA_STEP_GUARD_REJECT_TO_PRE_BA_OVER_M}" \
POST_BA_STEP_GUARD_PRE_BA_AGREEMENT_MAX_POSE_STEP_M_REPORT="${POST_BA_STEP_GUARD_PRE_BA_AGREEMENT_MAX_POSE_STEP_M}" \
POST_BA_STEP_GUARD_PRE_BA_AGREEMENT_LATE_START_MARGINALIZATIONS_REPORT="${POST_BA_STEP_GUARD_PRE_BA_AGREEMENT_LATE_START_MARGINALIZATIONS}" \
POST_BA_STEP_GUARD_PRE_BA_AGREEMENT_LATE_MAX_POSE_STEP_M_REPORT="${POST_BA_STEP_GUARD_PRE_BA_AGREEMENT_LATE_MAX_POSE_STEP_M}" \
POST_BA_STEP_GUARD_PRE_BA_AGREEMENT_MIN_COSINE_REPORT="${POST_BA_STEP_GUARD_PRE_BA_AGREEMENT_MIN_COSINE}" \
POST_BA_STEP_GUARD_PRE_BA_AGREEMENT_MAX_DELTA_M_REPORT="${POST_BA_STEP_GUARD_PRE_BA_AGREEMENT_MAX_DELTA_M}" \
POST_BA_STEP_GUARD_PRE_BA_AGREEMENT_MARGIN_M_REPORT="${POST_BA_STEP_GUARD_PRE_BA_AGREEMENT_MARGIN_M}" \
POST_BA_STEP_GUARD_PRE_BA_BLEND_ON_CLAMP_REPORT="${POST_BA_STEP_GUARD_PRE_BA_BLEND_ON_CLAMP}" \
PRE_LIO_TRACKING_STEP_GUARD_VELOCITY_SCALE_REPORT="${PRE_LIO_TRACKING_STEP_GUARD_VELOCITY_SCALE}" \
POST_BA_TRACKING_STEP_GUARD_VELOCITY_SCALE_REPORT="${POST_BA_TRACKING_STEP_GUARD_VELOCITY_SCALE}" \
SLIDING_WINDOW_MAX_FEEDBACK_GYRO_BIAS_STEP_REPORT="${SLIDING_WINDOW_MAX_FEEDBACK_GYRO_BIAS_STEP}" \
SLIDING_WINDOW_MAX_FEEDBACK_ACCEL_BIAS_STEP_REPORT="${SLIDING_WINDOW_MAX_FEEDBACK_ACCEL_BIAS_STEP}" \
SLIDING_WINDOW_MIN_BIAS_FEEDBACK_VISUAL_FACTORS_REPORT="${SLIDING_WINDOW_MIN_BIAS_FEEDBACK_VISUAL_FACTORS}" \
SLIDING_WINDOW_BIAS_FEEDBACK_OWNERSHIP_REPORT="${SLIDING_WINDOW_BIAS_FEEDBACK_OWNERSHIP}" \
SLIDING_WINDOW_SYNC_GUARDED_POSE_STATE_REPORT="${SLIDING_WINDOW_SYNC_GUARDED_POSE_STATE}" \
SLIDING_WINDOW_SMOOTHNESS_POSITION_VELOCITY_WEIGHT_REPORT="${SLIDING_WINDOW_SMOOTHNESS_POSITION_VELOCITY_WEIGHT}" \
SLIDING_WINDOW_SMOOTHNESS_USE_MOTION_TARGETS_REPORT="${SLIDING_WINDOW_SMOOTHNESS_USE_MOTION_TARGETS}" \
SLIDING_WINDOW_SMOOTHNESS_MOTION_TARGET_MIN_VISUAL_FACTORS_REPORT="${SLIDING_WINDOW_SMOOTHNESS_MOTION_TARGET_MIN_VISUAL_FACTORS}" \
SLIDING_WINDOW_SMOOTHNESS_MOTION_TARGET_MIN_SE3_PHOTOMETRIC_FACTORS_REPORT="${SLIDING_WINDOW_SMOOTHNESS_MOTION_TARGET_MIN_SE3_PHOTOMETRIC_FACTORS}" \
SLIDING_WINDOW_SMOOTHNESS_MOTION_TARGET_RECENT_WINDOW_REPORT="${SLIDING_WINDOW_SMOOTHNESS_MOTION_TARGET_RECENT_WINDOW}" \
SLIDING_WINDOW_SMOOTHNESS_MOTION_TARGET_MIN_RECENT_VISUAL_FACTORS_REPORT="${SLIDING_WINDOW_SMOOTHNESS_MOTION_TARGET_MIN_RECENT_VISUAL_FACTORS}" \
SLIDING_WINDOW_SMOOTHNESS_MOTION_TARGET_MIN_RECENT_SE3_PHOTOMETRIC_FACTORS_REPORT="${SLIDING_WINDOW_SMOOTHNESS_MOTION_TARGET_MIN_RECENT_SE3_PHOTOMETRIC_FACTORS}" \
SLIDING_WINDOW_SMOOTHNESS_MOTION_TARGET_START_AFTER_S_REPORT="${SLIDING_WINDOW_SMOOTHNESS_MOTION_TARGET_START_AFTER_S}" \
SLIDING_WINDOW_SMOOTHNESS_MOTION_TARGET_MAX_ROTATION_RATE_DELTA_RADPS_REPORT="${SLIDING_WINDOW_SMOOTHNESS_MOTION_TARGET_MAX_ROTATION_RATE_DELTA_RADPS}" \
SLIDING_WINDOW_SMOOTHNESS_MOTION_TARGET_MAX_POSITION_RATE_DELTA_MPS_REPORT="${SLIDING_WINDOW_SMOOTHNESS_MOTION_TARGET_MAX_POSITION_RATE_DELTA_MPS}" \
SLIDING_WINDOW_SMOOTHNESS_MOTION_TARGET_MAX_VELOCITY_ACCELERATION_DELTA_MPS2_REPORT="${SLIDING_WINDOW_SMOOTHNESS_MOTION_TARGET_MAX_VELOCITY_ACCELERATION_DELTA_MPS2}" \
SLIDING_WINDOW_IMU_VELOCITY_PRIOR_WEIGHT_REPORT="${SLIDING_WINDOW_IMU_VELOCITY_PRIOR_WEIGHT}" \
SLIDING_WINDOW_GYRO_BIAS_PRIOR_WEIGHT_REPORT="${SLIDING_WINDOW_GYRO_BIAS_PRIOR_WEIGHT}" \
SLIDING_WINDOW_ACCEL_BIAS_PRIOR_WEIGHT_REPORT="${SLIDING_WINDOW_ACCEL_BIAS_PRIOR_WEIGHT}" \
SLIDING_WINDOW_BIAS_WEIGHT_REPORT="${SLIDING_WINDOW_BIAS_WEIGHT}" \
SLIDING_WINDOW_GYRO_BIAS_WEIGHT_REPORT="${SLIDING_WINDOW_GYRO_BIAS_WEIGHT}" \
SLIDING_WINDOW_ACCEL_BIAS_WEIGHT_REPORT="${SLIDING_WINDOW_ACCEL_BIAS_WEIGHT}" \
SLIDING_WINDOW_BIAS_RANDOM_WALK_REFERENCE_DT_S_REPORT="${SLIDING_WINDOW_BIAS_RANDOM_WALK_REFERENCE_DT_S}" \
SLIDING_WINDOW_GYRO_BIAS_RANDOM_WALK_SIGMA_REPORT="${SLIDING_WINDOW_GYRO_BIAS_RANDOM_WALK_SIGMA}" \
SLIDING_WINDOW_ACCEL_BIAS_RANDOM_WALK_SIGMA_REPORT="${SLIDING_WINDOW_ACCEL_BIAS_RANDOM_WALK_SIGMA}" \
SLIDING_WINDOW_GUARDED_POSE_PRIOR_TRANSLATION_WEIGHT_REPORT="${SLIDING_WINDOW_GUARDED_POSE_PRIOR_TRANSLATION_WEIGHT}" \
SLIDING_WINDOW_GUARDED_POSE_PRIOR_ROTATION_WEIGHT_REPORT="${SLIDING_WINDOW_GUARDED_POSE_PRIOR_ROTATION_WEIGHT}" \
ENABLE_SLIDING_WINDOW_RELATIVE_TRANSLATION_FACTOR_REPORT="${ENABLE_SLIDING_WINDOW_RELATIVE_TRANSLATION_FACTOR}" \
SLIDING_WINDOW_RELATIVE_TRANSLATION_WEIGHT_REPORT="${SLIDING_WINDOW_RELATIVE_TRANSLATION_WEIGHT}" \
SLIDING_WINDOW_RELATIVE_TRANSLATION_HUBER_DELTA_M_REPORT="${SLIDING_WINDOW_RELATIVE_TRANSLATION_HUBER_DELTA_M}" \
SLIDING_WINDOW_RELATIVE_TRANSLATION_IN_FROM_FRAME_REPORT="${SLIDING_WINDOW_RELATIVE_TRANSLATION_IN_FROM_FRAME}" \
SLIDING_WINDOW_RELATIVE_ROTATION_WEIGHT_REPORT="${SLIDING_WINDOW_RELATIVE_ROTATION_WEIGHT}" \
SLIDING_WINDOW_RELATIVE_ROTATION_HUBER_DELTA_RAD_REPORT="${SLIDING_WINDOW_RELATIVE_ROTATION_HUBER_DELTA_RAD}" \
ENABLE_SLIDING_WINDOW_RELATIVE_DISTANCE_FACTOR_REPORT="${ENABLE_SLIDING_WINDOW_RELATIVE_DISTANCE_FACTOR}" \
SLIDING_WINDOW_RELATIVE_DISTANCE_WEIGHT_REPORT="${SLIDING_WINDOW_RELATIVE_DISTANCE_WEIGHT}" \
SLIDING_WINDOW_RELATIVE_DISTANCE_HUBER_DELTA_M_REPORT="${SLIDING_WINDOW_RELATIVE_DISTANCE_HUBER_DELTA_M}" \
ENABLE_SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_FACTOR_REPORT="${ENABLE_SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_FACTOR}" \
SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_WEIGHT_REPORT="${SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_WEIGHT}" \
SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_HUBER_DELTA_M_REPORT="${SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_HUBER_DELTA_M}" \
SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_IN_FROM_FRAME_REPORT="${SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_IN_FROM_FRAME}" \
SLIDING_WINDOW_MULTIHOP_RELATIVE_ROTATION_WEIGHT_REPORT="${SLIDING_WINDOW_MULTIHOP_RELATIVE_ROTATION_WEIGHT}" \
SLIDING_WINDOW_MULTIHOP_RELATIVE_ROTATION_HUBER_DELTA_RAD_REPORT="${SLIDING_WINDOW_MULTIHOP_RELATIVE_ROTATION_HUBER_DELTA_RAD}" \
ENABLE_SLIDING_WINDOW_MULTIHOP_RELATIVE_DISTANCE_FACTOR_REPORT="${ENABLE_SLIDING_WINDOW_MULTIHOP_RELATIVE_DISTANCE_FACTOR}" \
SLIDING_WINDOW_MULTIHOP_RELATIVE_DISTANCE_WEIGHT_REPORT="${SLIDING_WINDOW_MULTIHOP_RELATIVE_DISTANCE_WEIGHT}" \
SLIDING_WINDOW_MULTIHOP_RELATIVE_DISTANCE_HUBER_DELTA_M_REPORT="${SLIDING_WINDOW_MULTIHOP_RELATIVE_DISTANCE_HUBER_DELTA_M}" \
SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_MIN_DT_S_REPORT="${SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_MIN_DT_S}" \
SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_MAX_DT_S_REPORT="${SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_MAX_DT_S}" \
SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_MAX_FACTORS_REPORT="${SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_MAX_FACTORS}" \
ENABLE_SLIDING_WINDOW_DELAYED_PUBLISHED_MULTIHOP_RELATIVE_TRANSLATION_FACTOR_REPORT="${ENABLE_SLIDING_WINDOW_DELAYED_PUBLISHED_MULTIHOP_RELATIVE_TRANSLATION_FACTOR}" \
SLIDING_WINDOW_DELAYED_PUBLISHED_MULTIHOP_START_AFTER_S_REPORT="${SLIDING_WINDOW_DELAYED_PUBLISHED_MULTIHOP_START_AFTER_S}" \
SLIDING_WINDOW_DELAYED_PUBLISHED_MULTIHOP_MAX_FACTORS_REPORT="${SLIDING_WINDOW_DELAYED_PUBLISHED_MULTIHOP_MAX_FACTORS}" \
SLIDING_WINDOW_RELATIVE_MOTION_HISTORY_SOURCE_REPORT="${SLIDING_WINDOW_RELATIVE_MOTION_HISTORY_SOURCE}" \
SLIDING_WINDOW_RELATIVE_MOTION_HISTORY_PUBLISHED_AFTER_S_REPORT="${SLIDING_WINDOW_RELATIVE_MOTION_HISTORY_PUBLISHED_AFTER_S}" \
REFERENCE_ERROR_BIN_COUNT_REPORT="${REFERENCE_ERROR_BIN_COUNT}" \
REFERENCE_MIN_ERROR_BIN_MATCHES_REPORT="${REFERENCE_MIN_ERROR_BIN_MATCHES}" \
REFERENCE_MAX_ERROR_BIN_RMSE_M_REPORT="${REFERENCE_MAX_ERROR_BIN_RMSE_M}" \
REFERENCE_MAX_ERROR_BIN_MEAN_M_REPORT="${REFERENCE_MAX_ERROR_BIN_MEAN_M}" \
REFERENCE_MAX_ERROR_BIN_BIAS_NORM_M_REPORT="${REFERENCE_MAX_ERROR_BIN_BIAS_NORM_M}" \
MAX_RENDERED_DELIVERY_LAG_REPORT="${MAX_RENDERED_DELIVERY_LAG}" \
MIN_RENDERED_DELIVERY_RECEIVED_RATIO_REPORT="${MIN_RENDERED_DELIVERY_RECEIVED_RATIO}" \
MAX_TRACKING_STEP_GUARD_CLAMP_RATIO_REPORT="${MAX_TRACKING_STEP_GUARD_CLAMP_RATIO}" \
REFERENCE_TIME_OFFSET_SWEEP_MIN_REPORT="${REFERENCE_TIME_OFFSET_SWEEP_MIN}" \
REFERENCE_TIME_OFFSET_SWEEP_MAX_REPORT="${REFERENCE_TIME_OFFSET_SWEEP_MAX}" \
REFERENCE_TIME_OFFSET_SWEEP_STEP_REPORT="${REFERENCE_TIME_OFFSET_SWEEP_STEP}" \
SE3_PHOTOMETRIC_MAX_SAMPLES_REPORT="${SE3_PHOTOMETRIC_MAX_SAMPLES}" \
SE3_PHOTOMETRIC_RANK_SAMPLES_BY_GRADIENT_REPORT="${SE3_PHOTOMETRIC_RANK_SAMPLES_BY_GRADIENT}" \
SE3_PHOTOMETRIC_USE_RENDERED_GRADIENT_REPORT="${SE3_PHOTOMETRIC_USE_RENDERED_GRADIENT}" \
ENABLE_SE3_PHOTOMETRIC_POSE_CORRECTION_REPORT="${ENABLE_SE3_PHOTOMETRIC_POSE_CORRECTION}" \
SE3_PHOTOMETRIC_POSE_CORRECTION_GAIN_REPORT="${SE3_PHOTOMETRIC_POSE_CORRECTION_GAIN}" \
SE3_PHOTOMETRIC_POSE_CORRECTION_MAX_TRANSLATION_M_REPORT="${SE3_PHOTOMETRIC_POSE_CORRECTION_MAX_TRANSLATION_M}" \
SE3_PHOTOMETRIC_POSE_CORRECTION_MAX_ROTATION_RAD_REPORT="${SE3_PHOTOMETRIC_POSE_CORRECTION_MAX_ROTATION_RAD}" \
SE3_PHOTOMETRIC_POSE_CORRECTION_MAX_DT_NS_REPORT="${SE3_PHOTOMETRIC_POSE_CORRECTION_MAX_DT_NS}" \
RENDERED_IMAGE_QOS_RELIABILITY_REPORT="${RENDERED_IMAGE_QOS_RELIABILITY}" \
RENDERED_IMAGE_QOS_DURABILITY_REPORT="${RENDERED_IMAGE_QOS_DURABILITY}" \
RENDERED_IMAGE_QOS_DEPTH_REPORT="${RENDERED_IMAGE_QOS_DEPTH}" \
ENABLE_RENDERED_FEEDBACK_CONTRACT_REPORT="${ENABLE_RENDERED_FEEDBACK_CONTRACT}" \
RENDERED_FEEDBACK_TOPIC_REPORT="${RENDERED_FEEDBACK_TOPIC}" \
RENDERED_FEEDBACK_QOS_RELIABILITY_REPORT="${RENDERED_FEEDBACK_QOS_RELIABILITY}" \
RENDERED_FEEDBACK_QOS_DURABILITY_REPORT="${RENDERED_FEEDBACK_QOS_DURABILITY}" \
RENDERED_FEEDBACK_QOS_DEPTH_REPORT="${RENDERED_FEEDBACK_QOS_DEPTH}" \
ENABLE_RENDERED_FEEDBACK_INGRESS_QUEUE_REPORT="${ENABLE_RENDERED_FEEDBACK_INGRESS_QUEUE}" \
RENDERED_FEEDBACK_INGRESS_QUEUE_SIZE_REPORT="${RENDERED_FEEDBACK_INGRESS_QUEUE_SIZE}" \
RENDERED_FEEDBACK_INGRESS_DRAIN_MAX_PER_CYCLE_REPORT="${RENDERED_FEEDBACK_INGRESS_DRAIN_MAX_PER_CYCLE}" \
RENDERED_FEEDBACK_INGRESS_DRAIN_PERIOD_MS_REPORT="${RENDERED_FEEDBACK_INGRESS_DRAIN_PERIOD_MS}" \
MAPPER_FEEDBACK_PUBLISH_RENDERED_BEFORE_UPDATE_REPORT="${MAPPER_FEEDBACK_PUBLISH_RENDERED_BEFORE_UPDATE}" \
MAPPER_FEEDBACK_RENDERED_FEEDBACK_SOURCE_STREAM_REPORT="${MAPPER_FEEDBACK_RENDERED_FEEDBACK_SOURCE_STREAM}" \
MAPPER_FEEDBACK_RENDERED_FEEDBACK_IMAGE_TOPIC_REPORT="${MAPPER_FEEDBACK_RENDERED_FEEDBACK_IMAGE_TOPIC}" \
MAPPER_FEEDBACK_RENDERED_FEEDBACK_POSE_TOPIC_REPORT="${MAPPER_FEEDBACK_RENDERED_FEEDBACK_POSE_TOPIC}" \
MAPPER_FEEDBACK_RENDERED_FEEDBACK_IMAGE_QOS_RELIABILITY_REPORT="${MAPPER_FEEDBACK_RENDERED_FEEDBACK_IMAGE_QOS_RELIABILITY}" \
MAPPER_FEEDBACK_RENDERED_FEEDBACK_IMAGE_QOS_HISTORY_REPORT="${MAPPER_FEEDBACK_RENDERED_FEEDBACK_IMAGE_QOS_HISTORY}" \
MAPPER_FEEDBACK_RENDERED_FEEDBACK_IMAGE_QOS_DEPTH_REPORT="${MAPPER_FEEDBACK_RENDERED_FEEDBACK_IMAGE_QOS_DEPTH}" \
MAPPER_FEEDBACK_RENDERED_FEEDBACK_POSE_QOS_RELIABILITY_REPORT="${MAPPER_FEEDBACK_RENDERED_FEEDBACK_POSE_QOS_RELIABILITY}" \
MAPPER_FEEDBACK_RENDERED_FEEDBACK_POSE_QOS_HISTORY_REPORT="${MAPPER_FEEDBACK_RENDERED_FEEDBACK_POSE_QOS_HISTORY}" \
MAPPER_FEEDBACK_RENDERED_FEEDBACK_POSE_QOS_DEPTH_REPORT="${MAPPER_FEEDBACK_RENDERED_FEEDBACK_POSE_QOS_DEPTH}" \
MAPPER_FEEDBACK_SYNC_ANCHOR_STREAM_REPORT="${MAPPER_FEEDBACK_SYNC_ANCHOR_STREAM}" \
RENDERED_FRAME_CACHE_SIZE_REPORT="${RENDERED_FRAME_CACHE_SIZE}" \
OBSERVED_FRAME_CACHE_SIZE_REPORT="${OBSERVED_FRAME_CACHE_SIZE}" \
VISUAL_ALIGNMENT_MAX_SHIFT_PX_REPORT="${VISUAL_ALIGNMENT_MAX_SHIFT_PX}" \
VISUAL_ALIGNMENT_SCORE_MODE_REPORT="${VISUAL_ALIGNMENT_SCORE_MODE}" \
VISUAL_ALIGNMENT_FACTOR_SOURCE_REPORT="${VISUAL_ALIGNMENT_FACTOR_SOURCE}" \
VISUAL_FACTOR_SOURCE_ID_MODE_REPORT="${VISUAL_FACTOR_SOURCE_ID_MODE}" \
VISUAL_FACTOR_REFERENCE_STAMP_MODE_REPORT="${VISUAL_FACTOR_REFERENCE_STAMP_MODE}" \
ENABLE_VISUAL_FACTOR_TIME_INTERPOLATION_REPORT="${ENABLE_VISUAL_FACTOR_TIME_INTERPOLATION}" \
ENABLE_VISUAL_CACHE_RECONCILIATION_REPORT="${ENABLE_VISUAL_CACHE_RECONCILIATION}" \
ENABLE_VISUAL_CACHE_RECONCILIATION_MONOTONIC_UNIQUE_REPORT="${ENABLE_VISUAL_CACHE_RECONCILIATION_MONOTONIC_UNIQUE}" \
ENABLE_VISUAL_PAIR_MONOTONIC_UNIQUE_REPORT="${ENABLE_VISUAL_PAIR_MONOTONIC_UNIQUE}" \
ENABLE_VISUAL_WATERMARK_PAIR_SCHEDULER_REPORT="${ENABLE_VISUAL_WATERMARK_PAIR_SCHEDULER}" \
VISUAL_WATERMARK_PAIR_SCHEDULER_MAX_PAIRS_PER_POINTCLOUD_REPORT="${VISUAL_WATERMARK_PAIR_SCHEDULER_MAX_PAIRS_PER_POINTCLOUD}" \
ENABLE_VISUAL_CALLBACK_FACTOR_INGEST_REPORT="${ENABLE_VISUAL_CALLBACK_FACTOR_INGEST}" \
ENABLE_RENDERED_FEEDBACK_WATERMARK_QUEUE_REPORT="${ENABLE_RENDERED_FEEDBACK_WATERMARK_QUEUE}" \
DEFER_FUTURE_VISUAL_FACTORS_UNTIL_ACTIVE_REPORT="${DEFER_FUTURE_VISUAL_FACTORS_UNTIL_ACTIVE}" \
ENABLE_VISUAL_ADAPTIVE_STATE_RETENTION_REPORT="${ENABLE_VISUAL_ADAPTIVE_STATE_RETENTION}" \
VISUAL_ADAPTIVE_STATE_RETENTION_MARGIN_STATES_REPORT="${VISUAL_ADAPTIVE_STATE_RETENTION_MARGIN_STATES}" \
VISUAL_ADAPTIVE_STATE_RETENTION_MAX_STATES_REPORT="${VISUAL_ADAPTIVE_STATE_RETENTION_MAX_STATES}" \
ENABLE_VISUAL_EXPIRED_FACTOR_PROJECTION_REPORT="${ENABLE_VISUAL_EXPIRED_FACTOR_PROJECTION}" \
ENABLE_VISUAL_MARGINALIZATION_PRIOR_REPORT="${ENABLE_VISUAL_MARGINALIZATION_PRIOR}" \
VISUAL_EXPIRED_FACTOR_PROJECTION_MAX_AGE_S_REPORT="${VISUAL_EXPIRED_FACTOR_PROJECTION_MAX_AGE_S}" \
ENABLE_VISUAL_CACHE_RECONCILIATION_DEFER_TO_POINTCLOUD_REPORT="${ENABLE_VISUAL_CACHE_RECONCILIATION_DEFER_TO_POINTCLOUD}" \
ENABLE_VISUAL_PAIR_PROCESSING_DEFER_TO_POINTCLOUD_REPORT="${ENABLE_VISUAL_PAIR_PROCESSING_DEFER_TO_POINTCLOUD}" \
ENABLE_VISUAL_FACTOR_QUALITY_WEIGHTING_REPORT="${ENABLE_VISUAL_FACTOR_QUALITY_WEIGHTING}" \
VISUAL_FACTOR_QUALITY_MIN_WEIGHT_SCALE_REPORT="${VISUAL_FACTOR_QUALITY_MIN_WEIGHT_SCALE}" \
ENABLE_VISUAL_FACTOR_QUALITY_SELECTION_REPORT="${ENABLE_VISUAL_FACTOR_QUALITY_SELECTION}" \
ENABLE_VISUAL_FACTOR_QUALITY_REFERENCE_CAP_REPORT="${ENABLE_VISUAL_FACTOR_QUALITY_REFERENCE_CAP}" \
VISUAL_FACTOR_QUALITY_SELECTION_MAX_PER_REFERENCE_REPORT="${VISUAL_FACTOR_QUALITY_SELECTION_MAX_PER_REFERENCE}" \
VISUAL_FACTOR_QUALITY_SELECTION_START_AFTER_S_REPORT="${VISUAL_FACTOR_QUALITY_SELECTION_START_AFTER_S}" \
VISUAL_ALIGNMENT_SATURATION_MARGIN_PX_REPORT="${VISUAL_ALIGNMENT_SATURATION_MARGIN_PX}" \
VISUAL_ALIGNMENT_SATURATED_WEIGHT_SCALE_REPORT="${VISUAL_ALIGNMENT_SATURATED_WEIGHT_SCALE}" \
REFERENCE_COVERAGE_MODE_REPORT="${REFERENCE_COVERAGE_MODE}" \
python3 - "${ARTIFACT_DIR}/metrics.json" "${REPORT_JSON}" \
  "${MIN_POSES}" "${MIN_STATUS_SAMPLES}" "${MIN_POINT_FRAMES}" "${REQUIRE_BA_FEEDBACK}" \
  "${REQUIRE_REFERENCE_TRAJECTORY}" "${MIN_REFERENCE_POSES}" "${REQUIRE_NONDEGENERATE_BA}" \
  "${ENABLE_VISUAL_FACTORS}" "${REQUIRE_DESKEW}" "${VISUAL_PENDING_FACTOR_QUEUE_SIZE}" \
  "${VISUAL_FACTOR_MAX_DT_NS}" "${VISUAL_DEPTH_MAX_DT_NS}" \
  "${VISUAL_DEPTH_DILATION_PX}" \
  "${VISUAL_ALIGNMENT_WINDOW_WEIGHT}" "${SE3_PHOTOMETRIC_WINDOW_WEIGHT}" \
  "${SE3_PHOTOMETRIC_MIN_SAMPLES}" \
  "${SE3_PHOTOMETRIC_MIN_HESSIAN_RANK}" "${SE3_PHOTOMETRIC_MAX_HESSIAN_CONDITION}" \
  "${SE3_PHOTOMETRIC_MIN_SAMPLE_INLIER_RATIO}" \
  "${SE3_PHOTOMETRIC_MAX_MEAN_ABS_RESIDUAL_FOR_FACTOR}" \
  "${SE3_PHOTOMETRIC_COVERAGE_GRID_COLS}" "${SE3_PHOTOMETRIC_COVERAGE_GRID_ROWS}" \
  "${SE3_PHOTOMETRIC_MIN_COVERAGE_TILES}" "${MAPPER_FEEDBACK_SYNC_TOLERANCE_SEC}" \
  "${ENABLE_GAUSSIAN_MAP_FEEDBACK}" "${REQUIRE_GAUSSIAN_SNAPSHOT}" \
  "${MAPPER_FEEDBACK_TORCH_DEVICE}" "${MAPPER_FEEDBACK_TORCH_OPTIMIZATION_STEPS}" \
  "${MAPPER_FEEDBACK_ENABLE_TORCH_GAUSSIAN_OPTIMIZATION}" \
  "${MAPPER_FEEDBACK_TORCH_OPTIMIZATION_SAMPLING}" \
  "${MAPPER_FEEDBACK_ENABLE_TORCH_GAUSSIAN_EXTEND_VISIBILITY_FILTER}" \
  "${MAPPER_FEEDBACK_ENABLE_TORCH_GAUSSIAN_PRUNING}" \
  "${MAPPER_FEEDBACK_TORCH_MAX_FOREGROUND}" "${MAPPER_FEEDBACK_TORCH_PRUNE_COUNT_POLICY}" \
  "${MAPPER_FEEDBACK_GAUSSIAN_MAP_PUBLISH_MIN_INTERVAL_SEC}" \
  "${MAPPER_FEEDBACK_GAUSSIAN_MAP_PUBLISH_ON_EMPTY_EXTEND}" \
  "${SLIDING_WINDOW_OPTIMIZE_EVERY_N_FRAMES}" \
  "${SLIDING_WINDOW_MAX_FEEDBACK_TRANSLATION_M}" \
  "${SLIDING_WINDOW_MAX_FEEDBACK_ROTATION_RAD}" \
  "${SLIDING_WINDOW_MAX_FEEDBACK_VELOCITY_MPS}" \
  "${MAPPER_FEEDBACK_RENDER_MODE}" \
  "${TRACKING_MAX_POSE_STEP_M}" "${TRACKING_STEP_GUARD_VELOCITY_SCALE}" \
  "${TRACKING_STEP_GUARD_ACCELERATION_MPS2}" "${TRACKING_STEP_GUARD_MAX_VELOCITY_MPS}" \
  "${TRACKING_STEP_GUARD_MARGIN_M}" \
  "${REFERENCE_TUM_PATH}" "${REFERENCE_TRAJECTORY_ALIGN}" \
  "${REFERENCE_MAX_ASSOCIATION_DT}" "${REFERENCE_MIN_COVERAGE}" \
  "${REFERENCE_MAX_RMSE_M}" "${REFERENCE_MAX_MEAN_M}" \
  "${REFERENCE_MAX_ERROR_M}" "${REFERENCE_MAX_PATH_DRIFT}" <<'PY'
import json
import math
import os
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
visual_factor_max_dt_ns = int(sys.argv[13])
visual_depth_max_dt_ns = int(sys.argv[14])
visual_depth_dilation_px = int(sys.argv[15])
visual_alignment_window_weight = float(sys.argv[16])
visual_alignment_max_shift_px = int(os.environ["VISUAL_ALIGNMENT_MAX_SHIFT_PX_REPORT"])
visual_alignment_score_mode = os.environ["VISUAL_ALIGNMENT_SCORE_MODE_REPORT"]
visual_alignment_factor_source = os.environ["VISUAL_ALIGNMENT_FACTOR_SOURCE_REPORT"]
visual_factor_source_id_mode = os.environ["VISUAL_FACTOR_SOURCE_ID_MODE_REPORT"]
visual_factor_reference_stamp_mode = os.environ["VISUAL_FACTOR_REFERENCE_STAMP_MODE_REPORT"]
enable_rendered_feedback_contract = (
    os.environ["ENABLE_RENDERED_FEEDBACK_CONTRACT_REPORT"].lower() == "true"
)
rendered_feedback_topic = os.environ["RENDERED_FEEDBACK_TOPIC_REPORT"]
rendered_feedback_qos_reliability = os.environ["RENDERED_FEEDBACK_QOS_RELIABILITY_REPORT"]
rendered_feedback_qos_durability = os.environ["RENDERED_FEEDBACK_QOS_DURABILITY_REPORT"]
rendered_feedback_qos_depth = int(os.environ["RENDERED_FEEDBACK_QOS_DEPTH_REPORT"])
enable_rendered_feedback_ingress_queue = (
    os.environ["ENABLE_RENDERED_FEEDBACK_INGRESS_QUEUE_REPORT"].lower() == "true"
)
rendered_feedback_ingress_queue_size = int(
    os.environ["RENDERED_FEEDBACK_INGRESS_QUEUE_SIZE_REPORT"])
rendered_feedback_ingress_drain_max_per_cycle = int(
    os.environ["RENDERED_FEEDBACK_INGRESS_DRAIN_MAX_PER_CYCLE_REPORT"])
rendered_feedback_ingress_drain_period_ms = int(
    os.environ["RENDERED_FEEDBACK_INGRESS_DRAIN_PERIOD_MS_REPORT"])
mapper_feedback_publish_rendered_before_update = (
    os.environ["MAPPER_FEEDBACK_PUBLISH_RENDERED_BEFORE_UPDATE_REPORT"].lower() == "true"
)
mapper_feedback_rendered_feedback_source_stream = os.environ[
    "MAPPER_FEEDBACK_RENDERED_FEEDBACK_SOURCE_STREAM_REPORT"
]
mapper_feedback_rendered_feedback_image_topic = os.environ[
    "MAPPER_FEEDBACK_RENDERED_FEEDBACK_IMAGE_TOPIC_REPORT"
]
mapper_feedback_rendered_feedback_pose_topic = os.environ[
    "MAPPER_FEEDBACK_RENDERED_FEEDBACK_POSE_TOPIC_REPORT"
]
mapper_feedback_rendered_feedback_image_qos_reliability = os.environ[
    "MAPPER_FEEDBACK_RENDERED_FEEDBACK_IMAGE_QOS_RELIABILITY_REPORT"
]
mapper_feedback_rendered_feedback_image_qos_history = os.environ[
    "MAPPER_FEEDBACK_RENDERED_FEEDBACK_IMAGE_QOS_HISTORY_REPORT"
]
mapper_feedback_rendered_feedback_image_qos_depth = int(
    os.environ["MAPPER_FEEDBACK_RENDERED_FEEDBACK_IMAGE_QOS_DEPTH_REPORT"]
)
mapper_feedback_rendered_feedback_pose_qos_reliability = os.environ[
    "MAPPER_FEEDBACK_RENDERED_FEEDBACK_POSE_QOS_RELIABILITY_REPORT"
]
mapper_feedback_rendered_feedback_pose_qos_history = os.environ[
    "MAPPER_FEEDBACK_RENDERED_FEEDBACK_POSE_QOS_HISTORY_REPORT"
]
mapper_feedback_rendered_feedback_pose_qos_depth = int(
    os.environ["MAPPER_FEEDBACK_RENDERED_FEEDBACK_POSE_QOS_DEPTH_REPORT"]
)
enable_visual_factor_time_interpolation = (
    os.environ["ENABLE_VISUAL_FACTOR_TIME_INTERPOLATION_REPORT"].lower() == "true"
)
enable_visual_cache_reconciliation = (
    os.environ["ENABLE_VISUAL_CACHE_RECONCILIATION_REPORT"].lower() == "true"
)
visual_cache_reconciliation_monotonic_unique = (
    os.environ["ENABLE_VISUAL_CACHE_RECONCILIATION_MONOTONIC_UNIQUE_REPORT"].lower() == "true"
)
visual_pair_monotonic_unique = (
    os.environ["ENABLE_VISUAL_PAIR_MONOTONIC_UNIQUE_REPORT"].lower() == "true"
)
enable_visual_watermark_pair_scheduler = (
    os.environ["ENABLE_VISUAL_WATERMARK_PAIR_SCHEDULER_REPORT"].lower() == "true"
)
visual_watermark_pair_scheduler_max_pairs_per_pointcloud = int(
    os.environ["VISUAL_WATERMARK_PAIR_SCHEDULER_MAX_PAIRS_PER_POINTCLOUD_REPORT"]
)
enable_visual_callback_factor_ingest = (
    os.environ["ENABLE_VISUAL_CALLBACK_FACTOR_INGEST_REPORT"].lower() == "true"
)
enable_rendered_feedback_watermark_queue = (
    os.environ["ENABLE_RENDERED_FEEDBACK_WATERMARK_QUEUE_REPORT"].lower() == "true"
)
defer_future_visual_factors_until_active = (
    os.environ["DEFER_FUTURE_VISUAL_FACTORS_UNTIL_ACTIVE_REPORT"].lower() == "true"
)
enable_visual_adaptive_state_retention = (
    os.environ["ENABLE_VISUAL_ADAPTIVE_STATE_RETENTION_REPORT"].lower() == "true"
)
visual_adaptive_state_retention_margin_states = int(
    os.environ["VISUAL_ADAPTIVE_STATE_RETENTION_MARGIN_STATES_REPORT"]
)
visual_adaptive_state_retention_max_states = int(
    os.environ["VISUAL_ADAPTIVE_STATE_RETENTION_MAX_STATES_REPORT"]
)
enable_visual_expired_factor_projection = (
    os.environ["ENABLE_VISUAL_EXPIRED_FACTOR_PROJECTION_REPORT"].lower() == "true"
)
enable_visual_marginalization_prior = (
    os.environ["ENABLE_VISUAL_MARGINALIZATION_PRIOR_REPORT"].lower() == "true"
)
enable_visual_marginalization_prior_batching = (
    os.environ["ENABLE_VISUAL_MARGINALIZATION_PRIOR_BATCHING_REPORT"].lower() == "true"
)
enable_visual_marginalization_prior_saturation_gate = (
    os.environ["ENABLE_VISUAL_MARGINALIZATION_PRIOR_SATURATION_GATE_REPORT"].lower() == "true"
)
visual_marginalization_prior_zero_bias_columns = (
    os.environ["VISUAL_MARGINALIZATION_PRIOR_ZERO_BIAS_COLUMNS_REPORT"].lower() == "true"
)
enable_visual_factor_reference_snapshot = (
    os.environ["ENABLE_VISUAL_FACTOR_REFERENCE_SNAPSHOT_REPORT"].lower() == "true"
)
enable_rendered_feedback_source_pose_reference = (
    os.environ["ENABLE_RENDERED_FEEDBACK_SOURCE_POSE_REFERENCE_REPORT"].lower() == "true"
)
enable_rendered_feedback_source_motion_factor = (
    os.environ["ENABLE_RENDERED_FEEDBACK_SOURCE_MOTION_FACTOR_REPORT"].lower() == "true"
)
enable_rendered_feedback_source_motion_marginalized_prior = (
    os.environ["ENABLE_RENDERED_FEEDBACK_SOURCE_MOTION_MARGINALIZED_PRIOR_REPORT"].lower() == "true"
)
rendered_feedback_source_motion_translation_weight = float(
    os.environ["RENDERED_FEEDBACK_SOURCE_MOTION_TRANSLATION_WEIGHT_REPORT"]
)
rendered_feedback_source_motion_rotation_weight = float(
    os.environ["RENDERED_FEEDBACK_SOURCE_MOTION_ROTATION_WEIGHT_REPORT"]
)
rendered_feedback_source_motion_huber_delta_m = float(
    os.environ["RENDERED_FEEDBACK_SOURCE_MOTION_HUBER_DELTA_M_REPORT"]
)
rendered_feedback_source_motion_rotation_huber_delta_rad = float(
    os.environ["RENDERED_FEEDBACK_SOURCE_MOTION_ROTATION_HUBER_DELTA_RAD_REPORT"]
)
rendered_feedback_source_motion_min_dt_s = float(
    os.environ["RENDERED_FEEDBACK_SOURCE_MOTION_MIN_DT_S_REPORT"]
)
rendered_feedback_source_motion_max_dt_s = float(
    os.environ["RENDERED_FEEDBACK_SOURCE_MOTION_MAX_DT_S_REPORT"]
)
rendered_feedback_source_motion_in_from_frame = (
    os.environ["RENDERED_FEEDBACK_SOURCE_MOTION_IN_FROM_FRAME_REPORT"].lower() == "true"
)
visual_expired_factor_projection_max_age_s = float(
    os.environ["VISUAL_EXPIRED_FACTOR_PROJECTION_MAX_AGE_S_REPORT"]
)
visual_cache_reconciliation_defer_to_pointcloud = (
    os.environ["ENABLE_VISUAL_CACHE_RECONCILIATION_DEFER_TO_POINTCLOUD_REPORT"].lower() == "true"
)
visual_pair_processing_defer_to_pointcloud = (
    os.environ["ENABLE_VISUAL_PAIR_PROCESSING_DEFER_TO_POINTCLOUD_REPORT"].lower() == "true"
)
enable_visual_factor_quality_weighting = (
    os.environ["ENABLE_VISUAL_FACTOR_QUALITY_WEIGHTING_REPORT"].lower() == "true"
)
visual_factor_quality_min_weight_scale = float(
    os.environ["VISUAL_FACTOR_QUALITY_MIN_WEIGHT_SCALE_REPORT"]
)
enable_visual_factor_quality_selection = (
    os.environ["ENABLE_VISUAL_FACTOR_QUALITY_SELECTION_REPORT"].lower() == "true"
)
enable_visual_factor_quality_reference_cap = (
    os.environ["ENABLE_VISUAL_FACTOR_QUALITY_REFERENCE_CAP_REPORT"].lower() == "true"
)
visual_factor_quality_selection_max_per_reference = int(
    os.environ["VISUAL_FACTOR_QUALITY_SELECTION_MAX_PER_REFERENCE_REPORT"]
)
visual_factor_quality_selection_start_after_s = float(
    os.environ["VISUAL_FACTOR_QUALITY_SELECTION_START_AFTER_S_REPORT"]
)
visual_alignment_saturation_margin_px = float(
    os.environ["VISUAL_ALIGNMENT_SATURATION_MARGIN_PX_REPORT"]
)
visual_alignment_saturated_weight_scale = float(
    os.environ["VISUAL_ALIGNMENT_SATURATED_WEIGHT_SCALE_REPORT"]
)
se3_photometric_window_weight = float(sys.argv[17])
se3_photometric_max_samples = int(os.environ["SE3_PHOTOMETRIC_MAX_SAMPLES_REPORT"])
se3_photometric_min_samples = int(sys.argv[18])
se3_photometric_min_hessian_rank = int(sys.argv[19])
se3_photometric_max_hessian_condition = float(sys.argv[20])
se3_photometric_min_sample_inlier_ratio = float(sys.argv[21])
se3_photometric_max_mean_abs_residual_for_factor = float(sys.argv[22])
se3_photometric_coverage_grid_cols = int(sys.argv[23])
se3_photometric_coverage_grid_rows = int(sys.argv[24])
se3_photometric_min_coverage_tiles = int(sys.argv[25])
se3_photometric_rank_samples_by_gradient = (
    os.environ["SE3_PHOTOMETRIC_RANK_SAMPLES_BY_GRADIENT_REPORT"].lower() == "true"
)
se3_photometric_use_rendered_gradient = (
    os.environ["SE3_PHOTOMETRIC_USE_RENDERED_GRADIENT_REPORT"].lower() == "true"
)
enable_se3_photometric_pose_correction = (
    os.environ["ENABLE_SE3_PHOTOMETRIC_POSE_CORRECTION_REPORT"].lower() == "true"
)
se3_photometric_pose_correction_gain = float(
    os.environ["SE3_PHOTOMETRIC_POSE_CORRECTION_GAIN_REPORT"]
)
se3_photometric_pose_correction_max_translation_m = float(
    os.environ["SE3_PHOTOMETRIC_POSE_CORRECTION_MAX_TRANSLATION_M_REPORT"]
)
se3_photometric_pose_correction_max_rotation_rad = float(
    os.environ["SE3_PHOTOMETRIC_POSE_CORRECTION_MAX_ROTATION_RAD_REPORT"]
)
se3_photometric_pose_correction_max_dt_ns = int(
    os.environ["SE3_PHOTOMETRIC_POSE_CORRECTION_MAX_DT_NS_REPORT"]
)
mapper_feedback_sync_tolerance_sec = float(sys.argv[26])
mapper_feedback_sync_anchor_stream = os.environ["MAPPER_FEEDBACK_SYNC_ANCHOR_STREAM_REPORT"]
enable_gaussian_map_feedback = sys.argv[27].lower() == "true"
require_gaussian_snapshot = sys.argv[28].lower() == "true"
mapper_feedback_torch_device = sys.argv[29]
mapper_feedback_torch_optimization_steps = int(sys.argv[30])
mapper_feedback_torch_optimization_enabled = sys.argv[31].lower() == "true"
mapper_feedback_torch_optimization_sampling = sys.argv[32]
mapper_feedback_extend_visibility_filter = sys.argv[33].lower() == "true"
mapper_feedback_pruning = sys.argv[34].lower() == "true"
mapper_feedback_torch_max_foreground = int(sys.argv[35])
mapper_feedback_torch_prune_count_policy = sys.argv[36]
mapper_feedback_gaussian_map_publish_min_interval_sec = float(sys.argv[37])
mapper_feedback_gaussian_map_publish_on_empty_extend = sys.argv[38].lower() == "true"
sliding_window_optimize_every_n_frames = int(sys.argv[39])
sliding_window_max_feedback_translation_m = float(sys.argv[40])
sliding_window_max_feedback_rotation_rad = float(sys.argv[41])
sliding_window_max_feedback_velocity_mps = float(sys.argv[42])
mapper_feedback_render_mode = sys.argv[43]
mapper_feedback_pointcloud_coordinates = os.environ["MAPPER_FEEDBACK_POINTCLOUD_COORDINATES_REPORT"]
mapper_feedback_max_depth = float(os.environ["MAPPER_FEEDBACK_MAX_DEPTH_REPORT"])
mapper_feedback_require_projected_point_color = (
    os.environ["MAPPER_FEEDBACK_REQUIRE_PROJECTED_POINT_COLOR_REPORT"].lower() == "true"
)
mapper_feedback_zbuffer_projected_points = (
    os.environ["MAPPER_FEEDBACK_ZBUFFER_PROJECTED_POINTS_REPORT"].lower() == "true"
)
mapper_feedback_lr = [
    float(value) for value in os.environ["MAPPER_FEEDBACK_LR_REPORT"].split(",")
]
mapper_feedback_torch_optimization_every_n_keyframes = int(
    os.environ["MAPPER_FEEDBACK_OPTIMIZATION_EVERY_REPORT"]
)
mapper_feedback_select_every_k_frame = int(
    os.environ["MAPPER_FEEDBACK_SELECT_EVERY_REPORT"]
)
mapper_feedback_image_qos_reliability = os.environ["MAPPER_FEEDBACK_IMAGE_QOS_RELIABILITY_REPORT"]
mapper_feedback_image_qos_depth = int(os.environ["MAPPER_FEEDBACK_IMAGE_QOS_DEPTH_REPORT"])
gaussian_snapshot_lidar_factor_weight = float(
    os.environ["GAUSSIAN_SNAPSHOT_LIDAR_FACTOR_WEIGHT_REPORT"]
)
gaussian_snapshot_lidar_nearest_distance_m = float(
    os.environ["GAUSSIAN_SNAPSHOT_LIDAR_NEAREST_DISTANCE_M_REPORT"]
)
gaussian_snapshot_lidar_residual_preweight = (
    os.environ["GAUSSIAN_SNAPSHOT_LIDAR_RESIDUAL_PREWEIGHT_REPORT"].lower() == "true"
)
gaussian_snapshot_lidar_pose_correction_enabled = (
    os.environ["GAUSSIAN_SNAPSHOT_LIDAR_POSE_CORRECTION_REPORT"].lower() == "true"
)
gaussian_snapshot_lidar_pose_correction_gain = float(
    os.environ["GAUSSIAN_SNAPSHOT_LIDAR_POSE_CORRECTION_GAIN_REPORT"]
)
gaussian_snapshot_lidar_pose_correction_max_translation_m = float(
    os.environ["GAUSSIAN_SNAPSHOT_LIDAR_POSE_CORRECTION_MAX_TRANSLATION_M_REPORT"]
)
gaussian_snapshot_lidar_pose_correction_max_rotation_rad = float(
    os.environ["GAUSSIAN_SNAPSHOT_LIDAR_POSE_CORRECTION_MAX_ROTATION_RAD_REPORT"]
)
gaussian_snapshot_lidar_pose_correction_min_match_ratio = float(
    os.environ["GAUSSIAN_SNAPSHOT_LIDAR_POSE_CORRECTION_MIN_MATCH_RATIO_REPORT"]
)
gaussian_snapshot_lidar_pose_correction_max_mean_residual_m = float(
    os.environ["GAUSSIAN_SNAPSHOT_LIDAR_POSE_CORRECTION_MAX_MEAN_RESIDUAL_M_REPORT"]
)
gaussian_snapshot_lidar_pose_correction_coverage_grid_cols = int(
    os.environ["GAUSSIAN_SNAPSHOT_LIDAR_POSE_CORRECTION_COVERAGE_GRID_COLS_REPORT"]
)
gaussian_snapshot_lidar_pose_correction_coverage_grid_rows = int(
    os.environ["GAUSSIAN_SNAPSHOT_LIDAR_POSE_CORRECTION_COVERAGE_GRID_ROWS_REPORT"]
)
gaussian_snapshot_lidar_pose_correction_min_coverage_tiles = int(
    os.environ["GAUSSIAN_SNAPSHOT_LIDAR_POSE_CORRECTION_MIN_COVERAGE_TILES_REPORT"]
)
gaussian_snapshot_lidar_pose_correction_bidirectional_max_distance_m = float(
    os.environ["GAUSSIAN_SNAPSHOT_LIDAR_POSE_CORRECTION_BIDIRECTIONAL_MAX_DISTANCE_M_REPORT"]
)
gaussian_snapshot_lidar_plane_factor_enabled = (
    os.environ["GAUSSIAN_SNAPSHOT_LIDAR_PLANE_FACTOR_REPORT"].lower() == "true"
)
gaussian_snapshot_lidar_plane_factor_weight = float(
    os.environ["GAUSSIAN_SNAPSHOT_LIDAR_PLANE_FACTOR_WEIGHT_REPORT"]
)
gaussian_snapshot_lidar_plane_min_anisotropy = float(
    os.environ["GAUSSIAN_SNAPSHOT_LIDAR_PLANE_MIN_ANISOTROPY_REPORT"]
)
lidar_pose_factor_iterations = int(os.environ["LIDAR_POSE_FACTOR_ITERATIONS_REPORT"])
lidar_window_point_factor_weight = float(os.environ["LIDAR_WINDOW_POINT_FACTOR_WEIGHT_REPORT"])
lidar_window_plane_factor_weight = float(os.environ["LIDAR_WINDOW_PLANE_FACTOR_WEIGHT_REPORT"])
lidar_window_confidence_power = float(os.environ["LIDAR_WINDOW_CONFIDENCE_POWER_REPORT"])
sliding_window_max_states = int(os.environ["SLIDING_WINDOW_MAX_STATES_REPORT"])
sliding_window_max_iterations = int(os.environ["SLIDING_WINDOW_MAX_ITERATIONS_REPORT"])
sliding_window_marginalization_prior_weight = float(
    os.environ["SLIDING_WINDOW_MARGINALIZATION_PRIOR_WEIGHT_REPORT"]
)
enable_pre_lio_tracking_step_guard = (
    os.environ["ENABLE_PRE_LIO_TRACKING_STEP_GUARD_REPORT"].lower() == "true"
)
enable_post_ba_tracking_step_guard = (
    os.environ["ENABLE_POST_BA_TRACKING_STEP_GUARD_REPORT"].lower() == "true"
)
pre_lio_tracking_max_pose_step_m = float(os.environ["PRE_LIO_TRACKING_MAX_POSE_STEP_M_REPORT"])
post_ba_tracking_max_pose_step_m = float(os.environ["POST_BA_TRACKING_MAX_POSE_STEP_M_REPORT"])
post_ba_step_guard_confidence_max_pose_step_m = float(
    os.environ["POST_BA_STEP_GUARD_CONFIDENCE_MAX_POSE_STEP_M_REPORT"]
)
post_ba_step_guard_confidence_warmup_marginalizations = int(
    os.environ["POST_BA_STEP_GUARD_CONFIDENCE_WARMUP_MARGINALIZATIONS_REPORT"]
)
post_ba_step_guard_min_lidar_confidence = float(
    os.environ["POST_BA_STEP_GUARD_MIN_LIDAR_CONFIDENCE_REPORT"]
)
post_ba_step_guard_min_visual_inlier_ratio = float(
    os.environ["POST_BA_STEP_GUARD_MIN_VISUAL_INLIER_RATIO_REPORT"]
)
post_ba_step_guard_max_visual_residual = float(
    os.environ["POST_BA_STEP_GUARD_MAX_VISUAL_RESIDUAL_REPORT"]
)
post_ba_step_guard_min_visual_coverage_tiles = int(
    os.environ["POST_BA_STEP_GUARD_MIN_VISUAL_COVERAGE_TILES_REPORT"]
)
post_ba_step_guard_reject_to_pre_ba_over_m = float(
    os.environ["POST_BA_STEP_GUARD_REJECT_TO_PRE_BA_OVER_M_REPORT"]
)
sliding_window_smoothness_position_velocity_weight = float(
    os.environ["SLIDING_WINDOW_SMOOTHNESS_POSITION_VELOCITY_WEIGHT_REPORT"]
)
sliding_window_smoothness_use_motion_targets = (
    os.environ["SLIDING_WINDOW_SMOOTHNESS_USE_MOTION_TARGETS_REPORT"].lower()
    == "true"
)
sliding_window_smoothness_motion_target_min_visual_factors = int(
    os.environ["SLIDING_WINDOW_SMOOTHNESS_MOTION_TARGET_MIN_VISUAL_FACTORS_REPORT"]
)
sliding_window_smoothness_motion_target_min_se3_photometric_factors = int(
    os.environ["SLIDING_WINDOW_SMOOTHNESS_MOTION_TARGET_MIN_SE3_PHOTOMETRIC_FACTORS_REPORT"]
)
sliding_window_smoothness_motion_target_recent_window = int(
    os.environ["SLIDING_WINDOW_SMOOTHNESS_MOTION_TARGET_RECENT_WINDOW_REPORT"]
)
sliding_window_smoothness_motion_target_min_recent_visual_factors = int(
    os.environ["SLIDING_WINDOW_SMOOTHNESS_MOTION_TARGET_MIN_RECENT_VISUAL_FACTORS_REPORT"]
)
sliding_window_smoothness_motion_target_min_recent_se3_photometric_factors = int(
    os.environ["SLIDING_WINDOW_SMOOTHNESS_MOTION_TARGET_MIN_RECENT_SE3_PHOTOMETRIC_FACTORS_REPORT"]
)
sliding_window_smoothness_motion_target_start_after_s = float(
    os.environ["SLIDING_WINDOW_SMOOTHNESS_MOTION_TARGET_START_AFTER_S_REPORT"]
)
sliding_window_smoothness_motion_target_max_rotation_rate_delta_radps = float(
    os.environ["SLIDING_WINDOW_SMOOTHNESS_MOTION_TARGET_MAX_ROTATION_RATE_DELTA_RADPS_REPORT"]
)
sliding_window_smoothness_motion_target_max_position_rate_delta_mps = float(
    os.environ["SLIDING_WINDOW_SMOOTHNESS_MOTION_TARGET_MAX_POSITION_RATE_DELTA_MPS_REPORT"]
)
sliding_window_smoothness_motion_target_max_velocity_acceleration_delta_mps2 = float(
    os.environ[
        "SLIDING_WINDOW_SMOOTHNESS_MOTION_TARGET_MAX_VELOCITY_ACCELERATION_DELTA_MPS2_REPORT"
    ]
)
sliding_window_imu_velocity_prior_weight = float(
    os.environ["SLIDING_WINDOW_IMU_VELOCITY_PRIOR_WEIGHT_REPORT"]
)
sliding_window_gyro_bias_prior_weight = float(
    os.environ["SLIDING_WINDOW_GYRO_BIAS_PRIOR_WEIGHT_REPORT"]
)
sliding_window_accel_bias_prior_weight = float(
    os.environ["SLIDING_WINDOW_ACCEL_BIAS_PRIOR_WEIGHT_REPORT"]
)
sliding_window_max_feedback_gyro_bias_step = float(
    os.environ["SLIDING_WINDOW_MAX_FEEDBACK_GYRO_BIAS_STEP_REPORT"]
)
sliding_window_max_feedback_accel_bias_step = float(
    os.environ["SLIDING_WINDOW_MAX_FEEDBACK_ACCEL_BIAS_STEP_REPORT"]
)
sliding_window_min_bias_feedback_visual_factors = int(
    os.environ["SLIDING_WINDOW_MIN_BIAS_FEEDBACK_VISUAL_FACTORS_REPORT"]
)
sliding_window_bias_feedback_ownership = os.environ[
    "SLIDING_WINDOW_BIAS_FEEDBACK_OWNERSHIP_REPORT"
]
sliding_window_sync_guarded_pose_state = (
    os.environ["SLIDING_WINDOW_SYNC_GUARDED_POSE_STATE_REPORT"].lower() == "true"
)
sliding_window_guarded_pose_prior_translation_weight = float(
    os.environ["SLIDING_WINDOW_GUARDED_POSE_PRIOR_TRANSLATION_WEIGHT_REPORT"]
)
sliding_window_guarded_pose_prior_rotation_weight = float(
    os.environ["SLIDING_WINDOW_GUARDED_POSE_PRIOR_ROTATION_WEIGHT_REPORT"]
)
sliding_window_bias_weight = float(os.environ["SLIDING_WINDOW_BIAS_WEIGHT_REPORT"])
sliding_window_gyro_bias_weight = float(os.environ["SLIDING_WINDOW_GYRO_BIAS_WEIGHT_REPORT"])
sliding_window_accel_bias_weight = float(os.environ["SLIDING_WINDOW_ACCEL_BIAS_WEIGHT_REPORT"])
sliding_window_bias_random_walk_reference_dt_s = float(
    os.environ["SLIDING_WINDOW_BIAS_RANDOM_WALK_REFERENCE_DT_S_REPORT"]
)
sliding_window_gyro_bias_random_walk_sigma = float(
    os.environ["SLIDING_WINDOW_GYRO_BIAS_RANDOM_WALK_SIGMA_REPORT"]
)
sliding_window_accel_bias_random_walk_sigma = float(
    os.environ["SLIDING_WINDOW_ACCEL_BIAS_RANDOM_WALK_SIGMA_REPORT"]
)
enable_sliding_window_relative_translation_factor = (
    os.environ["ENABLE_SLIDING_WINDOW_RELATIVE_TRANSLATION_FACTOR_REPORT"] == "true"
)
sliding_window_relative_translation_weight = float(
    os.environ["SLIDING_WINDOW_RELATIVE_TRANSLATION_WEIGHT_REPORT"]
)
sliding_window_relative_translation_huber_delta_m = float(
    os.environ["SLIDING_WINDOW_RELATIVE_TRANSLATION_HUBER_DELTA_M_REPORT"]
)
sliding_window_relative_translation_in_from_frame = (
    os.environ["SLIDING_WINDOW_RELATIVE_TRANSLATION_IN_FROM_FRAME_REPORT"].lower() == "true"
)
sliding_window_relative_rotation_weight = float(
    os.environ["SLIDING_WINDOW_RELATIVE_ROTATION_WEIGHT_REPORT"]
)
sliding_window_relative_rotation_huber_delta_rad = float(
    os.environ["SLIDING_WINDOW_RELATIVE_ROTATION_HUBER_DELTA_RAD_REPORT"]
)
enable_sliding_window_relative_distance_factor = (
    os.environ["ENABLE_SLIDING_WINDOW_RELATIVE_DISTANCE_FACTOR_REPORT"] == "true"
)
sliding_window_relative_distance_weight = float(
    os.environ["SLIDING_WINDOW_RELATIVE_DISTANCE_WEIGHT_REPORT"]
)
sliding_window_relative_distance_huber_delta_m = float(
    os.environ["SLIDING_WINDOW_RELATIVE_DISTANCE_HUBER_DELTA_M_REPORT"]
)
enable_sliding_window_multihop_relative_translation_factor = (
    os.environ["ENABLE_SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_FACTOR_REPORT"] == "true"
)
sliding_window_multihop_relative_translation_weight = float(
    os.environ["SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_WEIGHT_REPORT"]
)
sliding_window_multihop_relative_translation_huber_delta_m = float(
    os.environ["SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_HUBER_DELTA_M_REPORT"]
)
sliding_window_multihop_relative_translation_in_from_frame = (
    os.environ[
        "SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_IN_FROM_FRAME_REPORT"
    ].lower()
    == "true"
)
sliding_window_multihop_relative_rotation_weight = float(
    os.environ["SLIDING_WINDOW_MULTIHOP_RELATIVE_ROTATION_WEIGHT_REPORT"]
)
sliding_window_multihop_relative_rotation_huber_delta_rad = float(
    os.environ["SLIDING_WINDOW_MULTIHOP_RELATIVE_ROTATION_HUBER_DELTA_RAD_REPORT"]
)
enable_sliding_window_multihop_relative_distance_factor = (
    os.environ["ENABLE_SLIDING_WINDOW_MULTIHOP_RELATIVE_DISTANCE_FACTOR_REPORT"] == "true"
)
sliding_window_multihop_relative_distance_weight = float(
    os.environ["SLIDING_WINDOW_MULTIHOP_RELATIVE_DISTANCE_WEIGHT_REPORT"]
)
sliding_window_multihop_relative_distance_huber_delta_m = float(
    os.environ["SLIDING_WINDOW_MULTIHOP_RELATIVE_DISTANCE_HUBER_DELTA_M_REPORT"]
)
sliding_window_multihop_relative_translation_min_dt_s = float(
    os.environ["SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_MIN_DT_S_REPORT"]
)
sliding_window_multihop_relative_translation_max_dt_s = float(
    os.environ["SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_MAX_DT_S_REPORT"]
)
sliding_window_multihop_relative_translation_max_factors = int(
    os.environ["SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_MAX_FACTORS_REPORT"]
)
enable_sliding_window_delayed_published_multihop_relative_translation_factor = (
    os.environ[
        "ENABLE_SLIDING_WINDOW_DELAYED_PUBLISHED_MULTIHOP_RELATIVE_TRANSLATION_FACTOR_REPORT"
    ].lower()
    == "true"
)
sliding_window_delayed_published_multihop_start_after_s = float(
    os.environ["SLIDING_WINDOW_DELAYED_PUBLISHED_MULTIHOP_START_AFTER_S_REPORT"]
)
sliding_window_delayed_published_multihop_max_factors = int(
    os.environ["SLIDING_WINDOW_DELAYED_PUBLISHED_MULTIHOP_MAX_FACTORS_REPORT"]
)
sliding_window_relative_motion_history_source = os.environ[
    "SLIDING_WINDOW_RELATIVE_MOTION_HISTORY_SOURCE_REPORT"
]
sliding_window_relative_motion_history_published_after_s = float(
    os.environ["SLIDING_WINDOW_RELATIVE_MOTION_HISTORY_PUBLISHED_AFTER_S_REPORT"]
)
post_ba_step_guard_pre_ba_agreement_max_pose_step_m = float(
    os.environ["POST_BA_STEP_GUARD_PRE_BA_AGREEMENT_MAX_POSE_STEP_M_REPORT"]
)
post_ba_step_guard_pre_ba_agreement_late_start_marginalizations = int(
    os.environ["POST_BA_STEP_GUARD_PRE_BA_AGREEMENT_LATE_START_MARGINALIZATIONS_REPORT"]
)
post_ba_step_guard_pre_ba_agreement_late_max_pose_step_m = float(
    os.environ["POST_BA_STEP_GUARD_PRE_BA_AGREEMENT_LATE_MAX_POSE_STEP_M_REPORT"]
)
post_ba_step_guard_pre_ba_agreement_min_cosine = float(
    os.environ["POST_BA_STEP_GUARD_PRE_BA_AGREEMENT_MIN_COSINE_REPORT"]
)
post_ba_step_guard_pre_ba_agreement_max_delta_m = float(
    os.environ["POST_BA_STEP_GUARD_PRE_BA_AGREEMENT_MAX_DELTA_M_REPORT"]
)
post_ba_step_guard_pre_ba_agreement_margin_m = float(
    os.environ["POST_BA_STEP_GUARD_PRE_BA_AGREEMENT_MARGIN_M_REPORT"]
)
post_ba_step_guard_pre_ba_blend_on_clamp = float(
    os.environ["POST_BA_STEP_GUARD_PRE_BA_BLEND_ON_CLAMP_REPORT"]
)
tracking_max_pose_step_m = float(sys.argv[44])
tracking_step_guard_velocity_scale = float(sys.argv[45])
pre_lio_tracking_step_guard_velocity_scale = float(
    os.environ["PRE_LIO_TRACKING_STEP_GUARD_VELOCITY_SCALE_REPORT"]
)
post_ba_tracking_step_guard_velocity_scale = float(
    os.environ["POST_BA_TRACKING_STEP_GUARD_VELOCITY_SCALE_REPORT"]
)
tracking_step_guard_acceleration_mps2 = float(sys.argv[46])
tracking_step_guard_max_velocity_mps = float(sys.argv[47])
tracking_step_guard_margin_m = float(sys.argv[48])
reference_tum_path = Path(sys.argv[49]) if sys.argv[49] else None
reference_trajectory_align = sys.argv[50]
reference_max_association_dt = float(sys.argv[51])
reference_min_coverage = float(sys.argv[52])
reference_coverage_mode = os.environ["REFERENCE_COVERAGE_MODE_REPORT"]
reference_max_rmse_m = float(sys.argv[53])
reference_max_mean_m = float(sys.argv[54])
reference_max_error_m = float(sys.argv[55])
reference_max_path_drift = float(sys.argv[56])
reference_error_bin_count = int(os.environ["REFERENCE_ERROR_BIN_COUNT_REPORT"])
reference_min_error_bin_matches = int(os.environ["REFERENCE_MIN_ERROR_BIN_MATCHES_REPORT"])
reference_max_error_bin_rmse_m = float(os.environ["REFERENCE_MAX_ERROR_BIN_RMSE_M_REPORT"])
reference_max_error_bin_mean_m = float(os.environ["REFERENCE_MAX_ERROR_BIN_MEAN_M_REPORT"])
reference_max_error_bin_bias_norm_m = float(
    os.environ["REFERENCE_MAX_ERROR_BIN_BIAS_NORM_M_REPORT"])
max_rendered_delivery_lag = int(os.environ["MAX_RENDERED_DELIVERY_LAG_REPORT"])
min_rendered_delivery_received_ratio = float(
    os.environ["MIN_RENDERED_DELIVERY_RECEIVED_RATIO_REPORT"])
max_tracking_step_guard_clamp_ratio = float(
    os.environ["MAX_TRACKING_STEP_GUARD_CLAMP_RATIO_REPORT"])
reference_time_offset_sweep_min = float(os.environ["REFERENCE_TIME_OFFSET_SWEEP_MIN_REPORT"])
reference_time_offset_sweep_max = float(os.environ["REFERENCE_TIME_OFFSET_SWEEP_MAX_REPORT"])
reference_time_offset_sweep_step = float(os.environ["REFERENCE_TIME_OFFSET_SWEEP_STEP_REPORT"])
has_external_reference_tum = reference_tum_path is not None and reference_tum_path.is_file() and reference_tum_path.stat().st_size > 0
imu_linear_acceleration_scale = float(os.environ["IMU_LINEAR_ACCELERATION_SCALE_REPORT"])
playback_rate = float(os.environ["PLAYBACK_RATE_REPORT"])
playback_duration_sec = float(os.environ["PLAYBACK_DURATION_REPORT"])
playback_start_offset = float(os.environ["PLAYBACK_START_OFFSET_REPORT"])
playback_clock_topics_all = os.environ["PLAYBACK_CLOCK_TOPICS_ALL_REPORT"].lower() == "true"
timeout_sec = float(os.environ["TIMEOUT_SEC_REPORT"])
post_play_settle_sec = float(os.environ["POST_PLAY_SETTLE_SEC_REPORT"])
post_play_drain_target_poses = int(os.environ["POST_PLAY_DRAIN_TARGET_POSES_REPORT"])
post_play_drain_target_visual_factors = int(
    os.environ["POST_PLAY_DRAIN_TARGET_VISUAL_FACTORS_REPORT"])
post_play_drain_target_se3_factors = int(os.environ["POST_PLAY_DRAIN_TARGET_SE3_FACTORS_REPORT"])
post_play_drain_timeout_sec = float(os.environ["POST_PLAY_DRAIN_TIMEOUT_SEC_REPORT"])
max_lidar_invalid_frames = int(os.environ["MAX_LIDAR_INVALID_FRAMES_REPORT"])
min_status_bin_sample_count = int(os.environ["MIN_STATUS_BIN_SAMPLE_COUNT_REPORT"])
min_visual_factor_delta_per_status_bin = int(
    os.environ["MIN_VISUAL_FACTOR_DELTA_PER_STATUS_BIN_REPORT"])
min_se3_photometric_factor_delta_per_status_bin = int(
    os.environ["MIN_SE3_PHOTOMETRIC_FACTOR_DELTA_PER_STATUS_BIN_REPORT"])
min_motion_target_delta_per_status_bin = int(
    os.environ["MIN_MOTION_TARGET_DELTA_PER_STATUS_BIN_REPORT"])


def count_tum_poses(path: Path | None) -> int:
    if path is None or not path.is_file():
        return 0
    count = 0
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            stripped = line.strip()
            if stripped and not stripped.startswith("#"):
                count += 1
    return count


external_reference_pose_count = count_tum_poses(reference_tum_path)

metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
topic_counts = metrics.get("topic_counts", {})
status = metrics.get("tracking_status", {})
last = status.get("last") or {}
mapping_status = metrics.get("mapping_status", {})
mapping_last = mapping_status.get("last") or {}
errors = []

observed_bias_feedback_ownership = str(
    last.get("sliding_window_bias_feedback_ownership", "") or "")
if observed_bias_feedback_ownership != sliding_window_bias_feedback_ownership:
    errors.append(
        "sliding_window_bias_feedback_ownership mismatch: requested "
        f"{sliding_window_bias_feedback_ownership!r} but tracking status reported "
        f"{observed_bias_feedback_ownership!r}"
    )


def summary_delta(bin_summary, key):
    summary = bin_summary.get("summary", {})
    if not isinstance(summary, dict):
        return 0
    field_summary = summary.get(key, {})
    if not isinstance(field_summary, dict):
        return 0
    try:
        return int(float(field_summary.get("delta", 0)))
    except (TypeError, ValueError):
        return 0


def summary_value(bin_summary, key, statistic, default=0.0):
    summary = bin_summary.get("summary", {})
    if not isinstance(summary, dict):
        return default
    field_summary = summary.get(key, {})
    if not isinstance(field_summary, dict):
        return default
    try:
        value = float(field_summary.get(statistic, default))
    except (TypeError, ValueError):
        return default
    return value if math.isfinite(value) else default


VISUAL_FACTOR_CONTINUITY_FIELDS = (
    "num_raw_images",
    "num_rendered_images",
    "num_rendered_feedbacks",
    "rendered_feedback_ingress_received",
    "rendered_feedback_ingress_drained",
    "rendered_feedback_ingress_drops",
    "rendered_feedback_ingress_queue_size",
    "rendered_feedback_frame_index_regressions",
    "rendered_feedback_preview_index_regressions",
    "rendered_feedback_frame_index_gap_count",
    "rendered_feedback_preview_index_gap_count",
    "rendered_feedback_frame_index_missing",
    "rendered_feedback_preview_index_missing",
    "rendered_feedback_duplicate_source_ids",
    "sliding_window_total_visual_factors",
    "sliding_window_total_se3_photometric_factors",
    "visual_rendered_miss_count",
    "visual_rendered_stale_count",
    "visual_rendered_size_mismatch_count",
    "visual_observed_miss_count",
    "visual_observed_stale_count",
    "visual_observed_size_mismatch_count",
    "visual_depth_miss_count",
    "visual_depth_stale_count",
    "visual_depth_size_mismatch_count",
    "visual_depth_embedded_observed_matches",
    "visual_depth_observed_stamp_matches",
    "visual_depth_source_pointcloud_fallback_queries",
    "visual_depth_source_pointcloud_fallback_matches",
    "visual_depth_source_pointcloud_fallback_misses",
    "visual_se3_photometric_total_batches",
    "visual_se3_photometric_valid_batches",
    "visual_se3_photometric_insufficient_sample_batches",
    "visual_se3_photometric_degenerate_batches",
    "visual_se3_photometric_quality_rejected_batches",
    "visual_alignment_saturated_count",
    "visual_pair_processed_count",
    "visual_pair_duplicate_count",
    "visual_alignment_marginalization_priors",
    "visual_se3_photometric_marginalization_priors",
    "visual_marginalization_prior_skipped_factors",
    "visual_marginalization_prior_saturation_rejected_factors",
    "visual_batched_marginalization_prior_batches",
    "visual_batched_marginalization_prior_visual_factors",
    "visual_batched_marginalization_prior_se3_factors",
    "visual_batched_marginalization_prior_skipped_batches",
    "visual_batched_marginalization_prior_skipped_factors",
    "sliding_window_marginalized_backsubstitutions",
    "sliding_window_marginalized_backsubstitution_chain_updates",
    "sliding_window_marginalized_backsubstitution_interpolations",
    "sliding_window_visual_marginalization_priors",
    "sliding_window_se3_photometric_marginalization_priors",
)


VISUAL_FACTOR_CONTINUITY_VALUE_FIELDS = (
    "visual_rendered_nearest_delta_ns",
    "visual_rendered_nearest_signed_delta_ns",
    "visual_observed_nearest_delta_ns",
    "visual_observed_nearest_signed_delta_ns",
)


def build_visual_factor_continuity(status):
    bins = []
    for bin_summary in status.get("binned_summary", []):
        if not isinstance(bin_summary, dict):
            continue
        item = {
            "index": int(bin_summary.get("index", len(bins))),
            "sample_count": int(bin_summary.get("sample_count", 0) or 0),
        }
        for field_name in VISUAL_FACTOR_CONTINUITY_FIELDS:
            item[f"{field_name}_delta"] = summary_delta(bin_summary, field_name)
        for field_name in VISUAL_FACTOR_CONTINUITY_VALUE_FIELDS:
            item[f"{field_name}_min"] = summary_value(bin_summary, field_name, "min")
            item[f"{field_name}_max"] = summary_value(bin_summary, field_name, "max")
            item[f"{field_name}_last"] = summary_value(bin_summary, field_name, "last")
        bins.append(item)

    if not bins:
        return {
            "available": False,
            "reason": "tracking_status.binned_summary is missing or empty",
            "bins": [],
        }

    def min_delta(field_name):
        return min(item[f"{field_name}_delta"] for item in bins)

    def max_delta(field_name):
        return max(item[f"{field_name}_delta"] for item in bins)

    def min_value(field_name, statistic):
        return min(item[f"{field_name}_{statistic}"] for item in bins)

    def max_value(field_name, statistic):
        return max(item[f"{field_name}_{statistic}"] for item in bins)

    worst_by_factor = sorted(
        bins,
        key=lambda item: (
            item["sliding_window_total_visual_factors_delta"] +
            item["sliding_window_total_se3_photometric_factors_delta"],
            item["sample_count"],
            item["index"],
        ),
    )[:3]
    return {
        "available": True,
        "binned_summary_finalized": bool(status.get("binned_summary_finalized", False)),
        "bin_count": len(bins),
        "min_visual_factor_delta": min_delta("sliding_window_total_visual_factors"),
        "min_se3_photometric_factor_delta": min_delta(
            "sliding_window_total_se3_photometric_factors"),
        "min_se3_valid_batch_delta": min_delta("visual_se3_photometric_valid_batches"),
        "max_rendered_stale_delta": max_delta("visual_rendered_stale_count"),
        "max_observed_stale_delta": max_delta("visual_observed_stale_count"),
        "max_depth_stale_delta": max_delta("visual_depth_stale_count"),
        "max_depth_embedded_observed_match_delta": max_delta(
            "visual_depth_embedded_observed_matches"),
        "max_depth_observed_stamp_match_delta": max_delta(
            "visual_depth_observed_stamp_matches"),
        "max_depth_source_pointcloud_fallback_query_delta": max_delta(
            "visual_depth_source_pointcloud_fallback_queries"),
        "max_depth_source_pointcloud_fallback_match_delta": max_delta(
            "visual_depth_source_pointcloud_fallback_matches"),
        "max_depth_source_pointcloud_fallback_miss_delta": max_delta(
            "visual_depth_source_pointcloud_fallback_misses"),
        "max_rendered_feedback_frame_regression_delta": max_delta(
            "rendered_feedback_frame_index_regressions"),
        "max_rendered_feedback_preview_regression_delta": max_delta(
            "rendered_feedback_preview_index_regressions"),
        "max_rendered_feedback_frame_gap_delta": max_delta(
            "rendered_feedback_frame_index_gap_count"),
        "max_rendered_feedback_preview_gap_delta": max_delta(
            "rendered_feedback_preview_index_gap_count"),
        "max_rendered_feedback_duplicate_source_delta": max_delta(
            "rendered_feedback_duplicate_source_ids"),
        "max_se3_quality_rejected_delta": max_delta(
            "visual_se3_photometric_quality_rejected_batches"),
        "max_visual_alignment_saturated_delta": max_delta("visual_alignment_saturated_count"),
        "max_rendered_nearest_delta_ns": max_value("visual_rendered_nearest_delta_ns", "max"),
        "max_observed_nearest_delta_ns": max_value("visual_observed_nearest_delta_ns", "max"),
        "min_rendered_nearest_signed_delta_ns": min_value(
            "visual_rendered_nearest_signed_delta_ns", "min"),
        "max_rendered_nearest_signed_delta_ns": max_value(
            "visual_rendered_nearest_signed_delta_ns", "max"),
        "min_observed_nearest_signed_delta_ns": min_value(
            "visual_observed_nearest_signed_delta_ns", "min"),
        "max_observed_nearest_signed_delta_ns": max_value(
            "visual_observed_nearest_signed_delta_ns", "max"),
        "worst_factor_bins": worst_by_factor,
        "bins": bins,
    }


visual_factor_continuity = build_visual_factor_continuity(status)


ESTIMATOR_FACTOR_DELTA_FIELDS = (
    "sliding_window_total_imu_factors",
    "sliding_window_total_visual_factors",
    "sliding_window_total_se3_photometric_factors",
    "sliding_window_marginalized_backsubstitutions",
    "sliding_window_marginalized_backsubstitution_chain_updates",
    "sliding_window_marginalized_backsubstitution_interpolations",
    "sliding_window_visual_marginalization_priors",
    "sliding_window_se3_photometric_marginalization_priors",
    "sliding_window_point_factors",
    "sliding_window_plane_factors",
    "sliding_window_relative_translation_factors",
    "sliding_window_relative_distance_factors",
    "sliding_window_smoothness_factors",
    "sliding_window_dense_priors",
)

ESTIMATOR_OBSERVABILITY_VALUE_FIELDS = (
    "sliding_window_normal_equation_min_singular_value",
    "sliding_window_normal_equation_max_singular_value",
    "sliding_window_normal_equation_condition_number",
    "sliding_window_dense_prior_min_singular_value",
    "sliding_window_dense_prior_max_singular_value",
    "sliding_window_dense_prior_gyro_bias_min_singular_value",
    "sliding_window_dense_prior_gyro_bias_max_singular_value",
    "sliding_window_dense_prior_accel_bias_min_singular_value",
    "sliding_window_dense_prior_accel_bias_max_singular_value",
    "sliding_window_gyro_bias_norm",
    "sliding_window_accel_bias_norm",
    "sliding_window_gyro_bias_x",
    "sliding_window_gyro_bias_y",
    "sliding_window_gyro_bias_z",
    "sliding_window_accel_bias_x",
    "sliding_window_accel_bias_y",
    "sliding_window_accel_bias_z",
    "sliding_window_gyro_bias_observability",
    "sliding_window_accel_bias_observability",
)


def build_estimator_observability_continuity(status):
    bins = []
    for bin_summary in status.get("binned_summary", []):
        if not isinstance(bin_summary, dict):
            continue
        item = {
            "index": int(bin_summary.get("index", len(bins))),
            "sample_count": int(bin_summary.get("sample_count", 0) or 0),
        }
        for field_name in ESTIMATOR_FACTOR_DELTA_FIELDS:
            item[f"{field_name}_delta"] = summary_delta(bin_summary, field_name)
        for field_name in ESTIMATOR_OBSERVABILITY_VALUE_FIELDS:
            item[f"{field_name}_min"] = summary_value(bin_summary, field_name, "min")
            item[f"{field_name}_max"] = summary_value(bin_summary, field_name, "max")
            item[f"{field_name}_last"] = summary_value(bin_summary, field_name, "last")
            item[f"{field_name}_delta"] = summary_value(bin_summary, field_name, "delta")
        bins.append(item)

    if not bins:
        return {
            "available": False,
            "reason": "tracking_status.binned_summary is missing or empty",
            "bins": [],
        }

    def min_value(field_name, statistic):
        return min(item[f"{field_name}_{statistic}"] for item in bins)

    def max_value(field_name, statistic):
        return max(item[f"{field_name}_{statistic}"] for item in bins)

    def max_abs_delta(field_name):
        return max(abs(item[f"{field_name}_delta"]) for item in bins)

    worst_information_bins = sorted(
        bins,
        key=lambda item: (
            item["sliding_window_normal_equation_min_singular_value_min"],
            -item["sliding_window_normal_equation_condition_number_max"],
            item["sample_count"],
            item["index"],
        ),
    )[:3]
    worst_bias_bins = sorted(
        bins,
        key=lambda item: (
            -item["sliding_window_gyro_bias_norm_max"],
            -item["sliding_window_accel_bias_norm_max"],
            item["index"],
        ),
    )[:3]
    return {
        "available": True,
        "binned_summary_finalized": bool(status.get("binned_summary_finalized", False)),
        "bin_count": len(bins),
        "min_normal_equation_min_singular_value": min_value(
            "sliding_window_normal_equation_min_singular_value", "min"),
        "max_normal_equation_condition_number": max_value(
            "sliding_window_normal_equation_condition_number", "max"),
        "min_dense_prior_min_singular_value": min_value(
            "sliding_window_dense_prior_min_singular_value", "min"),
        "min_dense_prior_gyro_bias_min_singular_value": min_value(
            "sliding_window_dense_prior_gyro_bias_min_singular_value", "min"),
        "min_dense_prior_accel_bias_min_singular_value": min_value(
            "sliding_window_dense_prior_accel_bias_min_singular_value", "min"),
        "max_dense_prior_gyro_bias_max_singular_value": max_value(
            "sliding_window_dense_prior_gyro_bias_max_singular_value", "max"),
        "max_dense_prior_accel_bias_max_singular_value": max_value(
            "sliding_window_dense_prior_accel_bias_max_singular_value", "max"),
        "max_gyro_bias_norm": max_value("sliding_window_gyro_bias_norm", "max"),
        "max_accel_bias_norm": max_value("sliding_window_accel_bias_norm", "max"),
        "max_abs_gyro_bias_norm_delta": max_abs_delta("sliding_window_gyro_bias_norm"),
        "max_abs_accel_bias_norm_delta": max_abs_delta("sliding_window_accel_bias_norm"),
        "worst_information_bins": worst_information_bins,
        "worst_bias_bins": worst_bias_bins,
        "bins": bins,
    }


estimator_observability_continuity = build_estimator_observability_continuity(status)


MAPPER_FEEDBACK_CONTINUITY_FIELDS = (
    "pointcloud_messages",
    "pose_messages",
    "image_messages",
    "depth_messages",
    "aligned_frames",
    "converted_frames",
    "dropped_pointcloud_messages",
    "dropped_pose_messages",
    "dropped_image_messages",
    "dropped_depth_messages",
    "pending_pointcloud_messages",
    "pending_pose_messages",
    "pending_image_messages",
    "pending_depth_messages",
    "rendered_preview_count",
    "rendered_feedback_published",
    "rendered_feedback_publish_errors",
    "render_error_count",
    "last_aligned_pointcloud_pose_delta_ns",
    "last_aligned_pointcloud_image_delta_ns",
    "max_abs_aligned_pointcloud_pose_delta_ns",
    "max_abs_aligned_pointcloud_image_delta_ns",
    "pointcloud_anchor_pose_too_new_drops",
    "pointcloud_anchor_image_too_new_drops",
    "pointcloud_anchor_depth_too_new_drops",
    "image_anchor_pointcloud_too_new_drops",
    "image_anchor_pose_too_new_drops",
    "image_anchor_depth_too_new_drops",
    "gaussian_extend_count",
    "gaussian_optimization_count",
)


def build_mapper_feedback_continuity(mapping_status):
    bins = []
    for bin_summary in mapping_status.get("binned_summary", []):
        if not isinstance(bin_summary, dict):
            continue
        item = {
            "index": int(bin_summary.get("index", len(bins))),
            "sample_count": int(bin_summary.get("sample_count", 0) or 0),
        }
        for field_name in MAPPER_FEEDBACK_CONTINUITY_FIELDS:
            item[f"{field_name}_delta"] = summary_delta(bin_summary, field_name)
        rendered_delta = item["rendered_preview_count_delta"]
        pose_delta = item["pose_messages_delta"]
        pointcloud_delta = item["pointcloud_messages_delta"]
        item["rendered_per_pose_delta_ratio"] = (
            float(rendered_delta) / float(pose_delta) if pose_delta > 0 else 0.0
        )
        item["rendered_per_pointcloud_delta_ratio"] = (
            float(rendered_delta) / float(pointcloud_delta) if pointcloud_delta > 0 else 0.0
        )
        bins.append(item)

    if not bins:
        return {
            "available": False,
            "reason": "mapping_status.binned_summary is missing or empty",
            "bins": [],
        }

    def min_delta(field_name):
        return min(item[f"{field_name}_delta"] for item in bins)

    def max_delta(field_name):
        return max(item[f"{field_name}_delta"] for item in bins)

    worst_render_bins = sorted(
        bins,
        key=lambda item: (
            item["rendered_preview_count_delta"],
            -item["dropped_pointcloud_messages_delta"] - item["dropped_pose_messages_delta"],
            item["sample_count"],
            item["index"],
        ),
    )[:3]
    return {
        "available": True,
        "binned_summary_finalized": bool(mapping_status.get("binned_summary_finalized", False)),
        "bin_count": len(bins),
        "min_rendered_preview_delta": min_delta("rendered_preview_count"),
        "min_aligned_frame_delta": min_delta("aligned_frames"),
        "max_dropped_pointcloud_delta": max_delta("dropped_pointcloud_messages"),
        "max_dropped_pose_delta": max_delta("dropped_pose_messages"),
        "max_dropped_image_delta": max_delta("dropped_image_messages"),
        "max_dropped_depth_delta": max_delta("dropped_depth_messages"),
        "max_render_error_delta": max_delta("render_error_count"),
        "worst_render_bins": worst_render_bins,
        "bins": bins,
    }


mapper_feedback_continuity = build_mapper_feedback_continuity(mapping_status)


def build_rendered_delivery_continuity(status, mapping_status):
    status_bins = status.get("binned_summary", [])
    mapping_bins = mapping_status.get("binned_summary", [])
    bins = []
    for index, (status_bin, mapping_bin) in enumerate(zip(status_bins, mapping_bins)):
        if not isinstance(status_bin, dict) or not isinstance(mapping_bin, dict):
            continue
        preview_delta = summary_delta(mapping_bin, "rendered_preview_count")
        feedback_delta = summary_delta(mapping_bin, "rendered_feedback_published")
        produced_delta = feedback_delta if feedback_delta > 0 else preview_delta
        ingress_received_delta = summary_delta(status_bin, "rendered_feedback_ingress_received")
        ingress_drained_delta = summary_delta(status_bin, "rendered_feedback_ingress_drained")
        ingress_drops_delta = summary_delta(status_bin, "rendered_feedback_ingress_drops")
        received_delta = summary_delta(status_bin, "num_rendered_images")
        consumed_delta = summary_delta(status_bin, "visual_pair_processed_count")
        bins.append({
            "index": int(status_bin.get("index", index)),
            "tracking_sample_count": int(status_bin.get("sample_count", 0) or 0),
            "mapping_sample_count": int(mapping_bin.get("sample_count", 0) or 0),
            "rendered_preview_count_delta": preview_delta,
            "rendered_feedback_published_delta": feedback_delta,
            "produced_count_delta": produced_delta,
            "rendered_feedback_ingress_received_delta": ingress_received_delta,
            "rendered_feedback_ingress_drained_delta": ingress_drained_delta,
            "rendered_feedback_ingress_drops_delta": ingress_drops_delta,
            "rendered_feedback_ingress_queue_size_max": summary_value(
                status_bin, "rendered_feedback_ingress_queue_size", "max"),
            "num_rendered_images_delta": received_delta,
            "visual_pair_processed_count_delta": consumed_delta,
            "produced_minus_ingress_received_delta": produced_delta - ingress_received_delta,
            "ingress_received_minus_drained_delta": (
                ingress_received_delta - ingress_drained_delta
            ),
            "produced_minus_received_delta": produced_delta - received_delta,
            "received_minus_consumed_delta": received_delta - consumed_delta,
        })

    tracking_last = status.get("last") or {}
    mapping_last = mapping_status.get("last") or {}
    produced_total = int(
        mapping_last.get(
            "rendered_feedback_published",
            mapping_last.get("rendered_preview_count", 0)) or 0)
    ingress_received_total = int(
        tracking_last.get("rendered_feedback_ingress_received", 0) or 0)
    ingress_drained_total = int(
        tracking_last.get("rendered_feedback_ingress_drained", 0) or 0)
    ingress_drops_total = int(
        tracking_last.get("rendered_feedback_ingress_drops", 0) or 0)
    received_total = int(tracking_last.get("num_rendered_images", 0) or 0)
    consumed_total = int(tracking_last.get("visual_pair_processed_count", 0) or 0)
    result = {
        "produced_total": produced_total,
        "ingress_received_total": ingress_received_total,
        "ingress_drained_total": ingress_drained_total,
        "ingress_drops_total": ingress_drops_total,
        "received_total": received_total,
        "consumed_total": consumed_total,
        "produced_minus_ingress_received_total": produced_total - ingress_received_total,
        "ingress_received_minus_drained_total": ingress_received_total - ingress_drained_total,
        "produced_minus_received_total": produced_total - received_total,
        "received_minus_consumed_total": received_total - consumed_total,
        "available": bool(bins),
        "binned_summary_finalized": bool(
            status.get("binned_summary_finalized", False) and
            mapping_status.get("binned_summary_finalized", False)),
        "bin_count": len(bins),
        "bins": bins,
    }
    if not bins:
        result["reason"] = "tracking or mapping binned summary is missing or empty"
        return result
    result["max_produced_minus_received_delta"] = max(
        item["produced_minus_received_delta"] for item in bins)
    result["max_produced_minus_ingress_received_delta"] = max(
        item["produced_minus_ingress_received_delta"] for item in bins)
    result["max_ingress_received_minus_drained_delta"] = max(
        item["ingress_received_minus_drained_delta"] for item in bins)
    result["max_ingress_drops_delta"] = max(
        item["rendered_feedback_ingress_drops_delta"] for item in bins)
    result["max_received_minus_consumed_delta"] = max(
        item["received_minus_consumed_delta"] for item in bins)
    result["worst_delivery_bins"] = sorted(
        bins,
        key=lambda item: (
            -item["produced_minus_received_delta"],
            -item["received_minus_consumed_delta"],
            item["index"],
        ),
    )[:3]
    return result


rendered_delivery_continuity = build_rendered_delivery_continuity(status, mapping_status)


def build_step_guard_continuity(status, trajectory_poses):
    bins = []
    for bin_summary in status.get("binned_summary", []):
        if not isinstance(bin_summary, dict):
            continue
        item = {
            "index": int(bin_summary.get("index", len(bins))),
            "sample_count": int(bin_summary.get("sample_count", 0) or 0),
            "tracking_step_guard_clamps_delta": summary_delta(
                bin_summary, "tracking_step_guard_clamps"),
            "tracking_step_guard_pre_lio_clamps_delta": summary_delta(
                bin_summary, "tracking_step_guard_pre_lio_clamps"),
            "tracking_step_guard_post_ba_clamps_delta": summary_delta(
                bin_summary, "tracking_step_guard_post_ba_clamps"),
            "tracking_step_guard_pre_ba_agreement_releases_delta": summary_delta(
                bin_summary, "tracking_step_guard_pre_ba_agreement_releases"),
            "tracking_step_guard_late_pre_ba_agreement_releases_delta": summary_delta(
                bin_summary, "tracking_step_guard_late_pre_ba_agreement_releases"),
            "tracking_step_guard_last_raw_step_m_max": summary_value(
                bin_summary, "tracking_step_guard_last_raw_step_m", "max"),
            "tracking_step_guard_last_raw_step_m_mean": summary_value(
                bin_summary, "tracking_step_guard_last_raw_step_m", "mean"),
            "tracking_step_guard_last_allowed_step_m_min": summary_value(
                bin_summary, "tracking_step_guard_last_allowed_step_m", "min"),
            "tracking_step_guard_last_allowed_step_m_max": summary_value(
                bin_summary, "tracking_step_guard_last_allowed_step_m", "max"),
            "tracking_step_guard_last_allowed_step_m_mean": summary_value(
                bin_summary, "tracking_step_guard_last_allowed_step_m", "mean"),
        }
        raw_mean = item["tracking_step_guard_last_raw_step_m_mean"]
        allowed_mean = item["tracking_step_guard_last_allowed_step_m_mean"]
        item["allowed_over_raw_step_mean_ratio"] = (
            allowed_mean / raw_mean if raw_mean > 1.0e-12 else 0.0
        )
        bins.append(item)

    tracking_last = status.get("last") or {}
    total_clamps = int(tracking_last.get("tracking_step_guard_clamps", 0) or 0)
    total_pre_lio_clamps = int(
        tracking_last.get("tracking_step_guard_pre_lio_clamps", 0) or 0)
    total_post_ba_clamps = int(
        tracking_last.get("tracking_step_guard_post_ba_clamps", 0) or 0)
    total_releases = int(
        tracking_last.get("tracking_step_guard_pre_ba_agreement_releases", 0) or 0)
    total_late_releases = int(
        tracking_last.get("tracking_step_guard_late_pre_ba_agreement_releases", 0) or 0)
    pose_count = int(trajectory_poses or 0)
    result = {
        "available": bool(bins),
        "binned_summary_finalized": bool(status.get("binned_summary_finalized", False)),
        "bin_count": len(bins),
        "trajectory_poses": pose_count,
        "total_clamps": total_clamps,
        "total_pre_lio_clamps": total_pre_lio_clamps,
        "total_post_ba_clamps": total_post_ba_clamps,
        "total_pre_ba_agreement_releases": total_releases,
        "total_late_pre_ba_agreement_releases": total_late_releases,
        "clamps_per_pose": total_clamps / float(max(1, pose_count)),
        "post_ba_clamps_per_pose": total_post_ba_clamps / float(max(1, pose_count)),
        "pre_ba_agreement_release_per_post_ba_clamp": (
            total_releases / float(max(1, total_post_ba_clamps))
        ),
        "bins": bins,
    }
    if not bins:
        result["reason"] = "tracking_status.binned_summary is missing or empty"
        return result
    result["max_clamp_delta"] = max(item["tracking_step_guard_clamps_delta"] for item in bins)
    result["max_post_ba_clamp_delta"] = max(
        item["tracking_step_guard_post_ba_clamps_delta"] for item in bins)
    result["min_allowed_over_raw_step_mean_ratio"] = min(
        item["allowed_over_raw_step_mean_ratio"] for item in bins
        if item["tracking_step_guard_last_raw_step_m_mean"] > 1.0e-12
    ) if any(item["tracking_step_guard_last_raw_step_m_mean"] > 1.0e-12 for item in bins) else 0.0
    result["worst_clamp_bins"] = sorted(
        bins,
        key=lambda item: (
            -item["tracking_step_guard_clamps_delta"],
            -item["tracking_step_guard_post_ba_clamps_delta"],
            item["index"],
        ),
    )[:3]
    return result


step_guard_continuity = build_step_guard_continuity(
    status, metrics.get("trajectory_poses", 0))


MOTION_TARGET_DELTA_FIELDS = (
    "sliding_window_smoothness_motion_target_applied_count",
    "sliding_window_smoothness_motion_target_support_skip_count",
    "sliding_window_smoothness_motion_target_recent_support_skip_count",
    "sliding_window_smoothness_motion_target_warmup_skip_count",
    "sliding_window_smoothness_motion_target_history_miss_count",
    "sliding_window_smoothness_motion_target_invalid_count",
    "sliding_window_smoothness_motion_target_clamp_count",
)

MOTION_TARGET_VALUE_FIELDS = (
    "sliding_window_smoothness_motion_target_last_rotation_rate_delta_norm",
    "sliding_window_smoothness_motion_target_last_position_rate_delta_norm",
    "sliding_window_smoothness_motion_target_last_velocity_acceleration_delta_norm",
    "sliding_window_smoothness_motion_target_max_rotation_rate_delta_norm",
    "sliding_window_smoothness_motion_target_max_position_rate_delta_norm",
    "sliding_window_smoothness_motion_target_max_velocity_acceleration_delta_norm",
    "sliding_window_smoothness_motion_target_recent_visual_factors",
    "sliding_window_smoothness_motion_target_recent_se3_photometric_factors",
)


def build_motion_target_continuity(status):
    bins = []
    for bin_summary in status.get("binned_summary", []):
        if not isinstance(bin_summary, dict):
            continue
        item = {
            "index": int(bin_summary.get("index", len(bins))),
            "sample_count": int(bin_summary.get("sample_count", 0) or 0),
        }
        for field_name in MOTION_TARGET_DELTA_FIELDS:
            item[f"{field_name}_delta"] = summary_delta(bin_summary, field_name)
        for field_name in MOTION_TARGET_VALUE_FIELDS:
            item[f"{field_name}_min"] = summary_value(bin_summary, field_name, "min")
            item[f"{field_name}_max"] = summary_value(bin_summary, field_name, "max")
            item[f"{field_name}_last"] = summary_value(bin_summary, field_name, "last")
        bins.append(item)

    tracking_last = status.get("last") or {}
    result = {
        "available": bool(bins),
        "binned_summary_finalized": bool(status.get("binned_summary_finalized", False)),
        "bin_count": len(bins),
        "total_applied": int(
            tracking_last.get("sliding_window_smoothness_motion_target_applied_count", 0) or 0),
        "total_support_skips": int(
            tracking_last.get(
                "sliding_window_smoothness_motion_target_support_skip_count", 0) or 0),
        "total_recent_support_skips": int(
            tracking_last.get(
                "sliding_window_smoothness_motion_target_recent_support_skip_count", 0) or 0),
        "total_warmup_skips": int(
            tracking_last.get(
                "sliding_window_smoothness_motion_target_warmup_skip_count", 0) or 0),
        "total_history_misses": int(
            tracking_last.get(
                "sliding_window_smoothness_motion_target_history_miss_count", 0) or 0),
        "total_invalid": int(
            tracking_last.get("sliding_window_smoothness_motion_target_invalid_count", 0) or 0),
        "total_clamps": int(
            tracking_last.get("sliding_window_smoothness_motion_target_clamp_count", 0) or 0),
        "bins": bins,
    }
    if not bins:
        result["reason"] = "tracking_status.binned_summary is missing or empty"
        return result

    def min_delta(field_name):
        return min(item[f"{field_name}_delta"] for item in bins)

    def max_delta(field_name):
        return max(item[f"{field_name}_delta"] for item in bins)

    def max_value(field_name, statistic):
        return max(item[f"{field_name}_{statistic}"] for item in bins)

    result.update({
        "min_applied_delta": min_delta(
            "sliding_window_smoothness_motion_target_applied_count"),
        "max_applied_delta": max_delta(
            "sliding_window_smoothness_motion_target_applied_count"),
        "max_support_skip_delta": max_delta(
            "sliding_window_smoothness_motion_target_support_skip_count"),
        "max_recent_support_skip_delta": max_delta(
            "sliding_window_smoothness_motion_target_recent_support_skip_count"),
        "max_warmup_skip_delta": max_delta(
            "sliding_window_smoothness_motion_target_warmup_skip_count"),
        "max_history_miss_delta": max_delta(
            "sliding_window_smoothness_motion_target_history_miss_count"),
        "max_invalid_delta": max_delta(
            "sliding_window_smoothness_motion_target_invalid_count"),
        "max_clamp_delta": max_delta(
            "sliding_window_smoothness_motion_target_clamp_count"),
        "max_rotation_rate_delta_norm": max_value(
            "sliding_window_smoothness_motion_target_max_rotation_rate_delta_norm", "max"),
        "max_position_rate_delta_norm": max_value(
            "sliding_window_smoothness_motion_target_max_position_rate_delta_norm", "max"),
        "max_velocity_acceleration_delta_norm": max_value(
            "sliding_window_smoothness_motion_target_max_velocity_acceleration_delta_norm", "max"),
        "worst_activation_bins": sorted(
            bins,
            key=lambda item: (
                item["sliding_window_smoothness_motion_target_applied_count_delta"],
                -item["sliding_window_smoothness_motion_target_support_skip_count_delta"] -
                item[
                    "sliding_window_smoothness_motion_target_recent_support_skip_count_delta"],
                -item["sliding_window_smoothness_motion_target_history_miss_count_delta"],
                item["index"],
            ),
        )[:3],
    })
    return result


motion_target_continuity = build_motion_target_continuity(status)

if metrics.get("trajectory_poses", 0) < min_poses:
    errors.append(f"trajectory poses {metrics.get('trajectory_poses', 0)} < {min_poses}")
if (
    post_play_drain_target_poses > 0
    and int(metrics.get("trajectory_poses", 0) or 0) < post_play_drain_target_poses
):
    errors.append(
        "post-play trajectory drain target not reached: "
        f"{metrics.get('trajectory_poses', 0)} < {post_play_drain_target_poses}"
    )
if (
    post_play_drain_target_visual_factors > 0
    and int(last.get("sliding_window_total_visual_factors", 0) or 0) <
    post_play_drain_target_visual_factors
):
    errors.append(
        "post-play visual-factor drain target not reached: "
        f"{last.get('sliding_window_total_visual_factors', 0)} < "
        f"{post_play_drain_target_visual_factors}"
    )
if (
    post_play_drain_target_se3_factors > 0
    and int(last.get("sliding_window_total_se3_photometric_factors", 0) or 0) <
    post_play_drain_target_se3_factors
):
    errors.append(
        "post-play SE3 photometric-factor drain target not reached: "
        f"{last.get('sliding_window_total_se3_photometric_factors', 0)} < "
        f"{post_play_drain_target_se3_factors}"
    )
if status.get("samples", 0) < min_status:
    errors.append(f"tracking status samples {status.get('samples', 0)} < {min_status}")
if topic_counts.get("/points_for_gs", 0) < min_point_frames:
    errors.append(f"/points_for_gs frames {topic_counts.get('/points_for_gs', 0)} < {min_point_frames}")
if max_rendered_delivery_lag > 0:
    rendered_lag = int(rendered_delivery_continuity.get("produced_minus_received_total", 0))
    if rendered_lag > max_rendered_delivery_lag:
        errors.append(
            f"rendered delivery lag {rendered_lag} > {max_rendered_delivery_lag}"
        )
if min_rendered_delivery_received_ratio > 0.0:
    produced_total = int(rendered_delivery_continuity.get("produced_total", 0))
    received_total = int(rendered_delivery_continuity.get("received_total", 0))
    if produced_total <= 0:
        errors.append("rendered delivery ratio requested but produced_total is zero")
    else:
        received_ratio = received_total / float(produced_total)
        if received_ratio < min_rendered_delivery_received_ratio:
            errors.append(
                f"rendered delivery received ratio {received_ratio:.6f} "
                f"< {min_rendered_delivery_received_ratio:.6f}"
            )
if max_tracking_step_guard_clamp_ratio > 0.0:
    clamp_ratio = float(step_guard_continuity.get("clamps_per_pose", 0.0))
    if clamp_ratio > max_tracking_step_guard_clamp_ratio:
        errors.append(
            f"tracking step-guard clamp ratio {clamp_ratio:.6f} "
            f"> {max_tracking_step_guard_clamp_ratio:.6f}"
        )
if (
    require_reference_trajectory
    and metrics.get("reference_trajectory_poses", 0) < min_reference_poses
    and not has_external_reference_tum
):
    errors.append(
        f"reference trajectory poses {metrics.get('reference_trajectory_poses', 0)} < {min_reference_poses}")

for key in (
    "image_stamp_regressions",
    "pointcloud_stamp_regressions",
    "imu_stamp_regressions",
    "external_odometry_prior_stamp_regressions",
    "imu_invalid_measurements",
    "external_odometry_prior_invalid_messages",
    "lidar_invalid_points",
    "lidar_invalid_point_times",
    "lidar_out_of_range_point_times",
    "pointcloud_imu_wait_dropped",
    "sliding_window_invalid_candidate_steps",
    "sliding_window_linearization_failure_count",
    "sliding_window_linear_solve_failure_count",
    "sliding_window_invalid_optimized_states",
    "sliding_window_orphan_factors",
):
    if int(last.get(key, 0)) != 0:
        errors.append(f"{key} is {last.get(key)}")

numeric_jacobian_blocks = int(last.get("sliding_window_numeric_jacobian_blocks", 0) or 0)
numeric_jacobian_columns = int(last.get("sliding_window_numeric_jacobian_columns", 0) or 0)
if not enable_visual_factor_time_interpolation:
    if numeric_jacobian_blocks != 0:
        errors.append(f"sliding_window_numeric_jacobian_blocks is {numeric_jacobian_blocks}")
    if numeric_jacobian_columns != 0:
        errors.append(f"sliding_window_numeric_jacobian_columns is {numeric_jacobian_columns}")

lidar_invalid_frames = int(last.get("lidar_invalid_frames", 0))
if lidar_invalid_frames > max_lidar_invalid_frames:
    errors.append(f"lidar_invalid_frames {lidar_invalid_frames} > {max_lidar_invalid_frames}")

if int(last.get("sliding_window_normal_equation_rows", 0)) <= 0:
    errors.append("sliding_window_normal_equation_rows is zero")
if int(last.get("sliding_window_total_imu_factors", 0)) <= 0:
    errors.append("sliding_window_total_imu_factors is zero")
if (
    int(last.get("sliding_window_point_factors", 0)) <= 0
    and int(last.get("sliding_window_plane_factors", 0)) <= 0
    and int(last.get("total_window_point_correspondences", 0)) <= 0
    and int(last.get("total_window_plane_correspondences", 0)) <= 0
):
    errors.append("LiDAR window factors and cumulative LiDAR correspondences are zero")
if require_gaussian_snapshot:
    if not bool(last.get("gaussian_snapshot_complete", False)):
        errors.append("gaussian_snapshot_complete is false")
    if int(last.get("gaussian_snapshot_points", 0)) <= 0:
        errors.append("gaussian_snapshot_points is zero")
    expected_chunks = int(last.get("gaussian_snapshot_expected_chunks", 0))
    received_chunks = int(last.get("gaussian_snapshot_chunks_received", 0))
    if expected_chunks <= 0:
        errors.append("gaussian_snapshot_expected_chunks is zero")
    elif received_chunks != expected_chunks:
        errors.append(
            f"gaussian snapshot chunks incomplete: {received_chunks}/{expected_chunks}")
if enable_gaussian_map_feedback:
    for key in (
        "pointcloud_messages",
        "pose_messages",
        "image_messages",
        "aligned_frames",
        "converted_frames",
        "dropped_pointcloud_messages",
        "dropped_pose_messages",
        "dropped_image_messages",
        "dropped_depth_messages",
        "pending_pointcloud_messages",
        "pending_pose_messages",
        "pending_image_messages",
        "pending_depth_messages",
        "rendered_preview_count",
        "rendered_feedback_published",
        "rendered_feedback_publish_errors",
        "render_error_count",
    ):
        if key not in mapping_last:
            errors.append(f"mapping_status.{key} is missing")
    if int(mapping_last.get("rendered_preview_count", 0)) <= 0:
        errors.append("mapping_status.rendered_preview_count is zero")
    if (
        enable_rendered_feedback_contract
        and int(mapping_last.get("rendered_feedback_published", 0)) <= 0
    ):
        errors.append("mapping_status.rendered_feedback_published is zero")
    if int(mapping_last.get("rendered_feedback_publish_errors", 0)) != 0:
        errors.append(
            "mapping_status.rendered_feedback_publish_errors is "
            f"{mapping_last.get('rendered_feedback_publish_errors')}")
    if int(mapping_last.get("render_error_count", 0)) != 0:
        errors.append(f"mapping_status.render_error_count is {mapping_last.get('render_error_count')}")
if (
    enable_gaussian_map_feedback
    and mapper_feedback_torch_optimization_enabled
    and mapper_feedback_torch_optimization_steps > 0
):
    optimization_count = int(mapping_last.get("gaussian_optimization_count", 0))
    optimization_steps = int(mapping_last.get("gaussian_optimization_steps", 0))
    optimization_errors = int(mapping_last.get("gaussian_optimization_errors", 0))
    if optimization_count <= 0 or optimization_steps <= 0:
        errors.append(
            "mapper Torch optimization requested but mapping status reports "
            f"optimization_count={optimization_count}, optimization_steps={optimization_steps}")
    if optimization_errors != 0:
        errors.append(f"gaussian_optimization_errors is {optimization_errors}")
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
if (
    not enable_visual_factors
    and (
        min_visual_factor_delta_per_status_bin > 0
        or min_se3_photometric_factor_delta_per_status_bin > 0
    )
):
    errors.append("per-bin visual factor gates require --enable-visual-factors")
if min_motion_target_delta_per_status_bin > 0 and not sliding_window_smoothness_use_motion_targets:
    errors.append("per-bin motion-target gates require --sliding-window-smoothness-use-motion-targets")
if (
    min_status_bin_sample_count > 0
    or min_visual_factor_delta_per_status_bin > 0
    or min_se3_photometric_factor_delta_per_status_bin > 0
    or min_motion_target_delta_per_status_bin > 0
):
    binned_summary = status.get("binned_summary", [])
    if not bool(status.get("binned_summary_finalized", False)):
        errors.append("tracking_status.binned_summary_finalized is false")
    if not isinstance(binned_summary, list) or not binned_summary:
        errors.append("tracking_status.binned_summary is missing or empty")
    else:
        for bin_index, bin_summary in enumerate(binned_summary):
            if not isinstance(bin_summary, dict):
                errors.append(f"tracking_status.binned_summary[{bin_index}] is not an object")
                continue
            sample_count = int(bin_summary.get("sample_count", 0))
            if min_status_bin_sample_count > 0 and sample_count < min_status_bin_sample_count:
                errors.append(
                    f"tracking_status bin {bin_index} sample_count {sample_count} "
                    f"< {min_status_bin_sample_count}")
            if enable_visual_factors:
                visual_delta = summary_delta(
                    bin_summary, "sliding_window_total_visual_factors")
                if (
                    min_visual_factor_delta_per_status_bin > 0
                    and visual_delta < min_visual_factor_delta_per_status_bin
                ):
                    errors.append(
                        f"tracking_status bin {bin_index} visual-factor delta "
                        f"{visual_delta} < {min_visual_factor_delta_per_status_bin}")
                se3_delta = summary_delta(
                    bin_summary, "sliding_window_total_se3_photometric_factors")
                if (
                    min_se3_photometric_factor_delta_per_status_bin > 0
                    and se3_delta < min_se3_photometric_factor_delta_per_status_bin
                ):
                    errors.append(
                        f"tracking_status bin {bin_index} SE3 photometric-factor delta "
                        f"{se3_delta} < {min_se3_photometric_factor_delta_per_status_bin}")
            if sliding_window_smoothness_use_motion_targets:
                motion_target_delta = summary_delta(
                    bin_summary, "sliding_window_smoothness_motion_target_applied_count")
                if (
                    min_motion_target_delta_per_status_bin > 0
                    and motion_target_delta < min_motion_target_delta_per_status_bin
                ):
                    errors.append(
                        f"tracking_status bin {bin_index} motion-target delta "
                        f"{motion_target_delta} < {min_motion_target_delta_per_status_bin}")
if (
    sliding_window_smoothness_use_motion_targets
    and int(last.get("sliding_window_smoothness_motion_target_applied_count", 0) or 0) <= 0
):
    errors.append("sliding-window smoothness motion targets requested but no targets were applied")
if enable_visual_factors:
    for key in (
        "visual_alignment_pending_queue_size",
        "visual_se3_photometric_pending_queue_size",
        "visual_alignment_pending_stale_drops",
        "visual_se3_photometric_pending_stale_drops",
        "visual_alignment_pending_queue_trim_drops",
        "visual_se3_photometric_pending_queue_trim_drops",
        "visual_alignment_pending_expired_drops",
        "visual_se3_photometric_pending_expired_drops",
        "visual_depth_embedded_observed_matches",
        "visual_depth_observed_stamp_matches",
        "visual_depth_source_pointcloud_fallback_queries",
        "visual_depth_source_pointcloud_fallback_matches",
        "visual_depth_source_pointcloud_fallback_misses",
        "rendered_feedback_contract_enabled",
        "num_rendered_feedbacks",
        "rendered_feedback_ingress_queue_enabled",
        "rendered_feedback_ingress_received",
        "rendered_feedback_ingress_drained",
        "rendered_feedback_ingress_drops",
        "rendered_feedback_ingress_queue_size",
        "rendered_feedback_ingress_queue_peak_size",
        "rendered_feedback_embedded_depth_pairs",
        "rendered_feedback_embedded_depth_invalid",
        "rendered_feedback_source_pose_reference_enabled",
        "rendered_feedback_source_pose_reference_factors",
        "rendered_feedback_source_pose_invalid",
        "rendered_feedback_source_motion_factor_enabled",
        "rendered_feedback_source_motion_marginalized_prior_enabled",
        "rendered_feedback_source_motion_queued_factors",
        "rendered_feedback_source_motion_factors",
        "rendered_feedback_source_motion_pending_factors",
        "rendered_feedback_source_motion_invalid",
        "rendered_feedback_source_motion_dt_skip_count",
        "rendered_feedback_source_motion_stale_drops",
        "rendered_feedback_source_motion_future_deferrals",
        "rendered_feedback_source_motion_marginalized_priors",
        "rendered_feedback_source_motion_marginalized_source_factors",
        "rendered_feedback_source_motion_marginalized_prior_skips",
        "last_rendered_feedback_frame_index",
        "last_rendered_feedback_preview_index",
        "rendered_feedback_frame_index_regressions",
        "rendered_feedback_preview_index_regressions",
        "rendered_feedback_frame_index_gap_count",
        "rendered_feedback_preview_index_gap_count",
        "rendered_feedback_frame_index_missing",
        "rendered_feedback_preview_index_missing",
        "rendered_feedback_duplicate_source_ids",
        "last_rendered_feedback_observed_delta_ns",
        "last_rendered_feedback_pose_delta_ns",
        "last_rendered_feedback_pointcloud_delta_ns",
        "visual_factor_reference_stamp_mode",
        "visual_watermark_pair_scheduler_enabled",
        "visual_watermark_pair_scheduler_processed_pairs",
        "visual_watermark_pair_scheduler_deferred_pairs",
        "visual_adaptive_state_retention_enabled",
        "visual_render_backlog_frames",
        "visual_expired_factor_projection_enabled",
        "visual_marginalization_prior_enabled",
        "visual_marginalization_prior_batching_enabled",
        "visual_marginalization_prior_saturation_gate_enabled",
        "visual_marginalization_prior_zero_bias_columns",
        "visual_alignment_expired_projected_factors",
        "visual_se3_photometric_expired_projected_factors",
        "visual_expired_projection_skipped_factors",
        "visual_alignment_marginalization_priors",
        "visual_se3_photometric_marginalization_priors",
        "visual_marginalization_prior_skipped_factors",
        "visual_marginalization_prior_saturation_rejected_factors",
        "visual_batched_marginalization_prior_batches",
        "visual_batched_marginalization_prior_visual_factors",
        "visual_batched_marginalization_prior_se3_factors",
        "visual_batched_marginalization_prior_skipped_batches",
        "visual_batched_marginalization_prior_skipped_factors",
        "sliding_window_visual_marginalization_priors",
        "sliding_window_se3_photometric_marginalization_priors",
        "visual_se3_photometric_total_batches",
        "visual_se3_photometric_valid_batches",
        "visual_se3_photometric_degenerate_batches",
        "visual_se3_photometric_quality_rejected_batches",
        "visual_se3_photometric_total_samples",
        "visual_se3_photometric_sampled_depth",
        "visual_se3_photometric_sample_inlier_ratio",
        "visual_se3_photometric_coverage_tiles",
        "visual_se3_photometric_coverage_total_tiles",
        "visual_se3_photometric_hessian_rank",
        "visual_se3_photometric_hessian_condition_number",
        "visual_se3_photometric_last_accepted_hessian_rank",
        "visual_se3_photometric_last_accepted_hessian_condition_number",
        "visual_se3_photometric_last_accepted_sampled_depth",
        "visual_se3_photometric_last_accepted_samples",
        "visual_se3_photometric_last_accepted_sample_inlier_ratio",
        "visual_se3_photometric_last_accepted_coverage_tiles",
        "visual_se3_photometric_last_accepted_coverage_total_tiles",
        "visual_se3_photometric_last_accepted_mean_abs_residual",
    ):
        if key not in last:
            errors.append(f"{key} is missing")
    if enable_rendered_feedback_source_pose_reference:
        if int(last.get("rendered_feedback_source_pose_reference_factors", 0)) <= 0:
            errors.append("rendered_feedback_source_pose_reference_factors is zero")
        if int(last.get("rendered_feedback_source_pose_invalid", 0)) != 0:
            errors.append(
                "rendered_feedback_source_pose_invalid is "
                f"{last.get('rendered_feedback_source_pose_invalid')}")
    if enable_rendered_feedback_source_motion_factor:
        if int(last.get("rendered_feedback_source_motion_queued_factors", 0)) <= 0:
            errors.append("rendered_feedback_source_motion_queued_factors is zero")
        active_source_motion = int(last.get("rendered_feedback_source_motion_factors", 0))
        marginalized_source_motion = int(
            last.get("rendered_feedback_source_motion_marginalized_priors", 0))
        if active_source_motion + marginalized_source_motion <= 0:
            errors.append(
                "rendered_feedback_source_motion_factors plus "
                "rendered_feedback_source_motion_marginalized_priors is zero")
        if int(last.get("rendered_feedback_source_motion_invalid", 0)) != 0:
            errors.append(
                "rendered_feedback_source_motion_invalid is "
                f"{last.get('rendered_feedback_source_motion_invalid')}")
    for key in ("visual_alignment_pending_queue_size", "visual_se3_photometric_pending_queue_size"):
        if int(last.get(key, 0)) > visual_pending_factor_queue_size:
            errors.append(f"{key} exceeds report queue budget: {last.get(key)}")
    for key in ("visual_alignment_pending_stale_drops", "visual_se3_photometric_pending_stale_drops"):
        if int(last.get(key, 0)) != 0:
            errors.append(f"{key} is {last.get(key)}")
    if int(last.get("visual_se3_photometric_total_batches", 0)) <= 0:
        errors.append("visual_se3_photometric_total_batches is zero")
    if enable_visual_marginalization_prior_batching:
        if not bool(last.get("visual_marginalization_prior_batching_enabled", False)):
            errors.append("visual_marginalization_prior_batching_enabled is false")
        if int(last.get("visual_batched_marginalization_prior_skipped_batches", 0)) != 0:
            errors.append(
                "visual_batched_marginalization_prior_skipped_batches is "
                f"{last.get('visual_batched_marginalization_prior_skipped_batches')}")
    if enable_visual_marginalization_prior_saturation_gate:
        if not bool(last.get("visual_marginalization_prior_saturation_gate_enabled", False)):
            errors.append("visual_marginalization_prior_saturation_gate_enabled is false")
    if enable_rendered_feedback_contract:
        for key in (
            "rendered_feedback_frame_index_regressions",
            "rendered_feedback_preview_index_regressions",
            "rendered_feedback_duplicate_source_ids",
        ):
            if int(last.get(key, 0)) != 0:
                errors.append(f"{key} is {last.get(key)}")
        if enable_rendered_feedback_ingress_queue:
            if not bool(last.get("rendered_feedback_ingress_queue_enabled", False)):
                errors.append("rendered_feedback_ingress_queue_enabled is false")
            if int(last.get("rendered_feedback_ingress_drops", 0)) != 0:
                errors.append(
                    "rendered_feedback_ingress_drops is "
                    f"{last.get('rendered_feedback_ingress_drops')}")
    if int(last.get("visual_se3_photometric_valid_batches", 0)) <= 0:
        errors.append("visual_se3_photometric_valid_batches is zero")
    if int(last.get("visual_se3_photometric_total_samples", 0)) <= 0:
        errors.append("visual_se3_photometric_total_samples is zero")
    if int(last.get("visual_se3_photometric_last_accepted_hessian_rank", 0)) < se3_photometric_min_hessian_rank:
        errors.append(
            "visual_se3_photometric_last_accepted_hessian_rank "
            f"{last.get('visual_se3_photometric_last_accepted_hessian_rank', 0)} "
            f"< {se3_photometric_min_hessian_rank}"
        )
    hessian_condition = float(
        last.get("visual_se3_photometric_last_accepted_hessian_condition_number", 0.0))
    if (
        se3_photometric_max_hessian_condition > 0.0
        and (hessian_condition <= 0.0 or hessian_condition > se3_photometric_max_hessian_condition)
    ):
        errors.append(
            "visual_se3_photometric_last_accepted_hessian_condition_number "
            f"{hessian_condition} > {se3_photometric_max_hessian_condition}"
        )
    sample_inlier_ratio = float(
        last.get("visual_se3_photometric_last_accepted_sample_inlier_ratio", 0.0))
    if sample_inlier_ratio < se3_photometric_min_sample_inlier_ratio:
        errors.append(
            "visual_se3_photometric_last_accepted_sample_inlier_ratio "
            f"{sample_inlier_ratio} < {se3_photometric_min_sample_inlier_ratio}"
        )
    coverage_tiles = int(last.get("visual_se3_photometric_last_accepted_coverage_tiles", 0))
    if coverage_tiles < se3_photometric_min_coverage_tiles:
        errors.append(
            "visual_se3_photometric_last_accepted_coverage_tiles "
            f"{coverage_tiles} < {se3_photometric_min_coverage_tiles}"
        )
    mean_abs_residual = float(
        last.get("visual_se3_photometric_last_accepted_mean_abs_residual", 0.0))
    if (
        se3_photometric_max_mean_abs_residual_for_factor > 0.0
        and mean_abs_residual > se3_photometric_max_mean_abs_residual_for_factor
    ):
        errors.append(
            "visual_se3_photometric_last_accepted_mean_abs_residual "
            f"{mean_abs_residual} > {se3_photometric_max_mean_abs_residual_for_factor}"
        )
if (
    enable_se3_photometric_pose_correction
    and int(last.get("visual_se3_photometric_pose_corrections", 0)) <= 0
):
    errors.append("visual_se3_photometric_pose_corrections is zero")
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
if enable_visual_factor_time_interpolation:
    interpolated_visual = int(last.get("visual_alignment_interpolated_factors", 0) or 0)
    interpolated_se3 = int(last.get("visual_se3_photometric_interpolated_factors", 0) or 0)
    if interpolated_visual + interpolated_se3 <= 0:
        errors.append("visual factor time interpolation produced no interpolated factors")
if enable_visual_cache_reconciliation:
    reconciled_pairs = int(last.get("visual_cache_reconciled_pairs", 0) or 0)
    if reconciled_pairs <= 0:
        errors.append("visual cache reconciliation produced no reconciled pairs")
if require_deskew and int(last.get("trajectory_deskew_queries", 0)) <= 0:
    errors.append("trajectory_deskew_queries is zero")
if require_deskew and int(last.get("trajectory_deskew_hits", 0)) <= 0:
    errors.append("trajectory_deskew_hits is zero")

report = {
    "ok": not errors,
    "errors": errors,
    "visual_factor_continuity": visual_factor_continuity,
    "estimator_observability_continuity": estimator_observability_continuity,
    "mapper_feedback_continuity": mapper_feedback_continuity,
    "rendered_delivery_continuity": rendered_delivery_continuity,
    "step_guard_continuity": step_guard_continuity,
    "motion_target_continuity": motion_target_continuity,
    "gate_config": {
        "max_rendered_delivery_lag": max_rendered_delivery_lag,
        "min_rendered_delivery_received_ratio": min_rendered_delivery_received_ratio,
        "max_tracking_step_guard_clamp_ratio": max_tracking_step_guard_clamp_ratio,
        "rendered_image_qos_reliability": os.environ["RENDERED_IMAGE_QOS_RELIABILITY_REPORT"],
        "rendered_image_qos_durability": os.environ["RENDERED_IMAGE_QOS_DURABILITY_REPORT"],
        "rendered_image_qos_depth": int(os.environ["RENDERED_IMAGE_QOS_DEPTH_REPORT"]),
        "enable_rendered_feedback_contract": enable_rendered_feedback_contract,
        "rendered_feedback_topic": rendered_feedback_topic,
        "rendered_feedback_qos_reliability": rendered_feedback_qos_reliability,
        "rendered_feedback_qos_durability": rendered_feedback_qos_durability,
        "rendered_feedback_qos_depth": rendered_feedback_qos_depth,
        "enable_rendered_feedback_ingress_queue": enable_rendered_feedback_ingress_queue,
        "rendered_feedback_ingress_queue_size": rendered_feedback_ingress_queue_size,
        "rendered_feedback_ingress_drain_max_per_cycle": (
            rendered_feedback_ingress_drain_max_per_cycle
        ),
        "rendered_feedback_ingress_drain_period_ms": (
            rendered_feedback_ingress_drain_period_ms
        ),
        "mapper_feedback_publish_rendered_before_update": (
            mapper_feedback_publish_rendered_before_update
        ),
        "mapper_feedback_rendered_feedback_source_stream": (
            mapper_feedback_rendered_feedback_source_stream
        ),
        "mapper_feedback_rendered_feedback_image_topic": (
            mapper_feedback_rendered_feedback_image_topic
        ),
        "mapper_feedback_rendered_feedback_pose_topic": (
            mapper_feedback_rendered_feedback_pose_topic
        ),
        "mapper_feedback_rendered_feedback_image_qos_reliability": (
            mapper_feedback_rendered_feedback_image_qos_reliability
        ),
        "mapper_feedback_rendered_feedback_image_qos_history": (
            mapper_feedback_rendered_feedback_image_qos_history
        ),
        "mapper_feedback_rendered_feedback_image_qos_depth": (
            mapper_feedback_rendered_feedback_image_qos_depth
        ),
        "mapper_feedback_rendered_feedback_pose_qos_reliability": (
            mapper_feedback_rendered_feedback_pose_qos_reliability
        ),
        "mapper_feedback_rendered_feedback_pose_qos_history": (
            mapper_feedback_rendered_feedback_pose_qos_history
        ),
        "mapper_feedback_rendered_feedback_pose_qos_depth": (
            mapper_feedback_rendered_feedback_pose_qos_depth
        ),
        "visual_factor_max_dt_ns": visual_factor_max_dt_ns,
        "visual_depth_max_dt_ns": visual_depth_max_dt_ns,
        "visual_depth_dilation_px": visual_depth_dilation_px,
        "rendered_frame_cache_size": int(os.environ["RENDERED_FRAME_CACHE_SIZE_REPORT"]),
        "observed_frame_cache_size": int(os.environ["OBSERVED_FRAME_CACHE_SIZE_REPORT"]),
        "visual_alignment_max_shift_px": visual_alignment_max_shift_px,
        "visual_alignment_score_mode": visual_alignment_score_mode,
        "visual_alignment_factor_source": visual_alignment_factor_source,
        "visual_factor_source_id_mode": visual_factor_source_id_mode,
        "visual_factor_reference_stamp_mode": visual_factor_reference_stamp_mode,
        "enable_visual_factor_time_interpolation": enable_visual_factor_time_interpolation,
        "enable_visual_cache_reconciliation": enable_visual_cache_reconciliation,
        "visual_cache_reconciliation_monotonic_unique": (
            visual_cache_reconciliation_monotonic_unique
        ),
        "visual_pair_monotonic_unique": visual_pair_monotonic_unique,
        "enable_visual_watermark_pair_scheduler": enable_visual_watermark_pair_scheduler,
        "visual_watermark_pair_scheduler_max_pairs_per_pointcloud": (
            visual_watermark_pair_scheduler_max_pairs_per_pointcloud
        ),
        "enable_visual_callback_factor_ingest": enable_visual_callback_factor_ingest,
        "enable_rendered_feedback_watermark_queue": (
            enable_rendered_feedback_watermark_queue
        ),
        "defer_future_visual_factors_until_active": (
            defer_future_visual_factors_until_active
        ),
        "enable_visual_adaptive_state_retention": enable_visual_adaptive_state_retention,
        "visual_adaptive_state_retention_margin_states": (
            visual_adaptive_state_retention_margin_states
        ),
        "visual_adaptive_state_retention_max_states": (
            visual_adaptive_state_retention_max_states
        ),
        "enable_visual_expired_factor_projection": enable_visual_expired_factor_projection,
        "enable_visual_marginalization_prior": enable_visual_marginalization_prior,
        "enable_visual_marginalization_prior_batching": (
            enable_visual_marginalization_prior_batching
        ),
        "enable_visual_marginalization_prior_saturation_gate": (
            enable_visual_marginalization_prior_saturation_gate
        ),
        "visual_marginalization_prior_zero_bias_columns": (
            visual_marginalization_prior_zero_bias_columns
        ),
        "enable_visual_factor_reference_snapshot": (
            enable_visual_factor_reference_snapshot
        ),
        "enable_rendered_feedback_source_pose_reference": (
            enable_rendered_feedback_source_pose_reference
        ),
        "enable_rendered_feedback_source_motion_factor": (
            enable_rendered_feedback_source_motion_factor
        ),
        "enable_rendered_feedback_source_motion_marginalized_prior": (
            enable_rendered_feedback_source_motion_marginalized_prior
        ),
        "rendered_feedback_source_motion_translation_weight": (
            rendered_feedback_source_motion_translation_weight
        ),
        "rendered_feedback_source_motion_rotation_weight": (
            rendered_feedback_source_motion_rotation_weight
        ),
        "rendered_feedback_source_motion_huber_delta_m": (
            rendered_feedback_source_motion_huber_delta_m
        ),
        "rendered_feedback_source_motion_rotation_huber_delta_rad": (
            rendered_feedback_source_motion_rotation_huber_delta_rad
        ),
        "rendered_feedback_source_motion_min_dt_s": (
            rendered_feedback_source_motion_min_dt_s
        ),
        "rendered_feedback_source_motion_max_dt_s": (
            rendered_feedback_source_motion_max_dt_s
        ),
        "rendered_feedback_source_motion_in_from_frame": (
            rendered_feedback_source_motion_in_from_frame
        ),
        "visual_expired_factor_projection_max_age_s": (
            visual_expired_factor_projection_max_age_s
        ),
        "visual_cache_reconciliation_defer_to_pointcloud": (
            visual_cache_reconciliation_defer_to_pointcloud
        ),
        "visual_pair_processing_defer_to_pointcloud": (
            visual_pair_processing_defer_to_pointcloud
        ),
        "visual_alignment_window_weight": visual_alignment_window_weight,
        "visual_alignment_saturation_margin_px": visual_alignment_saturation_margin_px,
        "visual_alignment_saturated_weight_scale": visual_alignment_saturated_weight_scale,
        "enable_visual_factor_quality_weighting": enable_visual_factor_quality_weighting,
        "visual_factor_quality_min_weight_scale": visual_factor_quality_min_weight_scale,
        "se3_photometric_window_weight": se3_photometric_window_weight,
        "se3_photometric_max_samples": se3_photometric_max_samples,
        "mapper_feedback_sync_tolerance_sec": mapper_feedback_sync_tolerance_sec,
        "mapper_feedback_sync_anchor_stream": mapper_feedback_sync_anchor_stream,
        "mapper_feedback_image_qos_reliability": mapper_feedback_image_qos_reliability,
        "mapper_feedback_image_qos_depth": mapper_feedback_image_qos_depth,
        "visual_pending_factor_queue_size": visual_pending_factor_queue_size,
        "enable_visual_factor_quality_selection": enable_visual_factor_quality_selection,
        "enable_visual_factor_quality_reference_cap": (
            enable_visual_factor_quality_reference_cap
        ),
        "visual_factor_quality_selection_max_per_reference": (
            visual_factor_quality_selection_max_per_reference
        ),
        "visual_factor_quality_selection_start_after_s": (
            visual_factor_quality_selection_start_after_s
        ),
        "se3_photometric_min_samples": se3_photometric_min_samples,
        "se3_photometric_min_hessian_rank": se3_photometric_min_hessian_rank,
        "se3_photometric_max_hessian_condition": se3_photometric_max_hessian_condition,
        "se3_photometric_min_sample_inlier_ratio": se3_photometric_min_sample_inlier_ratio,
        "se3_photometric_max_mean_abs_residual_for_factor": (
            se3_photometric_max_mean_abs_residual_for_factor
        ),
        "se3_photometric_coverage_grid_cols": se3_photometric_coverage_grid_cols,
        "se3_photometric_coverage_grid_rows": se3_photometric_coverage_grid_rows,
        "se3_photometric_min_coverage_tiles": se3_photometric_min_coverage_tiles,
        "se3_photometric_rank_samples_by_gradient": se3_photometric_rank_samples_by_gradient,
        "se3_photometric_use_rendered_gradient": se3_photometric_use_rendered_gradient,
        "enable_se3_photometric_pose_correction": enable_se3_photometric_pose_correction,
        "se3_photometric_pose_correction_gain": se3_photometric_pose_correction_gain,
        "se3_photometric_pose_correction_max_translation_m": (
            se3_photometric_pose_correction_max_translation_m
        ),
        "se3_photometric_pose_correction_max_rotation_rad": (
            se3_photometric_pose_correction_max_rotation_rad
        ),
        "se3_photometric_pose_correction_max_dt_ns": se3_photometric_pose_correction_max_dt_ns,
        "reference_tum_path": str(reference_tum_path) if reference_tum_path else "",
        "external_reference_tum_poses": external_reference_pose_count,
        "reference_trajectory_align": reference_trajectory_align,
        "reference_max_association_dt": reference_max_association_dt,
        "reference_min_coverage": reference_min_coverage,
        "reference_coverage_mode": reference_coverage_mode,
        "reference_max_rmse_m": reference_max_rmse_m,
        "reference_max_mean_m": reference_max_mean_m,
        "reference_max_error_m": reference_max_error_m,
        "reference_max_path_drift": reference_max_path_drift,
        "reference_error_bin_count": reference_error_bin_count,
        "reference_min_error_bin_matches": reference_min_error_bin_matches,
        "reference_max_error_bin_rmse_m": reference_max_error_bin_rmse_m,
        "reference_max_error_bin_mean_m": reference_max_error_bin_mean_m,
        "reference_max_error_bin_bias_norm_m": reference_max_error_bin_bias_norm_m,
        "reference_time_offset_sweep": {
            "min": reference_time_offset_sweep_min,
            "max": reference_time_offset_sweep_max,
            "step": reference_time_offset_sweep_step,
        },
        "playback_duration_sec": playback_duration_sec,
        "playback_rate": playback_rate,
        "playback_start_offset": playback_start_offset,
        "playback_clock_topics_all": playback_clock_topics_all,
        "timeout_sec": timeout_sec,
        "post_play_settle_sec": post_play_settle_sec,
        "post_play_drain_target_poses": post_play_drain_target_poses,
        "post_play_drain_target_visual_factors": post_play_drain_target_visual_factors,
        "post_play_drain_target_se3_factors": post_play_drain_target_se3_factors,
        "post_play_drain_timeout_sec": post_play_drain_timeout_sec,
        "imu_linear_acceleration_scale": imu_linear_acceleration_scale,
        "max_lidar_invalid_frames": max_lidar_invalid_frames,
        "min_status_bin_sample_count": min_status_bin_sample_count,
        "min_visual_factor_delta_per_status_bin": min_visual_factor_delta_per_status_bin,
        "min_se3_photometric_factor_delta_per_status_bin": (
            min_se3_photometric_factor_delta_per_status_bin
        ),
        "min_motion_target_delta_per_status_bin": min_motion_target_delta_per_status_bin,
        "lidar_pose_factor_iterations": lidar_pose_factor_iterations,
        "lidar_window_point_factor_weight": lidar_window_point_factor_weight,
        "lidar_window_plane_factor_weight": lidar_window_plane_factor_weight,
        "lidar_window_confidence_power": lidar_window_confidence_power,
        "sliding_window_max_states": sliding_window_max_states,
        "sliding_window_max_iterations": sliding_window_max_iterations,
        "sliding_window_marginalization_prior_weight": (
            sliding_window_marginalization_prior_weight
        ),
        "sliding_window_sync_guarded_pose_state": sliding_window_sync_guarded_pose_state,
        "sliding_window_guarded_pose_prior_translation_weight": (
            sliding_window_guarded_pose_prior_translation_weight
        ),
        "sliding_window_guarded_pose_prior_rotation_weight": (
            sliding_window_guarded_pose_prior_rotation_weight
        ),
        "enable_gaussian_map_feedback": enable_gaussian_map_feedback,
        "require_gaussian_snapshot": require_gaussian_snapshot,
        "gaussian_snapshot_lidar_factor_weight": gaussian_snapshot_lidar_factor_weight,
        "gaussian_snapshot_lidar_nearest_distance_m": (
            gaussian_snapshot_lidar_nearest_distance_m
        ),
        "gaussian_snapshot_lidar_residual_preweight": (
            gaussian_snapshot_lidar_residual_preweight
        ),
        "gaussian_snapshot_lidar_pose_correction_enabled": (
            gaussian_snapshot_lidar_pose_correction_enabled
        ),
        "gaussian_snapshot_lidar_pose_correction_gain": (
            gaussian_snapshot_lidar_pose_correction_gain
        ),
        "gaussian_snapshot_lidar_pose_correction_max_translation_m": (
            gaussian_snapshot_lidar_pose_correction_max_translation_m
        ),
        "gaussian_snapshot_lidar_pose_correction_max_rotation_rad": (
            gaussian_snapshot_lidar_pose_correction_max_rotation_rad
        ),
        "gaussian_snapshot_lidar_pose_correction_min_match_ratio": (
            gaussian_snapshot_lidar_pose_correction_min_match_ratio
        ),
        "gaussian_snapshot_lidar_pose_correction_max_mean_residual_m": (
            gaussian_snapshot_lidar_pose_correction_max_mean_residual_m
        ),
        "gaussian_snapshot_lidar_pose_correction_coverage_grid_cols": (
            gaussian_snapshot_lidar_pose_correction_coverage_grid_cols
        ),
        "gaussian_snapshot_lidar_pose_correction_coverage_grid_rows": (
            gaussian_snapshot_lidar_pose_correction_coverage_grid_rows
        ),
        "gaussian_snapshot_lidar_pose_correction_min_coverage_tiles": (
            gaussian_snapshot_lidar_pose_correction_min_coverage_tiles
        ),
        "gaussian_snapshot_lidar_pose_correction_bidirectional_max_distance_m": (
            gaussian_snapshot_lidar_pose_correction_bidirectional_max_distance_m
        ),
        "gaussian_snapshot_lidar_plane_factor_enabled": (
            gaussian_snapshot_lidar_plane_factor_enabled
        ),
        "gaussian_snapshot_lidar_plane_factor_weight": (
            gaussian_snapshot_lidar_plane_factor_weight
        ),
        "gaussian_snapshot_lidar_plane_min_anisotropy": (
            gaussian_snapshot_lidar_plane_min_anisotropy
        ),
        "mapper_feedback_torch_device": mapper_feedback_torch_device,
        "mapper_feedback_render_mode": mapper_feedback_render_mode,
        "mapper_feedback_pointcloud_coordinates": mapper_feedback_pointcloud_coordinates,
        "mapper_feedback_max_depth": mapper_feedback_max_depth,
        "mapper_feedback_require_projected_point_color": (
            mapper_feedback_require_projected_point_color
        ),
        "mapper_feedback_zbuffer_projected_points": mapper_feedback_zbuffer_projected_points,
        "mapper_feedback_gaussian_lr": {
            "position": mapper_feedback_lr[0],
            "feature": mapper_feedback_lr[1],
            "opacity": mapper_feedback_lr[2],
            "scaling": mapper_feedback_lr[3],
            "rotation": mapper_feedback_lr[4],
        },
        "mapper_feedback_torch_optimization_enabled": (
            mapper_feedback_torch_optimization_enabled
        ),
        "mapper_feedback_torch_optimization_steps": mapper_feedback_torch_optimization_steps,
        "mapper_feedback_torch_optimization_every_n_keyframes": (
            mapper_feedback_torch_optimization_every_n_keyframes
        ),
        "mapper_feedback_torch_optimization_sampling": (
            mapper_feedback_torch_optimization_sampling
        ),
        "mapper_feedback_select_every_k_frame": mapper_feedback_select_every_k_frame,
        "mapper_feedback_extend_visibility_filter": mapper_feedback_extend_visibility_filter,
        "mapper_feedback_pruning": mapper_feedback_pruning,
        "mapper_feedback_torch_max_foreground": mapper_feedback_torch_max_foreground,
        "mapper_feedback_torch_prune_count_policy": mapper_feedback_torch_prune_count_policy,
        "mapper_feedback_gaussian_map_publish_min_interval_sec": (
            mapper_feedback_gaussian_map_publish_min_interval_sec
        ),
        "mapper_feedback_gaussian_map_publish_on_empty_extend": (
            mapper_feedback_gaussian_map_publish_on_empty_extend
        ),
        "sliding_window_optimize_every_n_frames": sliding_window_optimize_every_n_frames,
        "sliding_window_max_feedback_translation_m": sliding_window_max_feedback_translation_m,
        "sliding_window_max_feedback_rotation_rad": sliding_window_max_feedback_rotation_rad,
        "sliding_window_max_feedback_velocity_mps": sliding_window_max_feedback_velocity_mps,
        "sliding_window_max_feedback_gyro_bias_step": (
            sliding_window_max_feedback_gyro_bias_step
        ),
        "sliding_window_max_feedback_accel_bias_step": (
            sliding_window_max_feedback_accel_bias_step
        ),
        "sliding_window_min_bias_feedback_visual_factors": (
            sliding_window_min_bias_feedback_visual_factors
        ),
        "sliding_window_bias_feedback_ownership": (
            sliding_window_bias_feedback_ownership
        ),
        "sliding_window_smoothness_position_velocity_weight": (
            sliding_window_smoothness_position_velocity_weight
        ),
        "sliding_window_smoothness_use_motion_targets": (
            sliding_window_smoothness_use_motion_targets
        ),
        "sliding_window_smoothness_motion_target_min_visual_factors": (
            sliding_window_smoothness_motion_target_min_visual_factors
        ),
        "sliding_window_smoothness_motion_target_min_se3_photometric_factors": (
            sliding_window_smoothness_motion_target_min_se3_photometric_factors
        ),
        "sliding_window_smoothness_motion_target_recent_window": (
            sliding_window_smoothness_motion_target_recent_window
        ),
        "sliding_window_smoothness_motion_target_min_recent_visual_factors": (
            sliding_window_smoothness_motion_target_min_recent_visual_factors
        ),
        "sliding_window_smoothness_motion_target_min_recent_se3_photometric_factors": (
            sliding_window_smoothness_motion_target_min_recent_se3_photometric_factors
        ),
        "sliding_window_smoothness_motion_target_start_after_s": (
            sliding_window_smoothness_motion_target_start_after_s
        ),
        "sliding_window_smoothness_motion_target_max_rotation_rate_delta_radps": (
            sliding_window_smoothness_motion_target_max_rotation_rate_delta_radps
        ),
        "sliding_window_smoothness_motion_target_max_position_rate_delta_mps": (
            sliding_window_smoothness_motion_target_max_position_rate_delta_mps
        ),
        "sliding_window_smoothness_motion_target_max_velocity_acceleration_delta_mps2": (
            sliding_window_smoothness_motion_target_max_velocity_acceleration_delta_mps2
        ),
        "sliding_window_imu_velocity_prior_weight": sliding_window_imu_velocity_prior_weight,
        "sliding_window_gyro_bias_prior_weight": sliding_window_gyro_bias_prior_weight,
        "sliding_window_accel_bias_prior_weight": sliding_window_accel_bias_prior_weight,
        "sliding_window_bias_weight": sliding_window_bias_weight,
        "sliding_window_gyro_bias_weight": sliding_window_gyro_bias_weight,
        "sliding_window_accel_bias_weight": sliding_window_accel_bias_weight,
        "sliding_window_bias_random_walk_reference_dt_s": (
            sliding_window_bias_random_walk_reference_dt_s
        ),
        "sliding_window_gyro_bias_random_walk_sigma": (
            sliding_window_gyro_bias_random_walk_sigma
        ),
        "sliding_window_accel_bias_random_walk_sigma": (
            sliding_window_accel_bias_random_walk_sigma
        ),
        "enable_sliding_window_relative_translation_factor": (
            enable_sliding_window_relative_translation_factor
        ),
        "sliding_window_relative_translation_weight": (
            sliding_window_relative_translation_weight
        ),
        "sliding_window_relative_translation_huber_delta_m": (
            sliding_window_relative_translation_huber_delta_m
        ),
        "sliding_window_relative_translation_in_from_frame": (
            sliding_window_relative_translation_in_from_frame
        ),
        "sliding_window_relative_rotation_weight": (
            sliding_window_relative_rotation_weight
        ),
        "sliding_window_relative_rotation_huber_delta_rad": (
            sliding_window_relative_rotation_huber_delta_rad
        ),
        "enable_sliding_window_relative_distance_factor": (
            enable_sliding_window_relative_distance_factor
        ),
        "sliding_window_relative_distance_weight": (
            sliding_window_relative_distance_weight
        ),
        "sliding_window_relative_distance_huber_delta_m": (
            sliding_window_relative_distance_huber_delta_m
        ),
        "enable_sliding_window_multihop_relative_translation_factor": (
            enable_sliding_window_multihop_relative_translation_factor
        ),
        "sliding_window_multihop_relative_translation_weight": (
            sliding_window_multihop_relative_translation_weight
        ),
        "sliding_window_multihop_relative_translation_huber_delta_m": (
            sliding_window_multihop_relative_translation_huber_delta_m
        ),
        "sliding_window_multihop_relative_translation_in_from_frame": (
            sliding_window_multihop_relative_translation_in_from_frame
        ),
        "sliding_window_multihop_relative_rotation_weight": (
            sliding_window_multihop_relative_rotation_weight
        ),
        "sliding_window_multihop_relative_rotation_huber_delta_rad": (
            sliding_window_multihop_relative_rotation_huber_delta_rad
        ),
        "enable_sliding_window_multihop_relative_distance_factor": (
            enable_sliding_window_multihop_relative_distance_factor
        ),
        "sliding_window_multihop_relative_distance_weight": (
            sliding_window_multihop_relative_distance_weight
        ),
        "sliding_window_multihop_relative_distance_huber_delta_m": (
            sliding_window_multihop_relative_distance_huber_delta_m
        ),
        "sliding_window_multihop_relative_translation_min_dt_s": (
            sliding_window_multihop_relative_translation_min_dt_s
        ),
        "sliding_window_multihop_relative_translation_max_dt_s": (
            sliding_window_multihop_relative_translation_max_dt_s
        ),
        "sliding_window_multihop_relative_translation_max_factors": (
            sliding_window_multihop_relative_translation_max_factors
        ),
        "enable_sliding_window_delayed_published_multihop_relative_translation_factor": (
            enable_sliding_window_delayed_published_multihop_relative_translation_factor
        ),
        "sliding_window_delayed_published_multihop_start_after_s": (
            sliding_window_delayed_published_multihop_start_after_s
        ),
        "sliding_window_delayed_published_multihop_max_factors": (
            sliding_window_delayed_published_multihop_max_factors
        ),
        "sliding_window_relative_motion_history_source": (
            sliding_window_relative_motion_history_source
        ),
        "sliding_window_relative_motion_history_published_after_s": (
            sliding_window_relative_motion_history_published_after_s
        ),
        "tracking_max_pose_step_m": tracking_max_pose_step_m,
        "enable_pre_lio_tracking_step_guard": enable_pre_lio_tracking_step_guard,
        "enable_post_ba_tracking_step_guard": enable_post_ba_tracking_step_guard,
        "pre_lio_tracking_max_pose_step_m": pre_lio_tracking_max_pose_step_m,
        "post_ba_tracking_max_pose_step_m": post_ba_tracking_max_pose_step_m,
        "post_ba_step_guard_confidence_max_pose_step_m": (
            post_ba_step_guard_confidence_max_pose_step_m
        ),
        "post_ba_step_guard_confidence_warmup_marginalizations": (
            post_ba_step_guard_confidence_warmup_marginalizations
        ),
        "post_ba_step_guard_min_lidar_confidence": (
            post_ba_step_guard_min_lidar_confidence
        ),
        "post_ba_step_guard_min_visual_inlier_ratio": (
            post_ba_step_guard_min_visual_inlier_ratio
        ),
        "post_ba_step_guard_max_visual_residual": (
            post_ba_step_guard_max_visual_residual
        ),
        "post_ba_step_guard_min_visual_coverage_tiles": (
            post_ba_step_guard_min_visual_coverage_tiles
        ),
        "post_ba_step_guard_reject_to_pre_ba_over_m": (
            post_ba_step_guard_reject_to_pre_ba_over_m
        ),
        "post_ba_step_guard_pre_ba_agreement_max_pose_step_m": (
            post_ba_step_guard_pre_ba_agreement_max_pose_step_m
        ),
        "post_ba_step_guard_pre_ba_agreement_late_start_marginalizations": (
            post_ba_step_guard_pre_ba_agreement_late_start_marginalizations
        ),
        "post_ba_step_guard_pre_ba_agreement_late_max_pose_step_m": (
            post_ba_step_guard_pre_ba_agreement_late_max_pose_step_m
        ),
        "post_ba_step_guard_pre_ba_agreement_min_cosine": (
            post_ba_step_guard_pre_ba_agreement_min_cosine
        ),
        "post_ba_step_guard_pre_ba_agreement_max_delta_m": (
            post_ba_step_guard_pre_ba_agreement_max_delta_m
        ),
        "post_ba_step_guard_pre_ba_agreement_margin_m": (
            post_ba_step_guard_pre_ba_agreement_margin_m
        ),
        "post_ba_step_guard_pre_ba_blend_on_clamp": (
            post_ba_step_guard_pre_ba_blend_on_clamp
        ),
        "tracking_step_guard_velocity_scale": tracking_step_guard_velocity_scale,
        "pre_lio_tracking_step_guard_velocity_scale": (
            pre_lio_tracking_step_guard_velocity_scale
        ),
        "post_ba_tracking_step_guard_velocity_scale": (
            post_ba_tracking_step_guard_velocity_scale
        ),
        "tracking_step_guard_acceleration_mps2": tracking_step_guard_acceleration_mps2,
        "tracking_step_guard_max_velocity_mps": tracking_step_guard_max_velocity_mps,
        "tracking_step_guard_margin_m": tracking_step_guard_margin_m,
    },
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
report_status=$?
set -e

COMPARE_JSON="${OUTPUT_DIR}/native_tracking_trajectory_compare.json"
REFERENCE_TUM="${ARTIFACT_DIR}/reference_trajectory.tum"
if [[ -n "${REFERENCE_TUM_PATH}" ]]; then
  REFERENCE_TUM="${REFERENCE_TUM_PATH}"
fi
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
    --coverage-mode "${REFERENCE_COVERAGE_MODE}"
    --max-rmse-m "${REFERENCE_MAX_RMSE_M}"
    --max-mean-m "${REFERENCE_MAX_MEAN_M}"
    --max-error-m "${REFERENCE_MAX_ERROR_M}"
    --max-path-drift "${REFERENCE_MAX_PATH_DRIFT}"
  )
  if [[ "${REFERENCE_ERROR_BIN_COUNT}" != "0" ]]; then
    compare_cmd+=(--error-bin-count "${REFERENCE_ERROR_BIN_COUNT}")
  fi
  if [[ "${REFERENCE_MIN_ERROR_BIN_MATCHES}" != "0" ]]; then
    compare_cmd+=(--min-error-bin-matches "${REFERENCE_MIN_ERROR_BIN_MATCHES}")
  fi
  if [[ "${REFERENCE_MAX_ERROR_BIN_RMSE_M}" != "0" && "${REFERENCE_MAX_ERROR_BIN_RMSE_M}" != "0.0" ]]; then
    compare_cmd+=(--max-error-bin-rmse-m "${REFERENCE_MAX_ERROR_BIN_RMSE_M}")
  fi
  if [[ "${REFERENCE_MAX_ERROR_BIN_MEAN_M}" != "0" && "${REFERENCE_MAX_ERROR_BIN_MEAN_M}" != "0.0" ]]; then
    compare_cmd+=(--max-error-bin-mean-m "${REFERENCE_MAX_ERROR_BIN_MEAN_M}")
  fi
  if [[ "${REFERENCE_MAX_ERROR_BIN_BIAS_NORM_M}" != "0" && "${REFERENCE_MAX_ERROR_BIN_BIAS_NORM_M}" != "0.0" ]]; then
    compare_cmd+=(--max-error-bin-bias-norm-m "${REFERENCE_MAX_ERROR_BIN_BIAS_NORM_M}")
  fi
  if [[ "${REFERENCE_MIN_CURRENT_PATH_RATIO}" != "0" && "${REFERENCE_MIN_CURRENT_PATH_RATIO}" != "0.0" ]]; then
    compare_cmd+=(--min-current-path-ratio "${REFERENCE_MIN_CURRENT_PATH_RATIO}")
  fi
  if [[ "${REFERENCE_MAX_CURRENT_PATH_RATIO}" != "0" && "${REFERENCE_MAX_CURRENT_PATH_RATIO}" != "0.0" ]]; then
    compare_cmd+=(--max-current-path-ratio "${REFERENCE_MAX_CURRENT_PATH_RATIO}")
  fi
  if [[ "${REFERENCE_TIME_OFFSET_SWEEP_STEP}" != "0" && "${REFERENCE_TIME_OFFSET_SWEEP_STEP}" != "0.0" ]]; then
    compare_cmd+=(
      --time-offset-sweep-min "${REFERENCE_TIME_OFFSET_SWEEP_MIN}"
      --time-offset-sweep-max "${REFERENCE_TIME_OFFSET_SWEEP_MAX}"
      --time-offset-sweep-step "${REFERENCE_TIME_OFFSET_SWEEP_STEP}"
    )
  fi
  set +e
  "${compare_cmd[@]}"
  compare_status=$?
  set -e

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
  if [[ "${compare_status}" -ne 0 ]]; then
    if [[ "${REQUIRE_REFERENCE_TRAJECTORY}" == "true" ]]; then
      echo "[native-tracking] reference trajectory comparison failed; report kept at ${REPORT_JSON}" >&2
      exit "${compare_status}"
    fi
    echo "[native-tracking] reference trajectory comparison did not meet thresholds; report kept non-gating at ${COMPARE_JSON}" >&2
  fi
elif [[ "${REQUIRE_REFERENCE_TRAJECTORY}" == "true" ]]; then
  echo "required reference/current trajectory file is empty or missing" >&2
  exit 1
fi

if [[ "${WRITE_STATUS_HISTORY}" == "true" && -s "${REFERENCE_TUM}" ]]; then
  "${ROOT_DIR}/scripts/diagnose_native_tracking_drift.py" \
    --run-dir "${OUTPUT_DIR}" \
    --reference-tum "${REFERENCE_TUM}" \
    --output-dir "${OUTPUT_DIR}/diagnostics"
fi

if [[ "${report_status}" -ne 0 ]]; then
  echo "[native-tracking] health report failed; report kept at ${REPORT_JSON}" >&2
  exit "${report_status}"
fi

echo "[native-tracking] report: ${REPORT_JSON}"
