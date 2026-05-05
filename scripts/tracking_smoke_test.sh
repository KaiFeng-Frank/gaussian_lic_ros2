#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
BAG_PATH="${ROOT_DIR}/bags/synthetic_frontend_raw_visual_demo"
TIMEOUT_SEC=20
ENABLE_SLIDING_WINDOW=true
ENABLE_LIDAR_PLANE_FACTOR=true
ENABLE_VISUAL_FACTOR_GATE=true
SLIDING_WINDOW_IMU_WEIGHT=1.0
SLIDING_WINDOW_MAX_NORMAL_EQUATION_CONDITION=1000000000000.0
EXPECT_IMU_FACTOR=true

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
  --no-visual-factor-gate           Disable rendered-image visual runtime checks.
  --no-imu-factor                   Set IMU factor weight to zero and require
                                    non-IMU window factors to trigger BA.
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
    --no-visual-factor-gate)
      ENABLE_VISUAL_FACTOR_GATE=false
      shift
      ;;
    --no-imu-factor)
      SLIDING_WINDOW_IMU_WEIGHT=0.0
      SLIDING_WINDOW_MAX_NORMAL_EQUATION_CONDITION=1000000000000000.0
      EXPECT_IMU_FACTOR=false
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
    --duration 4 \
    --image-width 32 \
    --image-height 32 \
    --image-pattern gaussian \
    --image-shift-x 1.25 \
    --image-shift-y -0.35
fi

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source install/setup.bash
set -u

launch_log="${ROOT_DIR}/log/tracking_smoke_launch.log"
play_log="${ROOT_DIR}/log/tracking_smoke_play.log"
render_log="${ROOT_DIR}/log/tracking_smoke_rendered_pub.log"
mkdir -p "${ROOT_DIR}/log"
rm -f "${launch_log}" "${play_log}" "${render_log}"

cleanup() {
  if [[ -n "${play_pid:-}" ]]; then
    kill "${play_pid}" 2>/dev/null || true
    wait "${play_pid}" 2>/dev/null || true
  fi
  if [[ -n "${render_pid:-}" ]]; then
    kill -- -"${render_pid}" 2>/dev/null || true
    wait "${render_pid}" 2>/dev/null || true
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
  enable_visual_alignment_window_factor:="${ENABLE_VISUAL_FACTOR_GATE}" \
  enable_se3_photometric_window_factor:="${ENABLE_VISUAL_FACTOR_GATE}" \
  sliding_window_imu_weight:="${SLIDING_WINDOW_IMU_WEIGHT}" \
  sliding_window_max_normal_equation_condition:="${SLIDING_WINDOW_MAX_NORMAL_EQUATION_CONDITION}" \
  lidar_min_points:=1 \
  lidar_nearest_distance_m:=2.0 \
  lidar_keyframe_translation_m:=0.0 \
  >"${launch_log}" 2>&1 &
launch_pid=$!

sleep 2
if [[ "${ENABLE_VISUAL_FACTOR_GATE}" == "true" ]]; then
  setsid ros2 run gaussian_lic_tools synthetic_gs_frame_pub \
    --ros-args \
    -p pointcloud_topic:=/__gaussian_lic_tracking_smoke_unused/points \
    -p pose_topic:=/__gaussian_lic_tracking_smoke_unused/pose \
    -p odometry_topic:=/__gaussian_lic_tracking_smoke_unused/odometry \
    -p image_topic:=/__gaussian_lic_tracking_smoke_unused/image \
    -p camera_info_topic:=/__gaussian_lic_tracking_smoke_unused/camera_info \
    -p depth_topic:=/__gaussian_lic_tracking_smoke_unused/depth \
    -p imu_topic:=/__gaussian_lic_tracking_smoke_unused/imu \
    -p pose_output_mode:=none \
    -p publish_depth:=false \
    -p publish_rendered_image:=true \
    -p rendered_image_topic:=/gaussian_lic/rendered_image \
    -p use_sim_time:=true \
    -p image_width:=32 \
    -p image_height:=32 \
    -p image_pattern:=gaussian \
    -p rendered_shift_x_px:=0.0 \
    -p rendered_shift_y_px:=0.0 \
    >"${render_log}" 2>&1 &
  render_pid=$!
  sleep 1
fi

ros2 bag play "${BAG_PATH}" --clock --rate 1.0 >"${play_log}" 2>&1 &
play_pid=$!

for topic in /pose_for_gs /points_for_gs /gaussian_lic/frontend/odometry /gaussian_lic/frontend/path; do
  timeout "${TIMEOUT_SEC}" ros2 topic echo --once "${topic}" >/dev/null
done

status_file=/tmp/gaussian_lic_tracking_smoke_status.txt
status_tmp=/tmp/gaussian_lic_tracking_smoke_status.tmp
rm -f "${status_file}" "${status_tmp}"

status_matches() {
  rg -q "executor_callback_serialization_enabled: true" "${status_file}" &&
    rg -q "sensor_qos_reliability: best_effort" "${status_file}" &&
    rg -q "sensor_qos_history: keep_last" "${status_file}" &&
    rg -q "sensor_qos_depth: 5" "${status_file}" &&
    rg -q "signed_nanosecond_time_math_enabled: true" "${status_file}" &&
    rg -q "last_image_stamp_ns: [1-9]" "${status_file}" &&
    rg -q "last_pointcloud_stamp_ns: [1-9]" "${status_file}" &&
    rg -q "last_imu_stamp_ns: [1-9]" "${status_file}" &&
    rg -q "lidar_invalid_points: 0" "${status_file}" &&
    rg -q "lidar_invalid_point_times: 0" "${status_file}" &&
    rg -q "lidar_out_of_range_point_times: 0" "${status_file}" &&
    rg -q "last_lidar_max_abs_point_time_offset_s:" "${status_file}" &&
    rg -q "image_stamp_regressions: 0" "${status_file}" &&
    rg -q "depth_stamp_regressions: 0" "${status_file}" &&
    rg -q "rendered_stamp_regressions: 0" "${status_file}" &&
    rg -q "pointcloud_stamp_regressions: 0" "${status_file}" &&
    rg -q "imu_stamp_regressions: 0" "${status_file}" &&
    rg -q "imu_invalid_measurements: 0" "${status_file}" &&
    rg -q "camera_info_invalid_intrinsics: 0" "${status_file}" &&
    rg -q "image_invalid_frames: 0" "${status_file}" &&
    rg -q "depth_invalid_frames: 0" "${status_file}" &&
    rg -q "rendered_invalid_frames: 0" "${status_file}" &&
    rg -q "sliding_window_enabled: true" "${status_file}" &&
    rg -q "sliding_window_rejected_steps:" "${status_file}" &&
    rg -q "sliding_window_limited_steps:" "${status_file}" &&
    rg -q "sliding_window_point_factor_skip_count:" "${status_file}" &&
    rg -q "sliding_window_plane_factor_skip_count:" "${status_file}" &&
    rg -q "sliding_window_visual_factor_skip_count:" "${status_file}" &&
    rg -q "sliding_window_se3_photometric_factor_skip_count:" "${status_file}" &&
    rg -q "sliding_window_smoothness_factor_skip_count:" "${status_file}" &&
    rg -q "sliding_window_imu_factor_skip_count:" "${status_file}" &&
    rg -q "sliding_window_optimization_skip_count:" "${status_file}" &&
    rg -q "sliding_window_normal_equation_rows: [1-9]" "${status_file}" &&
    rg -q "sliding_window_normal_equation_cols: [1-9]" "${status_file}" &&
    rg -q "sliding_window_normal_equation_rank: [1-9]" "${status_file}" &&
    rg -q "sliding_window_normal_equation_max_singular_value: .*[1-9]" "${status_file}" &&
    rg -q "sliding_window_normal_equation_condition_number: .*[1-9]" "${status_file}" &&
    rg -q "sliding_window_point_factors: [1-9]" "${status_file}" &&
    rg -q "sliding_window_smoothness_factors: [1-9]" "${status_file}" &&
    rg -q "sliding_window_dense_prior_rank: [1-9]" "${status_file}" &&
    rg -q "sliding_window_dense_prior_max_singular_value: .*[1-9]" "${status_file}" &&
    rg -q "trajectory_control_poses: [1-9]" "${status_file}" &&
    rg -q "total_window_point_correspondences: [1-9]" "${status_file}" &&
    rg -q "num_lidar_keyframes: [1-9]" "${status_file}" || return 1
  if [[ "${EXPECT_IMU_FACTOR}" == "true" ]]; then
    rg -q "state: 2" "${status_file}" &&
      rg -q "sliding_window_accepted_steps: [1-9]" "${status_file}" &&
      rg -q "sliding_window_normal_equation_degenerate: false" "${status_file}" &&
      rg -q "sliding_window_last_step_scale: .*[1-9]" "${status_file}" &&
      rg -q "sliding_window_last_damping: .*[1-9]" "${status_file}" &&
      rg -q "sliding_window_imu_factors: [1-9]" "${status_file}" &&
      rg -q "sliding_window_gyro_bias_observability: [1-9]" "${status_file}" &&
      rg -q "sliding_window_accel_bias_observability: [1-9]" "${status_file}" &&
      rg -q "sliding_window_imu_reanchors: [1-9]" "${status_file}" || return 1
  else
    rg -q "state: [23]" "${status_file}" &&
      rg -q "sliding_window_imu_factors: 0" "${status_file}" &&
      rg -q "sliding_window_imu_factor_skip_count: [1-9]" "${status_file}" || return 1
    if ! rg -q "sliding_window_(accepted|rejected)_steps: [1-9]" "${status_file}"; then
      return 1
    fi
  fi
  if [[ "${ENABLE_VISUAL_FACTOR_GATE}" == "true" ]]; then
    rg -q "visual_factor_enabled: true" "${status_file}" &&
      rg -q "visual_alignment_valid: true" "${status_file}" &&
      rg -q "visual_photometric_valid: true" "${status_file}" &&
      rg -q "visual_photometric_pixels: [1-9]" "${status_file}" &&
      rg -q "visual_rendered_cache_size: [1-9]" "${status_file}" &&
      rg -q "visual_depth_cache_size: [1-9]" "${status_file}" &&
      rg -q "visual_se3_photometric_valid: true" "${status_file}" &&
      rg -q "visual_se3_photometric_candidates: [1-9]" "${status_file}" &&
      rg -q "visual_se3_photometric_samples: [1-9]" "${status_file}" &&
      rg -q "visual_se3_photometric_rejected_depth:" "${status_file}" &&
      rg -q "visual_se3_photometric_rejected_gradient:" "${status_file}" &&
      rg -q "visual_se3_photometric_rejected_residual:" "${status_file}" &&
      rg -q "visual_se3_photometric_inlier_ratio: .*[1-9]" "${status_file}" &&
      rg -q "visual_se3_photometric_step_norm: .*[1-9]" "${status_file}" &&
      rg -q "sliding_window_visual_factors: [1-9]" "${status_file}" &&
      rg -q "sliding_window_se3_photometric_factors: [1-9]" "${status_file}" || return 1
  fi
}

deadline=$((SECONDS + TIMEOUT_SEC))
while (( SECONDS < deadline )); do
  if timeout 3 ros2 topic echo --once \
    /gaussian_lic/frontend/status gaussian_lic_msgs/msg/TrackingStatus \
    >"${status_tmp}" 2>/dev/null
  then
    mv "${status_tmp}" "${status_file}"
    if status_matches; then
      break
    fi
  fi
  sleep 0.5
done

if [[ ! -f "${status_file}" ]] || ! status_matches; then
  echo "tracking status did not satisfy smoke gates" >&2
  [[ -f "${status_file}" ]] && cat "${status_file}" >&2
  exit 1
fi

echo "[tracking-smoke] passed"
echo "[tracking-smoke] launch log: ${launch_log}"
