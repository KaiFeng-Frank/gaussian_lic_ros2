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
DIAGNOSTIC_LOG_PERIOD_STEPS="${DIAGNOSTIC_LOG_PERIOD_STEPS:-50}"
PRIOR_PUBLISH_DURATION="${PRIOR_PUBLISH_DURATION:-6}"
PRIOR_PUBLISH_RATE_HZ="${PRIOR_PUBLISH_RATE_HZ:-20}"
PRIOR_MAX_MESSAGES="${PRIOR_MAX_MESSAGES:-60}"
ENABLE_EXTERNAL_ODOMETRY_POSITION_FACTORS="${ENABLE_EXTERNAL_ODOMETRY_POSITION_FACTORS:-false}"
ENABLE_EXTERNAL_ODOMETRY_ORIENTATION_FACTORS="${ENABLE_EXTERNAL_ODOMETRY_ORIENTATION_FACTORS:-false}"
EXTERNAL_ODOMETRY_POSITION_FACTOR_WEIGHT="${EXTERNAL_ODOMETRY_POSITION_FACTOR_WEIGHT:-1.0}"
EXTERNAL_ODOMETRY_POSITION_FACTOR_HUBER_DELTA_M="${EXTERNAL_ODOMETRY_POSITION_FACTOR_HUBER_DELTA_M:-0.25}"
EXTERNAL_ODOMETRY_ORIENTATION_FACTOR_WEIGHT="${EXTERNAL_ODOMETRY_ORIENTATION_FACTOR_WEIGHT:-1.0}"
EXTERNAL_ODOMETRY_ORIENTATION_FACTOR_HUBER_DELTA_RAD="${EXTERNAL_ODOMETRY_ORIENTATION_FACTOR_HUBER_DELTA_RAD:-0.25}"
ENABLE_STARTUP_BIAS_AUTOCAL="${ENABLE_STARTUP_BIAS_AUTOCAL:-true}"
# FAST-LIVO/FAST-LIVO2 frontend_raw bags store IMU linear_acceleration in
# normalized-g units (~1.0 at rest). The continuous-time residuals operate in
# m/s^2, matching the rest of the native tracker stack.
IMU_LINEAR_ACCELERATION_SCALE="${IMU_LINEAR_ACCELERATION_SCALE:-9.80665}"
POINTCLOUD_ENABLE="${POINTCLOUD_ENABLE:-true}"
POINTCLOUD_FACTOR_WEIGHT="${POINTCLOUD_FACTOR_WEIGHT:-0.1}"
POINTCLOUD_USE_LIDAR_SCALE="${POINTCLOUD_USE_LIDAR_SCALE:-true}"
POINTCLOUD_MIN_LIDAR_SCALE="${POINTCLOUD_MIN_LIDAR_SCALE:-0.1}"
POINTCLOUD_WAIT_QUEUE_MAX_SIZE="${POINTCLOUD_WAIT_QUEUE_MAX_SIZE:-100}"
ENABLE_LIDAR_POINT_DESKEW="${ENABLE_LIDAR_POINT_DESKEW:-false}"
LIDAR_MAX_ABS_POINT_TIME_OFFSET_S="${LIDAR_MAX_ABS_POINT_TIME_OFFSET_S:-0.25}"
LIDAR_MAX_DESKEW_DELTA_M="${LIDAR_MAX_DESKEW_DELTA_M:-1.0}"
ENABLE_LIDAR_POSE_PRIOR_FACTOR="${ENABLE_LIDAR_POSE_PRIOR_FACTOR:-false}"
LIDAR_POSE_PRIOR_POSITION_WEIGHT="${LIDAR_POSE_PRIOR_POSITION_WEIGHT:-1.0}"
LIDAR_POSE_PRIOR_VELOCITY_WEIGHT="${LIDAR_POSE_PRIOR_VELOCITY_WEIGHT:-0.0}"
LIDAR_POSE_PRIOR_ANGULAR_VELOCITY_WEIGHT="${LIDAR_POSE_PRIOR_ANGULAR_VELOCITY_WEIGHT:-0.0}"
LIDAR_POSE_PRIOR_ORIENTATION_WEIGHT="${LIDAR_POSE_PRIOR_ORIENTATION_WEIGHT:-1.0}"
LIDAR_POSE_PRIOR_POSITION_HUBER_DELTA_M="${LIDAR_POSE_PRIOR_POSITION_HUBER_DELTA_M:-0.25}"
LIDAR_POSE_PRIOR_VELOCITY_HUBER_DELTA_MPS="${LIDAR_POSE_PRIOR_VELOCITY_HUBER_DELTA_MPS:-0.25}"
LIDAR_POSE_PRIOR_ANGULAR_VELOCITY_HUBER_DELTA_RADPS="${LIDAR_POSE_PRIOR_ANGULAR_VELOCITY_HUBER_DELTA_RADPS:-0.25}"
LIDAR_POSE_PRIOR_ORIENTATION_HUBER_DELTA_RAD="${LIDAR_POSE_PRIOR_ORIENTATION_HUBER_DELTA_RAD:-0.25}"
LIDAR_POSE_FACTOR_KEYFRAME_STRIDE="${LIDAR_POSE_FACTOR_KEYFRAME_STRIDE:-5}"
LIDAR_POSE_FACTOR_MIN_POINTS="${LIDAR_POSE_FACTOR_MIN_POINTS:-32}"
LIDAR_POSE_FACTOR_MAX_FRAME_POINTS="${LIDAR_POSE_FACTOR_MAX_FRAME_POINTS:-2000}"
LIDAR_POSE_FACTOR_MAX_MAP_POINTS="${LIDAR_POSE_FACTOR_MAX_MAP_POINTS:-20000}"
LIDAR_POSE_FACTOR_NEAREST_DISTANCE_M="${LIDAR_POSE_FACTOR_NEAREST_DISTANCE_M:-0.35}"
LIDAR_POSE_FACTOR_CORRECTION_GAIN="${LIDAR_POSE_FACTOR_CORRECTION_GAIN:-0.7}"
LIDAR_POSE_FACTOR_MAX_CORRECTION_M="${LIDAR_POSE_FACTOR_MAX_CORRECTION_M:-0.25}"
LIDAR_POSE_FACTOR_MAX_ROTATION_RAD="${LIDAR_POSE_FACTOR_MAX_ROTATION_RAD:-0.08}"
LIDAR_POSE_FACTOR_ROBUST_KERNEL_M="${LIDAR_POSE_FACTOR_ROBUST_KERNEL_M:-0.15}"
LIDAR_POSE_FACTOR_ITERATIONS="${LIDAR_POSE_FACTOR_ITERATIONS:-1}"
ENABLE_LIDAR_SCAN_TO_SCAN_PRIOR="${ENABLE_LIDAR_SCAN_TO_SCAN_PRIOR:-false}"
LIDAR_SCAN_TO_SCAN_POSITION_WEIGHT="${LIDAR_SCAN_TO_SCAN_POSITION_WEIGHT:-0.0}"
LIDAR_SCAN_TO_SCAN_VELOCITY_WEIGHT="${LIDAR_SCAN_TO_SCAN_VELOCITY_WEIGHT:-0.0}"
LIDAR_SCAN_TO_SCAN_ANGULAR_VELOCITY_WEIGHT="${LIDAR_SCAN_TO_SCAN_ANGULAR_VELOCITY_WEIGHT:-0.0}"
LIDAR_SCAN_TO_SCAN_POSITION_HUBER_DELTA_M="${LIDAR_SCAN_TO_SCAN_POSITION_HUBER_DELTA_M:-0.25}"
LIDAR_SCAN_TO_SCAN_VELOCITY_HUBER_DELTA_MPS="${LIDAR_SCAN_TO_SCAN_VELOCITY_HUBER_DELTA_MPS:-0.25}"
LIDAR_SCAN_TO_SCAN_ANGULAR_VELOCITY_HUBER_DELTA_RADPS="${LIDAR_SCAN_TO_SCAN_ANGULAR_VELOCITY_HUBER_DELTA_RADPS:-0.25}"
LIDAR_SCAN_TO_SCAN_MAX_VELOCITY_MPS="${LIDAR_SCAN_TO_SCAN_MAX_VELOCITY_MPS:-0.0}"
LIDAR_SCAN_TO_SCAN_MAX_ANGULAR_VELOCITY_RADPS="${LIDAR_SCAN_TO_SCAN_MAX_ANGULAR_VELOCITY_RADPS:-0.0}"
LIDAR_SCAN_TO_SCAN_RELATIVE_TRANSLATION_GAIN="${LIDAR_SCAN_TO_SCAN_RELATIVE_TRANSLATION_GAIN:-1.0}"
LIDAR_SCAN_TO_SCAN_ORIENTATION_WEIGHT="${LIDAR_SCAN_TO_SCAN_ORIENTATION_WEIGHT:-0.0}"
LIDAR_SCAN_TO_SCAN_ORIENTATION_HUBER_DELTA_RAD="${LIDAR_SCAN_TO_SCAN_ORIENTATION_HUBER_DELTA_RAD:-0.25}"
LIDAR_SCAN_TO_SCAN_USE_ODOMETRY_PREDICTION="${LIDAR_SCAN_TO_SCAN_USE_ODOMETRY_PREDICTION:-false}"
LIDAR_SCAN_TO_SCAN_USE_POINT_TO_PLANE_CORRECTION="${LIDAR_SCAN_TO_SCAN_USE_POINT_TO_PLANE_CORRECTION:-false}"
LIDAR_SCAN_TO_SCAN_USE_RELATIVE_POSE_FACTOR="${LIDAR_SCAN_TO_SCAN_USE_RELATIVE_POSE_FACTOR:-false}"
LIDAR_SCAN_TO_SCAN_MIN_TARGET_PREDICTION_RATIO="${LIDAR_SCAN_TO_SCAN_MIN_TARGET_PREDICTION_RATIO:-0.0}"
LIDAR_SCAN_TO_SCAN_MIN_TARGET_TRANSLATION_M="${LIDAR_SCAN_TO_SCAN_MIN_TARGET_TRANSLATION_M:-0.0}"
LIDAR_SCAN_TO_SCAN_USE_PREDICTION_ON_SMALL_TARGET="${LIDAR_SCAN_TO_SCAN_USE_PREDICTION_ON_SMALL_TARGET:-false}"
LIDAR_SCAN_TO_SCAN_SKIP_TRANSLATION_PRIORS_ON_SMALL_TARGET="${LIDAR_SCAN_TO_SCAN_SKIP_TRANSLATION_PRIORS_ON_SMALL_TARGET:-false}"
LIDAR_SCAN_TO_SCAN_DEAD_RECKON_ON_REJECT="${LIDAR_SCAN_TO_SCAN_DEAD_RECKON_ON_REJECT:-false}"
LIDAR_SCAN_TO_SCAN_APPLY_POSE_SEED="${LIDAR_SCAN_TO_SCAN_APPLY_POSE_SEED:-false}"
LIDAR_SCAN_TO_SCAN_STORE_CORRECTED_POSE="${LIDAR_SCAN_TO_SCAN_STORE_CORRECTED_POSE:-true}"
LIDAR_SCAN_TO_SCAN_YAW_ONLY_ANGULAR_VELOCITY="${LIDAR_SCAN_TO_SCAN_YAW_ONLY_ANGULAR_VELOCITY:-false}"
LIDAR_SCAN_TO_SCAN_POSE_SEED_POSITION_GAIN="${LIDAR_SCAN_TO_SCAN_POSE_SEED_POSITION_GAIN:-1.0}"
LIDAR_SCAN_TO_SCAN_POSE_SEED_ROTATION_GAIN="${LIDAR_SCAN_TO_SCAN_POSE_SEED_ROTATION_GAIN:-1.0}"
ENABLE_LIDAR_PLANE_NORMAL_FACTOR="${ENABLE_LIDAR_PLANE_NORMAL_FACTOR:-false}"
LIDAR_PLANE_NORMAL_FACTOR_WEIGHT="${LIDAR_PLANE_NORMAL_FACTOR_WEIGHT:-0.1}"
LIDAR_PLANE_NORMAL_HUBER_DELTA_RAD="${LIDAR_PLANE_NORMAL_HUBER_DELTA_RAD:-0.10}"
ENABLE_VISUAL_ROTATION_PRIOR="${ENABLE_VISUAL_ROTATION_PRIOR:-false}"
VISUAL_ROTATION_PRIOR_WEIGHT="${VISUAL_ROTATION_PRIOR_WEIGHT:-0.1}"
VISUAL_ROTATION_PRIOR_HUBER_DELTA_RAD="${VISUAL_ROTATION_PRIOR_HUBER_DELTA_RAD:-0.05}"
VISUAL_ROTATION_MAX_SHIFT_PX="${VISUAL_ROTATION_MAX_SHIFT_PX:-12}"
VISUAL_ROTATION_MIN_PIXELS="${VISUAL_ROTATION_MIN_PIXELS:-2000}"
VISUAL_ROTATION_MAX_PIXELS="${VISUAL_ROTATION_MAX_PIXELS:-20000}"
VISUAL_ROTATION_FRAME_STRIDE="${VISUAL_ROTATION_FRAME_STRIDE:-3}"
VISUAL_ROTATION_MAX_DT_NS="${VISUAL_ROTATION_MAX_DT_NS:-200000000}"
VISUAL_ROTATION_MAX_RMSE="${VISUAL_ROTATION_MAX_RMSE:-0.30}"
VISUAL_ROTATION_PIXEL_TO_RAD_SCALE="${VISUAL_ROTATION_PIXEL_TO_RAD_SCALE:-1.0}"
VISUAL_ROTATION_SIGN="${VISUAL_ROTATION_SIGN:-1.0}"
CAMERA_TO_IMU_ROTATION_XYZW="${CAMERA_TO_IMU_ROTATION_XYZW:-[-0.4991948721, 0.5038197882, -0.4930665852, 0.5038406923]}"
CAMERA_TO_IMU_TRANSLATION_M="${CAMERA_TO_IMU_TRANSLATION_M:-[0.0673699, 0.0412418, 0.0764217]}"
# FAST-LIVO2 frontend_raw keeps Livox points in the raw LiDAR frame. Pass the
# upstream Coco-LIC2 LiDAR->IMU mount into the continuous-time factors rather
# than silently relying on the node's identity fallback.
LIDAR_TO_IMU_ROTATION_XYZW="${LIDAR_TO_IMU_ROTATION_XYZW:-[0.0, 0.0, 0.0, 1.0]}"
LIDAR_TO_IMU_TRANSLATION_M="${LIDAR_TO_IMU_TRANSLATION_M:-[0.04165, 0.02326, -0.0284]}"
ENABLE_VISUAL_SE3_PRIOR="${ENABLE_VISUAL_SE3_PRIOR:-false}"
VISUAL_SE3_POSITION_WEIGHT="${VISUAL_SE3_POSITION_WEIGHT:-0.0}"
VISUAL_SE3_ORIENTATION_WEIGHT="${VISUAL_SE3_ORIENTATION_WEIGHT:-0.0}"
VISUAL_SE3_VELOCITY_WEIGHT="${VISUAL_SE3_VELOCITY_WEIGHT:-0.0}"
VISUAL_SE3_HUBER_DELTA_M="${VISUAL_SE3_HUBER_DELTA_M:-0.05}"
VISUAL_SE3_HUBER_DELTA_RAD="${VISUAL_SE3_HUBER_DELTA_RAD:-0.05}"
VISUAL_SE3_HUBER_DELTA_MPS="${VISUAL_SE3_HUBER_DELTA_MPS:-0.10}"
VISUAL_SE3_MAX_SAMPLES="${VISUAL_SE3_MAX_SAMPLES:-1000}"
VISUAL_SE3_MIN_SAMPLES="${VISUAL_SE3_MIN_SAMPLES:-32}"
VISUAL_SE3_MIN_GRADIENT="${VISUAL_SE3_MIN_GRADIENT:-0.0001}"
VISUAL_SE3_MAX_ABS_RESIDUAL="${VISUAL_SE3_MAX_ABS_RESIDUAL:-0.5}"
VISUAL_SE3_HUBER_DELTA_INTENSITY="${VISUAL_SE3_HUBER_DELTA_INTENSITY:-0.15}"
VISUAL_SE3_MIN_DEPTH_M="${VISUAL_SE3_MIN_DEPTH_M:-0.05}"
VISUAL_SE3_MAX_DEPTH_M="${VISUAL_SE3_MAX_DEPTH_M:-80.0}"
VISUAL_SE3_DEPTH_DILATION_PX="${VISUAL_SE3_DEPTH_DILATION_PX:-2}"
VISUAL_SE3_DEPTH_CACHE_SIZE="${VISUAL_SE3_DEPTH_CACHE_SIZE:-8}"
VISUAL_SE3_MAX_DT_NS="${VISUAL_SE3_MAX_DT_NS:-100000000}"
VISUAL_SE3_MIN_HESSIAN_RANK="${VISUAL_SE3_MIN_HESSIAN_RANK:-4}"
VISUAL_SE3_MAX_HESSIAN_CONDITION="${VISUAL_SE3_MAX_HESSIAN_CONDITION:-1000000000000.0}"
VISUAL_SE3_MIN_SAMPLE_INLIER_RATIO="${VISUAL_SE3_MIN_SAMPLE_INLIER_RATIO:-0.20}"
VISUAL_SE3_COVERAGE_GRID_COLS="${VISUAL_SE3_COVERAGE_GRID_COLS:-4}"
VISUAL_SE3_COVERAGE_GRID_ROWS="${VISUAL_SE3_COVERAGE_GRID_ROWS:-4}"
VISUAL_SE3_MIN_COVERAGE_TILES="${VISUAL_SE3_MIN_COVERAGE_TILES:-4}"
VISUAL_SE3_MAX_MEAN_ABS_RESIDUAL="${VISUAL_SE3_MAX_MEAN_ABS_RESIDUAL:-0.0}"
VISUAL_SE3_MAX_TRANSLATION_STEP_M="${VISUAL_SE3_MAX_TRANSLATION_STEP_M:-0.25}"
VISUAL_SE3_MAX_ROTATION_STEP_RAD="${VISUAL_SE3_MAX_ROTATION_STEP_RAD:-0.15}"
VISUAL_SE3_DELTA_SIGN="${VISUAL_SE3_DELTA_SIGN:-1.0}"
MAX_ITERATIONS_PER_STEP="${MAX_ITERATIONS_PER_STEP:-1}"
IMU_INFO_GYRO="${IMU_INFO_GYRO:-10.0}"
IMU_INFO_ACCEL="${IMU_INFO_ACCEL:-1.0}"
HOLD_GYRO_BIAS_CONSTANT="${HOLD_GYRO_BIAS_CONSTANT:-false}"
HOLD_ACCEL_BIAS_CONSTANT="${HOLD_ACCEL_BIAS_CONSTANT:-false}"
HOLD_GRAVITY_CONSTANT="${HOLD_GRAVITY_CONSTANT:-true}"
FIXED_CONTROL_POINT_INDEX="${FIXED_CONTROL_POINT_INDEX:--1}"
CERES_INITIAL_TRUST_REGION_RADIUS="${CERES_INITIAL_TRUST_REGION_RADIUS:-0.0}"
CERES_MAX_TRUST_REGION_RADIUS="${CERES_MAX_TRUST_REGION_RADIUS:-0.0}"
POSITION_SMOOTHNESS_WEIGHT="${POSITION_SMOOTHNESS_WEIGHT:-0.0}"
POSITION_SMOOTHNESS_HUBER_DELTA_M="${POSITION_SMOOTHNESS_HUBER_DELTA_M:-0.0}"
ROTATION_SMOOTHNESS_WEIGHT="${ROTATION_SMOOTHNESS_WEIGHT:-0.0}"
ROTATION_SMOOTHNESS_HUBER_DELTA_RAD="${ROTATION_SMOOTHNESS_HUBER_DELTA_RAD:-0.0}"
RETAINED_KNOT_PRIOR_COUNT="${RETAINED_KNOT_PRIOR_COUNT:-0}"
RETAINED_KNOT_POSITION_PRIOR_WEIGHT="${RETAINED_KNOT_POSITION_PRIOR_WEIGHT:-0.0}"
RETAINED_KNOT_POSITION_PRIOR_HUBER_DELTA_M="${RETAINED_KNOT_POSITION_PRIOR_HUBER_DELTA_M:-0.0}"
RETAINED_KNOT_ORIENTATION_PRIOR_WEIGHT="${RETAINED_KNOT_ORIENTATION_PRIOR_WEIGHT:-0.0}"
RETAINED_KNOT_ORIENTATION_PRIOR_HUBER_DELTA_RAD="${RETAINED_KNOT_ORIENTATION_PRIOR_HUBER_DELTA_RAD:-0.0}"
GYRO_BIAS_PRIOR_WEIGHT="${GYRO_BIAS_PRIOR_WEIGHT:-0.0}"
GYRO_BIAS_PRIOR_HUBER_DELTA_RADPS="${GYRO_BIAS_PRIOR_HUBER_DELTA_RADPS:-0.0}"
ACCEL_BIAS_PRIOR_WEIGHT="${ACCEL_BIAS_PRIOR_WEIGHT:-0.0}"
ACCEL_BIAS_PRIOR_HUBER_DELTA_MPS2="${ACCEL_BIAS_PRIOR_HUBER_DELTA_MPS2:-0.0}"
APPLY_POSITION_UPDATE_ON_ROTATION_REJECT="${APPLY_POSITION_UPDATE_ON_ROTATION_REJECT:-false}"
APPLY_LIMITED_ROTATION_UPDATE="${APPLY_LIMITED_ROTATION_UPDATE:-false}"
APPLY_LIMITED_POSITION_UPDATE="${APPLY_LIMITED_POSITION_UPDATE:-false}"
SCALE_POSITION_WITH_LIMITED_ROTATION="${SCALE_POSITION_WITH_LIMITED_ROTATION:-true}"
ENABLE_VOXEL_PLANE_EXTRACTION="${ENABLE_VOXEL_PLANE_EXTRACTION:-true}"
ENABLE_PERSISTENT_PLANE_MAP="${ENABLE_PERSISTENT_PLANE_MAP:-true}"
ENABLE_PERSISTENT_POINT_MAP="${ENABLE_PERSISTENT_POINT_MAP:-false}"
PERSISTENT_MAP_UPDATE_REQUIRES_ACCEPTED_SOLVE="${PERSISTENT_MAP_UPDATE_REQUIRES_ACCEPTED_SOLVE:-false}"
DEFER_PERSISTENT_PLANE_MAP_UPDATES_UNTIL_SOLVED="${DEFER_PERSISTENT_PLANE_MAP_UPDATES_UNTIL_SOLVED:-false}"
DEFERRED_PLANE_MAP_UPDATE_MAX_QUEUE="${DEFERRED_PLANE_MAP_UPDATE_MAX_QUEUE:-200}"
PERSISTENT_POINT_MAP_NEAREST_DISTANCE_M="${PERSISTENT_POINT_MAP_NEAREST_DISTANCE_M:-0.35}"
PERSISTENT_POINT_MAP_MERGE_DISTANCE_M="${PERSISTENT_POINT_MAP_MERGE_DISTANCE_M:-0.10}"
PERSISTENT_POINT_MAP_MIN_MATCH_AGE_S="${PERSISTENT_POINT_MAP_MIN_MATCH_AGE_S:-0.25}"
PERSISTENT_POINT_MAP_FACTOR_WEIGHT="${PERSISTENT_POINT_MAP_FACTOR_WEIGHT:-0.05}"
PERSISTENT_POINT_MAP_SUBSAMPLE_STRIDE="${PERSISTENT_POINT_MAP_SUBSAMPLE_STRIDE:-20}"
PERSISTENT_POINT_MAP_MAX_POINTS="${PERSISTENT_POINT_MAP_MAX_POINTS:-20000}"
PERSISTENT_POINT_MAP_MAX_CORRESPONDENCES="${PERSISTENT_POINT_MAP_MAX_CORRESPONDENCES:-64}"
PERSISTENT_POINT_MAP_MIN_OBSERVATIONS="${PERSISTENT_POINT_MAP_MIN_OBSERVATIONS:-3}"
ENABLE_GAUSSIAN_SNAPSHOT_LIDAR_FACTOR="${ENABLE_GAUSSIAN_SNAPSHOT_LIDAR_FACTOR:-false}"
GAUSSIAN_MAP_TOPIC="${GAUSSIAN_MAP_TOPIC:-/gaussian_lic/gaussian_map}"
GAUSSIAN_SNAPSHOT_LIDAR_FACTOR_WEIGHT="${GAUSSIAN_SNAPSHOT_LIDAR_FACTOR_WEIGHT:-0.05}"
GAUSSIAN_SNAPSHOT_LIDAR_NEAREST_DISTANCE_M="${GAUSSIAN_SNAPSHOT_LIDAR_NEAREST_DISTANCE_M:-0.35}"
GAUSSIAN_SNAPSHOT_LIDAR_MIN_OPACITY="${GAUSSIAN_SNAPSHOT_LIDAR_MIN_OPACITY:-0.01}"
GAUSSIAN_SNAPSHOT_LIDAR_SUBSAMPLE_STRIDE="${GAUSSIAN_SNAPSHOT_LIDAR_SUBSAMPLE_STRIDE:-20}"
GAUSSIAN_SNAPSHOT_MAP_SUBSAMPLE_STRIDE="${GAUSSIAN_SNAPSHOT_MAP_SUBSAMPLE_STRIDE:-1}"
GAUSSIAN_SNAPSHOT_LIDAR_MAX_CORRESPONDENCES="${GAUSSIAN_SNAPSHOT_LIDAR_MAX_CORRESPONDENCES:-64}"
LIDAR_HUBER_DELTA_M="${LIDAR_HUBER_DELTA_M:-0.10}"
VOXEL_PLANE_SIZE_M="${VOXEL_PLANE_SIZE_M:-0.8}"
VOXEL_PLANE_MIN_POINTS="${VOXEL_PLANE_MIN_POINTS:-6}"
VOXEL_PLANE_EIGEN_RATIO="${VOXEL_PLANE_EIGEN_RATIO:-0.10}"
VOXEL_PLANE_MAX_INLIER_M="${VOXEL_PLANE_MAX_INLIER_M:-0.20}"
VOXEL_PLANE_MAX_CORRESPONDENCES="${VOXEL_PLANE_MAX_CORRESPONDENCES:-48}"
PERSISTENT_PLANE_MAP_MIN_OBSERVATIONS="${PERSISTENT_PLANE_MAP_MIN_OBSERVATIONS:-3}"
MAX_POSITION_UPDATE_M="${MAX_POSITION_UPDATE_M:-2.0}"
MAX_ROTATION_UPDATE_RAD="${MAX_ROTATION_UPDATE_RAD:-0.50}"
UPDATE_GATE_EDGE_KNOT_MARGIN="${UPDATE_GATE_EDGE_KNOT_MARGIN:-0}"
POSITION_EXTRAPOLATION_DAMPING="${POSITION_EXTRAPOLATION_DAMPING:-0.0}"
STEP_PERIOD_SECONDS="${STEP_PERIOD_SECONDS:-0.20}"
POSE_OUTPUT_PERIOD_SECONDS="${POSE_OUTPUT_PERIOD_SECONDS:-0.0}"
TIME_OFFSET_SWEEP_MIN="${TIME_OFFSET_SWEEP_MIN:--12.0}"
TIME_OFFSET_SWEEP_MAX="${TIME_OFFSET_SWEEP_MAX:-12.0}"
TIME_OFFSET_SWEEP_STEP="${TIME_OFFSET_SWEEP_STEP:-0.1}"
TIME_OFFSET_SWEEP_MIN_MATCHES="${TIME_OFFSET_SWEEP_MIN_MATCHES:-50}"

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
  -p max_iterations_per_step:="${MAX_ITERATIONS_PER_STEP}" \
  -p imu_info_gyro:="${IMU_INFO_GYRO}" \
  -p imu_info_accel:="${IMU_INFO_ACCEL}" \
  -p hold_gyro_bias_constant:="${HOLD_GYRO_BIAS_CONSTANT}" \
  -p hold_accel_bias_constant:="${HOLD_ACCEL_BIAS_CONSTANT}" \
  -p hold_gravity_constant:="${HOLD_GRAVITY_CONSTANT}" \
  -p fixed_control_point_index:="${FIXED_CONTROL_POINT_INDEX}" \
  -p ceres_initial_trust_region_radius:="${CERES_INITIAL_TRUST_REGION_RADIUS}" \
  -p ceres_max_trust_region_radius:="${CERES_MAX_TRUST_REGION_RADIUS}" \
  -p position_smoothness_weight:="${POSITION_SMOOTHNESS_WEIGHT}" \
  -p position_smoothness_huber_delta_m:="${POSITION_SMOOTHNESS_HUBER_DELTA_M}" \
  -p rotation_smoothness_weight:="${ROTATION_SMOOTHNESS_WEIGHT}" \
  -p rotation_smoothness_huber_delta_rad:="${ROTATION_SMOOTHNESS_HUBER_DELTA_RAD}" \
  -p retained_knot_prior_count:="${RETAINED_KNOT_PRIOR_COUNT}" \
  -p retained_knot_position_prior_weight:="${RETAINED_KNOT_POSITION_PRIOR_WEIGHT}" \
  -p retained_knot_position_prior_huber_delta_m:="${RETAINED_KNOT_POSITION_PRIOR_HUBER_DELTA_M}" \
  -p retained_knot_orientation_prior_weight:="${RETAINED_KNOT_ORIENTATION_PRIOR_WEIGHT}" \
  -p retained_knot_orientation_prior_huber_delta_rad:="${RETAINED_KNOT_ORIENTATION_PRIOR_HUBER_DELTA_RAD}" \
  -p gyro_bias_prior_weight:="${GYRO_BIAS_PRIOR_WEIGHT}" \
  -p gyro_bias_prior_huber_delta_radps:="${GYRO_BIAS_PRIOR_HUBER_DELTA_RADPS}" \
  -p accel_bias_prior_weight:="${ACCEL_BIAS_PRIOR_WEIGHT}" \
  -p accel_bias_prior_huber_delta_mps2:="${ACCEL_BIAS_PRIOR_HUBER_DELTA_MPS2}" \
  -p seed_min_imu_count:=30 \
  -p enable_startup_bias_autocal:="${ENABLE_STARTUP_BIAS_AUTOCAL}" \
  -p imu_linear_acceleration_scale:="${IMU_LINEAR_ACCELERATION_SCALE}" \
  -p step_period_seconds:="${STEP_PERIOD_SECONDS}" \
  -p pose_output_period_seconds:="${POSE_OUTPUT_PERIOD_SECONDS}" \
  -p diagnostic_log_period_steps:="${DIAGNOSTIC_LOG_PERIOD_STEPS}" \
  -p pointcloud_enable:="${POINTCLOUD_ENABLE}" \
  -p pointcloud_factor_weight:="${POINTCLOUD_FACTOR_WEIGHT}" \
  -p pointcloud_use_lidar_scale:="${POINTCLOUD_USE_LIDAR_SCALE}" \
  -p pointcloud_min_lidar_scale:="${POINTCLOUD_MIN_LIDAR_SCALE}" \
  -p pointcloud_wait_queue_max_size:="${POINTCLOUD_WAIT_QUEUE_MAX_SIZE}" \
  -p enable_lidar_point_deskew:="${ENABLE_LIDAR_POINT_DESKEW}" \
  -p lidar_max_abs_point_time_offset_s:="${LIDAR_MAX_ABS_POINT_TIME_OFFSET_S}" \
  -p lidar_max_deskew_delta_m:="${LIDAR_MAX_DESKEW_DELTA_M}" \
  -p enable_lidar_pose_prior_factor:="${ENABLE_LIDAR_POSE_PRIOR_FACTOR}" \
  -p lidar_pose_prior_position_weight:="${LIDAR_POSE_PRIOR_POSITION_WEIGHT}" \
  -p lidar_pose_prior_velocity_weight:="${LIDAR_POSE_PRIOR_VELOCITY_WEIGHT}" \
  -p lidar_pose_prior_angular_velocity_weight:="${LIDAR_POSE_PRIOR_ANGULAR_VELOCITY_WEIGHT}" \
  -p lidar_pose_prior_orientation_weight:="${LIDAR_POSE_PRIOR_ORIENTATION_WEIGHT}" \
  -p lidar_pose_prior_position_huber_delta_m:="${LIDAR_POSE_PRIOR_POSITION_HUBER_DELTA_M}" \
  -p lidar_pose_prior_velocity_huber_delta_mps:="${LIDAR_POSE_PRIOR_VELOCITY_HUBER_DELTA_MPS}" \
  -p lidar_pose_prior_angular_velocity_huber_delta_radps:="${LIDAR_POSE_PRIOR_ANGULAR_VELOCITY_HUBER_DELTA_RADPS}" \
  -p lidar_pose_prior_orientation_huber_delta_rad:="${LIDAR_POSE_PRIOR_ORIENTATION_HUBER_DELTA_RAD}" \
  -p lidar_pose_factor_keyframe_stride:="${LIDAR_POSE_FACTOR_KEYFRAME_STRIDE}" \
  -p lidar_pose_factor_min_points:="${LIDAR_POSE_FACTOR_MIN_POINTS}" \
  -p lidar_pose_factor_max_frame_points:="${LIDAR_POSE_FACTOR_MAX_FRAME_POINTS}" \
  -p lidar_pose_factor_max_map_points:="${LIDAR_POSE_FACTOR_MAX_MAP_POINTS}" \
  -p lidar_pose_factor_nearest_distance_m:="${LIDAR_POSE_FACTOR_NEAREST_DISTANCE_M}" \
  -p lidar_pose_factor_correction_gain:="${LIDAR_POSE_FACTOR_CORRECTION_GAIN}" \
  -p lidar_pose_factor_max_correction_m:="${LIDAR_POSE_FACTOR_MAX_CORRECTION_M}" \
  -p lidar_pose_factor_max_rotation_rad:="${LIDAR_POSE_FACTOR_MAX_ROTATION_RAD}" \
  -p lidar_pose_factor_robust_kernel_m:="${LIDAR_POSE_FACTOR_ROBUST_KERNEL_M}" \
  -p lidar_pose_factor_iterations:="${LIDAR_POSE_FACTOR_ITERATIONS}" \
  -p enable_lidar_scan_to_scan_prior:="${ENABLE_LIDAR_SCAN_TO_SCAN_PRIOR}" \
  -p lidar_scan_to_scan_position_weight:="${LIDAR_SCAN_TO_SCAN_POSITION_WEIGHT}" \
  -p lidar_scan_to_scan_velocity_weight:="${LIDAR_SCAN_TO_SCAN_VELOCITY_WEIGHT}" \
  -p lidar_scan_to_scan_angular_velocity_weight:="${LIDAR_SCAN_TO_SCAN_ANGULAR_VELOCITY_WEIGHT}" \
  -p lidar_scan_to_scan_position_huber_delta_m:="${LIDAR_SCAN_TO_SCAN_POSITION_HUBER_DELTA_M}" \
  -p lidar_scan_to_scan_velocity_huber_delta_mps:="${LIDAR_SCAN_TO_SCAN_VELOCITY_HUBER_DELTA_MPS}" \
  -p lidar_scan_to_scan_angular_velocity_huber_delta_radps:="${LIDAR_SCAN_TO_SCAN_ANGULAR_VELOCITY_HUBER_DELTA_RADPS}" \
  -p lidar_scan_to_scan_max_velocity_mps:="${LIDAR_SCAN_TO_SCAN_MAX_VELOCITY_MPS}" \
  -p lidar_scan_to_scan_max_angular_velocity_radps:="${LIDAR_SCAN_TO_SCAN_MAX_ANGULAR_VELOCITY_RADPS}" \
  -p lidar_scan_to_scan_relative_translation_gain:="${LIDAR_SCAN_TO_SCAN_RELATIVE_TRANSLATION_GAIN}" \
  -p lidar_scan_to_scan_orientation_weight:="${LIDAR_SCAN_TO_SCAN_ORIENTATION_WEIGHT}" \
  -p lidar_scan_to_scan_orientation_huber_delta_rad:="${LIDAR_SCAN_TO_SCAN_ORIENTATION_HUBER_DELTA_RAD}" \
  -p lidar_scan_to_scan_use_odometry_prediction:="${LIDAR_SCAN_TO_SCAN_USE_ODOMETRY_PREDICTION}" \
  -p lidar_scan_to_scan_use_point_to_plane_correction:="${LIDAR_SCAN_TO_SCAN_USE_POINT_TO_PLANE_CORRECTION}" \
  -p lidar_scan_to_scan_use_relative_pose_factor:="${LIDAR_SCAN_TO_SCAN_USE_RELATIVE_POSE_FACTOR}" \
  -p lidar_scan_to_scan_min_target_prediction_ratio:="${LIDAR_SCAN_TO_SCAN_MIN_TARGET_PREDICTION_RATIO}" \
  -p lidar_scan_to_scan_min_target_translation_m:="${LIDAR_SCAN_TO_SCAN_MIN_TARGET_TRANSLATION_M}" \
  -p lidar_scan_to_scan_use_prediction_on_small_target:="${LIDAR_SCAN_TO_SCAN_USE_PREDICTION_ON_SMALL_TARGET}" \
  -p lidar_scan_to_scan_skip_translation_priors_on_small_target:="${LIDAR_SCAN_TO_SCAN_SKIP_TRANSLATION_PRIORS_ON_SMALL_TARGET}" \
  -p lidar_scan_to_scan_dead_reckon_on_reject:="${LIDAR_SCAN_TO_SCAN_DEAD_RECKON_ON_REJECT}" \
  -p lidar_scan_to_scan_apply_pose_seed:="${LIDAR_SCAN_TO_SCAN_APPLY_POSE_SEED}" \
  -p lidar_scan_to_scan_store_corrected_pose:="${LIDAR_SCAN_TO_SCAN_STORE_CORRECTED_POSE}" \
  -p lidar_scan_to_scan_yaw_only_angular_velocity:="${LIDAR_SCAN_TO_SCAN_YAW_ONLY_ANGULAR_VELOCITY}" \
  -p lidar_scan_to_scan_pose_seed_position_gain:="${LIDAR_SCAN_TO_SCAN_POSE_SEED_POSITION_GAIN}" \
  -p lidar_scan_to_scan_pose_seed_rotation_gain:="${LIDAR_SCAN_TO_SCAN_POSE_SEED_ROTATION_GAIN}" \
  -p enable_lidar_plane_normal_factor:="${ENABLE_LIDAR_PLANE_NORMAL_FACTOR}" \
  -p lidar_plane_normal_factor_weight:="${LIDAR_PLANE_NORMAL_FACTOR_WEIGHT}" \
  -p lidar_plane_normal_huber_delta_rad:="${LIDAR_PLANE_NORMAL_HUBER_DELTA_RAD}" \
  -p enable_visual_rotation_prior:="${ENABLE_VISUAL_ROTATION_PRIOR}" \
  -p visual_rotation_prior_weight:="${VISUAL_ROTATION_PRIOR_WEIGHT}" \
  -p visual_rotation_prior_huber_delta_rad:="${VISUAL_ROTATION_PRIOR_HUBER_DELTA_RAD}" \
  -p visual_rotation_max_shift_px:="${VISUAL_ROTATION_MAX_SHIFT_PX}" \
  -p visual_rotation_min_pixels:="${VISUAL_ROTATION_MIN_PIXELS}" \
  -p visual_rotation_max_pixels:="${VISUAL_ROTATION_MAX_PIXELS}" \
  -p visual_rotation_frame_stride:="${VISUAL_ROTATION_FRAME_STRIDE}" \
  -p visual_rotation_max_dt_ns:="${VISUAL_ROTATION_MAX_DT_NS}" \
  -p visual_rotation_max_rmse:="${VISUAL_ROTATION_MAX_RMSE}" \
  -p visual_rotation_pixel_to_rad_scale:="${VISUAL_ROTATION_PIXEL_TO_RAD_SCALE}" \
  -p visual_rotation_sign:="${VISUAL_ROTATION_SIGN}" \
  -p camera_to_imu_rotation_xyzw:="${CAMERA_TO_IMU_ROTATION_XYZW}" \
  -p camera_to_imu_translation_m:="${CAMERA_TO_IMU_TRANSLATION_M}" \
  -p lidar_to_imu_rotation_xyzw:="${LIDAR_TO_IMU_ROTATION_XYZW}" \
  -p lidar_to_imu_translation:="${LIDAR_TO_IMU_TRANSLATION_M}" \
  -p enable_visual_se3_prior:="${ENABLE_VISUAL_SE3_PRIOR}" \
  -p visual_se3_position_weight:="${VISUAL_SE3_POSITION_WEIGHT}" \
  -p visual_se3_orientation_weight:="${VISUAL_SE3_ORIENTATION_WEIGHT}" \
  -p visual_se3_velocity_weight:="${VISUAL_SE3_VELOCITY_WEIGHT}" \
  -p visual_se3_huber_delta_m:="${VISUAL_SE3_HUBER_DELTA_M}" \
  -p visual_se3_huber_delta_rad:="${VISUAL_SE3_HUBER_DELTA_RAD}" \
  -p visual_se3_huber_delta_mps:="${VISUAL_SE3_HUBER_DELTA_MPS}" \
  -p visual_se3_max_samples:="${VISUAL_SE3_MAX_SAMPLES}" \
  -p visual_se3_min_samples:="${VISUAL_SE3_MIN_SAMPLES}" \
  -p visual_se3_min_gradient:="${VISUAL_SE3_MIN_GRADIENT}" \
  -p visual_se3_max_abs_residual:="${VISUAL_SE3_MAX_ABS_RESIDUAL}" \
  -p visual_se3_huber_delta_intensity:="${VISUAL_SE3_HUBER_DELTA_INTENSITY}" \
  -p visual_se3_min_depth_m:="${VISUAL_SE3_MIN_DEPTH_M}" \
  -p visual_se3_max_depth_m:="${VISUAL_SE3_MAX_DEPTH_M}" \
  -p visual_se3_depth_dilation_px:="${VISUAL_SE3_DEPTH_DILATION_PX}" \
  -p visual_se3_depth_cache_size:="${VISUAL_SE3_DEPTH_CACHE_SIZE}" \
  -p visual_se3_max_dt_ns:="${VISUAL_SE3_MAX_DT_NS}" \
  -p visual_se3_min_hessian_rank:="${VISUAL_SE3_MIN_HESSIAN_RANK}" \
  -p visual_se3_max_hessian_condition:="${VISUAL_SE3_MAX_HESSIAN_CONDITION}" \
  -p visual_se3_min_sample_inlier_ratio:="${VISUAL_SE3_MIN_SAMPLE_INLIER_RATIO}" \
  -p visual_se3_coverage_grid_cols:="${VISUAL_SE3_COVERAGE_GRID_COLS}" \
  -p visual_se3_coverage_grid_rows:="${VISUAL_SE3_COVERAGE_GRID_ROWS}" \
  -p visual_se3_min_coverage_tiles:="${VISUAL_SE3_MIN_COVERAGE_TILES}" \
  -p visual_se3_max_mean_abs_residual:="${VISUAL_SE3_MAX_MEAN_ABS_RESIDUAL}" \
  -p visual_se3_max_translation_step_m:="${VISUAL_SE3_MAX_TRANSLATION_STEP_M}" \
  -p visual_se3_max_rotation_step_rad:="${VISUAL_SE3_MAX_ROTATION_STEP_RAD}" \
  -p visual_se3_delta_sign:="${VISUAL_SE3_DELTA_SIGN}" \
  -p lidar_huber_delta_m:="${LIDAR_HUBER_DELTA_M}" \
  -p enable_voxel_plane_extraction:="${ENABLE_VOXEL_PLANE_EXTRACTION}" \
  -p enable_persistent_plane_map:="${ENABLE_PERSISTENT_PLANE_MAP}" \
  -p enable_persistent_point_map:="${ENABLE_PERSISTENT_POINT_MAP}" \
  -p persistent_map_update_requires_accepted_solve:="${PERSISTENT_MAP_UPDATE_REQUIRES_ACCEPTED_SOLVE}" \
  -p defer_persistent_plane_map_updates_until_solved:="${DEFER_PERSISTENT_PLANE_MAP_UPDATES_UNTIL_SOLVED}" \
  -p deferred_plane_map_update_max_queue:="${DEFERRED_PLANE_MAP_UPDATE_MAX_QUEUE}" \
  -p persistent_point_map_nearest_distance_m:="${PERSISTENT_POINT_MAP_NEAREST_DISTANCE_M}" \
  -p persistent_point_map_merge_distance_m:="${PERSISTENT_POINT_MAP_MERGE_DISTANCE_M}" \
  -p persistent_point_map_min_match_age_s:="${PERSISTENT_POINT_MAP_MIN_MATCH_AGE_S}" \
  -p persistent_point_map_factor_weight:="${PERSISTENT_POINT_MAP_FACTOR_WEIGHT}" \
  -p persistent_point_map_subsample_stride:="${PERSISTENT_POINT_MAP_SUBSAMPLE_STRIDE}" \
  -p persistent_point_map_max_points:="${PERSISTENT_POINT_MAP_MAX_POINTS}" \
  -p persistent_point_map_max_correspondences:="${PERSISTENT_POINT_MAP_MAX_CORRESPONDENCES}" \
  -p persistent_point_map_min_observations_for_match:="${PERSISTENT_POINT_MAP_MIN_OBSERVATIONS}" \
  -p enable_gaussian_snapshot_lidar_factor:="${ENABLE_GAUSSIAN_SNAPSHOT_LIDAR_FACTOR}" \
  -p gaussian_map_topic:="${GAUSSIAN_MAP_TOPIC}" \
  -p gaussian_snapshot_lidar_factor_weight:="${GAUSSIAN_SNAPSHOT_LIDAR_FACTOR_WEIGHT}" \
  -p gaussian_snapshot_lidar_nearest_distance_m:="${GAUSSIAN_SNAPSHOT_LIDAR_NEAREST_DISTANCE_M}" \
  -p gaussian_snapshot_lidar_min_opacity:="${GAUSSIAN_SNAPSHOT_LIDAR_MIN_OPACITY}" \
  -p gaussian_snapshot_lidar_subsample_stride:="${GAUSSIAN_SNAPSHOT_LIDAR_SUBSAMPLE_STRIDE}" \
  -p gaussian_snapshot_map_subsample_stride:="${GAUSSIAN_SNAPSHOT_MAP_SUBSAMPLE_STRIDE}" \
  -p gaussian_snapshot_lidar_max_correspondences:="${GAUSSIAN_SNAPSHOT_LIDAR_MAX_CORRESPONDENCES}" \
  -p voxel_plane_size_m:="${VOXEL_PLANE_SIZE_M}" \
  -p voxel_plane_min_points:="${VOXEL_PLANE_MIN_POINTS}" \
  -p voxel_plane_eigen_ratio:="${VOXEL_PLANE_EIGEN_RATIO}" \
  -p voxel_plane_max_inlier_m:="${VOXEL_PLANE_MAX_INLIER_M}" \
  -p voxel_plane_max_correspondences:="${VOXEL_PLANE_MAX_CORRESPONDENCES}" \
  -p persistent_plane_map_min_observations_for_match:="${PERSISTENT_PLANE_MAP_MIN_OBSERVATIONS}" \
  -p max_position_update_m:="${MAX_POSITION_UPDATE_M}" \
  -p max_rotation_update_rad:="${MAX_ROTATION_UPDATE_RAD}" \
  -p update_gate_edge_knot_margin:="${UPDATE_GATE_EDGE_KNOT_MARGIN}" \
  -p position_extrapolation_damping:="${POSITION_EXTRAPOLATION_DAMPING}" \
  -p apply_position_update_on_rotation_reject:="${APPLY_POSITION_UPDATE_ON_ROTATION_REJECT}" \
  -p apply_limited_rotation_update:="${APPLY_LIMITED_ROTATION_UPDATE}" \
  -p apply_limited_position_update:="${APPLY_LIMITED_POSITION_UPDATE}" \
  -p scale_position_with_limited_rotation:="${SCALE_POSITION_WITH_LIMITED_ROTATION}" \
  -p enable_external_odometry_prior:="$([ -n "${PRIOR_TUM}" ] && echo true || echo false)" \
  -p enable_external_odometry_position_factors:="${ENABLE_EXTERNAL_ODOMETRY_POSITION_FACTORS}" \
  -p enable_external_odometry_orientation_factors:="${ENABLE_EXTERNAL_ODOMETRY_ORIENTATION_FACTORS}" \
  -p external_odometry_position_factor_weight:="${EXTERNAL_ODOMETRY_POSITION_FACTOR_WEIGHT}" \
  -p external_odometry_position_factor_huber_delta_m:="${EXTERNAL_ODOMETRY_POSITION_FACTOR_HUBER_DELTA_M}" \
  -p external_odometry_orientation_factor_weight:="${EXTERNAL_ODOMETRY_ORIENTATION_FACTOR_WEIGHT}" \
  -p external_odometry_orientation_factor_huber_delta_rad:="${EXTERNAL_ODOMETRY_ORIENTATION_FACTOR_HUBER_DELTA_RAD}" \
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
    --rate-hz "${PRIOR_PUBLISH_RATE_HZ}" \
    --duration "${PRIOR_PUBLISH_DURATION}" \
    --max-messages "${PRIOR_MAX_MESSAGES}" \
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
    --time-offset-sweep-min "${TIME_OFFSET_SWEEP_MIN}" \
    --time-offset-sweep-max "${TIME_OFFSET_SWEEP_MAX}" \
    --time-offset-sweep-step "${TIME_OFFSET_SWEEP_STEP}" \
    --time-offset-sweep-min-matches "${TIME_OFFSET_SWEEP_MIN_MATCHES}" \
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
node_log = "${NODE_LOG}"
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

runtime_diagnostics = {}
if os.path.isfile(node_log):
    txt = open(node_log, "r", encoding="utf-8", errors="replace").read()
    matches = re.findall(r"continuous-time diagnostics:\s*(.*)", txt)
    if matches:
        value_pattern = r"-?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+)(?:[eE][+-]?[0-9]+)?"
        for key, value in re.findall(rf"([a-z_]+)=({value_pattern})", matches[-1]):
            if "." in value or "e" in value.lower():
                runtime_diagnostics[key] = float(value)
            else:
                runtime_diagnostics[key] = int(value)

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
    "diagnostic_log_period_steps": int("${DIAGNOSTIC_LOG_PERIOD_STEPS}"),
    "imu_linear_acceleration_scale": float("${IMU_LINEAR_ACCELERATION_SCALE}"),
    "max_iterations_per_step": int("${MAX_ITERATIONS_PER_STEP}"),
    "imu_info_gyro": float("${IMU_INFO_GYRO}"),
    "imu_info_accel": float("${IMU_INFO_ACCEL}"),
    "hold_gyro_bias_constant": "${HOLD_GYRO_BIAS_CONSTANT}" == "true",
    "hold_accel_bias_constant": "${HOLD_ACCEL_BIAS_CONSTANT}" == "true",
    "hold_gravity_constant": "${HOLD_GRAVITY_CONSTANT}" == "true",
    "fixed_control_point_index": int("${FIXED_CONTROL_POINT_INDEX}"),
    "ceres_initial_trust_region_radius": float("${CERES_INITIAL_TRUST_REGION_RADIUS}"),
    "ceres_max_trust_region_radius": float("${CERES_MAX_TRUST_REGION_RADIUS}"),
    "position_smoothness_weight": float("${POSITION_SMOOTHNESS_WEIGHT}"),
    "position_smoothness_huber_delta_m": float("${POSITION_SMOOTHNESS_HUBER_DELTA_M}"),
    "rotation_smoothness_weight": float("${ROTATION_SMOOTHNESS_WEIGHT}"),
    "rotation_smoothness_huber_delta_rad": float("${ROTATION_SMOOTHNESS_HUBER_DELTA_RAD}"),
    "retained_knot_prior_count": int("${RETAINED_KNOT_PRIOR_COUNT}"),
    "retained_knot_position_prior_weight": float("${RETAINED_KNOT_POSITION_PRIOR_WEIGHT}"),
    "retained_knot_position_prior_huber_delta_m": float("${RETAINED_KNOT_POSITION_PRIOR_HUBER_DELTA_M}"),
    "retained_knot_orientation_prior_weight": float("${RETAINED_KNOT_ORIENTATION_PRIOR_WEIGHT}"),
    "retained_knot_orientation_prior_huber_delta_rad": float("${RETAINED_KNOT_ORIENTATION_PRIOR_HUBER_DELTA_RAD}"),
    "gyro_bias_prior_weight": float("${GYRO_BIAS_PRIOR_WEIGHT}"),
    "gyro_bias_prior_huber_delta_radps": float("${GYRO_BIAS_PRIOR_HUBER_DELTA_RADPS}"),
    "accel_bias_prior_weight": float("${ACCEL_BIAS_PRIOR_WEIGHT}"),
    "accel_bias_prior_huber_delta_mps2": float("${ACCEL_BIAS_PRIOR_HUBER_DELTA_MPS2}"),
    "pointcloud_factor_weight": float("${POINTCLOUD_FACTOR_WEIGHT}"),
    "pointcloud_use_lidar_scale": "${POINTCLOUD_USE_LIDAR_SCALE}" == "true",
    "pointcloud_min_lidar_scale": float("${POINTCLOUD_MIN_LIDAR_SCALE}"),
    "pointcloud_wait_queue_max_size": int("${POINTCLOUD_WAIT_QUEUE_MAX_SIZE}"),
    "enable_lidar_point_deskew": "${ENABLE_LIDAR_POINT_DESKEW}" == "true",
    "lidar_max_abs_point_time_offset_s": float("${LIDAR_MAX_ABS_POINT_TIME_OFFSET_S}"),
    "lidar_max_deskew_delta_m": float("${LIDAR_MAX_DESKEW_DELTA_M}"),
    "enable_lidar_pose_prior_factor": "${ENABLE_LIDAR_POSE_PRIOR_FACTOR}" == "true",
    "lidar_pose_prior_position_weight": float("${LIDAR_POSE_PRIOR_POSITION_WEIGHT}"),
    "lidar_pose_prior_velocity_weight": float("${LIDAR_POSE_PRIOR_VELOCITY_WEIGHT}"),
    "lidar_pose_prior_angular_velocity_weight": float("${LIDAR_POSE_PRIOR_ANGULAR_VELOCITY_WEIGHT}"),
    "lidar_pose_prior_orientation_weight": float("${LIDAR_POSE_PRIOR_ORIENTATION_WEIGHT}"),
    "lidar_pose_prior_position_huber_delta_m": float("${LIDAR_POSE_PRIOR_POSITION_HUBER_DELTA_M}"),
    "lidar_pose_prior_velocity_huber_delta_mps": float("${LIDAR_POSE_PRIOR_VELOCITY_HUBER_DELTA_MPS}"),
    "lidar_pose_prior_angular_velocity_huber_delta_radps": float("${LIDAR_POSE_PRIOR_ANGULAR_VELOCITY_HUBER_DELTA_RADPS}"),
    "lidar_pose_prior_orientation_huber_delta_rad": float("${LIDAR_POSE_PRIOR_ORIENTATION_HUBER_DELTA_RAD}"),
    "lidar_pose_factor_keyframe_stride": int("${LIDAR_POSE_FACTOR_KEYFRAME_STRIDE}"),
    "lidar_pose_factor_min_points": int("${LIDAR_POSE_FACTOR_MIN_POINTS}"),
    "lidar_pose_factor_max_frame_points": int("${LIDAR_POSE_FACTOR_MAX_FRAME_POINTS}"),
    "lidar_pose_factor_max_map_points": int("${LIDAR_POSE_FACTOR_MAX_MAP_POINTS}"),
    "lidar_pose_factor_nearest_distance_m": float("${LIDAR_POSE_FACTOR_NEAREST_DISTANCE_M}"),
    "lidar_pose_factor_correction_gain": float("${LIDAR_POSE_FACTOR_CORRECTION_GAIN}"),
    "lidar_pose_factor_max_correction_m": float("${LIDAR_POSE_FACTOR_MAX_CORRECTION_M}"),
    "lidar_pose_factor_max_rotation_rad": float("${LIDAR_POSE_FACTOR_MAX_ROTATION_RAD}"),
    "lidar_pose_factor_robust_kernel_m": float("${LIDAR_POSE_FACTOR_ROBUST_KERNEL_M}"),
    "lidar_pose_factor_iterations": int("${LIDAR_POSE_FACTOR_ITERATIONS}"),
    "enable_lidar_scan_to_scan_prior": "${ENABLE_LIDAR_SCAN_TO_SCAN_PRIOR}" == "true",
    "lidar_scan_to_scan_position_weight": float("${LIDAR_SCAN_TO_SCAN_POSITION_WEIGHT}"),
    "lidar_scan_to_scan_velocity_weight": float("${LIDAR_SCAN_TO_SCAN_VELOCITY_WEIGHT}"),
    "lidar_scan_to_scan_angular_velocity_weight": float("${LIDAR_SCAN_TO_SCAN_ANGULAR_VELOCITY_WEIGHT}"),
    "lidar_scan_to_scan_position_huber_delta_m": float("${LIDAR_SCAN_TO_SCAN_POSITION_HUBER_DELTA_M}"),
    "lidar_scan_to_scan_velocity_huber_delta_mps": float("${LIDAR_SCAN_TO_SCAN_VELOCITY_HUBER_DELTA_MPS}"),
    "lidar_scan_to_scan_angular_velocity_huber_delta_radps": float("${LIDAR_SCAN_TO_SCAN_ANGULAR_VELOCITY_HUBER_DELTA_RADPS}"),
    "lidar_scan_to_scan_max_velocity_mps": float("${LIDAR_SCAN_TO_SCAN_MAX_VELOCITY_MPS}"),
    "lidar_scan_to_scan_max_angular_velocity_radps": float("${LIDAR_SCAN_TO_SCAN_MAX_ANGULAR_VELOCITY_RADPS}"),
    "lidar_scan_to_scan_relative_translation_gain": float("${LIDAR_SCAN_TO_SCAN_RELATIVE_TRANSLATION_GAIN}"),
    "lidar_scan_to_scan_orientation_weight": float("${LIDAR_SCAN_TO_SCAN_ORIENTATION_WEIGHT}"),
    "lidar_scan_to_scan_orientation_huber_delta_rad": float("${LIDAR_SCAN_TO_SCAN_ORIENTATION_HUBER_DELTA_RAD}"),
    "lidar_scan_to_scan_use_odometry_prediction": "${LIDAR_SCAN_TO_SCAN_USE_ODOMETRY_PREDICTION}" == "true",
    "lidar_scan_to_scan_use_point_to_plane_correction": "${LIDAR_SCAN_TO_SCAN_USE_POINT_TO_PLANE_CORRECTION}" == "true",
    "lidar_scan_to_scan_use_relative_pose_factor": "${LIDAR_SCAN_TO_SCAN_USE_RELATIVE_POSE_FACTOR}" == "true",
    "lidar_scan_to_scan_min_target_prediction_ratio": float("${LIDAR_SCAN_TO_SCAN_MIN_TARGET_PREDICTION_RATIO}"),
    "lidar_scan_to_scan_min_target_translation_m": float("${LIDAR_SCAN_TO_SCAN_MIN_TARGET_TRANSLATION_M}"),
    "lidar_scan_to_scan_use_prediction_on_small_target": "${LIDAR_SCAN_TO_SCAN_USE_PREDICTION_ON_SMALL_TARGET}" == "true",
    "lidar_scan_to_scan_skip_translation_priors_on_small_target": "${LIDAR_SCAN_TO_SCAN_SKIP_TRANSLATION_PRIORS_ON_SMALL_TARGET}" == "true",
    "lidar_scan_to_scan_dead_reckon_on_reject": "${LIDAR_SCAN_TO_SCAN_DEAD_RECKON_ON_REJECT}" == "true",
    "lidar_scan_to_scan_apply_pose_seed": "${LIDAR_SCAN_TO_SCAN_APPLY_POSE_SEED}" == "true",
    "lidar_scan_to_scan_pose_seed_position_gain": float("${LIDAR_SCAN_TO_SCAN_POSE_SEED_POSITION_GAIN}"),
    "lidar_scan_to_scan_pose_seed_rotation_gain": float("${LIDAR_SCAN_TO_SCAN_POSE_SEED_ROTATION_GAIN}"),
    "enable_lidar_plane_normal_factor": "${ENABLE_LIDAR_PLANE_NORMAL_FACTOR}" == "true",
    "lidar_plane_normal_factor_weight": float("${LIDAR_PLANE_NORMAL_FACTOR_WEIGHT}"),
    "lidar_plane_normal_huber_delta_rad": float("${LIDAR_PLANE_NORMAL_HUBER_DELTA_RAD}"),
    "enable_visual_rotation_prior": "${ENABLE_VISUAL_ROTATION_PRIOR}" == "true",
    "visual_rotation_prior_weight": float("${VISUAL_ROTATION_PRIOR_WEIGHT}"),
    "visual_rotation_prior_huber_delta_rad": float("${VISUAL_ROTATION_PRIOR_HUBER_DELTA_RAD}"),
    "visual_rotation_max_shift_px": int("${VISUAL_ROTATION_MAX_SHIFT_PX}"),
    "visual_rotation_min_pixels": int("${VISUAL_ROTATION_MIN_PIXELS}"),
    "visual_rotation_max_pixels": int("${VISUAL_ROTATION_MAX_PIXELS}"),
    "visual_rotation_frame_stride": int("${VISUAL_ROTATION_FRAME_STRIDE}"),
    "visual_rotation_max_dt_ns": int("${VISUAL_ROTATION_MAX_DT_NS}"),
    "visual_rotation_max_rmse": float("${VISUAL_ROTATION_MAX_RMSE}"),
    "visual_rotation_pixel_to_rad_scale": float("${VISUAL_ROTATION_PIXEL_TO_RAD_SCALE}"),
    "visual_rotation_sign": float("${VISUAL_ROTATION_SIGN}"),
    "camera_to_imu_rotation_xyzw": "${CAMERA_TO_IMU_ROTATION_XYZW}",
    "camera_to_imu_translation_m": "${CAMERA_TO_IMU_TRANSLATION_M}",
    "lidar_to_imu_rotation_xyzw": "${LIDAR_TO_IMU_ROTATION_XYZW}",
    "lidar_to_imu_translation_m": "${LIDAR_TO_IMU_TRANSLATION_M}",
    "enable_visual_se3_prior": "${ENABLE_VISUAL_SE3_PRIOR}" == "true",
    "visual_se3_position_weight": float("${VISUAL_SE3_POSITION_WEIGHT}"),
    "visual_se3_orientation_weight": float("${VISUAL_SE3_ORIENTATION_WEIGHT}"),
    "visual_se3_velocity_weight": float("${VISUAL_SE3_VELOCITY_WEIGHT}"),
    "visual_se3_huber_delta_m": float("${VISUAL_SE3_HUBER_DELTA_M}"),
    "visual_se3_huber_delta_rad": float("${VISUAL_SE3_HUBER_DELTA_RAD}"),
    "visual_se3_huber_delta_mps": float("${VISUAL_SE3_HUBER_DELTA_MPS}"),
    "visual_se3_max_samples": int("${VISUAL_SE3_MAX_SAMPLES}"),
    "visual_se3_min_samples": int("${VISUAL_SE3_MIN_SAMPLES}"),
    "visual_se3_min_gradient": float("${VISUAL_SE3_MIN_GRADIENT}"),
    "visual_se3_max_abs_residual": float("${VISUAL_SE3_MAX_ABS_RESIDUAL}"),
    "visual_se3_huber_delta_intensity": float("${VISUAL_SE3_HUBER_DELTA_INTENSITY}"),
    "visual_se3_min_depth_m": float("${VISUAL_SE3_MIN_DEPTH_M}"),
    "visual_se3_max_depth_m": float("${VISUAL_SE3_MAX_DEPTH_M}"),
    "visual_se3_depth_dilation_px": int("${VISUAL_SE3_DEPTH_DILATION_PX}"),
    "visual_se3_depth_cache_size": int("${VISUAL_SE3_DEPTH_CACHE_SIZE}"),
    "visual_se3_max_dt_ns": int("${VISUAL_SE3_MAX_DT_NS}"),
    "visual_se3_min_hessian_rank": int("${VISUAL_SE3_MIN_HESSIAN_RANK}"),
    "visual_se3_max_hessian_condition": float("${VISUAL_SE3_MAX_HESSIAN_CONDITION}"),
    "visual_se3_min_sample_inlier_ratio": float("${VISUAL_SE3_MIN_SAMPLE_INLIER_RATIO}"),
    "visual_se3_coverage_grid_cols": int("${VISUAL_SE3_COVERAGE_GRID_COLS}"),
    "visual_se3_coverage_grid_rows": int("${VISUAL_SE3_COVERAGE_GRID_ROWS}"),
    "visual_se3_min_coverage_tiles": int("${VISUAL_SE3_MIN_COVERAGE_TILES}"),
    "visual_se3_max_mean_abs_residual": float("${VISUAL_SE3_MAX_MEAN_ABS_RESIDUAL}"),
    "visual_se3_max_translation_step_m": float("${VISUAL_SE3_MAX_TRANSLATION_STEP_M}"),
    "visual_se3_max_rotation_step_rad": float("${VISUAL_SE3_MAX_ROTATION_STEP_RAD}"),
    "visual_se3_delta_sign": float("${VISUAL_SE3_DELTA_SIGN}"),
    "max_position_update_m": float("${MAX_POSITION_UPDATE_M}"),
    "max_rotation_update_rad": float("${MAX_ROTATION_UPDATE_RAD}"),
    "update_gate_edge_knot_margin": int("${UPDATE_GATE_EDGE_KNOT_MARGIN}"),
    "position_extrapolation_damping": float("${POSITION_EXTRAPOLATION_DAMPING}"),
    "step_period_seconds": float("${STEP_PERIOD_SECONDS}"),
    "pose_output_period_seconds": float("${POSE_OUTPUT_PERIOD_SECONDS}"),
    "time_offset_sweep_min": float("${TIME_OFFSET_SWEEP_MIN}"),
    "time_offset_sweep_max": float("${TIME_OFFSET_SWEEP_MAX}"),
    "time_offset_sweep_step": float("${TIME_OFFSET_SWEEP_STEP}"),
    "time_offset_sweep_min_matches": int("${TIME_OFFSET_SWEEP_MIN_MATCHES}"),
    "apply_position_update_on_rotation_reject": "${APPLY_POSITION_UPDATE_ON_ROTATION_REJECT}" == "true",
    "apply_limited_rotation_update": "${APPLY_LIMITED_ROTATION_UPDATE}" == "true",
    "apply_limited_position_update": "${APPLY_LIMITED_POSITION_UPDATE}" == "true",
    "scale_position_with_limited_rotation": "${SCALE_POSITION_WITH_LIMITED_ROTATION}" == "true",
    "lidar_scan_to_scan_store_corrected_pose": "${LIDAR_SCAN_TO_SCAN_STORE_CORRECTED_POSE}" == "true",
    "lidar_scan_to_scan_yaw_only_angular_velocity": "${LIDAR_SCAN_TO_SCAN_YAW_ONLY_ANGULAR_VELOCITY}" == "true",
    "persistent_map_update_requires_accepted_solve": "${PERSISTENT_MAP_UPDATE_REQUIRES_ACCEPTED_SOLVE}" == "true",
    "defer_persistent_plane_map_updates_until_solved": "${DEFER_PERSISTENT_PLANE_MAP_UPDATES_UNTIL_SOLVED}" == "true",
    "deferred_plane_map_update_max_queue": int("${DEFERRED_PLANE_MAP_UPDATE_MAX_QUEUE}"),
    "enable_gaussian_snapshot_lidar_factor": "${ENABLE_GAUSSIAN_SNAPSHOT_LIDAR_FACTOR}" == "true",
    "gaussian_map_topic": "${GAUSSIAN_MAP_TOPIC}",
    "gaussian_snapshot_lidar_factor_weight": float("${GAUSSIAN_SNAPSHOT_LIDAR_FACTOR_WEIGHT}"),
    "gaussian_snapshot_lidar_nearest_distance_m": float("${GAUSSIAN_SNAPSHOT_LIDAR_NEAREST_DISTANCE_M}"),
    "gaussian_snapshot_lidar_min_opacity": float("${GAUSSIAN_SNAPSHOT_LIDAR_MIN_OPACITY}"),
    "gaussian_snapshot_lidar_subsample_stride": int("${GAUSSIAN_SNAPSHOT_LIDAR_SUBSAMPLE_STRIDE}"),
    "gaussian_snapshot_map_subsample_stride": int("${GAUSSIAN_SNAPSHOT_MAP_SUBSAMPLE_STRIDE}"),
    "gaussian_snapshot_lidar_max_correspondences": int("${GAUSSIAN_SNAPSHOT_LIDAR_MAX_CORRESPONDENCES}"),
    "enable_external_odometry_position_factors": "${ENABLE_EXTERNAL_ODOMETRY_POSITION_FACTORS}" == "true",
    "enable_external_odometry_orientation_factors": "${ENABLE_EXTERNAL_ODOMETRY_ORIENTATION_FACTORS}" == "true",
    "external_odometry_position_factor_weight": float("${EXTERNAL_ODOMETRY_POSITION_FACTOR_WEIGHT}"),
    "external_odometry_position_factor_huber_delta_m": float("${EXTERNAL_ODOMETRY_POSITION_FACTOR_HUBER_DELTA_M}"),
    "external_odometry_orientation_factor_weight": float("${EXTERNAL_ODOMETRY_ORIENTATION_FACTOR_WEIGHT}"),
    "external_odometry_orientation_factor_huber_delta_rad": float("${EXTERNAL_ODOMETRY_ORIENTATION_FACTOR_HUBER_DELTA_RAD}"),
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
    "runtime_diagnostics": runtime_diagnostics,
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
if runtime_diagnostics:
    print("  runtime_diagnostics   :", runtime_diagnostics)
PY

echo "continuous_time_native_reference_parity OK"
