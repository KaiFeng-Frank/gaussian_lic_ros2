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
MIN_POSES=20
MIN_STATUS_SAMPLES=1
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
POST_BA_STEP_GUARD_MIN_LIDAR_CONFIDENCE=0.6
POST_BA_STEP_GUARD_MIN_VISUAL_INLIER_RATIO=0.85
POST_BA_STEP_GUARD_MAX_VISUAL_RESIDUAL=0.3
POST_BA_STEP_GUARD_MIN_VISUAL_COVERAGE_TILES=8
POST_BA_STEP_GUARD_REJECT_TO_PRE_BA_OVER_M=0.0
POST_BA_STEP_GUARD_PRE_BA_AGREEMENT_MAX_POSE_STEP_M=0.0
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
SLIDING_WINDOW_IMU_WEIGHT=1.0
SLIDING_WINDOW_IMU_ROTATION_WEIGHT=1.0
SLIDING_WINDOW_IMU_VELOCITY_WEIGHT=1.0
SLIDING_WINDOW_IMU_POSITION_WEIGHT=1.0
SLIDING_WINDOW_IMU_VELOCITY_PRIOR_WEIGHT=0.0
SLIDING_WINDOW_BIAS_WEIGHT=1.0
SLIDING_WINDOW_POSE_TRANSLATION_WEIGHT=2.0
SLIDING_WINDOW_POSE_ROTATION_WEIGHT=2.0
SLIDING_WINDOW_SMOOTHNESS_ROTATION_WEIGHT=0.1
SLIDING_WINDOW_SMOOTHNESS_POSITION_WEIGHT=0.1
SLIDING_WINDOW_SMOOTHNESS_VELOCITY_WEIGHT=0.1
SLIDING_WINDOW_SMOOTHNESS_POSITION_VELOCITY_WEIGHT=0.0
SLIDING_WINDOW_SMOOTHNESS_BIAS_WEIGHT=0.1
ENABLE_SLIDING_WINDOW_RELATIVE_TRANSLATION_FACTOR=false
SLIDING_WINDOW_RELATIVE_TRANSLATION_WEIGHT=0.0
SLIDING_WINDOW_RELATIVE_TRANSLATION_HUBER_DELTA_M=0.1
ENABLE_SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_FACTOR=false
SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_WEIGHT=0.0
SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_HUBER_DELTA_M=0.15
SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_MIN_DT_S=0.45
SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_MAX_DT_S=1.05
SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_MAX_FACTORS=1
REQUIRE_BA_FEEDBACK=false
REQUIRE_NONDEGENERATE_BA=false
REQUIRE_DESKEW=false
ENABLE_VISUAL_FACTORS=false
ENABLE_MAPPER_FEEDBACK=false
MAPPER_FEEDBACK_SYNC_TOLERANCE_SEC=0.05
ENABLE_GAUSSIAN_MAP_FEEDBACK=false
REQUIRE_GAUSSIAN_SNAPSHOT=false
MAPPER_FEEDBACK_RENDER_MODE="${MAPPER_FEEDBACK_RENDER_MODE:-debug_input}"
MAPPER_FEEDBACK_RENDER_MODE_EXPLICIT=false
MAPPER_FEEDBACK_POINTCLOUD_COORDINATES=world
MAPPER_FEEDBACK_POINTCLOUD_COORDINATES_EXPLICIT=false
MAPPER_FEEDBACK_MAX_DEPTH=20.0
MAPPER_FEEDBACK_MAX_DEPTH_EXPLICIT=false
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
VISUAL_FACTOR_MAX_DT_NS=300000000
VISUAL_DEPTH_MAX_DT_NS=0
VISUAL_DEPTH_FRAME_CACHE_SIZE=64
VISUAL_DEPTH_DILATION_PX=5
VISUAL_PENDING_FACTOR_QUEUE_SIZE=128
VISUAL_ALIGNMENT_WINDOW_WEIGHT=1.0
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
ENABLE_EXTERNAL_ODOMETRY_PRIOR=false
REFERENCE_ODOMETRY_TOPIC="/gaussian_lic/frontend/input_odometry"
REFERENCE_POSE_TOPIC=""
REFERENCE_TUM_PATH=""
REQUIRE_REFERENCE_TRAJECTORY=false
MIN_REFERENCE_POSES=10
REFERENCE_TRAJECTORY_ALIGN=first
REFERENCE_MAX_ASSOCIATION_DT=0.2
REFERENCE_MIN_COVERAGE=0.4
REFERENCE_MAX_RMSE_M=2.0
REFERENCE_MAX_MEAN_M=1.5
REFERENCE_MAX_ERROR_M=5.0
REFERENCE_MAX_PATH_DRIFT=0.75
REFERENCE_ERROR_BIN_COUNT=8
REFERENCE_TIME_OFFSET_SWEEP_MIN=-0.5
REFERENCE_TIME_OFFSET_SWEEP_MAX=0.5
REFERENCE_TIME_OFFSET_SWEEP_STEP=0.1
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
  --playback-start-offset SEC   Start this many seconds into the bag. Default: 0.0.
  --rate RATE                  rosbag2 playback rate. Default: 1.0; Gaussian-map feedback uses 0.5 unless explicitly overridden.
  --clock-topics-all           Publish /clock immediately before each replayed message instead of using periodic --clock.
  --timeout SEC                Topic/report timeout. Default: 30.
  --post-play-settle SEC       Time to drain tracking callbacks after rosbag play exits. Default: 8.
  --min-poses N                Minimum recorded frontend odometry poses. Default: 20.
  --min-status-samples N       Minimum TrackingStatus samples. Default: 1.
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
  --sliding-window-multihop-relative-translation-min-dt-s SEC
                               Minimum span for the multi-hop relative translation factor. Default: 0.45.
  --sliding-window-multihop-relative-translation-max-dt-s SEC
                               Maximum span for the multi-hop relative translation factor. Default: 1.05.
  --sliding-window-multihop-relative-translation-max-factors N
                               Max multi-hop relative translation factors inserted per frame. Default: 1.
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
  --sliding-window-bias-weight W
                               IMU bias random-walk residual weight. Default: 1.0.
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
  --enable-sliding-window-relative-translation-factor
                               Add an internal adjacent-pose relative translation prior from raw IMU/pre-LIO propagation. Default: disabled.
  --sliding-window-relative-translation-weight W
                               Relative translation prior weight. Default: 0.0 disabled.
  --sliding-window-relative-translation-huber-delta-m M
                               Relative translation prior Huber delta. Default: 0.1.
  --require-ba-feedback        Require accepted sliding-window feedback.
  --require-nondegenerate-ba   Require the last reported BA normal equation and state cadence to be non-degenerate.
  --enable-visual-factors      Require mapper-rendered-image visual factors to be present externally.
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
  --visual-factor-max-dt-ns NS Max nearest-stamp delta for rendered/observed visual BA pairing. Default: 300000000.
  --visual-depth-max-dt-ns NS  Max nearest-stamp delta for sparse LiDAR depth selected by SE3 visual BA. Default: 0, follow --visual-factor-max-dt-ns.
  --visual-depth-dilation-px N Sparse LiDAR depth projection dilation radius for SE3 visual BA. Default: 5.
  --visual-alignment-window-weight W
                               Sliding-window 2D visual alignment factor weight. Default: 1.0.
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
  --reference-max-rmse-m M     Max native-vs-reference translation RMSE. Default: 2.0.
  --reference-max-mean-m M     Max native-vs-reference translation mean error. Default: 1.5.
  --reference-max-error-m M    Max native-vs-reference translation max error. Default: 5.0.
  --reference-max-path-drift R Max relative path-length drift. Default: 0.75.
  --reference-error-bin-count N
                               Number of time-ordered trajectory error bins archived in the reference comparison. Default: 8.
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
    --sliding-window-bias-weight)
      SLIDING_WINDOW_BIAS_WEIGHT="$2"
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
        PLAYBACK_RATE=0.5
      fi
      LIDAR_MAX_FRAME_POINTS=500
      TRACKING_MAX_POSE_STEP_M=0.025
      SLIDING_WINDOW_OPTIMIZE_EVERY_N_FRAMES=4
      SLIDING_WINDOW_MAX_FEEDBACK_TRANSLATION_M=0.05
      SLIDING_WINDOW_MAX_FEEDBACK_VELOCITY_MPS=0.5
      LIDAR_WINDOW_PLANE_FACTOR_WEIGHT=4.0
      ENABLE_SLIDING_WINDOW_RELATIVE_TRANSLATION_FACTOR=true
      SLIDING_WINDOW_RELATIVE_TRANSLATION_WEIGHT=0.1
      SLIDING_WINDOW_RELATIVE_TRANSLATION_HUBER_DELTA_M=0.1
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
    --visual-alignment-window-weight)
      VISUAL_ALIGNMENT_WINDOW_WEIGHT="$2"
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
    --reference-error-bin-count)
      REFERENCE_ERROR_BIN_COUNT="$2"
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
  visual_factor_max_dt_ns:="${VISUAL_FACTOR_MAX_DT_NS}" \
  visual_depth_max_dt_ns:="${VISUAL_DEPTH_MAX_DT_NS}" \
  depth_frame_cache_size:="${VISUAL_DEPTH_FRAME_CACHE_SIZE}" \
  sparse_lidar_depth_dilation_px:="${VISUAL_DEPTH_DILATION_PX}" \
  visual_alignment_window_weight:="${VISUAL_ALIGNMENT_WINDOW_WEIGHT}" \
  se3_photometric_window_weight:="${SE3_PHOTOMETRIC_WINDOW_WEIGHT}" \
  rendered_frame_cache_size:=64 \
  observed_frame_cache_size:=128 \
  visual_pending_factor_queue_size:="${VISUAL_PENDING_FACTOR_QUEUE_SIZE}" \
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
  post_ba_step_guard_min_lidar_confidence:="${POST_BA_STEP_GUARD_MIN_LIDAR_CONFIDENCE}" \
  post_ba_step_guard_min_visual_inlier_ratio:="${POST_BA_STEP_GUARD_MIN_VISUAL_INLIER_RATIO}" \
  post_ba_step_guard_max_visual_residual:="${POST_BA_STEP_GUARD_MAX_VISUAL_RESIDUAL}" \
  post_ba_step_guard_min_visual_coverage_tiles:="${POST_BA_STEP_GUARD_MIN_VISUAL_COVERAGE_TILES}" \
  post_ba_step_guard_reject_to_pre_ba_over_m:="${POST_BA_STEP_GUARD_REJECT_TO_PRE_BA_OVER_M}" \
  post_ba_step_guard_pre_ba_agreement_max_pose_step_m:="${POST_BA_STEP_GUARD_PRE_BA_AGREEMENT_MAX_POSE_STEP_M}" \
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
  sliding_window_imu_weight:="${SLIDING_WINDOW_IMU_WEIGHT}" \
  sliding_window_imu_rotation_weight:="${SLIDING_WINDOW_IMU_ROTATION_WEIGHT}" \
  sliding_window_imu_velocity_weight:="${SLIDING_WINDOW_IMU_VELOCITY_WEIGHT}" \
  sliding_window_imu_position_weight:="${SLIDING_WINDOW_IMU_POSITION_WEIGHT}" \
  sliding_window_imu_velocity_prior_weight:="${SLIDING_WINDOW_IMU_VELOCITY_PRIOR_WEIGHT}" \
  sliding_window_bias_weight:="${SLIDING_WINDOW_BIAS_WEIGHT}" \
  sliding_window_pose_translation_weight:="${SLIDING_WINDOW_POSE_TRANSLATION_WEIGHT}" \
  sliding_window_pose_rotation_weight:="${SLIDING_WINDOW_POSE_ROTATION_WEIGHT}" \
  sliding_window_smoothness_rotation_weight:="${SLIDING_WINDOW_SMOOTHNESS_ROTATION_WEIGHT}" \
  sliding_window_smoothness_position_weight:="${SLIDING_WINDOW_SMOOTHNESS_POSITION_WEIGHT}" \
  sliding_window_smoothness_velocity_weight:="${SLIDING_WINDOW_SMOOTHNESS_VELOCITY_WEIGHT}" \
  sliding_window_smoothness_position_velocity_weight:="${SLIDING_WINDOW_SMOOTHNESS_POSITION_VELOCITY_WEIGHT}" \
  sliding_window_smoothness_bias_weight:="${SLIDING_WINDOW_SMOOTHNESS_BIAS_WEIGHT}" \
  enable_sliding_window_relative_translation_factor:="${ENABLE_SLIDING_WINDOW_RELATIVE_TRANSLATION_FACTOR}" \
  sliding_window_relative_translation_weight:="${SLIDING_WINDOW_RELATIVE_TRANSLATION_WEIGHT}" \
  sliding_window_relative_translation_huber_delta_m:="${SLIDING_WINDOW_RELATIVE_TRANSLATION_HUBER_DELTA_M}" \
  enable_sliding_window_multihop_relative_translation_factor:="${ENABLE_SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_FACTOR}" \
  sliding_window_multihop_relative_translation_weight:="${SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_WEIGHT}" \
  sliding_window_multihop_relative_translation_huber_delta_m:="${SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_HUBER_DELTA_M}" \
  sliding_window_multihop_relative_translation_min_dt_s:="${SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_MIN_DT_S}" \
  sliding_window_multihop_relative_translation_max_dt_s:="${SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_MAX_DT_S}" \
  sliding_window_multihop_relative_translation_max_factors:="${SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_MAX_FACTORS}" \
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
    -p sync_tolerance_sec:="${MAPPER_FEEDBACK_SYNC_TOLERANCE_SEC}" \
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
stop_process_group "${record_pid}" TERM
unset record_pid

stop_process_group "${launch_pid}" TERM
unset launch_pid

set +e
IMU_LINEAR_ACCELERATION_SCALE_REPORT="${IMU_LINEAR_ACCELERATION_SCALE}" \
PLAYBACK_RATE_REPORT="${PLAYBACK_RATE}" \
PLAYBACK_START_OFFSET_REPORT="${PLAYBACK_START_OFFSET}" \
PLAYBACK_CLOCK_TOPICS_ALL_REPORT="${PLAYBACK_CLOCK_TOPICS_ALL}" \
MAX_LIDAR_INVALID_FRAMES_REPORT="${MAX_LIDAR_INVALID_FRAMES}" \
MAPPER_FEEDBACK_POINTCLOUD_COORDINATES_REPORT="${MAPPER_FEEDBACK_POINTCLOUD_COORDINATES}" \
MAPPER_FEEDBACK_MAX_DEPTH_REPORT="${MAPPER_FEEDBACK_MAX_DEPTH}" \
MAPPER_FEEDBACK_REQUIRE_PROJECTED_POINT_COLOR_REPORT="${MAPPER_FEEDBACK_REQUIRE_PROJECTED_POINT_COLOR}" \
MAPPER_FEEDBACK_ZBUFFER_PROJECTED_POINTS_REPORT="${MAPPER_FEEDBACK_ZBUFFER_PROJECTED_POINTS}" \
MAPPER_FEEDBACK_LR_REPORT="${MAPPER_FEEDBACK_POSITION_LR},${MAPPER_FEEDBACK_FEATURE_LR},${MAPPER_FEEDBACK_OPACITY_LR},${MAPPER_FEEDBACK_SCALING_LR},${MAPPER_FEEDBACK_ROTATION_LR}" \
MAPPER_FEEDBACK_OPTIMIZATION_EVERY_REPORT="${MAPPER_FEEDBACK_TORCH_OPTIMIZATION_EVERY_N_KEYFRAMES}" \
MAPPER_FEEDBACK_SELECT_EVERY_REPORT="${MAPPER_FEEDBACK_SELECT_EVERY_K_FRAME}" \
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
ENABLE_PRE_LIO_TRACKING_STEP_GUARD_REPORT="${ENABLE_PRE_LIO_TRACKING_STEP_GUARD}" \
ENABLE_POST_BA_TRACKING_STEP_GUARD_REPORT="${ENABLE_POST_BA_TRACKING_STEP_GUARD}" \
PRE_LIO_TRACKING_MAX_POSE_STEP_M_REPORT="${PRE_LIO_TRACKING_MAX_POSE_STEP_M}" \
POST_BA_TRACKING_MAX_POSE_STEP_M_REPORT="${POST_BA_TRACKING_MAX_POSE_STEP_M}" \
POST_BA_STEP_GUARD_CONFIDENCE_MAX_POSE_STEP_M_REPORT="${POST_BA_STEP_GUARD_CONFIDENCE_MAX_POSE_STEP_M}" \
POST_BA_STEP_GUARD_MIN_LIDAR_CONFIDENCE_REPORT="${POST_BA_STEP_GUARD_MIN_LIDAR_CONFIDENCE}" \
POST_BA_STEP_GUARD_MIN_VISUAL_INLIER_RATIO_REPORT="${POST_BA_STEP_GUARD_MIN_VISUAL_INLIER_RATIO}" \
POST_BA_STEP_GUARD_MAX_VISUAL_RESIDUAL_REPORT="${POST_BA_STEP_GUARD_MAX_VISUAL_RESIDUAL}" \
POST_BA_STEP_GUARD_MIN_VISUAL_COVERAGE_TILES_REPORT="${POST_BA_STEP_GUARD_MIN_VISUAL_COVERAGE_TILES}" \
POST_BA_STEP_GUARD_REJECT_TO_PRE_BA_OVER_M_REPORT="${POST_BA_STEP_GUARD_REJECT_TO_PRE_BA_OVER_M}" \
POST_BA_STEP_GUARD_PRE_BA_AGREEMENT_MAX_POSE_STEP_M_REPORT="${POST_BA_STEP_GUARD_PRE_BA_AGREEMENT_MAX_POSE_STEP_M}" \
POST_BA_STEP_GUARD_PRE_BA_AGREEMENT_MIN_COSINE_REPORT="${POST_BA_STEP_GUARD_PRE_BA_AGREEMENT_MIN_COSINE}" \
POST_BA_STEP_GUARD_PRE_BA_AGREEMENT_MAX_DELTA_M_REPORT="${POST_BA_STEP_GUARD_PRE_BA_AGREEMENT_MAX_DELTA_M}" \
POST_BA_STEP_GUARD_PRE_BA_AGREEMENT_MARGIN_M_REPORT="${POST_BA_STEP_GUARD_PRE_BA_AGREEMENT_MARGIN_M}" \
POST_BA_STEP_GUARD_PRE_BA_BLEND_ON_CLAMP_REPORT="${POST_BA_STEP_GUARD_PRE_BA_BLEND_ON_CLAMP}" \
PRE_LIO_TRACKING_STEP_GUARD_VELOCITY_SCALE_REPORT="${PRE_LIO_TRACKING_STEP_GUARD_VELOCITY_SCALE}" \
POST_BA_TRACKING_STEP_GUARD_VELOCITY_SCALE_REPORT="${POST_BA_TRACKING_STEP_GUARD_VELOCITY_SCALE}" \
SLIDING_WINDOW_SMOOTHNESS_POSITION_VELOCITY_WEIGHT_REPORT="${SLIDING_WINDOW_SMOOTHNESS_POSITION_VELOCITY_WEIGHT}" \
SLIDING_WINDOW_IMU_VELOCITY_PRIOR_WEIGHT_REPORT="${SLIDING_WINDOW_IMU_VELOCITY_PRIOR_WEIGHT}" \
ENABLE_SLIDING_WINDOW_RELATIVE_TRANSLATION_FACTOR_REPORT="${ENABLE_SLIDING_WINDOW_RELATIVE_TRANSLATION_FACTOR}" \
SLIDING_WINDOW_RELATIVE_TRANSLATION_WEIGHT_REPORT="${SLIDING_WINDOW_RELATIVE_TRANSLATION_WEIGHT}" \
SLIDING_WINDOW_RELATIVE_TRANSLATION_HUBER_DELTA_M_REPORT="${SLIDING_WINDOW_RELATIVE_TRANSLATION_HUBER_DELTA_M}" \
ENABLE_SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_FACTOR_REPORT="${ENABLE_SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_FACTOR}" \
SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_WEIGHT_REPORT="${SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_WEIGHT}" \
SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_HUBER_DELTA_M_REPORT="${SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_HUBER_DELTA_M}" \
SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_MIN_DT_S_REPORT="${SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_MIN_DT_S}" \
SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_MAX_DT_S_REPORT="${SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_MAX_DT_S}" \
SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_MAX_FACTORS_REPORT="${SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_MAX_FACTORS}" \
REFERENCE_ERROR_BIN_COUNT_REPORT="${REFERENCE_ERROR_BIN_COUNT}" \
REFERENCE_TIME_OFFSET_SWEEP_MIN_REPORT="${REFERENCE_TIME_OFFSET_SWEEP_MIN}" \
REFERENCE_TIME_OFFSET_SWEEP_MAX_REPORT="${REFERENCE_TIME_OFFSET_SWEEP_MAX}" \
REFERENCE_TIME_OFFSET_SWEEP_STEP_REPORT="${REFERENCE_TIME_OFFSET_SWEEP_STEP}" \
SE3_PHOTOMETRIC_MAX_SAMPLES_REPORT="${SE3_PHOTOMETRIC_MAX_SAMPLES}" \
SE3_PHOTOMETRIC_RANK_SAMPLES_BY_GRADIENT_REPORT="${SE3_PHOTOMETRIC_RANK_SAMPLES_BY_GRADIENT}" \
SE3_PHOTOMETRIC_USE_RENDERED_GRADIENT_REPORT="${SE3_PHOTOMETRIC_USE_RENDERED_GRADIENT}" \
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
mapper_feedback_sync_tolerance_sec = float(sys.argv[26])
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
sliding_window_imu_velocity_prior_weight = float(
    os.environ["SLIDING_WINDOW_IMU_VELOCITY_PRIOR_WEIGHT_REPORT"]
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
enable_sliding_window_multihop_relative_translation_factor = (
    os.environ["ENABLE_SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_FACTOR_REPORT"] == "true"
)
sliding_window_multihop_relative_translation_weight = float(
    os.environ["SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_WEIGHT_REPORT"]
)
sliding_window_multihop_relative_translation_huber_delta_m = float(
    os.environ["SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_HUBER_DELTA_M_REPORT"]
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
post_ba_step_guard_pre_ba_agreement_max_pose_step_m = float(
    os.environ["POST_BA_STEP_GUARD_PRE_BA_AGREEMENT_MAX_POSE_STEP_M_REPORT"]
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
reference_max_rmse_m = float(sys.argv[53])
reference_max_mean_m = float(sys.argv[54])
reference_max_error_m = float(sys.argv[55])
reference_max_path_drift = float(sys.argv[56])
reference_error_bin_count = int(os.environ["REFERENCE_ERROR_BIN_COUNT_REPORT"])
reference_time_offset_sweep_min = float(os.environ["REFERENCE_TIME_OFFSET_SWEEP_MIN_REPORT"])
reference_time_offset_sweep_max = float(os.environ["REFERENCE_TIME_OFFSET_SWEEP_MAX_REPORT"])
reference_time_offset_sweep_step = float(os.environ["REFERENCE_TIME_OFFSET_SWEEP_STEP_REPORT"])
has_external_reference_tum = reference_tum_path is not None and reference_tum_path.is_file() and reference_tum_path.stat().st_size > 0
imu_linear_acceleration_scale = float(os.environ["IMU_LINEAR_ACCELERATION_SCALE_REPORT"])
playback_rate = float(os.environ["PLAYBACK_RATE_REPORT"])
playback_start_offset = float(os.environ["PLAYBACK_START_OFFSET_REPORT"])
playback_clock_topics_all = os.environ["PLAYBACK_CLOCK_TOPICS_ALL_REPORT"].lower() == "true"
max_lidar_invalid_frames = int(os.environ["MAX_LIDAR_INVALID_FRAMES_REPORT"])


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
mapping_last = metrics.get("mapping_status", {}).get("last") or {}
errors = []

if metrics.get("trajectory_poses", 0) < min_poses:
    errors.append(f"trajectory poses {metrics.get('trajectory_poses', 0)} < {min_poses}")
if status.get("samples", 0) < min_status:
    errors.append(f"tracking status samples {status.get('samples', 0)} < {min_status}")
if topic_counts.get("/points_for_gs", 0) < min_point_frames:
    errors.append(f"/points_for_gs frames {topic_counts.get('/points_for_gs', 0)} < {min_point_frames}")
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
    "sliding_window_numeric_jacobian_blocks",
    "sliding_window_numeric_jacobian_columns",
    "sliding_window_orphan_factors",
):
    if int(last.get(key, 0)) != 0:
        errors.append(f"{key} is {last.get(key)}")

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
if enable_visual_factors:
    for key in (
        "visual_alignment_pending_queue_size",
        "visual_se3_photometric_pending_queue_size",
        "visual_alignment_pending_stale_drops",
        "visual_se3_photometric_pending_stale_drops",
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
    for key in ("visual_alignment_pending_queue_size", "visual_se3_photometric_pending_queue_size"):
        if int(last.get(key, 0)) > visual_pending_factor_queue_size:
            errors.append(f"{key} exceeds report queue budget: {last.get(key)}")
    for key in ("visual_alignment_pending_stale_drops", "visual_se3_photometric_pending_stale_drops"):
        if int(last.get(key, 0)) != 0:
            errors.append(f"{key} is {last.get(key)}")
    if int(last.get("visual_se3_photometric_total_batches", 0)) <= 0:
        errors.append("visual_se3_photometric_total_batches is zero")
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
    "gate_config": {
        "visual_factor_max_dt_ns": visual_factor_max_dt_ns,
        "visual_depth_max_dt_ns": visual_depth_max_dt_ns,
        "visual_depth_dilation_px": visual_depth_dilation_px,
        "visual_alignment_window_weight": visual_alignment_window_weight,
        "se3_photometric_window_weight": se3_photometric_window_weight,
        "se3_photometric_max_samples": se3_photometric_max_samples,
        "mapper_feedback_sync_tolerance_sec": mapper_feedback_sync_tolerance_sec,
        "visual_pending_factor_queue_size": visual_pending_factor_queue_size,
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
        "reference_tum_path": str(reference_tum_path) if reference_tum_path else "",
        "external_reference_tum_poses": external_reference_pose_count,
        "reference_trajectory_align": reference_trajectory_align,
        "reference_max_association_dt": reference_max_association_dt,
        "reference_min_coverage": reference_min_coverage,
        "reference_max_rmse_m": reference_max_rmse_m,
        "reference_max_mean_m": reference_max_mean_m,
        "reference_max_error_m": reference_max_error_m,
        "reference_max_path_drift": reference_max_path_drift,
        "reference_error_bin_count": reference_error_bin_count,
        "reference_time_offset_sweep": {
            "min": reference_time_offset_sweep_min,
            "max": reference_time_offset_sweep_max,
            "step": reference_time_offset_sweep_step,
        },
        "playback_rate": playback_rate,
        "playback_start_offset": playback_start_offset,
        "playback_clock_topics_all": playback_clock_topics_all,
        "imu_linear_acceleration_scale": imu_linear_acceleration_scale,
        "max_lidar_invalid_frames": max_lidar_invalid_frames,
        "lidar_pose_factor_iterations": lidar_pose_factor_iterations,
        "lidar_window_point_factor_weight": lidar_window_point_factor_weight,
        "lidar_window_plane_factor_weight": lidar_window_plane_factor_weight,
        "lidar_window_confidence_power": lidar_window_confidence_power,
        "sliding_window_max_states": sliding_window_max_states,
        "sliding_window_max_iterations": sliding_window_max_iterations,
        "sliding_window_marginalization_prior_weight": (
            sliding_window_marginalization_prior_weight
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
        "sliding_window_smoothness_position_velocity_weight": (
            sliding_window_smoothness_position_velocity_weight
        ),
        "sliding_window_imu_velocity_prior_weight": sliding_window_imu_velocity_prior_weight,
        "enable_sliding_window_relative_translation_factor": (
            enable_sliding_window_relative_translation_factor
        ),
        "sliding_window_relative_translation_weight": (
            sliding_window_relative_translation_weight
        ),
        "sliding_window_relative_translation_huber_delta_m": (
            sliding_window_relative_translation_huber_delta_m
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
        "sliding_window_multihop_relative_translation_min_dt_s": (
            sliding_window_multihop_relative_translation_min_dt_s
        ),
        "sliding_window_multihop_relative_translation_max_dt_s": (
            sliding_window_multihop_relative_translation_max_dt_s
        ),
        "sliding_window_multihop_relative_translation_max_factors": (
            sliding_window_multihop_relative_translation_max_factors
        ),
        "tracking_max_pose_step_m": tracking_max_pose_step_m,
        "enable_pre_lio_tracking_step_guard": enable_pre_lio_tracking_step_guard,
        "enable_post_ba_tracking_step_guard": enable_post_ba_tracking_step_guard,
        "pre_lio_tracking_max_pose_step_m": pre_lio_tracking_max_pose_step_m,
        "post_ba_tracking_max_pose_step_m": post_ba_tracking_max_pose_step_m,
        "post_ba_step_guard_confidence_max_pose_step_m": (
            post_ba_step_guard_confidence_max_pose_step_m
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
    --max-rmse-m "${REFERENCE_MAX_RMSE_M}"
    --max-mean-m "${REFERENCE_MAX_MEAN_M}"
    --max-error-m "${REFERENCE_MAX_ERROR_M}"
    --max-path-drift "${REFERENCE_MAX_PATH_DRIFT}"
  )
  if [[ "${REFERENCE_ERROR_BIN_COUNT}" != "0" ]]; then
    compare_cmd+=(--error-bin-count "${REFERENCE_ERROR_BIN_COUNT}")
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

if [[ "${report_status}" -ne 0 ]]; then
  echo "[native-tracking] health report failed; report kept at ${REPORT_JSON}" >&2
  exit "${report_status}"
fi

echo "[native-tracking] report: ${REPORT_JSON}"
