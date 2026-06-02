#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
#
# ============================================================================
# ITERATIVE MAP + TRAJECTORY CO-OPTIMIZATION OUTER LOOP
# ============================================================================
# The genuine cm-research lever that option_a_outerloop.sh did NOT touch:
# RETRAIN the Gaussian map at the current CT trajectory each outer iteration,
# then render that SHARPER map back into the photometric CT replay. Hypothesis:
#   sharper map @ better poses -> tighter photometric residual -> better CT
#   trajectory -> retrain map again -> repeat -> break the fixed-map 0.283 m
#   floor toward cm.
#
# Per outer iteration:
#   (0) ANCHOR  the current CT trajectory (camera-mode; cam0 == identity). The
#       SAME anchored TUM drives BOTH the retrain and the render, so the rebuilt
#       map and the render share one frame.
#   (1) RETRAIN: build a fresh Gaussian map driven by the anchored CT trajectory
#       (NOT the live tracker, NOT IMU fallback). -> point_cloud.ply.
#   (2) RENDER : render the refined ply at the anchored trajectory -> grays.
#   (3) FEEDBACK: mux grays into a RenderedFeedback bag keyed by observed stamp.
#   (4) CT REPLAY: deterministic CT run consuming the feedback -> new TUM.
#   (5) MEASURE: trajectory_compare vs the cocolic native reference.
#
# ----------------------------------------------------------------------------
# MAP-RETRAIN MECHANISM (REAL in code; FIRST exercised by this script).
# ----------------------------------------------------------------------------
# There is NO offline "build map from a TUM" tool. The mapper
# (src/gaussian_lic_mapping/src/mapping_node.cpp) is a PURE LIVE ros node that
# trains Gaussians ONLY from synchronized live topics. It does NOT compute poses
# itself -- it CONSUMES the per-frame BODY pose off pose_topic (/pose_for_gs):
#   - mapping_node.cpp:79  pose_topic_ = declare_parameter("pose_topic","/pose_for_gs")
#   - mapping_node.cpp:307 pose_sub_  = create_subscription<PoseStamped>(pose_topic_,...)
#   - frame_data.cpp:485-487 composes the CAMERA world pose from the BODY pose:
#       q_wc = q_w_pose * q_pose_camera ; t_wc = t_w_pose + q_w_pose*p_pose_camera
#     -- the SAME camera<-IMU extrinsic the renderer re-applies
#     (gaussian_map_renderer.cpp:380-381), and the SAME numeric extrinsic
#     (run_native_tracking_bag_report.sh:77-78 == option_a_helper.py _CAM_IMU_*).
#   - save_map writes the trained ply: mapping_node.cpp:2238-2244
#       write_torch_gaussian_ply(<dir>/point_cloud.ply). Call pattern verbatim
#       from collect_current_results.sh:671.
#
# The mapper's pose source is EXTERNAL. In the live pipeline lic2_contract_adapter
# subscribes to /gaussian_lic/frontend/input_odometry (nav_msgs/Odometry,
# lic2_contract_adapter_node.cpp:198,382-388) and forwards pose.pose VERBATIM to
# /pose_for_gs (publish_frontend_pose, :984-993; NO transform). It also brings
# raw /livox/lidar into the camera frame (fastlivo2 profile, :748-757) and
# republishes /points_for_gs + synced /image_for_gs. run_bag.launch.py:637-671
# spawns ONLY mapping_node + lic2_contract_adapter (NO tracker), so feeding our
# CT TUM as /gaussian_lic/frontend/input_odometry makes the mapper build at OUR
# poses. The raw sensor bag (CBD_Building_01_frontend_raw: /camera/image,
# /livox/lidar, /imu, /camera/camera_info -- verified via ros2 bag info) supplies
# the visuals; the camera intrinsics it carries (fx 646.78472 etc.) match the
# renderer's hardcoded kFx.. (gaussian_map_renderer.cpp:45-50) exactly.
#
# FRAME ALIGNMENT (the key correctness condition):
#   * We feed the mapper the SAME anchored(TUM_k) (anchor-tum --mode camera ->
#     cam0 == identity) the renderer is then given, so map and render share one
#     world frame.
#   * The mapper MUST apply the SAME camera<-IMU extrinsic the renderer applies.
#     run_bag.launch.py does NOT expose camera_to_pose_* (or pointcloud_coordinates,
#     sync_tolerance_sec, select_every_k_frame, max_depth) as launch args -- those
#     live in the `config` YAML it layers FIRST (run_bag.launch.py:146,231;
#     config arg :280). So we generate a PATCHED config YAML per run with the
#     correct camera_to_pose extrinsic + sensor pointcloud + relaxed sync, and
#     pass it via config:=. (default.yaml leaves camera_to_pose [0,0,0] identity,
#     which would put the trained map in the WRONG frame and the render gate
#     would then catch it as visible~0.)
#
# THE GAP / OPEN RISK (why this is UNVALIDATED, not a turnkey reproduction):
#   * The on-disk CBD map (results/.../CBD_Building_01_current_round_no_opacity_
#     prune_probe/point_cloud.ply, the one option_a renders) was built from a
#     PRE-BAKED ros1 mapper-contract bag whose /pose_for_gs poses were
#     ZERO-translation IMU-orientation-only (frontend_raw_to_ros1_mapper_contract.py
#     make_pose:151 position=(0,0,0); offline_stdout.json path_length_m: 0.0).
#     NO run in this repo has ever driven the LIVE adapter from an EXTERNAL FULL
#     6DOF trajectory. This script is the FIRST exercise of that wiring.
#   * The mapper syncs /pose_for_gs to image stamps within sync_tolerance_sec.
#     Our CT TUM stamps ARE the observed image stamps at 10 Hz, so an odometry
#     bag keyed at those exact ns and replayed under one unified --clock SHOULD
#     align -- but the alignment yield is unverified until the first run. We
#     relax sync_tolerance_sec and DISABLE imu/identity pose fallback so OUR
#     poses are the ONLY pose source.
#   ==> Treat iter0 as a smoke test: inspect ${RUN_ROOT}/iter0/retrain/mapper.log
#       for nonzero "aligned=" / "Initialized Torch Gaussian map" before
#       trusting iter1+, and the render gate (frame-0 visible > MIN_VISIBLE)
#       will abort loudly if the retrained map is degenerate or mis-framed.
#
# HONEST EXPECTATION: this MAY still converge at ~0.28 m. If the floor is the
# CT photometric FORMULATION (SE3 prior weight, single-pass joint solve) rather
# than map sharpness, retraining the map will not reach cm and cm then requires
# the upstream FAST-LIVO2-parity reimplementation. DO NOT claim cm from this.
#
# THE CALLER runs this via background Bash. Do NOT run the GPU loop inline.
# ============================================================================

set -euo pipefail

# --------------------------------------------------------------------------
# Parameters (all overridable via env)
# --------------------------------------------------------------------------
WORKSPACE="${WORKSPACE:-/home/frank/gaussian_lic_ros2}"
SOURCE_SETUP="${SOURCE_SETUP:-/opt/ros/jazzy/setup.bash}"
PY312="${PY312:-/usr/bin/python3.12}"

RENDERER="${RENDERER:-${WORKSPACE}/build/gaussian_lic_mapping/gaussian_map_renderer}"
HELPER="${HELPER:-${WORKSPACE}/scripts/option_a_helper.py}"
PARITY="${PARITY:-${WORKSPACE}/scripts/continuous_time_native_reference_parity.sh}"
EVAL="${EVAL:-${WORKSPACE}/scripts/trajectory_compare.py}"
BASE_CONFIG="${BASE_CONFIG:-${WORKSPACE}/src/gaussian_lic_bringup/config/default.yaml}"

# Sensor bag = raw frontend (/camera/image, /livox/lidar, /imu, /camera/camera_info).
SOURCE_BAG="${SOURCE_BAG:-/home/frank/data/fast_livo/CBD_Building_01_frontend_raw}"
IMAGE_TOPIC="${IMAGE_TOPIC:-/camera/image}"

# Seed CT trajectory the FIRST retrain + first CT replay are built from
# (best CT traj from the in-loop-render outer loop, 0.283 m).
SEED_TUM="${SEED_TUM:-${WORKSPACE}/results/fastlivo2/option_a_outerloop_20260531_200310/iter0/ct/continuous_time_trajectory.tum}"
REFERENCE_TUM="${REFERENCE_TUM:-${WORKSPACE}/baseline/fastlivo2/CBD_Building_01/native_reference/cocolic_livo_reference_10hz.tum}"

RENDERED_FEEDBACK_TOPIC="${RENDERED_FEEDBACK_TOPIC:-/gaussian_lic/rendered_feedback}"

# --- retrain (map rebuild) knobs ---
# Camera<-IMU extrinsic VERBATIM from run_native_tracking_bag_report.sh:77-78
# (== the renderer's hardcoded cam<-IMU). Baked into the patched config so the
# mapper's trained camera frame matches the renderer.
CAMERA_TO_POSE_TRANSLATION_M="${CAMERA_TO_POSE_TRANSLATION_M:-[0.0673699, 0.0412418, 0.0764217]}"
CAMERA_TO_POSE_RPY_RAD="${CAMERA_TO_POSE_RPY_RAD:-[-1.5768568829, 0.0154178108, -1.5646936365]}"
MAPPER_SYNC_TOLERANCE_SEC="${MAPPER_SYNC_TOLERANCE_SEC:-0.06}"     # relaxed; default 0.01
MAPPER_SELECT_EVERY_K_FRAME="${MAPPER_SELECT_EVERY_K_FRAME:-1}"    # train on every aligned frame
MAPPER_OPT_STEPS="${MAPPER_OPT_STEPS:-100}"                        # producing run used 100
MAPPER_MAX_DEPTH="${MAPPER_MAX_DEPTH:-20.0}"
# Mapper pointcloud semantics for adapter-forwarded raw lidar (sensor-frame),
# mirroring run_native's Gaussian-feedback default; the adapter brings the lidar
# into the camera frame via the fastlivo2 profile.
MAPPER_POINTCLOUD_COORDINATES="${MAPPER_POINTCLOUD_COORDINATES:-sensor}"
# FIX (skipped_unprojected/0-gaussian blocker): the adapter MUST NOT pre-rotate
# lidar->camera. The fastlivo2 profile applies the ~90deg lidar->camera optical
# rotation; the mapper's pointcloud_coordinates=sensor THEN applies the
# camera<-IMU extrinsic (camera_to_pose_rpy ~[-pi/2,0,-pi/2]) a SECOND time, so
# points land behind the camera (z<=0 -> skipped_depth=1.5M) or out of frustum
# (skipped_unprojected=1M) -> 0 gaussians. lidar<-IMU rotation is identity
# (run_native:76), so RAW livox points are already in the IMU/body frame the
# mapper extrinsic expects -> profile=identity yields exactly ONE correct
# transform. (Verified producing-run trained 1.47M gaussians with a single
# world->camera transform; co-opt's double transform was the regression.)
ADAPTER_PROFILE="${ADAPTER_PROFILE:-identity}"
BAG_PLAY_RATE="${BAG_PLAY_RATE:-0.25}"                            # producing-run rate (play.log:3)
RETRAIN_SETTLE_SEC="${RETRAIN_SETTLE_SEC:-25}"                    # drain after playback before save_map
NODE_WARMUP_SEC="${NODE_WARMUP_SEC:-10}"
SAVE_MAP_SERVICE="${SAVE_MAP_SERVICE:-/gaussian_lic/save_map}"
SAVE_TIMEOUT_SEC="${SAVE_TIMEOUT_SEC:-900}"
INPUT_ODOM_TOPIC="${INPUT_ODOM_TOPIC:-/gaussian_lic/frontend/input_odometry}"

# --- CT replay / eval knobs (same contract as option_a_outerloop.sh) ---
PLAYBACK_DURATION="${PLAYBACK_DURATION:-full}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-58}"
MIN_VISIBLE="${MIN_VISIBLE:-1000}"
MAX_ASSOC_DT="${MAX_ASSOC_DT:-0.2}"
NUM_OUTER="${NUM_OUTER:-2}"

RUN_ROOT="${RUN_ROOT:-${WORKSPACE}/results/fastlivo2/cooptimize_outerloop_$(date +%Y%m%d_%H%M%S)}"

export ROS_DOMAIN_ID

# --------------------------------------------------------------------------
# Logging + fail-loud helpers
# --------------------------------------------------------------------------
log()  { echo "[coopt] $(date +%H:%M:%S) $*"; }
fail() { echo "[coopt] FATAL: $*" >&2; exit 1; }

require_file() { [ -f "$1" ] || fail "missing file: $1 ($2)"; }
require_dir()  { [ -d "$1" ] || fail "missing dir: $1 ($2)"; }
require_exec() { [ -x "$1" ] || fail "not executable: $1 ($2)"; }

# --------------------------------------------------------------------------
# Preflight
# --------------------------------------------------------------------------
log "preflight: verifying inputs"
require_exec "${RENDERER}"      "built renderer"
require_file "${HELPER}"        "python3.12 helper"
require_file "${PARITY}"        "CT parity script"
require_file "${EVAL}"          "trajectory_compare.py"
require_file "${BASE_CONFIG}"   "base mapper/adapter params"
require_file "${SEED_TUM}"      "seed CT TUM"
require_dir  "${SOURCE_BAG}"    "source rosbag2 (raw frontend sensors)"
require_file "${REFERENCE_TUM}" "native reference TUM"
require_file "${SOURCE_SETUP}"  "ROS setup"
require_file "${WORKSPACE}/install/setup.bash" "workspace install setup"
[ -x "${PY312}" ] || fail "python3.12 not found at ${PY312}"

# ROS setup scripts reference unbound vars; relax -u only around sourcing.
set +u
# shellcheck disable=SC1090
source "${SOURCE_SETUP}"
# shellcheck disable=SC1091
source "${WORKSPACE}/install/setup.bash"
set -u

mkdir -p "${RUN_ROOT}"
log "run root: ${RUN_ROOT}"

# --------------------------------------------------------------------------
# Build the PATCHED config YAML once. run_bag.launch.py layers `config` FIRST in
# both mapping_parameters and adapter_parameters, so values here take effect for
# params the launch does not otherwise override. We patch ONLY the mapper params
# that are load-bearing for frame/sync correctness and are NOT exposed as launch
# args; everything else stays at default.yaml. (Done in python3.12 -- no PyYAML
# assumption; we do a minimal, key-scoped in-place edit of the mapping_node and
# lic2_contract_adapter sections.)
# --------------------------------------------------------------------------
PATCHED_CONFIG="${RUN_ROOT}/cooptimize_mapper.yaml"
log "patching config -> ${PATCHED_CONFIG}"
"${PY312}" - \
    "${BASE_CONFIG}" "${PATCHED_CONFIG}" \
    "${CAMERA_TO_POSE_TRANSLATION_M}" "${CAMERA_TO_POSE_RPY_RAD}" \
    "${MAPPER_SYNC_TOLERANCE_SEC}" "${MAPPER_SELECT_EVERY_K_FRAME}" \
    "${MAPPER_MAX_DEPTH}" "${MAPPER_POINTCLOUD_COORDINATES}" \
    "${MAPPER_OPT_STEPS}" "${ADAPTER_PROFILE}" <<'PY' || fail "config patch failed"
import re, sys
base, out, cam_t, cam_rpy, sync_tol, sel_k, max_depth, pc_coord, opt_steps, adapter_profile = sys.argv[1:11]
text = open(base, encoding="utf-8").read()
lines = text.splitlines()

def section_bounds(name):
    # top-level key "<name>:" then indented "ros__parameters:"; section ends at
    # the next top-level key (a line that starts at column 0 and ends with ':').
    start = None
    for i, ln in enumerate(lines):
        if ln.rstrip() == f"{name}:":
            start = i
            break
    if start is None:
        raise SystemExit(f"section {name!r} not found in {base}")
    end = len(lines)
    for j in range(start + 1, len(lines)):
        if lines[j] and not lines[j][0].isspace() and lines[j].rstrip().endswith(":"):
            end = j
            break
    return start, end

def set_key(name, key, value):
    s, e = section_bounds(name)
    # 4-space indent under ros__parameters (matches default.yaml style).
    indent = "    "
    pat = re.compile(rf"^{indent}{re.escape(key)}\s*:")
    for i in range(s, e):
        if pat.match(lines[i]):
            lines[i] = f"{indent}{key}: {value}"
            return
    # not present -> insert right after the section's "ros__parameters:" line.
    for i in range(s, e):
        if lines[i].strip() == "ros__parameters:":
            lines.insert(i + 1, f"{indent}{key}: {value}")
            return
    raise SystemExit(f"could not place {key} in section {name}")

# mapping_node: extrinsic + sync + frame semantics + training depth/steps.
set_key("mapping_node", "camera_to_pose_translation_m", cam_t)
set_key("mapping_node", "camera_to_pose_rpy_rad", cam_rpy)
set_key("mapping_node", "sync_tolerance_sec", sync_tol)
set_key("mapping_node", "select_every_k_frame", sel_k)
set_key("mapping_node", "max_depth", max_depth)
set_key("mapping_node", "pointcloud_coordinates", pc_coord)
set_key("mapping_node", "require_depth_topic", "false")
set_key("mapping_node", "torch_gaussian_optimization_steps", opt_steps)
set_key("mapping_node", "render_mode", "off")
set_key("mapping_node", "publish_rendered_preview", "false")
# lic2_contract_adapter: NO lidar->camera pre-transform (mapper's sensor-coords
# camera<-IMU extrinsic does the single transform) + no synthesized poses.
set_key("lic2_contract_adapter", "pointcloud_transform_profile", adapter_profile)
set_key("lic2_contract_adapter", "transform_pointcloud_to_camera_frame", "false")
set_key("lic2_contract_adapter", "pointcloud_transform_rotation", "[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]")
set_key("lic2_contract_adapter", "pointcloud_transform_translation", "[0.0, 0.0, 0.0]")
set_key("lic2_contract_adapter", "identity_pose_fallback", "false")
set_key("lic2_contract_adapter", "imu_pose_fallback", "false")

open(out, "w", encoding="utf-8").write("\n".join(lines) + "\n")
print(f"[patch] wrote {out}")
PY
[ -s "${PATCHED_CONFIG}" ] || fail "patched config empty: ${PATCHED_CONFIG}"

# --------------------------------------------------------------------------
# One-time: observed /camera/image stamps from the source bag (renderer
# iterates over these AND the CT node matches them). Shared across iterations.
# --------------------------------------------------------------------------
STAMPS_JSON="${RUN_ROOT}/observed_stamps.json"
log "extract-stamps: ${IMAGE_TOPIC} from ${SOURCE_BAG}"
"${PY312}" "${HELPER}" extract-stamps "${SOURCE_BAG}" "${IMAGE_TOPIC}" "${STAMPS_JSON}" \
  || fail "extract-stamps failed"
[ -s "${STAMPS_JSON}" ] || fail "observed stamps json empty: ${STAMPS_JSON}"

# --------------------------------------------------------------------------
# kill helper: stop a setsid process group cleanly (TERM then KILL the tree).
# User preference: no zombie GPU processes.
# --------------------------------------------------------------------------
stop_group() {
  local pid="$1" sig="${2:-TERM}"
  [ -n "${pid}" ] || return 0
  kill "-${sig}" -- -"${pid}" 2>/dev/null || kill "-${sig}" "${pid}" 2>/dev/null || true
  for _ in $(seq 1 100); do
    kill -0 "${pid}" 2>/dev/null || { wait "${pid}" 2>/dev/null || true; return 0; }
    sleep 0.1
  done
  kill -KILL -- -"${pid}" 2>/dev/null || kill -KILL "${pid}" 2>/dev/null || true
  wait "${pid}" 2>/dev/null || true
}

# Belt-and-suspenders teardown so an aborted iteration leaves no idle GPU node.
LAUNCH_PID=""
cleanup() {
  stop_group "${LAUNCH_PID}" TERM; LAUNCH_PID=""
  pkill -9 -f 'gaussian_lic_mapping mapping_node'           2>/dev/null || true
  pkill -9 -f 'gaussian_lic_frontend lic2_contract_adapter' 2>/dev/null || true
}
trap cleanup EXIT

# --------------------------------------------------------------------------
# retrain_step: rebuild the Gaussian map driven by the CURRENT (anchored) CT
# trajectory. Echoes the produced point_cloud.ply path on stdout (last line).
#   $1 = iter tag
#   $2 = anchored CT TUM (camera-mode; cam0 == identity == the render frame)
# --------------------------------------------------------------------------
retrain_step() {
  local tag="$1"
  local anchored_tum="$2"
  local rdir="${RUN_ROOT}/${tag}/retrain"
  mkdir -p "${rdir}"
  require_file "${anchored_tum}" "${tag} anchored TUM for retrain"

  local odom_bag="${rdir}/odom_bag"
  local map_out_dir="${rdir}/saved_map"
  local map_ply="${map_out_dir}/point_cloud.ply"
  local launch_log="${rdir}/mapper.log"
  local play_log="${rdir}/play.log"
  local save_log="${rdir}/save_map.log"

  # 1. sim-time odometry bag from the anchored CT TUM (exact stamp_ns keys, so
  #    poses land at the same sim-time instants as /camera/image under --clock).
  rm -rf "${odom_bag}"
  log "${tag}/retrain: mux-odometry from ${anchored_tum##*/} -> ${odom_bag}"
  "${PY312}" "${HELPER}" mux-odometry \
    "${anchored_tum}" "${odom_bag}" \
    --topic "${INPUT_ODOM_TOPIC}" \
    || fail "${tag} mux-odometry failed"
  [ -d "${odom_bag}" ] || fail "${tag} odom bag not created"

  # 2. launch adapter + mapper (NO bag-play inside launch; we drive playback).
  #    stub_mode:=false + use_composition:=false is REQUIRED -- native_node_condition
  #    (run_bag.launch.py:258-259) only spawns the real mapping_node when BOTH are
  #    false (default stub_mode is true -> would launch smoke stubs, no mapper).
  #    The PATCHED config supplies camera_to_pose extrinsic + pointcloud_coordinates
  #    + sync_tolerance the launch cannot pass as args.
  rm -rf "${map_out_dir}"; mkdir -p "${map_out_dir}"
  log "${tag}/retrain: launching run_bag.launch.py (adapter+mapper, fastlivo2, ext odom)"
  setsid ros2 launch gaussian_lic_bringup run_bag.launch.py \
      config:="${PATCHED_CONFIG}" \
      stub_mode:=false \
      use_composition:=false \
      play_bag:=false \
      use_sim_time:=true \
      frontend_adapter:=true \
      adapter_imu_pose_fallback:=false \
      adapter_identity_pose_fallback:=false \
      adapter_pointcloud_transform_profile:="${ADAPTER_PROFILE}" \
      adapter_raw_pointcloud_topic:=/livox/lidar \
      require_depth_topic:=false \
      enable_torch_gaussian_init:=true \
      enable_torch_gaussian_extend:=true \
      enable_torch_gaussian_optimization:=true \
      enable_torch_gaussian_pruning:=true \
      enable_torch_gaussian_densification:=false \
      torch_gaussian_optimization_steps:="${MAPPER_OPT_STEPS}" \
      torch_gaussian_device:=cuda \
      rviz:=false \
      > "${launch_log}" 2>&1 &
  LAUNCH_PID=$!

  # Mapper + adapter must come up before playback starts.
  log "${tag}/retrain: warming up nodes ${NODE_WARMUP_SEC}s"
  local warm_deadline=$(( $(date +%s) + NODE_WARMUP_SEC ))
  while [ "$(date +%s)" -lt "${warm_deadline}" ]; do
    if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
      tail -n 60 "${launch_log}" >&2 || true
      stop_group "${LAUNCH_PID}" TERM; LAUNCH_PID=""
      fail "${tag} launch died during warmup (see ${launch_log})"
    fi
    sleep 1 || true
  done

  # 3. play sensor + odometry bags under ONE unified --clock. `-i` may be given
  #    more than once for multiple input bags (verified: ros2 bag play --help,
  #    rosbag2 0.26.9). Odometry primes the pose buffer alongside the sensors.
  log "${tag}/retrain: ros2 bag play sensor + odom (rate ${BAG_PLAY_RATE}, --clock)"
  if ! ros2 bag play \
        -i "${SOURCE_BAG}" \
        -i "${odom_bag}" \
        --clock --read-ahead-queue-size 100 --rate "${BAG_PLAY_RATE}" \
        > "${play_log}" 2>&1; then
    tail -n 40 "${play_log}" >&2 || true
    stop_group "${LAUNCH_PID}" TERM; LAUNCH_PID=""
    fail "${tag} bag play failed (see ${play_log})"
  fi

  # 4. drain mapping callbacks before snapshotting the map.
  log "${tag}/retrain: settling ${RETRAIN_SETTLE_SEC}s before save_map"
  for _ in $(seq 1 "${RETRAIN_SETTLE_SEC}"); do sleep 1 || true; done

  if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    tail -n 80 "${launch_log}" >&2 || true
    LAUNCH_PID=""
    fail "${tag} mapper died before save_map (see ${launch_log})"
  fi

  # 5. save_map -> point_cloud.ply (mapping_node.cpp:2238-2244).
  log "${tag}/retrain: calling ${SAVE_MAP_SERVICE} -> ${map_out_dir}"
  timeout "${SAVE_TIMEOUT_SEC}" \
    ros2 service call "${SAVE_MAP_SERVICE}" gaussian_lic_msgs/srv/SaveMap \
      "{path: '${map_out_dir}', include_skybox: false}" \
      > "${save_log}" 2>&1 || true

  stop_group "${LAUNCH_PID}" TERM; LAUNCH_PID=""

  if [ ! -s "${map_ply}" ]; then
    tail -n 60 "${launch_log}" >&2 || true
    tail -n 20 "${save_log}" >&2 || true
    fail "${tag} retrain produced no ${map_ply} (mapper trained no Gaussian map -- grep '${launch_log}' for 'aligned=' / 'Initialized Torch Gaussian map' / 'CUDA out of memory')"
  fi

  # Surface the alignment / gaussian-count signal so the caller can judge the
  # retrain quality (load-bearing for the UNVALIDATED mechanism).
  grep -oE 'aligned=[0-9]+' "${launch_log}" | tail -n1 >&2 || true
  grep -E 'Initialized Torch Gaussian map|Extended Torch Gaussian map' "${launch_log}" | tail -n1 >&2 || true
  log "${tag}/retrain: refined ply $(du -h "${map_ply}" | cut -f1) -> ${map_ply}"

  printf '%s\n' "${map_ply}"
}

# --------------------------------------------------------------------------
# render_step: render the (refined) map at a PRE-ANCHORED TUM, gate on frame-0
# visible count, mux a fresh RenderedFeedback bag. Reuses the SAME anchored TUM
# the retrain used (passed in) so map+render share one frame.
#   $1 = iter tag ; $2 = anchored TUM ; $3 = map ply to render
# Echoes the feedback bag path on stdout (last line).
# --------------------------------------------------------------------------
render_step() {
  local tag="$1" anchored_tum="$2" map_ply="$3"
  local idir="${RUN_ROOT}/${tag}"
  mkdir -p "${idir}"
  require_file "${anchored_tum}" "${tag} anchored TUM"
  require_file "${map_ply}" "${tag} map ply"

  local render_dir="${idir}/render"
  local render_log="${idir}/render.log"
  local render_stamps="${render_dir}/stamps.json"
  local feedback_bag="${idir}/feedback_bag"

  rm -rf "${render_dir}"; mkdir -p "${render_dir}"
  log "${tag}/render: rendering refined map at anchored CT trajectory (GPU)"
  if ! "${RENDERER}" "${map_ply}" "${anchored_tum}" "${STAMPS_JSON}" "${render_dir}" \
        > "${render_log}" 2>&1; then
    tail -n 40 "${render_log}" >&2 || true
    fail "${tag} renderer exited non-zero (see ${render_log})"
  fi
  [ -s "${render_stamps}" ] || fail "${tag} renderer produced no stamps.json"

  local visible0
  visible0="$(grep -oE 'visible=[0-9]+' "${render_log}" | head -n1 | cut -d= -f2 || true)"
  [ -n "${visible0}" ] || { tail -n 40 "${render_log}" >&2; fail "${tag} no visible= in render log"; }
  log "${tag}/render: frame-0 visible=${visible0} (gate > ${MIN_VISIBLE})"
  if [ "${visible0}" -le "${MIN_VISIBLE}" ]; then
    tail -n 40 "${render_log}" >&2 || true
    fail "${tag} frame-0 visible=${visible0} <= ${MIN_VISIBLE} -> retrained map degenerate / mis-framed, aborting"
  fi

  local png_count
  png_count="$(find "${render_dir}" -maxdepth 1 -name '*.png' | wc -l | tr -d ' ')"
  log "${tag}/render: ${png_count} PNGs"
  [ "${png_count}" -gt 0 ] || fail "${tag} no rendered PNGs"

  rm -rf "${feedback_bag}"
  log "${tag}/render: mux-feedback -> ${feedback_bag}"
  "${PY312}" "${HELPER}" mux-feedback "${render_dir}" "${render_stamps}" "${feedback_bag}" \
    || fail "${tag} mux-feedback failed"
  [ -d "${feedback_bag}" ] || fail "${tag} feedback bag not created"

  printf '%s\n' "${feedback_bag}"
}

# --------------------------------------------------------------------------
# ct_step: one deterministic render-photometric CT pass (identical contract to
# option_a_outerloop.sh ct_step).
#   $1 = iter tag ; $2 = CT_SEED_TUM (ORIGINAL body frame) ; $3 = feedback bag
# Echoes the produced TUM path on stdout (last line).
# --------------------------------------------------------------------------
ct_step() {
  local tag="$1" seed_tum="$2" feedback_bag="$3"
  local odir="${RUN_ROOT}/${tag}/ct"
  local ct_log="${RUN_ROOT}/${tag}/ct_run.log"
  mkdir -p "${odir}"
  require_file "${seed_tum}" "${tag} CT seed TUM"
  require_dir  "${feedback_bag}" "${tag} feedback bag"

  local out_tum="${odir}/continuous_time_trajectory.tum"
  rm -f "${out_tum}"

  log "${tag}/ct: continuous-time render-photometric run (deterministic replay)"
  # The parity script has a known NON-FATAL trailing post-processing step that
  # exits non-zero (ValueError on an empty field) AFTER the deterministic TUM is
  # captured; we do NOT treat its exit code as authoritative -- the out_tum with
  # >0 poses is the success criterion.
  env \
      WORKSPACE="${WORKSPACE}" \
      SOURCE_SETUP="${SOURCE_SETUP}" \
      ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" \
      ENABLE_RENDER_PHOTOMETRIC=true \
      ENABLE_VISUAL_SE3_PRIOR=true \
      ENABLE_VISUAL_MAP_PHOTOMETRIC=true \
      HOLD_GRAVITY_CONSTANT=false \
      CT_SEED_TUM="${seed_tum}" \
      DETERMINISTIC_BAG_PATH="${SOURCE_BAG}" \
      DETERMINISTIC_FEEDBACK_BAG_PATH="${feedback_bag}" \
      RENDERED_FEEDBACK_TOPIC="${RENDERED_FEEDBACK_TOPIC}" \
      REFERENCE_TUM="__skip__" \
      PLAYBACK_DURATION="${PLAYBACK_DURATION}" \
      OUTPUT_DIR="${odir}" \
      bash "${PARITY}" > "${ct_log}" 2>&1 || true

  [ -s "${out_tum}" ] || { tail -n 60 "${ct_log}" >&2; fail "${tag} CT produced no TUM (real failure)"; }
  local nlines
  nlines="$(grep -cv '^#' "${out_tum}" || echo 0)"
  log "${tag}/ct: ${nlines}-pose TUM -> ${out_tum}"
  [ "${nlines}" -gt 0 ] || fail "${tag} CT TUM has zero poses"

  printf '%s\n' "${out_tum}"
}

# --------------------------------------------------------------------------
# eval_step: trajectory_compare.py vs the native reference (identical to
# option_a_outerloop.sh eval_step).
# --------------------------------------------------------------------------
eval_step() {
  local tag="$1" cur_tum="$2"
  local report="${RUN_ROOT}/${tag}/trajectory_compare_${tag}.json"

  log "${tag}/eval: vs ${REFERENCE_TUM##*/} (align yaw, max-assoc-dt ${MAX_ASSOC_DT})"
  python3 "${EVAL}" \
    --baseline "${REFERENCE_TUM}" --current "${cur_tum}" \
    --align yaw --max-association-dt "${MAX_ASSOC_DT}" \
    --output "${report}" --min-matches 1 --min-coverage 0.0 \
    --max-rmse-m 1e9 --max-mean-m 1e9 --max-error-m 1e9 --max-path-drift 1e9 \
    --json > "${RUN_ROOT}/${tag}/eval_${tag}.stdout" 2>&1 || true
  [ -s "${report}" ] || fail "${tag} eval produced no report"

  "${PY312}" - "${report}" "${cur_tum}" "${tag}" <<'PY'
import json, sys
report_path, tum_path, tag = sys.argv[1], sys.argv[2], sys.argv[3]
r = json.load(open(report_path))
rmse = r.get("translation", {}).get("rmse_m")
mean = r.get("translation", {}).get("mean_m")
matched = r.get("matched_poses")
cov = r.get("coverage")
stamps = []
for line in open(tum_path):
    line = line.strip()
    if not line or line.startswith("#"):
        continue
    parts = line.split()
    if len(parts) == 8:
        stamps.append(float(parts[0]))
span = (max(stamps) - min(stamps)) if len(stamps) >= 2 else 0.0
print(f"[coopt] ===== {tag} RESULT =====")
print(f"[coopt]   translation.rmse_m = {rmse}")
print(f"[coopt]   translation.mean_m = {mean}")
print(f"[coopt]   matched_poses      = {matched}  coverage = {cov}")
print(f"[coopt]   current TUM span_s = {span:.3f}  poses = {len(stamps)}")
print(f"[coopt]   report             = {report_path}")
PY
}

# ==========================================================================
# OUTER LOOP
# ==========================================================================
log "=============================================================="
log "CO-OPTIMIZATION OUTER LOOP START (${NUM_OUTER} iterations)"
log "  seed TUM   : ${SEED_TUM}"
log "  source bag : ${SOURCE_BAG}"
log "  reference  : ${REFERENCE_TUM}"
log "  config     : ${PATCHED_CONFIG}"
log "  MECHANISM  : retrain map @ CT traj via adapter external-odometry."
log "  HONEST     : mechanism is REAL in code but UNVALIDATED here; may still"
log "               converge ~0.28 m. Inspect iter0 retrain alignment first."
log "=============================================================="

CUR_TUM="${SEED_TUM}"
for ((i=0; i<NUM_OUTER; i++)); do
  TAG="iter${i}"
  log "---- ${TAG} (current traj: ${CUR_TUM}) ----"

  # (0) Anchor the current CT trajectory (camera-mode) -> the SINGLE frame used
  #     by BOTH the retrain (mapper builds map here) AND the render. Doing this
  #     once and reusing it for both keeps map+render frame-consistent.
  ADIR="${RUN_ROOT}/${TAG}"
  mkdir -p "${ADIR}"
  ANCHORED="${ADIR}/anchored.tum"
  log "${TAG}: anchor-tum (camera-mode) -> ${ANCHORED}"
  "${PY312}" "${HELPER}" anchor-tum --mode camera "${CUR_TUM}" "${ANCHORED}" \
    || fail "${TAG} anchor-tum failed"
  [ -s "${ANCHORED}" ] || fail "${TAG} anchored TUM empty"

  # (1) RETRAIN the map at the anchored current trajectory.
  MAP_PLY="$(retrain_step "${TAG}" "${ANCHORED}" | tail -n1)"
  log "${TAG}: refined map -> ${MAP_PLY}"

  # (2)+(3) RENDER the refined map at the SAME anchored trajectory -> feedback.
  FB="$(render_step "${TAG}" "${ANCHORED}" "${MAP_PLY}" | tail -n1)"
  log "${TAG}: feedback bag -> ${FB}"

  # (4) CT REPLAY with the fresh feedback -> new trajectory. CT_SEED_TUM stays
  #     the ORIGINAL seed frame (the estimator's own body world); the feedback
  #     supplies the photometric prior in map frame.
  NEW_TUM="$(ct_step "${TAG}" "${SEED_TUM}" "${FB}" | tail -n1)"

  # (5) MEASURE.
  eval_step "${TAG}" "${NEW_TUM}"

  CUR_TUM="${NEW_TUM}"
done

log "=============================================================="
log "CO-OPTIMIZATION OUTER LOOP DONE"
log "  final TUM  : ${CUR_TUM}"
log "  artifacts  : ${RUN_ROOT}"
log "  REMINDER   : compare per-iter rmse; if it sits at ~0.28 m the floor is"
log "               the CT photometric formulation, not map sharpness -> cm"
log "               needs the upstream reimplementation, NOT more iterations."
log "=============================================================="
