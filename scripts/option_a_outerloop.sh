#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
#
# ============================================================================
# Option-A in-loop-render OUTER LOOP
# ============================================================================
# Re-render the on-disk 3DGS map at the warm-started CT trajectory, mux the
# rendered grays into a RenderedFeedback bag keyed by observed image stamp,
# feed that back into a fresh continuous-time run, and iterate. Two outer
# iterations:
#
#   iter0: render(map @ CT_SEED_TUM) -> feedback bag -> CT run -> TUM1
#   iter1: render(map @ TUM1)        -> feedback bag -> CT run -> TUM2
#
# HONEST EXPECTATION: this pipeline is SOTA-capped (visual photometric SE3
# prior nudges the LiDAR-inertial CT estimate toward the rendered map). The
# realistic floor is ~0.284 m translation RMSE vs the native reference, NOT
# centimetre accuracy. Do not expect the second iteration to "converge to cm".
#
# Touches NOTHING in src/gaussian_lic_tracking. Pure orchestration of the
# already-built renderer + the existing parity script + a python3.12 muxer.
#
# All facts (renderer CLI/output, msg fields, env interface, eval CLI) were
# verified by Read/grep of the source this session:
#   - gaussian_map_renderer.cpp main@287 : CLI <ply> <ct_tum> <stamps_json> <out_dir>
#       output  : <out_dir>/<ns>.png (BGR8) + <out_dir>/stamps.json (=[ns,...])
#       poses   : TUM treated as BODY/IMU world poses; extrinsic applied
#                 internally (lines 380-381). Logs "visible=<n>" at idx%100==0.
#   - RenderedFeedback.msg : node uses .image (decode_image_gray accepts mono8)
#                 + .observed_stamp (sec/nanosec -> ns key).
#   - continuous_time_node.cpp : CT_SEED_TUM read from ENV (@4488);
#       deterministic_feedback_bag_path keyed by observed_stamp (@1311-1326,2172);
#       deterministic replay writes ${OUTPUT_DIR}/continuous_time_trajectory.tum
#       and exits (parity script lines 527,535-543).
#   - trajectory_compare.py : --baseline --current --align yaw
#                 --max-association-dt 0.2 -> translation.rmse_m
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

MAP_PLY="${MAP_PLY:-${WORKSPACE}/results/fastlivo2/CBD_Building_01_current_round_no_opacity_prune_probe/point_cloud.ply}"
SEED_TUM="${SEED_TUM:-${WORKSPACE}/results/fastlivo2/CT_seed/continuous_time_trajectory.tum}"
SOURCE_BAG="${SOURCE_BAG:-/home/frank/data/fast_livo/CBD_Building_01_frontend_raw}"
IMAGE_TOPIC="${IMAGE_TOPIC:-/camera/image}"
REFERENCE_TUM="${REFERENCE_TUM:-${WORKSPACE}/baseline/fastlivo2/CBD_Building_01/native_reference/cocolic_livo_reference_10hz.tum}"
RENDERED_FEEDBACK_TOPIC="${RENDERED_FEEDBACK_TOPIC:-/gaussian_lic/rendered_feedback}"

# CT run knobs (the in-loop-render ingest contract from the task)
PLAYBACK_DURATION="${PLAYBACK_DURATION:-full}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-57}"
MIN_VISIBLE="${MIN_VISIBLE:-1000}"
MAX_ASSOC_DT="${MAX_ASSOC_DT:-0.2}"

RUN_ROOT="${RUN_ROOT:-${WORKSPACE}/results/fastlivo2/option_a_outerloop_$(date +%Y%m%d_%H%M%S)}"

export ROS_DOMAIN_ID

# --------------------------------------------------------------------------
# Logging + fail-loud helpers
# --------------------------------------------------------------------------
log()  { echo "[option_a] $(date +%H:%M:%S) $*"; }
fail() { echo "[option_a] FATAL: $*" >&2; exit 1; }

require_file() { [ -f "$1" ] || fail "missing file: $1 ($2)"; }
require_dir()  { [ -d "$1" ] || fail "missing dir: $1 ($2)"; }
require_exec() { [ -x "$1" ] || fail "not executable: $1 ($2)"; }

# --------------------------------------------------------------------------
# Preflight
# --------------------------------------------------------------------------
log "preflight: verifying inputs"
require_exec "${RENDERER}"   "built renderer"
require_file "${HELPER}"     "python3.12 helper"
require_file "${PARITY}"     "CT parity script"
require_file "${EVAL}"       "trajectory_compare.py"
require_file "${MAP_PLY}"    "3DGS map ply"
require_file "${SEED_TUM}"   "CT seed TUM"
require_dir  "${SOURCE_BAG}" "source rosbag2"
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
# One-time: extract observed /camera/image stamps from the source bag.
# These ns values are BOTH what the renderer iterates over AND what the CT
# node matches against (current.stamp_ns). Shared across both iterations.
# --------------------------------------------------------------------------
STAMPS_JSON="${RUN_ROOT}/observed_stamps.json"
if [ ! -s "${STAMPS_JSON}" ]; then
  log "extract-stamps: reading ${IMAGE_TOPIC} stamps from ${SOURCE_BAG}"
  "${PY312}" "${HELPER}" extract-stamps \
    "${SOURCE_BAG}" \
    "${IMAGE_TOPIC}" \
    "${STAMPS_JSON}" \
    || fail "extract-stamps failed"
else
  log "extract-stamps: reusing ${STAMPS_JSON}"
fi
[ -s "${STAMPS_JSON}" ] || fail "observed stamps json empty: ${STAMPS_JSON}"

# --------------------------------------------------------------------------
# render_step: pre-transform a body TUM (anchor to first pose), render the
# map at it, gate on frame-0 visible count, then mux a fresh feedback bag.
#   $1 = iter tag (e.g. iter0)
#   $2 = input CT body TUM to render at
# Echoes the path of the produced feedback bag on stdout (last line).
# --------------------------------------------------------------------------
render_step() {
  local tag="$1"
  local in_tum="$2"
  local idir="${RUN_ROOT}/${tag}"
  mkdir -p "${idir}"

  require_file "${in_tum}" "${tag} input TUM"

  local anchored_tum="${idir}/anchored.tum"
  local render_dir="${idir}/render"
  local render_log="${idir}/render.log"
  local render_stamps="${render_dir}/stamps.json"
  local feedback_bag="${idir}/feedback_bag"

  log "${tag}: anchor-tum (camera-mode, anchor to first pose) -> ${anchored_tum}"
  "${PY312}" "${HELPER}" anchor-tum \
    --mode camera \
    "${in_tum}" \
    "${anchored_tum}" \
    || fail "${tag} anchor-tum failed"
  [ -s "${anchored_tum}" ] || fail "${tag} anchored TUM empty"

  # Idempotent: wipe a stale render dir so visible-count gating is fresh.
  rm -rf "${render_dir}"
  mkdir -p "${render_dir}"

  log "${tag}: rendering map at anchored CT trajectory (this is the long GPU step)"
  if ! "${RENDERER}" "${MAP_PLY}" "${anchored_tum}" "${STAMPS_JSON}" "${render_dir}" \
        > "${render_log}" 2>&1; then
    tail -n 40 "${render_log}" >&2 || true
    fail "${tag} renderer exited non-zero (see ${render_log})"
  fi

  [ -s "${render_stamps}" ] || fail "${tag} renderer produced no stamps.json"

  # Gate on the FIRST logged visible count (frame-0). Renderer logs
  # "visible=<n>" at idx%100==0, so the first match is frame 0.
  local visible0
  visible0="$(grep -oE 'visible=[0-9]+' "${render_log}" | head -n1 | cut -d= -f2 || true)"
  [ -n "${visible0}" ] || { tail -n 40 "${render_log}" >&2; fail "${tag} no visible= in render log"; }
  log "${tag}: frame-0 visible=${visible0} (gate > ${MIN_VISIBLE})"
  if [ "${visible0}" -le "${MIN_VISIBLE}" ]; then
    tail -n 40 "${render_log}" >&2 || true
    fail "${tag} frame-0 visible=${visible0} <= ${MIN_VISIBLE} -> anchor/frame bridge wrong, aborting"
  fi

  local png_count
  png_count="$(find "${render_dir}" -maxdepth 1 -name '*.png' | wc -l | tr -d ' ')"
  log "${tag}: renderer wrote ${png_count} PNGs"
  [ "${png_count}" -gt 0 ] || fail "${tag} no rendered PNGs"

  # Idempotent: a fresh feedback bag per iteration (muxer refuses to overwrite).
  rm -rf "${feedback_bag}"
  log "${tag}: mux-feedback -> ${feedback_bag}"
  "${PY312}" "${HELPER}" mux-feedback \
    "${render_dir}" \
    "${render_stamps}" \
    "${feedback_bag}" \
    || fail "${tag} mux-feedback failed"
  [ -d "${feedback_bag}" ] || fail "${tag} feedback bag not created"

  printf '%s\n' "${feedback_bag}"
}

# --------------------------------------------------------------------------
# ct_step: run one render-photometric deterministic CT pass.
#   $1 = iter tag
#   $2 = CT_SEED_TUM (body TUM, ORIGINAL frame -- NOT anchored)
#   $3 = feedback bag path
# Echoes the produced TUM path on stdout (last line).
# --------------------------------------------------------------------------
ct_step() {
  local tag="$1"
  local seed_tum="$2"
  local feedback_bag="$3"
  local odir="${RUN_ROOT}/${tag}/ct"
  local ct_log="${RUN_ROOT}/${tag}/ct_run.log"
  mkdir -p "${odir}"

  require_file "${seed_tum}" "${tag} CT seed TUM"
  require_dir  "${feedback_bag}" "${tag} feedback bag"

  local out_tum="${odir}/continuous_time_trajectory.tum"
  rm -f "${out_tum}"

  log "${tag}: continuous-time render-photometric run (deterministic replay)"
  # REFERENCE_TUM=__skip__ : the parity script's internal compare uses strict
  # association; we run our own eval below with --max-association-dt 0.2.
  # The parity script has a known NON-FATAL trailing post-processing step that
  # exits non-zero (ValueError on an empty field) AFTER the deterministic TUM is
  # captured. So we do NOT treat the parity exit code as authoritative; the
  # out_tum (with >0 poses) is the success criterion.
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
  log "${tag}: CT wrote ${nlines}-pose TUM -> ${out_tum}"
  [ "${nlines}" -gt 0 ] || fail "${tag} CT TUM has zero poses"

  printf '%s\n' "${out_tum}"
}

# --------------------------------------------------------------------------
# eval_step: trajectory_compare.py vs the native reference. Prints rmse + span.
#   $1 = iter tag
#   $2 = current TUM
# --------------------------------------------------------------------------
eval_step() {
  local tag="$1"
  local cur_tum="$2"
  local report="${RUN_ROOT}/${tag}/trajectory_compare_${tag}.json"

  log "${tag}: eval vs ${REFERENCE_TUM##*/} (align yaw, max-assoc-dt ${MAX_ASSOC_DT})"
  python3 "${EVAL}" \
    --baseline "${REFERENCE_TUM}" \
    --current "${cur_tum}" \
    --align yaw \
    --max-association-dt "${MAX_ASSOC_DT}" \
    --output "${report}" \
    --min-matches 1 \
    --min-coverage 0.0 \
    --max-rmse-m 1e9 --max-mean-m 1e9 --max-error-m 1e9 --max-path-drift 1e9 \
    --json > "${RUN_ROOT}/${tag}/eval_${tag}.stdout" 2>&1 || true
  [ -s "${report}" ] || fail "${tag} eval produced no report"

  # Extract rmse + matched + span from the JSON report (python3.12, no deps).
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
print(f"[option_a] ===== {tag} RESULT =====")
print(f"[option_a]   translation.rmse_m = {rmse}")
print(f"[option_a]   translation.mean_m = {mean}")
print(f"[option_a]   matched_poses      = {matched}  coverage = {cov}")
print(f"[option_a]   current TUM span_s = {span:.3f}  poses = {len(stamps)}")
print(f"[option_a]   report             = {report_path}")
PY
}

# ==========================================================================
# OUTER LOOP
# ==========================================================================
log "=============================================================="
log "OPTION-A OUTER LOOP START (2 iterations)"
log "  map        : ${MAP_PLY}"
log "  seed TUM   : ${SEED_TUM}"
log "  source bag : ${SOURCE_BAG}"
log "  reference  : ${REFERENCE_TUM}"
log "  HONEST: SOTA-capped, ~0.284 m expected, not cm."
log "=============================================================="

# ---- iter0 : render map @ CT_SEED_TUM -> feedback -> CT run -> TUM1 --------
log "---- ITER0 ----"
FB0="$(render_step "iter0" "${SEED_TUM}" | tail -n1)"
TUM1="$(ct_step "iter0" "${SEED_TUM}" "${FB0}" | tail -n1)"
eval_step "iter0" "${TUM1}"

# ---- iter1 : render map @ TUM1 -> feedback -> CT run -> TUM2 ---------------
# Render anchors TUM1 to its own first pose; CT_SEED_TUM stays the ORIGINAL
# seed frame so successive seeds remain in the same body world (the node's
# set_reference_trajectory expects the seed in the estimator's own frame).
log "---- ITER1 ----"
FB1="$(render_step "iter1" "${TUM1}" | tail -n1)"
TUM2="$(ct_step "iter1" "${SEED_TUM}" "${FB1}" | tail -n1)"
eval_step "iter1" "${TUM2}"

log "=============================================================="
log "OPTION-A OUTER LOOP DONE"
log "  iter0 TUM : ${TUM1}"
log "  iter1 TUM : ${TUM2}"
log "  artifacts : ${RUN_ROOT}"
log "=============================================================="
