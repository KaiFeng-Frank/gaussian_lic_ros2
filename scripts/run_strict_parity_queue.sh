#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DATA_ROOT="${DATA_ROOT:-/home/frank/data}"
ROSBAGS_SITE="${ROSBAGS_SITE:-/home/frank/.cache/gaussian_lic_ros2/rosbags-venv/lib/python3.12/site-packages}"
PYTHON_BIN="${PYTHON_BIN:-/usr/bin/python3}"

PLAY_RATE="0.15"
CURRENT_RECORD_SEC="0"
CURRENT_POST_PLAY_SETTLE="60"
TIMEOUT_SEC="30"
SAVE_TIMEOUT_SEC="600"
UPSTREAM_RUNTIME_SEC="0"
TORCH_DEVICE="cuda"
TORCH_OPTIMIZATION_STEPS="100"
TORCH_MAX_FOREGROUND="100000"
TORCH_SEED="20260505"
TORCH_SAMPLING="upstream_random"
QUALITY_LPIPS_DEVICE="${QUALITY_LPIPS_DEVICE:-cuda}"
OVERWRITE=false
DRY_RUN=false
FETCH_MISSING=false
SKIP_CONVERT=false
SKIP_MAPPER_CONTRACT=false
SKIP_BASELINE=false
SKIP_CURRENT=false
SKIP_REPORT=false
CONTINUE_ON_ERROR=false
TARGETS=()

usage() {
  cat <<'EOF'
Usage: scripts/run_strict_parity_queue.sh [OPTIONS] [profile[:sequence]...]

Run the strict artifact backlog as an executable queue instead of a one-off CBD
script. The queue reuses existing frontend_raw bags when present, can generate a
ROS1 mapper-contract bag, runs the upstream ROS1 baseline with the matching
profile config, collects ROS2 current rasterizer artifacts, and writes strict
readiness/reproduction reports.

Default targets:
  fastlivo:hku1
  fastlivo:hku2
  fastlivo:Visual_Challenge
  fastlivo:LiDAR_Degenerate
  fastlivo2:CBD_Building_01
  fastlivo2:Retail_Street
  m2dgr:room_01
  m2dgr:room_02
  m2dgr:room_03
  mcd:ntu_day_01
  r3live:hku_park_00

Options:
  --data-root DIR                 Default: /home/frank/data or DATA_ROOT.
  --fetch-missing                 Try the repository fetchers before failing on
                                  missing raw inputs.
  --overwrite                     Recreate generated frontend/mapper/current
                                  outputs where supported.
  --dry-run                       Print commands without executing them.
  --continue-on-error             Continue to the next target when one fails.
  --skip-convert                  Reuse existing frontend_raw bags only.
  --skip-mapper-contract          Reuse existing ROS1 mapper-contract bags only.
  --skip-baseline                 Reuse existing ROS1 baseline artifacts only.
  --skip-current                  Reuse existing ROS2 current artifacts only.
  --skip-report                   Stop before strict readiness/report.
  --play-rate R                   ROS2 replay rate. Default: 0.15.
  --current-record-sec SEC        ROS2 recording window. Default: 0, meaning
                                  record until bag playback and post-play
                                  settle complete.
  --current-post-play-settle SEC  ROS2 settle time. Default: 60.
  --timeout SEC                   Current-result wait timeout. Default: 30.
  --save-timeout SEC              SaveMap/final-render timeout. Default: 600.
  --upstream-runtime-sec SEC      Upstream baseline runtime cap. Default: 0.
  --torch-device DEVICE           Default: cuda.
  --torch-optimization-steps N    Default: 100, matching upstream optimize().
  --quality-lpips-device DEVICE   LPIPS evaluation device. Default: cuda, or
                                  QUALITY_LPIPS_DEVICE when set.
  --help                          Show this help.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --data-root)
      DATA_ROOT="$2"
      shift 2
      ;;
    --fetch-missing)
      FETCH_MISSING=true
      shift
      ;;
    --overwrite)
      OVERWRITE=true
      shift
      ;;
    --dry-run)
      DRY_RUN=true
      shift
      ;;
    --continue-on-error)
      CONTINUE_ON_ERROR=true
      shift
      ;;
    --skip-convert)
      SKIP_CONVERT=true
      shift
      ;;
    --skip-mapper-contract)
      SKIP_MAPPER_CONTRACT=true
      shift
      ;;
    --skip-baseline)
      SKIP_BASELINE=true
      shift
      ;;
    --skip-current)
      SKIP_CURRENT=true
      shift
      ;;
    --skip-report)
      SKIP_REPORT=true
      shift
      ;;
    --play-rate)
      PLAY_RATE="$2"
      shift 2
      ;;
    --current-record-sec)
      CURRENT_RECORD_SEC="$2"
      shift 2
      ;;
    --current-post-play-settle)
      CURRENT_POST_PLAY_SETTLE="$2"
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
    --upstream-runtime-sec)
      UPSTREAM_RUNTIME_SEC="$2"
      shift 2
      ;;
    --torch-device)
      TORCH_DEVICE="$2"
      shift 2
      ;;
    --torch-optimization-steps)
      TORCH_OPTIMIZATION_STEPS="$2"
      shift 2
      ;;
    --quality-lpips-device)
      QUALITY_LPIPS_DEVICE="$2"
      shift 2
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    --*)
      echo "unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
    *)
      TARGETS+=("$1")
      shift
      ;;
  esac
done

if [[ ${#TARGETS[@]} -eq 0 ]]; then
  TARGETS=(
    fastlivo:hku1
    fastlivo:hku2
    fastlivo:Visual_Challenge
    fastlivo:LiDAR_Degenerate
    fastlivo2:CBD_Building_01
    fastlivo2:Retail_Street
    m2dgr:room_01
    m2dgr:room_02
    m2dgr:room_03
    mcd:ntu_day_01
    r3live:hku_park_00
  )
fi

cd "${ROOT_DIR}"
DATA_ROOT="$(realpath -m "${DATA_ROOT}")"

run_cmd() {
  printf '[strict-queue]'
  printf ' %q' "$@"
  printf '\n'
  if [[ "${DRY_RUN}" != "true" ]]; then
    "$@"
  fi
}

run_optional_cmd() {
  printf '[strict-queue]'
  printf ' %q' "$@"
  printf ' || true\n'
  if [[ "${DRY_RUN}" != "true" ]]; then
    "$@" || true
  fi
}

has_frontend_raw() {
  [[ -f "$1/metadata.yaml" ]] && find "$1" -maxdepth 1 \( -name '*.db3' -o -name '*.mcap' \) -print -quit | rg -q .
}

overwrite_arg=()
if [[ "${OVERWRITE}" == "true" ]]; then
  overwrite_arg=(--overwrite)
fi

target_profile() {
  local target="$1"
  echo "${target%%:*}"
}

target_sequence() {
  local target="$1"
  if [[ "$target" == *:* ]]; then
    echo "${target#*:}"
  else
    case "$target" in
      fastlivo) echo "hku1" ;;
      fastlivo2) echo "CBD_Building_01" ;;
      m2dgr) echo "room_01" ;;
      mcd) echo "ntu_day_01" ;;
      r3live) echo "hku_park_00" ;;
      *) echo "" ;;
    esac
  fi
}

raw_input_for() {
  local profile="$1"
  local sequence="$2"
  case "${profile}:${sequence}" in
    fastlivo:hku1) echo "${DATA_ROOT}/fast_livo_mcap/FAST-LIVO/hku1/hku1_0.mcap" ;;
    fastlivo:hku2) echo "${DATA_ROOT}/fast_livo_mcap/FAST-LIVO/hku2/hku2_0.mcap" ;;
    fastlivo:Visual_Challenge) echo "${DATA_ROOT}/fast_livo_mcap/FAST-LIVO/Visual_Challenge/Visual_Challenge_0.mcap" ;;
    fastlivo:LiDAR_Degenerate) echo "${DATA_ROOT}/fast_livo_mcap/FAST-LIVO/LiDAR_Degenerate/LiDAR_Degenerate_0.mcap" ;;
    fastlivo2:*) echo "${DATA_ROOT}/fast_livo/${sequence}.bag" ;;
    m2dgr:*) echo "${DATA_ROOT}/m2dgr/${sequence}/${sequence}.bag" ;;
    mcd:ntu_day_01)
      echo "${DATA_ROOT}/mcd/ntu_day_01/ntu_day_01_d435i.bag,${DATA_ROOT}/mcd/ntu_day_01/ntu_day_01_mid70.bag,${DATA_ROOT}/mcd/ntu_day_01/ntu_day_01_vn100.bag"
      ;;
    r3live:*) echo "${DATA_ROOT}/r3live/${sequence}.bag" ;;
    *) return 1 ;;
  esac
}

frontend_raw_for() {
  local profile="$1"
  local sequence="$2"
  case "$profile" in
    fastlivo) echo "${DATA_ROOT}/fast_livo_mcap/${sequence}_frontend_raw" ;;
    fastlivo2) echo "${DATA_ROOT}/fast_livo/${sequence}_frontend_raw" ;;
    m2dgr) echo "${DATA_ROOT}/m2dgr/${sequence}_frontend_raw" ;;
    mcd) echo "${DATA_ROOT}/mcd/${sequence}_frontend_raw" ;;
    r3live) echo "${DATA_ROOT}/r3live/${sequence}_frontend_raw" ;;
    *) return 1 ;;
  esac
}

mapper_contract_for() {
  local profile="$1"
  local sequence="$2"
  case "$profile" in
    fastlivo) echo "${DATA_ROOT}/fast_livo_mcap/${sequence}_mapper_contract_${profile}_color.bag" ;;
    fastlivo2) echo "${DATA_ROOT}/fast_livo/${sequence}_mapper_contract_${profile}_color.bag" ;;
    m2dgr) echo "${DATA_ROOT}/m2dgr/${sequence}_mapper_contract_${profile}_color.bag" ;;
    mcd) echo "${DATA_ROOT}/mcd/${sequence}_mapper_contract_${profile}_color.bag" ;;
    r3live) echo "${DATA_ROOT}/r3live/${sequence}_mapper_contract_${profile}_color.bag" ;;
    *) return 1 ;;
  esac
}

profile_data_root() {
  local profile="$1"
  case "$profile" in
    fastlivo) echo "${DATA_ROOT}/fast_livo_mcap" ;;
    fastlivo2) echo "${DATA_ROOT}/fast_livo" ;;
    m2dgr) echo "${DATA_ROOT}/m2dgr" ;;
    mcd) echo "${DATA_ROOT}/mcd" ;;
    r3live) echo "${DATA_ROOT}/r3live" ;;
    *) return 1 ;;
  esac
}

pointcloud_transform_profile() {
  local profile="$1"
  if [[ "$profile" == "fastlivo2" ]]; then
    echo "fastlivo2"
  else
    echo "identity"
  fi
}

fetch_missing_raw() {
  local profile="$1"
  local sequence="$2"
  case "$profile" in
    fastlivo) run_cmd ./scripts/fetch_strict_data_queue.sh --data-root "${DATA_ROOT}" fastlivo-hf ;;
    fastlivo2) run_cmd ./scripts/fetch_fastlivo2_sequence.py --sequence "${sequence}" --output-dir "${DATA_ROOT}/fast_livo" ;;
    m2dgr) run_cmd ./scripts/fetch_strict_data_queue.sh --data-root "${DATA_ROOT}" m2dgr-room ;;
    mcd) run_cmd ./scripts/fetch_strict_data_queue.sh --data-root "${DATA_ROOT}" mcd-ntu-day-01 ;;
    r3live)
      echo "[strict-queue] no automated R3LIVE fetcher is available for ${sequence}; expected ${DATA_ROOT}/r3live/${sequence}.bag" >&2
      return 2
      ;;
    *) return 2 ;;
  esac
}

check_raw_exists() {
  local raw="$1"
  IFS=',' read -r -a parts <<<"${raw}"
  for part in "${parts[@]}"; do
    if [[ ! -f "$part" && ! -d "$part" ]]; then
      return 1
    fi
  done
  return 0
}

run_target() {
  local target="$1"
  local profile
  local sequence
  profile="$(target_profile "$target")"
  sequence="$(target_sequence "$target")"
  if [[ -z "$profile" || -z "$sequence" ]]; then
    echo "[strict-queue] invalid target: $target" >&2
    return 2
  fi
  local raw frontend mapper_contract baseline_dir current_dir config data_root transform
  raw="$(raw_input_for "$profile" "$sequence")"
  frontend="$(frontend_raw_for "$profile" "$sequence")"
  mapper_contract="$(mapper_contract_for "$profile" "$sequence")"
  baseline_dir="${ROOT_DIR}/baseline/${profile}/${sequence}"
  current_dir="${ROOT_DIR}/results/${profile}/${sequence}_strict_current"
  config="${ROOT_DIR}/src/gaussian_lic_bringup/config/${profile}.yaml"
  data_root="$(profile_data_root "$profile")"
  transform="$(pointcloud_transform_profile "$profile")"

  echo "[strict-queue] target=${profile}:${sequence}"
  if ! check_raw_exists "$raw"; then
    if [[ "${FETCH_MISSING}" == "true" ]]; then
      fetch_missing_raw "$profile" "$sequence"
    fi
    check_raw_exists "$raw" || {
      echo "[strict-queue] missing raw input for ${profile}:${sequence}: ${raw}" >&2
      return 2
    }
  fi

  if [[ "${SKIP_CONVERT}" != "true" ]]; then
    local frontend_ready=false
    if [[ -d "${frontend}" ]] && has_frontend_raw "${frontend}"; then
      frontend_ready=true
    fi
    if [[ "${OVERWRITE}" == "true" || "${frontend_ready}" != "true" ]]; then
      run_cmd env PYTHONPATH="${ROSBAGS_SITE}${PYTHONPATH:+:${PYTHONPATH}}" "${PYTHON_BIN}" scripts/ros1_to_frontend_raw.py \
        --profile "${profile}" \
        --input "${raw}" \
        --output "${frontend}" \
        --storage sqlite3 \
        "${overwrite_arg[@]}"
      run_cmd ./scripts/rosbag2_timing_audit.py \
        --bag "${frontend}" \
        --required-topic /camera/image \
        --required-topic /camera/camera_info \
        --required-topic /livox/lidar \
        --required-topic /imu \
        --strict-storage
    else
      echo "[strict-queue] reuse frontend_raw: ${frontend}"
    fi
  fi

  if [[ "${SKIP_MAPPER_CONTRACT}" != "true" ]]; then
    if [[ "${OVERWRITE}" == "true" || ! -f "${mapper_contract}" ]]; then
      run_cmd env PYTHONPATH="${ROSBAGS_SITE}${PYTHONPATH:+:${PYTHONPATH}}" "${PYTHON_BIN}" scripts/frontend_raw_to_ros1_mapper_contract.py \
        --input "${frontend}" \
        --output "${mapper_contract}" \
        --pointcloud-transform-profile "${transform}" \
        --max-z 20.0 \
        --min-points-per-cloud 1000 \
        --imu-pose-fallback \
        --colorize-pointcloud \
        "${overwrite_arg[@]}"
    else
      echo "[strict-queue] reuse mapper-contract: ${mapper_contract}"
    fi
  fi

  if [[ "${SKIP_BASELINE}" != "true" ]]; then
    if [[ "${OVERWRITE}" == "true" || ! -f "${baseline_dir}/baseline_manifest.json" ]]; then
      run_cmd ./scripts/run_upstream_baseline.sh \
        --bag "${mapper_contract}" \
        --sequence "${sequence}" \
        --output "${baseline_dir}" \
        --dataset "${profile}" \
        --config-name "${profile}.yaml" \
        --runtime-sec "${UPSTREAM_RUNTIME_SEC}"
    else
      echo "[strict-queue] reuse ROS1 baseline: ${baseline_dir}"
    fi
    if [[ ! -f "${baseline_dir}/baseline_manifest.json" ]]; then
      echo "[strict-queue] ROS1 baseline did not produce baseline_manifest.json: ${baseline_dir}" >&2
      return 1
    fi
  fi

  if [[ "${SKIP_CURRENT}" != "true" ]]; then
    current_args=(
      ./scripts/collect_current_results.sh
      --bag "${frontend}"
      --output "${current_dir}"
      --config "${config}"
      --frontend-adapter
      --imu-pose-fallback
      --sync-image-to-pointcloud
      --adapter-imu-orientation-history-size 200000
      --adapter-pointcloud-filter-min-z 0.001
      --adapter-pointcloud-filter-max-z 20.0
      --adapter-pointcloud-filter-min-points 1000
      --optional-depth
      --sensor-qos reliable
      --sensor-qos-depth 50
      --play-rate "${PLAY_RATE}"
      --post-play-settle "${CURRENT_POST_PLAY_SETTLE}"
      --render-mode rasterizer
      --record-sec "${CURRENT_RECORD_SEC}"
      --timeout "${TIMEOUT_SEC}"
      --save-timeout "${SAVE_TIMEOUT_SEC}"
      --torch
      --torch-device "${TORCH_DEVICE}"
      --torch-optimization-steps "${TORCH_OPTIMIZATION_STEPS}"
      --torch-optimization-sampling "${TORCH_SAMPLING}"
      --torch-optimization-seed "${TORCH_SEED}"
      --torch-max-foreground "${TORCH_MAX_FOREGROUND}"
      --torch-prune-min-opacity 0.0
      --torch-prune-count-policy uniform
      --torch-densification
      --final-render-eval
      --no-publish-gaussian-map
    )
    if [[ "$profile" == "fastlivo2" ]]; then
      current_args+=(--fastlivo2-camera-lidar-transform)
    fi
    if [[ "${OVERWRITE}" != "true" && -f "${current_dir}/metrics.json" && -f "${current_dir}/trajectory.tum" ]]; then
      echo "[strict-queue] reuse ROS2 current: ${current_dir}"
    else
      run_cmd "${current_args[@]}"
    fi
  fi

  if [[ "${SKIP_REPORT}" != "true" ]]; then
    local quality_python="${QUALITY_PYTHON:-}"
    if [[ -z "${quality_python}" ]]; then
      local quality_venv="${GAUSSIAN_LIC_QUALITY_VENV:-}"
      if [[ -n "${quality_venv}" && -x "${quality_venv}/bin/python" ]]; then
        quality_python="${quality_venv}/bin/python"
      elif [[ -x "${HOME}/.cache/gaussian_lic_ros2/quality-cuda-venv/bin/python" ]]; then
        quality_python="${HOME}/.cache/gaussian_lic_ros2/quality-cuda-venv/bin/python"
      elif [[ -x "${HOME}/.cache/gaussian_lic_ros2/quality-venv/bin/python" ]]; then
        quality_python="${HOME}/.cache/gaussian_lic_ros2/quality-venv/bin/python"
      else
        quality_python="/usr/bin/python3"
      fi
    fi
    local lpips_model="${ROOT_DIR}/external/Gaussian-LIC/src/lpips/lpips_alex.pt"
    local lpips_root="${ROOT_DIR}/external/Gaussian-LIC/src/lpips"
    local lpips_device="${QUALITY_LPIPS_DEVICE}"
    local baseline_gt="${baseline_dir}/upstream_result/gt"
    local current_gt="${current_dir}/gt"
    if [[ ! -d "${current_gt}" ]]; then
      current_gt="${baseline_gt}"
    fi
    if [[ -d "${baseline_dir}/upstream_result/render" && -d "${baseline_gt}" ]]; then
      run_optional_cmd "${quality_python}" scripts/eval_render_quality.py \
        --result-dir "${baseline_dir}" \
        --render-dir "${baseline_dir}/upstream_result/render" \
        --reference-dir "${baseline_gt}" \
        --lpips-model "${lpips_model}" \
        --lpips-device "${lpips_device}" \
        --lpips-pytorch-root "${lpips_root}"
    fi
    if [[ -d "${current_dir}/renders" && -d "${current_gt}" ]]; then
      run_optional_cmd "${quality_python}" scripts/eval_render_quality.py \
        --result-dir "${current_dir}" \
        --render-dir "${current_dir}/renders" \
        --reference-dir "${current_gt}" \
        --lpips-model "${lpips_model}" \
        --lpips-device "${lpips_device}" \
        --lpips-pytorch-root "${lpips_root}"
    fi

    set +e
    run_cmd ./scripts/baseline_readiness.py \
      --dataset-root "${data_root}" \
      --baseline-dir "${baseline_dir}" \
      --current-results-dir "${current_dir}" \
      --dataset "${profile}" \
      --sequence "${sequence}" \
      --strict \
      --output "${current_dir}/baseline_readiness_strict.json" \
      --markdown "${current_dir}/baseline_readiness_strict.md"
    local readiness_status=$?
    run_cmd ./scripts/reproduction_report.py \
      --baseline-dir "${baseline_dir}" \
      --current-dir "${current_dir}" \
      --dataset "${profile}" \
      --sequence "${sequence}" \
      --strict \
      --output "${current_dir}/reproduction_report_strict.json" \
      --markdown "${current_dir}/reproduction_report_strict.md"
    local reproduction_status=$?
    set -e
    if [[ ${readiness_status} -ne 0 || ${reproduction_status} -ne 0 ]]; then
      echo "[strict-queue] strict report failed for ${profile}:${sequence}" >&2
      return 1
    fi
  fi
}

failures=0
for target in "${TARGETS[@]}"; do
  if ! run_target "$target"; then
    failures=$((failures + 1))
    if [[ "${CONTINUE_ON_ERROR}" != "true" ]]; then
      exit 1
    fi
  fi
done

run_cmd ./scripts/audit_strict_data_inputs.py \
  --data-root "${DATA_ROOT}" \
  --output "${ROOT_DIR}/docs/strict_data_status.json" \
  --markdown "${ROOT_DIR}/docs/strict_data_status.md" \
  --cleanup-candidates 12 \
  --min-free-gb 100

run_cmd python3 scripts/check_strict_parity_matrix.py --allow-incomplete

if [[ ${failures} -ne 0 ]]; then
  echo "[strict-queue] completed with ${failures} failed target(s)" >&2
  exit 1
fi
echo "[strict-queue] completed"
