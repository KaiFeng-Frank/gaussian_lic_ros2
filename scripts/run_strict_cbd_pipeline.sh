#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DATASET_ROOT="/home/frank/data/fast_livo"
SEQUENCE="CBD_Building_01"
BAG_PATH=""
FRONTEND_RAW=""
MAPPER_CONTRACT=""
BASELINE_DIR=""
CURRENT_DIR=""
CURRENT_CONFIG=""
RENDER_MODE="rasterizer"
UPSTREAM_RUNTIME_SEC=0
CURRENT_RECORD_SEC=600
CURRENT_PLAY_RATE=0.25
CURRENT_POST_PLAY_SETTLE=60
CURRENT_TORCH_DEVICE=cuda
CURRENT_TORCH_OPTIMIZATION_STEPS=100
CURRENT_TORCH_OPTIMIZATION_SAMPLING=upstream_random
CURRENT_TORCH_OPTIMIZATION_SEED=20260505
CURRENT_TORCH_MAX_FOREGROUND=1500000
CURRENT_TORCH_PRUNE_MIN_OPACITY=0.0
CURRENT_TORCH_PRUNE_COUNT_POLICY=uniform
CURRENT_TORCH_PRUNE_MAX_WORLD_SCALE=0.0
CURRENT_TORCH_EXTEND_VISIBILITY_FILTER=true
CURRENT_TORCH_EXTEND_ALPHA_THRESHOLD=0.99
CURRENT_TORCH_OPACITY_RESET_INTERVAL=0
CURRENT_TORCH_DENSIFICATION=false
QUALITY_LPIPS_DEVICE="${QUALITY_LPIPS_DEVICE:-cuda}"
TIMEOUT_SEC=30
SAVE_TIMEOUT_SEC=600
OVERWRITE=false
SKIP_CONVERT=false
SKIP_BASELINE=false
SKIP_CURRENT=false
SKIP_REPORT=false

usage() {
  cat <<'EOF'
Usage: scripts/run_strict_cbd_pipeline.sh [OPTIONS]

Run the strict CBD_Building_01 reproduction chain:
  ROS1 FAST-LIVO2 bag -> ROS2 frontend_raw -> ROS1 mapper-contract bag
  -> ROS1 upstream baseline -> ROS2 current results -> strict report.

Options:
  --dataset-root DIR       Default: /home/frank/data/fast_livo
  --sequence NAME          Default: CBD_Building_01
  --bag FILE               Default: <dataset-root>/<sequence>.bag
  --frontend-raw DIR       Default: <dataset-root>/<sequence>_frontend_raw
  --mapper-contract FILE   Default: <dataset-root>/<sequence>_mapper_contract_fastlivo2_color.bag
  --baseline-dir DIR       Default: baseline/fastlivo2/<sequence>
  --current-dir DIR        Default: results/fastlivo2/<sequence>_current
  --current-config FILE    Current ROS2 parameter YAML. Default: bringup fastlivo2.yaml
  --render-mode MODE       debug_cpu, debug_input, rasterizer, or off. Default: rasterizer
  --upstream-runtime-sec N Pass to run_upstream_baseline.sh. Default: 0
  --current-record-sec N   Pass to collect_current_results.sh. Default: 600
  --current-play-rate R    Pass to collect_current_results.sh. Default: 0.25
  --current-post-play-settle SEC
                            Pass to collect_current_results.sh. Default: 60
  --current-torch-device DEVICE
                            Pass to collect_current_results.sh when --render-mode rasterizer is used.
  --current-torch-optimization-steps N
                            Max accumulated train-frame optimizer samples per keyframe in rasterizer mode.
                            Default: 100, matching upstream Gaussian-LIC optimize() max_iters.
  --current-torch-optimization-sampling MODE
                            Training-frame sampling in rasterizer mode. Default: upstream_random.
  --current-torch-optimization-seed N
                            Random seed for upstream_random sampling; 0 uses std::random_device.
  --current-torch-max-foreground N
                            Foreground Gaussian cap in rasterizer mode. Default: 1500000
  --current-torch-prune-min-opacity X
                            Foreground opacity pruning threshold in rasterizer mode. Default: 0.0,
                            matching the released LIC2 append-only map used for strict parity.
  --current-torch-prune-count-policy P
                            Foreground count-cap policy in rasterizer mode. Default: uniform
  --current-torch-prune-max-world-scale X
                            Drop foreground Gaussians whose max world scale exceeds X meters; 0 disables.
  --no-current-torch-extend-visibility-filter
                            Append all pending points instead of upstream-style alpha-hole filtering.
  --current-torch-extend-alpha-threshold X
                            Alpha threshold for current-view extension filtering. Default: 0.99
  --current-torch-opacity-reset-interval N
                            Steps between foreground opacity resets in rasterizer mode. Default: 0
  --current-torch-densification
                            Enable ROS2 gradient split/clone densification in rasterizer mode.
                            Default: off, matching released Gaussian-LIC2 append-only extend().
  --quality-lpips-device DEVICE
                            LPIPS evaluation device. Default: cuda, or QUALITY_LPIPS_DEVICE when set.
  --timeout N              Current-result wait timeout. Default: 30
  --save-timeout N         SaveMap/final-render timeout. Default: 600
  --overwrite              Recreate converted frontend/mapper-contract outputs.
  --skip-convert           Reuse existing converted bags.
  --skip-baseline          Reuse existing ROS1 baseline artifacts.
  --skip-current           Reuse existing ROS2 current artifacts.
  --skip-report            Stop before strict readiness/report.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --dataset-root)
      DATASET_ROOT="$2"
      shift 2
      ;;
    --sequence)
      SEQUENCE="$2"
      shift 2
      ;;
    --bag)
      BAG_PATH="$2"
      shift 2
      ;;
    --frontend-raw)
      FRONTEND_RAW="$2"
      shift 2
      ;;
    --mapper-contract)
      MAPPER_CONTRACT="$2"
      shift 2
      ;;
    --baseline-dir)
      BASELINE_DIR="$2"
      shift 2
      ;;
    --current-dir)
      CURRENT_DIR="$2"
      shift 2
      ;;
    --current-config)
      CURRENT_CONFIG="$2"
      shift 2
      ;;
    --render-mode)
      RENDER_MODE="$2"
      shift 2
      ;;
    --upstream-runtime-sec)
      UPSTREAM_RUNTIME_SEC="$2"
      shift 2
      ;;
    --current-record-sec)
      CURRENT_RECORD_SEC="$2"
      shift 2
      ;;
    --current-play-rate)
      CURRENT_PLAY_RATE="$2"
      shift 2
      ;;
    --current-post-play-settle)
      CURRENT_POST_PLAY_SETTLE="$2"
      shift 2
      ;;
    --current-torch-device)
      CURRENT_TORCH_DEVICE="$2"
      shift 2
      ;;
    --current-torch-optimization-steps)
      CURRENT_TORCH_OPTIMIZATION_STEPS="$2"
      shift 2
      ;;
    --current-torch-optimization-sampling)
      CURRENT_TORCH_OPTIMIZATION_SAMPLING="$2"
      shift 2
      ;;
    --current-torch-optimization-seed)
      CURRENT_TORCH_OPTIMIZATION_SEED="$2"
      shift 2
      ;;
    --current-torch-max-foreground)
      CURRENT_TORCH_MAX_FOREGROUND="$2"
      shift 2
      ;;
    --current-torch-prune-min-opacity)
      CURRENT_TORCH_PRUNE_MIN_OPACITY="$2"
      shift 2
      ;;
    --current-torch-prune-count-policy)
      CURRENT_TORCH_PRUNE_COUNT_POLICY="$2"
      shift 2
      ;;
    --current-torch-prune-max-world-scale)
      CURRENT_TORCH_PRUNE_MAX_WORLD_SCALE="$2"
      shift 2
      ;;
    --no-current-torch-extend-visibility-filter)
      CURRENT_TORCH_EXTEND_VISIBILITY_FILTER=false
      shift
      ;;
    --current-torch-extend-alpha-threshold)
      CURRENT_TORCH_EXTEND_ALPHA_THRESHOLD="$2"
      shift 2
      ;;
    --current-torch-opacity-reset-interval)
      CURRENT_TORCH_OPACITY_RESET_INTERVAL="$2"
      shift 2
      ;;
    --current-torch-densification)
      CURRENT_TORCH_DENSIFICATION=true
      shift
      ;;
    --quality-lpips-device)
      QUALITY_LPIPS_DEVICE="$2"
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
    --overwrite)
      OVERWRITE=true
      shift
      ;;
    --skip-convert)
      SKIP_CONVERT=true
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
DATASET_ROOT="$(realpath -m "${DATASET_ROOT}")"
BAG_PATH="${BAG_PATH:-${DATASET_ROOT}/${SEQUENCE}.bag}"
FRONTEND_RAW="${FRONTEND_RAW:-${DATASET_ROOT}/${SEQUENCE}_frontend_raw}"
MAPPER_CONTRACT="${MAPPER_CONTRACT:-${DATASET_ROOT}/${SEQUENCE}_mapper_contract_fastlivo2_color.bag}"
BASELINE_DIR="${BASELINE_DIR:-${ROOT_DIR}/baseline/fastlivo2/${SEQUENCE}}"
CURRENT_DIR="${CURRENT_DIR:-${ROOT_DIR}/results/fastlivo2/${SEQUENCE}_current}"
CURRENT_CONFIG="${CURRENT_CONFIG:-${ROOT_DIR}/src/gaussian_lic_bringup/config/fastlivo2.yaml}"
BAG_PATH="$(realpath -m "${BAG_PATH}")"
FRONTEND_RAW="$(realpath -m "${FRONTEND_RAW}")"
MAPPER_CONTRACT="$(realpath -m "${MAPPER_CONTRACT}")"
BASELINE_DIR="$(realpath -m "${BASELINE_DIR}")"
CURRENT_DIR="$(realpath -m "${CURRENT_DIR}")"
CURRENT_CONFIG="$(realpath -m "${CURRENT_CONFIG}")"

if [[ ! -f "${BAG_PATH}" ]]; then
  echo "missing strict sequence bag: ${BAG_PATH}" >&2
  echo "fetch it with: ./scripts/fetch_fastlivo2_sequence.py --sequence ${SEQUENCE} --output-dir ${DATASET_ROOT}" >&2
  exit 2
fi

overwrite_flag=()
if [[ "${OVERWRITE}" == "true" ]]; then
  overwrite_flag+=(--overwrite)
fi

if [[ "${SKIP_CONVERT}" != "true" ]]; then
  echo "[strict-cbd] converting ROS1 FAST-LIVO2 bag to ROS2 frontend_raw: ${FRONTEND_RAW}"
  python3 scripts/fastlivo2_ros1_to_frontend_raw.py \
    --input "${BAG_PATH}" \
    --output "${FRONTEND_RAW}" \
    --storage sqlite3 \
    "${overwrite_flag[@]}"

  ./scripts/rosbag2_timing_audit.py \
    --bag "${FRONTEND_RAW}" \
    --required-topic /camera/image \
    --required-topic /camera/camera_info \
    --required-topic /livox/lidar \
    --required-topic /imu \
    --strict-storage

  echo "[strict-cbd] converting frontend_raw to ROS1 mapper-contract bag: ${MAPPER_CONTRACT}"
  python3 scripts/frontend_raw_to_ros1_mapper_contract.py \
    --input "${FRONTEND_RAW}" \
    --output "${MAPPER_CONTRACT}" \
    --pointcloud-transform-profile fastlivo2 \
    --imu-pose-fallback \
    --colorize-pointcloud \
    "${overwrite_flag[@]}"
fi

if [[ "${SKIP_BASELINE}" != "true" ]]; then
  echo "[strict-cbd] running upstream ROS1 baseline: ${BASELINE_DIR}"
  ./scripts/run_upstream_baseline.sh \
    --bag "${MAPPER_CONTRACT}" \
    --sequence "${SEQUENCE}" \
    --output "${BASELINE_DIR}" \
    --runtime-sec "${UPSTREAM_RUNTIME_SEC}"
fi

if [[ "${SKIP_CURRENT}" != "true" ]]; then
  echo "[strict-cbd] collecting ROS2 current results: ${CURRENT_DIR}"
  current_torch_args=()
  if [[ "${RENDER_MODE}" == "rasterizer" ]]; then
    current_torch_args+=(
      --torch
      --torch-device "${CURRENT_TORCH_DEVICE}"
      --torch-optimization-steps "${CURRENT_TORCH_OPTIMIZATION_STEPS}"
      --torch-optimization-sampling "${CURRENT_TORCH_OPTIMIZATION_SAMPLING}"
      --torch-optimization-seed "${CURRENT_TORCH_OPTIMIZATION_SEED}"
      --torch-max-foreground "${CURRENT_TORCH_MAX_FOREGROUND}"
      --torch-prune-min-opacity "${CURRENT_TORCH_PRUNE_MIN_OPACITY}"
      --torch-prune-count-policy "${CURRENT_TORCH_PRUNE_COUNT_POLICY}"
      --torch-prune-max-world-scale "${CURRENT_TORCH_PRUNE_MAX_WORLD_SCALE}"
      --torch-extend-alpha-threshold "${CURRENT_TORCH_EXTEND_ALPHA_THRESHOLD}"
      --torch-opacity-reset-interval "${CURRENT_TORCH_OPACITY_RESET_INTERVAL}"
      --final-render-eval
      --no-publish-gaussian-map
    )
    if [[ "${CURRENT_TORCH_EXTEND_VISIBILITY_FILTER}" != "true" ]]; then
      current_torch_args+=(--no-torch-extend-visibility-filter)
    fi
    if [[ "${CURRENT_TORCH_DENSIFICATION}" == "true" ]]; then
      current_torch_args+=(--torch-densification)
    fi
  fi
  ./scripts/collect_current_results.sh \
    --bag "${FRONTEND_RAW}" \
    --output "${CURRENT_DIR}" \
    --config "${CURRENT_CONFIG}" \
    "${current_torch_args[@]}" \
    --frontend-adapter \
    --imu-pose-fallback \
    --sync-image-to-pointcloud \
    --fastlivo2-camera-lidar-transform \
    --optional-depth \
    --sensor-qos reliable \
    --sensor-qos-depth 50 \
    --play-rate "${CURRENT_PLAY_RATE}" \
    --post-play-settle "${CURRENT_POST_PLAY_SETTLE}" \
    --render-mode "${RENDER_MODE}" \
    --record-sec "${CURRENT_RECORD_SEC}" \
    --timeout "${TIMEOUT_SEC}" \
    --save-timeout "${SAVE_TIMEOUT_SEC}"
fi

if [[ "${SKIP_REPORT}" != "true" ]]; then
  echo "[strict-cbd] running strict readiness and reproduction report"
  BASELINE_QUALITY_REFERENCE_DIR="${BASELINE_DIR}/upstream_result/gt"
  CURRENT_QUALITY_REFERENCE_DIR="${CURRENT_DIR}/gt"
  if [[ ! -d "${CURRENT_QUALITY_REFERENCE_DIR}" ]]; then
    CURRENT_QUALITY_REFERENCE_DIR="${BASELINE_QUALITY_REFERENCE_DIR}"
  fi
  QUALITY_LPIPS_MODEL="${ROOT_DIR}/external/Gaussian-LIC/src/lpips/lpips_alex.pt"
  QUALITY_LPIPS_ROOT="${ROOT_DIR}/external/Gaussian-LIC/src/lpips"
  QUALITY_PYTHON="${QUALITY_PYTHON:-}"
  if [[ -z "${QUALITY_PYTHON}" ]]; then
    quality_venv="${GAUSSIAN_LIC_QUALITY_VENV:-${HOME}/.cache/gaussian_lic_ros2/quality-venv}"
    if [[ -x "${quality_venv}/bin/python" ]]; then
      QUALITY_PYTHON="${quality_venv}/bin/python"
    else
      QUALITY_PYTHON="/usr/bin/python3"
    fi
  fi
  if [[ -d "${BASELINE_QUALITY_REFERENCE_DIR}" ]]; then
    if [[ -d "${BASELINE_DIR}/upstream_result/render" ]]; then
      "${QUALITY_PYTHON}" scripts/eval_render_quality.py \
        --result-dir "${BASELINE_DIR}" \
        --render-dir "${BASELINE_DIR}/upstream_result/render" \
        --reference-dir "${BASELINE_QUALITY_REFERENCE_DIR}" \
        --lpips-model "${QUALITY_LPIPS_MODEL}" \
        --lpips-device "${QUALITY_LPIPS_DEVICE}" \
        --lpips-pytorch-root "${QUALITY_LPIPS_ROOT}" \
        >"${BASELINE_DIR}/quality_eval.json" || true
    fi
    if [[ -d "${CURRENT_DIR}/renders" && -d "${CURRENT_QUALITY_REFERENCE_DIR}" ]]; then
      "${QUALITY_PYTHON}" scripts/eval_render_quality.py \
        --result-dir "${CURRENT_DIR}" \
        --render-dir "${CURRENT_DIR}/renders" \
        --reference-dir "${CURRENT_QUALITY_REFERENCE_DIR}" \
        --lpips-model "${QUALITY_LPIPS_MODEL}" \
        --lpips-device "${QUALITY_LPIPS_DEVICE}" \
        --lpips-pytorch-root "${QUALITY_LPIPS_ROOT}" \
        >"${CURRENT_DIR}/quality_eval.json" || true
    fi
  fi
  report_status=0
  set +e
  ./scripts/baseline_readiness.py \
    --dataset-root "${DATASET_ROOT}" \
    --baseline-dir "${BASELINE_DIR}" \
    --current-results-dir "${CURRENT_DIR}" \
    --sequence "${SEQUENCE}" \
    --strict \
    --output "${CURRENT_DIR}/baseline_readiness_strict.json" \
    --markdown "${CURRENT_DIR}/baseline_readiness_strict.md"
  readiness_status=$?

  ./scripts/reproduction_report.py \
    --baseline-dir "${BASELINE_DIR}" \
    --current-dir "${CURRENT_DIR}" \
    --sequence "${SEQUENCE}" \
    --strict \
    --output "${CURRENT_DIR}/reproduction_report_strict.json" \
    --markdown "${CURRENT_DIR}/reproduction_report_strict.md"
  reproduction_status=$?
  set -e
  if [[ ${readiness_status} -ne 0 || ${reproduction_status} -ne 0 ]]; then
    report_status=1
  fi
  if [[ ${report_status} -ne 0 ]]; then
    echo "[strict-cbd] strict report failed; JSON/Markdown artifacts were still written" >&2
    exit "${report_status}"
  fi
fi

echo "[strict-cbd] done"
