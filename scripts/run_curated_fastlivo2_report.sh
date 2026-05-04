#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON_BIN="${PYTHON_BIN:-/usr/bin/python3}"
ROSBAGS_SITE="${GAUSSIAN_LIC_ROSBAGS_SITE:-/home/frank/.cache/gaussian_lic_ros2/rosbags-venv/lib/python3.12/site-packages}"

SEQUENCE="Bright_Screen_Wall_curated_8s"
FRONTEND_RAW="/home/frank/data/fast_livo/Bright_Screen_Wall_frontend_raw"
MAPPER_BAG="/home/frank/data/fast_livo/Bright_Screen_Wall_mapper_contract_8s.bag"
BASELINE_DIR="${ROOT_DIR}/baseline/fastlivo2/${SEQUENCE}"
CURRENT_DIR="${ROOT_DIR}/results/fastlivo2/Bright_Screen_Wall_current"
MAX_DURATION_SEC=8
BASELINE_RUNTIME_SEC=180
CURRENT_RECORD_SEC=8
CURRENT_TIMEOUT_SEC=30
RENDER_MODE="debug_cpu"
RUN_TORCH_CURRENT=false
TORCH_OPTIMIZATION_STEPS=0
TORCH_MAX_FOREGROUND=0
TORCH_PRUNE_MIN_OPACITY=0.005
IMU_POSE_FALLBACK=false
POINTCLOUD_TRANSFORM_PROFILE="identity"
BASELINE_DIR_SET=false
CURRENT_DIR_SET=false
SKIP_CONVERT=false
SKIP_CURRENT=false
SKIP_BASELINE=false
SKIP_BUILD=false
REBUILD=false
CHECK_ONLY=false

usage() {
  cat <<'EOF'
Usage: scripts/run_curated_fastlivo2_report.sh [options]

Run the executable curated FAST-LIVO2 reproduction chain:

  1. Convert official Bright_Screen_Wall frontend raw rosbag2 input into the
     ROS1 mapper-contract bag consumed by upstream Gaussian-LIC/Gaussian-LIC2.
  2. Collect ROS2 current mapper artifacts from the same frontend raw input.
  3. Run the ROS1 upstream baseline in Docker on the shared mapper-contract bag.
  4. Emit reproduction_report.json/.md with metrics, trajectory, and PLY gates.

Defaults match the locally validated Bright_Screen_Wall_curated_8s substitute.
The strict CBD_Building_01 paper gate remains separate until that raw bag is
available locally.

Options:
  --frontend-raw DIR       ROS2 frontend raw rosbag2 directory.
  --mapper-bag FILE        ROS1 mapper-contract bag to create/use.
  --baseline-dir DIR       Baseline archive directory.
  --current-dir DIR        Current ROS2 artifact directory.
  --sequence NAME          Report sequence name.
  --max-duration-sec SEC   Mapper-contract conversion duration. Default: 8.
  --baseline-runtime-sec SEC
                           Upstream baseline run cap. Default: 180.
  --current-record-sec SEC Current ROS2 output recording duration. Default: 8.
  --current-timeout SEC    Current ROS2 wait timeout. Default: 30.
  --torch-current          Enable the current Torch Gaussian preview path.
  --torch-optimization-steps N
                           Enable current Torch photometric update steps per keyframe.
  --torch-max-foreground N Retain at most N foreground Gaussians in current Torch collection.
  --torch-prune-min-opacity X
                           Drop current Torch foreground Gaussians below opacity X.
  --imu-pose-fallback      Use IMU gyro orientation fallback for current ROS2 collection.
  --fastlivo2-camera-lidar-transform
                           Transform raw FAST-LIVO2 LiDAR points into camera frame.
  --render-mode MODE       Current render mode. Default: debug_cpu.
  --skip-convert           Reuse an existing mapper-contract bag.
  --skip-current           Reuse an existing ROS2 current artifact directory.
  --skip-baseline          Reuse an existing ROS1 baseline archive.
  --skip-build             Require an existing built upstream workspace.
  --rebuild                Rebuild the persistent upstream baseline workspace.
  --check-only             Validate prerequisites and exit without running.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --frontend-raw)
      FRONTEND_RAW="$2"
      shift 2
      ;;
    --mapper-bag)
      MAPPER_BAG="$2"
      shift 2
      ;;
    --baseline-dir)
      BASELINE_DIR="$2"
      BASELINE_DIR_SET=true
      shift 2
      ;;
    --current-dir)
      CURRENT_DIR="$2"
      CURRENT_DIR_SET=true
      shift 2
      ;;
    --sequence)
      SEQUENCE="$2"
      shift 2
      ;;
    --max-duration-sec)
      MAX_DURATION_SEC="$2"
      shift 2
      ;;
    --baseline-runtime-sec)
      BASELINE_RUNTIME_SEC="$2"
      shift 2
      ;;
    --current-record-sec)
      CURRENT_RECORD_SEC="$2"
      shift 2
      ;;
    --current-timeout)
      CURRENT_TIMEOUT_SEC="$2"
      shift 2
      ;;
    --torch-current)
      RUN_TORCH_CURRENT=true
      shift
      ;;
    --torch-optimization-steps)
      RUN_TORCH_CURRENT=true
      TORCH_OPTIMIZATION_STEPS="$2"
      shift 2
      ;;
    --torch-max-foreground)
      RUN_TORCH_CURRENT=true
      TORCH_MAX_FOREGROUND="$2"
      shift 2
      ;;
    --torch-prune-min-opacity)
      RUN_TORCH_CURRENT=true
      TORCH_PRUNE_MIN_OPACITY="$2"
      shift 2
      ;;
    --imu-pose-fallback)
      IMU_POSE_FALLBACK=true
      shift
      ;;
    --fastlivo2-camera-lidar-transform)
      POINTCLOUD_TRANSFORM_PROFILE="fastlivo2"
      shift
      ;;
    --render-mode)
      RENDER_MODE="$2"
      shift 2
      ;;
    --skip-convert)
      SKIP_CONVERT=true
      shift
      ;;
    --skip-current)
      SKIP_CURRENT=true
      shift
      ;;
    --skip-baseline)
      SKIP_BASELINE=true
      shift
      ;;
    --skip-build)
      SKIP_BUILD=true
      shift
      ;;
    --rebuild)
      REBUILD=true
      shift
      ;;
    --check-only)
      CHECK_ONLY=true
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
if [[ "${BASELINE_DIR_SET}" != "true" ]]; then
  BASELINE_DIR="${ROOT_DIR}/baseline/fastlivo2/${SEQUENCE}"
fi
if [[ "${CURRENT_DIR_SET}" != "true" && "${SEQUENCE}" != "Bright_Screen_Wall_curated_8s" ]]; then
  CURRENT_DIR="${ROOT_DIR}/results/fastlivo2/${SEQUENCE}_current"
fi
if [[ "${RENDER_MODE}" == "rasterizer" ]]; then
  RUN_TORCH_CURRENT=true
fi
FRONTEND_RAW="$(realpath -m "${FRONTEND_RAW}")"
MAPPER_BAG="$(realpath -m "${MAPPER_BAG}")"
BASELINE_DIR="$(realpath -m "${BASELINE_DIR}")"
CURRENT_DIR="$(realpath -m "${CURRENT_DIR}")"

run_rosbags_python() {
  if [[ -d "${ROSBAGS_SITE}" ]]; then
    PYTHONPATH="${ROSBAGS_SITE}${PYTHONPATH:+:${PYTHONPATH}}" "${PYTHON_BIN}" "$@"
  else
    "${PYTHON_BIN}" "$@"
  fi
}

require_file() {
  local path="$1"
  if [[ ! -f "${path}" ]]; then
    echo "Missing required file: ${path}" >&2
    exit 2
  fi
}

require_dir() {
  local path="$1"
  if [[ ! -d "${path}" ]]; then
    echo "Missing required directory: ${path}" >&2
    exit 2
  fi
}

check_artifacts() {
  local dir="$1"
  local label="$2"
  require_dir "${dir}"
  require_file "${dir}/trajectory.tum"
  require_file "${dir}/point_cloud.ply"
  require_file "${dir}/metrics.json"
  echo "[${label}] artifacts OK: ${dir}"
}

require_dir "${FRONTEND_RAW}"
if [[ "${SKIP_CONVERT}" == "true" || "${SKIP_BASELINE}" == "true" || "${CHECK_ONLY}" == "true" ]]; then
  require_file "${MAPPER_BAG}"
fi

if [[ "${SKIP_CURRENT}" == "true" || "${CHECK_ONLY}" == "true" ]]; then
  check_artifacts "${CURRENT_DIR}" "current"
fi
if [[ "${SKIP_BASELINE}" == "true" || "${CHECK_ONLY}" == "true" ]]; then
  check_artifacts "${BASELINE_DIR}" "baseline"
fi
if [[ "${CHECK_ONLY}" == "true" ]]; then
  baseline_check=(./scripts/run_upstream_baseline.sh --check-only)
  if [[ "${SKIP_BUILD}" == "true" ]]; then
    baseline_check+=(--skip-build)
  fi
  "${baseline_check[@]}"
  echo "curated FAST-LIVO2 report prerequisites OK: sequence=${SEQUENCE}"
  exit 0
fi

if [[ "${SKIP_CONVERT}" != "true" ]]; then
  echo "[curated] converting frontend raw bag to ROS1 mapper-contract bag"
  mkdir -p "$(dirname "${MAPPER_BAG}")"
  run_rosbags_python \
    scripts/frontend_raw_to_ros1_mapper_contract.py \
    --input "${FRONTEND_RAW}" \
    --output "${MAPPER_BAG}" \
    --max-duration-sec "${MAX_DURATION_SEC}" \
    --overwrite
fi

if [[ "${SKIP_CURRENT}" != "true" ]]; then
  echo "[curated] collecting ROS2 current artifacts"
  current_args=(
    ./scripts/collect_current_results.sh
    --bag "${FRONTEND_RAW}"
    --frontend-adapter
    --identity-pose-fallback
    --optional-depth
    --render-mode "${RENDER_MODE}"
    --output "${CURRENT_DIR}"
    --record-sec "${CURRENT_RECORD_SEC}"
    --timeout "${CURRENT_TIMEOUT_SEC}"
  )
  if [[ "${RUN_TORCH_CURRENT}" == "true" ]]; then
    current_args+=(--torch)
  fi
  if [[ "${TORCH_OPTIMIZATION_STEPS}" -gt 0 ]]; then
    current_args+=(--torch-optimization-steps "${TORCH_OPTIMIZATION_STEPS}")
  fi
  if [[ "${TORCH_MAX_FOREGROUND}" -gt 0 ]]; then
    current_args+=(--torch-max-foreground "${TORCH_MAX_FOREGROUND}")
  fi
  if [[ "${TORCH_PRUNE_MIN_OPACITY}" != "0.005" ]]; then
    current_args+=(--torch-prune-min-opacity "${TORCH_PRUNE_MIN_OPACITY}")
  fi
  if [[ "${IMU_POSE_FALLBACK}" == "true" ]]; then
    current_args+=(--imu-pose-fallback)
  fi
  if [[ "${POINTCLOUD_TRANSFORM_PROFILE}" != "identity" ]]; then
    current_args+=(--fastlivo2-camera-lidar-transform)
  fi
  "${current_args[@]}"
fi

if [[ "${SKIP_BASELINE}" != "true" ]]; then
  echo "[curated] running ROS1 upstream Gaussian-LIC/Gaussian-LIC2 baseline"
  baseline_args=(
    ./scripts/run_upstream_baseline.sh
    --bag "${MAPPER_BAG}"
    --sequence "${SEQUENCE}"
    --output "${BASELINE_DIR}"
    --runtime-sec "${BASELINE_RUNTIME_SEC}"
  )
  if [[ "${SKIP_BUILD}" == "true" ]]; then
    baseline_args+=(--skip-build)
  fi
  if [[ "${REBUILD}" == "true" ]]; then
    baseline_args+=(--rebuild)
  fi
  "${baseline_args[@]}"
fi

check_artifacts "${CURRENT_DIR}" "current"
check_artifacts "${BASELINE_DIR}" "baseline"

echo "[curated] writing reproduction report"
./scripts/reproduction_report.py \
  --baseline-dir "${BASELINE_DIR}" \
  --current-dir "${CURRENT_DIR}" \
  --sequence "${SEQUENCE}" \
  --metric-key debug_points \
  --trajectory-align first \
  --min-trajectory-coverage 0.4 \
  --max-association-dt 0.2 \
  --pointcloud-align centroid \
  --max-pointcloud-points 50000 \
  --max-nearest-m 0.5 \
  --max-chamfer-rmse-m 0.5 \
  --max-chamfer-mean-m 0.5 \
  --max-chamfer-max-m 0.5 \
  --max-unmatched-ratio 0.25 \
  --min-point-count-ratio 0.5 \
  --max-point-count-ratio 5.0 \
  --max-centroid-drift-m 0.5 \
  --output "${CURRENT_DIR}/reproduction_report.json" \
  --markdown "${CURRENT_DIR}/reproduction_report.md"
