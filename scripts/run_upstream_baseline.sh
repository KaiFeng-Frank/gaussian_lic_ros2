#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${GAUSSIAN_LIC_NOETIC_IMAGE:-gaussian_lic_noetic_baseline:latest}"
SOFTWARE_DIR="${GAUSSIAN_LIC_SOFTWARE_DIR:-/home/frank/Software}"
HOST_CUDA="${GAUSSIAN_LIC_HOST_CUDA:-/usr/local/cuda-12.8}"
CONTAINER_CUDA="${GAUSSIAN_LIC_CONTAINER_CUDA:-/usr/local/cuda-11.7}"
COMPAT_LIBSTDCXX="${GAUSSIAN_LIC_COMPAT_LIBSTDCXX:-/home/frank/miniconda3/lib/libstdc++.so.6}"
OPENCV_DIR="${GAUSSIAN_LIC_OPENCV_DIR:-${SOFTWARE_DIR}/opencv/opencv-4.10.0/build}"
WORKSPACE_DIR="${GAUSSIAN_LIC_BASELINE_WORKSPACE:-/home/frank/.cache/gaussian_lic_ros2/noetic_catkin_gaussian}"
SEQUENCE="CBD_Building_01"
BAG_PATH=""
OUTPUT_DIR=""
CONFIG_NAME="fastlivo2.yaml"
RUN_TIMEOUT_SEC=0
PLAY_DELAY_SEC=3
REBUILD=false
SKIP_BUILD=false
CHECK_ONLY=false
USE_GPU=true
DOCKER_DNS="${GAUSSIAN_LIC_DOCKER_DNS:-8.8.8.8}"

usage() {
  cat <<'EOF'
Usage: scripts/run_upstream_baseline.sh [options]

Build if needed, run the ROS1 upstream Gaussian-LIC/Gaussian-LIC2 mapper in
Docker, and archive baseline artifacts shaped for reproduction_report.py.

Options:
  --bag FILE              ROS1 mapper-contract bag containing /points_for_gs,
                          /pose_for_gs, /image_for_gs, and /depth_for_gs.
                          If omitted, the script waits for live ROS1 topics.
  --sequence NAME         Sequence name. Default: CBD_Building_01.
  --output DIR            Baseline output directory. Default:
                          baseline/fastlivo2/<sequence>.
  --workspace DIR         Persistent catkin workspace. Default:
                          /home/frank/.cache/gaussian_lic_ros2/noetic_catkin_gaussian.
  --opencv-dir DIR        OpenCV build directory. Default: OpenCV 4.10 fallback.
  --runtime-sec SEC       Stop runtime after SEC seconds. Default: wait for bag/end.
  --rebuild               Recreate the persistent upstream workspace before running.
  --skip-build            Require an existing built workspace.
  --check-only            Validate local prerequisites and exit.
  --no-gpu                Omit GPU devices. Runtime normally needs GPU.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --bag)
      BAG_PATH="$2"
      shift 2
      ;;
    --sequence)
      SEQUENCE="$2"
      shift 2
      ;;
    --output)
      OUTPUT_DIR="$2"
      shift 2
      ;;
    --workspace)
      WORKSPACE_DIR="$2"
      shift 2
      ;;
    --opencv-dir)
      OPENCV_DIR="$2"
      shift 2
      ;;
    --runtime-sec)
      RUN_TIMEOUT_SEC="$2"
      shift 2
      ;;
    --rebuild)
      REBUILD=true
      shift
      ;;
    --skip-build)
      SKIP_BUILD=true
      shift
      ;;
    --check-only)
      CHECK_ONLY=true
      shift
      ;;
    --no-gpu)
      USE_GPU=false
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
OUTPUT_DIR="${OUTPUT_DIR:-${ROOT_DIR}/baseline/fastlivo2/${SEQUENCE}}"
OUTPUT_DIR="$(realpath -m "${OUTPUT_DIR}")"
WORKSPACE_DIR="$(realpath -m "${WORKSPACE_DIR}")"
SOFTWARE_DIR="$(realpath -m "${SOFTWARE_DIR}")"
OPENCV_DIR="$(realpath -m "${OPENCV_DIR}")"
HOST_CUDA="$(realpath -m "${HOST_CUDA}")"
COMPAT_LIBSTDCXX="$(realpath -m "${COMPAT_LIBSTDCXX}")"

if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
  echo "Missing Docker image: ${IMAGE}" >&2
  exit 2
fi
for path in "${SOFTWARE_DIR}" "${OPENCV_DIR}" "${HOST_CUDA}" "${ROOT_DIR}/external/Gaussian-LIC"; do
  if [[ ! -e "${path}" ]]; then
    echo "Missing required path: ${path}" >&2
    exit 2
  fi
done
if [[ ! -f "${COMPAT_LIBSTDCXX}" ]]; then
  echo "Missing compatible libstdc++ for libtorch runtime: ${COMPAT_LIBSTDCXX}" >&2
  exit 2
fi
for path in \
  "${SOFTWARE_DIR}/TensorRT-8.6.1.6/include/NvInfer.h" \
  "${SOFTWARE_DIR}/TensorRT-8.6.1.6/lib/libnvinfer.so" \
  "${SOFTWARE_DIR}/TensorRT-8.6.1.6/lib/libnvonnxparser.so" \
  "${SOFTWARE_DIR}/TensorRT-8.6.1.6/lib/libnvparsers.so" \
  "${SOFTWARE_DIR}/TensorRT-8.6.1.6/lib/libnvinfer_plugin.so"; do
  if [[ ! -e "${path}" ]]; then
    echo "Missing TensorRT artifact: ${path}" >&2
    exit 2
  fi
done
if [[ -n "${BAG_PATH}" ]]; then
  BAG_PATH="$(realpath -m "${BAG_PATH}")"
  if [[ ! -f "${BAG_PATH}" ]]; then
    echo "Missing ROS1 bag: ${BAG_PATH}" >&2
    exit 2
  fi
fi

case "${OUTPUT_DIR}" in
  "${ROOT_DIR}"/*)
    OUTPUT_IN_CONTAINER="/work/${OUTPUT_DIR#"${ROOT_DIR}/"}"
    ;;
  *)
    echo "Output directory must be inside the repository: ${OUTPUT_DIR}" >&2
    exit 2
    ;;
esac
case "${OPENCV_DIR}" in
  "${SOFTWARE_DIR}"/*)
    OPENCV_IN_CONTAINER="/root/Software/${OPENCV_DIR#"${SOFTWARE_DIR}/"}"
    ;;
  *)
    echo "OpenCV directory must be under ${SOFTWARE_DIR}: ${OPENCV_DIR}" >&2
    exit 2
    ;;
esac

if [[ "${CHECK_ONLY}" == "true" ]]; then
  echo "upstream baseline prerequisites OK: image=${IMAGE} opencv=${OPENCV_DIR} tensorrt=${SOFTWARE_DIR}/TensorRT-8.6.1.6"
  exit 0
fi

mkdir -p "${WORKSPACE_DIR}" "$(dirname "${OUTPUT_DIR}")"

docker_args=(--rm)
if [[ "${USE_GPU}" == "true" ]]; then
  if docker info --format '{{json .Runtimes}}' 2>/dev/null | rg -q '"nvidia"'; then
    docker_args+=(--gpus all)
  else
    for device in /dev/nvidia0 /dev/nvidiactl /dev/nvidia-uvm /dev/nvidia-uvm-tools /dev/nvidia-modeset; do
      if [[ -e "${device}" ]]; then
        docker_args+=(--device "${device}:${device}")
      fi
    done
    for library in \
      /usr/lib/x86_64-linux-gnu/libcuda.so \
      /usr/lib/x86_64-linux-gnu/libcuda.so.1 \
      /usr/lib/x86_64-linux-gnu/libcuda.so.* \
      /usr/lib/x86_64-linux-gnu/libnvidia-ml.so \
      /usr/lib/x86_64-linux-gnu/libnvidia-ml.so.1 \
      /usr/lib/x86_64-linux-gnu/libnvidia-ml.so.*; do
      if [[ -e "${library}" ]]; then
        docker_args+=(-v "${library}:${library}:ro")
      fi
    done
  fi
fi
if [[ -n "${DOCKER_DNS}" ]]; then
  docker_args+=(--dns "${DOCKER_DNS}")
fi
docker_args+=(
  -v "${ROOT_DIR}:/work"
  -v "${WORKSPACE_DIR}:/baseline_ws"
  -v "${SOFTWARE_DIR}:/root/Software:ro"
  -v "${SOFTWARE_DIR}:/home/frank/Software:ro"
  -v "${HOST_CUDA}:${CONTAINER_CUDA}:ro"
  -v "${COMPAT_LIBSTDCXX}:/compat/libstdc++.so.6:ro"
  -w /work
)

BAG_IN_CONTAINER=""
if [[ -n "${BAG_PATH}" ]]; then
  BAG_PARENT="$(dirname "${BAG_PATH}")"
  BAG_NAME="$(basename "${BAG_PATH}")"
  docker_args+=(-v "${BAG_PARENT}:/input_bag:ro")
  BAG_IN_CONTAINER="/input_bag/${BAG_NAME}"
fi

rebuild_flag=0
if [[ "${REBUILD}" == "true" ]]; then
  rebuild_flag=1
fi
skip_build_flag=0
if [[ "${SKIP_BUILD}" == "true" ]]; then
  skip_build_flag=1
fi
HOST_UID="$(id -u)"
HOST_GID="$(id -g)"

docker run "${docker_args[@]}" "${IMAGE}" bash -lc "set -euo pipefail
export PATH=/usr/local/bin:${CONTAINER_CUDA}/bin:\${PATH}
export LD_LIBRARY_PATH=${CONTAINER_CUDA}/lib64:\${LD_LIBRARY_PATH:-}
export LD_PRELOAD=/compat/libstdc++.so.6\${LD_PRELOAD:+:\${LD_PRELOAD}}
source /opt/ros/noetic/setup.bash

if [[ '${rebuild_flag}' == '1' ]]; then
  rm -rf /baseline_ws/build /baseline_ws/devel /baseline_ws/src
fi
if [[ ! -x /baseline_ws/devel/lib/gaussian_lic/gs_mapping ]]; then
  if [[ '${skip_build_flag}' == '1' ]]; then
    echo 'Missing built upstream workspace and --skip-build was requested' >&2
    exit 2
  fi
  rm -rf /baseline_ws/build /baseline_ws/devel /baseline_ws/src
  mkdir -p /baseline_ws/src
  cp -a /work/external/Gaussian-LIC /baseline_ws/src/Gaussian-LIC
  cd /baseline_ws/src/Gaussian-LIC
  sed -i 's|^set(OpenCV_DIR .*)|set(OpenCV_DIR ${OPENCV_IN_CONTAINER})|' CMakeLists.txt
  for file in \
    src/simple-knn/simple_knn.cu \
    src/rasterizer/cuda_rasterizer/rasterizer_impl.cu; do
    if ! grep -q '#include <cfloat>' \"\${file}\"; then
      sed -i '1i#include <cfloat>' \"\${file}\"
    fi
  done
  python3 /work/scripts/patch_upstream_baseline.py /baseline_ws/src/Gaussian-LIC
  cd /baseline_ws
  catkin_make \
    -DCMAKE_BUILD_TYPE=Release \
    -DCUDA_TOOLKIT_ROOT_DIR=${CONTAINER_CUDA} \
    -DCMAKE_CUDA_ARCHITECTURES=86
fi

source /baseline_ws/devel/setup.bash
rm -rf '${OUTPUT_IN_CONTAINER}'
mkdir -p '${OUTPUT_IN_CONTAINER}'
mkdir -p '${OUTPUT_IN_CONTAINER}/upstream_result'
cp /baseline_ws/src/Gaussian-LIC/config/${CONFIG_NAME} '${OUTPUT_IN_CONTAINER}/runtime_config.yaml'
sed -i 's/^depth_completion:.*/depth_completion: false/' '${OUTPUT_IN_CONTAINER}/runtime_config.yaml'

roscore >'${OUTPUT_IN_CONTAINER}/roscore.log' 2>&1 &
roscore_pid=\$!
cleanup() {
  for pid in \${mapper_pid:-} \${record_pid:-} \${player_pid:-} \${roscore_pid:-}; do
    if [[ -n \"\${pid}\" ]]; then
      kill -INT \"\${pid}\" >/dev/null 2>&1 || true
    fi
  done
  sleep 2
  for pid in \${mapper_pid:-} \${record_pid:-} \${player_pid:-} \${roscore_pid:-}; do
    if [[ -n \"\${pid}\" ]]; then
      kill -TERM \"\${pid}\" >/dev/null 2>&1 || true
    fi
  done
}
trap cleanup EXIT
for _ in \$(seq 1 60); do
  if rostopic list >/dev/null 2>&1; then
    break
  fi
  sleep 0.5
done

rosbag record -O '${OUTPUT_IN_CONTAINER}/input_contract.bag' \
  /pose_for_gs /points_for_gs /image_for_gs /depth_for_gs \
  >'${OUTPUT_IN_CONTAINER}/record.log' 2>&1 &
record_pid=\$!

rosrun gaussian_lic gs_mapping \
  _config_path:='${OUTPUT_IN_CONTAINER}/runtime_config.yaml' \
  _result_path:='${OUTPUT_IN_CONTAINER}/upstream_result' \
  _lpips_path:=/baseline_ws/src/Gaussian-LIC/src/lpips \
  >'${OUTPUT_IN_CONTAINER}/run.log' 2>&1 &
mapper_pid=\$!

sleep '${PLAY_DELAY_SEC}'
if [[ -n '${BAG_IN_CONTAINER}' ]]; then
  rosbag play '${BAG_IN_CONTAINER}' --clock >'${OUTPUT_IN_CONTAINER}/play.log' 2>&1 &
  player_pid=\$!
  wait \${player_pid} || true
  player_pid=''
fi

wait_limit='${RUN_TIMEOUT_SEC}'
if [[ \"\${wait_limit}\" == '0' ]]; then
  wait_limit=120
fi
for _ in \$(seq 1 \"\${wait_limit}\"); do
  if [[ -f '${OUTPUT_IN_CONTAINER}/upstream_result/point_cloud.ply' ]]; then
    break
  fi
  if ! kill -0 \${mapper_pid} >/dev/null 2>&1; then
    break
  fi
  sleep 1
done
if [[ ! -f '${OUTPUT_IN_CONTAINER}/upstream_result/point_cloud.ply' ]]; then
  echo 'Timed out waiting for upstream point_cloud.ply' >&2
fi

cleanup
trap - EXIT

test -f '${OUTPUT_IN_CONTAINER}/upstream_result/point_cloud.ply'
cp '${OUTPUT_IN_CONTAINER}/upstream_result/point_cloud.ply' '${OUTPUT_IN_CONTAINER}/point_cloud.ply'
if [[ -d '${OUTPUT_IN_CONTAINER}/upstream_result/render' ]]; then
  cp -a '${OUTPUT_IN_CONTAINER}/upstream_result/render' '${OUTPUT_IN_CONTAINER}/renders'
else
  mkdir -p '${OUTPUT_IN_CONTAINER}/renders'
fi
chown -R '${HOST_UID}:${HOST_GID}' '${OUTPUT_IN_CONTAINER}'
"

PYTHONPATH=/home/frank/.cache/gaussian_lic_ros2/rosbags-venv/lib/python3.12/site-packages \
  /usr/bin/python3 "${ROOT_DIR}/src/gaussian_lic_tools/gaussian_lic_tools/offline.py" \
  --bag "${OUTPUT_DIR}/input_contract.bag" \
  --bag-format ros1 \
  --output "${OUTPUT_DIR}/offline" \
  --pose-topic /pose_for_gs \
  --pointcloud-topic /points_for_gs \
  >"${OUTPUT_DIR}/offline_stdout.json"

cp "${OUTPUT_DIR}/offline/trajectory.tum" "${OUTPUT_DIR}/trajectory.tum"

/usr/bin/python3 - "${OUTPUT_DIR}" "${SEQUENCE}" <<'PY'
import json
import re
import sys
from pathlib import Path

root = Path(sys.argv[1])
sequence = sys.argv[2]
offline = json.loads((root / 'offline' / 'metrics.json').read_text(encoding='utf-8'))
run_log = (root / 'run.log').read_text(encoding='utf-8', errors='replace')

def find_float(label):
    match = re.search(rf'\[{re.escape(label)}\]\s+([-+0-9.eE]+)', run_log)
    return float(match.group(1)) if match else None

def find_int(label):
    match = re.search(rf'\[{re.escape(label)}\]\s+([0-9]+)', run_log)
    return int(match.group(1)) if match else None

metrics = {
    **offline,
    'schema': 'gaussian_lic_ros1_upstream_baseline/v1',
    'sequence': sequence,
    'run_log': str((root / 'run.log').resolve()),
    'runtime': {
        'total_mapping_time_s': find_float('Total Mapping Time'),
        'total_adding_time_s': find_float('Total Adding Time'),
        'total_extending_time_s': find_float('Total Extending Time'),
        'forward_s': find_float('Forward'),
        'backward_s': find_float('Backward'),
        'step_s': find_float('Step'),
        'cpu_to_gpu_s': find_float('CPU2GPU'),
    },
    'quality': {
        'final_gaussians': find_int('Number of Final Gaussians'),
        'train_psnr': find_float('Training View PSNR'),
        'train_ssim': find_float('Training View SSIM'),
        'train_lpips': find_float('Training View LPIPS'),
        'novel_psnr': find_float('In-Sequence Novel View PSNR'),
        'novel_ssim': find_float('In-Sequence Novel View SSIM'),
        'novel_lpips': find_float('In-Sequence Novel View LPIPS'),
    },
    'outputs': {
        **offline.get('outputs', {}),
        'trajectory_tum': str((root / 'trajectory.tum').resolve()),
        'point_cloud_ply': str((root / 'point_cloud.ply').resolve()),
        'renders': str((root / 'renders').resolve()),
        'upstream_result': str((root / 'upstream_result').resolve()),
    },
}
(root / 'metrics.json').write_text(json.dumps(metrics, indent=2, sort_keys=True) + '\n', encoding='utf-8')
PY

./scripts/baseline_manifest.py \
  --baseline "${OUTPUT_DIR}" \
  --sequence "${SEQUENCE}" \
  --write

echo "[baseline] artifacts: ${OUTPUT_DIR}"
echo "[baseline] passed"
