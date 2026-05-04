#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
TENSORRT_ROOT="${TENSORRT_ROOT:-${HOME}/Software/TensorRT-8.6.1.6}"

cd "${ROOT_DIR}"

if [[ ! -f "${TENSORRT_ROOT}/include/NvInfer.h" ]]; then
  echo "Missing TensorRT header: ${TENSORRT_ROOT}/include/NvInfer.h" >&2
  exit 2
fi
if [[ ! -e "${TENSORRT_ROOT}/lib/libnvinfer.so" && ! -e "${TENSORRT_ROOT}/lib64/libnvinfer.so" ]]; then
  echo "Missing TensorRT library under ${TENSORRT_ROOT}/lib or ${TENSORRT_ROOT}/lib64" >&2
  exit 2
fi

export GAUSSIAN_LIC_ENABLE_TENSORRT=ON
export TENSORRT_ROOT

./scripts/build_ros2.sh "$@"
