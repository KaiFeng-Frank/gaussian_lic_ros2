#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CKPT_DIR="${ROOT_DIR}/external/Gaussian-LIC/ckpt"
DEFAULT_TENSORRT_ROOT="${HOME}/Software/TensorRT-10.9.0.34-cuda12.8"
if [[ ! -x "${DEFAULT_TENSORRT_ROOT}/bin/trtexec" ]]; then
  DEFAULT_TENSORRT_ROOT="${HOME}/Software/TensorRT-8.6.1.6"
fi
TENSORRT_ROOT="${TENSORRT_ROOT:-${DEFAULT_TENSORRT_ROOT}}"
if [[ -z "${TRTEXEC:-}" ]]; then
  if [[ -x "${TENSORRT_ROOT}/bin/trtexec" ]]; then
    TRTEXEC="${TENSORRT_ROOT}/bin/trtexec"
  else
    TRTEXEC="${TENSORRT_ROOT}/targets/x86_64-linux-gnu/bin/trtexec"
  fi
fi
SPNET_PYTHON="${SPNET_PYTHON:-python3}"
OUTPUT_DIR="${OUTPUT_DIR:-${HOME}/Software/TensorRT-engines}"
HEIGHT=512
WIDTH=640
DOWNLOAD=false
WEIGHTS="${CKPT_DIR}/Large_300.pth"
GDRIVE_ID="11dujPviL4pKLEXytXK0mEmPBNQDqgEak"

usage() {
  cat <<'EOF'
Usage: scripts/build_spnet_engine.sh [OPTIONS]

Export the upstream Gaussian-LIC SPNet checkpoint to ONNX and build a TensorRT
FP16 engine for the ROS2 depth-completion wrapper.

Options:
  --weights FILE       Path to Large_300.pth. Default: external/Gaussian-LIC/ckpt/Large_300.pth
  --output-dir DIR     Engine output directory. Default: ~/Software/TensorRT-engines
  --height N           Input height. Supported by upstream scripts: 512 or 480. Default: 512
  --width N            Input width. Default: 640
  --download           Download Large_300.pth from the upstream Google Drive link if missing.
  --python PYTHON      Python with torch/onnx and SPNet deps. Default: python3
  --trtexec FILE       TensorRT trtexec path. Default: $TENSORRT_ROOT/targets/x86_64-linux-gnu/bin/trtexec
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --weights)
      WEIGHTS="$2"
      shift 2
      ;;
    --output-dir)
      OUTPUT_DIR="$2"
      shift 2
      ;;
    --height)
      HEIGHT="$2"
      shift 2
      ;;
    --width)
      WIDTH="$2"
      shift 2
      ;;
    --download)
      DOWNLOAD=true
      shift
      ;;
    --python)
      SPNET_PYTHON="$2"
      shift 2
      ;;
    --trtexec)
      TRTEXEC="$2"
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

WEIGHTS="$(realpath -m "${WEIGHTS}")"
OUTPUT_DIR="$(realpath -m "${OUTPUT_DIR}")"
mkdir -p "${OUTPUT_DIR}" "${CKPT_DIR}"

if [[ "${WIDTH}" != "640" ]] || { [[ "${HEIGHT}" != "512" ]] && [[ "${HEIGHT}" != "480" ]]; }; then
  echo "Unsupported SPNet shape: ${HEIGHT}x${WIDTH}; upstream exports support 512x640 and 480x640" >&2
  exit 2
fi

if [[ ! -f "${WEIGHTS}" && "${DOWNLOAD}" == "true" ]]; then
  echo "[spnet] downloading Large_300.pth from upstream Google Drive"
  "${SPNET_PYTHON}" - "${WEIGHTS}" "${GDRIVE_ID}" <<'PY'
import html
import re
import sys
from pathlib import Path

import requests

output = Path(sys.argv[1])
file_id = sys.argv[2]
session = requests.Session()
url = "https://drive.google.com/uc"
response = session.get(url, params={"export": "download", "id": file_id}, stream=True, timeout=30)
token = None
for key, value in response.cookies.items():
    if key.startswith("download_warning"):
        token = value
        break
if token is None:
    match = re.search(r"confirm=([0-9A-Za-z_]+)", response.text[:8192])
    if match:
        token = match.group(1)
params = {"export": "download", "confirm": token, "id": file_id} if token else None
form_match = re.search(r'<form[^>]+id="download-form"[^>]+action="([^"]+)"[^>]*>(.*?)</form>', response.text, re.S)
if form_match:
    url = html.unescape(form_match.group(1))
    params = {}
    for name, value in re.findall(r'<input[^>]+name="([^"]+)"[^>]+value="([^"]*)"', form_match.group(2)):
        params[html.unescape(name)] = html.unescape(value)
if params:
    response = session.get(
        url,
        params=params,
        stream=True,
        timeout=30,
    )
response.raise_for_status()
output.parent.mkdir(parents=True, exist_ok=True)
tmp = output.with_suffix(output.suffix + ".tmp")
with tmp.open("wb") as handle:
    for chunk in response.iter_content(chunk_size=1024 * 1024):
        if chunk:
            handle.write(chunk)
if tmp.stat().st_size < 1024 * 1024:
    text = tmp.read_bytes()[:4096]
    tmp.unlink(missing_ok=True)
    raise SystemExit(f"download did not look like a checkpoint: {text!r}")
tmp.replace(output)
print(f"downloaded {output} bytes={output.stat().st_size}")
PY
fi

if [[ ! -f "${WEIGHTS}" ]]; then
  cat >&2 <<EOF
Missing SPNet checkpoint: ${WEIGHTS}

Download the upstream checkpoint and re-run, or allow this script to fetch it:
  scripts/build_spnet_engine.sh --download

Upstream checkpoint source:
  https://drive.google.com/file/d/${GDRIVE_ID}/view
EOF
  exit 2
fi

if [[ ! -x "${TRTEXEC}" ]]; then
  echo "Missing TensorRT trtexec: ${TRTEXEC}" >&2
  exit 2
fi

trt_version="$("${TRTEXEC}" --help 2>&1 | head -20 | tr '\n' ' ' || true)"
trt_token="$(printf '%s\n' "${trt_version}" | sed -n 's/.*TensorRT v\([0-9][0-9]*\).*/\1/p' | head -1)"
if [[ -n "${trt_token}" ]]; then
  if (( trt_token >= 100000 )); then
    trt_major="$((trt_token / 10000))"
  else
    trt_major="$((trt_token / 1000))"
  fi
else
  trt_major="$(printf '%s\n' "${trt_version}" | sed -n 's/.*TensorRT[^0-9]*\([0-9][0-9]*\).*/\1/p' | head -1)"
fi
gpu_cc="$(nvidia-smi --query-gpu=compute_cap --format=csv,noheader 2>/dev/null | head -1 || true)"
if [[ "${gpu_cc}" == 12.* && -n "${trt_major}" && "${trt_major}" -lt 10 ]]; then
  cat >&2 <<EOF
TensorRT ${trt_major}.x at ${TRTEXEC} does not support the local sm_120 GPU (${gpu_cc}).
Use a TensorRT 10.x CUDA 12.x SDK, for example the local unpacked SDK:
  TENSORRT_ROOT=${HOME}/Software/TensorRT-10.9.0.34-cuda12.8 scripts/build_spnet_engine.sh
EOF
  exit 2
fi

"${SPNET_PYTHON}" - <<'PY'
import importlib.util
import sys

missing = [name for name in ("torch", "onnx") if importlib.util.find_spec(name) is None]
if missing:
    raise SystemExit("SPNet export Python is missing modules: " + ", ".join(missing))
PY

onnx_name="spnet_${HEIGHT}_${WIDTH}.onnx"
engine_name="spnet_${HEIGHT}_${WIDTH}_fp16.engine"
onnx_path="${OUTPUT_DIR}/${onnx_name}"
engine_path="${OUTPUT_DIR}/${engine_name}"
echo "[spnet] exporting ${onnx_path}"
PYTHONPATH="${CKPT_DIR}:${PYTHONPATH:-}" "${SPNET_PYTHON}" - \
  "${WEIGHTS}" "${onnx_path}" "${HEIGHT}" "${WIDTH}" <<'PY'
import sys
from pathlib import Path

import torch
from SPNet.src.networks import V2Net

weights = Path(sys.argv[1])
onnx_path = Path(sys.argv[2])
height = int(sys.argv[3])
width = int(sys.argv[4])

dims = [192, 384, 768, 1536]
depths = [3, 3, 27, 3]
dp_rate = 0.2
norm_type = "CNX"
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

net = V2Net(dims, depths, dp_rate, norm_type).to(device).eval()
checkpoint = torch.load(weights, map_location=device)
state_dict = checkpoint["network"] if isinstance(checkpoint, dict) and "network" in checkpoint else checkpoint
net.load_state_dict(state_dict)

rgb = torch.randn(1, 3, height, width, device=device)
depth = torch.randn(1, 1, height, width, device=device)
mask = torch.ones_like(depth)
mask = torch.where(depth == 0, torch.zeros_like(mask), mask)

onnx_path.parent.mkdir(parents=True, exist_ok=True)
torch.onnx.export(
    net,
    (rgb, depth, mask),
    str(onnx_path),
    input_names=["rgb", "depth", "mask"],
    output_names=["pred"],
    dynamic_axes={
        "rgb": {0: "batch", 2: "height", 3: "width"},
        "depth": {0: "batch", 2: "height", 3: "width"},
        "mask": {0: "batch", 2: "height", 3: "width"},
        "pred": {0: "batch", 2: "height", 3: "width"},
    },
    opset_version=17,
)
print(f"exported {onnx_path} using device={device}")
PY

for trt_lib in \
  "${TENSORRT_ROOT}/targets/x86_64-linux-gnu/lib" \
  "${TENSORRT_ROOT}/lib" \
  "${TENSORRT_ROOT}/usr/lib/x86_64-linux-gnu"; do
  if [[ -d "${trt_lib}" ]]; then
    export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:${trt_lib}"
  fi
done
cublas11_lib="$("${SPNET_PYTHON}" - <<'PY' || true
from pathlib import Path
try:
    import nvidia.cublas
except Exception:
    raise SystemExit(0)
base = Path(nvidia.cublas.__file__).resolve().parent / "lib"
if (base / "libcublas.so.11").is_file():
    print(base)
PY
)"
if [[ -n "${cublas11_lib}" && -d "${cublas11_lib}" ]]; then
  export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:${cublas11_lib}"
fi
for nvidia_module in cudnn cuda_nvrtc; do
  module_lib="$("${SPNET_PYTHON}" - "${nvidia_module}" <<'PY' || true
import importlib
import sys
from pathlib import Path

module_name = "nvidia." + sys.argv[1]
try:
    module = importlib.import_module(module_name)
except Exception:
    raise SystemExit(0)
base = Path(module.__file__).resolve().parent / "lib"
if base.is_dir():
    print(base)
PY
)"
  if [[ -n "${module_lib}" && -d "${module_lib}" ]]; then
    export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:${module_lib}"
  fi
done

echo "[spnet] building ${engine_path}"
"${TRTEXEC}" \
  --onnx="${onnx_path}" \
  --saveEngine="${engine_path}" \
  --fp16 \
  --optShapes=rgb:1x3x${HEIGHT}x${WIDTH},depth:1x1x${HEIGHT}x${WIDTH},mask:1x1x${HEIGHT}x${WIDTH}

echo "[spnet] engine ready: ${engine_path}"
