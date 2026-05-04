#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SKIP_BUILD=false
CHECK_TORCH=false
FULL_PROFILES=false
BAG_PATH="${ROOT_DIR}/bags/synthetic_gs_demo"

usage() {
  cat <<'EOF'
Usage: scripts/verify_workspace.sh [--skip-build] [--torch] [--full-profiles] [--bag DIR]

Runs local verification for the Gaussian-LIC ROS2 workspace.

Options:
  --skip-build     Reuse the current install/ tree.
  --torch          Rebuild mapping with libtorch enabled and run torch smoke checks.
  --full-profiles  Smoke-test every packaged dataset profile with the synthetic bag.
  --bag DIR        Synthetic rosbag2 directory. Default: bags/synthetic_gs_demo.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --skip-build)
      SKIP_BUILD=true
      shift
      ;;
    --torch)
      CHECK_TORCH=true
      shift
      ;;
    --full-profiles)
      FULL_PROFILES=true
      shift
      ;;
    --bag)
      BAG_PATH="$2"
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

cd "${ROOT_DIR}"

echo "[verify] bash syntax"
bash -n scripts/build_jazzy.sh scripts/create_synthetic_bag.sh scripts/fetch_upstreams.sh \
  scripts/smoke_test.sh scripts/verify_workspace.sh

echo "[verify] python syntax"
/usr/bin/python3 - <<'PY'
from pathlib import Path
import ast

paths = [
    Path("scripts/check_profiles.py"),
    Path("scripts/convert_ros1_bag_to_rosbag2.py"),
    Path("scripts/perf_regression.py"),
    Path("src/gaussian_lic_bringup/launch/run_bag.launch.py"),
    Path("src/gaussian_lic_bringup/setup.py"),
]
paths.extend(sorted(Path("src/gaussian_lic_tools/gaussian_lic_tools").glob("*.py")))
for path in paths:
    ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    print(path)
PY

echo "[verify] profile yaml"
./scripts/check_profiles.py

if [[ "${SKIP_BUILD}" != "true" ]]; then
  echo "[verify] build"
  ./scripts/build_jazzy.sh
fi

if [[ ! -f "${BAG_PATH}/metadata.yaml" ]]; then
  echo "[verify] creating synthetic bag at ${BAG_PATH}"
  ./scripts/create_synthetic_bag.sh --output "${BAG_PATH}" --duration 4
fi

echo "[verify] bag smoke"
./scripts/smoke_test.sh --bag "${BAG_PATH}" --tf

echo "[verify] offline artifact extraction"
set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u
ros2 run gaussian_lic_tools gaussian_lic_bag_check \
  --bag "${BAG_PATH}" \
  --json \
  >/tmp/gaussian_lic_bag_contract_verify.json
rg -q '"contract_ok": true' /tmp/gaussian_lic_bag_contract_verify.json
rm -rf /tmp/gaussian_lic_offline_verify
ros2 run gaussian_lic_tools gaussian_lic_offline \
  --bag "${BAG_PATH}" \
  --output /tmp/gaussian_lic_offline_verify \
  >/tmp/gaussian_lic_offline_verify.json
test -f /tmp/gaussian_lic_offline_verify/trajectory.tum
test -f /tmp/gaussian_lic_offline_verify/point_cloud_debug.ply
test -f /tmp/gaussian_lic_offline_verify/metrics.json

echo "[verify] live smoke"
./scripts/smoke_test.sh --tf

echo "[verify] image color fallback smoke"
./scripts/smoke_test.sh --image-color-fallback-check

if [[ "${FULL_PROFILES}" == "true" ]]; then
  for profile in fastlivo fastlivo2 m2dgr mcd r3live; do
    echo "[verify] profile smoke: ${profile}"
    ./scripts/smoke_test.sh \
      --bag "${BAG_PATH}" \
      --config "src/gaussian_lic_bringup/config/${profile}.yaml" \
      --render-mode debug_cpu \
      --skip-rendered-data-check \
      --tf
  done
fi

if [[ "${CHECK_TORCH}" == "true" ]]; then
  echo "[verify] torch build"
  GAUSSIAN_LIC_ENABLE_TORCH=ON ./scripts/build_jazzy.sh --packages-select gaussian_lic_mapping
  set +u
  source /opt/ros/jazzy/setup.bash
  source install/setup.bash
  set -u
  echo "[verify] torch backend probe"
  ros2 run gaussian_lic_mapping torch_backend_probe
  echo "[verify] torch smoke"
  ./scripts/smoke_test.sh --bag "${BAG_PATH}" --torch --tf
fi

echo "[verify] passed"
