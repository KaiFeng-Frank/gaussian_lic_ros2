#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
SKIP_BUILD=false
CHECK_TORCH=false
FULL_PROFILES=false
BAG_PATH="${ROOT_DIR}/bags/synthetic_gs_demo"
FRONTEND_RAW_BAG_PATH="${ROOT_DIR}/bags/synthetic_frontend_raw_demo"
FRONTEND_RAW_VISUAL_BAG_PATH="${ROOT_DIR}/bags/synthetic_frontend_raw_visual_demo"

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
bash -n scripts/build_jazzy.sh scripts/build_ros2.sh scripts/create_synthetic_bag.sh scripts/fetch_upstreams.sh \
  scripts/ci_replay_smoke.sh scripts/smoke_test.sh scripts/upstream_baseline_build_attempt.sh \
  scripts/tracking_smoke_test.sh scripts/verify_artifact_gates.sh scripts/verify_workspace.sh

echo "[verify] python syntax"
/usr/bin/python3 - <<'PY'
from pathlib import Path
import ast

paths = [
    Path("src/gaussian_lic_bringup/launch/run_bag.launch.py"),
    Path("src/gaussian_lic_bringup/setup.py"),
]
paths.extend(sorted(Path("scripts").glob("*.py")))
paths.extend(sorted(Path("src/gaussian_lic_tools/gaussian_lic_tools").glob("*.py")))
for path in paths:
    ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    print(path)
PY

echo "[verify] profile yaml"
./scripts/check_profiles.py

echo "[verify] ros2 semantic contract"
./scripts/check_ros2_semantics_contract.py

echo "[verify] strict documentation status"
./scripts/check_strict_doc_status.py

if [[ "${SKIP_BUILD}" != "true" ]]; then
  echo "[verify] build"
  ./scripts/build_ros2.sh
fi

if [[ ! -f "${BAG_PATH}/metadata.yaml" ]]; then
  echo "[verify] creating synthetic bag at ${BAG_PATH}"
  ./scripts/create_synthetic_bag.sh --output "${BAG_PATH}" --duration 4
fi

echo "[verify] creating frontend raw synthetic bag at ${FRONTEND_RAW_BAG_PATH}"
./scripts/create_synthetic_bag.sh \
  --frontend-raw \
  --frontend-raw-odometry \
  --output "${FRONTEND_RAW_BAG_PATH}" \
  --duration 4

echo "[verify] creating frontend raw visual synthetic bag at ${FRONTEND_RAW_VISUAL_BAG_PATH}"
./scripts/create_synthetic_bag.sh \
  --frontend-raw \
  --frontend-raw-odometry \
  --output "${FRONTEND_RAW_VISUAL_BAG_PATH}" \
  --duration 4 \
  --image-width 32 \
  --image-height 32 \
  --image-pattern gaussian \
  --image-shift-x 1.25 \
  --image-shift-y -0.35

echo "[verify] bag smoke"
./scripts/smoke_test.sh --bag "${BAG_PATH}" --tf

echo "[verify] minimal bag smoke"
./scripts/smoke_test.sh --bag "${BAG_PATH}" --minimal-inputs

echo "[verify] offline artifact extraction"
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source install/setup.bash
set -u
ros2 run gaussian_lic_tools gaussian_lic_bag_check \
  --bag "${BAG_PATH}" \
  --json \
  >/tmp/gaussian_lic_bag_contract_verify.json
rg -q '"contract_ok": true' /tmp/gaussian_lic_bag_contract_verify.json
ros2 run gaussian_lic_tools gaussian_lic_bag_check \
  --bag "${BAG_PATH}" \
  --contract mapper_minimal \
  --json \
  >/tmp/gaussian_lic_bag_minimal_contract_verify.json
rg -q '"contract_ok": true' /tmp/gaussian_lic_bag_minimal_contract_verify.json
ros2 run gaussian_lic_tools gaussian_lic_bag_check \
  --bag "${FRONTEND_RAW_BAG_PATH}" \
  --contract frontend_raw \
  --json \
  >/tmp/gaussian_lic_bag_frontend_raw_contract_verify.json
rg -q '"contract_ok": true' /tmp/gaussian_lic_bag_frontend_raw_contract_verify.json
rg -q '"frontend_pose_source"' /tmp/gaussian_lic_bag_frontend_raw_contract_verify.json
ros2 run gaussian_lic_tools gaussian_lic_bag_check \
  --bag "${FRONTEND_RAW_BAG_PATH}" \
  --contract frontend_sensor_raw \
  --json \
  >/tmp/gaussian_lic_bag_frontend_sensor_raw_contract_verify.json
rg -q '"contract_ok": true' /tmp/gaussian_lic_bag_frontend_sensor_raw_contract_verify.json
./scripts/rosbag2_timing_audit.py \
  --bag "${FRONTEND_RAW_VISUAL_BAG_PATH}" \
  --required-topic /imu \
  --required-topic /camera/image \
  --required-topic /livox/lidar \
  --strict-storage \
  >/tmp/gaussian_lic_frontend_raw_visual_timing_audit.txt
rm -rf /tmp/gaussian_lic_offline_verify
ros2 run gaussian_lic_tools gaussian_lic_offline \
  --bag "${BAG_PATH}" \
  --output /tmp/gaussian_lic_offline_verify \
  >/tmp/gaussian_lic_offline_verify.json
test -f /tmp/gaussian_lic_offline_verify/trajectory.tum
test -f /tmp/gaussian_lic_offline_verify/point_cloud_debug.ply
test -f /tmp/gaussian_lic_offline_verify/metrics.json
rg -q " 255 32 16$" /tmp/gaussian_lic_offline_verify/point_cloud_debug.ply
rg -q '"topic_hz"' /tmp/gaussian_lic_offline_verify/metrics.json
rg -q '"path_length_m"' /tmp/gaussian_lic_offline_verify/metrics.json
rg -q '"tracking_status"' /tmp/gaussian_lic_offline_verify/metrics.json
rg -q '"points_with_color": [1-9]' /tmp/gaussian_lic_offline_verify/metrics.json

echo "[verify] artifact gates"
./scripts/verify_artifact_gates.sh

echo "[verify] live smoke"
./scripts/smoke_test.sh --tf

echo "[verify] frontend adapter smoke"
./scripts/smoke_test.sh --frontend-adapter --tf

echo "[verify] native tracking smoke"
./scripts/tracking_smoke_test.sh --bag "${FRONTEND_RAW_VISUAL_BAG_PATH}"

echo "[verify] native tracking smoke without IMU factor trigger"
./scripts/tracking_smoke_test.sh --bag "${FRONTEND_RAW_VISUAL_BAG_PATH}" --no-imu-factor

echo "[verify] image color fallback smoke"
./scripts/smoke_test.sh --image-color-fallback-check

echo "[verify] optional depth smoke"
./scripts/smoke_test.sh --optional-depth-check

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
  GAUSSIAN_LIC_ENABLE_TORCH=ON ./scripts/build_ros2.sh --packages-select gaussian_lic_mapping
  set +u
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  source install/setup.bash
  set -u
  echo "[verify] torch backend probe"
  ros2 run gaussian_lic_mapping torch_backend_probe
  echo "[verify] torch smoke"
  ./scripts/smoke_test.sh --bag "${BAG_PATH}" --torch --tf
  echo "[verify] torch rasterizer preview smoke"
  ./scripts/smoke_test.sh --torch --tf --render-mode rasterizer --timeout 12
fi

echo "[verify] passed"
