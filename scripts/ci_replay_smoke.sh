#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BAG_PATH="${ROOT_DIR}/bags/ci_synthetic_gs_demo"
DURATION_SEC=4
TIMEOUT_SEC=20
PUBLISH_TF=true

usage() {
  cat <<'EOF'
Usage: scripts/ci_replay_smoke.sh [--bag DIR] [--duration SEC] [--timeout SEC] [--no-tf]

Records a short synthetic rosbag2 sequence, then replays it through the mapper
smoke test in both full-contract and mapper_minimal modes.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --bag)
      BAG_PATH="$2"
      shift 2
      ;;
    --duration)
      DURATION_SEC="$2"
      shift 2
      ;;
    --timeout)
      TIMEOUT_SEC="$2"
      shift 2
      ;;
    --no-tf)
      PUBLISH_TF=false
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

smoke_args=(--bag "${BAG_PATH}" --timeout "${TIMEOUT_SEC}")
if [[ "${PUBLISH_TF}" == "true" ]]; then
  smoke_args+=(--tf)
fi

echo "[ci-smoke] recording synthetic bag: ${BAG_PATH}"
./scripts/create_synthetic_bag.sh \
  --output "${BAG_PATH}" \
  --duration "${DURATION_SEC}"

echo "[ci-smoke] replaying full mapper contract"
./scripts/smoke_test.sh "${smoke_args[@]}"

echo "[ci-smoke] replaying mapper_minimal contract"
./scripts/smoke_test.sh \
  --bag "${BAG_PATH}" \
  --minimal-inputs \
  --timeout "${TIMEOUT_SEC}"

echo "[ci-smoke] passed"
