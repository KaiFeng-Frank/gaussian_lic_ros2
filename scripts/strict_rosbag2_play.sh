#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOOP=false
AUDIT=true
STRICT_STORAGE=false
DRY_RUN=false
READ_AHEAD_QUEUE_SIZE=100
RATE=""
START_OFFSET=""
REQUIRED_TOPICS=()

usage() {
  cat <<'EOF'
Usage: scripts/strict_rosbag2_play.sh [OPTIONS] BAG

Audit a rosbag2 directory and replay it with fixed strict-replay defaults.

Options:
  --loop                         Loop playback.
  --rate VALUE                   Pass a ros2 bag playback rate.
  --start-offset SEC             Pass a ros2 bag start offset.
  --read-ahead-queue-size N      Queue size for ros2 bag play. Default: 100.
  --required-topic TOPIC         Require a topic in metadata before playback.
  --strict-storage               Require sqlite3 storage for timestamp-order audit.
  --disable-audit                Skip rosbag2_timing_audit.py.
  --dry-run                      Print the command instead of execing ros2.
  -h, --help                     Show this help.
EOF
}

BAG=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --loop)
      LOOP=true
      shift
      ;;
    --rate)
      RATE="${2:?--rate requires a value}"
      shift 2
      ;;
    --start-offset)
      START_OFFSET="${2:?--start-offset requires a value}"
      shift 2
      ;;
    --read-ahead-queue-size)
      READ_AHEAD_QUEUE_SIZE="${2:?--read-ahead-queue-size requires a value}"
      shift 2
      ;;
    --required-topic)
      REQUIRED_TOPICS+=("${2:?--required-topic requires a value}")
      shift 2
      ;;
    --strict-storage)
      STRICT_STORAGE=true
      shift
      ;;
    --disable-audit)
      AUDIT=false
      shift
      ;;
    --dry-run)
      DRY_RUN=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    -*)
      echo "unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
    *)
      if [[ -n "${BAG}" ]]; then
        echo "multiple bag paths provided: ${BAG} and $1" >&2
        exit 2
      fi
      BAG="$1"
      shift
      ;;
  esac
done

if [[ -z "${BAG}" ]]; then
  usage >&2
  exit 2
fi

if [[ "${AUDIT}" == "true" ]]; then
  audit_cmd=("${ROOT_DIR}/scripts/rosbag2_timing_audit.py" --bag "${BAG}")
  for topic in "${REQUIRED_TOPICS[@]}"; do
    audit_cmd+=(--required-topic "${topic}")
  done
  if [[ "${STRICT_STORAGE}" == "true" ]]; then
    audit_cmd+=(--strict-storage)
  fi
  "${audit_cmd[@]}"
fi

play_cmd=(ros2 bag play "${BAG}" --clock --read-ahead-queue-size "${READ_AHEAD_QUEUE_SIZE}")
if [[ "${LOOP}" == "true" ]]; then
  play_cmd+=(--loop)
fi
if [[ -n "${RATE}" ]]; then
  play_cmd+=(--rate "${RATE}")
fi
if [[ -n "${START_OFFSET}" ]]; then
  play_cmd+=(--start-offset "${START_OFFSET}")
fi

if [[ "${DRY_RUN}" == "true" ]]; then
  printf '%q ' "${play_cmd[@]}"
  printf '\n'
  exit 0
fi

exec "${play_cmd[@]}"
