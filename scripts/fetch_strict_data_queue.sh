#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DATA_ROOT="${DATA_ROOT:-/home/frank/data}"
MIN_FREE_GB="${MIN_FREE_GB:-5}"
HF_BASE="${HF_BASE:-https://huggingface.co/datasets/DapengFeng/MCAP/resolve/main}"

usage() {
  cat <<'EOF'
Usage: scripts/fetch_strict_data_queue.sh [OPTIONS] [TARGET...]

Fetch the public raw inputs still needed by the strict parity matrix. The
script is deliberately restartable: completed exact-size FAST-LIVO MCAP files
are skipped, SharePoint/OneDrive files resume from .part files, and Google Drive
files use the repository Drive fetcher.

Targets:
  fastlivo-hf     FAST-LIVO MCAP mirror: hku1, hku2, Visual_Challenge, LiDAR_Degenerate
  m2dgr-room      M2DGR room_01, room_02, room_03 raw bags from the official README links
  mcd-ntu-day-01  MCD ntu_day_01 d435i, mid70, vn100 split bags
  all             Run all targets in the order above. Default.

Options:
  --data-root DIR      Data root. Default: /home/frank/data or DATA_ROOT.
  --min-free-gb N      Minimum free GiB for individual fetchers. Default: 5.
  --help               Show this help.
EOF
}

targets=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --data-root)
      DATA_ROOT="$2"
      shift 2
      ;;
    --min-free-gb)
      MIN_FREE_GB="$2"
      shift 2
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    all|fastlivo-hf|m2dgr-room|mcd-ntu-day-01)
      targets+=("$1")
      shift
      ;;
    *)
      echo "unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ ${#targets[@]} -eq 0 ]]; then
  targets=(all)
fi
if [[ " ${targets[*]} " == *" all "* ]]; then
  targets=(fastlivo-hf m2dgr-room mcd-ntu-day-01)
fi

fetch_hf_file() {
  local rel="$1"
  local expected_bytes="$2"
  local output="${DATA_ROOT}/fast_livo_mcap/${rel}"
  local tmp="${output}.part"
  local url="${HF_BASE}/${rel}?download=true"
  mkdir -p "$(dirname "$output")"
  if [[ -f "$output" ]]; then
    local actual
    actual="$(stat -c '%s' "$output")"
    if [[ "$actual" == "$expected_bytes" ]]; then
      echo "[strict-data] FAST-LIVO HF ready: $output ($actual bytes)"
      return 0
    fi
    echo "[strict-data] FAST-LIVO HF byte mismatch, refetching: $output got=$actual expected=$expected_bytes" >&2
    rm -f "$output"
  fi
  echo "[strict-data] FAST-LIVO HF fetch: $rel -> $output"
  curl -L \
    --fail \
    --retry 20 \
    --retry-delay 10 \
    --connect-timeout 30 \
    --continue-at - \
    --no-progress-meter \
    -o "$tmp" \
    "$url"
  local actual_part
  actual_part="$(stat -c '%s' "$tmp")"
  if [[ "$actual_part" != "$expected_bytes" ]]; then
    echo "[strict-data] FAST-LIVO HF byte mismatch: $tmp got=$actual_part expected=$expected_bytes" >&2
    return 4
  fi
  mv "$tmp" "$output"
}

fetch_fastlivo_hf() {
  fetch_hf_file "FAST-LIVO/hku2/hku2_0.mcap" "885629016"
  fetch_hf_file "FAST-LIVO/hku1/hku1_0.mcap" "1190284494"
  fetch_hf_file "FAST-LIVO/Visual_Challenge/Visual_Challenge_0.mcap" "1303247009"
  fetch_hf_file "FAST-LIVO/LiDAR_Degenerate/LiDAR_Degenerate_0.mcap" "6506600889"
}

fetch_m2dgr_room() {
  "${ROOT_DIR}/scripts/fetch_sharepoint_file.sh" \
    --url "https://sjtueducn-my.sharepoint.com/:u:/g/personal/594666_sjtu_edu_cn/EfG372xf9h9Dl0xjm5XcDgoB7JP0SsWJfAfpfO2CU-QOmw" \
    --output "${DATA_ROOT}/m2dgr/room_01/room_01.bag" \
    --cookie /tmp/m2dgr_room01_raw_cookies.txt \
    --min-free-gb "$MIN_FREE_GB"
  "${ROOT_DIR}/scripts/fetch_sharepoint_file.sh" \
    --url "https://sjtueducn-my.sharepoint.com/:u:/g/personal/594666_sjtu_edu_cn/EaVK6tu2gs5NnOpAhhWrTPEBK_cpPGiq_1vDXET2GTCeNQ" \
    --output "${DATA_ROOT}/m2dgr/room_02/room_02.bag" \
    --cookie /tmp/m2dgr_room02_raw_cookies.txt \
    --min-free-gb "$MIN_FREE_GB"
  "${ROOT_DIR}/scripts/fetch_sharepoint_file.sh" \
    --url "https://sjtueducn-my.sharepoint.com/:u:/g/personal/594666_sjtu_edu_cn/EZfZZNphLARHl0H4zLbM_kABbwkgl5efzhVqUeia8T-adQ" \
    --output "${DATA_ROOT}/m2dgr/room_03/room_03.bag" \
    --cookie /tmp/m2dgr_room03_raw_cookies.txt \
    --min-free-gb "$MIN_FREE_GB"
}

fetch_mcd_ntu_day_01() {
  local out="${DATA_ROOT}/mcd/ntu_day_01"
  mkdir -p "$out"
  "${ROOT_DIR}/scripts/fetch_google_drive_file.py" \
    --file "1E4oTZKaajNJA8KT9hcOsREU4If2mAHle" \
    --output "${out}/ntu_day_01_d435i.bag.bz2" \
    --manifest "${out}/ntu_day_01_d435i.fetch.json" \
    --progress-interval-mb 256 \
    --min-free-gb "$MIN_FREE_GB" \
    --json
  "${ROOT_DIR}/scripts/fetch_google_drive_file.py" \
    --file "1p7JCvUKh9BgKNPnt-SeC7oQgZ4S863KQ" \
    --output "${out}/ntu_day_01_mid70.bag.bz2" \
    --manifest "${out}/ntu_day_01_mid70.fetch.json" \
    --progress-interval-mb 256 \
    --min-free-gb "$MIN_FREE_GB" \
    --json
  "${ROOT_DIR}/scripts/fetch_google_drive_file.py" \
    --file "1bBKRlzwG4v7K4mBmLAQzfwp_O6yOR0Ld" \
    --output "${out}/ntu_day_01_vn100.bag.bz2" \
    --manifest "${out}/ntu_day_01_vn100.fetch.json" \
    --progress-interval-mb 256 \
    --min-free-gb "$MIN_FREE_GB" \
    --json
}

for target in "${targets[@]}"; do
  case "$target" in
    fastlivo-hf) fetch_fastlivo_hf ;;
    m2dgr-room) fetch_m2dgr_room ;;
    mcd-ntu-day-01) fetch_mcd_ntu_day_01 ;;
  esac
done

"${ROOT_DIR}/scripts/audit_strict_data_inputs.py" \
  --output "${ROOT_DIR}/docs/strict_data_status.json" \
  --markdown "${ROOT_DIR}/docs/strict_data_status.md" \
  --cleanup-candidates 12 \
  --min-free-gb 100
