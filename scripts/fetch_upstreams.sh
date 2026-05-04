#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
EXTERNAL_DIR="${ROOT_DIR}/external"
WITH_LEGACY_COCOLIC=0

usage() {
    cat <<'EOF'
Usage: scripts/fetch_upstreams.sh [--with-legacy-cocolic]

Fetch upstream sources used for the ROS2 port.

Default:
  external/Gaussian-LIC   APRIL-ZJU Gaussian-LIC/Gaussian-LIC2 upstream

Optional:
  --with-legacy-cocolic   also fetch APRIL-ZJU/Coco-LIC for legacy ROS1 reference
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --with-legacy-cocolic)
            WITH_LEGACY_COCOLIC=1
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "[fetch] unknown argument: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

clone_or_fetch() {
    local name="$1"
    local url="$2"
    local dest="${EXTERNAL_DIR}/${name}"

    if [[ -d "${dest}/.git" ]]; then
        echo "[fetch] ${name} already exists; fetching tags and remote refs"
        git -C "${dest}" fetch --tags origin
    else
        echo "[fetch] cloning ${name}"
        git clone --recursive "${url}" "${dest}"
    fi
}

mkdir -p "${EXTERNAL_DIR}"
echo "[fetch] primary upstream: Gaussian-LIC/Gaussian-LIC2"
clone_or_fetch "Gaussian-LIC" "https://github.com/APRIL-ZJU/Gaussian-LIC.git"

if [[ "${WITH_LEGACY_COCOLIC}" -eq 1 ]]; then
    echo "[fetch] legacy reference upstream: Coco-LIC"
    clone_or_fetch "Coco-LIC" "https://github.com/APRIL-ZJU/Coco-LIC.git"
else
    echo "[fetch] skipping legacy Coco-LIC; pass --with-legacy-cocolic to fetch it"
fi

echo "[fetch] done"
