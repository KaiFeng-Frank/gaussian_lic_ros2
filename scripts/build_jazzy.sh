#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export ROS_DISTRO="${ROS_DISTRO:-jazzy}"
exec "${ROOT_DIR}/scripts/build_ros2.sh" "$@"
