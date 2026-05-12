#!/usr/bin/env bash
# Smoke test: launch continuous_time_node, publish synthetic IMU samples via a
# Python helper, and verify it emits at least one odometry message.

set -eo pipefail

WORKSPACE="${WORKSPACE:-/home/frank/gaussian_lic_ros2}"
SOURCE_SETUP="${SOURCE_SETUP:-/opt/ros/jazzy/setup.bash}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_DOMAIN_ID

source "${SOURCE_SETUP}"
source "${WORKSPACE}/install/setup.bash"

LOG_DIR="${LOG_DIR:-/tmp/continuous_time_node_smoke}"
mkdir -p "${LOG_DIR}"
NODE_LOG="${LOG_DIR}/node.log"

# `setsid` puts the node in its own process group so cleanup can take down
# the `ros2 run` wrapper AND the actual `continuous_time_node` grandchild
# in one shot. Without this, killing only the wrapper PID leaves the real
# node running as an orphan — which is how we accidentally accumulated 15
# zombie nodes over the porting session.
setsid ros2 run gaussian_lic_tracking continuous_time_node \
  --ros-args \
  -p raw_imu_topic:=/imu_smoke \
  -p raw_pointcloud_topic:=/points_smoke \
  -p odometry_topic:=/continuous_time/odometry \
  -p path_topic:=/continuous_time/path \
  -p knot_interval_seconds:=0.05 \
  -p window_knot_count:=8 \
  -p marginalize_oldest_count:=1 \
  -p seed_min_imu_count:=15 \
  -p step_period_seconds:=0.05 \
  -p pointcloud_enable:=true \
  -p pointcloud_subsample_stride:=5 \
  -p pointcloud_max_points_per_msg:=64 \
  > "${NODE_LOG}" 2>&1 &
NODE_PID=$!
NODE_PGID=$(ps -o pgid= -p "${NODE_PID}" 2>/dev/null | tr -d ' ')

cleanup() {
  if [ -n "${NODE_PGID}" ]; then
    kill -9 -- "-${NODE_PGID}" 2>/dev/null || true
  fi
  pkill -9 -f "continuous_time_node --ros-args" 2>/dev/null || true
}
trap cleanup EXIT

# Give the node a moment to register topics.
sleep 2

/usr/bin/python3.12 "${WORKSPACE}/scripts/continuous_time_node_smoke.py"
RC=$?

if [ "${RC}" -ne 0 ]; then
  echo "--- node log (tail) ---"
  tail -30 "${NODE_LOG}" || true
fi

exit "${RC}"
