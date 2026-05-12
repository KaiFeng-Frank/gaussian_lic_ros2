#!/usr/bin/env bash
# Runs `continuous_time_node` against a short slice of a real frontend_raw
# rosbag2 and verifies the new continuous-time stack:
#   (1) accepts the real IMU + PointCloud2 topics without crashing,
#   (2) finishes seeding and starts solving inside the slice,
#   (3) emits a non-zero number of optimized Odometry messages.
#
# This is the first real-data smoke for the ported tracker. It is NOT a
# strict-parity gate — the LiDAR plane factor still uses a configurable
# plane prior, so the produced trajectory should only be evaluated on
# pipeline liveness, not absolute accuracy.

set -eo pipefail

WORKSPACE="${WORKSPACE:-/home/frank/gaussian_lic_ros2}"
SOURCE_SETUP="${SOURCE_SETUP:-/opt/ros/jazzy/setup.bash}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-43}"
export ROS_DOMAIN_ID

BAG_DIR="${BAG_DIR:-/home/frank/data/fast_livo/CBD_Building_01_frontend_raw}"
PLAYBACK_DURATION="${PLAYBACK_DURATION:-15}"   # seconds
PLAYBACK_RATE="${PLAYBACK_RATE:-0.5}"
LOG_DIR="${LOG_DIR:-/tmp/continuous_time_node_bag_smoke}"

source "${SOURCE_SETUP}"
source "${WORKSPACE}/install/setup.bash"

mkdir -p "${LOG_DIR}"
NODE_LOG="${LOG_DIR}/node.log"
PLAY_LOG="${LOG_DIR}/play.log"
TUM_PATH="${LOG_DIR}/continuous_time_trajectory.tum"

# `setsid` puts the node in its own process group so `kill -9 -- -PGID`
# can take down both the `ros2 run` wrapper AND the actual node grandchild.
setsid ros2 run gaussian_lic_tracking continuous_time_node \
  --ros-args \
  --remap /imu_for_gs:=/imu \
  --remap /points_for_gs:=/livox/lidar \
  -p odometry_topic:=/continuous_time/odometry \
  -p path_topic:=/continuous_time/path \
  -p knot_interval_seconds:=0.05 \
  -p window_knot_count:=10 \
  -p marginalize_oldest_count:=1 \
  -p seed_min_imu_count:=30 \
  -p step_period_seconds:=0.05 \
  -p pointcloud_enable:=true -p pointcloud_subsample_stride:=200 \
  -p output_tum_path:="${TUM_PATH}" \
  > "${NODE_LOG}" 2>&1 &
NODE_PID=$!
NODE_PGID=$(ps -o pgid= -p "${NODE_PID}" 2>/dev/null | tr -d ' ')

cleanup() {
  if [ -n "${NODE_PGID}" ]; then
    kill -9 -- "-${NODE_PGID}" 2>/dev/null || true
  fi
  for pid in "${NODE_PID:-}" "${PLAY_PID:-}"; do
    if [ -n "${pid}" ] && kill -0 "${pid}" 2>/dev/null; then
      kill -9 "${pid}" 2>/dev/null || true
    fi
  done
  pkill -9 -f "continuous_time_node --ros-args" 2>/dev/null || true
  pkill -9 -f "ros2 bag play ${BAG_DIR}" 2>/dev/null || true
}
trap cleanup EXIT

sleep 2

ros2 bag play "${BAG_DIR}" --rate "${PLAYBACK_RATE}" \
  > "${PLAY_LOG}" 2>&1 &
PLAY_PID=$!

# Use the Python rclpy client to count odometry messages during the slice.
/usr/bin/python3.12 - <<PY > "${LOG_DIR}/odom_count.txt"
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class Counter(Node):
    def __init__(self):
        super().__init__("continuous_time_bag_smoke_counter")
        self.count = 0
        self.create_subscription(Odometry, "/continuous_time/odometry", self._on, 50)
    def _on(self, msg):
        self.count += 1

rclpy.init()
node = Counter()
deadline = time.time() + float("${PLAYBACK_DURATION}")
while time.time() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
print(node.count)
node.destroy_node()
rclpy.shutdown()
PY
RC=$?

# Wait briefly for play to drain.
sleep 1

ODOM_COUNT=$(cat "${LOG_DIR}/odom_count.txt" 2>/dev/null | tail -1 | tr -d ' \n')
if [ -z "${ODOM_COUNT}" ]; then
  ODOM_COUNT=0
fi

TUM_LINES=0
if [ -f "${TUM_PATH}" ]; then
  TUM_LINES=$(grep -cv '^#' "${TUM_PATH}" 2>/dev/null || echo 0)
fi

if [ "${RC}" -eq 0 ] && [ "${ODOM_COUNT}" -gt 0 ] && [ "${TUM_LINES}" -gt 0 ]; then
  echo "continuous_time_node_bag_smoke OK (bag=${BAG_DIR##*/}, slice=${PLAYBACK_DURATION}s, rate=${PLAYBACK_RATE}, odometry_msgs=${ODOM_COUNT}, tum_lines=${TUM_LINES}, tum_path=${TUM_PATH})"
  exit 0
fi

echo "continuous_time_node_bag_smoke FAIL (rc=${RC}, odometry_msgs=${ODOM_COUNT}, tum_lines=${TUM_LINES})"
echo "--- node log (tail) ---"
tail -30 "${NODE_LOG}" || true
echo "--- play log (tail) ---"
tail -30 "${PLAY_LOG}" || true
exit 1
