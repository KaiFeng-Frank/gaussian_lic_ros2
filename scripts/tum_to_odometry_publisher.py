#!/usr/bin/env python3
"""Replays a TUM trajectory as `nav_msgs/Odometry` messages.

Used by `continuous_time_native_reference_parity.sh` to feed ground-truth
poses into `continuous_time_node`'s `external_odometry_prior_topic` so the
estimator can be seeded near the true pose instead of identity. The
replay walks the TUM file linearly and publishes one message per stamp
until the requested wall-clock duration elapses.
"""

from __future__ import annotations

import argparse
import math
import signal
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from nav_msgs.msg import Odometry


def load_tum(path: str) -> list[tuple[float, tuple[float, float, float],
                                       tuple[float, float, float, float]]]:
    out = []
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        if len(parts) != 8:
            continue
        try:
            stamp = float(parts[0])
            tx, ty, tz = float(parts[1]), float(parts[2]), float(parts[3])
            qx, qy, qz, qw = (
                float(parts[4]), float(parts[5]),
                float(parts[6]), float(parts[7]),
            )
        except ValueError:
            continue
        if not all(math.isfinite(v) for v in (stamp, tx, ty, tz, qx, qy, qz, qw)):
            continue
        out.append((stamp, (tx, ty, tz), (qx, qy, qz, qw)))
    return out


def make_odom(stamp: float, t: tuple[float, float, float],
              q: tuple[float, float, float, float], frame: str, child_frame: str) -> Odometry:
    msg = Odometry()
    msg.header.frame_id = frame
    msg.header.stamp.sec = int(stamp)
    msg.header.stamp.nanosec = int(round((stamp - int(stamp)) * 1.0e9))
    msg.child_frame_id = child_frame
    msg.pose.pose.position.x = t[0]
    msg.pose.pose.position.y = t[1]
    msg.pose.pose.position.z = t[2]
    msg.pose.pose.orientation.x = q[0]
    msg.pose.pose.orientation.y = q[1]
    msg.pose.pose.orientation.z = q[2]
    msg.pose.pose.orientation.w = q[3]
    return msg


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--tum", required=True, help="Source TUM trajectory")
    parser.add_argument("--topic", default="/external_odometry_prior")
    parser.add_argument("--frame", default="map")
    parser.add_argument("--child-frame", default="imu_link")
    parser.add_argument("--duration", type=float, default=15.0,
                        help="Wall-clock duration to publish for, in seconds.")
    parser.add_argument("--rate-hz", type=float, default=20.0)
    parser.add_argument("--max-messages", type=int, default=0,
                        help="If >0, stop after publishing this many messages.")
    args = parser.parse_args()

    poses = load_tum(args.tum)
    if not poses:
        print(f"tum_to_odometry_publisher: no usable poses in {args.tum}", file=sys.stderr)
        return 1

    rclpy.init()
    node = rclpy.create_node("tum_to_odometry_publisher")
    pub = node.create_publisher(
        Odometry, args.topic, QoSPresetProfiles.PARAMETER_EVENTS.value)

    def _on_term(_sig, _frame):
        try:
            node.destroy_node()
        finally:
            rclpy.shutdown()
            sys.exit(0)

    signal.signal(signal.SIGTERM, _on_term)
    signal.signal(signal.SIGINT, _on_term)

    period = 1.0 / max(args.rate_hz, 1.0e-3)
    deadline = time.time() + args.duration
    sent = 0
    index = 0
    while time.time() < deadline:
        if index >= len(poses):
            break
        stamp, t, q = poses[index]
        msg = make_odom(stamp, t, q, args.frame, args.child_frame)
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.0)
        sent += 1
        if args.max_messages > 0 and sent >= args.max_messages:
            break
        index += 1
        time.sleep(period)

    print(f"tum_to_odometry_publisher: sent={sent}/{len(poses)} poses to {args.topic}")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
