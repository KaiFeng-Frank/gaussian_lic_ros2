#!/usr/bin/env python3
"""Smoke test for the continuous_time_node.

Publishes a synthetic IMU burst on `/imu_smoke`, waits for at least one
odometry message on `/continuous_time/odometry`, and exits.
"""

from __future__ import annotations

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry


def make_imu(stamp_ns: int) -> Imu:
    msg = Imu()
    msg.header.frame_id = "imu_link"
    msg.header.stamp.sec = int(stamp_ns // 1_000_000_000)
    msg.header.stamp.nanosec = int(stamp_ns % 1_000_000_000)
    msg.angular_velocity.x = 0.0
    msg.angular_velocity.y = 0.0
    msg.angular_velocity.z = 0.1
    msg.linear_acceleration.x = 0.0
    msg.linear_acceleration.y = 0.0
    msg.linear_acceleration.z = 9.81
    return msg


class SmokeClient(Node):
    def __init__(self) -> None:
        super().__init__("continuous_time_node_smoke")
        self.imu_pub = self.create_publisher(
            Imu,
            "/imu_smoke",
            QoSPresetProfiles.SENSOR_DATA.value,
        )
        self.odom_count = 0
        self.create_subscription(
            Odometry,
            "/continuous_time/odometry",
            self._on_odom,
            10,
        )

    def _on_odom(self, msg: Odometry) -> None:  # noqa: ARG002
        self.odom_count += 1


def main() -> int:
    rclpy.init()
    node = SmokeClient()
    deadline = time.time() + 12.0

    # Publish 80 IMU samples at 50 Hz to comfortably exceed the seed window.
    stamp_ns = 1_000_000_000
    for _ in range(80):
        node.imu_pub.publish(make_imu(stamp_ns))
        stamp_ns += 20_000_000  # 50 Hz
        rclpy.spin_once(node, timeout_sec=0.01)

    while time.time() < deadline and node.odom_count == 0:
        rclpy.spin_once(node, timeout_sec=0.1)

    received = node.odom_count
    node.destroy_node()
    rclpy.shutdown()

    if received == 0:
        print(
            "continuous_time_node_smoke FAIL: no odometry messages received",
            file=sys.stderr,
        )
        return 1
    print(f"continuous_time_node_smoke OK (received {received} odometry messages)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
