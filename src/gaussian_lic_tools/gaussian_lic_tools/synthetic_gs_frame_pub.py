# SPDX-License-Identifier: GPL-3.0-or-later

import struct

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2, PointField


def parse_rgb(value):
    if isinstance(value, str):
        parts = value.replace(" ", "").split(",")
    else:
        parts = list(value)
    if len(parts) != 3:
        raise ValueError("RGB parameters must contain exactly three channels")
    channels = []
    for part in parts:
        channel = int(part)
        channels.append(max(0, min(255, channel)))
    return tuple(channels)


class SyntheticGsFramePublisher(Node):
    def __init__(self):
        super().__init__("synthetic_gs_frame_pub")
        self.declare_parameter("pointcloud_topic", "/points_for_gs")
        self.declare_parameter("pose_topic", "/pose_for_gs")
        self.declare_parameter("odometry_topic", "/gaussian_lic/frontend/input_odometry")
        self.declare_parameter("image_topic", "/image_for_gs")
        self.declare_parameter("camera_info_topic", "/camera_info_for_gs")
        self.declare_parameter("depth_topic", "/depth_for_gs")
        self.declare_parameter("imu_topic", "/imu_for_gs")
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("pointcloud_color_mode", "packed_rgb")
        self.declare_parameter("point_color_rgb", "255,32,16")
        self.declare_parameter("image_color_rgb", "0,0,0")
        self.declare_parameter("publish_depth", True)
        self.declare_parameter("pose_output_mode", "pose_stamped")
        self.declare_parameter("child_frame_id", "base_link")

        self.pointcloud_color_mode = str(
            self.get_parameter("pointcloud_color_mode").value).strip().lower()
        if self.pointcloud_color_mode not in {"packed_rgb", "rgb_fields", "none"}:
            raise ValueError(
            "pointcloud_color_mode must be packed_rgb, rgb_fields, or none")
        self.point_rgb = parse_rgb(self.get_parameter("point_color_rgb").value)
        self.image_rgb = parse_rgb(self.get_parameter("image_color_rgb").value)
        self.publish_depth = bool(self.get_parameter("publish_depth").value)
        self.pose_output_mode = str(
            self.get_parameter("pose_output_mode").value).strip().lower()
        if self.pose_output_mode not in {"pose_stamped", "odometry", "both", "none"}:
            raise ValueError(
                "pose_output_mode must be pose_stamped, odometry, both, or none")
        self.child_frame_id = str(self.get_parameter("child_frame_id").value)

        self.points_pub = self.create_publisher(
            PointCloud2, self.get_parameter("pointcloud_topic").value, 10)
        self.pose_pub = None
        if self.pose_output_mode in {"pose_stamped", "both"}:
            self.pose_pub = self.create_publisher(
                PoseStamped, self.get_parameter("pose_topic").value, 10)
        self.odometry_pub = None
        if self.pose_output_mode in {"odometry", "both"}:
            self.odometry_pub = self.create_publisher(
                Odometry, self.get_parameter("odometry_topic").value, 10)
        self.image_pub = self.create_publisher(
            Image, self.get_parameter("image_topic").value, 10)
        self.camera_info_pub = self.create_publisher(
            CameraInfo, self.get_parameter("camera_info_topic").value, 10)
        self.depth_pub = None
        if self.publish_depth:
            self.depth_pub = self.create_publisher(
                Image, self.get_parameter("depth_topic").value, 10)
        self.imu_pub = self.create_publisher(
            Imu, self.get_parameter("imu_topic").value, 10)

        rate = float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(1.0 / rate, self.publish_frame)
        self.frame_id = 0
        self.get_logger().info(
            "Publishing synthetic synchronized GS input frames "
            f"(pointcloud_color_mode={self.pointcloud_color_mode}, "
            f"publish_depth={self.publish_depth}, "
            f"pose_output_mode={self.pose_output_mode})")

    def make_pointcloud_data(self):
        if self.pointcloud_color_mode == "none":
            return (
                [
                    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
                ],
                12,
                struct.pack("fff", 0.0, 0.0, 1.0),
            )

        data = bytearray(16)
        struct.pack_into("fff", data, 0, 0.0, 0.0, 1.0)
        red, green, blue = self.point_rgb
        if self.pointcloud_color_mode == "rgb_fields":
            data[12] = red
            data[13] = green
            data[14] = blue
            return (
                [
                    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
                    PointField(name="r", offset=12, datatype=PointField.UINT8, count=1),
                    PointField(name="g", offset=13, datatype=PointField.UINT8, count=1),
                    PointField(name="b", offset=14, datatype=PointField.UINT8, count=1),
                ],
                16,
                bytes(data),
            )

        rgb_bits = (red << 16) | (green << 8) | blue
        rgb_float = struct.unpack("f", struct.pack("I", rgb_bits))[0]
        struct.pack_into("f", data, 12, rgb_float)
        return (
            [
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
            ],
            16,
            bytes(data),
        )

    def publish_frame(self):
        stamp = self.get_clock().now().to_msg()

        point_fields, point_step, point_data = self.make_pointcloud_data()
        points = PointCloud2()
        points.header.stamp = stamp
        points.header.frame_id = "map"
        points.height = 1
        points.width = 1
        points.fields = point_fields
        points.is_bigendian = False
        points.point_step = point_step
        points.row_step = points.point_step * points.width
        points.is_dense = True
        points.data = point_data
        self.points_pub.publish(points)

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = "map"
        pose.pose.orientation.w = 1.0
        if self.pose_pub is not None:
            self.pose_pub.publish(pose)
        if self.odometry_pub is not None:
            odometry = Odometry()
            odometry.header = pose.header
            odometry.child_frame_id = self.child_frame_id
            odometry.pose.pose = pose.pose
            self.odometry_pub.publish(odometry)

        image = Image()
        image.header.stamp = stamp
        image.header.frame_id = "camera"
        image.height = 1
        image.width = 1
        image.encoding = "bgr8"
        image.step = 3
        red, green, blue = self.image_rgb
        image.data = bytes([blue, green, red])
        self.image_pub.publish(image)

        camera_info = CameraInfo()
        camera_info.header.stamp = stamp
        camera_info.header.frame_id = "camera"
        camera_info.height = 1
        camera_info.width = 1
        camera_info.distortion_model = "plumb_bob"
        camera_info.d = []
        camera_info.k = [1.0, 0.0, 0.5, 0.0, 1.0, 0.5, 0.0, 0.0, 1.0]
        camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        camera_info.p = [1.0, 0.0, 0.5, 0.0, 0.0, 1.0, 0.5, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.camera_info_pub.publish(camera_info)

        if self.depth_pub is not None:
            depth = Image()
            depth.header.stamp = stamp
            depth.header.frame_id = "camera"
            depth.height = 1
            depth.width = 1
            depth.encoding = "32FC1"
            depth.step = 4
            depth.data = struct.pack("f", 1.0)
            self.depth_pub.publish(depth)

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = "imu"
        imu.orientation.w = 1.0
        imu.orientation_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        imu.angular_velocity_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        imu.linear_acceleration.z = 9.80665
        imu.linear_acceleration_covariance = [0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1]
        self.imu_pub.publish(imu)

        self.frame_id += 1


def main(args=None):
    rclpy.init(args=args)
    node = SyntheticGsFramePublisher()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt, RCLError):
        pass
    finally:
        try:
            node.destroy_node()
        except (KeyboardInterrupt, RCLError):
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except (KeyboardInterrupt, RCLError):
                pass


if __name__ == "__main__":
    main()
