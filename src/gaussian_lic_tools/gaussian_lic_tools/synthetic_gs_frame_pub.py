# SPDX-License-Identifier: GPL-3.0-or-later

import math
import struct

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
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
        self.declare_parameter("rendered_image_topic", "/gaussian_lic/rendered_image")
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("pointcloud_color_mode", "packed_rgb")
        self.declare_parameter("point_color_rgb", "255,32,16")
        self.declare_parameter("image_color_rgb", "0,0,0")
        self.declare_parameter("image_width", 1)
        self.declare_parameter("image_height", 1)
        self.declare_parameter("image_pattern", "solid")
        self.declare_parameter("image_shift_x_px", 0.0)
        self.declare_parameter("image_shift_y_px", 0.0)
        self.declare_parameter("publish_rendered_image", False)
        self.declare_parameter("rendered_shift_x_px", 0.0)
        self.declare_parameter("rendered_shift_y_px", 0.0)
        self.declare_parameter("publish_depth", True)
        self.declare_parameter("pose_output_mode", "pose_stamped")
        self.declare_parameter("child_frame_id", "base_link")
        self.declare_parameter("imu_samples_per_frame", 3)
        self.declare_parameter("imu_stamp_lead_ns", 10000000)

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
        self.imu_samples_per_frame = int(self.get_parameter("imu_samples_per_frame").value)
        if self.imu_samples_per_frame < 1:
            raise ValueError("imu_samples_per_frame must be at least 1")
        self.imu_stamp_lead_ns = int(self.get_parameter("imu_stamp_lead_ns").value)
        if self.imu_stamp_lead_ns < 0:
            raise ValueError("imu_stamp_lead_ns must be non-negative")
        self.image_width = int(self.get_parameter("image_width").value)
        self.image_height = int(self.get_parameter("image_height").value)
        if self.image_width <= 0 or self.image_height <= 0:
            raise ValueError("image_width and image_height must be positive")
        self.image_pattern = str(
            self.get_parameter("image_pattern").value).strip().lower()
        if self.image_pattern not in {"solid", "gradient", "gaussian"}:
            raise ValueError("image_pattern must be solid, gradient, or gaussian")
        self.image_shift_x_px = float(self.get_parameter("image_shift_x_px").value)
        self.image_shift_y_px = float(self.get_parameter("image_shift_y_px").value)
        self.publish_rendered_image = bool(
            self.get_parameter("publish_rendered_image").value)
        self.rendered_shift_x_px = float(
            self.get_parameter("rendered_shift_x_px").value)
        self.rendered_shift_y_px = float(
            self.get_parameter("rendered_shift_y_px").value)

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
        self.rendered_image_pub = None
        if self.publish_rendered_image:
            self.rendered_image_pub = self.create_publisher(
                Image,
                self.get_parameter("rendered_image_topic").value,
                QoSProfile(
                    depth=1,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                ),
            )
        self.camera_info_pub = self.create_publisher(
            CameraInfo, self.get_parameter("camera_info_topic").value, 10)
        self.depth_pub = None
        if self.publish_depth:
            self.depth_pub = self.create_publisher(
                Image, self.get_parameter("depth_topic").value, 10)
        self.imu_pub = self.create_publisher(
            Imu, self.get_parameter("imu_topic").value, 10)

        rate = float(self.get_parameter("publish_rate_hz").value)
        self.publish_period_ns = max(1, int(round(1000000000.0 / rate)))
        self.timer = self.create_timer(1.0 / rate, self.publish_frame)
        self.frame_id = 0
        self.last_frame_stamp_ns = None
        self.last_imu_stamp_ns = None

        self.get_logger().info(
            "Publishing synthetic synchronized GS input frames "
            f"(pointcloud_color_mode={self.pointcloud_color_mode}, "
            f"publish_depth={self.publish_depth}, "
            f"pose_output_mode={self.pose_output_mode}, "
            f"image={self.image_width}x{self.image_height}:{self.image_pattern}, "
            f"publish_rendered_image={self.publish_rendered_image})")

    @staticmethod
    def stamp_to_nsec(stamp):
        return stamp.sec * 1000000000 + stamp.nanosec

    @staticmethod
    def nsec_to_stamp(total_ns):
        return Time(
            sec=total_ns // 1000000000,
            nanosec=total_ns % 1000000000)

    def make_imu_stamp(self, stamp, sample_index):
        frame_ns = self.stamp_to_nsec(stamp)
        if frame_ns <= 0:
            return None
        if self.last_frame_stamp_ns is not None and frame_ns > self.last_frame_stamp_ns:
            lead_ns = min(self.imu_stamp_lead_ns, max(0, (frame_ns - self.last_frame_stamp_ns) // 4))
            start_ns = self.last_frame_stamp_ns + lead_ns
            end_ns = frame_ns - lead_ns
            if end_ns >= start_ns and self.imu_samples_per_frame > 1:
                alpha = float(sample_index) / float(self.imu_samples_per_frame - 1)
                candidate_ns = int(round(start_ns + alpha * (end_ns - start_ns)))
            else:
                candidate_ns = max(0, frame_ns - lead_ns)
        else:
            candidate_ns = max(0, frame_ns - self.imu_lead_ns_for_sample(sample_index))
        if self.last_imu_stamp_ns is not None and candidate_ns <= self.last_imu_stamp_ns:
            candidate_ns = self.last_imu_stamp_ns + 1
        if candidate_ns >= frame_ns:
            candidate_ns = frame_ns - 1
        if self.last_imu_stamp_ns is not None and candidate_ns <= self.last_imu_stamp_ns:
            return None
        return self.nsec_to_stamp(candidate_ns)

    def imu_lead_ns_for_sample(self, sample_index):
        if self.imu_samples_per_frame == 1:
            return self.imu_stamp_lead_ns
        max_lead_ns = max(
            self.imu_stamp_lead_ns,
            min(self.publish_period_ns - 1, int(0.9 * self.publish_period_ns)))
        alpha = float(self.imu_samples_per_frame - 1 - sample_index) / float(
            self.imu_samples_per_frame - 1)
        return int(round(self.imu_stamp_lead_ns + alpha * (max_lead_ns - self.imu_stamp_lead_ns)))

    @staticmethod
    def clamp_byte(value):
        return max(0, min(255, int(round(value))))

    def sample_image_rgb(self, x, y, shift_x, shift_y):
        if self.image_pattern == "solid":
            return self.image_rgb

        sample_x = float(x) - shift_x
        sample_y = float(y) - shift_y
        if self.image_pattern == "gradient":
            value = 32.0 + 4.0 * sample_x + 2.0 * sample_y
            gray = self.clamp_byte(value)
            return (gray, gray, gray)

        center_x = 0.5 * float(self.image_width - 1)
        center_y = 0.5 * float(self.image_height - 1)
        sigma = max(float(min(self.image_width, self.image_height)) / 5.0, 1.0)
        dx = sample_x - center_x
        dy = sample_y - center_y
        gray = self.clamp_byte(255.0 * math.exp(-(dx * dx + dy * dy) / (2.0 * sigma * sigma)))
        return (gray, gray, gray)

    def make_image(self, stamp, shift_x, shift_y):
        image = Image()
        image.header.stamp = stamp
        image.header.frame_id = "camera"
        image.height = self.image_height
        image.width = self.image_width
        image.encoding = "bgr8"
        image.step = self.image_width * 3
        data = bytearray(self.image_height * image.step)
        for y in range(self.image_height):
            for x in range(self.image_width):
                red, green, blue = self.sample_image_rgb(x, y, shift_x, shift_y)
                offset = y * image.step + x * 3
                data[offset] = blue
                data[offset + 1] = green
                data[offset + 2] = red
        image.data = bytes(data)
        return image

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

        for sample_index in range(self.imu_samples_per_frame):
            imu_stamp = self.make_imu_stamp(stamp, sample_index)
            if imu_stamp is None:
                continue
            imu = Imu()
            imu.header.stamp = imu_stamp
            imu.header.frame_id = "imu"
            imu.orientation.w = 1.0
            imu.orientation_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
            imu.angular_velocity_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
            imu.linear_acceleration.z = 9.80665
            imu.linear_acceleration_covariance = [0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1]
            self.imu_pub.publish(imu)
            self.last_imu_stamp_ns = self.stamp_to_nsec(imu_stamp)

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

        image = self.make_image(stamp, self.image_shift_x_px, self.image_shift_y_px)
        self.image_pub.publish(image)
        if self.rendered_image_pub is not None:
            rendered = self.make_image(
                stamp, self.rendered_shift_x_px, self.rendered_shift_y_px)
            self.rendered_image_pub.publish(rendered)

        camera_info = CameraInfo()
        camera_info.header.stamp = stamp
        camera_info.header.frame_id = "camera"
        camera_info.height = self.image_height
        camera_info.width = self.image_width
        camera_info.distortion_model = "plumb_bob"
        camera_info.d = []
        fx = max(float(self.image_width), 1.0)
        fy = max(float(self.image_height), 1.0)
        cx = 0.5 if self.image_width == 1 else 0.5 * float(self.image_width - 1)
        cy = 0.5 if self.image_height == 1 else 0.5 * float(self.image_height - 1)
        camera_info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        camera_info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.camera_info_pub.publish(camera_info)

        if self.depth_pub is not None:
            depth = Image()
            depth.header.stamp = stamp
            depth.header.frame_id = "camera"
            depth.height = self.image_height
            depth.width = self.image_width
            depth.encoding = "32FC1"
            depth.step = self.image_width * 4
            depth.data = struct.pack("f", 1.0) * (self.image_width * self.image_height)
            self.depth_pub.publish(depth)

        self.last_frame_stamp_ns = self.stamp_to_nsec(stamp)
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
