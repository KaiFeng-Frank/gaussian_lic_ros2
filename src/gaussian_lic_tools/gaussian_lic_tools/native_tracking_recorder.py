# SPDX-License-Identifier: GPL-3.0-or-later

import json
import math
import signal
from pathlib import Path

import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from gaussian_lic_msgs.msg import MappingStatus, TrackingStatus
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2


BINNED_STATUS_PREFIXES = (
    "gaussian_snapshot_",
    "last_lidar_",
    "last_window_",
    "lidar_invalid_",
    "lidar_out_of_range_",
    "pointcloud_imu_wait_",
    "last_rendered_feedback_",
    "rendered_feedback_",
    "sliding_window_",
    "total_window_",
    "tracking_step_guard_",
    "trajectory_deskew_",
    "visual_",
)

BINNED_STATUS_FIELDS = {
    "stamp",
    "stamp_ns",
    "num_raw_images",
    "num_rendered_images",
    "num_raw_pointclouds",
    "num_raw_imus",
    "num_published_poses",
}

BINNED_MAPPING_STATUS_FIELDS = {
    "stamp",
    "stamp_ns",
    "pointcloud_messages",
    "pose_messages",
    "image_messages",
    "depth_messages",
    "aligned_frames",
    "converted_frames",
    "dropped_pointcloud_messages",
    "dropped_pose_messages",
    "dropped_image_messages",
    "dropped_depth_messages",
    "pending_pointcloud_messages",
    "pending_pose_messages",
    "pending_image_messages",
    "pending_depth_messages",
    "rendered_preview_count",
    "render_error_count",
    "last_aligned_pointcloud_pose_delta_ns",
    "last_aligned_pointcloud_image_delta_ns",
    "max_abs_aligned_pointcloud_pose_delta_ns",
    "max_abs_aligned_pointcloud_image_delta_ns",
    "pointcloud_anchor_pose_too_new_drops",
    "pointcloud_anchor_image_too_new_drops",
    "pointcloud_anchor_depth_too_new_drops",
    "image_anchor_pointcloud_too_new_drops",
    "image_anchor_pose_too_new_drops",
    "image_anchor_depth_too_new_drops",
    "gaussian_init_count",
    "gaussian_extend_count",
    "gaussian_optimization_count",
    "gaussian_optimization_steps",
    "gaussian_densify_count",
    "gaussian_prune_count",
    "gaussian_opacity_reset_count",
}


def stamp_to_nsec(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def stamp_to_tum(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def finite_float(value):
    numeric = float(value)
    return numeric if math.isfinite(numeric) else 0.0


def status_to_dict(msg):
    output = {}
    for name in msg.get_fields_and_field_types():
        value = getattr(msg, name)
        if isinstance(value, (bool, int, float, str)):
            output[name] = value
        else:
            try:
                output[name] = list(value)
            except TypeError:
                output[name] = str(value)
    return output


def update_numeric_summary(summary, record):
    for key, value in record.items():
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            continue
        numeric = float(value)
        if not math.isfinite(numeric):
            continue
        entry = summary.setdefault(
            key,
            {
                "count": 0,
                "first": numeric,
                "last": numeric,
                "min": numeric,
                "max": numeric,
                "mean": 0.0,
                "delta": 0.0,
            },
        )
        entry["count"] += 1
        entry["last"] = numeric
        entry["min"] = min(entry["min"], numeric)
        entry["max"] = max(entry["max"], numeric)
        entry["mean"] += (numeric - entry["mean"]) / entry["count"]
        entry["delta"] = numeric - entry["first"]


def is_binned_status_field(name):
    return name in BINNED_STATUS_FIELDS or name.startswith(BINNED_STATUS_PREFIXES)


def compact_numeric_record(record, field_selector):
    compact = {}
    for key, value in record.items():
        if not field_selector(key):
            continue
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            continue
        numeric = float(value)
        if math.isfinite(numeric):
            compact[key] = numeric
    return compact


def compact_status_record(record):
    return compact_numeric_record(record, is_binned_status_field)


def compact_mapping_status_record(record):
    return compact_numeric_record(record, BINNED_MAPPING_STATUS_FIELDS.__contains__)


def compute_time_binned_summary(records, bin_count):
    if bin_count <= 0 or not records:
        return []

    stamped_records = [
        record for record in records
        if isinstance(record.get("stamp"), (int, float)) and math.isfinite(float(record["stamp"]))
    ]
    positive_stamped_records = [record for record in stamped_records if float(record["stamp"]) > 0.0]
    if positive_stamped_records:
        stamped_records = positive_stamped_records
    if not stamped_records:
        return []

    first_stamp = min(float(record["stamp"]) for record in stamped_records)
    last_stamp = max(float(record["stamp"]) for record in stamped_records)
    duration = max(0.0, last_stamp - first_stamp)
    bins = []

    for index in range(bin_count):
        if duration <= 0.0:
            start_stamp = first_stamp
            end_stamp = last_stamp
        else:
            start_stamp = first_stamp + duration * index / bin_count
            end_stamp = first_stamp + duration * (index + 1) / bin_count

        summary = {}
        sample_count = 0
        for record in stamped_records:
            stamp = float(record["stamp"])
            if index + 1 == bin_count:
                in_bin = start_stamp <= stamp <= end_stamp
            else:
                in_bin = start_stamp <= stamp < end_stamp
            if not in_bin:
                continue
            sample_count += 1
            update_numeric_summary(summary, record)

        bins.append({
            "index": index,
            "start_stamp": start_stamp,
            "end_stamp": end_stamp,
            "sample_count": sample_count,
            "summary": summary,
        })

    return bins


class NativeTrackingRecorder(Node):
    def __init__(self):
        super().__init__("native_tracking_recorder")
        self.output_dir = Path(self.declare_parameter("output_dir", "/tmp/gaussian_lic_native_tracking").value)
        self.odometry_topic = self.declare_parameter(
            "odometry_topic", "/gaussian_lic/frontend/odometry").value
        self.pointcloud_topic = self.declare_parameter("pointcloud_topic", "/points_for_gs").value
        self.status_topic = self.declare_parameter(
            "status_topic", "/gaussian_lic/frontend/status").value
        self.mapping_status_topic = self.declare_parameter("mapping_status_topic", "").value
        self.reference_odometry_topic = self.declare_parameter("reference_odometry_topic", "").value
        self.reference_pose_topic = self.declare_parameter("reference_pose_topic", "").value
        self.status_bin_count = int(self.declare_parameter("status_bin_count", 8).value)
        self.status_history_max_samples = int(
            self.declare_parameter("status_history_max_samples", 5000).value)
        self.write_status_history = bool(
            self.declare_parameter("write_status_history", False).value)
        self.output_dir.mkdir(parents=True, exist_ok=True)

        qos = QoSProfile(
            depth=50,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        self.poses = []
        self.reference_poses = []
        self.topic_counts = {
            self.odometry_topic: 0,
            self.pointcloud_topic: 0,
            self.status_topic: 0,
        }
        if self.reference_odometry_topic:
            self.topic_counts[self.reference_odometry_topic] = 0
        if self.reference_pose_topic:
            self.topic_counts[self.reference_pose_topic] = 0
        if self.mapping_status_topic:
            self.topic_counts[self.mapping_status_topic] = 0
        self.first_stamp_ns = None
        self.last_stamp_ns = None
        self.last_status = {}
        self.last_mapping_status = {}
        self.status_summary = {}
        self.mapping_status_summary = {}
        self.status_records = []
        self.mapping_status_records = []

        self.create_subscription(Odometry, self.odometry_topic, self.on_odometry, qos)
        self.create_subscription(PointCloud2, self.pointcloud_topic, self.on_pointcloud, qos)
        self.create_subscription(TrackingStatus, self.status_topic, self.on_status, qos)
        if self.mapping_status_topic:
            self.create_subscription(MappingStatus, self.mapping_status_topic, self.on_mapping_status, qos)
        if self.reference_odometry_topic:
            self.create_subscription(
                Odometry, self.reference_odometry_topic, self.on_reference_odometry, qos)
        if self.reference_pose_topic:
            self.create_subscription(
                PoseStamped, self.reference_pose_topic, self.on_reference_pose, qos)
        self.create_timer(1.0, self.flush)
        reference_topics = [
            topic for topic in (self.reference_odometry_topic, self.reference_pose_topic) if topic
        ]
        reference_suffix = f", references={reference_topics}" if reference_topics else ""
        self.get_logger().info(
            "Recording native tracking evidence to "
            f"{self.output_dir} from {self.odometry_topic}, {self.pointcloud_topic}, "
            f"{self.status_topic}{reference_suffix}")

    def update_span(self, stamp):
        stamp_ns = stamp_to_nsec(stamp)
        if self.first_stamp_ns is None or stamp_ns < self.first_stamp_ns:
            self.first_stamp_ns = stamp_ns
        if self.last_stamp_ns is None or stamp_ns > self.last_stamp_ns:
            self.last_stamp_ns = stamp_ns

    def on_odometry(self, msg):
        self.topic_counts[self.odometry_topic] += 1
        self.update_span(msg.header.stamp)
        self.poses.append(self.pose_to_tum_record(msg.header.stamp, msg.pose.pose))

    def pose_to_tum_record(self, stamp, pose):
        return (
            stamp_to_tum(stamp),
            finite_float(pose.position.x),
            finite_float(pose.position.y),
            finite_float(pose.position.z),
            finite_float(pose.orientation.x),
            finite_float(pose.orientation.y),
            finite_float(pose.orientation.z),
            finite_float(pose.orientation.w),
        )

    def on_reference_odometry(self, msg):
        self.topic_counts[self.reference_odometry_topic] += 1
        self.reference_poses.append(self.pose_to_tum_record(msg.header.stamp, msg.pose.pose))

    def on_reference_pose(self, msg):
        self.topic_counts[self.reference_pose_topic] += 1
        self.reference_poses.append(self.pose_to_tum_record(msg.header.stamp, msg.pose))

    @staticmethod
    def write_tum(path, poses):
        with path.open("w", encoding="utf-8") as stream:
            for pose in sorted(poses, key=lambda item: item[0]):
                stream.write(
                    f"{pose[0]:.9f} {pose[1]:.9f} {pose[2]:.9f} {pose[3]:.9f} "
                    f"{pose[4]:.9f} {pose[5]:.9f} {pose[6]:.9f} {pose[7]:.9f}\n")

    @staticmethod
    def path_length(poses):
        if len(poses) < 2:
            return 0.0
        total = 0.0
        ordered = sorted(poses, key=lambda item: item[0])
        for previous, current in zip(ordered, ordered[1:]):
            dx = current[1] - previous[1]
            dy = current[2] - previous[2]
            dz = current[3] - previous[3]
            total += math.sqrt(dx * dx + dy * dy + dz * dz)
        return total

    def on_pointcloud(self, msg):
        self.topic_counts[self.pointcloud_topic] += 1
        self.update_span(msg.header.stamp)

    def on_status(self, msg):
        self.topic_counts[self.status_topic] += 1
        self.last_status = status_to_dict(msg)
        self.last_status["stamp"] = stamp_to_tum(msg.header.stamp)
        self.last_status["stamp_ns"] = stamp_to_nsec(msg.header.stamp)
        update_numeric_summary(self.status_summary, self.last_status)
        self.status_records.append(compact_status_record(self.last_status))
        if self.status_history_max_samples > 0:
            overflow = len(self.status_records) - self.status_history_max_samples
            if overflow > 0:
                del self.status_records[:overflow]

    def on_mapping_status(self, msg):
        self.topic_counts[self.mapping_status_topic] += 1
        self.last_mapping_status = status_to_dict(msg)
        self.last_mapping_status["stamp"] = stamp_to_tum(msg.header.stamp)
        self.last_mapping_status["stamp_ns"] = stamp_to_nsec(msg.header.stamp)
        update_numeric_summary(self.mapping_status_summary, self.last_mapping_status)
        self.mapping_status_records.append(compact_mapping_status_record(self.last_mapping_status))
        if self.status_history_max_samples > 0:
            overflow = len(self.mapping_status_records) - self.status_history_max_samples
            if overflow > 0:
                del self.mapping_status_records[:overflow]

    def flush(self, final=False):
        trajectory_path = self.output_dir / "trajectory.tum"
        reference_trajectory_path = self.output_dir / "reference_trajectory.tum"
        metrics_path = self.output_dir / "metrics.json"
        status_history_path = self.output_dir / "tracking_status_history.jsonl"
        self.write_tum(trajectory_path, self.poses)
        self.write_tum(reference_trajectory_path, self.reference_poses)
        if final and self.write_status_history:
            with status_history_path.open("w", encoding="utf-8") as stream:
                for record in self.status_records:
                    stream.write(json.dumps(record, sort_keys=True) + "\n")

        duration_sec = 0.0
        if self.first_stamp_ns is not None and self.last_stamp_ns is not None:
            duration_sec = max(0.0, (self.last_stamp_ns - self.first_stamp_ns) * 1.0e-9)
        topic_hz = {
            topic: (count / duration_sec if duration_sec > 0.0 else 0.0)
            for topic, count in self.topic_counts.items()
        }
        tracking_status = {
            "samples": self.topic_counts[self.status_topic],
            "last": self.last_status,
            "summary": self.status_summary,
            "history_samples_retained": len(self.status_records),
            "status_bin_count": self.status_bin_count,
            "binned_summary_finalized": bool(final),
        }
        if final:
            tracking_status["binned_summary"] = compute_time_binned_summary(
                self.status_records, self.status_bin_count)

        metrics = {
            "duration_sec": duration_sec,
            "topic_counts": dict(self.topic_counts),
            "topic_hz": topic_hz,
            "trajectory_poses": len(self.poses),
            "trajectory_path_length_m": self.path_length(self.poses),
            "reference_trajectory_poses": len(self.reference_poses),
            "reference_trajectory_path_length_m": self.path_length(self.reference_poses),
            "tracking_status": tracking_status,
            "mapping_status": {
                "samples": self.topic_counts.get(self.mapping_status_topic, 0),
                "topic": self.mapping_status_topic,
                "last": self.last_mapping_status,
                "summary": self.mapping_status_summary,
                "history_samples_retained": len(self.mapping_status_records),
                "status_bin_count": self.status_bin_count,
                "binned_summary_finalized": bool(final),
            },
            "outputs": {
                "trajectory_tum": str(trajectory_path),
                "reference_trajectory_tum": str(reference_trajectory_path),
                "tracking_status_history_jsonl": (
                    str(status_history_path) if final and self.write_status_history else ""),
            },
        }
        if final:
            metrics["mapping_status"]["binned_summary"] = compute_time_binned_summary(
                self.mapping_status_records, self.status_bin_count)
        metrics_path.write_text(json.dumps(metrics, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def main(args=None):
    rclpy.init(args=args)
    node = NativeTrackingRecorder()

    def request_final_flush(_signum, _frame):
        raise KeyboardInterrupt

    previous_sigint = signal.getsignal(signal.SIGINT)
    previous_sigterm = signal.getsignal(signal.SIGTERM)
    signal.signal(signal.SIGINT, request_final_flush)
    signal.signal(signal.SIGTERM, request_final_flush)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RCLError as exc:
        if "context is not valid" not in str(exc):
            raise
    finally:
        signal.signal(signal.SIGINT, previous_sigint)
        signal.signal(signal.SIGTERM, previous_sigterm)
        node.flush(final=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
