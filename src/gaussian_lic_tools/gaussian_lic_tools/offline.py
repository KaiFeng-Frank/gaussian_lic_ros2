# SPDX-License-Identifier: GPL-3.0-or-later

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import struct
import sys


POINT_FIELD_INT8 = 1
POINT_FIELD_UINT8 = 2
POINT_FIELD_INT16 = 3
POINT_FIELD_UINT16 = 4
POINT_FIELD_INT32 = 5
POINT_FIELD_UINT32 = 6
POINT_FIELD_FLOAT32 = 7
POINT_FIELD_FLOAT64 = 8

FIELD_FORMATS = {
    POINT_FIELD_INT8: ("b", 1),
    POINT_FIELD_UINT8: ("B", 1),
    POINT_FIELD_INT16: ("h", 2),
    POINT_FIELD_UINT16: ("H", 2),
    POINT_FIELD_INT32: ("i", 4),
    POINT_FIELD_UINT32: ("I", 4),
    POINT_FIELD_FLOAT32: ("f", 4),
    POINT_FIELD_FLOAT64: ("d", 8),
}


@dataclass
class BagReadResult:
    bag_format: str
    storage_identifier: str
    topic_counts: dict
    poses: list
    points: list
    first_stamp_nsec: int | None
    last_stamp_nsec: int | None


def stamp_to_float(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def field_metadata(cloud_msg):
    return {field.name: field for field in cloud_msg.fields}


def packed_rgb_bits_to_channels(bits):
    red = (bits >> 16) & 0xFF
    green = (bits >> 8) & 0xFF
    blue = bits & 0xFF
    return red, green, blue


def scalar_color_to_u8(value):
    numeric = float(value)
    if 0.0 <= numeric <= 1.0:
        numeric *= 255.0
    return max(0, min(255, int(round(numeric))))


def pointcloud_data_bytes(cloud_msg):
    data = cloud_msg.data
    if isinstance(data, bytes):
        return data
    return bytes(data)


def field_offset(field, element_index=0):
    return int(field.offset) + element_index * FIELD_FORMATS[int(field.datatype)][1]


def unpack_field(data, offset, field, endian):
    datatype = int(field.datatype)
    fmt_size = FIELD_FORMATS.get(datatype)
    if fmt_size is None:
        raise ValueError(f"unsupported PointField datatype {datatype} for {field.name}")
    fmt, size = fmt_size
    if offset + size > len(data):
        raise ValueError(f"PointField {field.name} exceeds point data size")
    return struct.unpack_from(endian + fmt, data, offset)[0]


def unpack_packed_rgb(data, base_offset, field, endian):
    datatype = int(field.datatype)
    offset = field_offset(field)
    if datatype == POINT_FIELD_FLOAT32:
        bits = struct.unpack_from(endian + "I", data, base_offset + offset)[0]
    elif datatype in (POINT_FIELD_UINT32, POINT_FIELD_INT32):
        bits = int(unpack_field(data, base_offset + offset, field, endian)) & 0xFFFFFFFF
    else:
        bits = int(unpack_field(data, base_offset + offset, field, endian))
    return packed_rgb_bits_to_channels(bits)


def iter_xyzrgb_points(cloud_msg, max_points):
    fields = field_metadata(cloud_msg)
    for required_name in ("x", "y", "z"):
        if required_name not in fields:
            return

    color_mode = "none"
    if "rgb" in fields:
        color_mode = "rgb"
    elif "rgba" in fields:
        color_mode = "rgba"
    elif {"r", "g", "b"}.issubset(fields):
        color_mode = "channels"

    data = pointcloud_data_bytes(cloud_msg)
    width = int(cloud_msg.width)
    height = int(cloud_msg.height)
    point_step = int(cloud_msg.point_step)
    row_step = int(getattr(cloud_msg, "row_step", point_step * width))
    endian = ">" if bool(cloud_msg.is_bigendian) else "<"

    count = 0
    for row in range(height):
        row_offset = row * row_step
        for column in range(width):
            if count >= max_points:
                return

            base_offset = row_offset + column * point_step
            if base_offset + point_step > len(data):
                return

            try:
                x = float(
                    unpack_field(data, base_offset + field_offset(fields["x"]), fields["x"], endian)
                )
                y = float(
                    unpack_field(data, base_offset + field_offset(fields["y"]), fields["y"], endian)
                )
                z = float(
                    unpack_field(data, base_offset + field_offset(fields["z"]), fields["z"], endian)
                )
                if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                    continue

                if color_mode in {"rgb", "rgba"}:
                    red, green, blue = unpack_packed_rgb(data, base_offset, fields[color_mode], endian)
                elif color_mode == "channels":
                    red = scalar_color_to_u8(
                        unpack_field(data, base_offset + field_offset(fields["r"]), fields["r"], endian)
                    )
                    green = scalar_color_to_u8(
                        unpack_field(data, base_offset + field_offset(fields["g"]), fields["g"], endian)
                    )
                    blue = scalar_color_to_u8(
                        unpack_field(data, base_offset + field_offset(fields["b"]), fields["b"], endian)
                    )
                else:
                    red, green, blue = 255, 255, 255
            except (KeyError, TypeError, ValueError, struct.error):
                continue

            yield x, y, z, red, green, blue, color_mode != "none"
            count += 1


def pose_to_record(msg):
    pose = msg.pose.pose if hasattr(msg.pose, "pose") else msg.pose
    p = pose.position
    q = pose.orientation
    return (
        stamp_to_float(msg.header.stamp),
        float(p.x),
        float(p.y),
        float(p.z),
        float(q.x),
        float(q.y),
        float(q.z),
        float(q.w),
    )


def write_tum_trajectory(path, poses):
    with path.open("w", encoding="utf-8") as stream:
        for stamp, x, y, z, qx, qy, qz, qw in poses:
            stream.write(
                f"{stamp:.9f} {x:.9f} {y:.9f} {z:.9f} "
                f"{qx:.9f} {qy:.9f} {qz:.9f} {qw:.9f}\n"
            )


def write_xyzrgb_ply(path, points):
    with path.open("w", encoding="utf-8") as stream:
        stream.write("ply\n")
        stream.write("format ascii 1.0\n")
        stream.write(f"element vertex {len(points)}\n")
        stream.write("property float x\n")
        stream.write("property float y\n")
        stream.write("property float z\n")
        stream.write("property uchar red\n")
        stream.write("property uchar green\n")
        stream.write("property uchar blue\n")
        stream.write("end_header\n")
        for x, y, z, red, green, blue, _has_color in points:
            stream.write(f"{x:.9f} {y:.9f} {z:.9f} {red} {green} {blue}\n")


def compute_trajectory_path_length(poses):
    if len(poses) < 2:
        return 0.0

    total = 0.0
    _stamp, previous_x, previous_y, previous_z, *_orientation = poses[0]
    for _stamp, current_x, current_y, current_z, *_orientation in poses[1:]:
        dx = current_x - previous_x
        dy = current_y - previous_y
        dz = current_z - previous_z
        total += math.sqrt(dx * dx + dy * dy + dz * dz)
        previous_x = current_x
        previous_y = current_y
        previous_z = current_z
    return total


def compute_point_bounds(points):
    if not points:
        return {"min": [], "max": []}

    min_x = min(point[0] for point in points)
    min_y = min(point[1] for point in points)
    min_z = min(point[2] for point in points)
    max_x = max(point[0] for point in points)
    max_y = max(point[1] for point in points)
    max_z = max(point[2] for point in points)
    return {
        "min": [min_x, min_y, min_z],
        "max": [max_x, max_y, max_z],
    }


def compute_color_stats(points):
    if not points:
        return {
            "points_with_color": 0,
            "points_without_color": 0,
            "mean_rgb": [],
        }

    points_with_color = sum(1 for point in points if point[6])
    channel_sums = [0, 0, 0]
    for point in points:
        channel_sums[0] += int(point[3])
        channel_sums[1] += int(point[4])
        channel_sums[2] += int(point[5])

    count = len(points)
    return {
        "points_with_color": points_with_color,
        "points_without_color": count - points_with_color,
        "mean_rgb": [value / count for value in channel_sums],
    }


def compute_topic_rates(topic_counts, duration_sec):
    if duration_sec <= 0.0:
        return {topic: 0.0 for topic in sorted(topic_counts)}
    return {
        topic: count / duration_sec
        for topic, count in sorted(topic_counts.items())
    }


def detect_bag_format(bag_path, bag_format):
    if bag_format == "auto":
        if bag_path.is_dir():
            return "ros2"
        if bag_path.suffix == ".bag":
            return "ros1"
        raise ValueError("cannot auto-detect bag format; use --bag-format ros1 or ros2")
    return bag_format


def detect_ros2_storage_id(bag_path):
    import yaml

    metadata_path = bag_path / "metadata.yaml"
    if not metadata_path.exists():
        return ""
    metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
    return metadata.get("rosbag2_bagfile_information", {}).get("storage_identifier", "")


def open_ros2_reader(bag_path):
    import rosbag2_py

    storage_options = rosbag2_py.StorageOptions(
        uri=str(bag_path),
        storage_id=detect_ros2_storage_id(bag_path),
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    return reader


def read_ros2_bag(bag_path, args):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = open_ros2_reader(bag_path)
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    message_types = {}

    topic_counts = {}
    poses = []
    points = []
    first_stamp = None
    last_stamp = None

    while reader.has_next():
        topic, serialized, bag_time = reader.read_next()
        topic_counts[topic] = topic_counts.get(topic, 0) + 1
        first_stamp = bag_time if first_stamp is None else min(first_stamp, bag_time)
        last_stamp = bag_time if last_stamp is None else max(last_stamp, bag_time)

        should_decode_pose = topic == args.pose_topic
        should_decode_points = topic == args.pointcloud_topic and len(points) < args.max_points
        if should_decode_pose or should_decode_points:
            msg_type = topic_types.get(topic)
            if msg_type is not None:
                if msg_type not in message_types:
                    message_types[msg_type] = get_message(msg_type)
                msg = deserialize_message(serialized, message_types[msg_type])

                if should_decode_pose:
                    poses.append(pose_to_record(msg))
                elif should_decode_points:
                    remaining = args.max_points - len(points)
                    points.extend(iter_xyzrgb_points(msg, remaining))

        if args.max_messages > 0 and sum(topic_counts.values()) >= args.max_messages:
            break

    return BagReadResult(
        bag_format="ros2",
        storage_identifier=detect_ros2_storage_id(bag_path),
        topic_counts=topic_counts,
        poses=poses,
        points=points,
        first_stamp_nsec=first_stamp,
        last_stamp_nsec=last_stamp,
    )


def read_ros1_bag(bag_path, args):
    try:
        from rosbags.highlevel import AnyReader
    except ImportError as exc:
        raise RuntimeError(
            "ROS1 offline extraction requires the optional Python package 'rosbags'. "
            "Install it in the extraction environment with "
            "`/usr/bin/python3 -m pip install --user rosbags`."
        ) from exc

    topic_counts = {}
    poses = []
    points = []
    first_stamp = None
    last_stamp = None

    with AnyReader([bag_path]) as reader:
        for connection, bag_time, rawdata in reader.messages():
            topic = connection.topic
            topic_counts[topic] = topic_counts.get(topic, 0) + 1
            first_stamp = bag_time if first_stamp is None else min(first_stamp, bag_time)
            last_stamp = bag_time if last_stamp is None else max(last_stamp, bag_time)

            should_decode_pose = topic == args.pose_topic
            should_decode_points = topic == args.pointcloud_topic and len(points) < args.max_points
            if should_decode_pose or should_decode_points:
                msg = reader.deserialize(rawdata, connection.msgtype)
                if should_decode_pose:
                    poses.append(pose_to_record(msg))
                elif should_decode_points:
                    remaining = args.max_points - len(points)
                    points.extend(iter_xyzrgb_points(msg, remaining))

            if args.max_messages > 0 and sum(topic_counts.values()) >= args.max_messages:
                break

    return BagReadResult(
        bag_format="ros1",
        storage_identifier="rosbag1",
        topic_counts=topic_counts,
        poses=poses,
        points=points,
        first_stamp_nsec=first_stamp,
        last_stamp_nsec=last_stamp,
    )


def read_bag(bag_path, args):
    bag_format = detect_bag_format(bag_path, args.bag_format)
    if bag_format == "ros2":
        return read_ros2_bag(bag_path, args)
    if bag_format == "ros1":
        return read_ros1_bag(bag_path, args)
    raise ValueError(f"unsupported bag format: {bag_format}")


def run(args):
    bag_path = Path(args.bag).expanduser().resolve()
    output_dir = Path(args.output).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    result = read_bag(bag_path, args)

    trajectory_path = output_dir / "trajectory.tum"
    point_cloud_path = output_dir / "point_cloud_debug.ply"
    metrics_path = output_dir / "metrics.json"

    write_tum_trajectory(trajectory_path, result.poses)
    write_xyzrgb_ply(point_cloud_path, result.points)

    duration_sec = 0.0
    if result.first_stamp_nsec is not None and result.last_stamp_nsec is not None:
        duration_sec = max(0.0, (result.last_stamp_nsec - result.first_stamp_nsec) * 1e-9)

    metrics = {
        "bag": str(bag_path),
        "bag_format": result.bag_format,
        "storage_identifier": result.storage_identifier,
        "duration_sec": duration_sec,
        "message_count": int(sum(result.topic_counts.values())),
        "topic_counts": result.topic_counts,
        "topic_hz": compute_topic_rates(result.topic_counts, duration_sec),
        "pose_topic": args.pose_topic,
        "pointcloud_topic": args.pointcloud_topic,
        "trajectory_poses": len(result.poses),
        "trajectory": {
            "poses": len(result.poses),
            "path_length_m": compute_trajectory_path_length(result.poses),
        },
        "debug_points": len(result.points),
        "debug_cloud": {
            "points": len(result.points),
            "bounds": compute_point_bounds(result.points),
            **compute_color_stats(result.points),
        },
        "outputs": {
            "trajectory_tum": str(trajectory_path),
            "point_cloud_debug_ply": str(point_cloud_path),
        },
    }
    metrics_path.write_text(json.dumps(metrics, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(metrics, indent=2, sort_keys=True))


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Offline Gaussian-LIC bag reader for reproducibility artifacts."
    )
    parser.add_argument("--bag", required=True, help="ROS2 bag directory or ROS1 .bag")
    parser.add_argument("--bag-format", choices=("auto", "ros1", "ros2"), default="auto")
    parser.add_argument("--output", required=True, help="Output directory")
    parser.add_argument("--pose-topic", default="/pose_for_gs")
    parser.add_argument("--pointcloud-topic", default="/points_for_gs")
    parser.add_argument("--max-points", type=int, default=200000)
    parser.add_argument("--max-messages", type=int, default=0)
    args = parser.parse_args(argv)
    try:
        run(args)
    except RuntimeError as exc:
        print(f"offline extraction failed: {exc}", file=sys.stderr)
        return 3
    except Exception as exc:  # noqa: BLE001 - CLI reports artifact extraction failures uniformly.
        print(f"offline extraction failed: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
