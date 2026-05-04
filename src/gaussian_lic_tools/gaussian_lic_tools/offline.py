# SPDX-License-Identifier: GPL-3.0-or-later

import argparse
import json
from pathlib import Path
import struct

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import PointField
import rosbag2_py
from sensor_msgs_py import point_cloud2
import yaml


def stamp_to_float(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def field_metadata(cloud_msg):
    return {field.name: field.datatype for field in cloud_msg.fields}


def get_point_value(point, index, name):
    try:
        return point[index]
    except (IndexError, TypeError, ValueError):
        return point[name]


def packed_rgb_to_channels(value, packed_as_float):
    if packed_as_float:
        bits = struct.unpack("<I", struct.pack("<f", value))[0]
    else:
        bits = int(value)
    red = (bits >> 16) & 0xFF
    green = (bits >> 8) & 0xFF
    blue = bits & 0xFF
    return red, green, blue


def scalar_color_to_u8(value):
    numeric = float(value)
    if 0.0 <= numeric <= 1.0:
        numeric *= 255.0
    return max(0, min(255, int(round(numeric))))


def iter_xyzrgb_points(cloud_msg, max_points):
    fields = field_metadata(cloud_msg)
    color_mode = "none"
    rgb_packed_as_float = False
    if "rgb" in fields:
        color_mode = "rgb"
        rgb_packed_as_float = fields["rgb"] == PointField.FLOAT32
        requested_fields = ("x", "y", "z", "rgb")
    elif "rgba" in fields:
        color_mode = "rgba"
        rgb_packed_as_float = fields["rgba"] == PointField.FLOAT32
        requested_fields = ("x", "y", "z", "rgba")
    elif {"r", "g", "b"}.issubset(fields):
        color_mode = "channels"
        requested_fields = ("x", "y", "z", "r", "g", "b")
    else:
        requested_fields = ("x", "y", "z")

    points = point_cloud2.read_points(
        cloud_msg,
        field_names=requested_fields,
        skip_nans=True,
    )
    count = 0
    for point in points:
        if count >= max_points:
            return
        try:
            x = float(get_point_value(point, 0, "x"))
            y = float(get_point_value(point, 1, "y"))
            z = float(get_point_value(point, 2, "z"))
            if color_mode in {"rgb", "rgba"}:
                red, green, blue = packed_rgb_to_channels(
                    get_point_value(point, 3, color_mode),
                    rgb_packed_as_float,
                )
            elif color_mode == "channels":
                red = scalar_color_to_u8(get_point_value(point, 3, "r"))
                green = scalar_color_to_u8(get_point_value(point, 4, "g"))
                blue = scalar_color_to_u8(get_point_value(point, 5, "b"))
            else:
                red, green, blue = 255, 255, 255
        except (IndexError, KeyError, TypeError, ValueError):
            continue
        yield x, y, z, red, green, blue
        count += 1


def write_tum_trajectory(path, poses):
    with path.open("w", encoding="utf-8") as stream:
        for stamp, pose in poses:
            p = pose.pose.position
            q = pose.pose.orientation
            stream.write(
                f"{stamp:.9f} {p.x:.9f} {p.y:.9f} {p.z:.9f} "
                f"{q.x:.9f} {q.y:.9f} {q.z:.9f} {q.w:.9f}\n"
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
        for x, y, z, red, green, blue in points:
            stream.write(f"{x:.9f} {y:.9f} {z:.9f} {red} {green} {blue}\n")


def detect_storage_id(bag_path):
    metadata_path = bag_path / "metadata.yaml"
    if not metadata_path.exists():
        return ""
    metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
    return metadata.get("rosbag2_bagfile_information", {}).get("storage_identifier", "")


def open_reader(bag_path):
    storage_options = rosbag2_py.StorageOptions(
        uri=str(bag_path),
        storage_id=detect_storage_id(bag_path),
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    return reader


def run(args):
    bag_path = Path(args.bag).expanduser().resolve()
    output_dir = Path(args.output).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    reader = open_reader(bag_path)
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

      msg_type = topic_types.get(topic)
      if msg_type is None:
          continue
      if msg_type not in message_types:
          message_types[msg_type] = get_message(msg_type)
      msg = deserialize_message(serialized, message_types[msg_type])

      if topic == args.pose_topic:
          poses.append((stamp_to_float(msg.header.stamp), msg))
      elif topic == args.pointcloud_topic and len(points) < args.max_points:
          remaining = args.max_points - len(points)
          points.extend(iter_xyzrgb_points(msg, remaining))

      if args.max_messages > 0 and sum(topic_counts.values()) >= args.max_messages:
          break

    trajectory_path = output_dir / "trajectory.tum"
    point_cloud_path = output_dir / "point_cloud_debug.ply"
    metrics_path = output_dir / "metrics.json"

    write_tum_trajectory(trajectory_path, poses)
    write_xyzrgb_ply(point_cloud_path, points)

    duration_sec = 0.0
    if first_stamp is not None and last_stamp is not None:
        duration_sec = max(0.0, (last_stamp - first_stamp) * 1e-9)

    metrics = {
        "bag": str(bag_path),
        "duration_sec": duration_sec,
        "message_count": int(sum(topic_counts.values())),
        "topic_counts": topic_counts,
        "pose_topic": args.pose_topic,
        "pointcloud_topic": args.pointcloud_topic,
        "trajectory_poses": len(poses),
        "debug_points": len(points),
        "outputs": {
            "trajectory_tum": str(trajectory_path),
            "point_cloud_debug_ply": str(point_cloud_path),
        },
    }
    metrics_path.write_text(json.dumps(metrics, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(metrics, indent=2, sort_keys=True))


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Offline Gaussian-LIC ROS2 rosbag2 reader for reproducibility artifacts."
    )
    parser.add_argument("--bag", required=True, help="rosbag2 directory or storage URI")
    parser.add_argument("--output", required=True, help="Output directory")
    parser.add_argument("--pose-topic", default="/pose_for_gs")
    parser.add_argument("--pointcloud-topic", default="/points_for_gs")
    parser.add_argument("--max-points", type=int, default=200000)
    parser.add_argument("--max-messages", type=int, default=0)
    args = parser.parse_args(argv)
    run(args)


if __name__ == "__main__":
    main()
