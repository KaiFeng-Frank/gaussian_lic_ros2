# SPDX-License-Identifier: GPL-3.0-or-later

import argparse
import json
from pathlib import Path
import sys


DEFAULT_CONTRACT = {
    "/points_for_gs": "sensor_msgs/msg/PointCloud2",
    "/pose_for_gs": "geometry_msgs/msg/PoseStamped",
    "/image_for_gs": "sensor_msgs/msg/Image",
    "/camera_info_for_gs": "sensor_msgs/msg/CameraInfo",
    "/depth_for_gs": "sensor_msgs/msg/Image",
    "/imu_for_gs": "sensor_msgs/msg/Imu",
}


def load_ros2_metadata(bag_path):
    import yaml

    metadata_path = Path(bag_path).expanduser().resolve() / "metadata.yaml"
    if not metadata_path.exists():
        raise FileNotFoundError(f"metadata.yaml not found under {metadata_path.parent}")
    with metadata_path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    info = data.get("rosbag2_bagfile_information", {})
    if not isinstance(info, dict):
        raise ValueError("metadata.yaml is missing rosbag2_bagfile_information")
    return info


def load_ros1_info(bag_path):
    try:
        from rosbags.rosbag1 import Reader
    except ImportError as exc:
        raise RuntimeError(
            "ROS1 bag contract checks require the optional Python package 'rosbags'. "
            "Install it in the conversion environment with "
            "`/usr/bin/python3 -m pip install --user rosbags`."
        ) from exc

    with Reader(Path(bag_path).expanduser().resolve()) as reader:
        topics = {
            name: {
                "type": info.msgtype,
                "serialization_format": "ros1",
                "message_count": int(info.msgcount),
            }
            for name, info in reader.topics.items()
        }
        return {
            "storage_identifier": "rosbag1",
            "ros_distro": "ros1",
            "duration": {"nanoseconds": int(reader.duration)},
            "message_count": int(reader.message_count),
            "topics": topics,
        }


def ros2_metadata_topics(info):
    topics = {}
    for item in info.get("topics_with_message_count", []):
        topic_metadata = item.get("topic_metadata", {})
        name = topic_metadata.get("name")
        if not name:
            continue
        topics[name] = {
            "type": topic_metadata.get("type", ""),
            "serialization_format": topic_metadata.get("serialization_format", ""),
            "message_count": int(item.get("message_count", 0)),
        }
    return topics


def load_bag_info(bag_path, bag_format):
    path = Path(bag_path).expanduser().resolve()
    if bag_format == "auto":
        if path.is_dir():
            bag_format = "ros2"
        elif path.suffix == ".bag":
            bag_format = "ros1"
        else:
            raise ValueError("cannot auto-detect bag format; use --bag-format ros1 or ros2")

    if bag_format == "ros2":
        info = load_ros2_metadata(path)
        topics = ros2_metadata_topics(info)
    elif bag_format == "ros1":
        info = load_ros1_info(path)
        topics = info["topics"]
    else:
        raise ValueError(f"unsupported bag format: {bag_format}")
    return info, topics, bag_format


def load_contract(args):
    contract = dict(DEFAULT_CONTRACT)
    overrides = {
        "/points_for_gs": args.pointcloud_topic,
        "/pose_for_gs": args.pose_topic,
        "/image_for_gs": args.image_topic,
        "/camera_info_for_gs": args.camera_info_topic,
        "/depth_for_gs": args.depth_topic,
        "/imu_for_gs": args.imu_topic,
    }
    for default_name, actual_name in overrides.items():
        if actual_name == default_name:
            continue
        msg_type = contract.pop(default_name)
        contract[actual_name] = msg_type
    return contract


def check_contract(topics, contract):
    checks = {}
    errors = []
    for topic, expected_type in contract.items():
        actual = topics.get(topic)
        if actual is None:
            checks[topic] = {
                "ok": False,
                "expected_type": expected_type,
                "actual_type": None,
                "message_count": 0,
                "error": "missing topic",
            }
            errors.append(f"{topic}: missing topic")
            continue

        actual_type = actual["type"]
        count = actual["message_count"]
        topic_errors = []
        if actual_type != expected_type:
            topic_errors.append(f"type is {actual_type}, expected {expected_type}")
        if count <= 0:
            topic_errors.append("message_count is zero")

        checks[topic] = {
            "ok": not topic_errors,
            "expected_type": expected_type,
            "actual_type": actual_type,
            "serialization_format": actual["serialization_format"],
            "message_count": count,
            "error": "; ".join(topic_errors),
        }
        errors.extend(f"{topic}: {error}" for error in topic_errors)

    return checks, errors


def build_report(args):
    info, topics, detected_format = load_bag_info(args.bag, args.bag_format)
    contract = load_contract(args)
    checks, errors = check_contract(topics, contract)
    duration_nsec = int(info.get("duration", {}).get("nanoseconds", 0))
    return {
        "bag": str(Path(args.bag).expanduser().resolve()),
        "bag_format": detected_format,
        "storage_identifier": info.get("storage_identifier", ""),
        "ros_distro": info.get("ros_distro", ""),
        "duration_sec": duration_nsec * 1e-9,
        "message_count": int(info.get("message_count", 0)),
        "contract_ok": not errors,
        "errors": errors,
        "required_topics": checks,
        "all_topics": topics,
    }


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Validate that a ROS1 or ROS2 bag contains the Gaussian-LIC mapper topic contract."
    )
    parser.add_argument("--bag", required=True, help="ROS2 bag directory or ROS1 .bag")
    parser.add_argument("--bag-format", choices=("auto", "ros1", "ros2"), default="auto")
    parser.add_argument("--pointcloud-topic", default="/points_for_gs")
    parser.add_argument("--pose-topic", default="/pose_for_gs")
    parser.add_argument("--image-topic", default="/image_for_gs")
    parser.add_argument("--camera-info-topic", default="/camera_info_for_gs")
    parser.add_argument("--depth-topic", default="/depth_for_gs")
    parser.add_argument("--imu-topic", default="/imu_for_gs")
    parser.add_argument("--json", action="store_true", help="Print full JSON report")
    args = parser.parse_args(argv)

    try:
        report = build_report(args)
    except RuntimeError as exc:
        print(f"bag contract check failed: {exc}", file=sys.stderr)
        return 3
    except Exception as exc:  # noqa: BLE001 - CLI reports any metadata failure uniformly.
        print(f"bag contract check failed: {exc}", file=sys.stderr)
        return 2

    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))
    elif report["contract_ok"]:
        print(
            f"bag contract OK: {report['bag']} "
            f"topics={len(report['required_topics'])} messages={report['message_count']}"
        )
    else:
        print(json.dumps(report, indent=2, sort_keys=True))

    return 0 if report["contract_ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
