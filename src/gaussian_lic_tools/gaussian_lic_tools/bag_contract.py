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

MINIMAL_REQUIRED_TOPICS = {
    "/points_for_gs",
    "/pose_for_gs",
    "/image_for_gs",
}

FRONTEND_RAW_REQUIRED_CONTRACT = {
    "/camera/image": "sensor_msgs/msg/Image",
    "/camera/camera_info": "sensor_msgs/msg/CameraInfo",
    "/livox/lidar": "sensor_msgs/msg/PointCloud2",
    "/imu": "sensor_msgs/msg/Imu",
}

FRONTEND_RAW_OPTIONAL_CONTRACT = {
    "/camera/depth": "sensor_msgs/msg/Image",
}

FRONTEND_RAW_POSE_ALTERNATIVES = {
    "/gaussian_lic/frontend/pose": "geometry_msgs/msg/PoseStamped",
    "/gaussian_lic/frontend/input_odometry": "nav_msgs/msg/Odometry",
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


def load_contracts(args):
    if args.contract == "frontend_raw":
        required = {
            args.raw_image_topic: FRONTEND_RAW_REQUIRED_CONTRACT["/camera/image"],
            args.raw_camera_info_topic: FRONTEND_RAW_REQUIRED_CONTRACT["/camera/camera_info"],
            args.raw_pointcloud_topic: FRONTEND_RAW_REQUIRED_CONTRACT["/livox/lidar"],
            args.raw_imu_topic: FRONTEND_RAW_REQUIRED_CONTRACT["/imu"],
        }
        optional = {
            args.raw_depth_topic: FRONTEND_RAW_OPTIONAL_CONTRACT["/camera/depth"],
            args.frontend_pose_topic: FRONTEND_RAW_POSE_ALTERNATIVES[
                "/gaussian_lic/frontend/pose"
            ],
            args.raw_odometry_topic: FRONTEND_RAW_POSE_ALTERNATIVES[
                "/gaussian_lic/frontend/input_odometry"
            ],
        }
        return required, optional

    overrides = {
        "/points_for_gs": args.pointcloud_topic,
        "/pose_for_gs": args.pose_topic,
        "/image_for_gs": args.image_topic,
        "/camera_info_for_gs": args.camera_info_topic,
        "/depth_for_gs": args.depth_topic,
        "/imu_for_gs": args.imu_topic,
    }

    required_defaults = set(DEFAULT_CONTRACT)
    if args.contract == "mapper_minimal":
        required_defaults = set(MINIMAL_REQUIRED_TOPICS)

    required = {}
    optional = {}
    for default_name, actual_name in overrides.items():
        target = required if default_name in required_defaults else optional
        target[actual_name] = DEFAULT_CONTRACT[default_name]
    return required, optional


def check_topic_contract(topics, contract, required):
    checks = {}
    errors = []
    for topic, expected_type in contract.items():
        actual = topics.get(topic)
        if actual is None:
            checks[topic] = {
                "ok": not required,
                "required": required,
                "present": False,
                "expected_type": expected_type,
                "actual_type": None,
                "message_count": 0,
                "error": "missing topic" if required else "missing optional topic",
            }
            if required:
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
            "required": required,
            "present": True,
            "expected_type": expected_type,
            "actual_type": actual_type,
            "serialization_format": actual["serialization_format"],
            "message_count": count,
            "error": "; ".join(topic_errors),
        }
        errors.extend(f"{topic}: {error}" for error in topic_errors)

    return checks, errors


def check_alternative_group(topics, contract, group_name):
    checks, errors = check_topic_contract(topics, contract, required=False)
    ok = any(check["ok"] and check["present"] for check in checks.values())
    if not ok:
        errors.append(
            f"{group_name}: at least one valid topic is required from "
            f"{', '.join(contract)}"
        )
    return {
        "ok": ok,
        "required": True,
        "alternatives": checks,
    }, errors


def build_report(args):
    info, topics, detected_format = load_bag_info(args.bag, args.bag_format)
    required_contract, optional_contract = load_contracts(args)
    required_checks, required_errors = check_topic_contract(topics, required_contract, required=True)
    optional_checks, optional_errors = check_topic_contract(topics, optional_contract, required=False)
    alternative_groups = {}
    alternative_errors = []
    if args.contract == "frontend_raw":
        pose_contract = {
            args.frontend_pose_topic: FRONTEND_RAW_POSE_ALTERNATIVES[
                "/gaussian_lic/frontend/pose"
            ],
            args.raw_odometry_topic: FRONTEND_RAW_POSE_ALTERNATIVES[
                "/gaussian_lic/frontend/input_odometry"
            ],
        }
        alternative_group, alternative_errors = check_alternative_group(
            topics, pose_contract, "frontend_pose_source"
        )
        alternative_groups["frontend_pose_source"] = alternative_group
    errors = required_errors + optional_errors + alternative_errors
    duration_nsec = int(info.get("duration", {}).get("nanoseconds", 0))
    return {
        "bag": str(Path(args.bag).expanduser().resolve()),
        "bag_format": detected_format,
        "contract": args.contract,
        "storage_identifier": info.get("storage_identifier", ""),
        "ros_distro": info.get("ros_distro", ""),
        "duration_sec": duration_nsec * 1e-9,
        "message_count": int(info.get("message_count", 0)),
        "contract_ok": not errors,
        "errors": errors,
        "required_topics": required_checks,
        "optional_topics": optional_checks,
        "alternative_groups": alternative_groups,
        "all_topics": topics,
    }


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=(
            "Validate that a ROS1 or ROS2 bag contains a Gaussian-LIC mapper or "
            "LIC2 frontend topic contract."
        )
    )
    parser.add_argument("--bag", required=True, help="ROS2 bag directory or ROS1 .bag")
    parser.add_argument("--bag-format", choices=("auto", "ros1", "ros2"), default="auto")
    parser.add_argument(
        "--contract",
        choices=("full", "mapper_minimal", "frontend_raw"),
        default="full",
        help=(
            "Topic contract to validate. full requires the complete mapper input set; "
            "mapper_minimal requires point cloud, pose, and image only; "
            "frontend_raw validates raw topics consumed by lic2_contract_adapter."
        ),
    )
    parser.add_argument("--pointcloud-topic", default="/points_for_gs")
    parser.add_argument("--pose-topic", default="/pose_for_gs")
    parser.add_argument("--image-topic", default="/image_for_gs")
    parser.add_argument("--camera-info-topic", default="/camera_info_for_gs")
    parser.add_argument("--depth-topic", default="/depth_for_gs")
    parser.add_argument("--imu-topic", default="/imu_for_gs")
    parser.add_argument("--raw-image-topic", default="/camera/image")
    parser.add_argument("--raw-camera-info-topic", default="/camera/camera_info")
    parser.add_argument("--raw-depth-topic", default="/camera/depth")
    parser.add_argument("--raw-pointcloud-topic", default="/livox/lidar")
    parser.add_argument("--raw-imu-topic", default="/imu")
    parser.add_argument("--frontend-pose-topic", default="/gaussian_lic/frontend/pose")
    parser.add_argument("--raw-odometry-topic", default="/gaussian_lic/frontend/input_odometry")
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
