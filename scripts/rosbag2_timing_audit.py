#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later

import argparse
import json
from pathlib import Path
import sqlite3
import sys

import yaml


def load_metadata(bag_dir):
    metadata_path = bag_dir / "metadata.yaml"
    if not metadata_path.is_file():
        raise FileNotFoundError(f"missing rosbag2 metadata.yaml: {metadata_path}")
    data = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
    info = data.get("rosbag2_bagfile_information", data)
    return metadata_path, info


def metadata_topics(info):
    topics = {}
    for item in info.get("topics_with_message_count", []) or []:
        topic_metadata = item.get("topic_metadata", {})
        name = topic_metadata.get("name")
        if not name:
            continue
        topics[name] = {
            "type": topic_metadata.get("type", ""),
            "serialization_format": topic_metadata.get("serialization_format", ""),
            "offered_qos_profiles": topic_metadata.get("offered_qos_profiles", ""),
            "message_count": int(item.get("message_count", 0) or 0),
        }
    return topics


def storage_files(bag_dir, info):
    files = []
    for name in info.get("relative_file_paths", []) or []:
        path = bag_dir / name
        if path.is_file():
            files.append(path)
    if not files:
        files.extend(sorted(bag_dir.glob("*.db3")))
        files.extend(sorted(bag_dir.glob("*.mcap")))
    return files


def audit_sqlite_file(path, max_time_regression_ns):
    report = {
        "path": str(path),
        "ok": True,
        "message_count": 0,
        "first_timestamp_ns": None,
        "last_timestamp_ns": None,
        "topic_counts": {},
        "global_time_regressions": 0,
        "topic_time_regressions": {},
        "errors": [],
    }
    previous_global = None
    previous_by_topic = {}
    connection = None
    try:
        connection = sqlite3.connect(str(path))
        cursor = connection.cursor()
        cursor.execute(
            """
            SELECT topics.name, messages.timestamp
            FROM messages
            JOIN topics ON messages.topic_id = topics.id
            ORDER BY messages.id
            """
        )
        for topic, stamp_ns in cursor:
            stamp_ns = int(stamp_ns)
            if report["first_timestamp_ns"] is None:
                report["first_timestamp_ns"] = stamp_ns
            report["last_timestamp_ns"] = stamp_ns
            report["message_count"] += 1
            report["topic_counts"][topic] = report["topic_counts"].get(topic, 0) + 1
            if previous_global is not None and stamp_ns + max_time_regression_ns < previous_global:
                report["global_time_regressions"] += 1
            previous_topic = previous_by_topic.get(topic)
            if previous_topic is not None and stamp_ns + max_time_regression_ns < previous_topic:
                report["topic_time_regressions"][topic] = report["topic_time_regressions"].get(topic, 0) + 1
            previous_global = stamp_ns
            previous_by_topic[topic] = stamp_ns
    except sqlite3.Error as exc:
        report["ok"] = False
        report["errors"].append(f"sqlite audit failed: {exc}")
    finally:
        if connection is not None:
            connection.close()

    if report["global_time_regressions"]:
        report["ok"] = False
        report["errors"].append(f"global message timestamps regressed {report['global_time_regressions']} times")
    for topic, count in sorted(report["topic_time_regressions"].items()):
        report["ok"] = False
        report["errors"].append(f"{topic} timestamps regressed {count} times")
    if report["message_count"] == 0:
        report["ok"] = False
        report["errors"].append("sqlite bag has no messages")
    return report


def audit_rosbag2_headers(bag_dir, storage_id, max_time_regression_ns):
    report = {
        "available": False,
        "ok": True,
        "message_count": 0,
        "header_stamp_counts": {},
        "header_stamp_regressions": {},
        "record_time_regressions": 0,
        "topic_record_time_regressions": {},
        "errors": [],
    }
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        return audit_rosbags_headers(bag_dir, max_time_regression_ns, f"rosbag2 Python reader unavailable: {exc}")

    report["available"] = True
    report["reader"] = "rosbag2_py"
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=storage_id or "")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    try:
        reader.open(storage_options, converter_options)
        topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
        message_types = {}
        previous_record_time = None
        previous_record_time_by_topic = {}
        previous_header_stamp_by_topic = {}
        while reader.has_next():
            topic, serialized, record_time_ns = reader.read_next()
            record_time_ns = int(record_time_ns)
            report["message_count"] += 1
            if (
                previous_record_time is not None and
                record_time_ns + max_time_regression_ns < previous_record_time
            ):
                report["record_time_regressions"] += 1
            previous_topic_record_time = previous_record_time_by_topic.get(topic)
            if (
                previous_topic_record_time is not None and
                record_time_ns + max_time_regression_ns < previous_topic_record_time
            ):
                report["topic_record_time_regressions"][topic] = (
                    report["topic_record_time_regressions"].get(topic, 0) + 1
                )
            previous_record_time = record_time_ns
            previous_record_time_by_topic[topic] = record_time_ns

            msg_type = topic_types.get(topic)
            if not msg_type:
                continue
            if msg_type not in message_types:
                message_types[msg_type] = get_message(msg_type)
            msg = deserialize_message(serialized, message_types[msg_type])
            header = getattr(msg, "header", None)
            stamp = getattr(header, "stamp", None)
            if stamp is None:
                continue
            header_stamp_ns = int(stamp.sec) * 1000000000 + int(stamp.nanosec)
            report["header_stamp_counts"][topic] = report["header_stamp_counts"].get(topic, 0) + 1
            previous_header_stamp = previous_header_stamp_by_topic.get(topic)
            if (
                previous_header_stamp is not None and
                header_stamp_ns + max_time_regression_ns < previous_header_stamp
            ):
                report["header_stamp_regressions"][topic] = (
                    report["header_stamp_regressions"].get(topic, 0) + 1
                )
            previous_header_stamp_by_topic[topic] = header_stamp_ns
    except Exception as exc:  # noqa: BLE001 - malformed bag reports should stay data-shaped.
        report["ok"] = False
        report["errors"].append(f"rosbag2 message audit failed: {exc}")

    if report["record_time_regressions"]:
        report["ok"] = False
        report["errors"].append(f"record timestamps regressed {report['record_time_regressions']} times")
    for topic, count in sorted(report["topic_record_time_regressions"].items()):
        report["ok"] = False
        report["errors"].append(f"{topic} record timestamps regressed {count} times")
    for topic, count in sorted(report["header_stamp_regressions"].items()):
        report["ok"] = False
        report["errors"].append(f"{topic} header stamps regressed {count} times")
    if report["message_count"] == 0:
        report["ok"] = False
        report["errors"].append("rosbag2 reader returned no messages")
    return report


def audit_rosbags_headers(bag_dir, max_time_regression_ns, unavailable_reason):
    report = {
        "available": False,
        "ok": True,
        "reader": "rosbags",
        "message_count": 0,
        "header_stamp_counts": {},
        "header_stamp_regressions": {},
        "record_time_regressions": 0,
        "topic_record_time_regressions": {},
        "errors": [],
    }
    try:
        from rosbags.highlevel import AnyReader
        from rosbags.typesys import Stores, get_typestore
    except ImportError as exc:
        report["errors"].append(unavailable_reason)
        report["errors"].append(f"rosbags reader unavailable: {exc}")
        return report

    report["available"] = True
    typestore = get_typestore(Stores.ROS2_JAZZY)
    previous_record_time = None
    previous_record_time_by_topic = {}
    previous_header_stamp_by_topic = {}
    try:
        with AnyReader([bag_dir], default_typestore=typestore) as reader:
            for connection, record_time_ns, serialized in reader.messages():
                topic = connection.topic
                record_time_ns = int(record_time_ns)
                report["message_count"] += 1
                if (
                    previous_record_time is not None and
                    record_time_ns + max_time_regression_ns < previous_record_time
                ):
                    report["record_time_regressions"] += 1
                previous_topic_record_time = previous_record_time_by_topic.get(topic)
                if (
                    previous_topic_record_time is not None and
                    record_time_ns + max_time_regression_ns < previous_topic_record_time
                ):
                    report["topic_record_time_regressions"][topic] = (
                        report["topic_record_time_regressions"].get(topic, 0) + 1
                    )
                previous_record_time = record_time_ns
                previous_record_time_by_topic[topic] = record_time_ns

                msg = reader.deserialize(serialized, connection.msgtype)
                header = getattr(msg, "header", None)
                stamp = getattr(header, "stamp", None)
                if stamp is None:
                    continue
                header_stamp_ns = int(stamp.sec) * 1000000000 + int(stamp.nanosec)
                report["header_stamp_counts"][topic] = report["header_stamp_counts"].get(topic, 0) + 1
                previous_header_stamp = previous_header_stamp_by_topic.get(topic)
                if (
                    previous_header_stamp is not None and
                    header_stamp_ns + max_time_regression_ns < previous_header_stamp
                ):
                    report["header_stamp_regressions"][topic] = (
                        report["header_stamp_regressions"].get(topic, 0) + 1
                    )
                previous_header_stamp_by_topic[topic] = header_stamp_ns
    except Exception as exc:  # noqa: BLE001 - malformed bag reports should stay data-shaped.
        report["ok"] = False
        report["errors"].append(f"rosbags message audit failed: {exc}")

    if report["record_time_regressions"]:
        report["ok"] = False
        report["errors"].append(f"record timestamps regressed {report['record_time_regressions']} times")
    for topic, count in sorted(report["topic_record_time_regressions"].items()):
        report["ok"] = False
        report["errors"].append(f"{topic} record timestamps regressed {count} times")
    for topic, count in sorted(report["header_stamp_regressions"].items()):
        report["ok"] = False
        report["errors"].append(f"{topic} header stamps regressed {count} times")
    if report["message_count"] == 0:
        report["ok"] = False
        report["errors"].append("rosbags reader returned no messages")
    return report


def build_report(args):
    bag_dir = Path(args.bag).expanduser().resolve()
    metadata_path, info = load_metadata(bag_dir)
    storage_id = info.get("storage_identifier", "")
    files = storage_files(bag_dir, info)
    topics = metadata_topics(info)
    report = {
        "schema": "gaussian_lic_rosbag2_timing_audit/v1",
        "bag": str(bag_dir),
        "metadata": str(metadata_path),
        "storage_identifier": storage_id,
        "metadata_message_count": int(info.get("message_count", 0) or 0),
        "duration_ns": (info.get("duration") or {}).get("nanoseconds"),
        "starting_time_ns": (info.get("starting_time") or {}).get("nanoseconds_since_epoch"),
        "topics": topics,
        "storage_files": [str(path) for path in files],
        "sqlite_files": [],
        "message_header_audit": None,
        "metadata_only": False,
        "ok": True,
        "errors": [],
    }

    for topic in args.required_topic or []:
        if topic not in topics or topics[topic]["message_count"] <= 0:
            report["errors"].append(f"missing required topic or empty topic in metadata: {topic}")

    sqlite_paths = [path for path in files if path.suffix == ".db3"]
    if sqlite_paths:
        for path in sqlite_paths:
            report["sqlite_files"].append(audit_sqlite_file(path, args.max_time_regression_ns))
    else:
        message_audit = audit_rosbag2_headers(bag_dir, storage_id, args.max_time_regression_ns)
        report["message_header_audit"] = message_audit
        report["metadata_only"] = not message_audit.get("available", False)
        if args.strict_storage and report["metadata_only"]:
            report["errors"].append(
                f"strict storage audit requires inspectable message data; storage_identifier={storage_id!r}"
            )

    sqlite_ok = all(item.get("ok", False) for item in report["sqlite_files"]) if report["sqlite_files"] else True
    message_audit = report.get("message_header_audit")
    message_audit_ok = message_audit.get("ok", False) if message_audit else True
    if report["metadata_message_count"] <= 0:
        report["errors"].append("metadata message_count is zero")
    report["ok"] = sqlite_ok and message_audit_ok and not report["errors"]
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description="Audit rosbag2 replay timestamp ordering and required topic metadata.")
    parser.add_argument("--bag", required=True, help="rosbag2 directory containing metadata.yaml")
    parser.add_argument("--required-topic", action="append", help="Topic that must exist with a nonzero message count")
    parser.add_argument("--max-time-regression-ns", type=int, default=0)
    parser.add_argument(
        "--strict-storage",
        action="store_true",
        help="Fail when message timestamp order cannot be inspected locally.",
    )
    parser.add_argument("--output", help="Optional JSON output path")
    parser.add_argument("--json", action="store_true", help="Print full JSON report")
    args = parser.parse_args(argv)

    try:
        report = build_report(args)
    except Exception as exc:  # noqa: BLE001 - CLI should report malformed bag dirs uniformly.
        print(f"rosbag2 timing audit failed: {exc}", file=sys.stderr)
        return 2

    if args.output:
        output_path = Path(args.output)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    if args.json or not report["ok"]:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        if report["metadata_only"]:
            mode = "metadata-only"
        elif report.get("sqlite_files"):
            mode = "sqlite"
        else:
            mode = "rosbag2-headers"
        print(
            "rosbag2 timing audit OK: "
            f"bag={report['bag']} mode={mode} messages={report['metadata_message_count']}"
        )
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
