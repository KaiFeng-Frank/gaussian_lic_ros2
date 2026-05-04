#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later

import argparse
import json
from pathlib import Path


def detect_ros2_storage_id(bag_path):
    import yaml

    metadata_path = bag_path / "metadata.yaml"
    if not metadata_path.exists():
        return ""
    metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
    return metadata.get("rosbag2_bagfile_information", {}).get("storage_identifier", "")


def open_reader(bag_path):
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


def image_to_bgr(msg):
    import cv2
    import numpy as np

    height = int(msg.height)
    width = int(msg.width)
    step = int(msg.step)
    encoding = str(msg.encoding).lower()
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)

    if encoding in {"rgb8", "bgr8"}:
        expected = height * step
        if raw.size < expected:
            raise ValueError(f"image data shorter than height*step: {raw.size} < {expected}")
        rows = raw[:expected].reshape(height, step)
        rgb_or_bgr = rows[:, : width * 3].reshape(height, width, 3)
        if encoding == "rgb8":
            return cv2.cvtColor(rgb_or_bgr, cv2.COLOR_RGB2BGR)
        return rgb_or_bgr.copy()

    if encoding in {"mono8", "8uc1"}:
        expected = height * step
        if raw.size < expected:
            raise ValueError(f"image data shorter than height*step: {raw.size} < {expected}")
        rows = raw[:expected].reshape(height, step)
        gray = rows[:, :width]
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

    raise ValueError(f"unsupported image encoding for render extraction: {msg.encoding}")


def stamp_to_float(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def main():
    parser = argparse.ArgumentParser(description="Extract ROS2 rendered Image messages to strict render-pair files.")
    parser.add_argument("--bag", required=True, help="rosbag2 directory")
    parser.add_argument("--output", required=True, help="output render directory")
    parser.add_argument("--topic", default="/gaussian_lic/rendered_image")
    parser.add_argument("--first-name", default="train_0004.jpg")
    parser.add_argument("--prefix", default="render_")
    parser.add_argument("--max-images", type=int, default=1)
    args = parser.parse_args()

    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Image
    import cv2

    bag_path = Path(args.bag).expanduser().resolve()
    output = Path(args.output).expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)

    reader = open_reader(bag_path)
    written = []
    seen = 0
    errors = []
    while reader.has_next():
        topic, serialized, _bag_time = reader.read_next()
        if topic != args.topic:
            continue
        seen += 1
        if len(written) >= args.max_images:
            continue
        msg = deserialize_message(serialized, Image)
        try:
            bgr = image_to_bgr(msg)
            name = args.first_name if not written else f"{args.prefix}{len(written):04d}.jpg"
            path = output / name
            if not cv2.imwrite(str(path), bgr):
                raise RuntimeError(f"cv2.imwrite returned false for {path}")
            written.append(
                {
                    "path": str(path),
                    "stamp": stamp_to_float(msg.header.stamp),
                    "width": int(msg.width),
                    "height": int(msg.height),
                    "encoding": str(msg.encoding),
                }
            )
        except Exception as exc:  # noqa: BLE001 - report bad frames without hiding extraction state.
            errors.append(str(exc))

    report = {
        "schema": "gaussian_lic_render_extract/v1",
        "bag": str(bag_path),
        "topic": args.topic,
        "seen": seen,
        "written": len(written),
        "images": written,
        "errors": errors,
        "ok": bool(written) and not errors,
    }
    print(json.dumps(report, indent=2, sort_keys=True))
    raise SystemExit(0 if report["ok"] else 1)


if __name__ == "__main__":
    main()
