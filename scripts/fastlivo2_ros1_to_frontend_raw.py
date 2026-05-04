#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later

import argparse
import heapq
from pathlib import Path
import shutil
import struct
import sys


DEFAULT_INTRINSICS = {
    "fx": 646.78472,
    "fy": 646.65775,
    "cx": 313.456795,
    "cy": 261.399612,
    "width": 640,
    "height": 512,
}


def load_runtime_modules():
    try:
        import cv2
        import numpy as np
        from rosbags.highlevel import AnyReader
        from rosbags.rosbag2 import StoragePlugin, Writer
        from rosbags.typesys import Stores, get_typestore
    except ImportError as exc:
        raise RuntimeError(
            "FAST-LIVO2 conversion requires Python packages rosbags, numpy, and cv2. "
            "A local setup that works on this machine is:\n"
            "  /usr/bin/python3 -m venv /home/frank/.cache/gaussian_lic_ros2/rosbags-venv\n"
            "  /home/frank/.cache/gaussian_lic_ros2/rosbags-venv/bin/pip install 'numpy<2' rosbags\n"
            "  PYTHONPATH=/home/frank/.cache/gaussian_lic_ros2/rosbags-venv/lib/python3.12/site-packages "
            "/usr/bin/python3 scripts/fastlivo2_ros1_to_frontend_raw.py ..."
        ) from exc
    return cv2, np, AnyReader, StoragePlugin, Writer, Stores, get_typestore


def stamp_to_nsec(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def make_header(store, source_header, frame_id=None):
    time_cls = store.types["builtin_interfaces/msg/Time"]
    header_cls = store.types["std_msgs/msg/Header"]
    stamp = time_cls(sec=int(source_header.stamp.sec), nanosec=int(source_header.stamp.nanosec))
    return header_cls(stamp=stamp, frame_id=frame_id if frame_id is not None else source_header.frame_id)


def make_camera_info(store, np, header, width, height, fx, fy, cx, cy):
    camera_info_cls = store.types["sensor_msgs/msg/CameraInfo"]
    roi_cls = store.types["sensor_msgs/msg/RegionOfInterest"]
    return camera_info_cls(
        header=header,
        height=int(height),
        width=int(width),
        distortion_model="plumb_bob",
        d=np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float64),
        k=np.array([fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0], dtype=np.float64),
        r=np.array([1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0], dtype=np.float64),
        p=np.array([fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0], dtype=np.float64),
        binning_x=0,
        binning_y=0,
        roi=roi_cls(x_offset=0, y_offset=0, height=0, width=0, do_rectify=False),
    )


def make_pointcloud2(store, np, livox_msg, frame_id):
    pointfield_cls = store.types["sensor_msgs/msg/PointField"]
    pointcloud_cls = store.types["sensor_msgs/msg/PointCloud2"]
    header = make_header(store, livox_msg.header, frame_id)
    points = livox_msg.points
    point_count = int(min(len(points), int(livox_msg.point_num)))
    point_step = 16
    data = bytearray(point_count * point_step)
    for index in range(point_count):
        point = points[index]
        offset = index * point_step
        struct.pack_into(
            "<fffBBBB",
            data,
            offset,
            float(point.x),
            float(point.y),
            float(point.z),
            int(point.reflectivity) & 0xFF,
            int(point.tag) & 0xFF,
            int(point.line) & 0xFF,
            0,
        )
    fields = [
        pointfield_cls(name="x", offset=0, datatype=pointfield_cls.FLOAT32, count=1),
        pointfield_cls(name="y", offset=4, datatype=pointfield_cls.FLOAT32, count=1),
        pointfield_cls(name="z", offset=8, datatype=pointfield_cls.FLOAT32, count=1),
        pointfield_cls(name="intensity", offset=12, datatype=pointfield_cls.UINT8, count=1),
        pointfield_cls(name="tag", offset=13, datatype=pointfield_cls.UINT8, count=1),
        pointfield_cls(name="line", offset=14, datatype=pointfield_cls.UINT8, count=1),
    ]
    return pointcloud_cls(
        header=header,
        height=1,
        width=point_count,
        fields=fields,
        is_bigendian=False,
        point_step=point_step,
        row_step=point_count * point_step,
        data=np.frombuffer(bytes(data), dtype=np.uint8),
        is_dense=False,
    )


def make_image_and_info(store, cv2, np, compressed_msg, args):
    image_cls = store.types["sensor_msgs/msg/Image"]
    header = make_header(store, compressed_msg.header, args.camera_frame)
    encoded = np.asarray(compressed_msg.data, dtype=np.uint8)
    decoded = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
    if decoded is None:
        raise ValueError("OpenCV could not decode compressed image")
    if args.image_width > 0 and args.image_height > 0:
        decoded = cv2.resize(decoded, (args.image_width, args.image_height), interpolation=cv2.INTER_AREA)
    height, width = decoded.shape[:2]
    contiguous = np.ascontiguousarray(decoded)
    image = image_cls(
        header=header,
        height=int(height),
        width=int(width),
        encoding="bgr8",
        is_bigendian=0,
        step=int(width) * 3,
        data=contiguous.reshape(-1).astype(np.uint8, copy=False),
    )
    camera_info = make_camera_info(
        store,
        np,
        header,
        width,
        height,
        args.fx,
        args.fy,
        args.cx,
        args.cy,
    )
    return image, camera_info


def make_storage_plugin(StoragePlugin, name):
    if name == "mcap":
        return StoragePlugin.MCAP
    if name == "sqlite3":
        return StoragePlugin.SQLITE3
    raise ValueError(f"unsupported storage plugin: {name}")


def convert(args):
    cv2, np, AnyReader, StoragePlugin, Writer, Stores, get_typestore = load_runtime_modules()
    input_path = Path(args.input).expanduser().resolve()
    output_path = Path(args.output).expanduser().resolve()
    if output_path.exists():
        if not args.overwrite:
            raise FileExistsError(f"output already exists: {output_path}")
        shutil.rmtree(output_path)

    store = get_typestore(Stores.ROS2_JAZZY)
    storage_plugin = make_storage_plugin(StoragePlugin, args.storage)
    counts = {
        "images": 0,
        "camera_infos": 0,
        "lidar": 0,
        "imu": 0,
        "skipped": 0,
    }
    first_time = None
    pending = []
    sequence = 0
    sort_buffer_nsec = int(max(args.sort_buffer_sec, 0.0) * 1e9)

    def queue_write(connection, timestamp, payload):
        nonlocal sequence
        heapq.heappush(pending, (int(timestamp), sequence, connection, payload))
        sequence += 1

    def flush_until(watermark_nsec):
        while pending and pending[0][0] <= watermark_nsec:
            timestamp, _, connection, payload = heapq.heappop(pending)
            writer.write(connection, timestamp, payload)

    def flush_all():
        while pending:
            timestamp, _, connection, payload = heapq.heappop(pending)
            writer.write(connection, timestamp, payload)

    with AnyReader([input_path]) as reader, Writer(
        output_path,
        version=9,
        storage_plugin=storage_plugin,
    ) as writer:
        image_conn = writer.add_connection(args.output_image_topic, "sensor_msgs/msg/Image", typestore=store)
        camera_info_conn = writer.add_connection(
            args.output_camera_info_topic, "sensor_msgs/msg/CameraInfo", typestore=store
        )
        lidar_conn = writer.add_connection(args.output_lidar_topic, "sensor_msgs/msg/PointCloud2", typestore=store)
        imu_conn = writer.add_connection(args.output_imu_topic, "sensor_msgs/msg/Imu", typestore=store)

        for connection, bag_time, rawdata in reader.messages():
            if first_time is None:
                first_time = int(bag_time)
            if args.max_duration_sec > 0 and int(bag_time) - first_time > int(args.max_duration_sec * 1e9):
                break
            written = counts["images"] + counts["camera_infos"] + counts["lidar"] + counts["imu"]
            if args.max_written_messages > 0 and written >= args.max_written_messages:
                break

            if connection.topic not in (args.input_image_topic, args.input_lidar_topic, args.input_imu_topic):
                counts["skipped"] += 1
                continue

            msg = reader.deserialize(rawdata, connection.msgtype)
            if connection.topic == args.input_image_topic:
                image, camera_info = make_image_and_info(store, cv2, np, msg, args)
                timestamp = stamp_to_nsec(image.header.stamp)
                queue_write(image_conn, timestamp, store.serialize_cdr(image, "sensor_msgs/msg/Image"))
                queue_write(
                    camera_info_conn,
                    timestamp,
                    store.serialize_cdr(camera_info, "sensor_msgs/msg/CameraInfo"),
                )
                counts["images"] += 1
                counts["camera_infos"] += 1
            elif connection.topic == args.input_lidar_topic:
                cloud = make_pointcloud2(store, np, msg, args.lidar_frame)
                timestamp = stamp_to_nsec(cloud.header.stamp)
                queue_write(lidar_conn, timestamp, store.serialize_cdr(cloud, "sensor_msgs/msg/PointCloud2"))
                counts["lidar"] += 1
            elif connection.topic == args.input_imu_topic:
                timestamp = stamp_to_nsec(msg.header.stamp)
                queue_write(imu_conn, timestamp, store.serialize_cdr(msg, "sensor_msgs/msg/Imu"))
                counts["imu"] += 1
            flush_until(int(bag_time) - sort_buffer_nsec)

        flush_all()

    return {
        "input": str(input_path),
        "output": str(output_path),
        "storage": args.storage,
        "counts": counts,
    }


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Convert a FAST-LIVO2 ROS1 bag into a ROS2 frontend_sensor_raw bag."
    )
    parser.add_argument("--input", required=True, help="Input FAST-LIVO2 ROS1 .bag")
    parser.add_argument("--output", required=True, help="Output rosbag2 directory")
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--storage", choices=("mcap", "sqlite3"), default="mcap")
    parser.add_argument("--input-image-topic", default="/left_camera/image/compressed")
    parser.add_argument("--input-lidar-topic", default="/livox/lidar")
    parser.add_argument("--input-imu-topic", default="/livox/imu")
    parser.add_argument("--output-image-topic", default="/camera/image")
    parser.add_argument("--output-camera-info-topic", default="/camera/camera_info")
    parser.add_argument("--output-lidar-topic", default="/livox/lidar")
    parser.add_argument("--output-imu-topic", default="/imu")
    parser.add_argument("--camera-frame", default="camera")
    parser.add_argument("--lidar-frame", default="livox_frame")
    parser.add_argument("--image-width", type=int, default=DEFAULT_INTRINSICS["width"])
    parser.add_argument("--image-height", type=int, default=DEFAULT_INTRINSICS["height"])
    parser.add_argument("--fx", type=float, default=DEFAULT_INTRINSICS["fx"])
    parser.add_argument("--fy", type=float, default=DEFAULT_INTRINSICS["fy"])
    parser.add_argument("--cx", type=float, default=DEFAULT_INTRINSICS["cx"])
    parser.add_argument("--cy", type=float, default=DEFAULT_INTRINSICS["cy"])
    parser.add_argument("--max-duration-sec", type=float, default=0.0)
    parser.add_argument("--max-written-messages", type=int, default=0)
    parser.add_argument(
        "--sort-buffer-sec",
        type=float,
        default=5.0,
        help="Stable-sort converted rosbag2 writes by header stamp within this bag-time horizon.",
    )
    args = parser.parse_args(argv)

    try:
        report = convert(args)
    except Exception as exc:  # noqa: BLE001 - CLI should report conversion failures uniformly.
        print(f"FAST-LIVO2 conversion failed: {exc}", file=sys.stderr)
        return 2

    print(
        "FAST-LIVO2 conversion OK: "
        f"output={report['output']} "
        f"images={report['counts']['images']} "
        f"lidar={report['counts']['lidar']} "
        f"imu={report['counts']['imu']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
