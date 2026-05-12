#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later

import argparse
import heapq
from io import BytesIO
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

PROFILE_DEFAULTS = {
    "fastlivo": {
        "image_topics": ("/left_camera/image/compressed", "/left_camera/image"),
        "lidar_topics": ("/livox/lidar",),
        "imu_topics": ("/livox/imu",),
        "intrinsics": {
            "fx": 431.795259219,
            "fy": 431.550090267,
            "cx": 310.833037316,
            "cy": 266.985989326,
            "width": 640,
            "height": 512,
        },
        "lidar_frame": "livox_frame",
    },
    "fastlivo2": {
        "image_topics": ("/left_camera/image/compressed", "/left_camera/image"),
        "lidar_topics": ("/livox/lidar",),
        "imu_topics": ("/livox/imu",),
        "intrinsics": DEFAULT_INTRINSICS,
        "lidar_frame": "livox_frame",
    },
    "m2dgr": {
        "image_topics": ("/camera/color/image_raw", "/camera/color/image_raw/compressed"),
        "lidar_topics": ("/velodyne_points",),
        "imu_topics": ("/handsfree/imu",),
        "intrinsics": {
            "fx": 617.971050917033,
            "fy": 616.445131524790,
            "cx": 327.710279392468,
            "cy": 253.976983707814,
            "width": 640,
            "height": 480,
        },
        "lidar_frame": "velodyne",
    },
    "mcd": {
        "image_topics": (
            "/d435i/infra1/image_rect_raw",
            "/d435i/color/image_raw",
            "/d435i/color/image_raw/compressed",
        ),
        "lidar_topics": ("/livox/lidar", "/os_cloud_node/points", "/os1_cloud_node/points"),
        "imu_topics": ("/vn100/imu", "/vn200/imu", "/d435i/imu", "/os_cloud_node/imu", "/os1_cloud_node/imu", "/livox/imu"),
        "intrinsics": {
            "fx": 385.538839108671,
            "fy": 385.6733947077097,
            "cx": 328.2882031921083,
            "cy": 243.5295974916248,
            "width": 640,
            "height": 480,
        },
        "lidar_frame": "lidar",
    },
    "r3live": {
        "image_topics": ("/camera/image_color/compressed", "/camera/image_color"),
        "lidar_topics": ("/livox/lidar",),
        "imu_topics": ("/livox/imu",),
        "intrinsics": {
            "fx": 431.71205,
            "fy": 431.70855,
            "cx": 320.3404,
            "cy": 259.1696,
            "width": 640,
            "height": 512,
        },
        "lidar_frame": "livox_frame",
    },
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
            "ROS1 frontend_raw conversion requires Python packages rosbags, numpy, and cv2. "
            "A local setup that works on this machine is:\n"
            "  /usr/bin/python3 -m venv /home/frank/.cache/gaussian_lic_ros2/rosbags-venv\n"
            "  /home/frank/.cache/gaussian_lic_ros2/rosbags-venv/bin/pip install 'numpy<2' rosbags\n"
            "  PYTHONPATH=/home/frank/.cache/gaussian_lic_ros2/rosbags-venv/lib/python3.12/site-packages "
            "/usr/bin/python3 scripts/ros1_to_frontend_raw.py ..."
        ) from exc
    return cv2, np, AnyReader, StoragePlugin, Writer, Stores, get_typestore


class SequentialRosbag1Connection:
    def __init__(self, conn_id, topic, msgtype):
        self.id = int(conn_id)
        self.topic = topic
        self.msgtype = msgtype


class SequentialRosbag1Reader:
    """Read ROS1 bag chunks without trusting the terminal index records."""

    def __init__(self, path, typestore):
        self.path = Path(path)
        self.typestore = typestore
        self.connections = []
        self._connection_by_id = {}
        self._index_pos = 0
        self._conn_count = 0

    @staticmethod
    def _normalise_topic(topic):
        return f'{"/" * topic.startswith("/")}{"/".join(item for item in topic.split("/") if item)}'

    @staticmethod
    def _normalise_msgtype(msgtype):
        if "/msg/" in msgtype:
            return msgtype
        parts = msgtype.split("/")
        if len(parts) == 2:
            return f"{parts[0]}/msg/{parts[1]}"
        return msgtype

    @staticmethod
    def _parse_record_fields(data):
        fields = {}
        pos = 0
        while pos < len(data):
            if pos + 4 > len(data):
                raise ValueError("truncated ROS1 record field length")
            size = struct.unpack_from("<I", data, pos)[0]
            pos += 4
            if pos + size > len(data):
                raise ValueError("truncated ROS1 record field payload")
            name, sep, value = data[pos : pos + size].partition(b"=")
            pos += size
            if sep:
                fields[name.decode()] = value
        return fields

    def __enter__(self):
        self._scan_connections()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        return False

    def _open_after_bag_header(self):
        from rosbags.rosbag1.reader import Header, RecordType, read_uint32

        handle = self.path.open("rb")
        magic = handle.readline()
        if not magic.startswith(b"#ROSBAG V2.0"):
            handle.close()
            raise RuntimeError(f"not a ROS1 v2 bag: {self.path}")
        header = Header.read(handle, RecordType.BAGHEADER)
        self._index_pos = int(header.get_uint64("index_pos"))
        self._conn_count = int(header.get_uint32("conn_count"))
        padding = read_uint32(handle)
        handle.seek(padding, 1)
        return handle

    def _iter_chunk_records(self):
        from rosbags.rosbag1.reader import Header, RecordType, decompressors, read_bytes, read_uint32

        with self._open_after_bag_header() as handle:
            file_size = self.path.stat().st_size
            scan_limit = min(self._index_pos, file_size) if self._index_pos else file_size
            while handle.tell() < scan_limit:
                try:
                    header = Header.read(handle)
                    op = header.get_uint8("op")
                    size = read_uint32(handle)
                    payload = read_bytes(handle, size)
                except Exception:
                    break
                if op != RecordType.CHUNK:
                    continue
                raw = decompressors[header.get_string("compression")](payload)
                chunk = BytesIO(raw)
                while chunk.tell() < len(raw):
                    try:
                        chunk_header = Header.read(chunk)
                        chunk_op = chunk_header.get_uint8("op")
                        chunk_size = read_uint32(chunk)
                        chunk_payload = read_bytes(chunk, chunk_size)
                    except Exception:
                        break
                    yield chunk_header, chunk_op, chunk_payload

    def _scan_connections(self):
        from rosbags.rosbag1.reader import RecordType

        for header, op, payload in self._iter_chunk_records():
            if op != RecordType.CONNECTION:
                continue
            metadata = self._parse_record_fields(payload)
            conn_id = header.get_uint32("conn")
            topic = self._normalise_topic(header.get_string("topic"))
            msgtype = self._normalise_msgtype(metadata["type"].decode())
            self._connection_by_id[conn_id] = SequentialRosbag1Connection(conn_id, topic, msgtype)
            if self._conn_count and len(self._connection_by_id) >= self._conn_count:
                break
        self.connections = [self._connection_by_id[key] for key in sorted(self._connection_by_id)]
        if not self.connections:
            raise RuntimeError(f"could not recover ROS1 bag connections from unindexed bag: {self.path}")

    def messages(self, connections=(), start=None, stop=None):
        from rosbags.rosbag1.reader import RecordType

        selected = {connection.id for connection in connections} if connections else None
        for header, op, payload in self._iter_chunk_records():
            if op != RecordType.MSGDATA:
                continue
            conn_id = header.get_uint32("conn")
            if selected is not None and conn_id not in selected:
                continue
            if conn_id not in self._connection_by_id:
                continue
            timestamp = int(header.get_time("time"))
            if start is not None and timestamp < start:
                continue
            if stop is not None and timestamp >= stop:
                return
            yield self._connection_by_id[conn_id], timestamp, payload

    def deserialize(self, rawdata, msgtype):
        return self.typestore.deserialize_ros1(rawdata, msgtype)


def split_topics(value):
    if not value:
        return ()
    return tuple(item.strip() for item in str(value).split(",") if item.strip())


def split_paths(value):
    items = split_topics(value)
    if not items:
        return ()
    return tuple(Path(item).expanduser().resolve() for item in items)


def candidate_topics(primary, fallbacks):
    seen = set()
    topics = []
    for topic in split_topics(primary) + tuple(fallbacks):
        if topic and topic not in seen:
            topics.append(topic)
            seen.add(topic)
    return tuple(topics)


def choose_available_topic(available_topics, candidates, label):
    for topic in candidates:
        if topic in available_topics:
            return topic
    available = ", ".join(sorted(available_topics))
    expected = ", ".join(candidates)
    raise RuntimeError(f"missing {label} topic. Expected one of [{expected}], available [{available}]")


def apply_profile_defaults(args):
    profile = PROFILE_DEFAULTS[args.profile]
    intrinsics = profile["intrinsics"]
    if args.image_width is None:
        args.image_width = int(intrinsics["width"])
    if args.image_height is None:
        args.image_height = int(intrinsics["height"])
    for key in ("fx", "fy", "cx", "cy"):
        if getattr(args, key) is None:
            setattr(args, key, float(intrinsics[key]))
    if args.lidar_frame is None:
        args.lidar_frame = profile["lidar_frame"]
    args.profile_image_topics = tuple(profile["image_topics"])
    args.profile_lidar_topics = tuple(profile["lidar_topics"])
    args.profile_imu_topics = tuple(profile["imu_topics"])


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


def raw_image_to_bgr(cv2, np, source_msg):
    height = int(source_msg.height)
    width = int(source_msg.width)
    step = int(source_msg.step)
    encoding = str(source_msg.encoding).lower()
    data = np.asarray(source_msg.data, dtype=np.uint8).reshape(-1)
    if height <= 0 or width <= 0 or step <= 0:
        raise ValueError(f"invalid raw image shape: {width}x{height} step={step}")
    if data.size < height * step:
        raise ValueError(
            f"raw image payload is shorter than height*step: data={data.size} expected={height * step}"
        )

    rows = data[: height * step].reshape(height, step)
    if encoding == "bgr8":
        return np.ascontiguousarray(rows[:, : width * 3].reshape(height, width, 3))
    if encoding == "rgb8":
        rgb = rows[:, : width * 3].reshape(height, width, 3)
        return np.ascontiguousarray(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
    if encoding == "mono8":
        mono = rows[:, :width].reshape(height, width)
        return np.ascontiguousarray(cv2.cvtColor(mono, cv2.COLOR_GRAY2BGR))
    if encoding == "bgra8":
        bgra = rows[:, : width * 4].reshape(height, width, 4)
        return np.ascontiguousarray(cv2.cvtColor(bgra, cv2.COLOR_BGRA2BGR))
    if encoding == "rgba8":
        rgba = rows[:, : width * 4].reshape(height, width, 4)
        return np.ascontiguousarray(cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGR))
    raise ValueError(f"unsupported raw image encoding: {source_msg.encoding}")


def make_bgr_image_and_info(store, cv2, np, header, bgr, args):
    image_cls = store.types["sensor_msgs/msg/Image"]
    if args.image_width > 0 and args.image_height > 0:
        bgr = cv2.resize(bgr, (args.image_width, args.image_height), interpolation=cv2.INTER_AREA)
    height, width = bgr.shape[:2]
    contiguous = np.ascontiguousarray(bgr)
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


def make_pointcloud2(store, np, livox_msg, frame_id):
    pointfield_cls = store.types["sensor_msgs/msg/PointField"]
    pointcloud_cls = store.types["sensor_msgs/msg/PointCloud2"]
    header = make_header(store, livox_msg.header, frame_id)
    points = livox_msg.points
    point_count = int(min(len(points), int(livox_msg.point_num)))
    point_step = 20
    data = bytearray(point_count * point_step)
    for index in range(point_count):
        point = points[index]
        offset = index * point_step
        struct.pack_into(
            "<fffBBBBI",
            data,
            offset,
            float(point.x),
            float(point.y),
            float(point.z),
            int(point.reflectivity) & 0xFF,
            int(point.tag) & 0xFF,
            int(point.line) & 0xFF,
            0,
            int(getattr(point, "offset_time", 0)) & 0xFFFFFFFF,
        )
    fields = [
        pointfield_cls(name="x", offset=0, datatype=pointfield_cls.FLOAT32, count=1),
        pointfield_cls(name="y", offset=4, datatype=pointfield_cls.FLOAT32, count=1),
        pointfield_cls(name="z", offset=8, datatype=pointfield_cls.FLOAT32, count=1),
        pointfield_cls(name="intensity", offset=12, datatype=pointfield_cls.UINT8, count=1),
        pointfield_cls(name="tag", offset=13, datatype=pointfield_cls.UINT8, count=1),
        pointfield_cls(name="line", offset=14, datatype=pointfield_cls.UINT8, count=1),
        pointfield_cls(name="offset_time", offset=16, datatype=pointfield_cls.UINT32, count=1),
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


def make_pointcloud2_passthrough(store, np, source_msg, frame_id):
    pointcloud_cls = store.types["sensor_msgs/msg/PointCloud2"]
    return pointcloud_cls(
        header=make_header(store, source_msg.header, frame_id),
        height=int(source_msg.height),
        width=int(source_msg.width),
        fields=list(source_msg.fields),
        is_bigendian=bool(source_msg.is_bigendian),
        point_step=int(source_msg.point_step),
        row_step=int(source_msg.row_step),
        data=np.asarray(source_msg.data, dtype=np.uint8).reshape(-1),
        is_dense=bool(source_msg.is_dense),
    )


def make_lidar_pointcloud2(store, np, source_msg, frame_id):
    if hasattr(source_msg, "fields") and hasattr(source_msg, "point_step") and hasattr(source_msg, "row_step"):
        return make_pointcloud2_passthrough(store, np, source_msg, frame_id)
    if hasattr(source_msg, "points") and hasattr(source_msg, "point_num"):
        return make_pointcloud2(store, np, source_msg, frame_id)
    raise ValueError(f"unsupported LiDAR message type for frontend_raw conversion: {type(source_msg)!r}")


def make_image_and_info(store, cv2, np, source_msg, args):
    header = make_header(store, source_msg.header, args.camera_frame)
    if hasattr(source_msg, "height") and hasattr(source_msg, "width") and hasattr(source_msg, "encoding"):
        return make_bgr_image_and_info(store, cv2, np, header, raw_image_to_bgr(cv2, np, source_msg), args)

    encoded = np.asarray(source_msg.data, dtype=np.uint8)
    decoded = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
    if decoded is None:
        raise ValueError("OpenCV could not decode compressed image")
    return make_bgr_image_and_info(store, cv2, np, header, decoded, args)


def make_storage_plugin(StoragePlugin, name):
    if name == "mcap":
        return StoragePlugin.MCAP
    if name == "sqlite3":
        return StoragePlugin.SQLITE3
    raise ValueError(f"unsupported storage plugin: {name}")


def convert(args):
    cv2, np, AnyReader, StoragePlugin, Writer, Stores, get_typestore = load_runtime_modules()
    input_paths = split_paths(args.input)
    if not input_paths:
        raise ValueError("at least one input ROS1 bag path is required")
    for input_path in input_paths:
        if not input_path.exists():
            raise FileNotFoundError(f"input bag does not exist: {input_path}")
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

    def flush_until(writer, watermark_nsec):
        while pending and pending[0][0] <= watermark_nsec:
            timestamp, _, connection, payload = heapq.heappop(pending)
            writer.write(connection, timestamp, payload)

    def flush_all(writer):
        while pending:
            timestamp, _, connection, payload = heapq.heappop(pending)
            writer.write(connection, timestamp, payload)

    def convert_with_reader(reader_factory):
        nonlocal first_time, sequence
        first_time = None
        sequence = 0
        pending.clear()
        for key in counts:
            counts[key] = 0

        with reader_factory() as reader, Writer(
            output_path,
            version=9,
            storage_plugin=storage_plugin,
        ) as writer:
            return convert_open_reader(reader, writer)

    def convert_open_reader(reader, writer):
        nonlocal first_time

        image_conn = writer.add_connection(args.output_image_topic, "sensor_msgs/msg/Image", typestore=store)
        camera_info_conn = writer.add_connection(
            args.output_camera_info_topic, "sensor_msgs/msg/CameraInfo", typestore=store
        )
        lidar_conn = writer.add_connection(args.output_lidar_topic, "sensor_msgs/msg/PointCloud2", typestore=store)
        imu_conn = writer.add_connection(args.output_imu_topic, "sensor_msgs/msg/Imu", typestore=store)

        available_topics = {connection.topic for connection in reader.connections}
        input_image_topic = choose_available_topic(
            available_topics,
            candidate_topics(args.input_image_topic, args.profile_image_topics),
            "image",
        )
        input_lidar_topic = choose_available_topic(
            available_topics,
            candidate_topics(args.input_lidar_topic, args.profile_lidar_topics),
            "LiDAR",
        )
        input_imu_topic = choose_available_topic(
            available_topics,
            candidate_topics(args.input_imu_topic, args.profile_imu_topics),
            "IMU",
        )

        for connection, bag_time, rawdata in reader.messages():
            if first_time is None:
                first_time = int(bag_time)
            if args.max_duration_sec > 0 and int(bag_time) - first_time > int(args.max_duration_sec * 1e9):
                break
            written = counts["images"] + counts["camera_infos"] + counts["lidar"] + counts["imu"]
            if args.max_written_messages > 0 and written >= args.max_written_messages:
                break

            if connection.topic not in (input_image_topic, input_lidar_topic, input_imu_topic):
                counts["skipped"] += 1
                continue

            msg = reader.deserialize(rawdata, connection.msgtype)
            if connection.topic == input_image_topic:
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
            elif connection.topic == input_lidar_topic:
                cloud = make_lidar_pointcloud2(store, np, msg, args.lidar_frame)
                timestamp = stamp_to_nsec(cloud.header.stamp)
                queue_write(lidar_conn, timestamp, store.serialize_cdr(cloud, "sensor_msgs/msg/PointCloud2"))
                counts["lidar"] += 1
            elif connection.topic == input_imu_topic:
                timestamp = stamp_to_nsec(msg.header.stamp)
                queue_write(imu_conn, timestamp, store.serialize_cdr(msg, "sensor_msgs/msg/Imu"))
                counts["imu"] += 1
            flush_until(writer, int(bag_time) - sort_buffer_nsec)

        flush_all(writer)

        return {
            "input": [str(path) for path in input_paths],
            "output": str(output_path),
            "profile": args.profile,
            "storage": args.storage,
            "topics": {
                "image": input_image_topic,
                "lidar": input_lidar_topic,
                "imu": input_imu_topic,
            },
            "counts": counts,
        }

    try:
        return convert_with_reader(lambda: AnyReader(list(input_paths)))
    except Exception as exc:
        if len(input_paths) != 1 or "Bag index looks damaged" not in str(exc):
            raise
        if output_path.exists():
            shutil.rmtree(output_path)
        ros1_store = get_typestore(Stores.ROS1_NOETIC)
        print(
            f"frontend_raw conversion warning: falling back to sequential ROS1 chunk scan for {input_paths[0]}",
            file=sys.stderr,
        )
        return convert_with_reader(lambda: SequentialRosbag1Reader(input_paths[0], ros1_store))


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Convert a supported ROS1 LiDAR/IMU/camera bag into a ROS2 frontend_sensor_raw bag."
    )
    parser.add_argument("--input", required=True, help="Input ROS1 .bag, or comma-separated bags for split datasets.")
    parser.add_argument("--output", required=True, help="Output rosbag2 directory")
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--storage", choices=("mcap", "sqlite3"), default="mcap")
    parser.add_argument(
        "--profile",
        choices=tuple(sorted(PROFILE_DEFAULTS)),
        default="fastlivo2",
        help="Dataset profile used for topics and intrinsics. Defaults to fastlivo2.",
    )
    parser.add_argument("--input-image-topic", help="Override image topic, or comma-separated fallback topics.")
    parser.add_argument("--input-lidar-topic", help="Override LiDAR topic, or comma-separated fallback topics.")
    parser.add_argument("--input-imu-topic", help="Override IMU topic, or comma-separated fallback topics.")
    parser.add_argument("--output-image-topic", default="/camera/image")
    parser.add_argument("--output-camera-info-topic", default="/camera/camera_info")
    parser.add_argument("--output-lidar-topic", default="/livox/lidar")
    parser.add_argument("--output-imu-topic", default="/imu")
    parser.add_argument("--camera-frame", default="camera")
    parser.add_argument("--lidar-frame")
    parser.add_argument("--image-width", type=int)
    parser.add_argument("--image-height", type=int)
    parser.add_argument("--fx", type=float)
    parser.add_argument("--fy", type=float)
    parser.add_argument("--cx", type=float)
    parser.add_argument("--cy", type=float)
    parser.add_argument("--max-duration-sec", type=float, default=0.0)
    parser.add_argument("--max-written-messages", type=int, default=0)
    parser.add_argument(
        "--sort-buffer-sec",
        type=float,
        default=5.0,
        help="Stable-sort converted rosbag2 writes by header stamp within this bag-time horizon.",
    )
    args = parser.parse_args(argv)
    apply_profile_defaults(args)

    try:
        report = convert(args)
    except Exception as exc:  # noqa: BLE001 - CLI should report conversion failures uniformly.
        print(f"frontend_raw conversion failed: {exc}", file=sys.stderr)
        return 2

    print(
        "frontend_raw conversion OK: "
        f"profile={report['profile']} "
        f"output={report['output']} "
        f"image_topic={report['topics']['image']} "
        f"lidar_topic={report['topics']['lidar']} "
        f"imu_topic={report['topics']['imu']} "
        f"images={report['counts']['images']} "
        f"lidar={report['counts']['lidar']} "
        f"imu={report['counts']['imu']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
