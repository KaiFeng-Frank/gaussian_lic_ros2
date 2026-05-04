#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later

import argparse
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

POINT_FIELD_FLOAT32 = 7
FASTLIVO2_CAMERA_LIDAR_R = (
    0.006101930, -0.999863000, -0.015417200,
    -0.006154490, 0.015379600, -0.999863000,
    0.999962000, 0.006195980, -0.006059800,
)
FASTLIVO2_CAMERA_LIDAR_T = (0.019438453, 0.104689079, -0.025195139)


def load_runtime_modules():
    try:
        import numpy as np
        from rosbags.highlevel import AnyReader
        from rosbags.rosbag1 import Writer
        from rosbags.typesys import Stores, get_typestore
    except ImportError as exc:
        raise RuntimeError(
            "ROS2 frontend raw to ROS1 mapper conversion requires rosbags and numpy. "
            "A local setup that works on this machine is:\n"
            "  PYTHONPATH=/home/frank/.cache/gaussian_lic_ros2/rosbags-venv/lib/python3.12/site-packages "
            "/usr/bin/python3 scripts/frontend_raw_to_ros1_mapper_contract.py ..."
        ) from exc
    return np, AnyReader, Writer, Stores, get_typestore


def stamp_to_nsec(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def make_ros1_header(store, source_header, frame_id=None, seq=0):
    time_cls = store.types["builtin_interfaces/msg/Time"]
    header_cls = store.types["std_msgs/msg/Header"]
    stamp = time_cls(sec=int(source_header.stamp.sec), nanosec=int(source_header.stamp.nanosec))
    return header_cls(
        seq=int(seq),
        stamp=stamp,
        frame_id=frame_id if frame_id is not None else source_header.frame_id,
    )


def make_identity_pose(store, source_header, frame_id, child_frame, seq):
    point_cls = store.types["geometry_msgs/msg/Point"]
    quat_cls = store.types["geometry_msgs/msg/Quaternion"]
    pose_cls = store.types["geometry_msgs/msg/Pose"]
    pose_stamped_cls = store.types["geometry_msgs/msg/PoseStamped"]
    header = make_ros1_header(store, source_header, frame_id, seq)
    pose = pose_cls(
        position=point_cls(x=0.0, y=0.0, z=0.0),
        orientation=quat_cls(x=0.0, y=0.0, z=0.0, w=1.0),
    )
    pose_msg = pose_stamped_cls(header=header, pose=pose)
    pose_msg.header.frame_id = frame_id
    _ = child_frame
    return pose_msg


def convert_point_fields(store, source_fields):
    pointfield_cls = store.types["sensor_msgs/msg/PointField"]
    return [
        pointfield_cls(
            name=field.name,
            offset=int(field.offset),
            datatype=int(field.datatype),
            count=int(field.count),
        )
        for field in source_fields
    ]


def make_ros1_image(store, np, source_msg, frame_id, seq):
    image_cls = store.types["sensor_msgs/msg/Image"]
    return image_cls(
        header=make_ros1_header(store, source_msg.header, frame_id, seq),
        height=int(source_msg.height),
        width=int(source_msg.width),
        encoding=source_msg.encoding,
        is_bigendian=int(source_msg.is_bigendian),
        step=int(source_msg.step),
        data=np.asarray(source_msg.data, dtype=np.uint8).reshape(-1),
    )


def pointcloud_point_bytes(np, cloud_msg):
    data = np.asarray(cloud_msg.data, dtype=np.uint8).reshape(-1)
    point_step = int(cloud_msg.point_step)
    row_step = int(cloud_msg.row_step)
    width = int(cloud_msg.width)
    height = int(cloud_msg.height)
    rows = []
    for row in range(height):
        start = row * row_step
        end = start + width * point_step
        if end > data.size:
            break
        rows.append(data[start:end].reshape(width, point_step))
    if not rows:
        return np.empty((0, point_step), dtype=np.uint8)
    return np.ascontiguousarray(np.vstack(rows))


def float32_column(np, point_bytes, offset, endian):
    dtype = np.dtype(f"{endian}f4")
    return np.ascontiguousarray(point_bytes[:, offset : offset + 4]).view(dtype).reshape(-1)


def write_float32_column(np, point_bytes, offset, values, endian):
    dtype = np.dtype(f"{endian}f4")
    packed = np.asarray(values, dtype=dtype).reshape(-1, 1).view(np.uint8).reshape(-1, 4)
    point_bytes[:, offset : offset + 4] = packed


def transform_xyz(np, x, y, z, args):
    if args.pointcloud_transform_profile == "identity":
        return x, y, z
    if args.pointcloud_transform_profile != "fastlivo2":
        raise ValueError(f"unsupported pointcloud transform profile: {args.pointcloud_transform_profile}")

    r = np.asarray(FASTLIVO2_CAMERA_LIDAR_R, dtype=np.float64).reshape(3, 3)
    t = np.asarray(FASTLIVO2_CAMERA_LIDAR_T, dtype=np.float64).reshape(3, 1)
    xyz = np.vstack([
        np.asarray(x, dtype=np.float64),
        np.asarray(y, dtype=np.float64),
        np.asarray(z, dtype=np.float64),
    ])
    transformed = r @ xyz + t
    return transformed[0], transformed[1], transformed[2]


def filtered_pointcloud_payload(np, source_msg, args):
    fields = field_map(source_msg)
    if not {"x", "y", "z"}.issubset(fields):
        raise ValueError("PointCloud2 is missing x/y/z fields")
    if any(int(fields[name].datatype) != POINT_FIELD_FLOAT32 for name in ("x", "y", "z")):
        raise ValueError("PointCloud2 x/y/z fields must be FLOAT32")

    point_bytes = pointcloud_point_bytes(np, source_msg)
    if point_bytes.size == 0:
        return point_bytes.reshape(-1), 0, 0

    endian = ">" if bool(source_msg.is_bigendian) else "<"
    x = float32_column(np, point_bytes, int(fields["x"].offset), endian)
    y = float32_column(np, point_bytes, int(fields["y"].offset), endian)
    z = float32_column(np, point_bytes, int(fields["z"].offset), endian)
    x, y, z = transform_xyz(np, x, y, z, args)
    if args.pointcloud_transform_profile != "identity":
        write_float32_column(np, point_bytes, int(fields["x"].offset), x, endian)
        write_float32_column(np, point_bytes, int(fields["y"].offset), y, endian)
        write_float32_column(np, point_bytes, int(fields["z"].offset), z, endian)
    mask = np.isfinite(x) & np.isfinite(y) & np.isfinite(z) & (z > float(args.min_z))
    if args.max_z > 0.0:
        mask &= z <= float(args.max_z)
    filtered = np.ascontiguousarray(point_bytes[mask]).reshape(-1)
    return filtered, int(point_bytes.shape[0]), int(mask.sum())


def make_ros1_pointcloud(store, np, source_msg, frame_id, seq, args):
    filtered_data, raw_points, kept_points = filtered_pointcloud_payload(np, source_msg, args)
    pointcloud_cls = store.types["sensor_msgs/msg/PointCloud2"]
    cloud = pointcloud_cls(
        header=make_ros1_header(store, source_msg.header, frame_id, seq),
        height=1,
        width=kept_points,
        fields=convert_point_fields(store, source_msg.fields),
        is_bigendian=bool(source_msg.is_bigendian),
        point_step=int(source_msg.point_step),
        row_step=kept_points * int(source_msg.point_step),
        data=filtered_data,
        is_dense=True,
    )
    return cloud, raw_points, kept_points


def field_map(cloud_msg):
    return {field.name: field for field in cloud_msg.fields}


def unpack_float32(data, offset, endian):
    return struct.unpack_from(endian + "f", data, offset)[0]


def iter_xyz(cloud_msg, max_points):
    fields = field_map(cloud_msg)
    if not {"x", "y", "z"}.issubset(fields):
        return
    if any(int(fields[name].datatype) != POINT_FIELD_FLOAT32 for name in ("x", "y", "z")):
        return

    data = bytes(cloud_msg.data)
    point_step = int(cloud_msg.point_step)
    row_step = int(cloud_msg.row_step)
    endian = ">" if bool(cloud_msg.is_bigendian) else "<"
    count = 0
    for row in range(int(cloud_msg.height)):
        row_offset = row * row_step
        for column in range(int(cloud_msg.width)):
            if 0 < max_points <= count:
                return
            base = row_offset + column * point_step
            if base + point_step > len(data):
                return
            x = unpack_float32(data, base + int(fields["x"].offset), endian)
            y = unpack_float32(data, base + int(fields["y"].offset), endian)
            z = unpack_float32(data, base + int(fields["z"].offset), endian)
            yield x, y, z
            count += 1


def make_projected_depth(store, np, cloud_msg, args, seq):
    image_cls = store.types["sensor_msgs/msg/Image"]
    depth = np.zeros((args.height, args.width), dtype=np.float32)
    for x, y, z in iter_xyz(cloud_msg, args.max_depth_points):
        if not (z > 0.0):
            continue
        u = int(round(args.fx * x / z + args.cx))
        v = int(round(args.fy * y / z + args.cy))
        if 0 <= u < args.width and 0 <= v < args.height:
            current = depth[v, u]
            if current == 0.0 or z < current:
                depth[v, u] = z
    return image_cls(
        header=make_ros1_header(store, cloud_msg.header, args.camera_frame, seq),
        height=int(args.height),
        width=int(args.width),
        encoding="32FC1",
        is_bigendian=0,
        step=int(args.width) * 4,
        data=np.ascontiguousarray(depth).view(np.uint8).reshape(-1),
    )


def update_intrinsics_from_camera_info(args, camera_info):
    args.width = int(camera_info.width)
    args.height = int(camera_info.height)
    if len(camera_info.k) >= 6:
        args.fx = float(camera_info.k[0])
        args.fy = float(camera_info.k[4])
        args.cx = float(camera_info.k[2])
        args.cy = float(camera_info.k[5])


def convert(args):
    np, AnyReader, Writer, Stores, get_typestore = load_runtime_modules()
    input_path = Path(args.input).expanduser().resolve()
    output_path = Path(args.output).expanduser().resolve()
    if output_path.exists():
        if not args.overwrite:
            raise FileExistsError(f"output already exists: {output_path}")
        output_path.unlink() if output_path.is_file() else shutil.rmtree(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    ros2_store = get_typestore(Stores.ROS2_JAZZY)
    ros1_store = get_typestore(Stores.ROS1_NOETIC)
    first_time = None
    seq = 0
    counts = {
        "images": 0,
        "points": 0,
        "poses": 0,
        "depths": 0,
        "camera_infos": 0,
        "skipped": 0,
        "dropped_clouds": 0,
        "raw_point_samples": 0,
        "written_point_samples": 0,
    }

    with AnyReader([input_path], default_typestore=ros2_store) as reader, Writer(output_path) as writer:
        image_conn = writer.add_connection(args.output_image_topic, "sensor_msgs/msg/Image", typestore=ros1_store)
        point_conn = writer.add_connection(
            args.output_pointcloud_topic,
            "sensor_msgs/msg/PointCloud2",
            typestore=ros1_store,
        )
        pose_conn = writer.add_connection(
            args.output_pose_topic,
            "geometry_msgs/msg/PoseStamped",
            typestore=ros1_store,
        )
        depth_conn = writer.add_connection(args.output_depth_topic, "sensor_msgs/msg/Image", typestore=ros1_store)

        for connection, bag_time, rawdata in reader.messages():
            if first_time is None:
                first_time = int(bag_time)
            if args.max_duration_sec > 0 and int(bag_time) - first_time > int(args.max_duration_sec * 1e9):
                break

            if connection.topic == args.input_camera_info_topic:
                update_intrinsics_from_camera_info(args, reader.deserialize(rawdata, connection.msgtype))
                counts["camera_infos"] += 1
                continue
            if connection.topic not in (args.input_image_topic, args.input_pointcloud_topic):
                counts["skipped"] += 1
                continue

            msg = reader.deserialize(rawdata, connection.msgtype)
            seq += 1
            if connection.topic == args.input_image_topic:
                image = make_ros1_image(ros1_store, np, msg, args.camera_frame, seq)
                timestamp = stamp_to_nsec(image.header.stamp)
                writer.write(image_conn, timestamp, ros1_store.serialize_ros1(image, "sensor_msgs/msg/Image"))
                counts["images"] += 1
            elif connection.topic == args.input_pointcloud_topic:
                cloud, raw_points, kept_points = make_ros1_pointcloud(ros1_store, np, msg, args.lidar_frame, seq, args)
                counts["raw_point_samples"] += raw_points
                counts["written_point_samples"] += kept_points
                if kept_points < args.min_points_per_cloud:
                    counts["dropped_clouds"] += 1
                    continue
                pose = make_identity_pose(ros1_store, msg.header, args.world_frame, args.camera_frame, seq)
                depth = make_projected_depth(ros1_store, np, cloud, args, seq)
                timestamp = stamp_to_nsec(cloud.header.stamp)
                writer.write(point_conn, timestamp, ros1_store.serialize_ros1(cloud, "sensor_msgs/msg/PointCloud2"))
                writer.write(pose_conn, timestamp, ros1_store.serialize_ros1(pose, "geometry_msgs/msg/PoseStamped"))
                writer.write(depth_conn, timestamp, ros1_store.serialize_ros1(depth, "sensor_msgs/msg/Image"))
                counts["points"] += 1
                counts["poses"] += 1
                counts["depths"] += 1

            written_messages = counts["images"] + counts["points"] + counts["poses"] + counts["depths"]
            if args.max_written_messages > 0 and written_messages >= args.max_written_messages:
                break

    return {
        "input": str(input_path),
        "output": str(output_path),
        "counts": counts,
        "width": args.width,
        "height": args.height,
        "fx": args.fx,
        "fy": args.fy,
        "cx": args.cx,
        "cy": args.cy,
        "pointcloud_transform_profile": args.pointcloud_transform_profile,
    }


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Convert a ROS2 frontend raw bag into a ROS1 Gaussian-LIC mapper-contract bag."
    )
    parser.add_argument("--input", required=True, help="Input rosbag2 frontend raw directory")
    parser.add_argument("--output", required=True, help="Output ROS1 .bag")
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--input-image-topic", default="/camera/image")
    parser.add_argument("--input-camera-info-topic", default="/camera/camera_info")
    parser.add_argument("--input-pointcloud-topic", default="/livox/lidar")
    parser.add_argument("--output-image-topic", default="/image_for_gs")
    parser.add_argument("--output-pointcloud-topic", default="/points_for_gs")
    parser.add_argument("--output-pose-topic", default="/pose_for_gs")
    parser.add_argument("--output-depth-topic", default="/depth_for_gs")
    parser.add_argument("--world-frame", default="map")
    parser.add_argument("--camera-frame", default="camera")
    parser.add_argument("--lidar-frame", default="camera")
    parser.add_argument("--width", type=int, default=DEFAULT_INTRINSICS["width"])
    parser.add_argument("--height", type=int, default=DEFAULT_INTRINSICS["height"])
    parser.add_argument("--fx", type=float, default=DEFAULT_INTRINSICS["fx"])
    parser.add_argument("--fy", type=float, default=DEFAULT_INTRINSICS["fy"])
    parser.add_argument("--cx", type=float, default=DEFAULT_INTRINSICS["cx"])
    parser.add_argument("--cy", type=float, default=DEFAULT_INTRINSICS["cy"])
    parser.add_argument("--min-z", type=float, default=1e-3, help="Drop points with z <= this value before ROS1 export")
    parser.add_argument("--max-z", type=float, default=0.0, help="Drop points with z > this value; 0 disables the cap")
    parser.add_argument(
        "--pointcloud-transform-profile",
        choices=("identity", "fastlivo2"),
        default="identity",
        help="Optional static pointcloud transform before z filtering/depth projection.",
    )
    parser.add_argument("--min-points-per-cloud", type=int, default=1)
    parser.add_argument("--max-depth-points", type=int, default=200000)
    parser.add_argument("--max-duration-sec", type=float, default=0.0)
    parser.add_argument("--max-written-messages", type=int, default=0)
    args = parser.parse_args(argv)

    try:
        report = convert(args)
    except Exception as exc:  # noqa: BLE001 - CLI should report conversion failures uniformly.
        print(f"frontend raw to ROS1 mapper conversion failed: {exc}", file=sys.stderr)
        return 2

    print(
        "frontend raw to ROS1 mapper conversion OK: "
        f"output={report['output']} "
        f"images={report['counts']['images']} "
        f"points={report['counts']['points']} "
        f"poses={report['counts']['poses']} "
        f"depths={report['counts']['depths']} "
        f"raw_points={report['counts']['raw_point_samples']} "
        f"written_points={report['counts']['written_point_samples']} "
        f"dropped_clouds={report['counts']['dropped_clouds']} "
        f"pointcloud_transform={report['pointcloud_transform_profile']} "
        f"size={report['width']}x{report['height']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
