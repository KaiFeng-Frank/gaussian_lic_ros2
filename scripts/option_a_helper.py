#!/usr/bin/env python3.12
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Option-A in-loop-render helper. Three subcommands, each grounded in verified
# source facts (see option_a_outerloop.sh header for the citations):
#
#   extract-stamps  <bag_dir> <image_topic> <out_stamps_json>
#       Read the OBSERVED image header stamps from the source bag and write the
#       JSON array of {"ns": <int>} that gaussian_map_renderer.parse_stamps_ns()
#       consumes (renderer expects literally `"ns":<int>` tokens).
#
#   anchor-tum      <in_tum> <out_tum> [--mode camera|body]
#       Pre-transform a body(IMU)-world TUM trajectory so frame-0 aligns with the
#       map's camera-0 anchor. The renderer composes the CAMERA world pose from the
#       body pose via the HARDCODED cam->IMU extrinsic
#       (gaussian_map_renderer.cpp:380-381):
#           r_wc = q_body * q_cam_to_imu ;  t_wc = t_body + q_body * p_cam_to_imu
#       The on-disk map is anchored at the producing run's CAMERA-0. So to make the
#       rendered frame-0 == map cam0 we must drive the renderer's CAMERA-0 pose to
#       identity. Because the renderer RE-APPLIES the extrinsic, we:
#         camera mode (default, primary hypothesis):
#           T_wc_i  = T_body_i @ T_cam_to_imu          (camera world pose)
#           T_wc_i' = inv(T_wc_0) @ T_wc_i             (anchor cameras: cam0 -> I)
#           T_body_i' = T_wc_i' @ inv(T_cam_to_imu)    (back to body for the renderer)
#         body mode (variant / open risk):
#           T_body_i' = inv(T_body_0) @ T_body_i       (anchor bodies: leaves cam0 at
#                                                       the fixed extrinsic offset)
#       Output is TUM (qw last), the format both parse_tum() and the node's
#       CT_SEED_TUM reader expect.
#
#   mux-feedback    <render_dir> <stamps_json> <out_bag_dir> [--storage mcap]
#       Build a RenderedFeedback bag. For each rendered PNG <ns>.png in
#       <render_dir>, publish one gaussian_lic_msgs/msg/RenderedFeedback with
#       .image = the rendered frame (sensor_msgs/Image, encoding rgb8) and
#       .observed_stamp = that ns. The CT node consumes ONLY .image and
#       .observed_stamp (continuous_time_node.cpp:1324-1326:
#           if (decode_image_gray(m.image, rf)) {
#             rf.stamp_ns = stamp_to_nanoseconds(m.observed_stamp);
#       We populate the other fields to mirror the recorded /tmp/cbd_feedback_full122
#       layout (header.stamp == observed_stamp, frame_id 'camera', render_mode
#       'rasterizer', incrementing frame_index) for fidelity, but they are not
#       load-bearing for ingest.
#
# MUST run under /usr/bin/python3.12 with:
#   source /opt/ros/jazzy/setup.bash && source <ws>/install/setup.bash
# (default python3.13 is ABI-broken against the rosbag2/rclpy build here.)

import argparse
import json
import os
import sys


def cmd_extract_stamps(args):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    # storage_id left '' lets rosbag2 auto-detect (sqlite3 source bag, mcap, ...).
    reader.open(
        rosbag2_py.StorageOptions(uri=args.bag_dir, storage_id=""),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )
    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if args.image_topic not in topic_types:
        raise SystemExit(
            f"image topic {args.image_topic!r} not in bag; have {list(topic_types)}"
        )
    msgtype = get_message(topic_types[args.image_topic])

    stamps = []
    while reader.has_next():
        topic, data, _bag_ts = reader.read_next()
        if topic != args.image_topic:
            continue
        msg = deserialize_message(data, msgtype)
        ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        stamps.append(ns)
    if not stamps:
        raise SystemExit(f"no image messages on {args.image_topic} in {args.bag_dir}")
    stamps.sort()
    with open(args.out_json, "w", encoding="utf-8") as f:
        f.write("[")
        f.write(",".join('{"ns":%d}' % ns for ns in stamps))
        f.write("]")
    print(
        f"[extract-stamps] wrote {len(stamps)} observed stamps to {args.out_json} "
        f"span=[{stamps[0]},{stamps[-1]}]"
    )


def _load_tum(path):
    rows = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) != 8:
                continue
            rows.append([float(x) for x in parts])
    if not rows:
        raise SystemExit(f"no TUM poses in {path}")
    rows.sort(key=lambda r: r[0])
    return rows


def _tum_to_T(row):
    # row = [t, tx, ty, tz, qx, qy, qz, qw]
    import numpy as np

    t, tx, ty, tz, qx, qy, qz, qw = row
    n = (qx * qx + qy * qy + qz * qz + qw * qw) ** 0.5
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    R = np.array(
        [
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
        ]
    )
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [tx, ty, tz]
    return t, T


def _T_to_tum(t, T):
    import numpy as np

    R = T[:3, :3]
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0.0:
        s = (tr + 1.0) ** 0.5 * 2.0
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = (1.0 + R[0, 0] - R[1, 1] - R[2, 2]) ** 0.5 * 2.0
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = (1.0 + R[1, 1] - R[0, 0] - R[2, 2]) ** 0.5 * 2.0
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = (1.0 + R[2, 2] - R[0, 0] - R[1, 1]) ** 0.5 * 2.0
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s
    n = (qx * qx + qy * qy + qz * qz + qw * qw) ** 0.5
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    p = T[:3, 3]
    return f"{t:.9f} {p[0]:.9f} {p[1]:.9f} {p[2]:.9f} {qx:.9f} {qy:.9f} {qz:.9f} {qw:.9f}"


# cam->IMU(body) extrinsic, VERBATIM from gaussian_map_renderer.cpp:53-59 (xyzw + xyz).
_CAM_IMU_Q_XYZW = (-0.4991948721, 0.5038197882, -0.4930665852, 0.5038406923)
_CAM_IMU_T = (0.0673699, 0.0412418, 0.0764217)


def _cam_to_imu_T():
    import numpy as np

    qx, qy, qz, qw = _CAM_IMU_Q_XYZW
    return _tum_to_T([0.0, _CAM_IMU_T[0], _CAM_IMU_T[1], _CAM_IMU_T[2], qx, qy, qz, qw])[1]


def cmd_anchor_tum(args):
    import numpy as np

    rows = _load_tum(args.in_tum)
    Tci = _cam_to_imu_T()  # T_cam_to_imu (camera->body)
    Tci_inv = np.linalg.inv(Tci)

    _, T0_body = _tum_to_T(rows[0])
    if args.mode == "camera":
        T0_wc = T0_body @ Tci  # camera-0 world pose
        T0_wc_inv = np.linalg.inv(T0_wc)
    elif args.mode == "body":
        T0_body_inv = np.linalg.inv(T0_body)
    else:
        raise SystemExit(f"unknown mode {args.mode!r}")

    out_lines = []
    for row in rows:
        t, T_body = _tum_to_T(row)
        if args.mode == "camera":
            T_wc = T_body @ Tci
            T_wc_rel = T0_wc_inv @ T_wc  # cam0 -> identity (== map cam0)
            T_body_out = T_wc_rel @ Tci_inv  # back to body for the renderer
        else:  # body
            T_body_out = T0_body_inv @ T_body
        out_lines.append(_T_to_tum(t, T_body_out))

    with open(args.out_tum, "w", encoding="utf-8") as f:
        f.write("# stamp_s tx ty tz qx qy qz qw\n")
        f.write("\n".join(out_lines) + "\n")
    print(
        f"[anchor-tum] mode={args.mode} wrote {len(out_lines)} poses to {args.out_tum} "
        f"(frame-0 body identity check: first line below)"
    )
    print("[anchor-tum] " + out_lines[0])


def cmd_mux_feedback(args):
    import cv2
    import rosbag2_py
    from rclpy.serialization import serialize_message
    from gaussian_lic_msgs.msg import RenderedFeedback
    from sensor_msgs.msg import Image
    from builtin_interfaces.msg import Time

    with open(args.stamps_json, "r", encoding="utf-8") as f:
        raw = json.load(f)
    # Accept both the renderer's OUTPUT stamps.json (flat ints, authoritative =
    # the stamps actually rendered) and the object format [{"ns": N}, ...].
    stamps = [int(o["ns"]) if isinstance(o, dict) else int(o) for o in raw]
    stamps.sort()

    if os.path.exists(args.out_bag_dir):
        raise SystemExit(
            f"out_bag_dir {args.out_bag_dir} already exists; rosbag2 will not overwrite. "
            "Remove it first (the outer-loop script does this)."
        )

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=args.out_bag_dir, storage_id=args.storage),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )
    topic = "/gaussian_lic/rendered_feedback"
    # TopicMetadata(id, name, type, serialization_format, ...) per pybind sig.
    writer.create_topic(
        rosbag2_py.TopicMetadata(
            0, topic, "gaussian_lic_msgs/msg/RenderedFeedback", "cdr"
        )
    )

    written = 0
    missing = 0
    for idx, ns in enumerate(stamps):
        png = os.path.join(args.render_dir, f"{ns}.png")
        if not os.path.exists(png):
            missing += 1
            continue
        bgr = cv2.imread(png, cv2.IMREAD_COLOR)  # OpenCV returns BGR
        if bgr is None:
            missing += 1
            continue
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)  # match recorded encoding 'rgb8'
        h, w = rgb.shape[:2]

        sec = ns // 1_000_000_000
        nsec = ns % 1_000_000_000
        stamp = Time(sec=int(sec), nanosec=int(nsec))

        img = Image()
        img.header.stamp = stamp
        img.header.frame_id = "camera"
        img.height = int(h)
        img.width = int(w)
        img.encoding = "rgb8"
        img.is_bigendian = 0
        img.step = int(w * 3)
        img.data = rgb.reshape(-1).tobytes()

        fb = RenderedFeedback()
        fb.header.stamp = stamp
        fb.header.frame_id = "camera"
        fb.image = img  # load-bearing: decode_image_gray(m.image)
        fb.observed_stamp = stamp  # load-bearing: keys rendered_by_observed_stamp_
        fb.pose_stamp = stamp
        fb.pointcloud_stamp = stamp
        fb.frame_index = idx
        fb.rendered_preview_index = idx
        fb.render_mode = "rasterizer"

        writer.write(topic, serialize_message(fb), int(ns))
        written += 1

    del writer  # flush/close the bag
    print(
        f"[mux-feedback] wrote {written} RenderedFeedback msgs to {args.out_bag_dir} "
        f"(missing renders skipped={missing}, total stamps={len(stamps)}, storage={args.storage})"
    )
    if written == 0:
        raise SystemExit("[mux-feedback] wrote zero messages; aborting")


def cmd_mux_odometry(args):
    # Build a sim-time rosbag2 of nav_msgs/Odometry on
    # /gaussian_lic/frontend/input_odometry from a TUM trajectory, keyed by the
    # TUM stamp in nanoseconds. This is the EXTERNAL pose source for the map
    # retrain: ros2 bag play <this> alongside the sensor bag drives the
    # lic2_contract_adapter (raw_odometry_topic default
    # /gaussian_lic/frontend/input_odometry, adapter:382-390) which forwards it
    # verbatim to /pose_for_gs (publish_frontend_pose, adapter:984-993), and the
    # mapping_node consumes /pose_for_gs synchronized to /image_for_gs +
    # /points_for_gs (mapping_node:1023-1027) to build Gaussians at THESE poses.
    #
    # The bag timestamp == the TUM stamp_ns so that under --clock sim-time the
    # odometry lands at the same instants as the bag's /camera/image (the CT TUM
    # stamps are the observed image stamps at 10 Hz), keeping the mapper's
    # sync_tolerance_sec window satisfiable. The pose is published VERBATIM (the
    # adapter does not re-anchor); the frame the map ends up in is exactly the
    # frame of the TUM passed here -- so pass the SAME anchored.tum the renderer
    # will use (camera-mode anchor -> cam0 == identity) to keep the rebuilt map
    # consistent with anchor-tum --mode camera at render time.
    import rosbag2_py
    from rclpy.serialization import serialize_message
    from nav_msgs.msg import Odometry
    from builtin_interfaces.msg import Time

    rows = _load_tum(args.in_tum)

    if os.path.exists(args.out_bag_dir):
        raise SystemExit(
            f"out_bag_dir {args.out_bag_dir} already exists; rosbag2 will not "
            "overwrite. Remove it first (the outer-loop script does this)."
        )

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=args.out_bag_dir, storage_id=args.storage),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )
    topic = args.topic
    writer.create_topic(
        rosbag2_py.TopicMetadata(0, topic, "nav_msgs/msg/Odometry", "cdr")
    )

    written = 0
    for row in rows:
        t = row[0]
        tx, ty, tz, qx, qy, qz, qw = row[1:8]
        ns = int(round(t * 1e9))
        sec = ns // 1_000_000_000
        nsec = ns % 1_000_000_000
        stamp = Time(sec=int(sec), nanosec=int(nsec))

        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = args.frame
        od.child_frame_id = args.child_frame
        od.pose.pose.position.x = float(tx)
        od.pose.pose.position.y = float(ty)
        od.pose.pose.position.z = float(tz)
        od.pose.pose.orientation.x = float(qx)
        od.pose.pose.orientation.y = float(qy)
        od.pose.pose.orientation.z = float(qz)
        od.pose.pose.orientation.w = float(qw)

        writer.write(topic, serialize_message(od), int(ns))
        written += 1

    del writer
    print(
        f"[mux-odometry] wrote {written} Odometry msgs on {topic} to "
        f"{args.out_bag_dir} (frame={args.frame} child={args.child_frame} "
        f"storage={args.storage})"
    )
    if written == 0:
        raise SystemExit("[mux-odometry] wrote zero messages; aborting")


def main():
    ap = argparse.ArgumentParser(description="Option-A in-loop-render helper")
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("extract-stamps")
    p.add_argument("bag_dir")
    p.add_argument("image_topic")
    p.add_argument("out_json")
    p.set_defaults(func=cmd_extract_stamps)

    p = sub.add_parser("anchor-tum")
    p.add_argument("in_tum")
    p.add_argument("out_tum")
    p.add_argument("--mode", choices=["camera", "body"], default="camera")
    p.set_defaults(func=cmd_anchor_tum)

    p = sub.add_parser("mux-feedback")
    p.add_argument("render_dir")
    p.add_argument("stamps_json")
    p.add_argument("out_bag_dir")
    p.add_argument("--storage", default="mcap")
    p.set_defaults(func=cmd_mux_feedback)

    p = sub.add_parser("mux-odometry")
    p.add_argument("in_tum")
    p.add_argument("out_bag_dir")
    p.add_argument("--topic", default="/gaussian_lic/frontend/input_odometry")
    p.add_argument("--frame", default="map")
    p.add_argument("--child-frame", default="imu_link")
    p.add_argument("--storage", default="mcap")
    p.set_defaults(func=cmd_mux_odometry)

    args = ap.parse_args()
    args.func(args)


if __name__ == "__main__":
    sys.exit(main())
