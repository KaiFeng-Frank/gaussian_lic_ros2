#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later

import argparse
from pathlib import Path
import shutil
import sys


def load_rosbags_backend():
    try:
        from rosbags.convert import ConverterError, convert
        from rosbags.typesys import Stores, get_typestore
    except ImportError as exc:
        raise RuntimeError(
            "ROS1 bag conversion requires the optional Python package 'rosbags'.\n"
            "Install it in the conversion environment, for example:\n"
            "  /usr/bin/python3 -m pip install --user rosbags\n"
            "The Jazzy runtime path intentionally does not depend on ROS1 or rosbags."
        ) from exc
    return ConverterError, convert, Stores, get_typestore


def resolve_typestore(name, Stores, get_typestore):
    if name == "copy":
        return None
    try:
        return get_typestore(Stores(name))
    except ValueError as exc:
        valid = ", ".join(sorted(item.value for item in Stores))
        raise RuntimeError(f"unsupported typestore '{name}'. Valid values: copy, {valid}") from exc


def remove_existing_output(path: Path):
    if not path.exists():
        return
    if path.is_dir():
        shutil.rmtree(path)
    else:
        path.unlink()


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Convert a ROS1 .bag into rosbag2/mcap for Gaussian-LIC reproduction."
    )
    parser.add_argument(
        "--input",
        action="append",
        required=True,
        help="Input ROS1 .bag. Repeat to merge multiple bags chronologically.",
    )
    parser.add_argument("--output", required=True, help="Output rosbag2 directory")
    parser.add_argument(
        "--storage",
        default="mcap",
        choices=("mcap", "sqlite3"),
        help="rosbag2 storage backend to request when the converter backend supports it",
    )
    parser.add_argument(
        "--topic",
        action="append",
        dest="include_topics",
        help="Topic to include. Repeat for multiple topics. Alias for --include-topic.",
    )
    parser.add_argument("--include-topic", action="append", dest="include_topics")
    parser.add_argument("--exclude-topic", action="append", default=[])
    parser.add_argument("--include-msgtype", action="append", default=[])
    parser.add_argument("--exclude-msgtype", action="append", default=[])
    parser.add_argument(
        "--dst-typestore",
        default="ros2_jazzy",
        help="Destination typestore: ros2_jazzy, ros2_humble, latest, copy, etc. Default: ros2_jazzy.",
    )
    parser.add_argument("--dst-version", type=int, default=9, help="rosbag2 format version")
    parser.add_argument("--compress", choices=("none", "zstd"), default="none")
    parser.add_argument("--compress-mode", choices=("file", "message", "storage"), default="file")
    parser.add_argument("--force", action="store_true", help="Remove an existing output directory first")
    args = parser.parse_args(argv)

    input_paths = [Path(item).expanduser() for item in args.input]
    output_path = Path(args.output).expanduser()
    missing = [path for path in input_paths if not path.exists()]
    if missing:
        print("input bag does not exist: " + ", ".join(str(path) for path in missing), file=sys.stderr)
        return 2
    if output_path.suffix == ".bag":
        print("output must be a rosbag2 directory, not a ROS1 .bag path", file=sys.stderr)
        return 2
    if output_path.exists() and not args.force:
        print(f"output already exists: {output_path} (use --force to replace it)", file=sys.stderr)
        return 2

    try:
        ConverterError, convert, Stores, get_typestore = load_rosbags_backend()
        typestore = resolve_typestore(args.dst_typestore, Stores, get_typestore)
        if args.force:
            remove_existing_output(output_path)
        convert(
            input_paths,
            output_path,
            args.storage,
            args.dst_version,
            None if args.compress == "none" else args.compress,
            args.compress_mode,
            get_typestore(Stores.ROS1_NOETIC),
            typestore,
            args.exclude_topic,
            args.include_topics or [],
            args.exclude_msgtype,
            args.include_msgtype,
        )
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
        return 3
    except ConverterError as exc:
        print(f"conversion failed: {exc}", file=sys.stderr)
        return 4

    metadata_path = output_path / "metadata.yaml"
    if not metadata_path.exists():
        print(f"conversion finished but metadata.yaml was not created in {output_path}", file=sys.stderr)
        return 5

    print(f"converted {len(input_paths)} ROS1 bag(s) to rosbag2: {output_path}")
    print(f"storage={args.storage} typestore={args.dst_typestore} topics={args.include_topics or 'all'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
