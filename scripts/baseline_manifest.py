#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later

import argparse
import hashlib
import json
from pathlib import Path
import sys


REQUIRED_FILES = (
    "trajectory.tum",
    "point_cloud.ply",
    "metrics.json",
    "run.log",
)


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def file_entry(path):
    return {
        "path": str(path),
        "bytes": path.stat().st_size,
        "sha256": sha256_file(path),
    }


def count_tum_poses(path):
    count = 0
    for line_number, raw_line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        if len(parts) != 8:
            raise ValueError(f"{path}:{line_number}: expected 8 TUM columns, got {len(parts)}")
        for value in parts:
            float(value)
        count += 1
    return count


def count_ply_vertices(path):
    with path.open("r", encoding="utf-8") as stream:
        if stream.readline().strip() != "ply":
            raise ValueError(f"{path}: not a PLY file")
        for raw_line in stream:
            line = raw_line.strip()
            if line == "end_header":
                break
            parts = line.split()
            if len(parts) == 3 and parts[:2] == ["element", "vertex"]:
                return int(parts[2])
    raise ValueError(f"{path}: missing element vertex declaration")


def load_metrics(path):
    with path.open("r", encoding="utf-8") as stream:
        data = json.load(stream)
    if not isinstance(data, dict):
        raise ValueError(f"{path}: expected JSON object")
    return data


def render_entries(render_dir):
    if not render_dir.exists():
        return []
    return [
        file_entry(path)
        for path in sorted(render_dir.rglob("*"))
        if path.is_file()
    ]


def build_manifest(args):
    root = Path(args.baseline).expanduser().resolve()
    errors = []
    artifacts = {}

    if not root.exists():
        raise FileNotFoundError(f"baseline directory not found: {root}")
    if not root.is_dir():
        raise NotADirectoryError(f"baseline path is not a directory: {root}")

    for name in REQUIRED_FILES:
        path = root / name
        if not path.is_file():
            errors.append(f"missing required file: {name}")
            continue
        artifacts[name] = file_entry(path)

    render_dir = root / "renders"
    if not render_dir.is_dir():
        errors.append("missing required directory: renders")
        renders = []
    else:
        renders = render_entries(render_dir)
        if len(renders) < args.min_renders:
            errors.append(f"render files {len(renders)} < min_renders {args.min_renders}")

    trajectory_poses = 0
    point_cloud_vertices = 0
    metrics_keys = []

    if "trajectory.tum" in artifacts:
        try:
            trajectory_poses = count_tum_poses(root / "trajectory.tum")
        except Exception as exc:  # noqa: BLE001 - report artifact validation uniformly.
            errors.append(str(exc))
    if trajectory_poses <= 0:
        errors.append("trajectory.tum has no poses")

    if "point_cloud.ply" in artifacts:
        try:
            point_cloud_vertices = count_ply_vertices(root / "point_cloud.ply")
        except Exception as exc:  # noqa: BLE001 - report artifact validation uniformly.
            errors.append(str(exc))
    if point_cloud_vertices <= 0:
        errors.append("point_cloud.ply has no vertices")

    if "metrics.json" in artifacts:
        try:
            metrics = load_metrics(root / "metrics.json")
            metrics_keys = sorted(metrics)
        except Exception as exc:  # noqa: BLE001 - report artifact validation uniformly.
            errors.append(str(exc))

    manifest = {
        "schema": "gaussian_lic_baseline_manifest/v1",
        "dataset": args.dataset,
        "sequence": args.sequence or root.name,
        "baseline_dir": str(root),
        "required_files": list(REQUIRED_FILES),
        "artifacts": artifacts,
        "renders": {
            "path": str(render_dir),
            "files": renders,
            "file_count": len(renders),
            "bytes": sum(item["bytes"] for item in renders),
        },
        "summary": {
            "trajectory_poses": trajectory_poses,
            "point_cloud_vertices": point_cloud_vertices,
            "metrics_keys": metrics_keys,
        },
        "ok": not errors,
        "errors": errors,
    }
    return manifest


def main(argv=None):
    parser = argparse.ArgumentParser(description="Validate and fingerprint a Gaussian-LIC baseline artifact directory.")
    parser.add_argument("--baseline", required=True, help="Baseline directory, for example baseline/fastlivo2/<sequence>")
    parser.add_argument("--dataset", default="fastlivo2")
    parser.add_argument("--sequence", help="Sequence name. Defaults to the baseline directory name.")
    parser.add_argument("--min-renders", type=int, default=1)
    parser.add_argument("--output", help="Manifest JSON path. Defaults to <baseline>/baseline_manifest.json with --write.")
    parser.add_argument("--write", action="store_true", help="Write the manifest JSON file.")
    parser.add_argument("--json", action="store_true", help="Print the full JSON manifest.")
    args = parser.parse_args(argv)

    try:
        manifest = build_manifest(args)
    except Exception as exc:  # noqa: BLE001 - CLI should report malformed baseline directories uniformly.
        print(f"baseline manifest failed: {exc}", file=sys.stderr)
        return 2

    output_path = None
    if args.write or args.output:
        output_path = Path(args.output) if args.output else Path(args.baseline) / "baseline_manifest.json"
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    if args.json or not manifest["ok"]:
        print(json.dumps(manifest, indent=2, sort_keys=True))
    else:
        suffix = f" manifest={output_path}" if output_path else ""
        print(
            "baseline manifest OK: "
            f"sequence={manifest['sequence']} "
            f"poses={manifest['summary']['trajectory_poses']} "
            f"vertices={manifest['summary']['point_cloud_vertices']} "
            f"renders={manifest['renders']['file_count']}"
            f"{suffix}"
        )

    return 0 if manifest["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
