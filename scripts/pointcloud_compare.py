#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later

import argparse
from collections import defaultdict
from dataclasses import dataclass
import json
import math
from pathlib import Path
import statistics
import sys


@dataclass(frozen=True)
class PointCloud:
    path: str
    declared_vertices: int
    properties: list
    points: list
    colors: list


def parse_ply_header(stream, path):
    first_line = stream.readline()
    if first_line.strip() != "ply":
        raise ValueError(f"{path}: not a PLY file")

    vertex_count = None
    vertex_properties = []
    current_element = None
    is_ascii = False

    for raw_line in stream:
        line = raw_line.strip()
        if line == "end_header":
            break
        if not line or line.startswith("comment "):
            continue

        parts = line.split()
        if parts[:2] == ["format", "ascii"]:
            is_ascii = True
        elif parts[0] == "element":
            current_element = parts[1]
            if current_element == "vertex":
                vertex_count = int(parts[2])
        elif parts[0] == "property" and current_element == "vertex":
            if len(parts) >= 2 and parts[1] == "list":
                raise ValueError(f"{path}: list properties on vertex elements are not supported")
            if len(parts) != 3:
                raise ValueError(f"{path}: malformed vertex property line: {line}")
            vertex_properties.append(parts[2])
    else:
        raise ValueError(f"{path}: missing end_header")

    if not is_ascii:
        raise ValueError(f"{path}: only ascii PLY files are supported")
    if vertex_count is None:
        raise ValueError(f"{path}: missing element vertex declaration")
    for required in ("x", "y", "z"):
        if required not in vertex_properties:
            raise ValueError(f"{path}: missing vertex property {required}")
    return vertex_count, vertex_properties


def load_ply(path, max_points):
    resolved = str(Path(path).expanduser().resolve())
    points = []
    colors = []
    with Path(path).expanduser().open("r", encoding="utf-8") as stream:
        vertex_count, properties = parse_ply_header(stream, path)
        property_index = {name: index for index, name in enumerate(properties)}
        color_properties = ("red", "green", "blue")
        has_color = all(name in property_index for name in color_properties)
        stride = 1
        if max_points > 0 and vertex_count > max_points:
            stride = math.ceil(vertex_count / max_points)

        for row in range(vertex_count):
            raw_line = stream.readline()
            if not raw_line:
                raise ValueError(f"{path}: expected {vertex_count} vertices, found {row}")
            if row % stride != 0:
                continue

            parts = raw_line.split()
            if len(parts) < len(properties):
                raise ValueError(f"{path}: vertex row {row + 1} has too few columns")

            try:
                point = (
                    float(parts[property_index["x"]]),
                    float(parts[property_index["y"]]),
                    float(parts[property_index["z"]]),
                )
                if not all(math.isfinite(value) for value in point):
                    continue
                points.append(point)
                if has_color:
                    colors.append(
                        (
                            float(parts[property_index["red"]]),
                            float(parts[property_index["green"]]),
                            float(parts[property_index["blue"]]),
                        )
                    )
            except ValueError as exc:
                raise ValueError(f"{path}: non-numeric vertex row {row + 1}") from exc

    if not points:
        raise ValueError(f"{path}: no valid xyz vertices found")
    return PointCloud(resolved, vertex_count, properties, points, colors)


def add(lhs, rhs):
    return lhs[0] + rhs[0], lhs[1] + rhs[1], lhs[2] + rhs[2]


def subtract(lhs, rhs):
    return lhs[0] - rhs[0], lhs[1] - rhs[1], lhs[2] - rhs[2]


def scale(point, factor):
    return point[0] * factor, point[1] * factor, point[2] * factor


def distance(lhs, rhs):
    dx = lhs[0] - rhs[0]
    dy = lhs[1] - rhs[1]
    dz = lhs[2] - rhs[2]
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def centroid(points):
    total = (0.0, 0.0, 0.0)
    for point in points:
        total = add(total, point)
    return scale(total, 1.0 / len(points))


def bounds(points):
    return {
        "min": [
            min(point[axis] for point in points)
            for axis in range(3)
        ],
        "max": [
            max(point[axis] for point in points)
            for axis in range(3)
        ],
    }


def mean_rgb(colors):
    if not colors:
        return []
    total = (0.0, 0.0, 0.0)
    for color in colors:
        total = add(total, color)
    return list(scale(total, 1.0 / len(colors)))


def voxel_downsample(points, voxel_size):
    if voxel_size <= 0.0:
        return list(points)

    cells = {}
    for point in points:
        key = tuple(math.floor(value / voxel_size) for value in point)
        if key not in cells:
            cells[key] = [0.0, 0.0, 0.0, 0]
        cell = cells[key]
        cell[0] += point[0]
        cell[1] += point[1]
        cell[2] += point[2]
        cell[3] += 1

    return [
        (cell[0] / cell[3], cell[1] / cell[3], cell[2] / cell[3])
        for cell in cells.values()
    ]


def build_spatial_index(points, cell_size):
    index = defaultdict(list)
    for point in points:
        key = tuple(math.floor(value / cell_size) for value in point)
        index[key].append(point)
    return index


def nearest_distances(source, target, max_nearest):
    if not source or not target:
        return [], len(source)

    cell_size = max(max_nearest, 1e-9)
    index = build_spatial_index(target, cell_size)
    distances = []
    unmatched = 0
    search_cells = 1

    for point in source:
        base_key = tuple(math.floor(value / cell_size) for value in point)
        best = None
        for dx in range(-search_cells, search_cells + 1):
            for dy in range(-search_cells, search_cells + 1):
                for dz in range(-search_cells, search_cells + 1):
                    for candidate in index.get(
                        (base_key[0] + dx, base_key[1] + dy, base_key[2] + dz),
                        (),
                    ):
                        candidate_distance = distance(point, candidate)
                        if best is None or candidate_distance < best:
                            best = candidate_distance

        if best is None or best > max_nearest:
            unmatched += 1
            distances.append(max_nearest)
        else:
            distances.append(best)

    return distances, unmatched


def summarize_distances(distances):
    if not distances:
        return {
            "mean_m": 0.0,
            "median_m": 0.0,
            "rmse_m": 0.0,
            "max_m": 0.0,
        }
    return {
        "mean_m": sum(distances) / len(distances),
        "median_m": statistics.median(distances),
        "rmse_m": math.sqrt(sum(value * value for value in distances) / len(distances)),
        "max_m": max(distances),
    }


def translated(points, offset):
    return [subtract(point, offset) for point in points]


def compute_report(args):
    baseline = load_ply(args.baseline, args.max_points)
    current = load_ply(args.current, args.max_points)
    baseline_centroid = centroid(baseline.points)
    current_centroid = centroid(current.points)

    offset = (0.0, 0.0, 0.0)
    if args.align == "centroid":
        offset = subtract(current_centroid, baseline_centroid)

    baseline_points = voxel_downsample(baseline.points, args.voxel_size)
    current_points = voxel_downsample(translated(current.points, offset), args.voxel_size)
    aligned_current_centroid = centroid(translated(current.points, offset))
    centroid_drift = distance(baseline_centroid, aligned_current_centroid)

    forward, forward_unmatched = nearest_distances(
        baseline_points,
        current_points,
        args.max_nearest_m,
    )
    reverse, reverse_unmatched = nearest_distances(
        current_points,
        baseline_points,
        args.max_nearest_m,
    )
    bidirectional = forward + reverse

    count_ratio = len(current.points) / max(1, len(baseline.points))
    unmatched_ratio = (forward_unmatched + reverse_unmatched) / max(1, len(bidirectional))
    color_report = {
        "compared": bool(baseline.colors and current.colors),
        "baseline_mean_rgb": mean_rgb(baseline.colors),
        "current_mean_rgb": mean_rgb(current.colors),
        "mean_rgb_distance": 0.0,
    }
    if color_report["compared"]:
        color_report["mean_rgb_distance"] = distance(
            tuple(color_report["baseline_mean_rgb"]),
            tuple(color_report["current_mean_rgb"]),
        )

    distance_summary = summarize_distances(bidirectional)
    errors = []
    if count_ratio < args.min_count_ratio:
        errors.append(f"point count ratio {count_ratio:.3f} < {args.min_count_ratio:.3f}")
    if count_ratio > args.max_count_ratio:
        errors.append(f"point count ratio {count_ratio:.3f} > {args.max_count_ratio:.3f}")
    if centroid_drift > args.max_centroid_drift_m:
        errors.append(f"centroid drift {centroid_drift:.6f} m > {args.max_centroid_drift_m:.6f} m")
    if distance_summary["rmse_m"] > args.max_chamfer_rmse_m:
        errors.append(
            f"bidirectional nearest rmse {distance_summary['rmse_m']:.6f} m "
            f"> {args.max_chamfer_rmse_m:.6f} m"
        )
    if distance_summary["mean_m"] > args.max_chamfer_mean_m:
        errors.append(
            f"bidirectional nearest mean {distance_summary['mean_m']:.6f} m "
            f"> {args.max_chamfer_mean_m:.6f} m"
        )
    if distance_summary["max_m"] > args.max_chamfer_max_m:
        errors.append(
            f"bidirectional nearest max {distance_summary['max_m']:.6f} m "
            f"> {args.max_chamfer_max_m:.6f} m"
        )
    if unmatched_ratio > args.max_unmatched_ratio:
        errors.append(f"unmatched ratio {unmatched_ratio:.2%} > {args.max_unmatched_ratio:.2%}")
    if color_report["compared"] and color_report["mean_rgb_distance"] > args.max_mean_rgb_drift:
        errors.append(
            f"mean RGB drift {color_report['mean_rgb_distance']:.6f} "
            f"> {args.max_mean_rgb_drift:.6f}"
        )

    return {
        "baseline": {
            "path": baseline.path,
            "declared_vertices": baseline.declared_vertices,
            "loaded_points": len(baseline.points),
            "downsampled_points": len(baseline_points),
            "centroid": list(baseline_centroid),
            "bounds": bounds(baseline.points),
            "has_rgb": bool(baseline.colors),
        },
        "current": {
            "path": current.path,
            "declared_vertices": current.declared_vertices,
            "loaded_points": len(current.points),
            "downsampled_points": len(current_points),
            "centroid": list(aligned_current_centroid),
            "bounds": bounds(translated(current.points, offset)),
            "has_rgb": bool(current.colors),
        },
        "alignment": {
            "mode": args.align,
            "current_offset_removed": list(offset),
        },
        "voxel_size_m": args.voxel_size,
        "count_ratio": count_ratio,
        "centroid_drift_m": centroid_drift,
        "nearest": {
            "forward": {
                **summarize_distances(forward),
                "unmatched": forward_unmatched,
            },
            "reverse": {
                **summarize_distances(reverse),
                "unmatched": reverse_unmatched,
            },
            "bidirectional": {
                **distance_summary,
                "unmatched": forward_unmatched + reverse_unmatched,
                "unmatched_ratio": unmatched_ratio,
            },
        },
        "color": color_report,
        "thresholds": {
            "max_nearest_m": args.max_nearest_m,
            "min_count_ratio": args.min_count_ratio,
            "max_count_ratio": args.max_count_ratio,
            "max_centroid_drift_m": args.max_centroid_drift_m,
            "max_chamfer_rmse_m": args.max_chamfer_rmse_m,
            "max_chamfer_mean_m": args.max_chamfer_mean_m,
            "max_chamfer_max_m": args.max_chamfer_max_m,
            "max_unmatched_ratio": args.max_unmatched_ratio,
            "max_mean_rgb_drift": args.max_mean_rgb_drift,
        },
        "ok": not errors,
        "errors": errors,
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description="Compare two ASCII PLY point clouds for map drift.")
    parser.add_argument("--baseline", required=True, help="Baseline point_cloud.ply")
    parser.add_argument("--current", required=True, help="Current point_cloud.ply")
    parser.add_argument("--output", help="Optional JSON report path")
    parser.add_argument("--max-points", type=int, default=200000)
    parser.add_argument("--voxel-size", type=float, default=0.05)
    parser.add_argument("--align", choices=("none", "centroid"), default="none")
    parser.add_argument("--max-nearest-m", type=float, default=0.5)
    parser.add_argument("--min-count-ratio", type=float, default=0.5)
    parser.add_argument("--max-count-ratio", type=float, default=2.0)
    parser.add_argument("--max-centroid-drift-m", type=float, default=0.2)
    parser.add_argument("--max-chamfer-rmse-m", type=float, default=0.15)
    parser.add_argument("--max-chamfer-mean-m", type=float, default=0.1)
    parser.add_argument("--max-chamfer-max-m", type=float, default=0.5)
    parser.add_argument("--max-unmatched-ratio", type=float, default=0.05)
    parser.add_argument("--max-mean-rgb-drift", type=float, default=40.0)
    parser.add_argument("--json", action="store_true", help="Print the full JSON report")
    args = parser.parse_args(argv)

    try:
        report = compute_report(args)
    except Exception as exc:  # noqa: BLE001 - CLI should report malformed PLY files uniformly.
        print(f"point cloud comparison failed: {exc}", file=sys.stderr)
        return 2

    if args.output:
        Path(args.output).write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    if args.json or not report["ok"]:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        print(
            "point cloud comparison OK: "
            f"baseline={report['baseline']['downsampled_points']}pts "
            f"current={report['current']['downsampled_points']}pts "
            f"rmse={report['nearest']['bidirectional']['rmse_m']:.6f}m"
        )

    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
