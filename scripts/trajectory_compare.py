#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import statistics
import sys


@dataclass(frozen=True)
class TumPose:
    stamp: float
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


def load_tum(path):
    poses = []
    for line_number, raw_line in enumerate(Path(path).read_text(encoding="utf-8").splitlines(), start=1):
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        if len(parts) != 8:
            raise ValueError(f"{path}:{line_number}: expected 8 TUM columns, got {len(parts)}")
        try:
            values = [float(part) for part in parts]
        except ValueError as exc:
            raise ValueError(f"{path}:{line_number}: non-numeric TUM value") from exc
        poses.append(TumPose(*values))

    if not poses:
        raise ValueError(f"{path}: no TUM poses found")
    return sorted(poses, key=lambda pose: pose.stamp)


def associate_by_timestamp(baseline, current, max_dt):
    matches = []
    current_index = 0
    for baseline_pose in baseline:
        while (
            current_index + 1 < len(current)
            and abs(current[current_index + 1].stamp - baseline_pose.stamp)
            <= abs(current[current_index].stamp - baseline_pose.stamp)
        ):
            current_index += 1

        if current_index >= len(current):
            break

        current_pose = current[current_index]
        if abs(current_pose.stamp - baseline_pose.stamp) <= max_dt:
            matches.append((baseline_pose, current_pose))
            current_index += 1
        elif current_pose.stamp < baseline_pose.stamp:
            current_index += 1

    return matches


def translation_tuple(pose):
    return pose.x, pose.y, pose.z


def subtract_offset(position, offset):
    return (
        position[0] - offset[0],
        position[1] - offset[1],
        position[2] - offset[2],
    )


def mean_position(positions):
    if not positions:
        return 0.0, 0.0, 0.0
    inv_count = 1.0 / len(positions)
    return (
        sum(position[0] for position in positions) * inv_count,
        sum(position[1] for position in positions) * inv_count,
        sum(position[2] for position in positions) * inv_count,
    )


def yaw_alignment_transform(baseline_positions, current_positions):
    baseline_mean = mean_position(baseline_positions)
    current_mean = mean_position(current_positions)
    dot_sum = 0.0
    cross_sum = 0.0
    for baseline_position, current_position in zip(baseline_positions, current_positions):
        current_x = current_position[0] - current_mean[0]
        current_y = current_position[1] - current_mean[1]
        baseline_x = baseline_position[0] - baseline_mean[0]
        baseline_y = baseline_position[1] - baseline_mean[1]
        dot_sum += current_x * baseline_x + current_y * baseline_y
        cross_sum += current_x * baseline_y - current_y * baseline_x
    yaw = math.atan2(cross_sum, dot_sum) if dot_sum != 0.0 or cross_sum != 0.0 else 0.0
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    translated_current_mean_x = cos_yaw * current_mean[0] - sin_yaw * current_mean[1]
    translated_current_mean_y = sin_yaw * current_mean[0] + cos_yaw * current_mean[1]
    return {
        "yaw_rad": yaw,
        "cos": cos_yaw,
        "sin": sin_yaw,
        "tx": baseline_mean[0] - translated_current_mean_x,
        "ty": baseline_mean[1] - translated_current_mean_y,
        "tz": baseline_mean[2] - current_mean[2],
    }


def apply_yaw_alignment(position, transform):
    x = transform["cos"] * position[0] - transform["sin"] * position[1] + transform["tx"]
    y = transform["sin"] * position[0] + transform["cos"] * position[1] + transform["ty"]
    z = position[2] + transform["tz"]
    return x, y, z


def distance(lhs, rhs):
    dx = lhs[0] - rhs[0]
    dy = lhs[1] - rhs[1]
    dz = lhs[2] - rhs[2]
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def path_length(positions):
    if len(positions) < 2:
        return 0.0
    return sum(distance(previous, current) for previous, current in zip(positions, positions[1:]))


def summarize_matches(matches, align):
    baseline_positions = [translation_tuple(pair[0]) for pair in matches]
    raw_current_positions = [translation_tuple(pair[1]) for pair in matches]
    alignment_details = {}
    if matches and align == "first":
        baseline_first = baseline_positions[0]
        current_first = raw_current_positions[0]
        offset = (
            current_first[0] - baseline_first[0],
            current_first[1] - baseline_first[1],
            current_first[2] - baseline_first[2],
        )
        current_positions = [subtract_offset(position, offset) for position in raw_current_positions]
        alignment_details = {"translation_offset": offset}
    elif matches and align == "yaw":
        yaw_transform = yaw_alignment_transform(baseline_positions, raw_current_positions)
        current_positions = [
            apply_yaw_alignment(position, yaw_transform)
            for position in raw_current_positions
        ]
        alignment_details = {
            "yaw_rad": yaw_transform["yaw_rad"],
            "translation": [yaw_transform["tx"], yaw_transform["ty"], yaw_transform["tz"]],
        }
    else:
        current_positions = raw_current_positions
    translation_errors = [
        distance(baseline_position, current_position)
        for baseline_position, current_position in zip(baseline_positions, current_positions)
    ]

    if translation_errors:
        mean_error = sum(translation_errors) / len(translation_errors)
        rmse = math.sqrt(sum(error * error for error in translation_errors) / len(translation_errors))
        max_error = max(translation_errors)
        median_error = statistics.median(translation_errors)
    else:
        mean_error = 0.0
        rmse = 0.0
        max_error = 0.0
        median_error = 0.0

    baseline_path = path_length(baseline_positions)
    current_path = path_length(current_positions)
    absolute_path_drift = abs(current_path - baseline_path)
    if baseline_path > 0.0:
        relative_path_drift = absolute_path_drift / baseline_path
    else:
        relative_path_drift = 0.0

    return {
        "alignment_details": alignment_details,
        "translation": {
            "rmse_m": rmse,
            "mean_m": mean_error,
            "median_m": median_error,
            "max_m": max_error,
        },
        "path_length": {
            "baseline_m": baseline_path,
            "current_m": current_path,
            "absolute_drift_m": absolute_path_drift,
            "relative_drift": relative_path_drift,
        },
    }


def shifted_poses(poses, offset_s):
    if abs(offset_s) <= 1.0e-12:
        return poses
    return [
        TumPose(
            pose.stamp + offset_s,
            pose.x,
            pose.y,
            pose.z,
            pose.qx,
            pose.qy,
            pose.qz,
            pose.qw,
        )
        for pose in poses
    ]


def compute_time_offset_sweep(baseline, current, args):
    step = args.time_offset_sweep_step
    if step <= 0.0:
        return None
    if args.time_offset_sweep_min > args.time_offset_sweep_max:
        raise ValueError("time offset sweep min must be <= max")

    candidates = []
    min_matches = args.time_offset_sweep_min_matches or args.min_matches
    offset = args.time_offset_sweep_min
    # Use an integer-like loop guard to avoid floating-point drift preventing
    # the max endpoint from being evaluated.
    max_iterations = int(math.floor((args.time_offset_sweep_max - offset) / step)) + 1
    for index in range(max_iterations + 1):
        candidate_offset = args.time_offset_sweep_min + index * step
        if candidate_offset > args.time_offset_sweep_max + 1.0e-12:
            break
        matches = associate_by_timestamp(
            baseline,
            shifted_poses(current, candidate_offset),
            args.max_association_dt,
        )
        if len(matches) < min_matches:
            continue
        summary = summarize_matches(matches, args.align)
        candidates.append({
            "offset_sec": candidate_offset,
            "matched_poses": len(matches),
            "coverage": len(matches) / max(1, len(baseline)),
            "translation_rmse_m": summary["translation"]["rmse_m"],
            "translation_mean_m": summary["translation"]["mean_m"],
            "path_relative_drift": summary["path_length"]["relative_drift"],
        })

    candidates.sort(key=lambda item: (item["translation_rmse_m"], -item["matched_poses"]))
    return {
        "enabled": True,
        "min_offset_sec": args.time_offset_sweep_min,
        "max_offset_sec": args.time_offset_sweep_max,
        "step_sec": step,
        "min_matches": min_matches,
        "best": candidates[0] if candidates else None,
        "top_offsets": candidates[:10],
    }


def compute_report(args):
    baseline = load_tum(args.baseline)
    current = load_tum(args.current)
    matches = associate_by_timestamp(baseline, current, args.max_association_dt)

    errors = []
    if len(matches) < args.min_matches:
        errors.append(f"matched poses {len(matches)} < min_matches {args.min_matches}")

    coverage = len(matches) / max(1, len(baseline))
    if coverage < args.min_coverage:
        errors.append(f"coverage {coverage:.2%} < min_coverage {args.min_coverage:.2%}")

    summary = summarize_matches(matches, args.align)
    alignment_details = summary["alignment_details"]
    rmse = summary["translation"]["rmse_m"]
    mean_error = summary["translation"]["mean_m"]
    max_error = summary["translation"]["max_m"]
    relative_path_drift = summary["path_length"]["relative_drift"]
    baseline_path = summary["path_length"]["baseline_m"]

    thresholds = {
        "max_rmse_m": args.max_rmse_m,
        "max_mean_m": args.max_mean_m,
        "max_error_m": args.max_error_m,
        "max_path_drift": args.max_path_drift,
        "min_matches": args.min_matches,
        "min_coverage": args.min_coverage,
    }

    if rmse > args.max_rmse_m:
        errors.append(f"translation rmse {rmse:.6f} m > {args.max_rmse_m:.6f} m")
    if mean_error > args.max_mean_m:
        errors.append(f"translation mean {mean_error:.6f} m > {args.max_mean_m:.6f} m")
    if max_error > args.max_error_m:
        errors.append(f"translation max {max_error:.6f} m > {args.max_error_m:.6f} m")
    if baseline_path > 0.0 and relative_path_drift > args.max_path_drift:
        errors.append(f"path drift {relative_path_drift:.2%} > {args.max_path_drift:.2%}")

    report = {
        "baseline": str(Path(args.baseline).expanduser().resolve()),
        "current": str(Path(args.current).expanduser().resolve()),
        "alignment": args.align,
        "alignment_details": alignment_details,
        "max_association_dt_sec": args.max_association_dt,
        "baseline_poses": len(baseline),
        "current_poses": len(current),
        "matched_poses": len(matches),
        "coverage": coverage,
        "translation": summary["translation"],
        "path_length": summary["path_length"],
        "thresholds": thresholds,
        "ok": not errors,
        "errors": errors,
    }
    time_offset_sweep = compute_time_offset_sweep(baseline, current, args)
    if time_offset_sweep is not None:
        report["time_offset_sweep"] = time_offset_sweep
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description="Compare two TUM trajectories for reproduction drift.")
    parser.add_argument("--baseline", required=True, help="Baseline trajectory.tum")
    parser.add_argument("--current", required=True, help="Current trajectory.tum")
    parser.add_argument("--output", help="Optional JSON report path")
    parser.add_argument(
        "--align",
        choices=("none", "first", "yaw"),
        default="none",
        help=(
            "Apply no alignment, remove the first matched translation offset, or fit a "
            "no-scale yaw+xyz translation transform for local-world-to-reference ATE."
        ),
    )
    parser.add_argument("--max-association-dt", type=float, default=0.02)
    parser.add_argument("--min-matches", type=int, default=2)
    parser.add_argument("--min-coverage", type=float, default=0.8)
    parser.add_argument("--max-rmse-m", type=float, default=0.05)
    parser.add_argument("--max-mean-m", type=float, default=0.03)
    parser.add_argument("--max-error-m", type=float, default=0.15)
    parser.add_argument("--max-path-drift", type=float, default=0.05)
    parser.add_argument(
        "--time-offset-sweep-min",
        type=float,
        default=0.0,
        help="Minimum current-trajectory timestamp offset, in seconds, for optional drift diagnostics.",
    )
    parser.add_argument(
        "--time-offset-sweep-max",
        type=float,
        default=0.0,
        help="Maximum current-trajectory timestamp offset, in seconds, for optional drift diagnostics.",
    )
    parser.add_argument(
        "--time-offset-sweep-step",
        type=float,
        default=0.0,
        help="Positive step size, in seconds, to enable timestamp offset diagnostics.",
    )
    parser.add_argument(
        "--time-offset-sweep-min-matches",
        type=int,
        default=0,
        help="Minimum matches required for a timestamp offset candidate; defaults to --min-matches.",
    )
    parser.add_argument("--json", action="store_true", help="Print the full JSON report")
    args = parser.parse_args(argv)

    try:
        report = compute_report(args)
    except Exception as exc:  # noqa: BLE001 - CLI should report malformed trajectory files uniformly.
        print(f"trajectory comparison failed: {exc}", file=sys.stderr)
        return 2

    if args.output:
        Path(args.output).write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    if args.json or not report["ok"]:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        print(
            "trajectory comparison OK: "
            f"matches={report['matched_poses']} "
            f"rmse={report['translation']['rmse_m']:.6f}m "
            f"path_drift={report['path_length']['relative_drift']:.2%}"
        )

    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
