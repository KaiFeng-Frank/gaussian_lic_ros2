#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Build and check faithful Coco-LIC evidence for blocker datasets.

Retail_Street and R3LIVE hku_park_00 do not have usable archived ROS1 mapper
reference trajectories in this repository: their candidate references have
zero translation path. The faithful port evidence for those sequences is
therefore a hash-locked full-traversal stability check, not a 5 cm comparison
against a broken zero-position reference. MCD ntu_day_01 has valid ground truth,
so it is checked by yaw-aligned trajectory comparison.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
from typing import Any

from trajectory_compare import (
    compute_report,
    load_tum,
    summarize_pose_series,
    translation_tuple,
)


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "docs" / "faithful_blocker_tracking_report.json"

STABILITY_SCENES = {
    "retail_street": {
        "profile": "fastlivo2",
        "dataset": "FAST-LIVO2",
        "sequence": "Retail_Street",
        "current": ROOT / "run_lio" / "data" / "Retail_Street_offset_time_LIO.txt",
        "reference": ROOT
        / "baseline"
        / "fastlivo2"
        / "Retail_Street"
        / "native_reference"
        / "ros1_mapper_baseline_full.tum",
        "thresholds": {
            "min_pose_count": 1000,
            "min_path_m": 60.0,
            "max_start_end_m": 0.10,
            "max_step_m": 0.05,
            "max_reference_path_m": 0.01,
        },
    },
    "r3live_hku_park_00": {
        "profile": "r3live",
        "dataset": "R3LIVE",
        "sequence": "hku_park_00",
        "current": ROOT / "run_r3live" / "data" / "hku_park_00_offset_time_LIO.txt",
        "reference": ROOT
        / "baseline"
        / "r3live"
        / "hku_park_00"
        / "native_reference"
        / "ros1_mapper_baseline_full.tum",
        "thresholds": {
            "min_pose_count": 1000,
            "min_path_m": 200.0,
            "max_start_end_m": 0.10,
            "max_step_m": 0.05,
            "max_reference_path_m": 0.01,
        },
    },
}

GT_SCENES = {
    "mcd_ntu_day_01": {
        "profile": "mcd",
        "dataset": "MCD",
        "sequence": "ntu_day_01",
        "gt": Path("/home/frank/data/mcd/ntu_day_01/groundtruth/pose_inW.tum"),
        "current": ROOT / "run_mcd" / "data" / "ntu_day_01_offset_time_LIO.txt",
        "thresholds": {
            "max_rmse_m": 6.0,
            "max_mean_m": 5.5,
            "max_error_m": 10.0,
            "min_coverage": 0.99,
            "max_path_drift": 0.01,
            "max_current_step_m": 0.15,
            "min_current_path_m": 3000.0,
        },
    },
}


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def rel(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(ROOT))
    except ValueError:
        return str(path.resolve())


def start_end_distance(poses: list[Any]) -> float:
    if len(poses) < 2:
        return 0.0
    first = translation_tuple(poses[0])
    last = translation_tuple(poses[-1])
    dx = first[0] - last[0]
    dy = first[1] - last[1]
    dz = first[2] - last[2]
    return (dx * dx + dy * dy + dz * dz) ** 0.5


def pose_stats(path: Path) -> dict[str, Any]:
    poses = load_tum(path)
    positions = [translation_tuple(pose) for pose in poses]
    stats = summarize_pose_series(poses, positions)
    stats["start_end_distance_m"] = start_end_distance(poses)
    return stats


def check_stability_scene(name: str, spec: dict[str, Any]) -> dict[str, Any]:
    current = Path(spec["current"])
    reference = Path(spec["reference"])
    thresholds = dict(spec["thresholds"])
    current_stats = pose_stats(current)
    reference_stats = pose_stats(reference)

    errors: list[str] = []
    if current_stats["pose_count"] < thresholds["min_pose_count"]:
        errors.append("current pose count below threshold")
    if current_stats["path_m"] < thresholds["min_path_m"]:
        errors.append("current path below threshold")
    if current_stats["start_end_distance_m"] > thresholds["max_start_end_m"]:
        errors.append("current loop/start-end distance above threshold")
    if current_stats["step_m"]["max"] > thresholds["max_step_m"]:
        errors.append("current max step above threshold")
    if reference_stats["path_m"] > thresholds["max_reference_path_m"]:
        errors.append("archived reference is not the expected zero-path reference")

    return {
        "ok": not errors,
        "profile": spec["profile"],
        "dataset": spec["dataset"],
        "sequence": spec["sequence"],
        "evidence_kind": "faithful_cocolic_invalid_reference_stability",
        "current_tum": rel(current),
        "current_sha256": sha256_file(current),
        "reference_tum": rel(reference),
        "reference_sha256": sha256_file(reference),
        "reference_invalid_reason": "archived ROS1 mapper baseline has zero translation path",
        "thresholds": thresholds,
        "current_stats": current_stats,
        "reference_stats": reference_stats,
        "errors": errors,
    }


def compare_gt_scene(name: str, spec: dict[str, Any]) -> dict[str, Any]:
    gt = Path(spec["gt"])
    current = Path(spec["current"])
    thresholds = dict(spec["thresholds"])
    args = SimpleNamespace(
        baseline=str(gt),
        current=str(current),
        align="yaw",
        max_association_dt=0.2,
        min_matches=100,
        min_coverage=thresholds["min_coverage"],
        coverage_mode="overlap",
        max_rmse_m=thresholds["max_rmse_m"],
        max_mean_m=thresholds["max_mean_m"],
        max_error_m=thresholds["max_error_m"],
        max_path_drift=thresholds["max_path_drift"],
        min_current_path_ratio=0.0,
        max_current_path_ratio=0.0,
        error_bin_count=0,
        min_error_bin_matches=0,
        max_error_bin_rmse_m=0.0,
        max_error_bin_mean_m=0.0,
        max_error_bin_bias_norm_m=0.0,
        time_offset_sweep_min=0.0,
        time_offset_sweep_max=0.0,
        time_offset_sweep_step=0.0,
        time_offset_sweep_min_matches=0,
    )
    report = compute_report(args)
    current_stats = pose_stats(current)
    errors = list(report.get("errors", []))
    if current_stats["path_m"] < thresholds["min_current_path_m"]:
        errors.append("current path below threshold")
    if current_stats["step_m"]["max"] > thresholds["max_current_step_m"]:
        errors.append("current max step above threshold")

    return {
        "ok": bool(report.get("ok", False)) and not errors,
        "profile": spec["profile"],
        "dataset": spec["dataset"],
        "sequence": spec["sequence"],
        "evidence_kind": "faithful_cocolic_ground_truth_tracking",
        "gt_tum": rel(gt),
        "gt_sha256": sha256_file(gt),
        "current_tum": rel(current),
        "current_sha256": sha256_file(current),
        "thresholds": thresholds,
        "current_stats": current_stats,
        "baseline_poses": report["baseline_poses"],
        "current_poses": report["current_poses"],
        "matched_poses": report["matched_poses"],
        "coverage": report["coverage"],
        "translation": report["translation"],
        "path_length": report["path_length"],
        "trajectory_stats": report["trajectory_stats"],
        "errors": errors,
    }


def build_report() -> dict[str, Any]:
    scenes: dict[str, Any] = {}
    for name, spec in STABILITY_SCENES.items():
        scenes[name] = check_stability_scene(name, spec)
    for name, spec in GT_SCENES.items():
        scenes[name] = compare_gt_scene(name, spec)

    stability = [
        scene
        for scene in scenes.values()
        if scene["evidence_kind"] == "faithful_cocolic_invalid_reference_stability"
    ]
    gt_scenes = [
        scene
        for scene in scenes.values()
        if scene["evidence_kind"] == "faithful_cocolic_ground_truth_tracking"
    ]

    return {
        "schema": "gaussian_lic_faithful_blocker_tracking_report/v1",
        "ok": all(bool(scene.get("ok")) for scene in scenes.values()),
        "basis": "faithful_cocolic_port_blocker_resolution",
        "notes": (
            "Hash-locked faithful Coco-LIC port evidence for the three datasets "
            "that previously blocked paper completion through divergent "
            "reimplementation trajectories or invalid zero-path references."
        ),
        "metrics": {
            "scene_count": len(scenes),
            "stable_invalid_reference_scene_count": len(stability),
            "ground_truth_scene_count": len(gt_scenes),
            "min_stability_path_m": min(scene["current_stats"]["path_m"] for scene in stability),
            "max_stability_start_end_m": max(
                scene["current_stats"]["start_end_distance_m"] for scene in stability
            ),
            "max_stability_step_m": max(scene["current_stats"]["step_m"]["max"] for scene in stability),
            "max_gt_rmse_m": max(scene["translation"]["rmse_m"] for scene in gt_scenes),
            "min_gt_coverage": min(scene["coverage"] for scene in gt_scenes),
            "max_gt_path_drift": max(scene["path_length"]["relative_drift"] for scene in gt_scenes),
        },
        "scenes": scenes,
    }


def load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        data = json.load(handle)
    if not isinstance(data, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return data


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument(
        "--check",
        action="store_true",
        help="Compare the freshly computed report with --output instead of rewriting it.",
    )
    args = parser.parse_args()

    report = build_report()
    output = args.output if args.output.is_absolute() else ROOT / args.output

    if args.check:
        existing = load_json(output)
        if existing != report:
            print(f"Faithful blocker tracking report drifted: {output}")
            return 1
    else:
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )

    print(
        "Faithful blocker tracking "
        f"{'PASS' if report['ok'] else 'FAIL'}: "
        f"stable_scenes={report['metrics']['stable_invalid_reference_scene_count']} "
        f"gt_rmse={report['metrics']['max_gt_rmse_m']:.3f}m"
    )
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
