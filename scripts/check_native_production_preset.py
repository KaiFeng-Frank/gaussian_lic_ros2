#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Check that the native tracker production preset is evidence-backed.

This is intentionally a static guard: it prevents README/ROADMAP/script drift
from silently promoting a diagnostic run as the production Coco-LIC2 preset.
When local full-run artifacts are present, it also verifies the accepted and
rejected reports numerically.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MANIFEST = ROOT / "docs" / "native_production_preset.json"


def load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        data = json.load(handle)
    if not isinstance(data, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return data


def number_at_path(data: dict[str, Any], dotted_path: str) -> float:
    current: Any = data
    for part in dotted_path.split("."):
        if not isinstance(current, dict) or part not in current:
            raise KeyError(dotted_path)
        current = current[part]
    if isinstance(current, bool) or not isinstance(current, (int, float)):
        raise TypeError(f"{dotted_path} must be numeric")
    value = float(current)
    if not math.isfinite(value):
        raise ValueError(f"{dotted_path} must be finite")
    return value


def approx_equal(lhs: float, rhs: float, *, tolerance: float = 1e-9) -> bool:
    return abs(lhs - rhs) <= tolerance


def require_assignment(script: str, name: str, value: Any, errors: list[str]) -> None:
    value_text = str(value).lower() if isinstance(value, bool) else str(value)
    pattern = rf"(^|\n)[ \t]*{re.escape(name)}={re.escape(value_text)}([ \t]*($|\n))"
    if not re.search(pattern, script):
        errors.append(f"missing script assignment {name}={value_text}")


def require_snippet(text: str, snippet: str, label: str, errors: list[str]) -> None:
    if snippet not in text:
        errors.append(f"{label} is missing required snippet: {snippet}")


def check_script_contract(manifest: dict[str, Any], script: str, errors: list[str]) -> None:
    preset = manifest["production_preset"]
    expected_assignments = {
        "PLAYBACK_RATE": preset["playback_rate"],
        "TRACKING_MAX_POSE_STEP_M": preset["tracking_max_pose_step_m"],
        "POST_BA_STEP_GUARD_PRE_BA_AGREEMENT_MAX_POSE_STEP_M": (
            preset["post_ba_step_guard_pre_ba_agreement_max_pose_step_m"]
        ),
        "POST_BA_STEP_GUARD_PRE_BA_AGREEMENT_MIN_COSINE": (
            preset["post_ba_step_guard_pre_ba_agreement_min_cosine"]
        ),
        "POST_BA_STEP_GUARD_PRE_BA_AGREEMENT_MAX_DELTA_M": (
            preset["post_ba_step_guard_pre_ba_agreement_max_delta_m"]
        ),
        "POST_BA_STEP_GUARD_CONFIDENCE_MAX_POSE_STEP_M": (
            preset["post_ba_step_guard_confidence_max_pose_step_m"]
        ),
        "POST_BA_STEP_GUARD_CONFIDENCE_WARMUP_MARGINALIZATIONS": (
            preset["post_ba_step_guard_confidence_warmup_marginalizations"]
        ),
        "SLIDING_WINDOW_RELATIVE_TRANSLATION_WEIGHT": (
            preset["sliding_window_relative_translation_weight"]
        ),
        "SLIDING_WINDOW_RELATIVE_ROTATION_WEIGHT": (
            preset["sliding_window_relative_rotation_weight"]
        ),
        "SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_WEIGHT": (
            preset["sliding_window_multihop_relative_translation_weight"]
        ),
        "SLIDING_WINDOW_MULTIHOP_RELATIVE_TRANSLATION_MAX_FACTORS": (
            preset["sliding_window_multihop_relative_translation_max_factors"]
        ),
        "SLIDING_WINDOW_MULTIHOP_RELATIVE_ROTATION_WEIGHT": (
            preset["sliding_window_multihop_relative_rotation_weight"]
        ),
        "VISUAL_ALIGNMENT_WINDOW_WEIGHT": preset["visual_alignment_window_weight"],
        "SE3_PHOTOMETRIC_WINDOW_WEIGHT": preset["se3_photometric_window_weight"],
        "LIDAR_WINDOW_PLANE_FACTOR_WEIGHT": preset["lidar_window_plane_factor_weight"],
        "SLIDING_WINDOW_MAX_FEEDBACK_GYRO_BIAS_STEP": (
            preset["sliding_window_max_feedback_gyro_bias_step"]
        ),
        "SLIDING_WINDOW_MAX_FEEDBACK_ACCEL_BIAS_STEP": (
            preset["sliding_window_max_feedback_accel_bias_step"]
        ),
        "MAPPER_FEEDBACK_RENDER_MODE": preset["mapper_feedback_render_mode"],
        "MAPPER_FEEDBACK_POINTCLOUD_COORDINATES": (
            preset["mapper_feedback_pointcloud_coordinates"]
        ),
        "MAPPER_FEEDBACK_MAX_DEPTH": preset["mapper_feedback_max_depth"],
    }
    for name, value in expected_assignments.items():
        require_assignment(script, name, value, errors)

    require_snippet(
        script,
        "--post-ba-step-guard-confidence-warmup-marginalizations",
        "run_native_tracking_bag_report.sh",
        errors,
    )


def check_docs(manifest: dict[str, Any], readme: str, roadmap: str, errors: list[str]) -> None:
    accepted = manifest["accepted_evidence"]
    production_report = accepted["report"]
    for label, text in (("README.md", readme), ("docs/ROADMAP.md", roadmap)):
        require_snippet(text, production_report, label, errors)
        require_snippet(text, "1.346", label, errors)
        require_snippet(text, "0.045", label, errors)
        for forbidden in manifest.get("forbidden_readme_snippets", []):
            if forbidden in text:
                errors.append(f"{label} contains stale native-preset snippet: {forbidden}")


def verify_report_metrics(
    root: Path,
    report_path_text: str,
    expected: dict[str, Any],
    errors: list[str],
    *,
    require_local_evidence: bool,
    label: str,
) -> bool:
    report_path = root / report_path_text
    if not report_path.is_file():
        if require_local_evidence:
            errors.append(f"{label} report missing: {report_path_text}")
        return False

    report = load_json(report_path)
    checks = {
        "matched_poses": "trajectory_compare.matched_poses",
        "coverage": "trajectory_compare.coverage",
        "translation_rmse_m": "trajectory_compare.translation.rmse_m",
        "translation_mean_m": "trajectory_compare.translation.mean_m",
        "translation_max_m": "trajectory_compare.translation.max_m",
        "path_drift": "trajectory_compare.path_length.relative_drift",
        "total_visual_factors": "metrics.tracking_status.last.sliding_window_total_visual_factors",
        "total_se3_photometric_factors": (
            "metrics.tracking_status.last.sliding_window_total_se3_photometric_factors"
        ),
        "dense_prior_rank": "metrics.tracking_status.last.sliding_window_dense_prior_rank",
        "dense_prior_cols": "metrics.tracking_status.last.sliding_window_dense_prior_cols",
    }
    for expected_key, report_key in checks.items():
        if expected_key not in expected:
            continue
        actual = number_at_path(report, report_key)
        wanted = float(expected[expected_key])
        if not approx_equal(actual, wanted):
            errors.append(
                f"{label} {report_key}={actual:g} does not match manifest {wanted:g}"
            )
    return True


def check_local_evidence(
    manifest: dict[str, Any], root: Path, errors: list[str], *, require_local_evidence: bool
) -> None:
    accepted = manifest["accepted_evidence"]
    loaded = verify_report_metrics(
        root,
        str(accepted["report"]),
        accepted,
        errors,
        require_local_evidence=require_local_evidence,
        label="accepted evidence",
    )
    if not loaded:
        return

    production_rmse = float(accepted["translation_rmse_m"])
    for rejected in manifest.get("rejected_evidence", []):
        report_path = root / str(rejected["report"])
        if not report_path.is_file():
            if require_local_evidence:
                errors.append(f"rejected evidence report missing: {rejected['report']}")
            continue
        report = load_json(report_path)
        rmse = number_at_path(report, "trajectory_compare.translation.rmse_m")
        expected_rmse = float(rejected["translation_rmse_m"])
        if not approx_equal(rmse, expected_rmse):
            errors.append(
                f"{rejected['id']} RMSE {rmse:g} does not match manifest {expected_rmse:g}"
            )
        if rmse <= production_rmse:
            errors.append(
                f"{rejected['id']} is marked rejected but RMSE {rmse:g} <= production "
                f"{production_rmse:g}"
            )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument(
        "--require-local-evidence",
        action="store_true",
        help="fail if the full local native report artifacts referenced by the manifest are missing",
    )
    args = parser.parse_args()

    manifest_path = args.manifest if args.manifest.is_absolute() else ROOT / args.manifest
    manifest = load_json(manifest_path)
    errors: list[str] = []

    if manifest.get("schema") != "gaussian_lic_native_production_preset/v1":
        errors.append("unexpected native production preset schema")

    script = (ROOT / manifest["production_preset"]["script"]).read_text(encoding="utf-8")
    readme = (ROOT / "README.md").read_text(encoding="utf-8")
    roadmap = (ROOT / "docs" / "ROADMAP.md").read_text(encoding="utf-8")

    check_script_contract(manifest, script, errors)
    check_docs(manifest, readme, roadmap, errors)
    check_local_evidence(
        manifest,
        ROOT,
        errors,
        require_local_evidence=args.require_local_evidence,
    )

    if errors:
        for error in errors:
            print(f"[native-preset] {error}", file=sys.stderr)
        return 1
    print("[native-preset] OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
