#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Audit whether the repository proves full paper/super-paper parity.

The strict parity matrix is a release gate. This audit is deliberately stricter:
it fails while required evidence is only a liveness/producer-chain gate, while a
native-tracking gate lacks reference trajectory parity, or while the repository
documentation still self-declares paper-level completion as pending.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace
from typing import Any

from trajectory_compare import (
    compute_report as compute_trajectory_report,
    load_tum,
    path_length,
    translation_tuple,
)


DEFAULT_MANIFEST = Path("docs/strict_parity_matrix.json")
README = Path("README.md")
PAPER_TRACKING_RMSE_MAX_M = 0.05
PAPER_TRACKING_COVERAGE_MIN = 0.99
PAPER_TRACKING_MAX_ERROR_M = 0.15


def load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        data = json.load(handle)
    if not isinstance(data, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return data


def threshold_value(entry: dict[str, Any], path: str, key: str) -> float | None:
    for threshold in entry.get("thresholds", ()):
        if not isinstance(threshold, dict) or threshold.get("path") != path:
            continue
        value = threshold.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            return None
        return float(value)
    return None


def threshold_paths(entry: dict[str, Any]) -> set[str]:
    return {
        str(threshold.get("path"))
        for threshold in entry.get("thresholds", ())
        if isinstance(threshold, dict)
    }


def is_paper_tracking_sponsor(entry: dict[str, Any]) -> bool:
    """Return true when an entry provides cm-grade reference tracking parity.

    The completion audit accepts liveness/probe evidence only when another
    required entry for the same profile/sequence proves the stricter paper-grade
    tracking claim. This keeps the audit from double-counting intentionally
    weaker producer-chain checks while still blocking unsponsored liveness-only
    evidence.
    """

    if not entry.get("required", True):
        return False
    parity = entry.get("paper_tracking_parity")
    if not isinstance(parity, dict) or not parity.get("reference_parity"):
        return False
    rmse_max = parity.get("rmse_max_m")
    coverage_min = parity.get("coverage_min")
    if isinstance(rmse_max, bool) or not isinstance(rmse_max, (int, float)):
        return False
    if isinstance(coverage_min, bool) or not isinstance(coverage_min, (int, float)):
        return False
    return (
        float(rmse_max) <= PAPER_TRACKING_RMSE_MAX_M
        and float(coverage_min) >= PAPER_TRACKING_COVERAGE_MIN
        and bool(parity.get("hash_locked", False))
    )


def tracking_sponsor_keys(entries: list[dict[str, Any]]) -> set[tuple[str, str]]:
    sponsors: set[tuple[str, str]] = set()
    for entry in entries:
        if not is_paper_tracking_sponsor(entry):
            continue
        profile = str(entry.get("profile", ""))
        sequence = str(entry.get("sequence", ""))
        if profile and sequence:
            sponsors.add((profile, sequence))
    return sponsors


def run_strict_matrix(root: Path) -> dict[str, Any]:
    completed = subprocess.run(
        ["./scripts/check_strict_parity_matrix.py"],
        cwd=root,
        check=False,
        capture_output=True,
        text=True,
        timeout=120,
    )
    return {
        "ok": completed.returncode == 0,
        "returncode": completed.returncode,
        "stdout": completed.stdout.strip(),
        "stderr": completed.stderr.strip(),
    }


def resolve_path(root: Path, value: Any) -> Path | None:
    if not isinstance(value, str) or not value:
        return None
    path = Path(value).expanduser()
    if not path.is_absolute():
        path = root / path
    return path


def summarize_tum_path(path: Path) -> dict[str, Any]:
    poses = load_tum(path)
    positions = [translation_tuple(pose) for pose in poses]
    steps: list[dict[str, Any]] = []
    for index, (prev_pose, curr_pose, prev_position, curr_position) in enumerate(
        zip(poses, poses[1:], positions, positions[1:]),
        start=1,
    ):
        distance_m = (
            (curr_position[0] - prev_position[0]) ** 2
            + (curr_position[1] - prev_position[1]) ** 2
            + (curr_position[2] - prev_position[2]) ** 2
        ) ** 0.5
        dt_s = curr_pose.stamp - prev_pose.stamp
        speed_mps = distance_m / dt_s if dt_s > 0.0 else None
        steps.append(
            {
                "index": index,
                "from_stamp": prev_pose.stamp,
                "to_stamp": curr_pose.stamp,
                "dt_s": dt_s,
                "distance_m": distance_m,
                "speed_mps": speed_mps,
                "from_xyz": prev_position,
                "to_xyz": curr_position,
            }
        )
    max_step = max(steps, key=lambda step: step["distance_m"], default=None)
    return {
        "poses": len(poses),
        "path_m": path_length(positions),
        "max_step_m": float(max_step["distance_m"]) if max_step else 0.0,
        "max_step": max_step,
        "steps": steps,
    }


def first_step_exceeding(summary: dict[str, Any], limit_m: float) -> dict[str, Any] | None:
    for step in summary.get("steps", ()):
        if float(step.get("distance_m", 0.0)) > limit_m:
            return step
    return None


def format_step_event(step: dict[str, Any] | None) -> str:
    if not step:
        return "no step detail"
    speed = step.get("speed_mps")
    speed_text = "inf" if speed is None else f"{float(speed):.3g}"
    return (
        f"idx={int(step.get('index', 0))}, "
        f"stamp={float(step.get('from_stamp', 0.0)):.9f}->{float(step.get('to_stamp', 0.0)):.9f}, "
        f"dt={float(step.get('dt_s', 0.0)):.3g}s, "
        f"step={float(step.get('distance_m', 0.0)):.3g}m, "
        f"speed={speed_text}m/s"
    )


def native_report_diagnostics(entry: dict[str, Any], root: Path) -> list[str]:
    """Return paper-completion blockers declared by the native report itself."""

    if entry.get("kind") != "native_tracking_report":
        return []
    report_path = resolve_path(root, entry.get("report"))
    if report_path is None:
        return ["native tracking evidence lacks report path"]
    if not report_path.is_file():
        return [f"native tracking report is missing: {report_path}"]
    try:
        report = load_json(report_path)
    except Exception as exc:  # noqa: BLE001
        return [f"native tracking report failed to load: {exc}"]

    reasons: list[str] = []
    if not bool(report.get("ok", True)):
        errors = report.get("errors")
        if isinstance(errors, list) and errors:
            detail = "; ".join(str(error) for error in errors[:5])
        else:
            detail = "ok=false without explicit errors"
        reasons.append(f"native tracking report self-reports failure: {detail}")

    summary = report.get("runtime_diagnostic_summary")
    if isinstance(summary, dict):
        rejection_values = []
        for key in (
            "output_pose_rejections_final",
            "output_pose_guard_log_rejections",
        ):
            value = summary.get(key)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                continue
            rejection_values.append(int(value))
        max_rejections = max(rejection_values, default=0)
        if max_rejections > 0:
            reasons.append(
                "native tracking output guard rejected poses before paper compare: "
                f"{max_rejections}"
            )
    return reasons


def candidate_tracking_diagnostics(entry: dict[str, Any], root: Path) -> list[str]:
    candidate = entry.get("paper_tracking_candidate")
    if not isinstance(candidate, dict):
        return []

    reasons: list[str] = []
    reference_path = resolve_path(root, candidate.get("reference_tum"))
    current_path = resolve_path(root, candidate.get("current_tum"))
    if reference_path is None:
        reasons.append("paper tracking candidate lacks reference_tum")
        return reasons
    if current_path is None:
        reasons.append("paper tracking candidate lacks current_tum")
        return reasons
    if not reference_path.is_file():
        reasons.append(f"paper tracking candidate reference is missing: {reference_path}")
        return reasons
    if not current_path.is_file():
        reasons.append(f"paper tracking candidate current trajectory is missing: {current_path}")
        return reasons

    try:
        reference_summary = summarize_tum_path(reference_path)
        current_summary = summarize_tum_path(current_path)
    except Exception as exc:  # noqa: BLE001
        reasons.append(f"paper tracking candidate TUM load failed: {exc}")
        return reasons

    min_reference_path_m = float(candidate.get("min_reference_path_m", 0.0))
    if reference_summary["path_m"] < min_reference_path_m:
        reasons.append(
            "paper tracking candidate reference is not a usable trajectory: "
            f"path {reference_summary['path_m']:.3g} m < {min_reference_path_m:.3g} m "
            f"({candidate.get('reference_kind', 'unknown')})"
        )

    max_current_step_m = float(candidate.get("max_current_step_m", 0.0))
    if max_current_step_m > 0.0 and current_summary["max_step_m"] > max_current_step_m:
        first_bad_step = first_step_exceeding(current_summary, max_current_step_m)
        max_step = current_summary.get("max_step")
        reasons.append(
            "paper tracking candidate current trajectory diverges before parity compare: "
            f"max step {current_summary['max_step_m']:.3g} m > {max_current_step_m:.3g} m; "
            f"first_bad=({format_step_event(first_bad_step)}); "
            f"max_bad=({format_step_event(max_step)})"
        )

    args = SimpleNamespace(
        baseline=str(reference_path),
        current=str(current_path),
        align=str(candidate.get("align", "yaw")),
        max_association_dt=float(candidate.get("max_association_dt", 0.2)),
        min_matches=int(candidate.get("min_matches", 10)),
        min_coverage=PAPER_TRACKING_COVERAGE_MIN,
        coverage_mode=str(candidate.get("coverage_mode", "overlap")),
        max_rmse_m=PAPER_TRACKING_RMSE_MAX_M,
        max_mean_m=0.03,
        max_error_m=PAPER_TRACKING_MAX_ERROR_M,
        max_path_drift=0.05,
        min_current_path_ratio=0.0,
        max_current_path_ratio=0.0,
        error_bin_count=0,
        min_error_bin_matches=0,
        max_error_bin_rmse_m=0.0,
        max_error_bin_mean_m=0.0,
        max_error_bin_bias_norm_m=0.0,
        time_offset_sweep_min=0.0,
        time_offset_sweep_max=0.0,
        time_offset_sweep_step=0.1,
        time_offset_sweep_min_matches=0,
    )
    try:
        report = compute_trajectory_report(args)
    except Exception as exc:  # noqa: BLE001
        reasons.append(f"paper tracking candidate comparison failed to run: {exc}")
        return reasons

    if not report.get("ok"):
        translation = report.get("translation", {})
        reasons.append(
            "paper tracking candidate comparison fails 5 cm parity: "
            f"matched={report.get('matched_poses', 0)}, "
            f"coverage={float(report.get('coverage', 0.0)):.2%}, "
            f"rmse={float(translation.get('rmse_m', 0.0)):.3g} m"
        )
    return reasons


def add_blocker(blockers: list[dict[str, Any]], entry: dict[str, Any], reason: str) -> None:
    blockers.append(
        {
            "id": entry.get("id", "<unknown>"),
            "kind": entry.get("kind", "<unknown>"),
            "profile": entry.get("profile", ""),
            "sequence": entry.get("sequence", ""),
            "reason": reason,
        }
    )


def audit_entry(
    entry: dict[str, Any],
    blockers: list[dict[str, Any]],
    sponsors: set[tuple[str, str]],
    root: Path,
) -> None:
    if not entry.get("required", True):
        return
    kind = entry.get("kind")
    notes = str(entry.get("notes", "")).lower()
    entry_id = str(entry.get("id", "")).lower()
    sponsored = (str(entry.get("profile", "")), str(entry.get("sequence", ""))) in sponsors
    for reason in native_report_diagnostics(entry, root):
        add_blocker(blockers, entry, reason)
    candidate_reasons = candidate_tracking_diagnostics(entry, root)
    if candidate_reasons and not sponsored:
        for reason in candidate_reasons:
            add_blocker(blockers, entry, reason)
        return

    if (
        "liveness" in notes
        or "producer-chain" in notes
        or "liveness" in entry_id
    ) and not sponsored:
        add_blocker(blockers, entry, "required evidence is only a liveness/producer-chain gate")

    if kind == "pending":
        add_blocker(blockers, entry, "required evidence is pending")
        return

    if kind != "native_tracking_report":
        return

    paths = threshold_paths(entry)
    has_reference_parity = bool(entry.get("requires_reference_parity"))
    has_rmse_gate = "trajectory_compare.translation.rmse_m" in paths
    has_coverage_gate = "trajectory_compare.coverage" in paths
    if not has_reference_parity and not sponsored:
        add_blocker(blockers, entry, "native tracking evidence lacks reference trajectory parity")
    if not has_rmse_gate and not sponsored:
        add_blocker(blockers, entry, "native tracking evidence lacks trajectory RMSE gate")
    if not has_coverage_gate and not sponsored:
        add_blocker(blockers, entry, "native tracking evidence lacks trajectory coverage gate")

    rmse_max = threshold_value(entry, "trajectory_compare.translation.rmse_m", "max")
    if rmse_max is not None and rmse_max > PAPER_TRACKING_RMSE_MAX_M and not sponsored:
        add_blocker(
            blockers,
            entry,
            f"native tracking RMSE threshold {rmse_max:g} m is looser than "
            f"{PAPER_TRACKING_RMSE_MAX_M:g} m paper-grade cm parity",
        )


def audit_readme(root: Path, blockers: list[dict[str, Any]]) -> None:
    readme = root / README
    if not readme.exists():
        blockers.append(
            {
                "id": "documentation/readme",
                "kind": "documentation",
                "profile": "",
                "sequence": "",
                "reason": "README.md is missing",
            }
        )
        return
    text = readme.read_text(encoding="utf-8")
    tokens = (
        "not a claim of universal super-paper performance",
        "RMSE-gated continuous-time native tracker parity",
        "full RMSE-gated native Coco-LIC2 tracking BA remains",
        "not a 100% paper/super-paper parity claim",
    )
    for token in tokens:
        if token in text:
            blockers.append(
                {
                    "id": "documentation/readme",
                    "kind": "documentation",
                    "profile": "",
                    "sequence": "",
                    "reason": f"README still self-declares incomplete paper parity: {token}",
                }
            )


def render_markdown(report: dict[str, Any]) -> str:
    lines = [
        "# Paper Completion Audit",
        "",
        f"Status: {'PASS' if report['ok'] else 'INCOMPLETE'}",
        f"Strict matrix: {'PASS' if report['strict_matrix']['ok'] else 'FAIL'}",
        f"Blocking items: {len(report['blockers'])}",
        "",
        "| Evidence | Kind | Reason |",
        "| --- | --- | --- |",
    ]
    for blocker in report["blockers"]:
        lines.append(f"| `{blocker['id']}` | {blocker['kind']} | {blocker['reason']} |")
    if not report["blockers"]:
        lines.append("| all | - | no blockers |")
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--repo-root", type=Path, default=Path("."))
    parser.add_argument("--output", type=Path)
    parser.add_argument("--markdown", type=Path)
    parser.add_argument(
        "--allow-incomplete",
        action="store_true",
        help="Write/print the audit but return success while blockers remain.",
    )
    args = parser.parse_args()

    root = args.repo_root.resolve()
    manifest_path = args.manifest if args.manifest.is_absolute() else root / args.manifest
    manifest = load_json(manifest_path)
    strict_matrix = run_strict_matrix(root)
    blockers: list[dict[str, Any]] = []
    if not strict_matrix["ok"]:
        blockers.append(
            {
                "id": "strict_parity_matrix",
                "kind": "script_check",
                "profile": "",
                "sequence": "",
                "reason": "strict parity matrix does not pass",
            }
        )
    entries = [entry for entry in manifest.get("evidence", ()) if isinstance(entry, dict)]
    sponsors = tracking_sponsor_keys(entries)
    for entry in entries:
        audit_entry(entry, blockers, sponsors, root)
    audit_readme(root, blockers)

    report = {
        "schema": "gaussian_lic_paper_completion_audit/v1",
        "ok": not blockers,
        "strict_matrix": strict_matrix,
        "paper_tracking_rmse_max_m": PAPER_TRACKING_RMSE_MAX_M,
        "paper_tracking_coverage_min": PAPER_TRACKING_COVERAGE_MIN,
        "paper_tracking_sponsors": [
            {"profile": profile, "sequence": sequence}
            for profile, sequence in sorted(sponsors)
        ],
        "blockers": blockers,
    }
    if args.output:
        output = args.output if args.output.is_absolute() else root / args.output
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    if args.markdown:
        markdown = args.markdown if args.markdown.is_absolute() else root / args.markdown
        markdown.parent.mkdir(parents=True, exist_ok=True)
        markdown.write_text(render_markdown(report), encoding="utf-8")

    print(
        "paper completion audit "
        f"{'PASS' if report['ok'] else 'INCOMPLETE'}: blockers={len(blockers)}"
    )
    for blocker in blockers:
        print(f"- {blocker['id']}: {blocker['reason']}", file=sys.stderr)
    if report["ok"] or args.allow_incomplete:
        return 0
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
