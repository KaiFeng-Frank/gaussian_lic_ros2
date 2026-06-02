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
from typing import Any


DEFAULT_MANIFEST = Path("docs/strict_parity_matrix.json")
README = Path("README.md")
PAPER_TRACKING_RMSE_MAX_M = 0.05
PAPER_TRACKING_COVERAGE_MIN = 0.99


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
) -> None:
    if not entry.get("required", True):
        return
    kind = entry.get("kind")
    notes = str(entry.get("notes", "")).lower()
    entry_id = str(entry.get("id", "")).lower()
    sponsored = (str(entry.get("profile", "")), str(entry.get("sequence", ""))) in sponsors

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
        audit_entry(entry, blockers, sponsors)
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
