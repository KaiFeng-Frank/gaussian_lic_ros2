#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Archive a passing strict reproduction report into the evidence matrix."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any


DEFAULT_MANIFEST = Path("docs/strict_parity_matrix.json")


def load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        data = json.load(handle)
    if not isinstance(data, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return data


def write_json(path: Path, data: dict[str, Any]) -> None:
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def lookup(data: dict[str, Any], dotted: str) -> Any:
    current: Any = data
    for part in dotted.split("."):
        if not isinstance(current, dict) or part not in current:
            raise KeyError(dotted)
        current = current[part]
    return current


def finite_number(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"{name} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def relative_path(path: Path, root: Path) -> str:
    resolved = path.expanduser().resolve()
    try:
        return str(resolved.relative_to(root.resolve()))
    except ValueError:
        return str(resolved)


def dataset_name(profile: str) -> str:
    mapping = {
        "fastlivo": "FAST-LIVO",
        "fastlivo2": "FAST-LIVO2",
        "m2dgr": "M2DGR",
        "mcd": "MCD",
        "r3live": "R3LIVE",
    }
    return mapping.get(profile, profile)


def make_entry(args: argparse.Namespace, report: dict[str, Any], repo_root: Path) -> dict[str, Any]:
    if not report.get("ok"):
        raise ValueError("refusing to archive reproduction report with ok=false")
    if args.strict and not report.get("strict"):
        raise ValueError("refusing to archive non-strict report for a strict evidence entry")

    matched = int(finite_number(lookup(report, "trajectory.matched_poses"), "trajectory.matched_poses"))
    if args.min_matched_poses is None:
        min_matched = max(64, int(matched * args.matched_pose_fraction))
    else:
        min_matched = int(args.min_matched_poses)

    thresholds: list[dict[str, Any]] = [
        {"path": "trajectory.matched_poses", "min": min_matched},
        {"path": "trajectory.coverage", "min": args.min_coverage},
        {
            "path": "novel_view_quality.render_pairs.summary.failed_pair_ratio",
            "max": args.max_failed_pair_ratio,
        },
        {"path": "novel_view_quality.render_pairs.summary.mean_ssim", "min": args.min_mean_ssim},
        {"path": "point_cloud.nearest.bidirectional.mean_m", "max": args.max_nearest_mean_m},
    ]

    return {
        "id": f"{args.profile}/{args.sequence}/mapper_contract_cuda_strict",
        "kind": "reproduction_report",
        "profile": args.profile,
        "dataset": args.dataset or dataset_name(args.profile),
        "sequence": args.sequence,
        "required": True,
        "report": relative_path(args.report, repo_root),
        "strict": args.strict,
        "min_render_pairs": args.min_render_pairs,
        "gates": ["baseline_manifest", "metrics", "trajectory", "point_cloud"],
        "thresholds": thresholds,
    }


def upsert_entry(entries: list[dict[str, Any]], entry: dict[str, Any]) -> None:
    entry_id = entry["id"]
    for index, existing in enumerate(entries):
        if existing.get("id") == entry_id:
            entries[index] = entry
            return

    pending_id = f"{entry['profile']}/full_sequence_strict"
    for index, existing in enumerate(entries):
        if existing.get("id") == pending_id:
            entries.insert(index, entry)
            return
    entries.append(entry)


def mark_pending_resolved(entries: list[dict[str, Any]], profile: str, sequence: str, reason: str | None) -> None:
    pending_id = f"{profile}/full_sequence_strict"
    for entry in entries:
        if entry.get("id") != pending_id:
            continue
        entry["required"] = False
        entry["reason"] = reason or (
            f"{profile} {sequence} mapper-contract strict artifacts now pass."
        )
        return


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--repo-root", type=Path, default=Path("."))
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--profile", required=True)
    parser.add_argument("--sequence", required=True)
    parser.add_argument("--dataset", default="")
    parser.add_argument("--min-render-pairs", type=int, default=64)
    parser.add_argument("--min-matched-poses", type=int)
    parser.add_argument("--matched-pose-fraction", type=float, default=0.8)
    parser.add_argument("--min-coverage", type=float, default=0.8)
    parser.add_argument("--max-failed-pair-ratio", type=float, default=0.1)
    parser.add_argument("--min-mean-ssim", type=float, default=0.75)
    parser.add_argument("--max-nearest-mean-m", type=float, default=0.05)
    parser.add_argument("--strict", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--resolve-pending", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--pending-reason")
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    repo_root = args.repo_root.expanduser().resolve()
    manifest_path = args.manifest if args.manifest.is_absolute() else repo_root / args.manifest
    report_path = args.report if args.report.is_absolute() else repo_root / args.report
    args.report = report_path

    manifest = load_json(manifest_path)
    entries = manifest.get("evidence")
    if not isinstance(entries, list):
        raise ValueError(f"{manifest_path} must contain an evidence list")

    report = load_json(report_path)
    entry = make_entry(args, report, repo_root)
    upsert_entry(entries, entry)
    if args.resolve_pending:
        mark_pending_resolved(entries, args.profile, args.sequence, args.pending_reason)

    if args.dry_run:
        print(json.dumps(entry, indent=2, ensure_ascii=False))
    else:
        write_json(manifest_path, manifest)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
