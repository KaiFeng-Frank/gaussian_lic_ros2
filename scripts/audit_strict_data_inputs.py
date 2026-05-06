#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Audit local dataset inputs required for full strict Gaussian-LIC2 parity."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
from typing import Any


DEFAULT_DATA_ROOT = Path("/home/frank/data")
DEFAULT_MATRIX = Path("docs/strict_parity_matrix.json")


CATALOG: dict[str, dict[str, Any]] = {
    "fastlivo": {
        "dataset": "FAST-LIVO",
        "roots": ("fast_livo", "fastlivo"),
        "required_sequences": ("LiDAR_Degenerate", "Visual_Challenge", "hku1", "hku2"),
        "source_urls": (
            "https://github.com/hku-mars/FAST-LIVO",
            "https://connecthkuhk-my.sharepoint.com/personal/zhengcr_connect_hku_hk/_layouts/15/onedrive.aspx?id=%2Fpersonal%2Fzhengcr%5Fconnect%5Fhku%5Fhk%2FDocuments%2FFAST%2DLIVO%2DDatasets&ga=1",
        ),
    },
    "fastlivo2": {
        "dataset": "FAST-LIVO2",
        "roots": ("fast_livo", "fastlivo2"),
        "required_sequences": ("CBD_Building_01", "Retail_Street"),
        "source_urls": (
            "https://github.com/hku-mars/FAST-LIVO2",
            "https://drive.google.com/drive/folders/1bf5LQ8iSxw-fD8BObZmouw7lRxNacfrA?usp=drive_link",
        ),
    },
    "m2dgr": {
        "dataset": "M2DGR",
        "roots": ("m2dgr", "M2DGR"),
        "required_sequences": ("room_01", "room_02", "room_03"),
        "source_urls": ("https://github.com/SJTU-ViSYS/M2DGR",),
    },
    "mcd": {
        "dataset": "MCD",
        "roots": ("mcd", "MCD"),
        "required_sequences": ("ntu_day_01",),
        "source_urls": ("https://mcdviral.github.io/Download.html",),
    },
    "r3live": {
        "dataset": "R3LIVE",
        "roots": ("r3live", "R3LIVE"),
        "required_sequences": ("hku_park_00",),
        "source_urls": (
            "https://github.com/ziv-lin/r3live_dataset",
            "https://github.com/hku-mars/r3live",
        ),
    },
}


RAW_PATTERNS = ("*.bag", "*.bag.bz2")
REFERENCE_PATTERNS = ("*.tum", "*.txt", "*.csv")


def utc_now() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def normalize(value: str) -> str:
    return re.sub(r"[^a-z0-9]+", "", value.lower())


def sequence_matches(path: Path, sequence: str) -> bool:
    return normalize(sequence) in normalize(str(path))


def path_entry(path: Path) -> dict[str, Any]:
    entry: dict[str, Any] = {
        "path": str(path),
        "exists": path.exists(),
        "is_file": path.is_file(),
        "is_dir": path.is_dir(),
    }
    if path.is_file():
        entry["bytes"] = path.stat().st_size
    return entry


def limited_rglob(root: Path, patterns: tuple[str, ...], limit: int) -> list[Path]:
    if not root.is_dir():
        return []
    matches: list[Path] = []
    for pattern in patterns:
        for path in sorted(root.rglob(pattern)):
            matches.append(path)
            if len(matches) >= limit:
                return matches
    return matches


def rosbag2_dirs(root: Path, limit: int) -> list[Path]:
    if not root.is_dir():
        return []
    dirs: list[Path] = []
    for metadata in sorted(root.rglob("metadata.yaml")):
        bag_dir = metadata.parent
        if any(item.suffix in (".db3", ".mcap") for item in bag_dir.iterdir() if item.is_file()):
            dirs.append(bag_dir)
            if len(dirs) >= limit:
                break
    return dirs


def artifact_dirs(root: Path, limit: int) -> list[Path]:
    if not root.is_dir():
        return []
    markers = (
        "baseline_manifest.json",
        "metrics.json",
        "reproduction_report.json",
        "reproduction_report_strict.json",
        "native_tracking_report.json",
    )
    dirs: list[Path] = []
    seen: set[Path] = set()
    for marker in markers:
        for path in sorted(root.rglob(marker)):
            try:
                relative = path.relative_to(root)
            except ValueError:
                continue
            artifact_dir = root / relative.parts[0] if relative.parts else path.parent
            if artifact_dir not in seen:
                dirs.append(artifact_dir)
                seen.add(artifact_dir)
            if len(dirs) >= limit:
                return dirs
    return dirs


def reference_files(root: Path, sequence: str, limit: int) -> list[Path]:
    candidates: list[Path] = []
    for path in limited_rglob(root, REFERENCE_PATTERNS, limit * 8):
        text = normalize(str(path))
        if not sequence_matches(path, sequence):
            continue
        if any(token in text for token in ("groundtruth", "gt", "traj", "trajectory", "pose", "odom")):
            candidates.append(path)
            if len(candidates) >= limit:
                break
    return candidates


def candidate_roots(data_root: Path, profile: str) -> list[Path]:
    roots = []
    for item in CATALOG[profile]["roots"]:
        roots.append((data_root / str(item)).expanduser().resolve())
    return roots


def find_matches(paths: list[Path], sequence: str, limit: int) -> list[dict[str, Any]]:
    return [path_entry(path) for path in paths if sequence_matches(path, sequence)][:limit]


def audit_sequence(
    repo_root: Path,
    data_root: Path,
    profile: str,
    sequence: str,
    max_candidates: int,
) -> dict[str, Any]:
    roots = candidate_roots(data_root, profile)
    raw_paths: list[Path] = []
    frontend_paths: list[Path] = []
    references: list[Path] = []
    for root in roots:
        raw_paths.extend(limited_rglob(root, RAW_PATTERNS, max_candidates * 8))
        frontend_paths.extend(rosbag2_dirs(root, max_candidates * 8))
        references.extend(reference_files(root, sequence, max_candidates))

    baseline_root = repo_root / "baseline" / profile
    results_root = repo_root / "results" / profile
    baselines = artifact_dirs(baseline_root, max_candidates * 8)
    currents = artifact_dirs(results_root, max_candidates * 16)

    raw = find_matches(raw_paths, sequence, max_candidates)
    frontend = [
        path_entry(path)
        for path in frontend_paths
        if sequence_matches(path, sequence) and "frontendraw" in normalize(path.name)
    ][:max_candidates]
    baseline = find_matches(baselines, sequence, max_candidates)
    current = find_matches(currents, sequence, max_candidates)
    reference = [path_entry(path) for path in references[:max_candidates]]

    missing = []
    if not raw:
        missing.append("raw_ros1_bag")
    if not frontend:
        missing.append("frontend_raw_rosbag2")
    if not baseline:
        missing.append("ros1_baseline_artifacts")
    if not current:
        missing.append("ros2_current_artifacts")
    if not reference:
        missing.append("native_reference_trajectory")

    return {
        "sequence": sequence,
        "roots": [str(root) for root in roots],
        "raw_ros1_bag": raw,
        "frontend_raw_rosbag2": frontend,
        "ros1_baseline_artifacts": baseline,
        "ros2_current_artifacts": current,
        "native_reference_trajectory": reference,
        "ok": not missing,
        "missing": missing,
    }


def load_matrix_status(repo_root: Path, matrix_path: Path) -> dict[str, Any]:
    path = matrix_path if matrix_path.is_absolute() else repo_root / matrix_path
    if not path.is_file():
        return {"path": str(path), "exists": False, "required_count": 0, "required_non_pending_count": 0}
    with path.open("r", encoding="utf-8") as handle:
        matrix = json.load(handle)
    evidence = matrix.get("evidence", [])
    required = [item for item in evidence if item.get("required")]
    non_pending = [item for item in required if item.get("kind") != "pending"]
    pending = [item for item in required if item.get("kind") == "pending"]
    return {
        "path": str(path),
        "exists": True,
        "required_count": len(required),
        "required_non_pending_count": len(non_pending),
        "pending_required_count": len(pending),
        "required_non_pending_fraction": len(non_pending) / len(required) if required else 0.0,
        "pending_required_ids": [str(item.get("id", "")) for item in pending],
    }


def disk_status(path: Path) -> dict[str, Any]:
    target = path if path.exists() else path.parent
    usage = shutil.disk_usage(target)
    return {
        "path": str(target),
        "total_bytes": usage.total,
        "used_bytes": usage.used,
        "free_bytes": usage.free,
        "free_gb": usage.free / 1024 / 1024 / 1024,
    }


def du_bytes(path: Path) -> int:
    try:
        output = subprocess.check_output(["du", "-sb", str(path)], text=True)
        return int(output.split()[0])
    except (OSError, subprocess.CalledProcessError, ValueError, IndexError):
        return 0


def evidence_keep_dirs(repo_root: Path, matrix_path: Path) -> set[Path]:
    path = matrix_path if matrix_path.is_absolute() else repo_root / matrix_path
    keep: set[Path] = set()
    if not path.is_file():
        return keep
    try:
        with path.open("r", encoding="utf-8") as handle:
            matrix = json.load(handle)
    except (OSError, json.JSONDecodeError):
        return keep
    for item in matrix.get("evidence", []):
        report = item.get("report")
        if not report:
            continue
        report_path = repo_root / str(report)
        try:
            relative_parts = report_path.relative_to(repo_root).parts
        except ValueError:
            continue
        if len(relative_parts) >= 3 and relative_parts[0] in ("results", "baseline"):
            keep.add(repo_root / relative_parts[0] / relative_parts[1] / relative_parts[2])
        else:
            keep.add(report_path.parent)
    return keep


def cleanup_candidates(repo_root: Path, matrix_path: Path, limit: int) -> list[dict[str, Any]]:
    keep = evidence_keep_dirs(repo_root, matrix_path)
    candidates: list[dict[str, Any]] = []
    root = repo_root / "results"
    if not root.is_dir():
        return candidates
    for profile_dir in sorted(item for item in root.iterdir() if item.is_dir()):
        for run_dir in sorted(item for item in profile_dir.iterdir() if item.is_dir()):
            if run_dir in keep:
                continue
            size = du_bytes(run_dir)
            if size > 0:
                candidates.append({"path": str(run_dir), "bytes": size})
    candidates.sort(key=lambda item: int(item["bytes"]), reverse=True)
    return candidates[:limit]


def build_report(args: argparse.Namespace) -> dict[str, Any]:
    repo_root = args.repo_root.expanduser().resolve()
    data_root = args.data_root.expanduser().resolve()
    profiles: dict[str, Any] = {}
    for profile in args.profiles:
        catalog = CATALOG[profile]
        sequences = [
            audit_sequence(repo_root, data_root, profile, sequence, args.max_candidates)
            for sequence in catalog["required_sequences"]
        ]
        missing_items = sorted({item for seq in sequences for item in seq["missing"]})
        profiles[profile] = {
            "dataset": catalog["dataset"],
            "source_urls": catalog["source_urls"],
            "required_sequences": sequences,
            "ok": all(seq["ok"] for seq in sequences),
            "missing": missing_items,
        }

    disk = disk_status(data_root)
    matrix = load_matrix_status(repo_root, args.matrix)
    report = {
        "schema": "gaussian_lic_strict_data_inputs/v1",
        "generated_at": utc_now(),
        "repo_root": str(repo_root),
        "data_root": str(data_root),
        "disk": disk,
        "min_free_gb": args.min_free_gb,
        "disk_ok": disk["free_gb"] >= args.min_free_gb,
        "strict_parity_matrix": matrix,
        "profiles": profiles,
    }
    report["ok"] = report["disk_ok"] and all(profile["ok"] for profile in profiles.values())
    if args.cleanup_candidates > 0:
        report["cleanup_candidates"] = cleanup_candidates(repo_root, args.matrix, args.cleanup_candidates)
    return report


def render_markdown(report: dict[str, Any]) -> str:
    matrix = report["strict_parity_matrix"]
    lines = [
        "# Strict Data Input Audit",
        "",
        f"Generated: `{report['generated_at']}`",
        f"Data root: `{report['data_root']}`",
        f"Disk free: `{report['disk']['free_gb']:.2f} GiB` "
        f"(minimum requested `{report['min_free_gb']:.2f} GiB`) - "
        f"{'PASS' if report['disk_ok'] else 'FAIL'}",
        f"Strict evidence currently materialized: "
        f"`{matrix.get('required_non_pending_count', 0)}/{matrix.get('required_count', 0)}`",
        "",
        "| Profile | Dataset | Status | Missing Inputs |",
        "| --- | --- | --- | --- |",
    ]
    for profile, payload in report["profiles"].items():
        missing = ", ".join(payload["missing"]) if payload["missing"] else "none"
        status = "PASS" if payload["ok"] else "MISSING"
        lines.append(f"| `{profile}` | {payload['dataset']} | {status} | {missing} |")

    lines.extend(["", "## Sequence Detail", ""])
    for profile, payload in report["profiles"].items():
        lines.append(f"### {profile}")
        for sequence in payload["required_sequences"]:
            missing = ", ".join(sequence["missing"]) if sequence["missing"] else "none"
            lines.append(f"- `{sequence['sequence']}`: {'PASS' if sequence['ok'] else 'MISSING'}; missing: {missing}")
            for key in (
                "raw_ros1_bag",
                "frontend_raw_rosbag2",
                "ros1_baseline_artifacts",
                "ros2_current_artifacts",
                "native_reference_trajectory",
            ):
                paths = [item["path"] for item in sequence[key]]
                if paths:
                    lines.append(f"  - {key}: `{paths[0]}`")
        lines.append("")

    cleanup = report.get("cleanup_candidates", [])
    if cleanup:
        lines.extend(["## Largest Non-Matrix Artifact Candidates", ""])
        for item in cleanup:
            gib = int(item["bytes"]) / 1024 / 1024 / 1024
            lines.append(f"- `{item['path']}` - {gib:.2f} GiB")
        lines.append("")

    lines.extend(["## Source URLs", ""])
    for profile, payload in report["profiles"].items():
        lines.append(f"- `{profile}`: " + ", ".join(payload["source_urls"]))
    return "\n".join(lines).rstrip() + "\n"


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=Path("."))
    parser.add_argument("--data-root", type=Path, default=DEFAULT_DATA_ROOT)
    parser.add_argument("--matrix", type=Path, default=DEFAULT_MATRIX)
    parser.add_argument("--profiles", nargs="+", choices=tuple(sorted(CATALOG)), default=tuple(sorted(CATALOG)))
    parser.add_argument("--max-candidates", type=int, default=8)
    parser.add_argument("--min-free-gb", type=float, default=100.0)
    parser.add_argument("--cleanup-candidates", type=int, default=12)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--markdown", type=Path)
    parser.add_argument("--strict", action="store_true", help="Exit nonzero when any data input is missing.")
    args = parser.parse_args(argv)

    report = build_report(args)
    if args.output:
        write_text(args.output, json.dumps(report, indent=2, sort_keys=True) + "\n")
    if args.markdown:
        write_text(args.markdown, render_markdown(report))
    if not args.output and not args.markdown:
        print(json.dumps(report, indent=2, sort_keys=True))

    if args.strict and not report["ok"]:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
