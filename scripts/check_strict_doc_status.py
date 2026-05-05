#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Keep strict CBD documentation aligned with the latest archived report."""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path


DEFAULT_REPORT = Path(
    "results/fastlivo2/CBD_Building_01_current_upstream_sampling_probe/"
    "reproduction_report_strict.json"
)
DOCS = (Path("README.md"), Path("docs/ROADMAP.md"), Path("docs/RELEASE_MILESTONES.md"))


def _load_json(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as handle:
        data = json.load(handle)
    if not isinstance(data, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return data


def _read_docs(paths: tuple[Path, ...]) -> dict[Path, str]:
    docs: dict[Path, str] = {}
    for path in paths:
        docs[path] = path.read_text(encoding="utf-8")
    return docs


def _comparison(report: dict, key: str) -> dict:
    quality = report.get("novel_view_quality", {})
    metrics = quality.get("metrics", {})
    comparisons = metrics.get("comparisons", [])
    for item in comparisons:
        if item.get("key") == key:
            return item
    raise KeyError(f"missing novel_view_quality comparison for {key}")


def _require(text: str, token: str, path: Path, errors: list[str]) -> None:
    if token not in text:
        errors.append(f"{path}: missing required strict-status text: {token!r}")


def _forbid(text: str, pattern: str, path: Path, errors: list[str]) -> None:
    if re.search(pattern, text, flags=re.IGNORECASE | re.DOTALL):
        errors.append(f"{path}: forbidden stale strict-pass claim matches: {pattern}")


def check_docs(report: dict, docs: dict[Path, str]) -> list[str]:
    errors: list[str] = []
    report_ok = bool(report.get("ok"))
    combined = "\n".join(docs.values())

    if report_ok:
        return errors

    ssim = _comparison(report, "novel_ssim")
    ssim_regression = float(ssim["regression"]) * 100.0
    point_cloud = report.get("point_cloud", {})
    centroid_drift = float(point_cloud["centroid_drift_m"])
    nearest_mean = float(point_cloud["nearest"]["bidirectional"]["mean_m"])
    current_dir = str(report.get("current_dir", ""))
    current_rel = current_dir.split("/gaussian_lic_ros2/", 1)[-1]

    for path, text in docs.items():
        if path.name == "README.md":
            _require(text, "latest archived strict `CBD_Building_01` report is not green", path, errors)
            _require(text, "blocked on novel SSIM", path, errors)
        if path.match("docs/ROADMAP.md"):
            _require(text, "- [ ] Strict `CBD_Building_01` ROS2 current report", path, errors)
            _require(text, "fails `reproduction_report.py --strict`", path, errors)
        if path.match("docs/RELEASE_MILESTONES.md"):
            _require(text, "latest archived strict `CBD_Building_01` report is not green", path, errors)
            _require(text, "blocked on novel SSIM", path, errors)
        _require(text, current_rel, path, errors)
        _require(text, f"{ssim_regression:.2f}%", path, errors)
        _require(text, f"{centroid_drift:.6f}", path, errors)
        _require(text, f"{nearest_mean:.6f}", path, errors)

    stale_patterns = [
        r"passing local strict `CBD_Building_01`",
        r"Strict FAST-LIVO2 `CBD_Building_01` report .* gates passing",
        r"Strict `CBD_Building_01` paper-data gate \| .* passes the strict local gate",
        r"passing local strict report",
        r"strict chain now passes",
        r"passes the local paper metric gate",
        r"Current vs ROS1 quality passes the paper metric gate",
        r"Chamfer/point-cloud parity passes",
        r"chain passes the local trajectory, PSNR/SSIM/LPIPS, GT-associated render-pair,\s*and Chamfer gates",
        r"- \[x\] Strict `CBD_Building_01` ROS2 current report",
        r"passes `baseline_readiness\.py --strict`",
        r"passes `reproduction_report\.py --strict`",
    ]
    for pattern in stale_patterns:
        _forbid(combined, pattern, Path("README.md docs/ROADMAP.md"), errors)

    return errors


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path, default=DEFAULT_REPORT)
    args = parser.parse_args()

    if not args.report.exists():
        print(f"[strict-doc-status] skipped: {args.report} is not present")
        return 0

    report = _load_json(args.report)
    docs = _read_docs(DOCS)
    errors = check_docs(report, docs)
    if errors:
        for error in errors:
            print(error, file=sys.stderr)
        return 1
    state = "PASS" if report.get("ok") else "BLOCKED"
    print(f"[strict-doc-status] docs match {args.report} ({state})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
