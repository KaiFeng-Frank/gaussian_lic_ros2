#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Check the committed Coco-LIC2 ROS2 port evidence.

This is a static release gate for the faithful Coco-LIC / Gaussian-LIC2
frontend port. It makes the cm-grade trajectory evidence executable instead of
leaving it as prose in PORT_RESULTS.md.
"""

from __future__ import annotations

import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
REFERENCE_SUFFIX = (
    "baseline/fastlivo2/CBD_Building_01/native_reference/"
    "cocolic_livo_reference_10hz.tum"
)


@dataclass(frozen=True)
class ReportGate:
    name: str
    report_path: Path
    trajectory_suffix: str
    max_rmse_m: float
    max_mean_m: float
    max_error_m: float
    max_path_drift: float
    min_matches: int = 1231
    min_coverage: float = 0.999
    min_current_poses: int = 10000
    max_association_dt_sec: float = 0.02


REPORT_GATES = (
    ReportGate(
        name="lio",
        report_path=Path("run_lio/lio_vs_reference_report.json"),
        trajectory_suffix="run_lio/data/CBD_Building_01_frontend_raw_offset_time_full_LIO.txt",
        max_rmse_m=0.03,
        max_mean_m=0.03,
        max_error_m=0.15,
        max_path_drift=0.01,
    ),
    ReportGate(
        name="lico",
        report_path=Path("run_lio/lico_vs_reference_report.json"),
        trajectory_suffix="run_lio/data/CBD_Building_01_frontend_raw_offset_time_full_LICO.txt",
        max_rmse_m=0.03,
        max_mean_m=0.03,
        max_error_m=0.15,
        max_path_drift=0.01,
    ),
    ReportGate(
        name="lico_camerafixed",
        report_path=Path("run_lio/lico_camerafixed_vs_reference_report.json"),
        trajectory_suffix="run_lio/data/CBD_Building_01_frontend_raw_offset_time_full_LICO.txt",
        max_rmse_m=0.04,
        max_mean_m=0.03,
        max_error_m=0.15,
        max_path_drift=0.01,
    ),
)

REQUIRED_DOC_SNIPPETS = (
    "run_lio/lio_vs_reference_report.json",
    "run_lio/lico_vs_reference_report.json",
    "run_lio/lico_camerafixed_vs_reference_report.json",
    "run_lio/data/CBD_Building_01_frontend_raw_offset_time_full_LIO.txt",
    "run_lio/data/CBD_Building_01_frontend_raw_offset_time_full_LICO.txt",
    "100% coverage",
    "1231/1231 poses",
)

REQUIRED_CTEST_SNIPPETS = (
    "add_test(NAME cocolic_bag_ingest_test",
    "add_test(NAME cocolic_ingest_offset_time_test",
    "add_test(NAME cocolic_storage_autodetect_test",
)


def load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        data = json.load(handle)
    if not isinstance(data, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return data


def value_at_path(data: dict[str, Any], dotted_path: str) -> Any:
    value: Any = data
    for part in dotted_path.split("."):
        if not isinstance(value, dict) or part not in value:
            raise KeyError(dotted_path)
        value = value[part]
    return value


def finite_number(data: dict[str, Any], dotted_path: str) -> float:
    value = value_at_path(data, dotted_path)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"{dotted_path} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{dotted_path} must be finite")
    return result


def require(condition: bool, errors: list[str], message: str) -> None:
    if not condition:
        errors.append(message)


def require_path_suffix(
    report: dict[str, Any],
    key: str,
    suffix: str,
    errors: list[str],
    label: str,
) -> None:
    value = report.get(key)
    if not isinstance(value, str):
        errors.append(f"{label}: {key} must be a string")
        return
    require(value.endswith(suffix), errors, f"{label}: {key} must end with {suffix}")


def check_thresholds_are_not_looser(
    report: dict[str, Any], gate: ReportGate, errors: list[str]
) -> None:
    thresholds = report.get("thresholds")
    if not isinstance(thresholds, dict):
        errors.append(f"{gate.name}: thresholds must be a JSON object")
        return
    checks = {
        "max_rmse_m": 0.05,
        "max_mean_m": 0.03,
        "max_error_m": gate.max_error_m,
        "max_path_drift": 0.05,
        "min_coverage": 0.8,
        "min_matches": 2,
    }
    for key, ceiling in checks.items():
        value = thresholds.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            errors.append(f"{gate.name}: thresholds.{key} must be numeric")
            continue
        if key.startswith("min_"):
            if float(value) < float(ceiling):
                errors.append(f"{gate.name}: thresholds.{key}={value} is weaker than {ceiling}")
        elif float(value) > float(ceiling):
            errors.append(f"{gate.name}: thresholds.{key}={value} is weaker than {ceiling}")


def check_report(gate: ReportGate, errors: list[str]) -> None:
    report_file = ROOT / gate.report_path
    if not report_file.exists():
        errors.append(f"{gate.name}: missing report {gate.report_path}")
        return
    trajectory_file = ROOT / gate.trajectory_suffix
    require(
        trajectory_file.exists(),
        errors,
        f"{gate.name}: missing output trajectory {gate.trajectory_suffix}",
    )
    report = load_json(report_file)
    require(report.get("ok") is True, errors, f"{gate.name}: report ok must be true")
    require_path_suffix(report, "baseline", REFERENCE_SUFFIX, errors, gate.name)
    require_path_suffix(report, "current", gate.trajectory_suffix, errors, gate.name)
    require(report.get("alignment") == "yaw", errors, f"{gate.name}: alignment must be yaw")
    require(report.get("coverage_mode") == "all", errors, f"{gate.name}: coverage_mode must be all")
    require(
        int(report.get("matched_poses", -1)) >= gate.min_matches,
        errors,
        f"{gate.name}: matched_poses must be >= {gate.min_matches}",
    )
    require(
        int(report.get("baseline_poses", -1)) == gate.min_matches,
        errors,
        f"{gate.name}: baseline_poses must equal {gate.min_matches}",
    )
    require(
        int(report.get("coverage_denominator_poses", -1)) == gate.min_matches,
        errors,
        f"{gate.name}: coverage_denominator_poses must equal {gate.min_matches}",
    )
    require(
        int(report.get("current_poses", -1)) >= gate.min_current_poses,
        errors,
        f"{gate.name}: current_poses must be >= {gate.min_current_poses}",
    )
    require(
        finite_number(report, "coverage") >= gate.min_coverage,
        errors,
        f"{gate.name}: coverage must be >= {gate.min_coverage}",
    )
    require(
        finite_number(report, "max_association_dt_sec") <= gate.max_association_dt_sec,
        errors,
        f"{gate.name}: max_association_dt_sec must be <= {gate.max_association_dt_sec}",
    )
    require(
        finite_number(report, "translation.rmse_m") <= gate.max_rmse_m,
        errors,
        f"{gate.name}: translation.rmse_m must be <= {gate.max_rmse_m}",
    )
    require(
        finite_number(report, "translation.mean_m") <= gate.max_mean_m,
        errors,
        f"{gate.name}: translation.mean_m must be <= {gate.max_mean_m}",
    )
    require(
        finite_number(report, "translation.max_m") <= gate.max_error_m,
        errors,
        f"{gate.name}: translation.max_m must be <= {gate.max_error_m}",
    )
    require(
        finite_number(report, "path_length.relative_drift") <= gate.max_path_drift,
        errors,
        f"{gate.name}: path_length.relative_drift must be <= {gate.max_path_drift}",
    )
    require(
        finite_number(report, "path_length.current_to_baseline_ratio") >= 0.99,
        errors,
        f"{gate.name}: path length ratio must be >= 0.99",
    )
    require(
        finite_number(report, "path_length.current_to_baseline_ratio") <= 1.01,
        errors,
        f"{gate.name}: path length ratio must be <= 1.01",
    )
    check_thresholds_are_not_looser(report, gate, errors)


def check_docs(errors: list[str]) -> None:
    port_results = ROOT / "src" / "cocolic" / "PORT_RESULTS.md"
    text = port_results.read_text(encoding="utf-8")
    for snippet in REQUIRED_DOC_SNIPPETS:
        require(snippet in text, errors, f"PORT_RESULTS.md missing {snippet}")
    if "super-paper sub-cm is structurally" not in text:
        errors.append("PORT_RESULTS.md must avoid claiming unverifiable sub-cm super-paper parity")


def check_ctests(errors: list[str]) -> None:
    cmake_lists = ROOT / "src" / "cocolic" / "CMakeLists.txt"
    text = cmake_lists.read_text(encoding="utf-8")
    for snippet in REQUIRED_CTEST_SNIPPETS:
        require(snippet in text, errors, f"CMakeLists.txt missing {snippet}")


def main() -> int:
    errors: list[str] = []
    for gate in REPORT_GATES:
        check_report(gate, errors)
    check_docs(errors)
    check_ctests(errors)
    if errors:
        print("Coco-LIC2 port evidence check failed:", file=sys.stderr)
        for error in errors:
            print(f"  - {error}", file=sys.stderr)
        return 1
    names = ", ".join(gate.name for gate in REPORT_GATES)
    print(f"Coco-LIC2 port evidence check passed: {names}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
