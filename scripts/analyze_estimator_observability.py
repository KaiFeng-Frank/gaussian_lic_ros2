#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later

"""Summarize native tracker estimator observability from saved reports.

The input may be either artifacts/metrics.json or native_tracking_report.json.
The output mirrors the estimator_observability_continuity block produced by
run_native_tracking_bag_report.sh so older evidence can be diagnosed without
rerunning a bag.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any


ESTIMATOR_FACTOR_DELTA_FIELDS = (
    "sliding_window_total_imu_factors",
    "sliding_window_total_visual_factors",
    "sliding_window_total_se3_photometric_factors",
    "sliding_window_point_factors",
    "sliding_window_plane_factors",
    "sliding_window_relative_translation_factors",
    "sliding_window_relative_distance_factors",
    "sliding_window_smoothness_factors",
    "sliding_window_dense_priors",
)

ESTIMATOR_OBSERVABILITY_VALUE_FIELDS = (
    "sliding_window_normal_equation_min_singular_value",
    "sliding_window_normal_equation_max_singular_value",
    "sliding_window_normal_equation_condition_number",
    "sliding_window_dense_prior_min_singular_value",
    "sliding_window_dense_prior_max_singular_value",
    "sliding_window_gyro_bias_norm",
    "sliding_window_accel_bias_norm",
    "sliding_window_gyro_bias_x",
    "sliding_window_gyro_bias_y",
    "sliding_window_gyro_bias_z",
    "sliding_window_accel_bias_x",
    "sliding_window_accel_bias_y",
    "sliding_window_accel_bias_z",
    "sliding_window_gyro_bias_observability",
    "sliding_window_accel_bias_observability",
)


def finite_float(value: Any, default: float = 0.0) -> float:
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return default
    return numeric if math.isfinite(numeric) else default


def summary_delta(bin_summary: dict[str, Any], key: str) -> int:
    summary = bin_summary.get("summary", {})
    if not isinstance(summary, dict):
        return 0
    field_summary = summary.get(key, {})
    if not isinstance(field_summary, dict):
        return 0
    return int(finite_float(field_summary.get("delta", 0.0), 0.0))


def summary_value(
    bin_summary: dict[str, Any],
    key: str,
    statistic: str,
    default: float = 0.0,
) -> float:
    summary = bin_summary.get("summary", {})
    if not isinstance(summary, dict):
        return default
    field_summary = summary.get(key, {})
    if not isinstance(field_summary, dict):
        return default
    return finite_float(field_summary.get(statistic, default), default)


def build_estimator_observability_continuity(status: dict[str, Any]) -> dict[str, Any]:
    bins: list[dict[str, Any]] = []
    for bin_summary in status.get("binned_summary", []):
        if not isinstance(bin_summary, dict):
            continue
        item: dict[str, Any] = {
            "index": int(bin_summary.get("index", len(bins)) or 0),
            "sample_count": int(bin_summary.get("sample_count", 0) or 0),
        }
        for field_name in ESTIMATOR_FACTOR_DELTA_FIELDS:
            item[f"{field_name}_delta"] = summary_delta(bin_summary, field_name)
        for field_name in ESTIMATOR_OBSERVABILITY_VALUE_FIELDS:
            item[f"{field_name}_min"] = summary_value(bin_summary, field_name, "min")
            item[f"{field_name}_max"] = summary_value(bin_summary, field_name, "max")
            item[f"{field_name}_last"] = summary_value(bin_summary, field_name, "last")
            item[f"{field_name}_delta"] = summary_value(bin_summary, field_name, "delta")
        bins.append(item)

    if not bins:
        return {
            "available": False,
            "reason": "tracking_status.binned_summary is missing or empty",
            "bins": [],
        }

    def min_value(field_name: str, statistic: str) -> float:
        return min(float(item[f"{field_name}_{statistic}"]) for item in bins)

    def max_value(field_name: str, statistic: str) -> float:
        return max(float(item[f"{field_name}_{statistic}"]) for item in bins)

    def max_abs_delta(field_name: str) -> float:
        return max(abs(float(item[f"{field_name}_delta"])) for item in bins)

    worst_information_bins = sorted(
        bins,
        key=lambda item: (
            item["sliding_window_normal_equation_min_singular_value_min"],
            -item["sliding_window_normal_equation_condition_number_max"],
            item["sample_count"],
            item["index"],
        ),
    )[:3]
    worst_bias_bins = sorted(
        bins,
        key=lambda item: (
            -item["sliding_window_gyro_bias_norm_max"],
            -item["sliding_window_accel_bias_norm_max"],
            item["index"],
        ),
    )[:3]
    return {
        "available": True,
        "binned_summary_finalized": bool(status.get("binned_summary_finalized", False)),
        "bin_count": len(bins),
        "min_normal_equation_min_singular_value": min_value(
            "sliding_window_normal_equation_min_singular_value", "min"),
        "max_normal_equation_condition_number": max_value(
            "sliding_window_normal_equation_condition_number", "max"),
        "min_dense_prior_min_singular_value": min_value(
            "sliding_window_dense_prior_min_singular_value", "min"),
        "max_gyro_bias_norm": max_value("sliding_window_gyro_bias_norm", "max"),
        "max_accel_bias_norm": max_value("sliding_window_accel_bias_norm", "max"),
        "max_abs_gyro_bias_norm_delta": max_abs_delta("sliding_window_gyro_bias_norm"),
        "max_abs_accel_bias_norm_delta": max_abs_delta("sliding_window_accel_bias_norm"),
        "worst_information_bins": worst_information_bins,
        "worst_bias_bins": worst_bias_bins,
        "bins": bins,
    }


def load_tracking_status(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if "estimator_observability_continuity" in payload:
        return {
            "precomputed": payload["estimator_observability_continuity"],
        }
    metrics = payload.get("metrics", payload)
    status = metrics.get("tracking_status", {})
    if not isinstance(status, dict):
        raise ValueError(f"{path} does not contain metrics.tracking_status")
    return status


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="metrics.json or native_tracking_report.json")
    parser.add_argument("--output", type=Path, help="Optional JSON output path")
    parser.add_argument(
        "--require-available",
        action="store_true",
        help="Exit nonzero if no finalized binned observability summary is available",
    )
    args = parser.parse_args()

    status = load_tracking_status(args.input)
    if "precomputed" in status:
        summary = status["precomputed"]
    else:
        summary = build_estimator_observability_continuity(status)

    encoded = json.dumps(summary, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded, encoding="utf-8")
    else:
        print(encoded, end="")

    if args.require_available and not bool(summary.get("available", False)):
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
