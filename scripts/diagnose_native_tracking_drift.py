#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Generate long-horizon native tracking drift diagnostics.

The script consumes a `run_native_tracking_bag_report.sh` output directory with
`artifacts/tracking_status_history.jsonl` enabled and writes:

* tracking_status_timeseries.csv: bias, factor-count, and Hessian health traces
* reference_motion_regime.csv: reference speed, yaw-rate, and acceleration
* PNG plots when matplotlib is available
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path


def finite(value: object, default: float = 0.0) -> float:
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return default
    return numeric if math.isfinite(numeric) else default


def read_status_history(path: Path) -> list[dict[str, float]]:
    records: list[dict[str, float]] = []
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            stripped = line.strip()
            if not stripped:
                continue
            raw = json.loads(stripped)
            record = {
                key: finite(value)
                for key, value in raw.items()
                if isinstance(value, (int, float)) and not isinstance(value, bool)
            }
            records.append(record)
    records.sort(key=lambda item: item.get("stamp", 0.0))
    return records


def read_tum(path: Path) -> list[tuple[float, float, float, float, float, float, float, float]]:
    poses = []
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            values = [float(token) for token in stripped.split()[:8]]
            if len(values) == 8:
                poses.append(tuple(values))
    poses.sort(key=lambda item: item[0])
    return poses


def yaw_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def unwrap_delta(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def mean(values: list[float]) -> float:
    finite_values = [value for value in values if math.isfinite(value)]
    return sum(finite_values) / len(finite_values) if finite_values else 0.0


def max_abs(values: list[float]) -> float:
    finite_values = [abs(value) for value in values if math.isfinite(value)]
    return max(finite_values) if finite_values else 0.0


def records_between(
    records: list[dict[str, float]], start_s: float, end_s: float
) -> list[dict[str, float]]:
    if not records:
        return []
    start_stamp = records[0].get("stamp", 0.0)
    return [
        record for record in records
        if start_s <= record.get("stamp", 0.0) - start_stamp <= end_s
    ]


def values_between(
    records: list[dict[str, float]], field: str, start_s: float, end_s: float
) -> list[float]:
    return [record.get(field, 0.0) for record in records_between(records, start_s, end_s)]


def max_consecutive_delta(records: list[dict[str, float]], field: str) -> float:
    if len(records) < 2:
        return 0.0
    deltas = [
        records[index].get(field, 0.0) - records[index - 1].get(field, 0.0)
        for index in range(1, len(records))
    ]
    return max_abs(deltas)


def motion_rows_from_poses(
    poses: list[tuple[float, float, float, float, float, float, float, float]]
) -> list[dict[str, float]]:
    if len(poses) < 2:
        raise RuntimeError("reference trajectory needs at least two poses")
    rows = []
    start_stamp = poses[0][0]
    previous_speed = 0.0
    previous_yaw = yaw_from_quaternion(poses[0][4], poses[0][5], poses[0][6], poses[0][7])
    for index in range(1, len(poses)):
        prev = poses[index - 1]
        pose = poses[index]
        dt = pose[0] - prev[0]
        if dt <= 0.0:
            continue
        dx = pose[1] - prev[1]
        dy = pose[2] - prev[2]
        dz = pose[3] - prev[3]
        speed = math.sqrt(dx * dx + dy * dy + dz * dz) / dt
        yaw = yaw_from_quaternion(pose[4], pose[5], pose[6], pose[7])
        yaw_rate = unwrap_delta(yaw - previous_yaw) / dt
        accel = (speed - previous_speed) / dt
        rows.append({
            "stamp": pose[0],
            "time_s": pose[0] - start_stamp,
            "speed_mps": speed,
            "accel_mps2": accel,
            "yaw_rad": yaw,
            "yaw_rate_radps": yaw_rate,
        })
        previous_speed = speed
        previous_yaw = yaw
    return rows


def write_status_csv(records: list[dict[str, float]], path: Path) -> None:
    if not records:
        raise RuntimeError("no TrackingStatus history records")
    start_stamp = records[0].get("stamp", 0.0)
    fields = [
        "stamp",
        "time_s",
        "sliding_window_gyro_bias_x",
        "sliding_window_gyro_bias_y",
        "sliding_window_gyro_bias_z",
        "sliding_window_accel_bias_x",
        "sliding_window_accel_bias_y",
        "sliding_window_accel_bias_z",
        "sliding_window_gyro_bias_norm",
        "sliding_window_accel_bias_norm",
        "sliding_window_gyro_bias_random_walk_sqrt_info_mean",
        "sliding_window_accel_bias_random_walk_sqrt_info_mean",
        "sliding_window_gyro_bias_random_walk_sqrt_info_max",
        "sliding_window_accel_bias_random_walk_sqrt_info_max",
        "sliding_window_gyro_bias_observability",
        "sliding_window_accel_bias_observability",
        "sliding_window_imu_factors",
        "sliding_window_point_factors",
        "sliding_window_plane_factors",
        "sliding_window_visual_factors",
        "sliding_window_se3_photometric_factors",
        "sliding_window_relative_translation_factors",
        "sliding_window_smoothness_factors",
        "last_window_point_correspondences",
        "last_window_plane_correspondences",
        "visual_se3_photometric_valid_batches",
        "sliding_window_normal_equation_min_singular_value",
        "sliding_window_normal_equation_max_singular_value",
        "sliding_window_normal_equation_condition_number",
        "sliding_window_normal_equation_rank",
        "sliding_window_normal_equation_rank_ratio",
        "sliding_window_dense_prior_rows",
        "sliding_window_dense_prior_rank",
        "sliding_window_marginalized_states",
        "sliding_window_schur_marginalizations",
        "sliding_window_fallback_marginalization_priors",
        "sliding_window_marginalized_backsubstitutions",
        "sliding_window_marginalized_backsubstitution_chain_updates",
        "sliding_window_marginalized_backsubstitution_interpolations",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for record in records:
            row = {field: record.get(field, 0.0) for field in fields}
            row["time_s"] = record.get("stamp", 0.0) - start_stamp
            writer.writerow(row)


def write_reference_motion_csv(poses: list[tuple[float, float, float, float, float, float, float, float]], path: Path) -> None:
    fields = ["stamp", "time_s", "speed_mps", "accel_mps2", "yaw_rad", "yaw_rate_radps"]
    rows = motion_rows_from_poses(poses)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def write_summary(
    records: list[dict[str, float]],
    reference_motion: list[dict[str, float]],
    report: dict[str, object],
    path: Path,
) -> None:
    def value_at_or_after(field: str, threshold_s: float) -> float:
        start = records[0].get("stamp", 0.0)
        for record in records:
            if record.get("stamp", 0.0) - start >= threshold_s:
                return record.get(field, 0.0)
        return records[-1].get(field, 0.0)

    def last(field: str) -> float:
        return records[-1].get(field, 0.0)

    run_duration_s = records[-1].get("stamp", 0.0) - records[0].get("stamp", 0.0)
    has_post70_horizon = run_duration_s >= 70.0
    trajectory = report.get("trajectory_compare", {})
    translation = trajectory.get("translation", {}) if isinstance(trajectory, dict) else {}
    path_length = trajectory.get("path_length", {}) if isinstance(trajectory, dict) else {}
    factor_fields = (
        "sliding_window_imu_factors",
        "sliding_window_point_factors",
        "sliding_window_plane_factors",
        "sliding_window_visual_factors",
        "sliding_window_se3_photometric_factors",
        "sliding_window_relative_translation_factors",
    )
    factor_means_pre70 = {
        field: mean(values_between(records, field, 10.0, 70.0))
        for field in factor_fields
    }
    factor_means_post70 = {
        field: mean(values_between(records, field, 70.0, 1.0e9))
        for field in factor_fields
    }
    factor_drop_ratios = {
        field: (
            factor_means_post70[field] / factor_means_pre70[field]
            if factor_means_pre70[field] > 0.0 else 0.0
        )
        for field in factor_fields
    }
    lambda_min_pre70 = mean(
        values_between(records, "sliding_window_normal_equation_min_singular_value", 10.0, 70.0))
    lambda_min_post70 = mean(
        values_between(records, "sliding_window_normal_equation_min_singular_value", 70.0, 1.0e9))
    condition_pre70 = mean(
        values_between(records, "sliding_window_normal_equation_condition_number", 10.0, 70.0))
    condition_post70 = mean(
        values_between(records, "sliding_window_normal_equation_condition_number", 70.0, 1.0e9))
    bias_random_walk_sqrt_info = {
        "gyro_mean_pre70": mean(
            values_between(
                records, "sliding_window_gyro_bias_random_walk_sqrt_info_mean", 10.0, 70.0)),
        "gyro_mean_post70": mean(
            values_between(
                records, "sliding_window_gyro_bias_random_walk_sqrt_info_mean", 70.0, 1.0e9)),
        "gyro_max_end": last("sliding_window_gyro_bias_random_walk_sqrt_info_max"),
        "accel_mean_pre70": mean(
            values_between(
                records, "sliding_window_accel_bias_random_walk_sqrt_info_mean", 10.0, 70.0)),
        "accel_mean_post70": mean(
            values_between(
                records, "sliding_window_accel_bias_random_walk_sqrt_info_mean", 70.0, 1.0e9)),
        "accel_max_end": last("sliding_window_accel_bias_random_walk_sqrt_info_max"),
    }

    motion_pre70 = [row for row in reference_motion if 10.0 <= row.get("time_s", 0.0) <= 70.0]
    motion_post70 = [row for row in reference_motion if row.get("time_s", 0.0) >= 70.0]
    reference_motion_regime = {
        "speed_mean_pre70_mps": mean([row.get("speed_mps", 0.0) for row in motion_pre70]),
        "speed_mean_post70_mps": mean([row.get("speed_mps", 0.0) for row in motion_post70]),
        "accel_abs_max_pre70_mps2": max_abs([row.get("accel_mps2", 0.0) for row in motion_pre70]),
        "accel_abs_max_post70_mps2": max_abs([row.get("accel_mps2", 0.0) for row in motion_post70]),
        "yaw_rate_abs_max_pre70_radps": max_abs(
            [row.get("yaw_rate_radps", 0.0) for row in motion_pre70]),
        "yaw_rate_abs_max_post70_radps": max_abs(
            [row.get("yaw_rate_radps", 0.0) for row in motion_post70]),
    }
    bias_jump = {
        "gyro_norm_max_consecutive_delta": max_consecutive_delta(
            records, "sliding_window_gyro_bias_norm"),
        "accel_norm_max_consecutive_delta": max_consecutive_delta(
            records, "sliding_window_accel_bias_norm"),
        "gyro_norm_max_consecutive_delta_post70": max_consecutive_delta(
            records_between(records, 70.0, 1.0e9), "sliding_window_gyro_bias_norm"),
        "accel_norm_max_consecutive_delta_post70": max_consecutive_delta(
            records_between(records, 70.0, 1.0e9), "sliding_window_accel_bias_norm"),
    }
    lambda_collapse_ratio = (
        lambda_min_post70 / lambda_min_pre70 if lambda_min_pre70 > 0.0 else 0.0
    )
    summary = {
        "sample_count": len(records),
        "run_duration_s": run_duration_s,
        "insufficient_70s_horizon": not has_post70_horizon,
        "trajectory_rmse_m": translation.get("rmse_m"),
        "trajectory_path_drift": path_length.get("relative_drift"),
        "gyro_bias_norm_delta_70s_to_end": last("sliding_window_gyro_bias_norm") -
        value_at_or_after("sliding_window_gyro_bias_norm", 70.0),
        "accel_bias_norm_delta_70s_to_end": last("sliding_window_accel_bias_norm") -
        value_at_or_after("sliding_window_accel_bias_norm", 70.0),
        "lambda_min_70s": value_at_or_after("sliding_window_normal_equation_min_singular_value", 70.0),
        "lambda_min_end": last("sliding_window_normal_equation_min_singular_value"),
        "condition_70s": value_at_or_after("sliding_window_normal_equation_condition_number", 70.0),
        "condition_end": last("sliding_window_normal_equation_condition_number"),
        "dense_prior_rows_end": last("sliding_window_dense_prior_rows"),
        "schur_marginalizations_end": last("sliding_window_schur_marginalizations"),
        "fallback_marginalization_priors_end": last("sliding_window_fallback_marginalization_priors"),
        "bias_jump": bias_jump,
        "factor_means_pre70": factor_means_pre70,
        "factor_means_post70": factor_means_post70,
        "factor_drop_ratios_post70_over_pre70": factor_drop_ratios,
        "bias_random_walk_sqrt_info": bias_random_walk_sqrt_info,
        "lambda_min_mean_pre70": lambda_min_pre70,
        "lambda_min_mean_post70": lambda_min_post70,
        "lambda_min_post70_over_pre70": lambda_collapse_ratio,
        "condition_mean_pre70": condition_pre70,
        "condition_mean_post70": condition_post70,
        "reference_motion_regime": reference_motion_regime,
        "hypothesis_flags": {
            "bias_jump_after_70s": has_post70_horizon and (
                bias_jump["gyro_norm_max_consecutive_delta_post70"] > 0.01
                or bias_jump["accel_norm_max_consecutive_delta_post70"] > 0.10
            ),
            "factor_count_drop_after_70s": has_post70_horizon and any(
                ratio < 0.50 for ratio in factor_drop_ratios.values()
            ),
            "lambda_min_collapse_after_70s": has_post70_horizon and
            0.0 < lambda_collapse_ratio < 0.20,
            "reference_motion_regime_change_after_70s": has_post70_horizon and (
                reference_motion_regime["accel_abs_max_post70_mps2"] >
                2.0 * max(reference_motion_regime["accel_abs_max_pre70_mps2"], 1.0e-9)
                or reference_motion_regime["yaw_rate_abs_max_post70_radps"] >
                2.0 * max(reference_motion_regime["yaw_rate_abs_max_pre70_radps"], 1.0e-9)
            ),
        },
    }
    path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def try_write_plots(status_csv: Path, motion_csv: Path, output_dir: Path) -> None:
    try:
        import matplotlib.pyplot as plt
    except Exception:
        return

    def read_csv(path: Path) -> dict[str, list[float]]:
        with path.open("r", encoding="utf-8") as handle:
            reader = csv.DictReader(handle)
            columns: dict[str, list[float]] = {}
            for row in reader:
                for key, value in row.items():
                    columns.setdefault(key, []).append(finite(value))
            return columns

    status = read_csv(status_csv)
    motion = read_csv(motion_csv)
    time = status.get("time_s", [])

    plt.figure(figsize=(11, 6))
    for key in (
        "sliding_window_gyro_bias_x",
        "sliding_window_gyro_bias_y",
        "sliding_window_gyro_bias_z",
        "sliding_window_accel_bias_x",
        "sliding_window_accel_bias_y",
        "sliding_window_accel_bias_z",
    ):
        plt.plot(time, status.get(key, []), label=key.replace("sliding_window_", ""))
    plt.axvline(70.0, color="k", linestyle="--", linewidth=1.0)
    plt.xlabel("time [s]")
    plt.ylabel("bias")
    plt.legend(fontsize=8)
    plt.tight_layout()
    plt.savefig(output_dir / "bias_trajectory.png", dpi=160)
    plt.close()

    plt.figure(figsize=(11, 6))
    for key in (
        "sliding_window_imu_factors",
        "sliding_window_point_factors",
        "sliding_window_plane_factors",
        "sliding_window_visual_factors",
        "sliding_window_se3_photometric_factors",
        "sliding_window_relative_translation_factors",
    ):
        plt.plot(time, status.get(key, []), label=key.replace("sliding_window_", ""))
    plt.axvline(70.0, color="k", linestyle="--", linewidth=1.0)
    plt.xlabel("time [s]")
    plt.ylabel("factor count")
    plt.legend(fontsize=8)
    plt.tight_layout()
    plt.savefig(output_dir / "factor_counts.png", dpi=160)
    plt.close()

    plt.figure(figsize=(11, 6))
    lambda_min = status.get("sliding_window_normal_equation_min_singular_value", [])
    condition = status.get("sliding_window_normal_equation_condition_number", [])
    plt.semilogy(time, [max(value, 1.0e-30) for value in lambda_min], label="lambda_min")
    plt.semilogy(time, [max(value, 1.0) for value in condition], label="condition")
    plt.axvline(70.0, color="k", linestyle="--", linewidth=1.0)
    plt.xlabel("time [s]")
    plt.legend(fontsize=8)
    plt.tight_layout()
    plt.savefig(output_dir / "hessian_health.png", dpi=160)
    plt.close()

    plt.figure(figsize=(11, 6))
    motion_time = motion.get("time_s", [])
    plt.plot(motion_time, motion.get("speed_mps", []), label="speed_mps")
    plt.plot(motion_time, motion.get("accel_mps2", []), label="accel_mps2")
    plt.plot(motion_time, motion.get("yaw_rate_radps", []), label="yaw_rate_radps")
    plt.axvline(70.0, color="k", linestyle="--", linewidth=1.0)
    plt.xlabel("time [s]")
    plt.legend(fontsize=8)
    plt.tight_layout()
    plt.savefig(output_dir / "reference_motion_regime.png", dpi=160)
    plt.close()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", required=True, type=Path)
    parser.add_argument("--reference-tum", type=Path)
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()

    run_dir = args.run_dir
    artifact_dir = run_dir / "artifacts"
    metrics_path = artifact_dir / "metrics.json"
    history_path = artifact_dir / "tracking_status_history.jsonl"
    report_path = run_dir / "native_tracking_report.json"
    if not history_path.is_file():
        raise SystemExit(
            f"missing {history_path}; rerun with --write-status-history")
    if not metrics_path.is_file():
        raise SystemExit(f"missing {metrics_path}")
    output_dir = args.output_dir or (run_dir / "diagnostics")
    output_dir.mkdir(parents=True, exist_ok=True)

    records = read_status_history(history_path)
    reference_tum = args.reference_tum
    if reference_tum is None:
        metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
        reference_tum_value = metrics.get("outputs", {}).get("reference_trajectory_tum", "")
        reference_tum = Path(reference_tum_value) if reference_tum_value else None
    if reference_tum is None or not reference_tum.is_file() or reference_tum.stat().st_size == 0:
        raise SystemExit("reference TUM is required for motion-regime diagnostics")
    reference_poses = read_tum(reference_tum)
    reference_motion = motion_rows_from_poses(reference_poses)
    report = json.loads(report_path.read_text(encoding="utf-8")) if report_path.is_file() else {}

    status_csv = output_dir / "tracking_status_timeseries.csv"
    motion_csv = output_dir / "reference_motion_regime.csv"
    write_status_csv(records, status_csv)
    write_reference_motion_csv(reference_poses, motion_csv)
    write_summary(records, reference_motion, report, output_dir / "diagnostic_summary.json")
    try_write_plots(status_csv, motion_csv, output_dir)
    print(f"diagnostics written to {output_dir}")


if __name__ == "__main__":
    main()
