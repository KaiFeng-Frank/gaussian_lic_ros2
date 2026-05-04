#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later

import argparse
import json
from pathlib import Path
from types import SimpleNamespace
import sys

from baseline_manifest import build_manifest
from perf_regression import DEFAULT_KEYS, is_latency_key, load_json, lookup
from pointcloud_compare import compute_report as compute_pointcloud_report
from trajectory_compare import compute_report as compute_trajectory_report


def auto_point_cloud_path(directory):
    for name in ("point_cloud.ply", "point_cloud_debug.ply"):
        path = directory / name
        if path.is_file():
            return path
    return directory / "point_cloud.ply"


def compare_metrics(baseline_path, current_path, keys, max_regression):
    errors = []
    skipped = []
    comparisons = []
    try:
        baseline = load_json(baseline_path)
        current = load_json(current_path)
    except Exception as exc:  # noqa: BLE001 - report malformed metrics uniformly.
        return {
            "ok": False,
            "baseline": str(baseline_path),
            "current": str(current_path),
            "comparisons": comparisons,
            "skipped": skipped,
            "errors": [str(exc)],
        }

    for key in keys:
        try:
            base = lookup(baseline, key)
            cur = lookup(current, key)
        except (KeyError, TypeError) as exc:
            skipped.append({"key": key, "reason": str(exc)})
            continue

        if base == 0.0:
            skipped.append({"key": key, "reason": "baseline is zero"})
            continue

        if is_latency_key(key):
            regression = (cur - base) / abs(base)
            direction = "lower_is_better"
        else:
            regression = (base - cur) / abs(base)
            direction = "higher_is_better"
        ok = regression <= max_regression
        if not ok:
            errors.append(f"{key} regression {regression:.2%} > {max_regression:.2%}")

        comparisons.append(
            {
                "key": key,
                "baseline": base,
                "current": cur,
                "direction": direction,
                "regression": regression,
                "max_regression": max_regression,
                "ok": ok,
            }
        )

    if not comparisons:
        errors.append("no comparable metrics")

    return {
        "ok": not errors,
        "baseline": str(baseline_path),
        "current": str(current_path),
        "comparisons": comparisons,
        "skipped": skipped,
        "errors": errors,
    }


def run_baseline_manifest(args, baseline_dir):
    if args.skip_baseline_manifest:
        return {"ok": True, "skipped": True, "errors": []}
    manifest_args = SimpleNamespace(
        baseline=str(baseline_dir),
        dataset=args.dataset,
        sequence=args.sequence or baseline_dir.name,
        min_renders=args.min_renders,
    )
    try:
        return build_manifest(manifest_args)
    except Exception as exc:  # noqa: BLE001 - keep report generation best-effort.
        return {"ok": False, "errors": [str(exc)]}


def run_trajectory_compare(args, baseline_dir, current_dir):
    if args.skip_trajectory:
        return {"ok": True, "skipped": True, "errors": []}
    compare_args = SimpleNamespace(
        baseline=str(baseline_dir / args.trajectory_name),
        current=str(current_dir / args.trajectory_name),
        align=args.trajectory_align,
        max_association_dt=args.max_association_dt,
        min_matches=args.min_trajectory_matches,
        min_coverage=args.min_trajectory_coverage,
        max_rmse_m=args.max_trajectory_rmse_m,
        max_mean_m=args.max_trajectory_mean_m,
        max_error_m=args.max_trajectory_error_m,
        max_path_drift=args.max_trajectory_path_drift,
    )
    try:
        return compute_trajectory_report(compare_args)
    except Exception as exc:  # noqa: BLE001 - keep report generation best-effort.
        return {"ok": False, "errors": [str(exc)]}


def run_pointcloud_compare(args, baseline_dir, current_dir):
    if args.skip_pointcloud:
        return {"ok": True, "skipped": True, "errors": []}
    baseline_path = Path(args.baseline_point_cloud) if args.baseline_point_cloud else baseline_dir / "point_cloud.ply"
    current_path = Path(args.current_point_cloud) if args.current_point_cloud else auto_point_cloud_path(current_dir)
    compare_args = SimpleNamespace(
        baseline=str(baseline_path),
        current=str(current_path),
        max_points=args.max_pointcloud_points,
        voxel_size=args.pointcloud_voxel_size,
        align=args.pointcloud_align,
        max_nearest_m=args.max_nearest_m,
        min_count_ratio=args.min_point_count_ratio,
        max_count_ratio=args.max_point_count_ratio,
        max_centroid_drift_m=args.max_centroid_drift_m,
        max_chamfer_rmse_m=args.max_chamfer_rmse_m,
        max_chamfer_mean_m=args.max_chamfer_mean_m,
        max_chamfer_max_m=args.max_chamfer_max_m,
        max_unmatched_ratio=args.max_unmatched_ratio,
        max_mean_rgb_drift=args.max_mean_rgb_drift,
        derive_gaussian_rgb=args.derive_gaussian_rgb,
    )
    try:
        return compute_pointcloud_report(compare_args)
    except Exception as exc:  # noqa: BLE001 - keep report generation best-effort.
        return {"ok": False, "errors": [str(exc)]}


def gate_status(gate_report):
    if gate_report.get("skipped"):
        return "SKIP"
    return "PASS" if gate_report.get("ok") else "FAIL"


def render_markdown(report):
    lines = [
        "# Gaussian-LIC Reproduction Report",
        "",
        f"Status: {'PASS' if report['ok'] else 'FAIL'}",
        f"Dataset: {report['dataset']}",
        f"Sequence: {report['sequence']}",
        "",
        "| Gate | Status |",
        "| --- | --- |",
    ]
    for name in ("baseline_manifest", "metrics", "trajectory", "point_cloud"):
        lines.append(f"| {name} | {gate_status(report[name])} |")

    metrics = report["metrics"]
    if metrics.get("comparisons"):
        lines.extend(["", "## Metrics", "", "| Key | Baseline | Current | Regression | Status |", "| --- | ---: | ---: | ---: | --- |"])
        for item in metrics["comparisons"]:
            lines.append(
                f"| {item['key']} | {item['baseline']:.6g} | {item['current']:.6g} | "
                f"{item['regression']:.2%} | {'PASS' if item['ok'] else 'FAIL'} |"
            )

    trajectory = report["trajectory"]
    if "translation" in trajectory:
        lines.extend(
            [
                "",
                "## Trajectory",
                "",
                f"Matched poses: {trajectory['matched_poses']}",
                f"Translation RMSE: {trajectory['translation']['rmse_m']:.6f} m",
                f"Path drift: {trajectory['path_length']['relative_drift']:.2%}",
            ]
        )

    point_cloud = report["point_cloud"]
    if "nearest" in point_cloud:
        lines.extend(
            [
                "",
                "## Point Cloud",
                "",
                f"Count ratio: {point_cloud['count_ratio']:.3f}",
                f"Centroid drift: {point_cloud['centroid_drift_m']:.6f} m",
                f"Bidirectional nearest RMSE: {point_cloud['nearest']['bidirectional']['rmse_m']:.6f} m",
            ]
        )

    failures = []
    for name in ("baseline_manifest", "metrics", "trajectory", "point_cloud"):
        failures.extend(f"{name}: {error}" for error in report[name].get("errors", []))
    if failures:
        lines.extend(["", "## Failures", ""])
        lines.extend(f"- {failure}" for failure in failures)

    return "\n".join(lines) + "\n"


def build_report(args):
    baseline_dir = Path(args.baseline_dir).expanduser().resolve()
    current_dir = Path(args.current_dir).expanduser().resolve()
    metric_keys = tuple(args.metric_keys) if args.metric_keys else DEFAULT_KEYS

    metrics_report = {"ok": True, "skipped": True, "errors": []}
    if not args.skip_metrics:
        metrics_report = compare_metrics(
            baseline_dir / args.metrics_name,
            current_dir / args.metrics_name,
            metric_keys,
            args.max_metric_regression,
        )

    report = {
        "schema": "gaussian_lic_reproduction_report/v1",
        "dataset": args.dataset,
        "sequence": args.sequence or baseline_dir.name,
        "baseline_dir": str(baseline_dir),
        "current_dir": str(current_dir),
        "baseline_manifest": run_baseline_manifest(args, baseline_dir),
        "metrics": metrics_report,
        "trajectory": run_trajectory_compare(args, baseline_dir, current_dir),
        "point_cloud": run_pointcloud_compare(args, baseline_dir, current_dir),
    }
    report["ok"] = all(
        report[name].get("ok", False)
        for name in ("baseline_manifest", "metrics", "trajectory", "point_cloud")
    )
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description="Build a Gaussian-LIC baseline-vs-current reproduction report.")
    parser.add_argument("--baseline-dir", required=True, help="Archived ROS1 baseline directory")
    parser.add_argument("--current-dir", required=True, help="Current ROS2 result directory")
    parser.add_argument("--dataset", default="fastlivo2")
    parser.add_argument("--sequence")
    parser.add_argument("--output", help="Optional JSON report path")
    parser.add_argument("--markdown", help="Optional Markdown report path")
    parser.add_argument("--json", action="store_true", help="Print full JSON report")

    parser.add_argument("--skip-baseline-manifest", action="store_true")
    parser.add_argument("--min-renders", type=int, default=1)

    parser.add_argument("--skip-metrics", action="store_true")
    parser.add_argument("--metrics-name", default="metrics.json")
    parser.add_argument("--metric-key", action="append", dest="metric_keys")
    parser.add_argument("--max-metric-regression", type=float, default=0.15)

    parser.add_argument("--skip-trajectory", action="store_true")
    parser.add_argument("--trajectory-name", default="trajectory.tum")
    parser.add_argument("--trajectory-align", choices=("none", "first"), default="none")
    parser.add_argument("--max-association-dt", type=float, default=0.02)
    parser.add_argument("--min-trajectory-matches", type=int, default=2)
    parser.add_argument("--min-trajectory-coverage", type=float, default=0.8)
    parser.add_argument("--max-trajectory-rmse-m", type=float, default=0.05)
    parser.add_argument("--max-trajectory-mean-m", type=float, default=0.03)
    parser.add_argument("--max-trajectory-error-m", type=float, default=0.15)
    parser.add_argument("--max-trajectory-path-drift", type=float, default=0.05)

    parser.add_argument("--skip-pointcloud", action="store_true")
    parser.add_argument("--baseline-point-cloud")
    parser.add_argument("--current-point-cloud")
    parser.add_argument("--max-pointcloud-points", type=int, default=200000)
    parser.add_argument("--pointcloud-voxel-size", type=float, default=0.05)
    parser.add_argument("--pointcloud-align", choices=("none", "centroid"), default="none")
    parser.add_argument("--max-nearest-m", type=float, default=0.5)
    parser.add_argument("--min-point-count-ratio", type=float, default=0.5)
    parser.add_argument("--max-point-count-ratio", type=float, default=2.0)
    parser.add_argument("--max-centroid-drift-m", type=float, default=0.2)
    parser.add_argument("--max-chamfer-rmse-m", type=float, default=0.15)
    parser.add_argument("--max-chamfer-mean-m", type=float, default=0.1)
    parser.add_argument("--max-chamfer-max-m", type=float, default=0.5)
    parser.add_argument("--max-unmatched-ratio", type=float, default=0.05)
    parser.add_argument("--max-mean-rgb-drift", type=float, default=40.0)
    parser.add_argument(
        "--derive-gaussian-rgb",
        action="store_true",
        help="Derive RGB means from Gaussian PLY f_dc_0..2 coefficients when explicit RGB is absent",
    )
    args = parser.parse_args(argv)

    try:
        report = build_report(args)
    except Exception as exc:  # noqa: BLE001 - CLI should report unexpected assembly failures uniformly.
        print(f"reproduction report failed: {exc}", file=sys.stderr)
        return 2

    if args.output:
        output_path = Path(args.output)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    if args.markdown:
        markdown_path = Path(args.markdown)
        markdown_path.parent.mkdir(parents=True, exist_ok=True)
        markdown_path.write_text(render_markdown(report), encoding="utf-8")

    if args.json or not report["ok"]:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        print(
            "reproduction report OK: "
            f"sequence={report['sequence']} "
            f"metrics={gate_status(report['metrics'])} "
            f"trajectory={gate_status(report['trajectory'])} "
            f"point_cloud={gate_status(report['point_cloud'])}"
        )

    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
