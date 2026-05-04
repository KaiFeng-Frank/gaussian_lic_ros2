#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later

import argparse
import json
from pathlib import Path
import sys


def load_json(path):
    with Path(path).open("r", encoding="utf-8") as stream:
        return json.load(stream)


def status(value):
    return "PASS" if value else "FAIL"


def format_float(value, digits=6):
    if value is None:
        return ""
    return f"{float(value):.{digits}g}"


def build_summary(artifact_dir):
    root = Path(artifact_dir).expanduser().resolve()
    full_contract = load_json(root / "bag_contract_full.json")
    minimal_contract = load_json(root / "bag_contract_mapper_minimal.json")
    frontend_raw_contract = load_json(root / "bag_contract_frontend_raw.json")
    metrics = load_json(root / "offline" / "metrics.json")

    debug_cloud = metrics.get("debug_cloud", {})
    trajectory = metrics.get("trajectory", {})
    return {
        "artifact_dir": str(root),
        "bag": metrics.get("bag", ""),
        "bag_format": metrics.get("bag_format", ""),
        "storage_identifier": metrics.get("storage_identifier", ""),
        "contracts": {
            "full": bool(full_contract.get("contract_ok")),
            "mapper_minimal": bool(minimal_contract.get("contract_ok")),
            "frontend_raw": bool(frontend_raw_contract.get("contract_ok")),
        },
        "frontend_raw_bag": frontend_raw_contract.get("bag", ""),
        "message_count": int(metrics.get("message_count", 0)),
        "duration_sec": float(metrics.get("duration_sec", 0.0)),
        "trajectory_poses": int(metrics.get("trajectory_poses", 0)),
        "path_length_m": float(trajectory.get("path_length_m", 0.0)),
        "debug_points": int(metrics.get("debug_points", 0)),
        "points_with_color": int(debug_cloud.get("points_with_color", 0)),
        "points_without_color": int(debug_cloud.get("points_without_color", 0)),
        "mean_rgb": debug_cloud.get("mean_rgb", []),
        "ok": (
            bool(full_contract.get("contract_ok"))
            and bool(minimal_contract.get("contract_ok"))
            and bool(frontend_raw_contract.get("contract_ok"))
        ),
    }


def render_markdown(summary):
    mean_rgb = summary["mean_rgb"]
    mean_rgb_text = ", ".join(format_float(value, 3) for value in mean_rgb) if mean_rgb else ""
    lines = [
        "# Gaussian-LIC CI Replay Smoke",
        "",
        f"Status: {status(summary['ok'])}",
        f"Bag: `{summary['bag']}`",
        f"Frontend raw bag: `{summary['frontend_raw_bag']}`",
        f"Format: `{summary['bag_format']}` / `{summary['storage_identifier']}`",
        "",
        "| Gate | Status |",
        "| --- | --- |",
        f"| full bag contract | {status(summary['contracts']['full'])} |",
        f"| mapper_minimal contract | {status(summary['contracts']['mapper_minimal'])} |",
        f"| frontend_raw contract | {status(summary['contracts']['frontend_raw'])} |",
        "",
        "| Metric | Value |",
        "| --- | ---: |",
        f"| messages | {summary['message_count']} |",
        f"| duration_sec | {format_float(summary['duration_sec'])} |",
        f"| trajectory_poses | {summary['trajectory_poses']} |",
        f"| path_length_m | {format_float(summary['path_length_m'])} |",
        f"| debug_points | {summary['debug_points']} |",
        f"| points_with_color | {summary['points_with_color']} |",
        f"| points_without_color | {summary['points_without_color']} |",
        f"| mean_rgb | {mean_rgb_text} |",
    ]
    return "\n".join(lines) + "\n"


def main(argv=None):
    parser = argparse.ArgumentParser(description="Render a Markdown summary for CI replay smoke artifacts.")
    parser.add_argument("--artifact-dir", required=True)
    parser.add_argument("--output", help="Optional Markdown output path")
    args = parser.parse_args(argv)

    try:
        summary = build_summary(args.artifact_dir)
        markdown = render_markdown(summary)
    except Exception as exc:  # noqa: BLE001 - CLI should report malformed replay artifacts uniformly.
        print(f"replay artifact summary failed: {exc}", file=sys.stderr)
        return 2

    if args.output:
        output_path = Path(args.output)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(markdown, encoding="utf-8")
    else:
        print(markdown, end="")

    return 0 if summary["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
