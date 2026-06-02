#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Check the full-dataset strict parity evidence matrix."""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
from pathlib import Path
from typing import Any


DEFAULT_MANIFEST = Path("docs/strict_parity_matrix.json")


def load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        data = json.load(handle)
    if not isinstance(data, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return data


def lookup(data: dict[str, Any], path: str) -> Any:
    current: Any = data
    for part in path.split("."):
        if not isinstance(current, dict) or part not in current:
            raise KeyError(path)
        current = current[part]
    return current


def as_number(value: Any, path: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"{path} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{path} must be finite")
    return result


def check_thresholds(report: dict[str, Any], thresholds: list[dict[str, Any]]) -> list[str]:
    errors: list[str] = []
    for threshold in thresholds:
        path = str(threshold["path"])
        try:
            value = as_number(lookup(report, path), path)
        except (KeyError, TypeError, ValueError) as exc:
            errors.append(str(exc))
            continue

        if "min" in threshold and value < float(threshold["min"]):
            errors.append(f"{path} {value:g} < min {float(threshold['min']):g}")
        if "max" in threshold and value > float(threshold["max"]):
            errors.append(f"{path} {value:g} > max {float(threshold['max']):g}")
        if "eq" in threshold and value != float(threshold["eq"]):
            errors.append(f"{path} {value:g} != {float(threshold['eq']):g}")
    return errors


def check_required_gates(report: dict[str, Any], gates: list[str]) -> list[str]:
    errors: list[str] = []
    for gate in gates:
        item = report.get(gate)
        if not isinstance(item, dict):
            errors.append(f"{gate} gate is missing")
            continue
        if not item.get("ok"):
            errors.append(f"{gate} gate is not OK")
    return errors


def check_reproduction(entry: dict[str, Any], root: Path) -> dict[str, Any]:
    report_path = root / str(entry["report"])
    result: dict[str, Any] = {
        "id": entry["id"],
        "kind": entry["kind"],
        "profile": entry.get("profile", ""),
        "sequence": entry.get("sequence", ""),
        "required": bool(entry.get("required", True)),
        "report": str(report_path),
        "ok": False,
        "errors": [],
    }
    if not report_path.is_file():
        result["errors"].append(f"missing report: {report_path}")
        return result

    try:
        report = load_json(report_path)
    except Exception as exc:  # noqa: BLE001
        result["errors"].append(f"could not load report: {exc}")
        return result

    if not report.get("ok"):
        result["errors"].append("reproduction report ok=false")

    if entry.get("strict") and not report.get("strict"):
        result["errors"].append("strict evidence requires report.strict=true")

    gates = list(entry.get("gates", ("baseline_manifest", "metrics", "trajectory", "point_cloud")))
    if entry.get("strict"):
        gates.append("novel_view_quality")
    result["errors"].extend(check_required_gates(report, gates))

    min_render_pairs = entry.get("min_render_pairs")
    if min_render_pairs is not None:
        try:
            matched_pairs = as_number(
                lookup(report, "novel_view_quality.render_pairs.matched_pair_count"),
                "novel_view_quality.render_pairs.matched_pair_count",
            )
            if matched_pairs < float(min_render_pairs):
                result["errors"].append(
                    f"matched render pairs {matched_pairs:g} < min {float(min_render_pairs):g}"
                )
            result["matched_render_pairs"] = matched_pairs
        except (KeyError, TypeError, ValueError) as exc:
            result["errors"].append(str(exc))

    result["errors"].extend(check_thresholds(report, list(entry.get("thresholds", ()))))
    result["ok"] = not result["errors"]
    return result


def check_native_tracking(entry: dict[str, Any], root: Path) -> dict[str, Any]:
    report_path = root / str(entry["report"])
    result: dict[str, Any] = {
        "id": entry["id"],
        "kind": entry["kind"],
        "profile": entry.get("profile", ""),
        "sequence": entry.get("sequence", ""),
        "required": bool(entry.get("required", True)),
        "report": str(report_path),
        "ok": False,
        "errors": [],
    }
    if not report_path.is_file():
        result["errors"].append(f"missing report: {report_path}")
        return result

    try:
        report = load_json(report_path)
    except Exception as exc:  # noqa: BLE001
        result["errors"].append(f"could not load report: {exc}")
        return result

    if not report.get("ok"):
        result["errors"].append("native tracking report ok=false")
    result["errors"].extend(check_thresholds(report, list(entry.get("thresholds", ()))))

    if entry.get("requires_reference_parity"):
        compare = report.get("trajectory_compare")
        if not isinstance(compare, dict) or not compare.get("ok"):
            result["errors"].append("required reference trajectory parity is missing or not OK")

    result["ok"] = not result["errors"]
    return result


def check_pending(entry: dict[str, Any]) -> dict[str, Any]:
    required = bool(entry.get("required", True))
    reason = str(entry.get("reason", "pending strict evidence"))
    return {
        "id": entry["id"],
        "kind": entry["kind"],
        "profile": entry.get("profile", ""),
        "sequence": entry.get("sequence", ""),
        "required": required,
        "report": "",
        "ok": not required,
        "errors": [reason] if required else [],
    }


def check_script(entry: dict[str, Any], root: Path) -> dict[str, Any]:
    command = entry.get("command")
    result: dict[str, Any] = {
        "id": entry["id"],
        "kind": entry["kind"],
        "profile": entry.get("profile", ""),
        "sequence": entry.get("sequence", ""),
        "required": bool(entry.get("required", True)),
        "report": "",
        "ok": False,
        "errors": [],
    }
    if not isinstance(command, list) or not command or not all(
        isinstance(part, str) and part for part in command
    ):
        result["errors"].append("script_check evidence requires a non-empty string command list")
        return result
    timeout_sec = int(entry.get("timeout_sec", 60))
    result["report"] = " ".join(command)
    try:
        completed = subprocess.run(
            command,
            cwd=root,
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout_sec,
        )
    except subprocess.TimeoutExpired:
        result["errors"].append(f"script_check timed out after {timeout_sec}s")
        return result
    except OSError as exc:
        result["errors"].append(f"could not run script_check: {exc}")
        return result
    result["stdout"] = completed.stdout.strip()
    result["stderr"] = completed.stderr.strip()
    if completed.returncode != 0:
        result["errors"].append(
            f"script_check exited {completed.returncode}: "
            f"{completed.stderr.strip() or completed.stdout.strip()}"
        )
    result["ok"] = not result["errors"]
    return result


def check_entry(entry: dict[str, Any], root: Path) -> dict[str, Any]:
    kind = entry.get("kind")
    if kind == "reproduction_report":
        return check_reproduction(entry, root)
    if kind == "native_tracking_report":
        return check_native_tracking(entry, root)
    if kind == "script_check":
        return check_script(entry, root)
    if kind == "pending":
        return check_pending(entry)
    return {
        "id": entry.get("id", "<unknown>"),
        "kind": kind,
        "profile": entry.get("profile", ""),
        "sequence": entry.get("sequence", ""),
        "required": bool(entry.get("required", True)),
        "report": "",
        "ok": False,
        "errors": [f"unknown evidence kind: {kind!r}"],
    }


def summarize(manifest: dict[str, Any], entries: list[dict[str, Any]]) -> dict[str, Any]:
    required = [entry for entry in entries if entry.get("required")]
    required_ok = [entry for entry in required if entry.get("ok")]
    profiles = list(manifest.get("target_profiles", ()))
    covered_profiles = sorted(
        {
            str(entry.get("profile", ""))
            for entry in required_ok
            if entry.get("profile")
        }
    )
    missing_profiles = [profile for profile in profiles if profile not in covered_profiles]
    return {
        "schema": "gaussian_lic_strict_parity_matrix_report/v1",
        "manifest": manifest.get("schema", ""),
        "ok": len(required_ok) == len(required) and not missing_profiles,
        "required_count": len(required),
        "required_ok_count": len(required_ok),
        "target_profiles": profiles,
        "covered_profiles": covered_profiles,
        "missing_profiles": missing_profiles,
        "entries": entries,
    }


def render_markdown(report: dict[str, Any]) -> str:
    lines = [
        "# Strict Parity Evidence Matrix",
        "",
        f"Status: {'PASS' if report['ok'] else 'INCOMPLETE'}",
        f"Required evidence: {report['required_ok_count']}/{report['required_count']}",
        f"Covered profiles: {', '.join(report['covered_profiles']) or 'none'}",
    ]
    if report.get("missing_profiles"):
        lines.append(f"Missing profiles: {', '.join(report['missing_profiles'])}")
    lines.extend(["", "| Evidence | Kind | Required | Status |", "| --- | --- | --- | --- |"])
    for entry in report["entries"]:
        status = "PASS" if entry.get("ok") else "FAIL"
        lines.append(
            f"| `{entry['id']}` | {entry['kind']} | {str(entry['required']).lower()} | {status} |"
        )
        for error in entry.get("errors", []):
            lines.append(f"| | | | {error} |")
    return "\n".join(lines) + "\n"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--repo-root", type=Path, default=Path("."))
    parser.add_argument("--output", type=Path, help="Optional JSON report path.")
    parser.add_argument("--markdown", type=Path, help="Optional Markdown report path.")
    parser.add_argument(
        "--allow-incomplete",
        action="store_true",
        help="Write the report but return success even when required evidence is missing.",
    )
    args = parser.parse_args(argv)

    root = args.repo_root.resolve()
    manifest_path = args.manifest if args.manifest.is_absolute() else root / args.manifest
    try:
        manifest = load_json(manifest_path)
    except Exception as exc:  # noqa: BLE001
        print(f"strict parity matrix failed: {exc}", file=sys.stderr)
        return 2

    entries = [check_entry(entry, root) for entry in manifest.get("evidence", ())]
    report = summarize(manifest, entries)

    if args.output:
        output = args.output if args.output.is_absolute() else root / args.output
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    if args.markdown:
        markdown = args.markdown if args.markdown.is_absolute() else root / args.markdown
        markdown.parent.mkdir(parents=True, exist_ok=True)
        markdown.write_text(render_markdown(report), encoding="utf-8")

    print(
        "strict parity matrix "
        f"{'PASS' if report['ok'] else 'INCOMPLETE'}: "
        f"required={report['required_ok_count']}/{report['required_count']} "
        f"covered_profiles={','.join(report['covered_profiles']) or 'none'}"
    )
    if not report["ok"]:
        for entry in entries:
            if entry.get("required") and not entry.get("ok"):
                messages = "; ".join(entry.get("errors", ()))
                print(f"- {entry['id']}: {messages}", file=sys.stderr)
        if report["missing_profiles"]:
            print(f"- missing profiles: {', '.join(report['missing_profiles'])}", file=sys.stderr)
    if report["ok"] or args.allow_incomplete:
        return 0
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
