#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
from types import SimpleNamespace
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

from baseline_manifest import REQUIRED_FILES, build_manifest
from reproduction_report import build_report as build_reproduction_report


DEFAULT_SEQUENCE = "CBD_Building_01"
DEFAULT_DATASET_ROOT = "/home/frank/data/fast_livo"
DEFAULT_BASELINE_DIR = "baseline/fastlivo2/CBD_Building_01"
DEFAULT_RESULTS_DIR = "results/fastlivo2/current"
SOURCE_URLS = {
    "gaussian_lic": "https://github.com/APRIL-ZJU/Gaussian-LIC",
    "gaussian_lic_raw_readme": "https://raw.githubusercontent.com/APRIL-ZJU/Gaussian-LIC/master/README.md",
    "gaussian_lic2_page": "https://xingxingzuo.github.io/gaussian_lic2/",
    "fast_livo2": "https://github.com/hku-mars/FAST-LIVO2",
    "fast_livo2_raw_readme": "https://raw.githubusercontent.com/hku-mars/FAST-LIVO2/main/README.md",
    "fast_livo2_sharepoint": (
        "https://connecthkuhk-my.sharepoint.com/:f:/g/personal/"
        "zhengcr_connect_hku_hk/ErdFNQtjMxZOorYKDTtK4ugBkogXfq1OfDm90GECouuIQA?e=KngY9Z"
    ),
    "fast_livo2_google_drive": "https://drive.google.com/drive/folders/1bf5LQ8iSxw-fD8BObZmouw7lRxNacfrA?usp=drive_link",
}


def utc_now():
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def path_entry(path):
    entry = {
        "path": str(path),
        "exists": path.exists(),
        "is_file": path.is_file(),
        "is_dir": path.is_dir(),
    }
    if path.is_file():
        entry["bytes"] = path.stat().st_size
    return entry


def limited_rglob(root, pattern, limit):
    matches = []
    for path in sorted(root.rglob(pattern)):
        matches.append(path)
        if len(matches) >= limit:
            break
    return matches


def sequence_matches(path, sequence):
    normalized_sequence = sequence.lower().replace("-", "_")
    normalized_name = path.name.lower().replace("-", "_")
    normalized_parts = "/".join(part.lower().replace("-", "_") for part in path.parts)
    return normalized_sequence in normalized_name or normalized_sequence in normalized_parts


def scan_dataset_root(root, sequence, max_candidates):
    root = root.expanduser().resolve()
    report = {
        "root": str(root),
        "exists": root.exists(),
        "is_dir": root.is_dir(),
        "sequence": sequence,
        "ros1_bags": [],
        "rosbag2_dirs": [],
        "parsed_sequence_dirs": [],
        "sequence_candidates": [],
        "ok": False,
        "errors": [],
    }
    if not root.exists():
        report["errors"].append(f"dataset root does not exist: {root}")
        return report
    if not root.is_dir():
        report["errors"].append(f"dataset root is not a directory: {root}")
        return report

    ros1_bags = limited_rglob(root, "*.bag", max_candidates)
    metadata_files = limited_rglob(root, "metadata.yaml", max_candidates)
    parsed_markers = []
    for marker_name in ("all_image", "all_pcd_body"):
        parsed_markers.extend(limited_rglob(root, marker_name, max_candidates))

    for bag in ros1_bags:
        report["ros1_bags"].append(path_entry(bag))
    for metadata in metadata_files:
        bag_dir = metadata.parent
        storage_files = sorted(
            item for item in bag_dir.iterdir()
            if item.is_file() and item.suffix in (".db3", ".mcap")
        )
        entry = path_entry(bag_dir)
        entry["metadata"] = str(metadata)
        entry["storage_files"] = [path_entry(item) for item in storage_files[:max_candidates]]
        entry["storage_file_count"] = len(storage_files)
        report["rosbag2_dirs"].append(entry)
    for marker in sorted(set(parsed_markers)):
        entry = path_entry(marker.parent)
        entry["marker"] = marker.name
        report["parsed_sequence_dirs"].append(entry)

    candidates = []
    for bag in ros1_bags:
        if sequence_matches(bag, sequence):
            candidates.append(path_entry(bag))
    for metadata in metadata_files:
        if sequence_matches(metadata.parent, sequence):
            candidates.append(path_entry(metadata.parent))
    for marker in parsed_markers:
        if sequence_matches(marker.parent, sequence):
            candidates.append(path_entry(marker.parent))
    report["sequence_candidates"] = candidates[:max_candidates]

    has_data = bool(report["ros1_bags"] or report["rosbag2_dirs"] or report["parsed_sequence_dirs"])
    has_sequence = bool(report["sequence_candidates"])
    if not has_data:
        report["errors"].append(f"no ROS1 bags, rosbag2 metadata, or parsed FAST-LIVO2 directories found under {root}")
    if not has_sequence:
        report["errors"].append(f"no candidate data found for sequence {sequence}")
    report["ok"] = has_data and has_sequence
    return report


def check_baseline_dir(baseline_dir, dataset, sequence, min_renders):
    baseline_dir = baseline_dir.expanduser().resolve()
    report = {
        "path": str(baseline_dir),
        "exists": baseline_dir.exists(),
        "required_files": list(REQUIRED_FILES),
        "required_directory": "renders",
        "ok": False,
        "errors": [],
    }
    if not baseline_dir.exists():
        report["errors"].append(f"baseline directory does not exist: {baseline_dir}")
        report["missing"] = list(REQUIRED_FILES) + ["renders/"]
        return report
    if not baseline_dir.is_dir():
        report["errors"].append(f"baseline path is not a directory: {baseline_dir}")
        return report

    args = SimpleNamespace(
        baseline=str(baseline_dir),
        dataset=dataset,
        sequence=sequence,
        min_renders=min_renders,
    )
    try:
        manifest = build_manifest(args)
    except Exception as exc:  # noqa: BLE001 - readiness should report malformed artifact dirs uniformly.
        report["errors"].append(str(exc))
        return report

    report["manifest"] = manifest
    report["ok"] = bool(manifest.get("ok"))
    report["errors"].extend(manifest.get("errors", []))
    return report


def check_results_dir(results_dir):
    results_dir = results_dir.expanduser().resolve()
    required = ("trajectory.tum", "point_cloud.ply", "metrics.json")
    report = {
        "path": str(results_dir),
        "exists": results_dir.exists(),
        "required_files": list(required),
        "files": {},
        "ok": False,
        "errors": [],
    }
    if not results_dir.exists():
        report["errors"].append(f"results directory does not exist: {results_dir}")
        return report
    if not results_dir.is_dir():
        report["errors"].append(f"results path is not a directory: {results_dir}")
        return report
    for name in required:
        entry = path_entry(results_dir / name)
        report["files"][name] = entry
        if not entry["is_file"]:
            report["errors"].append(f"missing current result file: {name}")
    report["ok"] = not report["errors"]
    return report


def current_results_arg_was_set(argv):
    return any(item == "--current-results-dir" or item.startswith("--current-results-dir=") for item in argv)


def file_mtime(path):
    try:
        return path.stat().st_mtime
    except OSError:
        return 0.0


def strict_report_ok(path):
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except Exception:  # noqa: BLE001 - discovery should fall back to file presence if a report is malformed.
        return False
    return bool(data.get("ok"))


def current_results_score(path):
    required = ("trajectory.tum", "point_cloud.ply", "metrics.json")
    if not path.is_dir() or not all((path / name).is_file() for name in required):
        return None

    strict_ok = strict_report_ok(path / "reproduction_report_strict.json")
    readiness_ok = strict_report_ok(path / "baseline_readiness_strict.json")
    quality_ready = (path / "quality_eval.json").is_file() or (path / "renders").is_dir()
    latest_mtime = max(
        file_mtime(path / "reproduction_report_strict.json"),
        file_mtime(path / "baseline_readiness_strict.json"),
        file_mtime(path / "metrics.json"),
        file_mtime(path / "point_cloud.ply"),
        file_mtime(path),
    )
    return (
        1 if strict_ok else 0,
        1 if readiness_ok else 0,
        1 if quality_ready else 0,
        latest_mtime,
    )


def resolve_default_current_results_dir(default_results, sequence):
    """Select an archived current artifact when the historical default path is empty.

    `results/fastlivo2/current` is useful for ad-hoc local runs, while the
    strict CBD pipeline archives sequence-specific directories.  Auto-discovery
    keeps the no-argument readiness command aligned with those archived strict
    artifacts without changing behavior when the caller supplies an explicit
    current-results directory.
    """
    default_path = Path(default_results).expanduser()
    if default_path.exists():
        return default_path

    repo_root = Path(__file__).resolve().parents[1]
    search_root = repo_root / "results" / "fastlivo2"
    if not search_root.is_dir():
        return default_path

    sequence_current = search_root / f"{sequence}_current"
    sequence_strict_current = search_root / f"{sequence}_strict_current"
    for candidate in (sequence_current, sequence_strict_current):
        if current_results_score(candidate) is not None:
            return candidate

    candidates = []
    prefix = f"{sequence}_"
    for path in search_root.iterdir():
        if not path.is_dir():
            continue
        name = path.name
        if not (name == sequence or name.startswith(prefix)):
            continue
        if "current" not in name:
            continue
        score = current_results_score(path)
        if score is None:
            continue
        candidates.append((score, path))
    if not candidates:
        return default_path
    return max(candidates, key=lambda item: item[0])[1]


def reproduction_args(args, baseline_dir, current_results):
    strict_max_regression = args.strict_max_regression
    return SimpleNamespace(
        baseline_dir=str(baseline_dir),
        current_dir=str(current_results),
        dataset=args.dataset,
        sequence=args.sequence,
        output=None,
        markdown=None,
        json=False,
        strict=True,
        strict_max_regression=strict_max_regression,
        baseline_renders_dir=args.baseline_renders_dir,
        current_renders_dir=args.current_renders_dir,
        min_render_pairs=args.min_render_pairs,
        max_render_pairs=args.max_render_pairs,
        min_render_pair_psnr=args.min_render_pair_psnr,
        min_render_pair_ssim=args.min_render_pair_ssim,
        max_render_pair_failure_ratio=args.max_render_pair_failure_ratio,
        min_mean_render_pair_psnr=args.min_mean_render_pair_psnr,
        min_mean_render_pair_ssim=args.min_mean_render_pair_ssim,
        skip_baseline_manifest=False,
        min_renders=args.min_renders,
        skip_metrics=False,
        metrics_name="metrics.json",
        metric_keys=None,
        max_metric_regression=strict_max_regression,
        skip_trajectory=False,
        trajectory_name="trajectory.tum",
        trajectory_align="none",
        max_association_dt=0.02,
        min_trajectory_matches=2,
        min_trajectory_coverage=0.8,
        max_trajectory_rmse_m=0.05,
        max_trajectory_mean_m=0.03,
        max_trajectory_error_m=0.15,
        max_trajectory_path_drift=0.05,
        time_offset_sweep_min=0.0,
        time_offset_sweep_max=0.0,
        time_offset_sweep_step=0.0,
        time_offset_sweep_min_matches=0,
        skip_pointcloud=False,
        baseline_point_cloud=None,
        current_point_cloud=None,
        max_pointcloud_points=200000,
        pointcloud_voxel_size=0.05,
        pointcloud_align="none",
        max_nearest_m=0.5,
        min_point_count_ratio=0.5,
        max_point_count_ratio=2.0,
        max_centroid_drift_m=0.2,
        max_chamfer_rmse_m=0.15,
        max_chamfer_mean_m=0.1,
        max_chamfer_max_m=0.5,
        max_unmatched_ratio=0.05,
        max_mean_rgb_drift=40.0,
        derive_gaussian_rgb=False,
        gaussian_color_baseline_point_cloud=None,
        gaussian_color_current_point_cloud=None,
        gaussian_color_max_mean_rgb_drift=40.0,
    )


def compact_gate_errors(report):
    errors = []
    for name in ("baseline_manifest", "metrics", "novel_view_quality", "trajectory", "point_cloud", "gaussian_color"):
        section = report.get(name, {})
        errors.extend(f"{name}: {error}" for error in section.get("errors", []))
    return errors


def check_strict_reproduction(args, baseline_dir, current_results):
    if not args.strict:
        return {"ok": True, "skipped": True, "errors": []}
    baseline_dir = baseline_dir.expanduser().resolve()
    current_results = current_results.expanduser().resolve()
    if not baseline_dir.is_dir() or not current_results.is_dir():
        return {
            "ok": False,
            "strict": True,
            "errors": ["strict reproduction requires existing baseline and current result directories"],
        }
    try:
        report = build_reproduction_report(reproduction_args(args, baseline_dir, current_results))
    except Exception as exc:  # noqa: BLE001 - readiness should turn strict report failures into blockers.
        return {"ok": False, "strict": True, "errors": [str(exc)]}
    return {
        "ok": bool(report.get("ok")),
        "strict": True,
        "report": report,
        "errors": compact_gate_errors(report),
    }


def load_fetch_attempt(dataset_root, sequence):
    root = dataset_root.expanduser().resolve()
    path = root / f"{sequence}.fetch.json"
    report = {
        "path": str(path),
        "exists": path.is_file(),
        "ok": False,
        "error": None,
    }
    if not path.is_file():
        return report
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except Exception as exc:  # noqa: BLE001 - readiness should not fail on a malformed previous attempt.
        report["error"] = f"could not parse fetch manifest: {exc}"
        return report
    report["manifest"] = data
    report["ok"] = bool(data.get("ok"))
    report["error"] = data.get("error")
    output_path = Path(data.get("output_path") or root / f"{sequence}.bag").expanduser()
    if not output_path.is_absolute():
        output_path = root / output_path
    expected_bytes = data.get("entry", {}).get("bytes")
    if output_path.is_file():
        actual_bytes = output_path.stat().st_size
        size_matches = expected_bytes is None or int(expected_bytes) == actual_bytes
        report["local_artifact"] = path_entry(output_path)
        report["local_artifact"]["expected_bytes"] = expected_bytes
        report["local_artifact"]["size_matches"] = size_matches
        if size_matches:
            report["ok"] = True
            report["previous_error"] = report["error"]
            report["error"] = None
    return report


def probe_url(name, url, timeout):
    request = Request(
        url,
        method="HEAD",
        headers={
            "User-Agent": "gaussian-lic-ros2-baseline-readiness/1.0",
            "Accept": "*/*",
        },
    )
    result = {
        "name": name,
        "url": url,
        "ok": False,
        "http_status": None,
        "final_url": None,
        "content_type": None,
        "content_length": None,
        "error": None,
    }
    try:
        with urlopen(request, timeout=timeout) as response:
            result["http_status"] = response.status
            result["final_url"] = response.geturl()
            result["content_type"] = response.headers.get("content-type")
            result["content_length"] = response.headers.get("content-length")
            result["ok"] = 200 <= response.status < 400
    except HTTPError as exc:
        result["http_status"] = exc.code
        result["final_url"] = exc.geturl()
        result["content_type"] = exc.headers.get("content-type") if exc.headers else None
        result["content_length"] = exc.headers.get("content-length") if exc.headers else None
        result["error"] = str(exc)
    except (OSError, URLError) as exc:
        result["error"] = str(exc)
    return result


def probe_sources(timeout):
    return [probe_url(name, url, timeout) for name, url in SOURCE_URLS.items()]


def local_upstream_status(root):
    readme = root / "external" / "Gaussian-LIC" / "README.md"
    report = {
        "path": str(readme),
        "exists": readme.is_file(),
        "lic2_release_notice": False,
        "lic2_dataset_release_checked": None,
        "fast_livo2_quickstart": False,
        "errors": [],
    }
    if not readme.is_file():
        report["errors"].append(f"missing upstream README: {readme}")
        return report
    text = readme.read_text(encoding="utf-8", errors="replace")
    report["lic2_release_notice"] = "Gaussian-LIC2 is released" in text
    report["fast_livo2_quickstart"] = "CBD_Building_01" in text and "FAST-LIVO2" in text
    if "- [ ] Release our Gaussian-LIC2 dataset" in text:
        report["lic2_dataset_release_checked"] = False
    elif "- [x] Release our Gaussian-LIC2 dataset" in text:
        report["lic2_dataset_release_checked"] = True
    return report


def build_next_actions(report):
    actions = []
    dataset = report["dataset"]
    baseline = report["baseline"]
    if not dataset["ok"]:
        fetch_attempt = report.get("fetch_attempt", {})
        if fetch_attempt.get("exists") and fetch_attempt.get("error"):
            short_error = fetch_attempt["error"].split(" Please try", 1)[0]
            actions.append(
                "Retry the FAST-LIVO2 sequence fetch or use another official mirror; "
                f"last fetch error: {short_error}"
            )
        else:
            actions.append(
                "Download the FAST-LIVO2 sequence with: "
                f"./scripts/fetch_fastlivo2_sequence.py --sequence {dataset['sequence']} "
                f"--output-dir {dataset['root']}"
            )
    if dataset["ok"] and not baseline["ok"]:
        actions.append(
            "Run upstream Gaussian-LIC/Gaussian-LIC2 on the downloaded bag and archive "
            f"trajectory, point cloud, renders, metrics, and log under {baseline['path']}"
        )
    if baseline["ok"] and not report["current_results"]["ok"]:
        actions.append(
            "Run scripts/collect_current_results.sh on the selected ROS2 bag and write current "
            f"artifacts under {report['current_results']['path']}"
        )
    if baseline["ok"] and report["current_results"]["ok"]:
        strict = report.get("strict_reproduction", {})
        if strict.get("skipped"):
            actions.append(
                "Run scripts/reproduction_report.py with the baseline and current result directories."
            )
        elif not strict.get("ok"):
            actions.append(
                "Fix strict reproduction blockers until PSNR/SSIM/LPIPS, trajectory, and Chamfer gates pass."
            )
    return actions


def render_markdown(report):
    def status(ok):
        return "PASS" if ok else "BLOCKED"

    lines = [
        "# Gaussian-LIC Baseline Readiness",
        "",
        f"Status: {status(report['ok'])}",
        f"Generated: {report['generated_at']}",
        f"Dataset: {report['dataset_name']}",
        f"Sequence: {report['sequence']}",
        "",
        "| Gate | Status | Path |",
        "| --- | --- | --- |",
        f"| FAST-LIVO2 data | {status(report['dataset']['ok'])} | `{report['dataset']['root']}` |",
        f"| ROS1 baseline artifacts | {status(report['baseline']['ok'])} | `{report['baseline']['path']}` |",
        f"| ROS2 current artifacts | {status(report['current_results']['ok'])} | `{report['current_results']['path']}` |",
    ]
    strict = report.get("strict_reproduction", {"ok": True, "skipped": True})
    if not strict.get("skipped"):
        lines.append(f"| Strict reproduction report | {status(strict['ok'])} | `{report['current_results']['path']}` |")
    source_ready = report["upstream"]["lic2_release_notice"] and report["upstream"]["fast_livo2_quickstart"]
    lines.append(f"| Upstream LIC2 source/readme | {status(source_ready)} | `{report['upstream']['path']}` |")

    if report["dataset"]["sequence_candidates"]:
        lines.extend(["", "## Dataset Candidates", ""])
        for item in report["dataset"]["sequence_candidates"]:
            size = f" ({item.get('bytes', 0)} bytes)" if item.get("bytes") is not None else ""
            lines.append(f"- `{item['path']}`{size}")

    failures = []
    for section in ("dataset", "baseline", "current_results"):
        failures.extend(f"{section}: {error}" for error in report[section].get("errors", []))
    failures.extend(f"strict_reproduction: {error}" for error in report.get("strict_reproduction", {}).get("errors", []))
    failures.extend(f"upstream: {error}" for error in report["upstream"].get("errors", []))
    if report["fetch_attempt"].get("error"):
        failures.append(f"fetch_attempt: {report['fetch_attempt']['error']}")
    if failures:
        lines.extend(["", "## Blockers", ""])
        lines.extend(f"- {failure}" for failure in failures)

    if report["next_actions"]:
        lines.extend(["", "## Next Actions", ""])
        lines.extend(f"- {action}" for action in report["next_actions"])

    if report["source_probes"]:
        lines.extend(["", "## Source Probes", "", "| Source | HTTP | OK |", "| --- | ---: | --- |"])
        for item in report["source_probes"]:
            lines.append(f"| {item['name']} | {item.get('http_status') or ''} | {status(item['ok'])} |")

    return "\n".join(lines) + "\n"


def build_report(args):
    repo_root = Path(__file__).resolve().parents[1]
    dataset_root = Path(args.dataset_root)
    baseline_dir = Path(args.baseline_dir)
    current_results = Path(args.current_results_dir)
    dataset = scan_dataset_root(dataset_root, args.sequence, args.max_candidates)
    fetch_attempt = load_fetch_attempt(dataset_root, args.sequence)
    baseline = check_baseline_dir(baseline_dir, args.dataset, args.sequence, args.min_renders)
    current = check_results_dir(current_results)
    strict_reproduction = check_strict_reproduction(args, baseline_dir, current_results)
    report = {
        "schema": "gaussian_lic_baseline_readiness/v1",
        "generated_at": utc_now(),
        "dataset_name": args.dataset,
        "sequence": args.sequence,
        "official_sources": SOURCE_URLS,
        "dataset": dataset,
        "fetch_attempt": fetch_attempt,
        "baseline": baseline,
        "current_results": current,
        "strict_reproduction": strict_reproduction,
        "upstream": local_upstream_status(repo_root),
        "source_probes": probe_sources(args.probe_timeout) if args.probe_sources else [],
    }
    report["ok"] = (
        report["dataset"]["ok"]
        and report["baseline"]["ok"]
        and report["current_results"]["ok"]
        and report["strict_reproduction"]["ok"]
    )
    report["next_actions"] = build_next_actions(report)
    return report


def main(argv=None):
    raw_argv = list(sys.argv[1:] if argv is None else argv)
    parser = argparse.ArgumentParser(
        description="Report whether FAST-LIVO2 data, ROS1 baseline artifacts, and ROS2 current artifacts are ready."
    )
    parser.add_argument("--dataset-root", default=DEFAULT_DATASET_ROOT)
    parser.add_argument("--baseline-dir", default=DEFAULT_BASELINE_DIR)
    parser.add_argument("--current-results-dir", default=DEFAULT_RESULTS_DIR)
    parser.add_argument("--dataset", default="fastlivo2")
    parser.add_argument("--sequence", default=DEFAULT_SEQUENCE)
    parser.add_argument("--min-renders", type=int, default=1)
    parser.add_argument("--max-candidates", type=int, default=20)
    parser.add_argument(
        "--strict",
        action="store_true",
        help="Run the full reproduction report gate, including PSNR/SSIM/LPIPS and render-pair checks.",
    )
    parser.add_argument("--strict-max-regression", type=float, default=0.05)
    parser.add_argument("--baseline-renders-dir", help="Override baseline render image directory for --strict.")
    parser.add_argument("--current-renders-dir", help="Override current render image directory for --strict.")
    parser.add_argument("--min-render-pairs", type=int, default=1)
    parser.add_argument("--max-render-pairs", type=int, default=64)
    parser.add_argument("--min-render-pair-psnr", type=float, default=15.0)
    parser.add_argument("--min-render-pair-ssim", type=float, default=0.4)
    parser.add_argument("--max-render-pair-failure-ratio", type=float, default=0.10)
    parser.add_argument("--min-mean-render-pair-psnr", type=float, default=20.0)
    parser.add_argument("--min-mean-render-pair-ssim", type=float, default=0.75)
    parser.add_argument("--probe-sources", action="store_true", help="Probe official source URLs with HEAD requests.")
    parser.add_argument("--probe-timeout", type=float, default=10.0)
    parser.add_argument("--output", help="Optional JSON report path.")
    parser.add_argument("--markdown", help="Optional Markdown report path.")
    parser.add_argument("--json", action="store_true", help="Print full JSON report.")
    parser.add_argument(
        "--allow-missing",
        action="store_true",
        help="Return success even when readiness gates are blocked. Useful for status jobs.",
    )
    args = parser.parse_args(argv)
    if not current_results_arg_was_set(raw_argv):
        args.current_results_dir = str(
            resolve_default_current_results_dir(args.current_results_dir, args.sequence)
        )

    report = build_report(args)

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
        strict_status = "SKIP" if report["strict_reproduction"].get("skipped") else "PASS"
        print(
            "baseline readiness OK: "
            f"sequence={report['sequence']} "
            f"dataset={report['dataset']['root']} "
            f"baseline={report['baseline']['path']} "
            f"current={report['current_results']['path']} "
            f"strict={strict_status}"
        )

    return 0 if report["ok"] or args.allow_missing else 1


if __name__ == "__main__":
    raise SystemExit(main())
