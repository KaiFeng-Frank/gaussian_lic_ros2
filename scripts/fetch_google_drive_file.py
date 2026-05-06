#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Fetch one public Google Drive file with the repository's resumable-style checks."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import sys
from urllib.request import HTTPCookieProcessor, build_opener

from fetch_fastlivo2_sequence import (
    ensure_enough_space,
    open_download_response,
    stream_response,
    write_manifest,
)


def extract_file_id(value: str) -> str:
    patterns = (
        r"/file/d/([A-Za-z0-9_-]{20,})",
        r"[?&]id=([A-Za-z0-9_-]{20,})",
        r"^([A-Za-z0-9_-]{20,})$",
    )
    for pattern in patterns:
        match = re.search(pattern, value)
        if match:
            return match.group(1)
    raise ValueError(f"could not extract Google Drive file id from: {value}")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--file", required=True, help="Google Drive file URL or file id.")
    parser.add_argument("--output", required=True, help="Output file path.")
    parser.add_argument("--expected-bytes", type=int, help="Optional exact byte count gate.")
    parser.add_argument("--timeout", type=float, default=60.0)
    parser.add_argument("--chunk-size", type=int, default=4 * 1024 * 1024)
    parser.add_argument("--progress-interval-mb", type=int, default=256)
    parser.add_argument("--min-free-gb", type=float, default=5.0)
    parser.add_argument("--skip-existing", action="store_true", default=True)
    parser.add_argument("--manifest", help="Optional JSON manifest path.")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args(argv)

    output = Path(args.output).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    file_id = extract_file_id(args.file)
    result = {
        "schema": "gaussian_lic_google_drive_file_fetch/v1",
        "file": args.file,
        "file_id": file_id,
        "output_path": str(output),
        "expected_bytes": args.expected_bytes,
        "downloaded": False,
        "skipped": False,
        "ok": False,
    }

    if args.skip_existing and output.is_file() and (
        args.expected_bytes is None or output.stat().st_size == args.expected_bytes
    ):
        result["ok"] = True
        result["skipped"] = True
    else:
        try:
            ensure_enough_space(output.parent, args.expected_bytes, args.min_free_gb)
            opener = build_opener(HTTPCookieProcessor())
            response = open_download_response(opener, file_id, args.timeout)
            with response:
                result["download"] = stream_response(
                    response,
                    output,
                    args.expected_bytes,
                    args.chunk_size,
                    args.progress_interval_mb,
                )
            result["ok"] = True
            result["downloaded"] = True
        except Exception as exc:  # noqa: BLE001 - CLI should produce a manifest-friendly error.
            result["error"] = str(exc)
            if args.manifest:
                write_manifest(Path(args.manifest).expanduser().resolve(), result)
            print(f"Google Drive file fetch failed: {exc}", file=sys.stderr)
            return 2

    if args.manifest:
        write_manifest(Path(args.manifest).expanduser().resolve(), result)
    if args.json:
        print(json.dumps(result, indent=2, sort_keys=True))
    else:
        state = "skipped" if result["skipped"] else "downloaded"
        print(f"Google Drive file fetch OK: {state} output={output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
