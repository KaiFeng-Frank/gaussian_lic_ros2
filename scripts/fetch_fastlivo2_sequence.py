#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later

import argparse
from email.message import Message
import html
import json
from pathlib import Path
import re
import sys
import time
from urllib.error import HTTPError, URLError
from urllib.parse import urlencode
from urllib.request import HTTPCookieProcessor, Request, build_opener


FOLDER_ID = "1bf5LQ8iSxw-fD8BObZmouw7lRxNacfrA"
FOLDER_URL = f"https://drive.google.com/drive/folders/{FOLDER_ID}?usp=drive_link"
DEFAULT_SEQUENCE = "CBD_Building_01"
USER_AGENT = "gaussian-lic-ros2-fastlivo2-fetch/1.0"


def normalize_drive_html(text):
    normalized = html.unescape(text)
    replacements = {
        "\\x22": '"',
        "\\x5b": "[",
        "\\x5d": "]",
        "\\/": "/",
        "\\u003d": "=",
        "\\u0026": "&",
    }
    for old, new in replacements.items():
        normalized = normalized.replace(old, new)
    return normalized


def discover_folder_entries(folder_html):
    normalized = normalize_drive_html(folder_html)
    pattern = re.compile(
        r'\["(?P<id>[A-Za-z0-9_-]{20,})",\["' + re.escape(FOLDER_ID) +
        r'"\],"(?P<name>[^"]+\.bag)","application/octet-stream",(?P<tail>.{0,700})'
    )
    entries = {}
    for match in pattern.finditer(normalized):
        tail = match.group("tail")
        numbers = [int(item) for item in re.findall(r"\b[1-9][0-9]{6,}\b", tail)]
        plausible_sizes = [value for value in numbers if 1_000_000 <= value <= 200_000_000_000]
        name = match.group("name")
        entries[name] = {
            "name": name,
            "file_id": match.group("id"),
            "bytes": min(plausible_sizes) if plausible_sizes else None,
            "url": f"https://drive.google.com/file/d/{match.group('id')}/view",
        }
    return sorted(entries.values(), key=lambda item: item["name"])


def fetch_text(opener, url, timeout):
    request = Request(url, headers={"User-Agent": USER_AGENT})
    with opener.open(request, timeout=timeout) as response:
        return response.read().decode("utf-8", errors="replace")


def get_header(headers, name):
    if isinstance(headers, Message):
        return headers.get(name)
    return headers.get(name)


def parse_download_form(page):
    action_match = re.search(r'<form[^>]*id="download-form"[^>]*action="([^"]+)"', page)
    if not action_match:
        raise RuntimeError("Google Drive warning page did not contain a download form")
    params = {}
    for match in re.finditer(r'<input[^>]*type="hidden"[^>]*name="([^"]+)"[^>]*value="([^"]*)"', page):
        params[html.unescape(match.group(1))] = html.unescape(match.group(2))
    if "id" not in params or "confirm" not in params:
        raise RuntimeError("Google Drive warning page did not contain required hidden inputs")
    return html.unescape(action_match.group(1)), params


def strip_tags(text):
    return html.unescape(re.sub(r"<[^>]+>", " ", text)).strip()


def summarize_html_error(page):
    title = re.search(r"<title>(.*?)</title>", page, flags=re.DOTALL)
    caption = re.search(r'<p class="uc-error-caption">(.*?)</p>', page, flags=re.DOTALL)
    subcaption = re.search(r'<p class="uc-error-subcaption">(.*?)</p>', page, flags=re.DOTALL)
    parts = []
    for match in (title, caption, subcaption):
        if match:
            parts.append(strip_tags(match.group(1)))
    return " ".join(part for part in parts if part)


def ensure_download_response(response):
    content_type = get_header(response.headers, "content-type") or ""
    disposition = get_header(response.headers, "content-disposition")
    if disposition and "text/html" not in content_type:
        return response
    if "text/html" not in content_type:
        return response

    page = response.read().decode("utf-8", errors="replace")
    response.close()
    message = summarize_html_error(page) or "Google Drive returned HTML instead of file content"
    raise RuntimeError(f"Google Drive download blocked: {message}")


def open_download_response(opener, file_id, timeout):
    url = f"https://drive.google.com/uc?{urlencode({'export': 'download', 'id': file_id})}"
    request = Request(url, headers={"User-Agent": USER_AGENT})
    response = opener.open(request, timeout=timeout)
    content_type = get_header(response.headers, "content-type") or ""
    disposition = get_header(response.headers, "content-disposition")
    if disposition or "text/html" not in content_type:
        return response

    page = response.read().decode("utf-8", errors="replace")
    response.close()
    action, params = parse_download_form(page)
    request = Request(f"{action}?{urlencode(params)}", headers={"User-Agent": USER_AGENT})
    response = opener.open(request, timeout=timeout)
    return ensure_download_response(response)


def content_length(headers):
    value = get_header(headers, "content-length")
    if not value:
        return None
    try:
        return int(value)
    except ValueError:
        return None


def ensure_enough_space(output_dir, expected_bytes, min_free_gb):
    if expected_bytes is None:
        return
    stat = output_dir.stat()
    # statvfs is available on Posix paths through Path via os.
    import os  # Local import keeps the top-level dependency list minimal.

    vfs = os.statvfs(output_dir)
    free_bytes = vfs.f_bavail * vfs.f_frsize
    required_bytes = expected_bytes + int(min_free_gb * 1024 * 1024 * 1024)
    if free_bytes < required_bytes:
        raise RuntimeError(
            f"not enough free space in {output_dir}: "
            f"need at least {required_bytes} bytes, have {free_bytes} bytes"
        )


def stream_response(response, output_path, expected_bytes, chunk_size, progress_interval_mb):
    part_path = output_path.with_suffix(output_path.suffix + ".part")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    downloaded = 0
    last_report = time.monotonic()
    next_progress = progress_interval_mb * 1024 * 1024
    started = time.monotonic()

    with part_path.open("wb") as stream:
        while True:
            chunk = response.read(chunk_size)
            if not chunk:
                break
            stream.write(chunk)
            downloaded += len(chunk)
            now = time.monotonic()
            if downloaded >= next_progress or now - last_report >= 30:
                total = expected_bytes or content_length(response.headers)
                if total:
                    percent = downloaded / total * 100.0
                    print(
                        f"downloaded {downloaded / 1024 / 1024:.1f} MiB / "
                        f"{total / 1024 / 1024:.1f} MiB ({percent:.1f}%)",
                        file=sys.stderr,
                        flush=True,
                    )
                else:
                    print(f"downloaded {downloaded / 1024 / 1024:.1f} MiB", file=sys.stderr, flush=True)
                last_report = now
                next_progress = downloaded + progress_interval_mb * 1024 * 1024

    if expected_bytes is not None and downloaded != expected_bytes:
        raise RuntimeError(f"downloaded {downloaded} bytes, expected {expected_bytes} bytes")
    part_path.replace(output_path)
    elapsed = max(time.monotonic() - started, 0.001)
    return {
        "path": str(output_path),
        "bytes": downloaded,
        "elapsed_sec": elapsed,
        "mbps": downloaded / 1024 / 1024 / elapsed,
    }


def write_manifest(path, payload):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def main(argv=None):
    parser = argparse.ArgumentParser(description="Fetch one FAST-LIVO2 Google Drive sequence bag.")
    parser.add_argument("--sequence", default=DEFAULT_SEQUENCE, help="Sequence name without .bag suffix.")
    parser.add_argument("--output-dir", default="/home/frank/data/fast_livo")
    parser.add_argument("--folder-url", default=FOLDER_URL)
    parser.add_argument("--timeout", type=float, default=60.0)
    parser.add_argument("--chunk-size", type=int, default=4 * 1024 * 1024)
    parser.add_argument("--progress-interval-mb", type=int, default=256)
    parser.add_argument("--min-free-gb", type=float, default=10.0)
    parser.add_argument("--dry-run", action="store_true", help="Discover the file but do not download it.")
    parser.add_argument("--list", action="store_true", help="List discovered .bag entries and exit.")
    parser.add_argument("--skip-existing", action="store_true", default=True)
    parser.add_argument("--json", action="store_true", help="Print JSON result.")
    parser.add_argument("--manifest", help="Optional JSON manifest path.")
    args = parser.parse_args(argv)

    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    opener = build_opener(HTTPCookieProcessor())

    try:
        folder_html = fetch_text(opener, args.folder_url, args.timeout)
        entries = discover_folder_entries(folder_html)
    except (HTTPError, OSError, RuntimeError, URLError) as exc:
        print(f"FAST-LIVO2 discovery failed: {exc}", file=sys.stderr)
        return 2

    if args.list:
        print(json.dumps({"folder_url": args.folder_url, "entries": entries}, indent=2, sort_keys=True))
        return 0

    target_name = args.sequence if args.sequence.endswith(".bag") else f"{args.sequence}.bag"
    entry = next((item for item in entries if item["name"] == target_name), None)
    if entry is None:
        available = ", ".join(item["name"] for item in entries[:20])
        print(f"FAST-LIVO2 sequence not found: {target_name}. Available: {available}", file=sys.stderr)
        return 1

    output_path = output_dir / target_name
    result = {
        "schema": "gaussian_lic_fastlivo2_fetch/v1",
        "folder_url": args.folder_url,
        "sequence": args.sequence.removesuffix(".bag"),
        "entry": entry,
        "output_path": str(output_path),
        "downloaded": False,
        "skipped": False,
        "ok": False,
    }

    if args.dry_run:
        result["ok"] = True
        result["dry_run"] = True
    elif args.skip_existing and output_path.is_file() and entry["bytes"] and output_path.stat().st_size == entry["bytes"]:
        result["ok"] = True
        result["skipped"] = True
    else:
        try:
            ensure_enough_space(output_dir, entry["bytes"], args.min_free_gb)
            response = open_download_response(opener, entry["file_id"], args.timeout)
            with response:
                result["download"] = stream_response(
                    response,
                    output_path,
                    entry["bytes"],
                    args.chunk_size,
                    args.progress_interval_mb,
                )
            result["downloaded"] = True
            result["ok"] = True
        except (HTTPError, OSError, RuntimeError, URLError) as exc:
            result["error"] = str(exc)
            print(f"FAST-LIVO2 download failed: {exc}", file=sys.stderr)
            if args.manifest:
                write_manifest(Path(args.manifest), result)
            if args.json:
                print(json.dumps(result, indent=2, sort_keys=True))
            return 2

    if args.manifest:
        write_manifest(Path(args.manifest), result)
    if args.json or args.dry_run:
        print(json.dumps(result, indent=2, sort_keys=True))
    else:
        print(f"FAST-LIVO2 sequence ready: {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
