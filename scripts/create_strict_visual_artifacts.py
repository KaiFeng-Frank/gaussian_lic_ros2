#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Create compact visual artifacts from a strict reproduction run."""

from __future__ import annotations

import argparse
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--current-dir",
        type=Path,
        default=Path("results/fastlivo2/CBD_Building_01_current_round_no_opacity_prune_probe"),
        help="Strict ROS2 current result directory.",
    )
    parser.add_argument(
        "--baseline-dir",
        type=Path,
        default=Path("baseline/fastlivo2/CBD_Building_01"),
        help="Strict ROS1 baseline directory.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("docs/assets"),
        help="Output directory for README/roadmap visual artifacts.",
    )
    parser.add_argument("--montage", default="strict_cbd_montage.jpg")
    parser.add_argument("--gif", default="strict_cbd_render_demo.gif")
    parser.add_argument("--columns", type=int, default=4)
    parser.add_argument("--frames", type=int, default=16)
    return parser.parse_args()


def image_names(directory: Path) -> list[str]:
    if not directory.is_dir():
        return []
    return sorted(path.name for path in directory.glob("*.jpg"))


def choose_evenly(names: list[str], count: int) -> list[str]:
    if len(names) <= count:
        return names
    if count <= 1:
        return [names[len(names) // 2]]
    return [names[round(index * (len(names) - 1) / (count - 1))] for index in range(count)]


def load_cell(path: Path, size: tuple[int, int]) -> Image.Image:
    image = Image.open(path).convert("RGB")
    image.thumbnail(size, Image.Resampling.LANCZOS)
    cell = Image.new("RGB", size, (18, 20, 24))
    cell.paste(image, ((size[0] - image.width) // 2, (size[1] - image.height) // 2))
    return cell


def label(draw: ImageDraw.ImageDraw, xy: tuple[int, int], text: str) -> None:
    draw.rectangle((xy[0] - 4, xy[1] - 3, xy[0] + 8 * len(text) + 4, xy[1] + 15), fill=(0, 0, 0))
    draw.text(xy, text, fill=(245, 245, 245), font=ImageFont.load_default())


def make_montage(
    names: list[str],
    current_render_dir: Path,
    current_gt_dir: Path,
    baseline_render_dir: Path,
    output: Path,
    columns: int,
) -> None:
    cell_size = (320, 192)
    label_height = 22
    rows_per_sample = 3
    width = columns * cell_size[0]
    sample_rows = (len(names) + columns - 1) // columns
    height = sample_rows * rows_per_sample * (cell_size[1] + label_height)
    canvas = Image.new("RGB", (width, height), (12, 14, 18))
    draw = ImageDraw.Draw(canvas)
    source_rows = [
        ("ROS2 current", current_render_dir),
        ("Ground truth", current_gt_dir),
        ("ROS1 baseline", baseline_render_dir),
    ]
    for sample_index, name in enumerate(names):
        column = sample_index % columns
        sample_row = sample_index // columns
        for row_offset, (source_label, directory) in enumerate(source_rows):
            x0 = column * cell_size[0]
            y0 = (sample_row * rows_per_sample + row_offset) * (cell_size[1] + label_height)
            draw.text((x0 + 8, y0 + 4), f"{source_label} {name}", fill=(230, 230, 230), font=ImageFont.load_default())
            cell = load_cell(directory / name, cell_size)
            canvas.paste(cell, (x0, y0 + label_height))
    output.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(output, quality=88, optimize=True)


def make_gif(
    names: list[str],
    current_render_dir: Path,
    current_gt_dir: Path,
    output: Path,
) -> None:
    frame_size = (320, 192)
    frames: list[Image.Image] = []
    for name in names:
        current = load_cell(current_render_dir / name, frame_size)
        gt = load_cell(current_gt_dir / name, frame_size)
        frame = Image.new("RGB", (frame_size[0] * 2, frame_size[1] + 24), (12, 14, 18))
        frame.paste(current, (0, 24))
        frame.paste(gt, (frame_size[0], 24))
        draw = ImageDraw.Draw(frame)
        label(draw, (8, 5), f"ROS2 current {name}")
        label(draw, (frame_size[0] + 8, 5), "Ground truth")
        frames.append(frame)
    output.parent.mkdir(parents=True, exist_ok=True)
    frames[0].save(
        output,
        save_all=True,
        append_images=frames[1:],
        duration=220,
        loop=0,
        optimize=True,
    )


def main() -> int:
    args = parse_args()
    current_render_dir = args.current_dir / "renders"
    current_gt_dir = args.current_dir / "gt"
    baseline_render_dir = args.baseline_dir / "renders"
    common_names = sorted(
        set(image_names(current_render_dir))
        & set(image_names(current_gt_dir))
        & set(image_names(baseline_render_dir))
    )
    if not common_names:
        raise SystemExit("No common render/GT/baseline JPG frames found.")

    montage_names = choose_evenly(common_names, max(args.columns * 2, args.columns))
    gif_names = choose_evenly(common_names, args.frames)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    make_montage(
        montage_names,
        current_render_dir,
        current_gt_dir,
        baseline_render_dir,
        args.output_dir / args.montage,
        args.columns,
    )
    make_gif(gif_names, current_render_dir, current_gt_dir, args.output_dir / args.gif)
    print(f"montage={args.output_dir / args.montage}")
    print(f"gif={args.output_dir / args.gif}")
    print(f"frames={len(common_names)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
