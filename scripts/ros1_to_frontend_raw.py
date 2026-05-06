#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Compatibility entrypoint for the profile-aware ROS1 to frontend_raw converter."""

from pathlib import Path
import runpy


if __name__ == "__main__":
    runpy.run_path(str(Path(__file__).with_name("fastlivo2_ros1_to_frontend_raw.py")), run_name="__main__")
