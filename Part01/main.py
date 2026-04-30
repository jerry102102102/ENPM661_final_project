#!/usr/bin/env python3
"""Backward-compatible wrapper for the static nonholonomic A* baseline."""

from __future__ import annotations

from pathlib import Path
import runpy


if __name__ == "__main__":
    script = Path(__file__).resolve().parents[1] / "scripts" / "run_baseline_astar.py"
    runpy.run_path(script.as_posix(), run_name="__main__")
