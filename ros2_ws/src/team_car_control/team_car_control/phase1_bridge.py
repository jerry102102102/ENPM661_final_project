"""Helpers for importing the Part 1 planner from this repository."""

from __future__ import annotations

from pathlib import Path
import sys


def ensure_phase2_part1_on_path() -> Path:
    """Find the Part 1 planner directory and add it to ``sys.path``."""

    candidates = [Path.cwd(), *Path(__file__).resolve().parents]

    for base in candidates:
        for dirname in ("Part01", "phase2_part1"):
            phase2_part1_dir = base / dirname
            if (phase2_part1_dir / "astar_planner.py").exists():
                phase2_part1_path = phase2_part1_dir.resolve()
                if phase2_part1_path.as_posix() not in sys.path:
                    sys.path.insert(0, phase2_part1_path.as_posix())
                return phase2_part1_path

    raise FileNotFoundError("Could not locate Part01 in this workspace.")
