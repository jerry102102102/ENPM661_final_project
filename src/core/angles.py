"""Angle helpers."""

from __future__ import annotations

import math


def normalize_angle(theta: float) -> float:
    """Normalize an angle to [-pi, pi)."""

    return (theta + math.pi) % (2.0 * math.pi) - math.pi
