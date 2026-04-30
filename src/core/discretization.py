"""Continuous-to-discrete state helpers."""

from __future__ import annotations

import math

from src.core.angles import normalize_angle
from src.models.state import Pose2D


def theta_to_bin(theta: float, theta_bins: int) -> int:
    """Convert a heading in radians to a discrete heading bin."""

    wrapped = normalize_angle(theta) + math.pi
    return int(math.floor(wrapped / (2.0 * math.pi / theta_bins))) % theta_bins


def discretize_pose(pose: Pose2D, xy_resolution_m: float, theta_bins: int) -> tuple[int, int, int]:
    """Convert a continuous pose to the old Part 1 A* grid key."""

    return (
        int(round(pose.x / xy_resolution_m)),
        int(round(pose.y / xy_resolution_m)),
        theta_to_bin(pose.theta, theta_bins),
    )
