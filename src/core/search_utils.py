"""Search bookkeeping helpers used by planners."""

from __future__ import annotations

from src.core.angles import normalize_angle
from src.core.costs import euclidean_distance_xy
from src.models.state import Pose2D


def goal_satisfied(
    pose: Pose2D,
    goal_xy: tuple[float, float],
    goal_tolerance_m: float,
    goal_heading_rad: float,
    goal_heading_tolerance_rad: float,
) -> bool:
    """Return True when position and heading tolerances are both satisfied."""

    if euclidean_distance_xy(pose, goal_xy) > goal_tolerance_m:
        return False
    heading_error = abs(normalize_angle(pose.theta - goal_heading_rad))
    return heading_error <= goal_heading_tolerance_rad
