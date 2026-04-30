"""Planner heuristics."""

from __future__ import annotations

from src.core.costs import euclidean_distance_xy
from src.models.state import Pose2D


def euclidean_goal_heuristic(pose: Pose2D, goal_xy: tuple[float, float]) -> float:
    """Admissible geometric distance heuristic used by the static baseline."""

    return euclidean_distance_xy(pose, goal_xy)
