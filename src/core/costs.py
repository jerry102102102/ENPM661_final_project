"""Cost utilities."""

from __future__ import annotations

import math

from src.models.state import Pose2D, TrajectorySegment


def euclidean_distance_xy(a: Pose2D | tuple[float, float], b: Pose2D | tuple[float, float]) -> float:
    """Return Euclidean XY distance between poses or XY tuples."""

    ax, ay = (a.x, a.y) if isinstance(a, Pose2D) else a
    bx, by = (b.x, b.y) if isinstance(b, Pose2D) else b
    return math.hypot(ax - bx, ay - by)


def arc_length_cost(samples: list[Pose2D]) -> float:
    """Approximate trajectory arc length from sampled poses."""

    return sum(
        math.hypot(samples[index].x - samples[index - 1].x, samples[index].y - samples[index - 1].y)
        for index in range(1, len(samples))
    )


def segment_arc_length(segment: TrajectorySegment) -> float:
    """Return the stored segment cost when available, else recompute it."""

    return segment.cost if segment.cost > 0.0 else arc_length_cost(segment.samples)
