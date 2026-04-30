"""Static and dynamic obstacle data structures."""

from __future__ import annotations

from dataclasses import dataclass

from src.models.state import OrientedBox


@dataclass(frozen=True)
class StaticWorld:
    """Static map geometry and bounds."""

    bounds: tuple[float, float, float, float]
    obstacles: list[OrientedBox]


@dataclass(frozen=True)
class DynamicCircleObstacle:
    """Known constant-velocity circular obstacle."""

    initial_x: float
    initial_y: float
    velocity_x: float
    velocity_y: float
    radius: float
    label: str = "dynamic_obstacle"
