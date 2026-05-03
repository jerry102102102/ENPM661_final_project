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
    """Known constant-velocity circular obstacle.

    Optional active-time bounds let piecewise-linear or periodic obstacle
    trajectories be represented as time-gated constant-velocity segments.
    Existing always-active obstacles keep using the default ``None`` bounds.
    """

    initial_x: float
    initial_y: float
    velocity_x: float
    velocity_y: float
    radius: float
    label: str = "dynamic_obstacle"
    active_start_time_s: float | None = None
    active_end_time_s: float | None = None

    def is_active_at(self, time_s: float) -> bool:
        """Return whether this segment is active at an absolute time."""

        if self.active_start_time_s is not None and time_s < self.active_start_time_s:
            return False
        if self.active_end_time_s is not None and time_s > self.active_end_time_s:
            return False
        return True
