"""Temporal trajectory validation against static and dynamic obstacles."""

from __future__ import annotations

from src.configs.defaults import CollisionParams
from src.core.dynamic_collision import dynamic_pose_collision_any
from src.core.static_collision import is_pose_static_valid
from src.core.time_parameterization import annotate_trajectory_samples
from src.models.obstacles import DynamicCircleObstacle, StaticWorld
from src.models.state import TrajectorySegment


def temporal_collision_free(
    trajectory: TrajectorySegment,
    start_time_s: float,
    static_world: StaticWorld,
    dynamic_obstacles: list[DynamicCircleObstacle],
    collision: CollisionParams,
    clearance: float = 0.0,
    timestamp_mode: str = "normalized",
    timestamp_dt_s: float | None = None,
) -> bool:
    """Validate a trajectory over space and time."""

    timed_samples = annotate_trajectory_samples(
        trajectory,
        start_time_s,
        mode=timestamp_mode,
        dt_s=timestamp_dt_s,
    )
    for timed_pose in timed_samples:
        if not is_pose_static_valid(timed_pose.pose, collision, clearance, static_world):
            return False
        if dynamic_pose_collision_any(timed_pose.pose, timed_pose.time_s, dynamic_obstacles, collision, clearance):
            return False
    return True
