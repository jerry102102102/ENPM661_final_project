"""Dynamic moving-circle collision utilities."""

from __future__ import annotations

import math

from src.configs.defaults import CollisionParams
from src.models.obstacles import DynamicCircleObstacle
from src.models.state import Pose2D, TimedPose2D


def obstacle_position_at_time(obstacle: DynamicCircleObstacle, time_s: float) -> tuple[float, float]:
    """Return the obstacle center at a query time under constant velocity."""

    return (
        obstacle.initial_x + (obstacle.velocity_x * time_s),
        obstacle.initial_y + (obstacle.velocity_y * time_s),
    )


def dynamic_pose_collision(
    pose: Pose2D,
    time_s: float,
    obstacle: DynamicCircleObstacle,
    collision: CollisionParams,
    clearance: float = 0.0,
) -> bool:
    """Return True if the robot footprint collides with one moving circle."""

    obstacle_x, obstacle_y = obstacle_position_at_time(obstacle, time_s)
    effective_radius = collision.radius_m + obstacle.radius + clearance
    return math.hypot(pose.x - obstacle_x, pose.y - obstacle_y) <= effective_radius


def dynamic_pose_collision_any(
    pose: Pose2D,
    time_s: float,
    dynamic_obstacles: list[DynamicCircleObstacle],
    collision: CollisionParams,
    clearance: float = 0.0,
) -> bool:
    """Return True if a pose collides with any dynamic obstacle at time_s."""

    return any(dynamic_pose_collision(pose, time_s, obstacle, collision, clearance) for obstacle in dynamic_obstacles)


def dynamic_trajectory_collision(
    timed_samples: list[TimedPose2D],
    dynamic_obstacles: list[DynamicCircleObstacle],
    collision: CollisionParams,
    clearance: float = 0.0,
) -> bool:
    """Return True if any timed sample collides with any dynamic obstacle."""

    return any(
        dynamic_pose_collision_any(sample.pose, sample.time_s, dynamic_obstacles, collision, clearance)
        for sample in timed_samples
    )
