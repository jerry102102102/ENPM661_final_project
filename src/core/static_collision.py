"""Static world and footprint collision checking."""

from __future__ import annotations

import math

from src.configs.defaults import CollisionParams
from src.models.obstacles import StaticWorld
from src.models.state import OrientedBox, Pose2D, TrajectorySegment


def point_inside_world_bounds(
    x: float,
    y: float,
    effective_radius: float,
    bounds: tuple[float, float, float, float],
) -> bool:
    """Check that a circular robot centered at (x, y) stays inside bounds."""

    min_x, max_x, min_y, max_y = bounds
    return (
        min_x + effective_radius <= x <= max_x - effective_radius
        and min_y + effective_radius <= y <= max_y - effective_radius
    )


def circle_collides_with_oriented_box(x: float, y: float, radius: float, box: OrientedBox) -> bool:
    """Check circle-vs-oriented-rectangle collision."""

    dx = x - box.center_x
    dy = y - box.center_y
    cos_a = math.cos(-box.angle_rad)
    sin_a = math.sin(-box.angle_rad)
    local_x = (dx * cos_a) - (dy * sin_a)
    local_y = (dx * sin_a) + (dy * cos_a)

    half_w = box.width / 2.0
    half_h = box.height / 2.0
    closest_x = min(max(local_x, -half_w), half_w)
    closest_y = min(max(local_y, -half_h), half_h)
    return math.hypot(local_x - closest_x, local_y - closest_y) <= radius


def circle_collides_with_obstacles(
    x: float,
    y: float,
    effective_radius: float,
    obstacles: list[OrientedBox],
) -> bool:
    """Check a circular footprint against all static obstacles."""

    return any(circle_collides_with_oriented_box(x, y, effective_radius, obstacle) for obstacle in obstacles)


def _box_corners(box: OrientedBox) -> list[tuple[float, float]]:
    half_w = box.width / 2.0
    half_h = box.height / 2.0
    cos_a = math.cos(box.angle_rad)
    sin_a = math.sin(box.angle_rad)
    corners = []
    for local_x, local_y in ((-half_w, -half_h), (half_w, -half_h), (half_w, half_h), (-half_w, half_h)):
        corners.append(
            (
                box.center_x + (local_x * cos_a) - (local_y * sin_a),
                box.center_y + (local_x * sin_a) + (local_y * cos_a),
            )
        )
    return corners


def _project(corners: list[tuple[float, float]], axis: tuple[float, float]) -> tuple[float, float]:
    values = [(corner[0] * axis[0]) + (corner[1] * axis[1]) for corner in corners]
    return min(values), max(values)


def oriented_boxes_intersect(a: OrientedBox, b: OrientedBox) -> bool:
    """Check oriented-box overlap with the separating axis theorem."""

    corners_a = _box_corners(a)
    corners_b = _box_corners(b)
    axes = [
        (math.cos(a.angle_rad), math.sin(a.angle_rad)),
        (-math.sin(a.angle_rad), math.cos(a.angle_rad)),
        (math.cos(b.angle_rad), math.sin(b.angle_rad)),
        (-math.sin(b.angle_rad), math.cos(b.angle_rad)),
    ]
    for axis in axes:
        min_a, max_a = _project(corners_a, axis)
        min_b, max_b = _project(corners_b, axis)
        if max_a < min_b or max_b < min_a:
            return False
    return True


def robot_box_from_pose(pose: Pose2D, collision: CollisionParams, clearance: float) -> OrientedBox:
    """Build an oriented rectangular robot footprint in world coordinates."""

    if collision.length_m is None or collision.width_m is None:
        raise ValueError("Box footprint requires length_m and width_m")
    offset_x, offset_y = collision.center_offset_m
    cos_t = math.cos(pose.theta)
    sin_t = math.sin(pose.theta)
    return OrientedBox(
        name="robot_footprint",
        center_x=pose.x + (offset_x * cos_t) - (offset_y * sin_t),
        center_y=pose.y + (offset_x * sin_t) + (offset_y * cos_t),
        width=collision.length_m + (2.0 * clearance),
        height=collision.width_m + (2.0 * clearance),
        angle_rad=pose.theta,
    )


def box_inside_world_bounds(box: OrientedBox, bounds: tuple[float, float, float, float]) -> bool:
    """Check that all footprint corners remain inside bounds."""

    min_x, max_x, min_y, max_y = bounds
    return all(min_x <= x <= max_x and min_y <= y <= max_y for x, y in _box_corners(box))


def is_pose_static_valid(
    pose: Pose2D,
    collision: CollisionParams,
    clearance: float,
    static_world: StaticWorld,
) -> bool:
    """Return True when a pose is inside bounds and static-collision-free."""

    if collision.footprint_type == "box":
        box = robot_box_from_pose(pose, collision, clearance)
        if not box_inside_world_bounds(box, static_world.bounds):
            return False
        return not any(oriented_boxes_intersect(box, obstacle) for obstacle in static_world.obstacles)

    effective_radius = collision.radius_m + clearance
    if not point_inside_world_bounds(pose.x, pose.y, effective_radius, static_world.bounds):
        return False
    return not circle_collides_with_obstacles(pose.x, pose.y, effective_radius, static_world.obstacles)


def is_trajectory_static_valid(
    trajectory: TrajectorySegment | list[Pose2D],
    collision: CollisionParams,
    clearance: float,
    static_world: StaticWorld,
) -> bool:
    """Check every sampled pose along a trajectory against static geometry."""

    samples = trajectory.samples if isinstance(trajectory, TrajectorySegment) else trajectory
    return all(is_pose_static_valid(sample, collision, clearance, static_world) for sample in samples)
