"""Static-geometric roadmap construction helpers."""

from __future__ import annotations

import math
import random
from collections.abc import Iterable

from src.configs.defaults import CollisionParams, PlannerConfig, VehicleParams
from src.core.angles import normalize_angle
from src.core.costs import euclidean_distance_xy
from src.core.discretization import discretize_pose
from src.core.rollout import simulate_primitive
from src.core.static_collision import is_pose_static_valid, is_trajectory_static_valid
from src.models.obstacles import StaticWorld
from src.models.primitives import MotionPrimitive, get_action_set
from src.models.roadmap import Roadmap, RoadmapNode
from src.models.state import Pose2D


DEFAULT_DISCRETE_HEADINGS_RAD = (
    0.0,
    math.radians(45.0),
    math.radians(-45.0),
    math.radians(90.0),
    math.radians(-90.0),
    math.pi,
)


def _dedupe_poses(poses: Iterable[Pose2D], config: PlannerConfig) -> list[Pose2D]:
    """Remove duplicate poses under the planner discretization."""

    unique: dict[tuple[int, int, int], Pose2D] = {}
    for pose in poses:
        unique.setdefault(discretize_pose(pose, config.xy_resolution_m, config.theta_bins), pose)
    return list(unique.values())


def _sample_valid_xy_points_random(
    count: int,
    static_world: StaticWorld,
    collision: CollisionParams,
    clearance: float,
    *,
    seed: int | None = None,
    max_attempts: int | None = None,
) -> list[tuple[float, float]]:
    rng = random.Random(seed)
    min_x, max_x, min_y, max_y = static_world.bounds
    attempts_left = max_attempts or max(1000, count * 100)
    points: list[tuple[float, float]] = []
    while len(points) < count and attempts_left > 0:
        attempts_left -= 1
        x = rng.uniform(min_x, max_x)
        y = rng.uniform(min_y, max_y)
        if is_pose_static_valid(Pose2D(x, y, 0.0), collision, clearance, static_world):
            points.append((x, y))
    if len(points) < count:
        raise RuntimeError(f"Only sampled {len(points)} valid XY points out of requested {count}.")
    return points


def _sample_valid_xy_points_grid(
    static_world: StaticWorld,
    collision: CollisionParams,
    clearance: float,
    *,
    spacing_m: float,
    max_points: int | None = None,
) -> list[tuple[float, float]]:
    min_x, max_x, min_y, max_y = static_world.bounds
    points: list[tuple[float, float]] = []
    y = min_y + spacing_m
    while y <= max_y - spacing_m + 1e-9:
        x = min_x + spacing_m
        while x <= max_x - spacing_m + 1e-9:
            if is_pose_static_valid(Pose2D(x, y, 0.0), collision, clearance, static_world):
                points.append((x, y))
                if max_points is not None and len(points) >= max_points:
                    return points
            x += spacing_m
        y += spacing_m
    return points


def sample_xy_heading_roadmap_nodes(
    xy_count: int,
    static_world: StaticWorld,
    collision: CollisionParams,
    clearance: float,
    config: PlannerConfig,
    *,
    headings_rad: Iterable[float] = DEFAULT_DISCRETE_HEADINGS_RAD,
    sampling_mode: str = "random",
    grid_spacing_m: float | None = None,
    seed: int | None = None,
) -> list[Pose2D]:
    """Sample XY points in free space and attach a discrete heading set.

    This is the reusable PRM-style node sampler requested by the project spec:
    the returned nodes are independent of a single start/goal query.
    """

    if sampling_mode == "grid":
        spacing = grid_spacing_m or max(config.xy_resolution_m * 4.0, 0.20)
        xy_points = _sample_valid_xy_points_grid(
            static_world,
            collision,
            clearance,
            spacing_m=spacing,
            max_points=xy_count,
        )
    elif sampling_mode == "random":
        xy_points = _sample_valid_xy_points_random(
            xy_count,
            static_world,
            collision,
            clearance,
            seed=seed,
        )
    else:
        raise ValueError(f"Unknown sampling_mode: {sampling_mode}")

    heading_values = [normalize_angle(theta) for theta in headings_rad]
    candidate_poses = [
        Pose2D(x, y, theta)
        for x, y in xy_points
        for theta in heading_values
        if is_pose_static_valid(Pose2D(x, y, theta), collision, clearance, static_world)
    ]
    return _dedupe_poses(candidate_poses, config)


def sample_roadmap_nodes(
    count: int,
    static_world: StaticWorld,
    collision: CollisionParams,
    clearance: float,
    *,
    seed: int | None = None,
    max_attempts: int | None = None,
) -> list[Pose2D]:
    """Sample static-valid nodes in (x, y, theta)."""

    rng = random.Random(seed)
    min_x, max_x, min_y, max_y = static_world.bounds
    attempts_left = max_attempts or max(1000, count * 100)
    nodes: list[Pose2D] = []
    while len(nodes) < count and attempts_left > 0:
        attempts_left -= 1
        pose = Pose2D(
            x=rng.uniform(min_x, max_x),
            y=rng.uniform(min_y, max_y),
            theta=rng.uniform(-math.pi, math.pi),
        )
        if is_pose_static_valid(pose, collision, clearance, static_world):
            nodes.append(pose)
    if len(nodes) < count:
        raise RuntimeError(f"Only sampled {len(nodes)} valid nodes out of requested {count}.")
    return nodes


def _heading_error(a: float, b: float) -> float:
    return abs(normalize_angle(a - b))


def _nearest_reachable_node(
    pose: Pose2D,
    nodes: list[RoadmapNode],
    source_id: int,
    position_tolerance_m: float,
    heading_tolerance_rad: float,
) -> RoadmapNode | None:
    candidates = [
        node
        for node in nodes
        if node.node_id != source_id
        and euclidean_distance_xy(pose, node.pose) <= position_tolerance_m
        and _heading_error(pose.theta, node.pose.theta) <= heading_tolerance_rad
    ]
    if not candidates:
        return None
    return min(candidates, key=lambda node: euclidean_distance_xy(pose, node.pose))


def connect_nodes_with_primitives(
    roadmap: Roadmap,
    primitives: list[MotionPrimitive],
    vehicle_params: VehicleParams,
    collision: CollisionParams,
    clearance: float,
    static_world: StaticWorld,
    config: PlannerConfig,
    *,
    position_tolerance_m: float | None = None,
    heading_tolerance_rad: float | None = None,
) -> Roadmap:
    """Connect sampled nodes by rolling out primitives and keeping static-valid edges."""

    nodes = list(roadmap.nodes.values())
    position_tolerance = position_tolerance_m or config.xy_resolution_m
    heading_tolerance = heading_tolerance_rad or (math.pi / config.theta_bins)
    for source in nodes:
        for primitive in primitives:
            trajectory = simulate_primitive(
                source.pose,
                primitive,
                vehicle_params,
                config.action_duration_s,
                config.integration_dt_s,
            )
            target = _nearest_reachable_node(
                trajectory.end,
                nodes,
                source.node_id,
                position_tolerance,
                heading_tolerance,
            )
            if target is None:
                continue
            static_valid = is_trajectory_static_valid(trajectory, collision, clearance, static_world)
            if not static_valid:
                continue
            roadmap.add_edge(
                source_id=source.node_id,
                target_id=target.node_id,
                primitive_label=primitive.label,
                trajectory=trajectory,
                duration_s=trajectory.duration_s,
                static_valid=True,
            )
    return roadmap


def connect_sampled_nodes_with_primitives(
    roadmap: Roadmap,
    primitives: list[MotionPrimitive],
    vehicle_params: VehicleParams,
    collision: CollisionParams,
    clearance: float,
    static_world: StaticWorld,
    config: PlannerConfig,
    *,
    position_tolerance_m: float | None = None,
    heading_tolerance_rad: float | None = None,
    max_outgoing_edges_per_node: int | None = None,
) -> Roadmap:
    """Connect a sampled nonholonomic roadmap with primitive endpoints."""

    nodes = list(roadmap.nodes.values())
    position_tolerance = position_tolerance_m or max(config.xy_resolution_m * 2.0, 0.10)
    heading_tolerance = heading_tolerance_rad or max(math.pi / config.theta_bins, math.radians(12.0))
    existing_pairs: set[tuple[int, int, str | None]] = set()

    for source in nodes:
        outgoing_added = 0
        for primitive in primitives:
            trajectory = simulate_primitive(
                source.pose,
                primitive,
                vehicle_params,
                config.action_duration_s,
                config.integration_dt_s,
            )
            if trajectory.cost <= 1e-12:
                continue
            if not is_trajectory_static_valid(trajectory, collision, clearance, static_world):
                continue

            target = _nearest_reachable_node(
                trajectory.end,
                nodes,
                source.node_id,
                position_tolerance,
                heading_tolerance,
            )
            if target is None:
                continue
            key = (source.node_id, target.node_id, primitive.label)
            if key in existing_pairs:
                continue
            roadmap.add_edge(
                source_id=source.node_id,
                target_id=target.node_id,
                primitive_label=primitive.label,
                trajectory=trajectory,
                duration_s=trajectory.duration_s,
                static_valid=True,
            )
            existing_pairs.add(key)
            outgoing_added += 1
            if max_outgoing_edges_per_node is not None and outgoing_added >= max_outgoing_edges_per_node:
                break

    return roadmap


def _add_pose_if_new(
    roadmap: Roadmap,
    pose: Pose2D,
    pose_to_node_id: dict[tuple[int, int, int], int],
    config: PlannerConfig,
) -> RoadmapNode:
    key = discretize_pose(pose, config.xy_resolution_m, config.theta_bins)
    existing_id = pose_to_node_id.get(key)
    if existing_id is not None:
        return roadmap.nodes[existing_id]
    node = roadmap.add_node(pose)
    pose_to_node_id[key] = node.node_id
    return node


def build_primitive_expansion_roadmap(
    seed_poses: list[Pose2D],
    primitives: list[MotionPrimitive],
    vehicle_params: VehicleParams,
    collision: CollisionParams,
    clearance: float,
    static_world: StaticWorld,
    config: PlannerConfig,
    *,
    max_nodes: int = 500,
    max_expansions: int = 500,
) -> Roadmap:
    """Build a usable static roadmap by expanding nonholonomic primitives.

    Unlike the random sample connector, this builder creates new nodes at
    primitive endpoints. It is intentionally simple, deterministic, and useful
    for the first end-to-end temporal planner.
    """

    roadmap = Roadmap()
    pose_to_node_id: dict[tuple[int, int, int], int] = {}
    frontier: list[int] = []

    for pose in seed_poses:
        if not is_pose_static_valid(pose, collision, clearance, static_world):
            continue
        node = _add_pose_if_new(roadmap, pose, pose_to_node_id, config)
        frontier.append(node.node_id)

    expansion_count = 0
    cursor = 0
    while cursor < len(frontier) and len(roadmap.nodes) < max_nodes and expansion_count < max_expansions:
        source_id = frontier[cursor]
        cursor += 1
        expansion_count += 1
        source = roadmap.nodes[source_id]

        for primitive in primitives:
            trajectory = simulate_primitive(
                source.pose,
                primitive,
                vehicle_params,
                config.action_duration_s,
                config.integration_dt_s,
            )
            if trajectory.cost <= 1e-12:
                continue
            if not is_trajectory_static_valid(trajectory, collision, clearance, static_world):
                continue

            target_key = discretize_pose(trajectory.end, config.xy_resolution_m, config.theta_bins)
            if target_key not in pose_to_node_id and len(roadmap.nodes) >= max_nodes:
                continue
            node_count_before = len(roadmap.nodes)
            target = _add_pose_if_new(roadmap, trajectory.end, pose_to_node_id, config)
            target_is_new = len(roadmap.nodes) > node_count_before
            if target.node_id == source_id:
                continue
            if not any(
                roadmap.edges[edge_id].target_id == target.node_id
                and roadmap.edges[edge_id].primitive_label == primitive.label
                for edge_id in roadmap.outgoing_edges.get(source_id, [])
            ):
                roadmap.add_edge(
                    source_id=source_id,
                    target_id=target.node_id,
                    primitive_label=primitive.label,
                    trajectory=trajectory,
                    duration_s=trajectory.duration_s,
                    static_valid=True,
                )
            if target_is_new and len(roadmap.nodes) < max_nodes:
                frontier.append(target.node_id)

    return roadmap


def build_static_primitive_roadmap(
    sample_count: int,
    rpm1: float,
    rpm2: float,
    vehicle_params: VehicleParams,
    collision: CollisionParams,
    clearance: float,
    static_world: StaticWorld,
    config: PlannerConfig,
    *,
    seed: int | None = None,
    position_tolerance_m: float | None = None,
    heading_tolerance_rad: float | None = None,
) -> Roadmap:
    """Build a minimal roadmap for future temporal query work."""

    roadmap = Roadmap()
    for pose in sample_roadmap_nodes(sample_count, static_world, collision, clearance, seed=seed):
        roadmap.add_node(pose)
    return connect_nodes_with_primitives(
        roadmap=roadmap,
        primitives=get_action_set(rpm1, rpm2, vehicle_params),
        vehicle_params=vehicle_params,
        collision=collision,
        clearance=clearance,
        static_world=static_world,
        config=config,
        position_tolerance_m=position_tolerance_m,
        heading_tolerance_rad=heading_tolerance_rad,
    )


def build_sampled_nonholonomic_roadmap(
    xy_sample_count: int,
    primitives: list[MotionPrimitive],
    vehicle_params: VehicleParams,
    collision: CollisionParams,
    clearance: float,
    static_world: StaticWorld,
    config: PlannerConfig,
    *,
    headings_rad: Iterable[float] = DEFAULT_DISCRETE_HEADINGS_RAD,
    sampling_mode: str = "random",
    grid_spacing_m: float | None = None,
    seed: int | None = None,
    position_tolerance_m: float | None = None,
    heading_tolerance_rad: float | None = None,
    max_outgoing_edges_per_node: int | None = None,
) -> Roadmap:
    """Build the Phase 1 reusable sampled nonholonomic static roadmap."""

    roadmap = Roadmap()
    poses = sample_xy_heading_roadmap_nodes(
        xy_sample_count,
        static_world,
        collision,
        clearance,
        config,
        headings_rad=headings_rad,
        sampling_mode=sampling_mode,
        grid_spacing_m=grid_spacing_m,
        seed=seed,
    )
    for pose in poses:
        roadmap.add_node(pose)
    return connect_sampled_nodes_with_primitives(
        roadmap,
        primitives,
        vehicle_params,
        collision,
        clearance,
        static_world,
        config,
        position_tolerance_m=position_tolerance_m,
        heading_tolerance_rad=heading_tolerance_rad,
        max_outgoing_edges_per_node=max_outgoing_edges_per_node,
    )
