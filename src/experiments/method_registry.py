"""Uniform experiment-facing wrappers for planner variants."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
import time
from collections.abc import Callable

from src.builders.roadmap_builder import (
    DEFAULT_DISCRETE_HEADINGS_RAD,
    build_primitive_expansion_roadmap,
    build_sampled_nonholonomic_roadmap,
)
from src.configs.defaults import CollisionParams, PlannerConfig, VehicleParams
from src.core.temporal_validation import temporal_collision_free
from src.models.obstacles import DynamicCircleObstacle, StaticWorld
from src.models.primitives import get_action_set
from src.models.state import Pose2D, TrajectorySegment
from src.planners.baseline_nonholonomic_astar import plan as static_astar_plan
from src.planners.reactive_replanning_baseline import (
    ReactiveReplanningBaseline,
    ReactiveReplanningConfig,
)
from src.planners.temporal_roadmap_planner import TemporalRoadmapPlanner, TemporalRoadmapPlannerConfig


MethodPlanFn = Callable[
    [Pose2D, Pose2D, list[DynamicCircleObstacle], "MethodRunConfig"],
    "MethodRunResult",
]


@dataclass(frozen=True)
class MethodRunConfig:
    """Shared configuration for experiment method wrappers."""

    static_world: StaticWorld
    vehicle_params: VehicleParams
    collision_params: CollisionParams
    planner_config: PlannerConfig
    rpm1: float = 30.0
    rpm2: float = 60.0
    clearance: float = 0.0
    xy_sample_count: int = 100
    headings_rad: tuple[float, ...] = DEFAULT_DISCRETE_HEADINGS_RAD
    sampling_mode: str = "grid"
    grid_spacing_m: float | None = 0.2
    seed: int | None = None
    position_tolerance_m: float | None = 0.15
    heading_tolerance_rad: float | None = 0.4
    max_outgoing_edges_per_node: int | None = None
    expansion_max_nodes: int = 500
    expansion_max_expansions: int = 500
    temporal_max_arrival_time_s: float = 30.0
    temporal_time_bin_size_s: float = 0.25
    temporal_goal_tolerance_m: float = 0.15
    temporal_goal_heading_tolerance_rad: float = math.radians(30.0)
    temporal_connection_position_tolerance_m: float = 0.15
    temporal_connection_heading_tolerance_rad: float = math.radians(30.0)
    temporal_annotation_end_time_s: float = 8.0
    heuristic_mode: str = "euclidean"
    reactive_config: ReactiveReplanningConfig = field(default_factory=ReactiveReplanningConfig)


@dataclass(frozen=True)
class MethodRunResult:
    """Common result schema used by experiment scripts."""

    method_name: str
    success: bool
    message: str
    path_length: float | None
    traversal_time: float | None
    query_time_sec: float
    build_time_sec: float | None = None
    annotation_time_sec: float | None = None
    expanded_labels: int | None = None
    expanded_nodes: int | None = None
    rejected_dynamic_edges: int | None = None
    replans: int | None = None
    dynamic_collision_failures: int | None = None
    cache_stats: dict[str, object] | None = None
    roadmap_nodes: int | None = None
    roadmap_edges: int | None = None
    path_segments: list[TrajectorySegment] = field(default_factory=list)


@dataclass(frozen=True)
class ExperimentMethod:
    """Named planner variant exposed to experiment scripts."""

    name: str
    plan: MethodPlanFn


def _path_length(segments: list[TrajectorySegment]) -> float:
    return sum(segment.cost for segment in segments)


def _traversal_time(segments: list[TrajectorySegment]) -> float:
    return sum(segment.duration_s for segment in segments)


def _dynamic_collision_failures(
    segments: list[TrajectorySegment],
    start_time_s: float,
    dynamic_obstacles: list[DynamicCircleObstacle],
    config: MethodRunConfig,
) -> int:
    failures = 0
    current_time = start_time_s
    for segment in segments:
        if not temporal_collision_free(
            segment,
            current_time,
            config.static_world,
            dynamic_obstacles,
            config.collision_params,
            clearance=config.clearance,
        ):
            failures += 1
        current_time += segment.duration_s
    return failures


def _temporal_config(mode: str, config: MethodRunConfig) -> TemporalRoadmapPlannerConfig:
    return TemporalRoadmapPlannerConfig(
        temporal_annotation_mode=mode,
        max_arrival_time_s=config.temporal_max_arrival_time_s,
        time_bin_size_s=config.temporal_time_bin_size_s,
        goal_tolerance_m=config.temporal_goal_tolerance_m,
        goal_heading_tolerance_rad=config.temporal_goal_heading_tolerance_rad,
        connection_position_tolerance_m=config.temporal_connection_position_tolerance_m,
        connection_heading_tolerance_rad=config.temporal_connection_heading_tolerance_rad,
        heuristic_mode=config.heuristic_mode,
    )


def build_sampled_temporal_planner(
    mode: str,
    start: Pose2D,
    dynamic_obstacles: list[DynamicCircleObstacle],
    config: MethodRunConfig,
) -> tuple[TemporalRoadmapPlanner, float, float]:
    """Build a sampled temporal planner once for repeated-query experiments."""

    primitives = get_action_set(config.rpm1, config.rpm2, config.vehicle_params)
    build_start = time.perf_counter()
    roadmap = build_sampled_nonholonomic_roadmap(
        xy_sample_count=config.xy_sample_count,
        primitives=primitives,
        vehicle_params=config.vehicle_params,
        collision=config.collision_params,
        clearance=config.clearance,
        static_world=config.static_world,
        config=config.planner_config,
        headings_rad=config.headings_rad,
        sampling_mode=config.sampling_mode,
        grid_spacing_m=config.grid_spacing_m,
        seed=config.seed,
        position_tolerance_m=config.position_tolerance_m,
        heading_tolerance_rad=config.heading_tolerance_rad,
        max_outgoing_edges_per_node=config.max_outgoing_edges_per_node,
    )
    build_time = time.perf_counter() - build_start
    planner = TemporalRoadmapPlanner(
        roadmap,
        config.static_world,
        config.collision_params,
        config.vehicle_params,
        primitives,
        config.planner_config,
        _temporal_config(mode, config),
    )
    annotation_start = time.perf_counter()
    if mode == "actea":
        planner.annotate_edge_obstacle_interactions(
            dynamic_obstacles,
            start_time_s=0.0,
            end_time_s=config.temporal_annotation_end_time_s,
            clearance=config.clearance,
        )
        planner.annotate_temporal_intervals(
            dynamic_obstacles,
            start_time_s=0.0,
            end_time_s=config.temporal_annotation_end_time_s,
            clearance=config.clearance,
        )
    elif mode == "bin_cache":
        planner.annotate_edge_obstacle_interactions(
            dynamic_obstacles,
            start_time_s=0.0,
            end_time_s=config.temporal_annotation_end_time_s,
            clearance=config.clearance,
        )
    _ = start
    annotation_time = time.perf_counter() - annotation_start
    return planner, build_time, annotation_time


def _run_temporal_planner(
    method_name: str,
    mode: str,
    start: Pose2D,
    goal: Pose2D,
    dynamic_obstacles: list[DynamicCircleObstacle],
    config: MethodRunConfig,
    *,
    expansion_graph: bool = False,
) -> MethodRunResult:
    primitives = get_action_set(config.rpm1, config.rpm2, config.vehicle_params)
    if expansion_graph:
        build_start = time.perf_counter()
        roadmap = build_primitive_expansion_roadmap(
            seed_poses=[start],
            primitives=primitives,
            vehicle_params=config.vehicle_params,
            collision=config.collision_params,
            clearance=config.clearance,
            static_world=config.static_world,
            config=config.planner_config,
            max_nodes=config.expansion_max_nodes,
            max_expansions=config.expansion_max_expansions,
        )
        build_time = time.perf_counter() - build_start
        annotation_time = 0.0
        planner = TemporalRoadmapPlanner(
            roadmap,
            config.static_world,
            config.collision_params,
            config.vehicle_params,
            primitives,
            config.planner_config,
            _temporal_config(mode, config),
        )
    else:
        planner, build_time, annotation_time = build_sampled_temporal_planner(mode, start, dynamic_obstacles, config)

    query_start = time.perf_counter()
    result = planner.plan(start, goal, dynamic_obstacles, clearance=config.clearance)
    query_time = time.perf_counter() - query_start
    path_segments = result.path.segments if result.path else []
    return MethodRunResult(
        method_name=method_name,
        success=result.success,
        message=result.message,
        path_length=result.path.total_cost if result.path else None,
        traversal_time=result.path.total_traversal_time if result.path else None,
        query_time_sec=query_time,
        build_time_sec=build_time,
        annotation_time_sec=annotation_time,
        expanded_labels=result.expanded_labels,
        rejected_dynamic_edges=result.rejected_dynamic_edges,
        cache_stats=result.debug.get("temporal_cache_stats"),
        roadmap_nodes=result.roadmap_node_count,
        roadmap_edges=result.roadmap_edge_count,
        path_segments=path_segments,
    )


def _run_static_astar(
    start: Pose2D,
    goal: Pose2D,
    dynamic_obstacles: list[DynamicCircleObstacle],
    config: MethodRunConfig,
) -> MethodRunResult:
    result = static_astar_plan(
        start_pose=start,
        goal_xy=(goal.x, goal.y),
        rpm1=config.rpm1,
        rpm2=config.rpm2,
        clearance=config.clearance,
        vehicle_params=config.vehicle_params,
        collision_params=config.collision_params,
        static_world=config.static_world,
        config=config.planner_config,
    )
    dynamic_failures = _dynamic_collision_failures(result.path_segments, 0.0, dynamic_obstacles, config)
    success = result.success and dynamic_failures == 0
    message = result.message if success else f"{result.message} Dynamic collisions during execution: {dynamic_failures}."
    return MethodRunResult(
        method_name="static_astar",
        success=success,
        message=message,
        path_length=result.path_cost if math.isfinite(result.path_cost) else None,
        traversal_time=_traversal_time(result.path_segments) if result.success else None,
        query_time_sec=result.runtime_sec,
        build_time_sec=0.0,
        annotation_time_sec=0.0,
        expanded_nodes=result.expanded_nodes,
        dynamic_collision_failures=dynamic_failures,
        path_segments=result.path_segments,
    )


def _run_reactive_replanning(
    start: Pose2D,
    goal: Pose2D,
    dynamic_obstacles: list[DynamicCircleObstacle],
    config: MethodRunConfig,
) -> MethodRunResult:
    result = ReactiveReplanningBaseline(config.reactive_config).plan(
        start,
        (goal.x, goal.y),
        dynamic_obstacles,
        rpm1=config.rpm1,
        rpm2=config.rpm2,
        clearance=config.clearance,
        vehicle_params=config.vehicle_params,
        collision_params=config.collision_params,
        static_world=config.static_world,
        planner_config=config.planner_config,
    )
    return MethodRunResult(
        method_name="reactive_replanning",
        success=result.success,
        message=result.message,
        path_length=result.path_cost if math.isfinite(result.path_cost) else None,
        traversal_time=result.total_traversal_time,
        query_time_sec=result.query_time_total,
        build_time_sec=0.0,
        annotation_time_sec=0.0,
        expanded_nodes=result.expanded_nodes_total,
        replans=result.number_of_replans,
        dynamic_collision_failures=result.dynamic_collision_failures,
        path_segments=result.path_segments,
    )


def _run_expansion_temporal(
    start: Pose2D,
    goal: Pose2D,
    dynamic_obstacles: list[DynamicCircleObstacle],
    config: MethodRunConfig,
) -> MethodRunResult:
    return _run_temporal_planner(
        "expansion_temporal",
        "online",
        start,
        goal,
        dynamic_obstacles,
        config,
        expansion_graph=True,
    )


def _run_sampled_temporal_online(
    start: Pose2D,
    goal: Pose2D,
    dynamic_obstacles: list[DynamicCircleObstacle],
    config: MethodRunConfig,
) -> MethodRunResult:
    return _run_temporal_planner("sampled_temporal_online", "online", start, goal, dynamic_obstacles, config)


def _run_sampled_temporal_bin_cache(
    start: Pose2D,
    goal: Pose2D,
    dynamic_obstacles: list[DynamicCircleObstacle],
    config: MethodRunConfig,
) -> MethodRunResult:
    return _run_temporal_planner("sampled_temporal_bin_cache", "bin_cache", start, goal, dynamic_obstacles, config)


def _run_sampled_temporal_actea(
    start: Pose2D,
    goal: Pose2D,
    dynamic_obstacles: list[DynamicCircleObstacle],
    config: MethodRunConfig,
) -> MethodRunResult:
    return _run_temporal_planner("sampled_temporal_actea", "actea", start, goal, dynamic_obstacles, config)


METHOD_REGISTRY: dict[str, ExperimentMethod] = {
    "static_astar": ExperimentMethod("static_astar", _run_static_astar),
    "expansion_temporal": ExperimentMethod("expansion_temporal", _run_expansion_temporal),
    "sampled_temporal_online": ExperimentMethod("sampled_temporal_online", _run_sampled_temporal_online),
    "sampled_temporal_actea": ExperimentMethod("sampled_temporal_actea", _run_sampled_temporal_actea),
    "sampled_temporal_bin_cache": ExperimentMethod("sampled_temporal_bin_cache", _run_sampled_temporal_bin_cache),
    "reactive_replanning": ExperimentMethod("reactive_replanning", _run_reactive_replanning),
}

METHOD_ALIASES: dict[str, str] = {
    "sampled_temporal_exact3c": "sampled_temporal_actea",
    "sampled_temporal_cached": "sampled_temporal_bin_cache",
}


def registered_method_names() -> tuple[str, ...]:
    return tuple(METHOD_REGISTRY)


def plan_with_method(
    method_name: str,
    start: Pose2D,
    goal: Pose2D,
    dynamic_obstacles: list[DynamicCircleObstacle],
    config: MethodRunConfig,
) -> MethodRunResult:
    """Run one registered method with the common experiment interface."""

    try:
        method = METHOD_REGISTRY[METHOD_ALIASES.get(method_name, method_name)]
    except KeyError as exc:
        raise ValueError(f"Unknown method: {method_name}") from exc
    return method.plan(start, goal, dynamic_obstacles, config)
