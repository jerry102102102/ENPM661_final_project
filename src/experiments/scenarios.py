"""Deterministic benchmark scenarios for ACTEA experiments."""

from __future__ import annotations

import math
from dataclasses import replace

from src.configs.defaults import CollisionParams, PlannerConfig, VehicleParams
from src.experiments.method_registry import MethodRunConfig
from src.models.obstacles import DynamicCircleObstacle, StaticWorld
from src.models.state import OrientedBox, Pose2D
from src.planners.reactive_replanning_baseline import ReactiveReplanningConfig


HEADINGS_4: tuple[float, ...] = (0.0, math.pi / 2.0, math.pi, -math.pi / 2.0)
HEADINGS_8: tuple[float, ...] = (
    0.0,
    math.pi / 4.0,
    math.pi / 2.0,
    3.0 * math.pi / 4.0,
    math.pi,
    -3.0 * math.pi / 4.0,
    -math.pi / 2.0,
    -math.pi / 4.0,
)


def make_benchmark_world_config(
    *,
    world_size_m: float,
    xy_sample_count: int,
    grid_spacing_m: float,
    temporal_annotation_end_time_s: float,
    static_obstacles: list[OrientedBox] | None = None,
    headings_rad: tuple[float, ...] = HEADINGS_8,
    max_outgoing_edges_per_node: int | None = 8,
    max_iterations: int = 30000,
    max_expanded_labels: int = 60000,
) -> MethodRunConfig:
    """Return a large-world experiment config with roadmap and temporal settings."""

    planner_config = PlannerConfig(
        world_width_m=world_size_m,
        world_height_m=world_size_m,
        xy_resolution_m=0.10,
        theta_bins=64,
        action_duration_s=1.0,
        integration_dt_s=0.1,
        goal_tolerance_m=0.35,
        goal_heading_tolerance_rad=math.radians(35.0),
        max_iterations=max_iterations,
    )
    return MethodRunConfig(
        static_world=StaticWorld(planner_config.bounds, static_obstacles or []),
        vehicle_params=VehicleParams(0.1, 0.4, 0.4, "differential_drive"),
        collision_params=CollisionParams(radius_m=0.06, source="experiment"),
        planner_config=planner_config,
        xy_sample_count=xy_sample_count,
        headings_rad=headings_rad,
        sampling_mode="grid",
        grid_spacing_m=grid_spacing_m,
        position_tolerance_m=max(0.28, grid_spacing_m * 0.75),
        heading_tolerance_rad=math.radians(32.0),
        max_outgoing_edges_per_node=max_outgoing_edges_per_node,
        expansion_max_nodes=900,
        expansion_max_expansions=900,
        temporal_max_arrival_time_s=temporal_annotation_end_time_s,
        temporal_time_bin_size_s=0.25,
        temporal_goal_tolerance_m=0.35,
        temporal_goal_heading_tolerance_rad=math.radians(35.0),
        temporal_connection_position_tolerance_m=max(0.30, grid_spacing_m * 0.75),
        temporal_connection_heading_tolerance_rad=math.radians(35.0),
        temporal_annotation_end_time_s=temporal_annotation_end_time_s,
        reactive_config=ReactiveReplanningConfig(
            replan_period_s=1.0,
            lookahead_time_s=4.0,
            max_replans=60,
            max_total_time_s=temporal_annotation_end_time_s,
            collision_buffer=0.02,
            execution_step_dt_s=0.1,
            replan_on_predicted_invalidity=True,
        ),
    )


def benchmark_world_presets() -> dict[str, MethodRunConfig]:
    """Large-world presets used by the main experiments."""

    return {
        "6x6": make_benchmark_world_config(
            world_size_m=6.0,
            xy_sample_count=180,
            grid_spacing_m=0.50,
            temporal_annotation_end_time_s=15.0,
        ),
        "8x8": make_benchmark_world_config(
            world_size_m=8.0,
            xy_sample_count=320,
            grid_spacing_m=0.50,
            temporal_annotation_end_time_s=20.0,
        ),
        "10x10": make_benchmark_world_config(
            world_size_m=10.0,
            xy_sample_count=520,
            grid_spacing_m=0.50,
            temporal_annotation_end_time_s=30.0,
        ),
    }


def _repeated_query_static_obstacles(world_size_m: float) -> list[OrientedBox]:
    mid = world_size_m / 2.0
    return [
        OrientedBox("central_gate_lower", mid, 1.55, 0.18, 2.45),
        OrientedBox("central_gate_upper", mid, world_size_m - 1.55, 0.18, 2.45),
        OrientedBox("left_cross_wall", mid - 1.55, mid, 1.55, 0.18),
        OrientedBox("right_cross_wall", mid + 1.55, mid, 1.55, 0.18),
    ]


def make_repeated_query_config(*, world_size_m: float = 8.0, xy_sample_count: int = 320) -> MethodRunConfig:
    """Large repeated-query world with static gates and an 8-heading roadmap."""

    if world_size_m <= 6.0:
        horizon = 15.0
    elif world_size_m <= 8.0:
        horizon = 20.0
    else:
        horizon = 30.0
    return make_benchmark_world_config(
        world_size_m=world_size_m,
        xy_sample_count=xy_sample_count,
        grid_spacing_m=0.50,
        temporal_annotation_end_time_s=horizon,
        static_obstacles=_repeated_query_static_obstacles(world_size_m),
        headings_rad=HEADINGS_8,
        max_outgoing_edges_per_node=8,
    )


def make_open_world_config(
    *,
    xy_sample_count: int = 100,
    grid_spacing_m: float = 0.2,
    max_iterations: int = 10000,
) -> MethodRunConfig:
    """Compatibility wrapper for older smoke tests using a 2 m x 2 m world."""

    planner_config = PlannerConfig(
        world_width_m=2.0,
        world_height_m=2.0,
        xy_resolution_m=0.05,
        theta_bins=32,
        action_duration_s=1.0,
        integration_dt_s=0.1,
        goal_tolerance_m=0.15,
        max_iterations=max_iterations,
    )
    return MethodRunConfig(
        static_world=StaticWorld(planner_config.bounds, []),
        vehicle_params=VehicleParams(0.1, 0.4, 0.4, "differential_drive"),
        collision_params=CollisionParams(radius_m=0.05, source="experiment"),
        planner_config=planner_config,
        xy_sample_count=xy_sample_count,
        headings_rad=HEADINGS_4,
        sampling_mode="grid",
        grid_spacing_m=grid_spacing_m,
        position_tolerance_m=0.15,
        heading_tolerance_rad=0.4,
        temporal_annotation_end_time_s=8.0,
        reactive_config=ReactiveReplanningConfig(max_replans=20, max_total_time_s=60.0),
    )


def repeated_query_pairs() -> list[tuple[Pose2D, Pose2D]]:
    """Compatibility workload for smoke tests."""

    return [
        (Pose2D(0.2, 0.2, 0.0), Pose2D(1.6, 0.2, 0.0)),
        (Pose2D(0.2, 1.0, 0.0), Pose2D(1.6, 1.0, 0.0)),
        (Pose2D(0.2, 1.4, 0.0), Pose2D(1.6, 1.4, 0.0)),
    ]


def repeated_query_workload(world_size_m: float = 8.0, count: int = 200) -> list[tuple[Pose2D, Pose2D]]:
    """Generate mixed long-range, diagonal, gate, and heading-sensitive queries."""

    w = world_size_m
    m = 0.5
    mid = w / 2.0
    base = [
        (Pose2D(m, 1.0, 0.0), Pose2D(w - m, 1.0, 0.0)),
        (Pose2D(m, w - 1.0, 0.0), Pose2D(w - m, w - 1.0, math.pi)),
        (Pose2D(1.0, m, math.pi / 2.0), Pose2D(w - 1.0, w - m, math.pi / 2.0)),
        (Pose2D(w - 1.0, m, math.pi / 2.0), Pose2D(1.0, w - m, math.pi)),
        (Pose2D(m, mid - 1.0, 0.0), Pose2D(w - m, mid + 1.0, 0.0)),
        (Pose2D(m, mid + 1.0, 0.0), Pose2D(w - m, mid - 1.0, -math.pi / 2.0)),
        (Pose2D(1.0, mid, 0.0), Pose2D(w - 1.0, mid, 0.0)),
        (Pose2D(mid - 1.5, m, math.pi / 2.0), Pose2D(mid + 1.5, w - m, math.pi / 2.0)),
        (Pose2D(w - m, 2.0, math.pi), Pose2D(m, w - 2.0, math.pi)),
        (Pose2D(2.0, w - m, -math.pi / 2.0), Pose2D(w - 2.0, m, 0.0)),
        (Pose2D(1.5, 1.5, math.pi / 4.0), Pose2D(w - 1.5, w - 1.5, math.pi / 4.0)),
        (Pose2D(1.5, w - 1.5, -math.pi / 4.0), Pose2D(w - 1.5, 1.5, -math.pi / 4.0)),
    ]
    return [base[index % len(base)] for index in range(count)]


def representative_dynamic_obstacles() -> list[DynamicCircleObstacle]:
    """Moving obstacle scenario with many time-sensitive edge windows."""

    return [
        DynamicCircleObstacle(3.0, -0.4, 0.0, 0.42, 0.16, "northbound_flow_a"),
        DynamicCircleObstacle(4.1, 8.4, 0.0, -0.40, 0.16, "southbound_flow_b"),
        DynamicCircleObstacle(-0.4, 3.2, 0.42, 0.0, 0.15, "eastbound_flow_c"),
        DynamicCircleObstacle(8.4, 4.7, -0.40, 0.0, 0.15, "westbound_flow_d"),
        DynamicCircleObstacle(2.5, 8.3, 0.12, -0.36, 0.13, "diagonal_flow_e"),
        DynamicCircleObstacle(8.2, 2.2, -0.36, 0.14, 0.13, "diagonal_flow_f"),
    ]


def actea_correctness_obstacle_scenarios() -> list[tuple[str, list[DynamicCircleObstacle]]]:
    """Obstacle scenarios used to compare ACTEA lookup against online checks."""

    return [
        ("static_blocker", [DynamicCircleObstacle(0.8, 0.2, 0.0, 0.0, 0.08, "static_blocker")]),
        ("crossing", [DynamicCircleObstacle(0.9, -0.1, 0.0, 0.28, 0.07, "crossing")]),
        (
            "multi_obstacle",
            [
                DynamicCircleObstacle(0.7, 0.2, 0.0, 0.0, 0.07, "fixed_a"),
                DynamicCircleObstacle(1.1, 1.7, -0.02, -0.20, 0.07, "descending_b"),
                DynamicCircleObstacle(0.4, 1.0, 0.18, 0.0, 0.06, "sweeper_c"),
            ],
        ),
    ]


def density_obstacles(level: str) -> list[DynamicCircleObstacle]:
    """Return deterministic low/medium/high dynamic-obstacle scenarios."""

    if level == "low":
        return representative_dynamic_obstacles()[:2]
    if level == "medium":
        return representative_dynamic_obstacles()[:4]
    if level == "high":
        return representative_dynamic_obstacles()
    raise ValueError(f"Unknown density level: {level}")


def ablation_configs() -> list[tuple[str, MethodRunConfig]]:
    """Roadmap regimes for the roadmap-scale ACTEA ablation."""

    static_obstacles = _repeated_query_static_obstacles(8.0)
    return [
        (
            "sparse",
            make_benchmark_world_config(
                world_size_m=8.0,
                xy_sample_count=160,
                grid_spacing_m=0.65,
                temporal_annotation_end_time_s=20.0,
                static_obstacles=static_obstacles,
                headings_rad=HEADINGS_4,
                max_outgoing_edges_per_node=5,
            ),
        ),
        (
            "medium",
            make_benchmark_world_config(
                world_size_m=8.0,
                xy_sample_count=300,
                grid_spacing_m=0.50,
                temporal_annotation_end_time_s=20.0,
                static_obstacles=static_obstacles,
                headings_rad=HEADINGS_8,
                max_outgoing_edges_per_node=7,
            ),
        ),
        (
            "dense",
            make_benchmark_world_config(
                world_size_m=10.0,
                xy_sample_count=480,
                grid_spacing_m=0.50,
                temporal_annotation_end_time_s=30.0,
                static_obstacles=_repeated_query_static_obstacles(10.0),
                headings_rad=HEADINGS_8,
                max_outgoing_edges_per_node=8,
            ),
        ),
        (
            "extra_dense",
            make_benchmark_world_config(
                world_size_m=10.0,
                xy_sample_count=760,
                grid_spacing_m=0.40,
                temporal_annotation_end_time_s=30.0,
                static_obstacles=_repeated_query_static_obstacles(10.0),
                headings_rad=HEADINGS_8,
                max_outgoing_edges_per_node=6,
                max_expanded_labels=70000,
            ),
        ),
    ]


def _hard_base(world_size_m: float = 8.0) -> MethodRunConfig:
    return make_benchmark_world_config(
        world_size_m=world_size_m,
        xy_sample_count=320 if world_size_m <= 8.0 else 520,
        grid_spacing_m=0.50,
        temporal_annotation_end_time_s=20.0 if world_size_m <= 8.0 else 30.0,
        headings_rad=HEADINGS_8,
        max_outgoing_edges_per_node=8,
    )


def _five_queries(world_size_m: float) -> list[tuple[Pose2D, Pose2D]]:
    return repeated_query_workload(world_size_m, 5)


def hard_scene_families() -> list[dict[str, object]]:
    """Six dynamic-scene families with dense moving traffic and five queries each."""

    base8 = _hard_base(8.0)
    base10 = _hard_base(10.0)
    gate_obstacles_8 = _repeated_query_static_obstacles(8.0)
    gate_obstacles_10 = _repeated_query_static_obstacles(10.0)
    bottleneck_config = replace(base8, static_world=StaticWorld(base8.planner_config.bounds, gate_obstacles_8))
    tight_config = replace(
        base8,
        static_world=StaticWorld(
            base8.planner_config.bounds,
            [
                OrientedBox("upper_corridor_wall", 4.0, 5.25, 7.0, 0.22),
                OrientedBox("lower_corridor_wall", 4.0, 2.75, 7.0, 0.22),
                OrientedBox("entry_chicane_a", 2.2, 3.55, 0.22, 1.1),
                OrientedBox("entry_chicane_b", 5.8, 4.45, 0.22, 1.1),
            ],
        ),
    )
    cluttered_config = replace(
        base10,
        static_world=StaticWorld(
            base10.planner_config.bounds,
            gate_obstacles_10
            + [
                OrientedBox("corridor_split_a", 2.6, 7.2, 2.2, 0.22),
                OrientedBox("corridor_split_b", 7.4, 2.8, 2.2, 0.22),
                OrientedBox("central_island", 5.0, 5.0, 0.8, 0.8),
            ],
        ),
    )
    return [
        {
            "scene_family": "cross_traffic_dense",
            "config": base8,
            "queries": _five_queries(8.0),
            "obstacles": [
                DynamicCircleObstacle(1.2, -0.5, 0.0, 0.38, 0.14, "flow_y_1"),
                DynamicCircleObstacle(2.2, 8.5, 0.0, -0.36, 0.14, "flow_y_2"),
                DynamicCircleObstacle(3.4, -0.6, 0.0, 0.42, 0.14, "flow_y_3"),
                DynamicCircleObstacle(4.6, 8.6, 0.0, -0.40, 0.14, "flow_y_4"),
                DynamicCircleObstacle(5.8, -0.5, 0.0, 0.37, 0.14, "flow_y_5"),
                DynamicCircleObstacle(6.8, 8.5, 0.0, -0.39, 0.14, "flow_y_6"),
            ],
        },
        {
            "scene_family": "moving_bottleneck_multiwave",
            "config": bottleneck_config,
            "queries": _five_queries(8.0),
            "obstacles": [
                DynamicCircleObstacle(3.55, 1.2, 0.0, 0.30, 0.15, "gate_wave_a"),
                DynamicCircleObstacle(4.05, 6.8, 0.0, -0.32, 0.15, "gate_wave_b"),
                DynamicCircleObstacle(4.45, 1.0, 0.0, 0.34, 0.14, "gate_wave_c"),
                DynamicCircleObstacle(3.75, 7.0, 0.0, -0.28, 0.14, "gate_wave_d"),
                DynamicCircleObstacle(2.6, 4.05, 0.28, 0.0, 0.13, "gate_cross_e"),
                DynamicCircleObstacle(5.4, 3.85, -0.28, 0.0, 0.13, "gate_cross_f"),
            ],
        },
        {
            "scene_family": "longitudinal_interaction_dense",
            "config": base8,
            "queries": _five_queries(8.0),
            "obstacles": [
                DynamicCircleObstacle(1.5, 1.0, 0.16, 0.0, 0.14, "slow_same_a"),
                DynamicCircleObstacle(2.8, 1.5, 0.12, 0.0, 0.14, "slow_same_b"),
                DynamicCircleObstacle(6.5, 2.0, -0.22, 0.0, 0.14, "opposite_c"),
                DynamicCircleObstacle(6.8, 6.8, -0.18, 0.0, 0.14, "opposite_d"),
                DynamicCircleObstacle(2.0, 5.8, 0.18, 0.0, 0.14, "same_e"),
                DynamicCircleObstacle(5.5, 4.0, -0.20, 0.0, 0.14, "opposite_f"),
            ],
        },
        {
            "scene_family": "tight_corridor_heading_dense",
            "config": tight_config,
            "queries": _five_queries(8.0),
            "obstacles": [
                DynamicCircleObstacle(1.5, 3.35, 0.25, 0.0, 0.12, "corridor_lane_a"),
                DynamicCircleObstacle(6.5, 4.65, -0.25, 0.0, 0.12, "corridor_lane_b"),
                DynamicCircleObstacle(3.2, 3.45, 0.0, 0.18, 0.12, "vertical_block_c"),
                DynamicCircleObstacle(4.8, 4.55, 0.0, -0.18, 0.12, "vertical_block_d"),
                DynamicCircleObstacle(2.2, 4.1, 0.24, -0.05, 0.12, "diagonal_e"),
                DynamicCircleObstacle(5.8, 3.9, -0.24, 0.05, 0.12, "diagonal_f"),
            ],
        },
        {
            "scene_family": "cluttered_multi_corridor",
            "config": cluttered_config,
            "queries": _five_queries(10.0),
            "obstacles": [
                DynamicCircleObstacle(1.2, -0.6, 0.0, 0.34, 0.14, "corridor_cross_a"),
                DynamicCircleObstacle(3.2, 10.6, 0.0, -0.32, 0.14, "corridor_cross_b"),
                DynamicCircleObstacle(5.0, -0.5, 0.0, 0.30, 0.14, "corridor_cross_c"),
                DynamicCircleObstacle(7.2, 10.5, 0.0, -0.34, 0.14, "corridor_cross_d"),
                DynamicCircleObstacle(-0.5, 3.0, 0.34, 0.0, 0.14, "corridor_lateral_e"),
                DynamicCircleObstacle(10.5, 7.0, -0.32, 0.0, 0.14, "corridor_lateral_f"),
                DynamicCircleObstacle(2.0, 8.5, 0.24, -0.22, 0.13, "diagonal_g"),
                DynamicCircleObstacle(8.5, 2.0, -0.24, 0.22, 0.13, "diagonal_h"),
            ],
        },
        {
            "scene_family": "mixed_flow_intersection",
            "config": base10,
            "queries": _five_queries(10.0),
            "obstacles": [
                DynamicCircleObstacle(5.0, -0.6, 0.0, 0.36, 0.14, "north_a"),
                DynamicCircleObstacle(5.5, 10.6, 0.0, -0.34, 0.14, "south_b"),
                DynamicCircleObstacle(-0.6, 5.0, 0.36, 0.0, 0.14, "east_c"),
                DynamicCircleObstacle(10.6, 5.5, -0.34, 0.0, 0.14, "west_d"),
                DynamicCircleObstacle(2.0, 10.5, 0.22, -0.30, 0.13, "diag_e"),
                DynamicCircleObstacle(8.0, -0.5, -0.22, 0.30, 0.13, "diag_f"),
                DynamicCircleObstacle(-0.5, 2.2, 0.30, 0.22, 0.13, "diag_g"),
                DynamicCircleObstacle(10.5, 8.0, -0.30, -0.22, 0.13, "diag_h"),
                DynamicCircleObstacle(3.0, 4.2, 0.20, 0.08, 0.12, "inner_i"),
                DynamicCircleObstacle(7.0, 5.8, -0.20, -0.08, 0.12, "inner_j"),
            ],
        },
    ]
