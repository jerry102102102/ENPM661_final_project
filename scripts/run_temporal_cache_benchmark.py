#!/usr/bin/env python3
"""Compare online validation, bin cache, and ACTEA interval annotation."""

from __future__ import annotations

import json
import math
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[1]
if REPO_ROOT.as_posix() not in sys.path:
    sys.path.insert(0, REPO_ROOT.as_posix())

from src.builders.roadmap_builder import build_sampled_nonholonomic_roadmap
from src.configs.defaults import CollisionParams, PlannerConfig, VehicleParams
from src.models.obstacles import DynamicCircleObstacle, StaticWorld
from src.models.primitives import get_action_set
from src.models.state import Pose2D
from src.planners.temporal_roadmap_planner import TemporalRoadmapPlanner, TemporalRoadmapPlannerConfig


def _write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def _make_world():
    config = PlannerConfig(
        world_width_m=2.0,
        world_height_m=2.0,
        xy_resolution_m=0.05,
        theta_bins=32,
        action_duration_s=1.0,
        integration_dt_s=0.1,
    )
    static_world = StaticWorld(config.bounds, [])
    vehicle = VehicleParams(0.1, 0.4, 0.4, "differential_drive")
    collision = CollisionParams(radius_m=0.05, source="benchmark")
    primitives = get_action_set(30.0, 60.0, vehicle)
    roadmap = build_sampled_nonholonomic_roadmap(
        xy_sample_count=100,
        primitives=primitives,
        vehicle_params=vehicle,
        collision=collision,
        clearance=0.0,
        static_world=static_world,
        config=config,
        headings_rad=[0.0, math.pi / 2.0, -math.pi / 2.0, math.pi],
        sampling_mode="grid",
        grid_spacing_m=0.2,
        position_tolerance_m=0.15,
        heading_tolerance_rad=0.4,
    )
    return config, static_world, vehicle, collision, primitives, roadmap


def _run_mode(mode: str) -> dict:
    config, static_world, vehicle, collision, primitives, roadmap = _make_world()
    dynamic_obstacles = [DynamicCircleObstacle(0.8, 0.2, 0.0, 0.0, 0.08, "blocker")]
    planner = TemporalRoadmapPlanner(
        roadmap,
        static_world,
        collision,
        vehicle,
        primitives,
        config,
        TemporalRoadmapPlannerConfig(
            max_arrival_time_s=30.0,
            time_bin_size_s=0.25,
            goal_tolerance_m=0.15,
            goal_heading_tolerance_rad=math.radians(30.0),
            connection_position_tolerance_m=0.15,
            connection_heading_tolerance_rad=math.radians(30.0),
            temporal_annotation_mode=mode,
        ),
    )
    if mode == "actea":
        planner.annotate_edge_obstacle_interactions(dynamic_obstacles, start_time_s=0.0, end_time_s=8.0)
        planner.annotate_temporal_intervals(dynamic_obstacles, start_time_s=0.0, end_time_s=8.0, step_s=0.25)
    elif mode == "bin_cache":
        planner.annotate_edge_obstacle_interactions(dynamic_obstacles, start_time_s=0.0, end_time_s=8.0)

    queries = [
        (Pose2D(0.2, 0.2, 0.0), Pose2D(1.6, 0.2, 0.0)),
        (Pose2D(0.2, 0.2, 0.0), Pose2D(1.6, 0.2, 0.0)),
        (Pose2D(0.2, 1.0, 0.0), Pose2D(1.6, 1.0, 0.0)),
    ]
    results = [planner.plan(start, goal, dynamic_obstacles) for start, goal in queries]
    return {
        "mode": mode,
        "successes": [result.success for result in results],
        "expanded_labels": [result.expanded_labels for result in results],
        "rejected_dynamic_edges": [result.rejected_dynamic_edges for result in results],
        "cache_stats": [result.debug.get("temporal_cache_stats", {}) for result in results],
        "cache_total_stats": results[-1].debug.get("temporal_cache_total_stats", {}),
        "roadmap_nodes": len(roadmap.nodes),
        "roadmap_edges": len(roadmap.edges),
    }


def main() -> int:
    payload = {
        "modes": [_run_mode(mode) for mode in ("online", "bin_cache", "actea")],
    }
    output = Path("outputs/temporal_cache_benchmark/summary.json")
    _write_json(output, payload)
    for item in payload["modes"]:
        print(f"mode={item['mode']} successes={item['successes']} expanded={item['expanded_labels']}")
        print(f"  cache_total_stats={item['cache_total_stats']}")
    print(f"Summary JSON: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
