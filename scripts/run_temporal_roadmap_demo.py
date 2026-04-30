#!/usr/bin/env python3
"""Run a small sampled nonholonomic temporal roadmap demo."""

from __future__ import annotations

import argparse
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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run temporal roadmap demo.")
    parser.add_argument("--xy-samples", type=int, default=100)
    parser.add_argument("--seed-obstacle-radius", type=float, default=0.08)
    parser.add_argument("--use-cache", action="store_true")
    parser.add_argument("--precompute-intervals", action="store_true")
    parser.add_argument("--output-json", type=Path, default=Path("outputs/temporal_roadmap_demo/summary.json"))
    return parser.parse_args()


def _write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    config = PlannerConfig(
        world_width_m=2.0,
        world_height_m=2.0,
        xy_resolution_m=0.05,
        theta_bins=32,
        action_duration_s=1.0,
        integration_dt_s=0.1,
    )
    static_world = StaticWorld(config.bounds, [])
    vehicle = VehicleParams(
        wheel_radius_m=0.1,
        track_width_m=0.4,
        wheelbase_m=0.4,
        motion_model="differential_drive",
    )
    collision = CollisionParams(radius_m=0.05, source="demo")
    primitives = get_action_set(30.0, 60.0, vehicle)
    start = Pose2D(0.2, 0.2, 0.0)
    goal = Pose2D(1.6, 0.2, 0.0)

    roadmap = build_sampled_nonholonomic_roadmap(
        xy_sample_count=args.xy_samples,
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
    dynamic_obstacles = [
        DynamicCircleObstacle(
            initial_x=0.8,
            initial_y=0.2,
            velocity_x=0.0,
            velocity_y=0.0,
            radius=args.seed_obstacle_radius,
            label="crossing_blocker",
        )
    ]

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
            use_temporal_cache=args.use_cache or args.precompute_intervals,
            use_temporal_intervals=args.precompute_intervals,
        ),
    )
    if args.precompute_intervals:
        planner.annotate_edge_obstacle_interactions(
            dynamic_obstacles,
            start_time_s=0.0,
            end_time_s=8.0,
        )
        planner.annotate_temporal_intervals(
            dynamic_obstacles,
            start_time_s=0.0,
            end_time_s=8.0,
            step_s=0.25,
        )
    result = planner.plan(start, goal, dynamic_obstacles, clearance=0.0)

    payload = {
        "success": result.success,
        "message": result.message,
        "total_cost": result.path.total_cost if result.path else None,
        "total_traversal_time": result.path.total_traversal_time if result.path else None,
        "expanded_labels": result.expanded_labels,
        "generated_labels": result.generated_labels,
        "pruned_labels": result.pruned_labels,
        "rejected_edges_due_to_dynamic_collision": result.rejected_dynamic_edges,
        "roadmap_nodes": result.roadmap_node_count,
        "roadmap_edges": result.roadmap_edge_count,
        "roadmap_type": "sampled_nonholonomic_prm",
        "use_cache": args.use_cache,
        "precompute_intervals": args.precompute_intervals,
        "start": {"x": start.x, "y": start.y, "theta": start.theta},
        "goal": {"x": goal.x, "y": goal.y, "theta": goal.theta},
        "path_node_count": len(result.path.node_ids) if result.path else 0,
        "path_actions": result.path.actions if result.path else [],
        "arrival_times": result.path.arrival_times if result.path else [],
        "debug": result.debug,
    }
    _write_json(args.output_json, payload)

    print(f"Path found: {result.success}")
    print(f"Message: {result.message}")
    print(f"Total cost: {payload['total_cost']}")
    print(f"Total traversal time: {payload['total_traversal_time']}")
    print(f"Expanded labels: {result.expanded_labels}")
    print(f"Rejected edges due to dynamic collision: {result.rejected_dynamic_edges}")
    print(f"Roadmap size: nodes={result.roadmap_node_count}, edges={result.roadmap_edge_count}")
    if result.debug.get("temporal_cache_stats"):
        print(f"Temporal cache stats: {result.debug['temporal_cache_stats']}")
    print(f"Summary JSON: {args.output_json}")
    return 0 if result.success else 2


if __name__ == "__main__":
    raise SystemExit(main())
