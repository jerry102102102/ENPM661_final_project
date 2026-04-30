#!/usr/bin/env python3
"""Build small static primitive roadmaps for debugging."""

from __future__ import annotations

from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[1]
if REPO_ROOT.as_posix() not in sys.path:
    sys.path.insert(0, REPO_ROOT.as_posix())

import math

from src.builders.roadmap_builder import (
    build_sampled_nonholonomic_roadmap,
    build_static_primitive_roadmap,
    connect_nodes_with_primitives,
)
from src.configs.defaults import PlannerConfig, builtin_collision_params, builtin_vehicle_params
from src.configs.environment import build_project3_world
from src.core.rollout import simulate_primitive
from src.models.obstacles import StaticWorld
from src.models.primitives import MotionPrimitive, get_action_set
from src.models.roadmap import Roadmap
from src.models.state import Pose2D


def main() -> int:
    smoke_config = PlannerConfig(world_width_m=2.0, world_height_m=2.0, integration_dt_s=0.1)
    smoke_vehicle = builtin_vehicle_params("turtlebot")
    smoke_collision = builtin_collision_params("turtlebot_circle")
    smoke_primitive = MotionPrimitive(60.0, 60.0, "straight")
    start = Pose2D(0.4, 0.4, 0.0)
    rollout = simulate_primitive(start, smoke_primitive, smoke_vehicle, 1.0, 0.1)
    smoke_roadmap = Roadmap()
    smoke_roadmap.add_node(start)
    smoke_roadmap.add_node(rollout.end)
    connect_nodes_with_primitives(
        smoke_roadmap,
        [smoke_primitive],
        smoke_vehicle,
        smoke_collision,
        clearance=0.0,
        static_world=StaticWorld((0.0, 2.0, 0.0, 2.0), []),
        config=smoke_config,
        position_tolerance_m=1e-9,
        heading_tolerance_rad=1e-9,
    )

    roadmap = build_static_primitive_roadmap(
        sample_count=30,
        rpm1=20.0,
        rpm2=40.0,
        vehicle_params=builtin_vehicle_params("team_car"),
        collision=builtin_collision_params("team_car_circle"),
        clearance=0.02,
        static_world=build_project3_world(),
        config=PlannerConfig(),
        seed=7,
        position_tolerance_m=0.25,
        heading_tolerance_rad=0.75,
    )
    sampled_config = PlannerConfig(
        world_width_m=2.0,
        world_height_m=2.0,
        xy_resolution_m=0.05,
        theta_bins=32,
        action_duration_s=1.0,
        integration_dt_s=0.1,
    )
    sampled_vehicle = builtin_vehicle_params("turtlebot")
    sampled_collision = builtin_collision_params("turtlebot_circle")
    sampled_roadmap = build_sampled_nonholonomic_roadmap(
        xy_sample_count=100,
        primitives=get_action_set(30.0, 60.0, sampled_vehicle),
        vehicle_params=sampled_vehicle,
        collision=sampled_collision,
        clearance=0.0,
        static_world=StaticWorld(sampled_config.bounds, []),
        config=sampled_config,
        headings_rad=[0.0, math.pi / 2.0, -math.pi / 2.0, math.pi],
        sampling_mode="grid",
        grid_spacing_m=0.2,
        position_tolerance_m=0.15,
        heading_tolerance_rad=0.4,
    )
    print(f"smoke_nodes={len(smoke_roadmap.nodes)} smoke_edges={len(smoke_roadmap.edges)}")
    print(f"legacy_random_nodes={len(roadmap.nodes)} legacy_random_edges={len(roadmap.edges)}")
    print(f"sampled_prm_nodes={len(sampled_roadmap.nodes)} sampled_prm_edges={len(sampled_roadmap.edges)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
