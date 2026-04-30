#!/usr/bin/env python3
"""Debug temporal validation on a single primitive rollout."""

from __future__ import annotations

from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[1]
if REPO_ROOT.as_posix() not in sys.path:
    sys.path.insert(0, REPO_ROOT.as_posix())

from src.configs.defaults import PlannerConfig, builtin_collision_params, builtin_vehicle_params
from src.configs.environment import build_project3_world
from src.core.rollout import simulate_primitive
from src.core.temporal_validation import temporal_collision_free
from src.models.obstacles import DynamicCircleObstacle
from src.models.primitives import MotionPrimitive
from src.models.state import Pose2D


def main() -> int:
    config = PlannerConfig()
    collision = builtin_collision_params("turtlebot_circle")
    vehicle = builtin_vehicle_params("turtlebot")
    trajectory = simulate_primitive(
        Pose2D(0.25, 0.25, 0.0),
        MotionPrimitive(60.0, 60.0, "straight"),
        vehicle,
        action_duration_s=1.0,
        integration_dt_s=0.1,
    )
    dynamic_obstacles = [
        DynamicCircleObstacle(initial_x=0.6, initial_y=0.25, velocity_x=0.0, velocity_y=0.0, radius=0.05),
    ]
    ok = temporal_collision_free(
        trajectory,
        start_time_s=0.0,
        static_world=build_project3_world(),
        dynamic_obstacles=dynamic_obstacles,
        collision=collision,
        clearance=0.0,
    )
    print(f"temporal_collision_free={ok}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
