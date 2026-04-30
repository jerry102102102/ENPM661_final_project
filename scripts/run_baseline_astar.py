#!/usr/bin/env python3
"""Run the preserved static nonholonomic A* baseline."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[1]
if REPO_ROOT.as_posix() not in sys.path:
    sys.path.insert(0, REPO_ROOT.as_posix())

from src.configs.defaults import PlannerConfig, builtin_collision_params, builtin_vehicle_params
from src.configs.environment import build_project3_world
from src.models.state import Pose2D
from src.planners.baseline_nonholonomic_astar import plan


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run static nonholonomic A* baseline.")
    parser.add_argument("--start", nargs=3, type=float, metavar=("x", "y", "theta_deg"), required=True)
    parser.add_argument("--goal", nargs=2, type=float, metavar=("x", "y"), required=True)
    parser.add_argument("--rpm1", type=float, required=True)
    parser.add_argument("--rpm2", type=float, required=True)
    parser.add_argument("--clearance", type=float, default=0.02)
    parser.add_argument("--motion-profile", choices=["team_car", "turtlebot"], default="team_car")
    parser.add_argument(
        "--collision-profile",
        choices=["team_car_circle", "team_car_box", "turtlebot_circle"],
        default="team_car_circle",
    )
    parser.add_argument("--max-iterations", type=int, default=50000)
    parser.add_argument("--output-dir", type=Path, default=Path("outputs/baseline_astar"))
    parser.add_argument("--save-prefix", default="baseline_astar")
    return parser.parse_args()


def _write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    config = PlannerConfig(max_iterations=args.max_iterations, output_dir=args.output_dir)
    start_pose = Pose2D(args.start[0], args.start[1], math.radians(args.start[2]))
    goal_xy = (args.goal[0], args.goal[1])

    result = plan(
        start_pose=start_pose,
        goal_xy=goal_xy,
        rpm1=args.rpm1,
        rpm2=args.rpm2,
        clearance=args.clearance,
        vehicle_params=builtin_vehicle_params(args.motion_profile),
        collision_params=builtin_collision_params(args.collision_profile),
        static_world=build_project3_world(),
        config=config,
    )

    summary = {
        "success": result.success,
        "message": result.message,
        "runtime_sec": result.runtime_sec,
        "expanded_nodes": result.expanded_nodes,
        "path_cost": result.path_cost if math.isfinite(result.path_cost) else "inf",
        "path_sample_count": len(result.path_samples),
        "path_segment_count": len(result.path_segments),
        "explored_segment_count": len(result.explored_segments),
        "start": {"x": start_pose.x, "y": start_pose.y, "theta_deg": args.start[2]},
        "goal": {"x": goal_xy[0], "y": goal_xy[1]},
        "motion_profile": args.motion_profile,
        "collision_profile": args.collision_profile,
    }

    summary_path = args.output_dir / f"{args.save_prefix}_summary.json"
    log_path = args.output_dir / f"{args.save_prefix}_planner_log.json"
    _write_json(summary_path, summary)
    _write_json(log_path, result.planner_log)

    print(f"Success: {result.success}")
    print(f"Message: {result.message}")
    print(f"Expanded nodes: {result.expanded_nodes}")
    print(f"Runtime: {result.runtime_sec:.3f} sec")
    print(f"Summary: {summary_path}")
    print(f"Planner log: {log_path}")
    return 0 if result.success else 2


if __name__ == "__main__":
    raise SystemExit(main())
