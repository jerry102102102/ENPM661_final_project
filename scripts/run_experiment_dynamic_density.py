#!/usr/bin/env python3
"""Experiment 2: dynamic obstacle density comparison."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[1]
if REPO_ROOT.as_posix() not in sys.path:
    sys.path.insert(0, REPO_ROOT.as_posix())

from src.experiments.io_utils import method_result_to_dict, obstacle_to_dict, pose_to_dict, write_csv, write_json
from src.experiments.method_registry import plan_with_method
from src.experiments.scenarios import density_obstacles, make_open_world_config
from src.models.state import Pose2D


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run dynamic obstacle density experiment.")
    parser.add_argument("--output-dir", type=Path, default=Path("outputs/experiments/dynamic_density"))
    parser.add_argument("--xy-samples", type=int, default=100)
    parser.add_argument(
        "--methods",
        nargs="+",
        default=[
            "static_astar",
            "reactive_replanning",
            "expansion_temporal",
            "sampled_temporal_online",
            "sampled_temporal_actea",
            "sampled_temporal_bin_cache",
        ],
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    config = make_open_world_config(xy_sample_count=args.xy_samples)
    start = Pose2D(0.2, 0.2, 0.0)
    goal = Pose2D(1.6, 0.2, 0.0)
    rows: list[dict] = []
    scenario_payload: dict[str, object] = {}

    for density in ("low", "medium", "high"):
        obstacles = density_obstacles(density)
        scenario_payload[density] = [obstacle_to_dict(obstacle) for obstacle in obstacles]
        for method in args.methods:
            result = plan_with_method(method, start, goal, obstacles, config)
            row = method_result_to_dict(result)
            row.update(
                {
                    "experiment_name": "dynamic_density",
                    "density": density,
                    "obstacle_count": len(obstacles),
                    "start_pose": pose_to_dict(start),
                    "goal_pose": pose_to_dict(goal),
                }
            )
            rows.append(row)

    payload = {
        "experiment_name": "dynamic_density",
        "scenarios": scenario_payload,
        "rows": rows,
    }
    write_json(args.output_dir / "results.json", payload)
    write_csv(args.output_dir / "results.csv", rows)
    print(f"Wrote {args.output_dir / 'results.json'}")
    print(f"Wrote {args.output_dir / 'results.csv'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
