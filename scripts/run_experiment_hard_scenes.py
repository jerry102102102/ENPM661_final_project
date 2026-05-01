#!/usr/bin/env python3
"""Experiment 3: hard dynamic-scene benchmark for ACTEA and baselines."""

from __future__ import annotations

import argparse
from dataclasses import replace
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[1]
if REPO_ROOT.as_posix() not in sys.path:
    sys.path.insert(0, REPO_ROOT.as_posix())

from src.experiments.io_utils import method_result_to_dict, obstacle_to_dict, pose_to_dict, write_csv, write_json
from src.experiments.method_registry import MethodRunConfig, plan_with_method
from src.experiments.scenarios import hard_scene_families
from src.models.obstacles import DynamicCircleObstacle


def _scale_obstacles(obstacles: list[DynamicCircleObstacle], speed_scale: float, seed_index: int) -> list[DynamicCircleObstacle]:
    jitter = 1.0 + 0.05 * seed_index
    return [
        DynamicCircleObstacle(
            obstacle.initial_x,
            obstacle.initial_y,
            obstacle.velocity_x * speed_scale * jitter,
            obstacle.velocity_y * speed_scale * jitter,
            obstacle.radius,
            obstacle.label,
        )
        for obstacle in obstacles
    ]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run hard dynamic-scene benchmark.")
    parser.add_argument("--output-dir", type=Path, default=Path("outputs/experiments/hard_scenes"))
    parser.add_argument("--seeds", type=int, nargs="+", default=[0])
    parser.add_argument("--speed-scales", type=float, nargs="+", default=[1.0])
    parser.add_argument("--heuristic-mode", default="goal_distance_heading_time_lb")
    parser.add_argument(
        "--methods",
        nargs="+",
        default=[
            "reactive_replanning",
            "expansion_temporal",
            "sampled_temporal_online",
            "sampled_temporal_actea",
        ],
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rows: list[dict] = []
    scene_payload: dict[str, object] = {}
    for scene in hard_scene_families():
        scene_family = str(scene["scene_family"])
        base_config = scene["config"]
        assert isinstance(base_config, MethodRunConfig)
        queries = scene["queries"]
        base_obstacles = scene["obstacles"]
        assert isinstance(base_obstacles, list)
        scene_payload[scene_family] = {
            "obstacles": [obstacle_to_dict(obstacle) for obstacle in base_obstacles],
            "query_count": len(queries),
        }
        for seed in args.seeds:
            config = replace(base_config, seed=seed, heuristic_mode=args.heuristic_mode)
            for speed_scale in args.speed_scales:
                obstacles = _scale_obstacles(base_obstacles, speed_scale, seed)
                for query_index, (start, goal) in enumerate(queries):
                    for method in args.methods:
                        result = plan_with_method(method, start, goal, obstacles, config)
                        row = method_result_to_dict(result)
                        row.update(
                            {
                                "experiment_name": "hard_dynamic_scenes",
                                "scene_family": scene_family,
                                "seed": seed,
                                "query_index": query_index,
                                "world_size_m": config.planner_config.world_width_m,
                                "heuristic_mode": args.heuristic_mode,
                                "start_pose": pose_to_dict(start),
                                "goal_pose": pose_to_dict(goal),
                                "obstacle_count": len(obstacles),
                                "obstacle_speed_scale": speed_scale,
                                "total_runtime": (result.build_time_sec or 0.0)
                                + (result.annotation_time_sec or 0.0)
                                + result.query_time_sec,
                            }
                        )
                        rows.append(row)

    payload = {
        "experiment_name": "hard_dynamic_scenes",
        "heuristic_mode": args.heuristic_mode,
        "scenarios": scene_payload,
        "rows": rows,
    }
    write_json(args.output_dir / "results.json", payload)
    write_csv(args.output_dir / "results.csv", rows)
    print(f"Wrote {args.output_dir / 'results.json'}")
    print(f"Wrote {args.output_dir / 'results.csv'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
