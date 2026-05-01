#!/usr/bin/env python3
"""Experiment 4: sampled roadmap reuse vs expansion-built temporal graph."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[1]
if REPO_ROOT.as_posix() not in sys.path:
    sys.path.insert(0, REPO_ROOT.as_posix())

from src.experiments.io_utils import method_result_to_dict, obstacle_to_dict, pose_to_dict, write_csv, write_json
from src.experiments.method_registry import plan_with_method
from src.experiments.scenarios import make_open_world_config, repeated_query_pairs, representative_dynamic_obstacles


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run sampled roadmap vs expansion graph experiment.")
    parser.add_argument("--output-dir", type=Path, default=Path("outputs/experiments/sampled_vs_expansion"))
    parser.add_argument("--xy-samples", type=int, default=100)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    config = make_open_world_config(xy_sample_count=args.xy_samples)
    obstacles = representative_dynamic_obstacles()
    methods = ["expansion_temporal", "sampled_temporal_online", "sampled_temporal_actea"]
    rows: list[dict] = []
    for query_index, (start, goal) in enumerate(repeated_query_pairs()):
        for method in methods:
            result = plan_with_method(method, start, goal, obstacles, config)
            row = method_result_to_dict(result)
            row.update(
                {
                    "experiment_name": "sampled_vs_expansion",
                    "query_index": query_index,
                    "start_pose": pose_to_dict(start),
                    "goal_pose": pose_to_dict(goal),
                    "total_runtime": (result.build_time_sec or 0.0)
                    + (result.annotation_time_sec or 0.0)
                    + result.query_time_sec,
                }
            )
            rows.append(row)

    payload = {
        "experiment_name": "sampled_vs_expansion",
        "obstacles": [obstacle_to_dict(obstacle) for obstacle in obstacles],
        "rows": rows,
    }
    write_json(args.output_dir / "results.json", payload)
    write_csv(args.output_dir / "results.csv", rows)
    print(f"Wrote {args.output_dir / 'results.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
