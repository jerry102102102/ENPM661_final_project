#!/usr/bin/env python3
"""Independent heuristic ablation for sampled temporal online and ACTEA planners."""

from __future__ import annotations

import argparse
from dataclasses import replace
from pathlib import Path
import sys
import time

REPO_ROOT = Path(__file__).resolve().parents[1]
if REPO_ROOT.as_posix() not in sys.path:
    sys.path.insert(0, REPO_ROOT.as_posix())

from src.experiments.io_utils import method_result_to_dict, obstacle_to_dict, write_csv, write_json
from src.experiments.method_registry import MethodRunResult, build_sampled_temporal_planner
from src.experiments.scenarios import make_repeated_query_config, repeated_query_workload, representative_dynamic_obstacles


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run temporal planner heuristic ablation.")
    parser.add_argument("--output-dir", type=Path, default=Path("outputs/experiments/heuristic_ablation"))
    parser.add_argument("--world-size", type=float, default=8.0)
    parser.add_argument("--xy-samples", type=int, default=320)
    parser.add_argument("--query-count", type=int, default=50)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    base_config = make_repeated_query_config(world_size_m=args.world_size, xy_sample_count=args.xy_samples)
    obstacles = representative_dynamic_obstacles()
    queries = repeated_query_workload(args.world_size, args.query_count)
    rows: list[dict] = []
    for heuristic_mode in ("euclidean", "goal_distance_heading_time_lb"):
        config = replace(base_config, heuristic_mode=heuristic_mode)
        for method, mode in (("sampled_temporal_online", "online"), ("sampled_temporal_actea", "actea")):
            planner, build_time, annotation_time = build_sampled_temporal_planner(mode, queries[0][0], obstacles, config)
            query_time_total = 0.0
            success_count = 0
            expanded_labels_total = 0
            rejected_dynamic_edges_total = 0
            last_result = None
            for start, goal in queries:
                query_start = time.perf_counter()
                result = planner.plan(start, goal, obstacles, clearance=config.clearance)
                query_time_total += time.perf_counter() - query_start
                success_count += int(result.success)
                expanded_labels_total += result.expanded_labels
                rejected_dynamic_edges_total += result.rejected_dynamic_edges
                last_result = result
            method_result = MethodRunResult(
                method_name=method,
                success=success_count == len(queries),
                message=f"{success_count}/{len(queries)} queries succeeded.",
                path_length=last_result.path.total_cost if last_result and last_result.path else None,
                traversal_time=last_result.path.total_traversal_time if last_result and last_result.path else None,
                query_time_sec=query_time_total / max(len(queries), 1),
                build_time_sec=build_time,
                annotation_time_sec=annotation_time,
                expanded_labels=expanded_labels_total,
                rejected_dynamic_edges=rejected_dynamic_edges_total,
                cache_stats=last_result.debug.get("temporal_cache_total_stats") if last_result else {},
                roadmap_nodes=len(planner.roadmap.nodes),
                roadmap_edges=len(planner.roadmap.edges),
                path_segments=last_result.path.segments if last_result and last_result.path else [],
            )
            row = method_result_to_dict(method_result)
            row.update(
                {
                    "experiment_name": "heuristic_ablation",
                    "heuristic_mode": heuristic_mode,
                    "world_size_m": args.world_size,
                    "query_count": len(queries),
                    "avg_query_time_sec": query_time_total / max(len(queries), 1),
                    "total_query_time_sec": query_time_total,
                    "total_runtime": build_time + annotation_time + query_time_total,
                    "success_rate": success_count / max(len(queries), 1),
                    "expanded_labels_total": expanded_labels_total,
                    "rejected_dynamic_edges_total": rejected_dynamic_edges_total,
                }
            )
            rows.append(row)

    payload = {
        "experiment_name": "heuristic_ablation",
        "world_size_m": args.world_size,
        "obstacles": [obstacle_to_dict(obstacle) for obstacle in obstacles],
        "rows": rows,
    }
    write_json(args.output_dir / "results.json", payload)
    write_csv(args.output_dir / "results.csv", rows)
    print(f"Wrote {args.output_dir / 'results.json'}")
    print(f"Wrote {args.output_dir / 'results.csv'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
