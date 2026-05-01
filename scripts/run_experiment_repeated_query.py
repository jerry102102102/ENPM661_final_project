#!/usr/bin/env python3
"""Experiment 2: repeated-query scaling for sampled temporal roadmap variants."""

from __future__ import annotations

import argparse
from dataclasses import replace
from pathlib import Path
import sys
import time

REPO_ROOT = Path(__file__).resolve().parents[1]
if REPO_ROOT.as_posix() not in sys.path:
    sys.path.insert(0, REPO_ROOT.as_posix())

from src.experiments.io_utils import method_result_to_dict, obstacle_to_dict, pose_to_dict, write_csv, write_json
from src.experiments.method_registry import MethodRunResult, build_sampled_temporal_planner
from src.experiments.scenarios import make_repeated_query_config, repeated_query_workload, representative_dynamic_obstacles


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run repeated-query experiment.")
    parser.add_argument("--output-dir", type=Path, default=Path("outputs/experiments/repeated_query"))
    parser.add_argument("--world-size", type=float, default=8.0)
    parser.add_argument("--xy-samples", type=int, default=320)
    parser.add_argument("--query-counts", type=int, nargs="+", default=[10, 25, 50, 100, 200])
    parser.add_argument("--heuristic-mode", default="goal_distance_heading_time_lb")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    config = replace(
        make_repeated_query_config(world_size_m=args.world_size, xy_sample_count=args.xy_samples),
        heuristic_mode=args.heuristic_mode,
    )
    obstacles = representative_dynamic_obstacles()
    base_queries = repeated_query_workload(args.world_size, max(args.query_counts))
    rows: list[dict] = []
    methods = [
        ("sampled_temporal_online", "online"),
        ("sampled_temporal_bin_cache", "bin_cache"),
        ("sampled_temporal_actea", "actea"),
    ]

    for method_name, mode in methods:
        planner, build_time, annotation_time = build_sampled_temporal_planner(mode, base_queries[0][0], obstacles, config)
        cumulative_query_time = 0.0
        success_count = 0
        expanded_labels_total = 0
        rejected_dynamic_edges_total = 0
        interval_hits_total = 0
        cache_hits_total = 0
        interaction_cache_hits_total = 0
        max_query_count = max(args.query_counts)
        for query_index in range(max_query_count):
            start, goal = base_queries[query_index]
            query_start = time.perf_counter()
            result = planner.plan(start, goal, obstacles, clearance=config.clearance)
            query_time = time.perf_counter() - query_start
            cumulative_query_time += query_time
            if result.success:
                success_count += 1
            expanded_labels_total += result.expanded_labels
            rejected_dynamic_edges_total += result.rejected_dynamic_edges
            stats = result.debug.get("temporal_cache_stats", {})
            if isinstance(stats, dict):
                interval_hits_total += int(stats.get("interval_cache_hits_free", 0)) + int(stats.get("interval_cache_hits_blocked", 0))
                cache_hits_total += int(stats.get("temporal_cache_hits_free", 0)) + int(stats.get("temporal_cache_hits_blocked", 0))
                interaction_cache_hits_total += int(stats.get("interaction_cache_hits", 0))
            method_result = MethodRunResult(
                method_name=method_name,
                success=result.success,
                message=result.message,
                path_length=result.path.total_cost if result.path else None,
                traversal_time=result.path.total_traversal_time if result.path else None,
                query_time_sec=query_time,
                build_time_sec=build_time,
                annotation_time_sec=annotation_time,
                expanded_labels=result.expanded_labels,
                rejected_dynamic_edges=result.rejected_dynamic_edges,
                cache_stats=result.debug.get("temporal_cache_stats"),
                roadmap_nodes=result.roadmap_node_count,
                roadmap_edges=result.roadmap_edge_count,
                path_segments=result.path.segments if result.path else [],
            )
            if query_index + 1 in args.query_counts:
                row = method_result_to_dict(method_result)
                row.update(
                    {
                        "experiment_name": "repeated_query_scaling",
                        "query_count": query_index + 1,
                        "query_index": query_index,
                        "world_size_m": args.world_size,
                        "heuristic_mode": args.heuristic_mode,
                        "start_pose": pose_to_dict(start),
                        "goal_pose": pose_to_dict(goal),
                        "build_time": build_time,
                        "annotation_time": annotation_time,
                        "avg_query_time_sec": cumulative_query_time / (query_index + 1),
                        "avg_query_time": cumulative_query_time / (query_index + 1),
                        "cumulative_query_time_sec": cumulative_query_time,
                        "total_runtime": build_time + annotation_time + cumulative_query_time,
                        "total_runtime_over_N": build_time + annotation_time + cumulative_query_time,
                        "amortized_cost_per_query_sec": (build_time + annotation_time + cumulative_query_time) / (query_index + 1),
                        "success_rate": success_count / (query_index + 1),
                        "expanded_labels_total": expanded_labels_total,
                        "rejected_dynamic_edges_total": rejected_dynamic_edges_total,
                        "interval_hits": interval_hits_total,
                        "cache_hits": cache_hits_total,
                        "interaction_cache_hits_total": interaction_cache_hits_total,
                    }
                )
                rows.append(row)

    payload = {
        "experiment_name": "repeated_query_scaling",
        "world_size_m": args.world_size,
        "heuristic_mode": args.heuristic_mode,
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
