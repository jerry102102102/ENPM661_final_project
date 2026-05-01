#!/usr/bin/env python3
"""Experiment 4: roadmap-scale ablation for ACTEA."""

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
from src.experiments.scenarios import ablation_configs, repeated_query_workload, representative_dynamic_obstacles


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run roadmap-scale ablation experiment.")
    parser.add_argument("--output-dir", type=Path, default=Path("outputs/experiments/roadmap_scale_ablation"))
    parser.add_argument("--queries-per-regime", type=int, default=25)
    parser.add_argument("--extra-dense-queries", type=int, default=10)
    parser.add_argument("--heuristic-mode", default="goal_distance_heading_time_lb")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    obstacles = representative_dynamic_obstacles()
    rows: list[dict] = []
    for variant, config in ablation_configs():
        config = replace(config, heuristic_mode=args.heuristic_mode)
        world_size = config.planner_config.world_width_m
        query_limit = args.extra_dense_queries if variant == "extra_dense" else args.queries_per_regime
        queries = repeated_query_workload(world_size, query_limit)
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
                    "experiment_name": "roadmap_scale_ablation",
                    "roadmap_scale": variant,
                    "world_size_m": world_size,
                    "query_count": len(queries),
                    "heuristic_mode": args.heuristic_mode,
                    "xy_sample_count": config.xy_sample_count,
                    "heading_count": len(config.headings_rad),
                    "grid_spacing_m": config.grid_spacing_m,
                    "position_tolerance_m": config.position_tolerance_m,
                    "heading_tolerance_rad": config.heading_tolerance_rad,
                    "max_outgoing_edges_per_node": config.max_outgoing_edges_per_node,
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
        "experiment_name": "roadmap_scale_ablation",
        "heuristic_mode": args.heuristic_mode,
        "obstacles": [obstacle_to_dict(obstacle) for obstacle in obstacles],
        "rows": rows,
    }
    write_json(args.output_dir / "results.json", payload)
    write_csv(args.output_dir / "results.csv", rows)
    print(f"Wrote {args.output_dir / 'results.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
