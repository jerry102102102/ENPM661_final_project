#!/usr/bin/env python3
"""Experiment 1: ACTEA interval lookup vs online exact validation."""

from __future__ import annotations

import argparse
import random
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[1]
if REPO_ROOT.as_posix() not in sys.path:
    sys.path.insert(0, REPO_ROOT.as_posix())

from src.builders.roadmap_builder import build_sampled_nonholonomic_roadmap
from src.core.temporal_cache import CachedTemporalValidator
from src.core.temporal_validation import temporal_collision_free
from src.experiments.io_utils import obstacle_to_dict, write_csv, write_json
from src.experiments.scenarios import actea_correctness_obstacle_scenarios, make_open_world_config
from src.models.primitives import get_action_set


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run ACTEA correctness experiment.")
    parser.add_argument("--output-dir", type=Path, default=Path("outputs/experiments/actea_correctness"))
    parser.add_argument("--xy-samples", type=int, default=100)
    parser.add_argument("--samples", type=int, default=200)
    parser.add_argument("--seed", type=int, default=7)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rng = random.Random(args.seed)
    config = make_open_world_config(xy_sample_count=args.xy_samples)
    primitives = get_action_set(config.rpm1, config.rpm2, config.vehicle_params)
    roadmap = build_sampled_nonholonomic_roadmap(
        xy_sample_count=config.xy_sample_count,
        primitives=primitives,
        vehicle_params=config.vehicle_params,
        collision=config.collision_params,
        clearance=config.clearance,
        static_world=config.static_world,
        config=config.planner_config,
        headings_rad=config.headings_rad,
        sampling_mode=config.sampling_mode,
        grid_spacing_m=config.grid_spacing_m,
        seed=config.seed,
        position_tolerance_m=config.position_tolerance_m,
        heading_tolerance_rad=config.heading_tolerance_rad,
    )
    edge_ids = list(roadmap.edges)
    rows: list[dict] = []
    false_blocked = 0
    false_free = 0
    agreement = 0
    by_scenario: dict[str, dict[str, int]] = {}
    obstacle_payload: dict[str, object] = {}
    for scenario_id, obstacles in actea_correctness_obstacle_scenarios():
        obstacle_payload[scenario_id] = [obstacle_to_dict(obstacle) for obstacle in obstacles]
        validator = CachedTemporalValidator(
            static_world=config.static_world,
            collision=config.collision_params,
            clearance=config.clearance,
            time_bin_size_s=config.temporal_time_bin_size_s,
            use_interval_lookup=True,
        )
        for edge in roadmap.edges.values():
            validator.annotate_edge_temporal_annotation(
                edge,
                obstacles,
                start_time_s=0.0,
                end_time_s=config.temporal_annotation_end_time_s,
            )
        by_scenario[scenario_id] = {"samples": 0, "agreement": 0, "false_blocked": 0, "false_free": 0}
        for sample_index in range(args.samples):
            edge = roadmap.edges[rng.choice(edge_ids)]
            departure = rng.uniform(0.0, config.temporal_annotation_end_time_s)
            annotation = validator.annotation_store.get(edge.edge_id, obstacles)
            actea_status = annotation.status_at(departure) if annotation else None
            online_status = temporal_collision_free(
                edge.trajectory,
                departure,
                config.static_world,
                obstacles,
                config.collision_params,
                clearance=config.clearance,
            )
            matched = actea_status == online_status
            by_scenario[scenario_id]["samples"] += 1
            if matched:
                agreement += 1
                by_scenario[scenario_id]["agreement"] += 1
            elif actea_status is False and online_status is True:
                false_blocked += 1
                by_scenario[scenario_id]["false_blocked"] += 1
            elif actea_status is True and online_status is False:
                false_free += 1
                by_scenario[scenario_id]["false_free"] += 1
            rows.append(
                {
                    "experiment_name": "actea_correctness",
                    "obstacle_scenario_id": scenario_id,
                    "sample_index": sample_index,
                    "edge_id": edge.edge_id,
                    "departure_time_s": departure,
                    "actea_valid": actea_status,
                    "online_valid": online_status,
                    "agreement": matched,
                }
            )

    total_samples = args.samples * len(by_scenario)
    summary = {
        "experiment_name": "actea_correctness",
        "sample_count": total_samples,
        "obstacle_scenario_count": len(by_scenario),
        "agreement_rate": agreement / max(total_samples, 1),
        "disagreement_rate": (false_blocked + false_free) / max(total_samples, 1),
        "false_blocked": false_blocked,
        "false_free": false_free,
        "roadmap_nodes": len(roadmap.nodes),
        "roadmap_edges": len(roadmap.edges),
        "obstacle_scenarios": obstacle_payload,
        "by_scenario": by_scenario,
    }
    write_json(args.output_dir / "results.json", {"summary": summary, "rows": rows})
    write_csv(args.output_dir / "results.csv", rows)
    write_csv(args.output_dir / "summary.csv", [summary])
    print(f"Agreement rate: {summary['agreement_rate']:.3f}")
    print(f"Wrote {args.output_dir / 'results.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
