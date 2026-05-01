#!/usr/bin/env python3
"""Generate SVG roadmap/path overlays for representative scenarios."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[1]
if REPO_ROOT.as_posix() not in sys.path:
    sys.path.insert(0, REPO_ROOT.as_posix())

from src.experiments.method_registry import build_sampled_temporal_planner
from src.experiments.scenarios import make_repeated_query_config, repeated_query_workload, representative_dynamic_obstacles
from src.experiments.io_utils import write_json, pose_to_dict, obstacle_to_dict


def _project(x: float, y: float, *, width: int, height: int, margin: int, world_w: float, world_h: float) -> tuple[float, float]:
    px = margin + (x / world_w) * (width - 2 * margin)
    py = height - margin - (y / world_h) * (height - 2 * margin)
    return px, py


def _polyline(points: list[tuple[float, float]], color: str, width: float, opacity: float = 1.0) -> str:
    data = " ".join(f"{x:.1f},{y:.1f}" for x, y in points)
    return f'<polyline points="{data}" fill="none" stroke="{color}" stroke-width="{width}" opacity="{opacity}" stroke-linecap="round" stroke-linejoin="round"/>'


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate representative path overlay SVG.")
    parser.add_argument("--output-dir", type=Path, default=Path("outputs/experiments/figures"))
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    config = make_repeated_query_config(world_size_m=8.0, xy_sample_count=320)
    obstacles = representative_dynamic_obstacles()
    start, goal = repeated_query_workload(8.0, 1)[0]
    planner, build_time, annotation_time = build_sampled_temporal_planner("actea", start, obstacles, config)
    result = planner.plan(start, goal, obstacles, clearance=config.clearance)
    representative_annotation = next(
        (
            annotation
            for annotation in planner.temporal_annotation_store.annotations.values()
            if annotation.blocked_intervals_exact and annotation.valid_intervals_exact
        ),
        None,
    )
    representative_edge = planner.roadmap.edges.get(representative_annotation.edge_id) if representative_annotation else None

    width = 980
    height = 640
    margin = 60
    world_w = config.planner_config.world_width_m
    world_h = config.planner_config.world_height_m
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        '<text x="490" y="30" text-anchor="middle" font-family="Arial" font-size="18">Sampled Temporal Roadmap with ACTEA Path</text>',
        f'<rect x="{margin}" y="{margin}" width="{width - 2 * margin}" height="{height - 2 * margin}" fill="#f8fafc" stroke="#334155"/>',
    ]

    for obstacle in config.static_world.obstacles:
        ox, oy = _project(obstacle.center_x - obstacle.width / 2.0, obstacle.center_y + obstacle.height / 2.0, width=width, height=height, margin=margin, world_w=world_w, world_h=world_h)
        ow = obstacle.width / world_w * (width - 2 * margin)
        oh = obstacle.height / world_h * (height - 2 * margin)
        parts.append(f'<rect x="{ox:.1f}" y="{oy:.1f}" width="{ow:.1f}" height="{oh:.1f}" fill="#475569" opacity="0.45"/>')

    for edge in planner.roadmap.edges.values():
        pts = [
            _project(sample.x, sample.y, width=width, height=height, margin=margin, world_w=world_w, world_h=world_h)
            for sample in edge.trajectory.samples
        ]
        parts.append(_polyline(pts, "#94a3b8", 0.6, 0.25))

    for node in planner.roadmap.nodes.values():
        x, y = _project(node.pose.x, node.pose.y, width=width, height=height, margin=margin, world_w=world_w, world_h=world_h)
        parts.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="1.5" fill="#64748b" opacity="0.45"/>')

    if result.path:
        for segment in result.path.segments:
            pts = [
                _project(sample.x, sample.y, width=width, height=height, margin=margin, world_w=world_w, world_h=world_h)
                for sample in segment.samples
            ]
            parts.append(_polyline(pts, "#2563eb", 4.0, 0.95))

    if representative_edge is not None:
        pts = [
            _project(sample.x, sample.y, width=width, height=height, margin=margin, world_w=world_w, world_h=world_h)
            for sample in representative_edge.trajectory.samples
        ]
        parts.append(_polyline(pts, "#f97316", 6.0, 0.98))

    for obstacle in obstacles:
        x0, y0 = _project(obstacle.initial_x, obstacle.initial_y, width=width, height=height, margin=margin, world_w=world_w, world_h=world_h)
        x1, y1 = _project(
            obstacle.initial_x + obstacle.velocity_x * config.temporal_annotation_end_time_s,
            obstacle.initial_y + obstacle.velocity_y * config.temporal_annotation_end_time_s,
            width=width,
            height=height,
            margin=margin,
            world_w=world_w,
            world_h=world_h,
        )
        radius_px = obstacle.radius / world_w * (width - 2 * margin)
        parts.append(f'<line x1="{x0:.1f}" y1="{y0:.1f}" x2="{x1:.1f}" y2="{y1:.1f}" stroke="#ef4444" stroke-width="2.5" stroke-dasharray="6 4"/>')
        parts.append(f'<circle cx="{x0:.1f}" cy="{y0:.1f}" r="{radius_px:.1f}" fill="#ef4444" opacity="0.35" stroke="#991b1b"/>')

    sx, sy = _project(start.x, start.y, width=width, height=height, margin=margin, world_w=world_w, world_h=world_h)
    gx, gy = _project(goal.x, goal.y, width=width, height=height, margin=margin, world_w=world_w, world_h=world_h)
    parts.append(f'<circle cx="{sx:.1f}" cy="{sy:.1f}" r="7" fill="#16a34a"/><text x="{sx + 10:.1f}" y="{sy - 8:.1f}" font-family="Arial" font-size="12">start</text>')
    parts.append(f'<circle cx="{gx:.1f}" cy="{gy:.1f}" r="7" fill="#7c3aed"/><text x="{gx + 10:.1f}" y="{gy - 8:.1f}" font-family="Arial" font-size="12">goal</text>')
    if representative_annotation is not None:
        timeline_x = 90
        timeline_y = height - 48
        timeline_w = width - 180
        horizon_start, horizon_end = representative_annotation.temporal_horizon_s
        horizon_span = max(horizon_end - horizon_start, 1e-9)
        parts.append(f'<text x="{timeline_x}" y="{timeline_y - 18}" font-family="Arial" font-size="12">ACTEA departure-time annotation for edge {representative_annotation.edge_id}</text>')
        parts.append(f'<rect x="{timeline_x}" y="{timeline_y}" width="{timeline_w}" height="14" fill="#dcfce7" stroke="#166534"/>')
        for interval_start, interval_end in representative_annotation.blocked_intervals_exact:
            x0 = timeline_x + ((interval_start - horizon_start) / horizon_span) * timeline_w
            x1 = timeline_x + ((interval_end - horizon_start) / horizon_span) * timeline_w
            parts.append(f'<rect x="{x0:.1f}" y="{timeline_y}" width="{max(x1 - x0, 1.0):.1f}" height="14" fill="#ef4444" opacity="0.88"/>')
        parts.append(f'<text x="{timeline_x + timeline_w - 8}" y="{timeline_y - 18}" text-anchor="end" font-family="Arial" font-size="12" fill="#166534">green = valid, red = blocked</text>')
        parts.append(f'<text x="{timeline_x}" y="{timeline_y + 28}" font-family="Arial" font-size="11">{horizon_start:.1f}s</text>')
        parts.append(f'<text x="{timeline_x + timeline_w}" y="{timeline_y + 28}" text-anchor="end" font-family="Arial" font-size="11">{horizon_end:.1f}s</text>')
    parts.append("</svg>")

    svg_path = args.output_dir / "representative_actea_overlay.svg"
    svg_path.write_text("\n".join(parts) + "\n", encoding="utf-8")
    write_json(
        args.output_dir / "representative_actea_overlay.json",
        {
            "success": result.success,
            "method": "sampled_temporal_actea",
            "build_time_sec": build_time,
            "annotation_time_sec": annotation_time,
            "start": pose_to_dict(start),
            "goal": pose_to_dict(goal),
            "obstacles": [obstacle_to_dict(obstacle) for obstacle in obstacles],
            "representative_edge_id": representative_annotation.edge_id if representative_annotation else None,
            "representative_edge_has_blocked_intervals": bool(representative_annotation and representative_annotation.blocked_intervals_exact),
            "representative_edge_has_valid_intervals": bool(representative_annotation and representative_annotation.valid_intervals_exact),
            "path_length": result.path.total_cost if result.path else None,
            "traversal_time": result.path.total_traversal_time if result.path else None,
            "representative_edge_annotation": representative_annotation,
        },
    )
    print(f"Wrote {svg_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
