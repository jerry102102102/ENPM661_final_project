"""Serialization helpers for experiment outputs."""

from __future__ import annotations

import csv
import json
from dataclasses import asdict, is_dataclass
from pathlib import Path
from typing import Any

from src.models.obstacles import DynamicCircleObstacle
from src.models.state import Pose2D, TrajectorySegment
from src.experiments.method_registry import MethodRunResult


def ensure_dir(path: Path) -> Path:
    path.mkdir(parents=True, exist_ok=True)
    return path


def pose_to_dict(pose: Pose2D) -> dict[str, float]:
    return {"x": pose.x, "y": pose.y, "theta": pose.theta}


def obstacle_to_dict(obstacle: DynamicCircleObstacle) -> dict[str, float | str]:
    return {
        "label": obstacle.label,
        "initial_x": obstacle.initial_x,
        "initial_y": obstacle.initial_y,
        "velocity_x": obstacle.velocity_x,
        "velocity_y": obstacle.velocity_y,
        "radius": obstacle.radius,
    }


def segment_to_dict(segment: TrajectorySegment) -> dict[str, Any]:
    return {
        "start": pose_to_dict(segment.start),
        "end": pose_to_dict(segment.end),
        "cost": segment.cost,
        "duration_s": segment.duration_s,
        "action_name": segment.action_name,
        "sample_count": len(segment.samples),
        "samples": [pose_to_dict(sample) for sample in segment.samples],
    }


def method_result_to_dict(result: MethodRunResult, *, include_path: bool = False) -> dict[str, Any]:
    cache_stats = result.cache_stats or {}
    payload = {
        "method_name": result.method_name,
        "success": result.success,
        "message": result.message,
        "path_length": result.path_length,
        "traversal_time": result.traversal_time,
        "query_time_sec": result.query_time_sec,
        "build_time_sec": result.build_time_sec,
        "annotation_time_sec": result.annotation_time_sec,
        "expanded_labels": result.expanded_labels,
        "expanded_nodes": result.expanded_nodes,
        "rejected_dynamic_edges": result.rejected_dynamic_edges,
        "replans": result.replans,
        "dynamic_collision_failures": result.dynamic_collision_failures,
        "cache_stats": cache_stats,
        "cache_hits_free": cache_stats.get("temporal_cache_hits_free", 0),
        "cache_hits_blocked": cache_stats.get("temporal_cache_hits_blocked", 0),
        "interval_hits_free": cache_stats.get("interval_cache_hits_free", 0),
        "interval_hits_blocked": cache_stats.get("interval_cache_hits_blocked", 0),
        "interaction_cache_hits": cache_stats.get("interaction_cache_hits", 0),
        "roadmap_nodes": result.roadmap_nodes,
        "roadmap_edges": result.roadmap_edges,
        "path_segment_count": len(result.path_segments),
    }
    if include_path:
        payload["path_segments"] = [segment_to_dict(segment) for segment in result.path_segments]
    return payload


def flatten_for_csv(row: dict[str, Any]) -> dict[str, Any]:
    flat: dict[str, Any] = {}
    for key, value in row.items():
        if isinstance(value, (str, int, float, bool)) or value is None:
            flat[key] = value
        else:
            flat[key] = json.dumps(to_jsonable(value), sort_keys=True)
    return flat


def to_jsonable(value: Any) -> Any:
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, Pose2D):
        return pose_to_dict(value)
    if isinstance(value, DynamicCircleObstacle):
        return obstacle_to_dict(value)
    if isinstance(value, TrajectorySegment):
        return segment_to_dict(value)
    if is_dataclass(value):
        return to_jsonable(asdict(value))
    if isinstance(value, dict):
        return {str(key): to_jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [to_jsonable(item) for item in value]
    return value


def write_json(path: Path, payload: Any) -> None:
    ensure_dir(path.parent)
    path.write_text(json.dumps(to_jsonable(payload), indent=2, sort_keys=True) + "\n", encoding="utf-8")


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    ensure_dir(path.parent)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    flat_rows = [flatten_for_csv(row) for row in rows]
    fieldnames = sorted({key for row in flat_rows for key in row})
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(flat_rows)
