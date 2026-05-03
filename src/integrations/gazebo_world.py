"""Import simple Gazebo JSON worlds into the ACTEA planner representation."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any

from src.configs.defaults import CollisionParams, PlannerConfig, VehicleParams
from src.experiments.method_registry import MethodRunConfig
from src.experiments.scenarios import HEADINGS_8
from src.models.obstacles import DynamicCircleObstacle, StaticWorld
from src.models.state import OrientedBox, Pose2D
from src.planners.reactive_replanning_baseline import ReactiveReplanningConfig


@dataclass(frozen=True)
class GazeboWorldImport:
    """Planner-ready representation of a simple Gazebo world config."""

    world_name: str
    static_world: StaticWorld
    dynamic_obstacles: list[DynamicCircleObstacle]
    planner_config: PlannerConfig
    origin_offset_xy: tuple[float, float]
    annotation_horizon_s: float

    def transform_pose(self, pose: Pose2D) -> Pose2D:
        """Shift a Gazebo-frame pose into the planner's positive world frame."""

        return Pose2D(
            pose.x + self.origin_offset_xy[0],
            pose.y + self.origin_offset_xy[1],
            pose.theta,
        )

    def inverse_transform_pose(self, pose: Pose2D) -> Pose2D:
        """Shift a planner-frame pose back into the original Gazebo frame."""

        return Pose2D(
            pose.x - self.origin_offset_xy[0],
            pose.y - self.origin_offset_xy[1],
            pose.theta,
        )


def _entity_xy_bounds(config: dict[str, Any]) -> tuple[float, float, float, float]:
    xs: list[float] = []
    ys: list[float] = []
    for entity in config.get("static_entities", []):
        x = float(entity["position"]["x"])
        y = float(entity["position"]["y"])
        half_x = float(entity["scale"]["length"]) / 2.0
        half_y = float(entity["scale"]["width"]) / 2.0
        xs.extend([x - half_x, x + half_x])
        ys.extend([y - half_y, y + half_y])
    for entity in config.get("dynamic_entities", []):
        radius = float(entity["radius"])
        for waypoint in entity.get("waypoints", []):
            x = float(waypoint["pos"]["x"])
            y = float(waypoint["pos"]["y"])
            xs.extend([x - radius, x + radius])
            ys.extend([y - radius, y + radius])
    if not xs or not ys:
        return (0.0, 1.0, 0.0, 1.0)
    return (min(xs), max(xs), min(ys), max(ys))


def _transform_xy(x: float, y: float, offset: tuple[float, float]) -> tuple[float, float]:
    return (x + offset[0], y + offset[1])


def _static_entities_to_boxes(config: dict[str, Any], offset: tuple[float, float]) -> list[OrientedBox]:
    boxes: list[OrientedBox] = []
    for entity in config.get("static_entities", []):
        center_x, center_y = _transform_xy(float(entity["position"]["x"]), float(entity["position"]["y"]), offset)
        boxes.append(
            OrientedBox(
                name=str(entity["name"]),
                center_x=center_x,
                center_y=center_y,
                width=float(entity["scale"]["length"]),
                height=float(entity["scale"]["width"]),
                angle_rad=math.radians(float(entity.get("rotation", 0.0))),
            )
        )
    return boxes


def _segment_obstacle(
    *,
    label: str,
    radius: float,
    start_time_s: float,
    end_time_s: float,
    start_xy: tuple[float, float],
    end_xy: tuple[float, float],
) -> DynamicCircleObstacle | None:
    dt = end_time_s - start_time_s
    if dt <= 1e-9:
        return None
    vx = (end_xy[0] - start_xy[0]) / dt
    vy = (end_xy[1] - start_xy[1]) / dt
    # DynamicCircleObstacle uses an absolute-time affine equation:
    # p(t) = initial + velocity * t.  For a segment active over [t0, t1],
    # the intercept must be back-projected to t = 0.
    return DynamicCircleObstacle(
        initial_x=start_xy[0] - vx * start_time_s,
        initial_y=start_xy[1] - vy * start_time_s,
        velocity_x=vx,
        velocity_y=vy,
        radius=radius,
        label=label,
        active_start_time_s=start_time_s,
        active_end_time_s=end_time_s,
    )


def expand_periodic_dynamic_entities(
    config: dict[str, Any],
    *,
    offset: tuple[float, float],
    horizon_s: float,
) -> list[DynamicCircleObstacle]:
    """Expand Gazebo actor waypoints into time-gated constant-velocity circles.

    Gazebo actors in the classmate map use looping waypoint scripts.  ACTEA
    supports constant-velocity circular obstacles, so each waypoint segment is
    unfolded over the finite planning horizon as an active-time-bounded segment.
    """

    obstacles: list[DynamicCircleObstacle] = []
    for entity in config.get("dynamic_entities", []):
        waypoints = sorted(entity.get("waypoints", []), key=lambda item: float(item["time"]))
        if len(waypoints) < 2:
            continue
        period_s = float(waypoints[-1]["time"])
        if period_s <= 0.0:
            continue
        radius = float(entity["radius"])
        repeat_index = 0
        while repeat_index * period_s < horizon_s - 1e-9:
            repeat_offset = repeat_index * period_s
            for segment_index, (a, b) in enumerate(zip(waypoints[:-1], waypoints[1:])):
                start_time = repeat_offset + float(a["time"])
                end_time = repeat_offset + float(b["time"])
                if start_time >= horizon_s:
                    continue
                clipped_end_time = min(end_time, horizon_s)
                start_xy = _transform_xy(float(a["pos"]["x"]), float(a["pos"]["y"]), offset)
                end_xy_raw = _transform_xy(float(b["pos"]["x"]), float(b["pos"]["y"]), offset)
                if clipped_end_time < end_time - 1e-9:
                    ratio = (clipped_end_time - start_time) / (end_time - start_time)
                    end_xy = (
                        start_xy[0] + (end_xy_raw[0] - start_xy[0]) * ratio,
                        start_xy[1] + (end_xy_raw[1] - start_xy[1]) * ratio,
                    )
                else:
                    end_xy = end_xy_raw
                obstacle = _segment_obstacle(
                    label=f"{entity['name']}_period{repeat_index}_segment{segment_index}",
                    radius=radius,
                    start_time_s=start_time,
                    end_time_s=clipped_end_time,
                    start_xy=start_xy,
                    end_xy=end_xy,
                )
                if obstacle is not None:
                    obstacles.append(obstacle)
            repeat_index += 1
    return obstacles


def import_gazebo_world_config(
    config_path: str | Path,
    *,
    annotation_horizon_s: float = 20.0,
    margin_m: float = 0.75,
    xy_resolution_m: float = 0.10,
) -> GazeboWorldImport:
    """Load a Gazebo JSON world and convert it into planner-ready objects."""

    path = Path(config_path)
    with path.open("r", encoding="utf-8") as handle:
        config = json.load(handle)

    min_x, max_x, min_y, max_y = _entity_xy_bounds(config)
    offset = (-min_x + margin_m, -min_y + margin_m)
    width = (max_x - min_x) + 2.0 * margin_m
    height = (max_y - min_y) + 2.0 * margin_m
    planner_config = PlannerConfig(
        world_width_m=width,
        world_height_m=height,
        xy_resolution_m=xy_resolution_m,
        theta_bins=64,
        action_duration_s=1.0,
        integration_dt_s=0.1,
        goal_tolerance_m=0.35,
        goal_heading_tolerance_rad=math.radians(35.0),
        max_iterations=30000,
    )
    static_world = StaticWorld(
        bounds=(0.0, width, 0.0, height),
        obstacles=_static_entities_to_boxes(config, offset),
    )
    return GazeboWorldImport(
        world_name=str(config.get("world_name", path.stem)),
        static_world=static_world,
        dynamic_obstacles=expand_periodic_dynamic_entities(config, offset=offset, horizon_s=annotation_horizon_s),
        planner_config=planner_config,
        origin_offset_xy=offset,
        annotation_horizon_s=annotation_horizon_s,
    )


def method_run_config_from_gazebo_import(
    imported: GazeboWorldImport,
    *,
    xy_sample_count: int = 500,
    grid_spacing_m: float = 0.45,
) -> MethodRunConfig:
    """Build the shared experiment/planner config for an imported Gazebo world."""

    return MethodRunConfig(
        static_world=imported.static_world,
        vehicle_params=VehicleParams(0.1, 0.4, 0.4, "differential_drive"),
        collision_params=CollisionParams(radius_m=0.08, source="gazebo_import"),
        planner_config=imported.planner_config,
        xy_sample_count=xy_sample_count,
        headings_rad=HEADINGS_8,
        sampling_mode="grid",
        grid_spacing_m=grid_spacing_m,
        position_tolerance_m=max(0.30, grid_spacing_m * 0.75),
        heading_tolerance_rad=math.radians(35.0),
        max_outgoing_edges_per_node=8,
        temporal_max_arrival_time_s=imported.annotation_horizon_s,
        temporal_time_bin_size_s=0.25,
        temporal_goal_tolerance_m=0.35,
        temporal_goal_heading_tolerance_rad=math.radians(35.0),
        temporal_connection_position_tolerance_m=max(0.30, grid_spacing_m * 0.75),
        temporal_connection_heading_tolerance_rad=math.radians(35.0),
        temporal_annotation_end_time_s=imported.annotation_horizon_s,
        reactive_config=ReactiveReplanningConfig(
            replan_period_s=1.0,
            lookahead_time_s=4.0,
            max_replans=60,
            max_total_time_s=imported.annotation_horizon_s,
            execution_step_dt_s=0.1,
            replan_on_predicted_invalidity=True,
        ),
    )
