"""Static nonholonomic A* baseline preserved from Part 1 behavior."""

from __future__ import annotations

from dataclasses import dataclass
import heapq
import math
import time
from typing import Any

from src.configs.defaults import CollisionParams, PlannerConfig, VehicleParams
from src.core.discretization import discretize_pose
from src.core.heuristics import euclidean_goal_heuristic
from src.core.rollout import simulate_primitive
from src.core.search_utils import goal_satisfied
from src.core.static_collision import is_trajectory_static_valid
from src.models.obstacles import StaticWorld
from src.models.primitives import get_action_set
from src.models.state import Pose2D, TrajectorySegment


@dataclass(frozen=True)
class SearchNode:
    """A static A* search node."""

    pose: Pose2D
    parent_key: tuple[int, int, int] | None
    segment_from_parent: TrajectorySegment | None
    action: tuple[float, float] | None
    cost_to_come: float


@dataclass(frozen=True)
class PlannerResult:
    """Result from the static nonholonomic A* baseline."""

    success: bool
    message: str
    runtime_sec: float
    expanded_nodes: int
    path_cost: float
    path_segments: list[TrajectorySegment]
    explored_segments: list[TrajectorySegment]
    final_pose: Pose2D | None
    planner_log: dict[str, Any]

    @property
    def path_samples(self) -> list[Pose2D]:
        samples: list[Pose2D] = []
        for segment in self.path_segments:
            if samples:
                samples.extend(segment.samples[1:])
            else:
                samples.extend(segment.samples)
        return samples


def _reconstruct_path(
    goal_key: tuple[int, int, int],
    node_store: dict[tuple[int, int, int], SearchNode],
) -> list[TrajectorySegment]:
    segments: list[TrajectorySegment] = []
    key: tuple[int, int, int] | None = goal_key
    while key is not None:
        node = node_store[key]
        if node.segment_from_parent is not None:
            segments.append(node.segment_from_parent)
        key = node.parent_key
    segments.reverse()
    return segments


def _build_planner_log(
    goal_key: tuple[int, int, int] | None,
    node_store: dict[tuple[int, int, int], SearchNode],
) -> dict[str, Any]:
    path_nodes: list[dict[str, Any]] = []
    if goal_key is not None:
        key: tuple[int, int, int] | None = goal_key
        reversed_nodes: list[SearchNode] = []
        while key is not None:
            node = node_store[key]
            reversed_nodes.append(node)
            key = node.parent_key
        for node in reversed(reversed_nodes):
            path_nodes.append(
                {
                    "pose": {
                        "x": node.pose.x,
                        "y": node.pose.y,
                        "theta_rad": node.pose.theta,
                        "theta_deg": math.degrees(node.pose.theta),
                    },
                    "cost_to_come": node.cost_to_come,
                    "action": list(node.action) if node.action is not None else None,
                    "action_name": node.segment_from_parent.action_name if node.segment_from_parent else None,
                }
            )
    return {"planner": "baseline_nonholonomic_astar", "path_nodes": path_nodes}


def plan(
    start_pose: Pose2D,
    goal_xy: tuple[float, float],
    rpm1: float,
    rpm2: float,
    clearance: float,
    vehicle_params: VehicleParams,
    collision_params: CollisionParams,
    static_world: StaticWorld,
    config: PlannerConfig,
) -> PlannerResult:
    """Run static nonholonomic A* over discretized (x, y, theta)."""

    start_time = time.perf_counter()
    start_key = discretize_pose(start_pose, config.xy_resolution_m, config.theta_bins)
    start_node = SearchNode(
        pose=start_pose,
        parent_key=None,
        segment_from_parent=None,
        action=None,
        cost_to_come=0.0,
    )

    node_store: dict[tuple[int, int, int], SearchNode] = {start_key: start_node}
    best_cost: dict[tuple[int, int, int], float] = {start_key: 0.0}
    closed: set[tuple[int, int, int]] = set()
    explored_segments: list[TrajectorySegment] = []
    open_heap: list[tuple[float, float, int, tuple[int, int, int]]] = []
    tie_breaker = 0

    start_h = euclidean_goal_heuristic(start_pose, goal_xy)
    heapq.heappush(open_heap, (start_h, start_h, tie_breaker, start_key))

    if goal_satisfied(
        start_pose,
        goal_xy,
        config.goal_tolerance_m,
        config.goal_heading_rad,
        config.goal_heading_tolerance_rad,
    ):
        return PlannerResult(
            success=True,
            message="Start is already inside goal pose tolerance.",
            runtime_sec=time.perf_counter() - start_time,
            expanded_nodes=0,
            path_cost=0.0,
            path_segments=[],
            explored_segments=[],
            final_pose=start_pose,
            planner_log=_build_planner_log(start_key, node_store),
        )

    actions = get_action_set(rpm1, rpm2, vehicle_params)
    expanded_nodes = 0

    while open_heap and expanded_nodes < config.max_iterations:
        _, _, _, current_key = heapq.heappop(open_heap)
        if current_key in closed:
            continue

        current_node = node_store[current_key]
        closed.add(current_key)
        expanded_nodes += 1

        if goal_satisfied(
            current_node.pose,
            goal_xy,
            config.goal_tolerance_m,
            config.goal_heading_rad,
            config.goal_heading_tolerance_rad,
        ):
            path_segments = _reconstruct_path(current_key, node_store)
            return PlannerResult(
                success=True,
                message="Path found.",
                runtime_sec=time.perf_counter() - start_time,
                expanded_nodes=expanded_nodes,
                path_cost=current_node.cost_to_come,
                path_segments=path_segments,
                explored_segments=explored_segments,
                final_pose=current_node.pose,
                planner_log=_build_planner_log(current_key, node_store),
            )

        for primitive in actions:
            segment = simulate_primitive(
                start_pose=current_node.pose,
                primitive=primitive,
                vehicle_params=vehicle_params,
                action_duration_s=config.action_duration_s,
                integration_dt_s=config.integration_dt_s,
            )
            if segment.cost <= 1e-12:
                continue
            if not is_trajectory_static_valid(segment, collision_params, clearance, static_world):
                continue

            explored_segments.append(segment)
            child_key = discretize_pose(segment.end, config.xy_resolution_m, config.theta_bins)
            if child_key in closed:
                continue

            new_cost = current_node.cost_to_come + segment.cost
            if new_cost >= best_cost.get(child_key, math.inf):
                continue

            best_cost[child_key] = new_cost
            node_store[child_key] = SearchNode(
                pose=segment.end,
                parent_key=current_key,
                segment_from_parent=segment,
                action=(primitive.command_a, primitive.command_b),
                cost_to_come=new_cost,
            )
            heuristic = euclidean_goal_heuristic(segment.end, goal_xy)
            tie_breaker += 1
            heapq.heappush(open_heap, (new_cost + heuristic, heuristic, tie_breaker, child_key))

    if expanded_nodes >= config.max_iterations:
        message = f"No path found before max_iterations={config.max_iterations}."
    else:
        message = "No path found; open list exhausted."

    return PlannerResult(
        success=False,
        message=message,
        runtime_sec=time.perf_counter() - start_time,
        expanded_nodes=expanded_nodes,
        path_cost=math.inf,
        path_segments=[],
        explored_segments=explored_segments,
        final_pose=None,
        planner_log=_build_planner_log(None, node_store),
    )
