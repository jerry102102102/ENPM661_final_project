"""Reactive nonholonomic replanning baseline built on static A*."""

from __future__ import annotations

from dataclasses import dataclass
import math
import time

from src.configs.defaults import CollisionParams, PlannerConfig, VehicleParams
from src.core.search_utils import goal_satisfied
from src.core.temporal_validation import temporal_collision_free
from src.models.obstacles import DynamicCircleObstacle, StaticWorld
from src.models.state import Pose2D, TrajectorySegment
from src.planners.baseline_nonholonomic_astar import plan as static_astar_plan


@dataclass(frozen=True)
class ReactiveReplanningConfig:
    """Configuration for the reactive replanning baseline."""

    replan_period_s: float = 1.0
    lookahead_time_s: float = 3.0
    max_replans: int = 50
    max_total_time_s: float = 120.0
    collision_buffer: float = 0.0
    execution_step_dt_s: float = 0.1
    replan_on_predicted_invalidity: bool = True


@dataclass(frozen=True)
class ReactiveReplanningResult:
    """Result from repeated static A* replanning in a dynamic scene."""

    success: bool
    message: str
    runtime_sec: float
    query_time_total: float
    number_of_replans: int
    dynamic_collision_failures: int
    total_traversal_time: float
    path_cost: float
    path_segments: list[TrajectorySegment]
    final_pose: Pose2D | None
    expanded_nodes_total: int
    planner_log: dict[str, object]

    @property
    def path_samples(self) -> list[Pose2D]:
        samples: list[Pose2D] = []
        for segment in self.path_segments:
            if samples:
                samples.extend(segment.samples[1:])
            else:
                samples.extend(segment.samples)
        return samples


class ReactiveReplanningBaseline:
    """Reactive baseline that repeatedly invokes static nonholonomic A*.

    The planner does not reason globally about future obstacle timing. It plans
    a static path, checks the next primitive segment against dynamic obstacles
    at execution time, and replans or waits when the segment is blocked.
    """

    def __init__(self, config: ReactiveReplanningConfig | None = None) -> None:
        self.config = config or ReactiveReplanningConfig()

    def _segment_is_temporally_valid(
        self,
        segment: TrajectorySegment,
        departure_time_s: float,
        dynamic_obstacles: list[DynamicCircleObstacle],
        *,
        clearance: float,
        collision_params: CollisionParams,
        static_world: StaticWorld,
    ) -> bool:
        return temporal_collision_free(
            segment,
            departure_time_s,
            static_world,
            dynamic_obstacles,
            collision_params,
            clearance=clearance + self.config.collision_buffer,
            timestamp_mode="step",
            timestamp_dt_s=self.config.execution_step_dt_s,
        )

    def _remaining_path_has_predicted_conflict(
        self,
        segments: list[TrajectorySegment],
        first_index: int,
        current_time_s: float,
        dynamic_obstacles: list[DynamicCircleObstacle],
        *,
        clearance: float,
        collision_params: CollisionParams,
        static_world: StaticWorld,
    ) -> bool:
        if not self.config.replan_on_predicted_invalidity or self.config.lookahead_time_s <= 0.0:
            return False
        predicted_time = current_time_s
        lookahead_end = current_time_s + self.config.lookahead_time_s
        for segment in segments[first_index:]:
            if predicted_time > lookahead_end:
                break
            if not self._segment_is_temporally_valid(
                segment,
                predicted_time,
                dynamic_obstacles,
                clearance=clearance,
                collision_params=collision_params,
                static_world=static_world,
            ):
                return True
            predicted_time += segment.duration_s
        return False

    def plan(
        self,
        start_pose: Pose2D,
        goal_xy: tuple[float, float],
        dynamic_obstacles: list[DynamicCircleObstacle],
        *,
        rpm1: float,
        rpm2: float,
        clearance: float,
        vehicle_params: VehicleParams,
        collision_params: CollisionParams,
        static_world: StaticWorld,
        planner_config: PlannerConfig,
        start_time_s: float = 0.0,
    ) -> ReactiveReplanningResult:
        """Run reactive replanning from start to goal in a dynamic scene."""

        wall_start = time.perf_counter()
        current_pose = start_pose
        current_time_s = start_time_s
        query_time_total = 0.0
        expanded_nodes_total = 0
        number_of_replans = 0
        dynamic_collision_failures = 0
        path_cost = 0.0
        path_segments: list[TrajectorySegment] = []
        plan_attempts: list[dict[str, object]] = []
        scheduled_replans = 0
        predicted_dynamic_conflicts = 0

        while current_time_s - start_time_s <= self.config.max_total_time_s:
            if goal_satisfied(
                current_pose,
                goal_xy,
                planner_config.goal_tolerance_m,
                planner_config.goal_heading_rad,
                planner_config.goal_heading_tolerance_rad,
            ):
                return ReactiveReplanningResult(
                    success=True,
                    message="Reactive replanning reached the goal.",
                    runtime_sec=time.perf_counter() - wall_start,
                    query_time_total=query_time_total,
                    number_of_replans=number_of_replans,
                    dynamic_collision_failures=dynamic_collision_failures,
                    total_traversal_time=current_time_s - start_time_s,
                    path_cost=path_cost,
                    path_segments=path_segments,
                    final_pose=current_pose,
                    expanded_nodes_total=expanded_nodes_total,
                    planner_log={
                        "planner": "reactive_replanning",
                        "attempts": plan_attempts,
                        "scheduled_replans": scheduled_replans,
                        "predicted_dynamic_conflicts": predicted_dynamic_conflicts,
                    },
                )

            static_result = static_astar_plan(
                start_pose=current_pose,
                goal_xy=goal_xy,
                rpm1=rpm1,
                rpm2=rpm2,
                clearance=clearance,
                vehicle_params=vehicle_params,
                collision_params=collision_params,
                static_world=static_world,
                config=planner_config,
            )
            query_time_total += static_result.runtime_sec
            expanded_nodes_total += static_result.expanded_nodes
            plan_attempts.append(
                {
                    "time_s": current_time_s,
                    "success": static_result.success,
                    "expanded_nodes": static_result.expanded_nodes,
                    "path_segment_count": len(static_result.path_segments),
                }
            )
            if not static_result.success:
                return ReactiveReplanningResult(
                    success=False,
                    message=f"Static A* failed during reactive replanning: {static_result.message}",
                    runtime_sec=time.perf_counter() - wall_start,
                    query_time_total=query_time_total,
                    number_of_replans=number_of_replans,
                    dynamic_collision_failures=dynamic_collision_failures,
                    total_traversal_time=current_time_s - start_time_s,
                    path_cost=math.inf,
                    path_segments=path_segments,
                    final_pose=current_pose,
                    expanded_nodes_total=expanded_nodes_total,
                    planner_log={
                        "planner": "reactive_replanning",
                        "attempts": plan_attempts,
                        "scheduled_replans": scheduled_replans,
                        "predicted_dynamic_conflicts": predicted_dynamic_conflicts,
                    },
                )

            progressed = False
            blocked = False
            for segment_index, segment in enumerate(static_result.path_segments):
                if current_time_s - start_time_s + segment.duration_s > self.config.max_total_time_s:
                    return ReactiveReplanningResult(
                        success=False,
                        message=f"No path found before max_total_time_s={self.config.max_total_time_s}.",
                        runtime_sec=time.perf_counter() - wall_start,
                        query_time_total=query_time_total,
                        number_of_replans=number_of_replans,
                        dynamic_collision_failures=dynamic_collision_failures,
                        total_traversal_time=current_time_s - start_time_s,
                        path_cost=math.inf,
                        path_segments=path_segments,
                        final_pose=current_pose,
                        expanded_nodes_total=expanded_nodes_total,
                        planner_log={
                            "planner": "reactive_replanning",
                            "attempts": plan_attempts,
                            "scheduled_replans": scheduled_replans,
                            "predicted_dynamic_conflicts": predicted_dynamic_conflicts,
                        },
                    )
                if not self._segment_is_temporally_valid(
                    segment,
                    current_time_s,
                    dynamic_obstacles,
                    clearance=clearance,
                    collision_params=collision_params,
                    static_world=static_world,
                ):
                    dynamic_collision_failures += 1
                    blocked = True
                    break

                path_segments.append(segment)
                path_cost += segment.cost
                current_time_s += segment.duration_s
                current_pose = segment.end
                progressed = True
                if goal_satisfied(
                    current_pose,
                    goal_xy,
                    planner_config.goal_tolerance_m,
                    planner_config.goal_heading_rad,
                    planner_config.goal_heading_tolerance_rad,
                ):
                    break
                if dynamic_obstacles and self._remaining_path_has_predicted_conflict(
                    static_result.path_segments,
                    segment_index + 1,
                    current_time_s,
                    dynamic_obstacles,
                    clearance=clearance,
                    collision_params=collision_params,
                    static_world=static_world,
                ):
                    predicted_dynamic_conflicts += 1
                    blocked = True
                    break
                if dynamic_obstacles and current_time_s - float(plan_attempts[-1]["time_s"]) >= self.config.replan_period_s:
                    scheduled_replans += 1
                    blocked = True
                    break

            if not blocked:
                continue

            number_of_replans += 1
            if number_of_replans > self.config.max_replans:
                return ReactiveReplanningResult(
                    success=False,
                    message=f"No path found before max_replans={self.config.max_replans}.",
                    runtime_sec=time.perf_counter() - wall_start,
                    query_time_total=query_time_total,
                    number_of_replans=number_of_replans,
                    dynamic_collision_failures=dynamic_collision_failures,
                    total_traversal_time=current_time_s - start_time_s,
                    path_cost=math.inf,
                    path_segments=path_segments,
                    final_pose=current_pose,
                    expanded_nodes_total=expanded_nodes_total,
                    planner_log={
                        "planner": "reactive_replanning",
                        "attempts": plan_attempts,
                        "scheduled_replans": scheduled_replans,
                        "predicted_dynamic_conflicts": predicted_dynamic_conflicts,
                    },
                )

            if not progressed:
                current_time_s += self.config.replan_period_s

        return ReactiveReplanningResult(
            success=False,
            message=f"No path found before max_total_time_s={self.config.max_total_time_s}.",
            runtime_sec=time.perf_counter() - wall_start,
            query_time_total=query_time_total,
            number_of_replans=number_of_replans,
            dynamic_collision_failures=dynamic_collision_failures,
            total_traversal_time=current_time_s - start_time_s,
            path_cost=math.inf,
            path_segments=path_segments,
            final_pose=current_pose,
            expanded_nodes_total=expanded_nodes_total,
            planner_log={
                "planner": "reactive_replanning",
                "attempts": plan_attempts,
                "scheduled_replans": scheduled_replans,
                "predicted_dynamic_conflicts": predicted_dynamic_conflicts,
            },
        )
