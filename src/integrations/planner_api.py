"""Controller-facing function API for the ACTEA temporal roadmap planner."""

from __future__ import annotations

from dataclasses import dataclass

from src.core.time_parameterization import annotate_trajectory_samples
from src.experiments.method_registry import MethodRunConfig, build_sampled_temporal_planner
from src.models.obstacles import DynamicCircleObstacle
from src.models.state import Pose2D, TimedPose2D, TrajectorySegment
from src.planners.temporal_roadmap_planner import TemporalRoadmapPlanner


@dataclass(frozen=True)
class ControllerCommandSegment:
    """One primitive command segment that a closed-loop controller can track."""

    duration_s: float
    action_name: str | None
    command: tuple[float, float] | None
    rpm_l: float | None
    rpm_r: float | None
    steering_angle_deg: float | None
    drive_rpm: float | None


@dataclass(frozen=True)
class ControllerRoute:
    """Planner output packaged for a closed-loop trajectory follower."""

    success: bool
    message: str
    waypoints: list[Pose2D]
    timed_waypoints: list[TimedPose2D]
    segments: list[TrajectorySegment]
    commands: list[ControllerCommandSegment]
    traversal_time_s: float | None
    path_cost: float | None
    expanded_labels: int
    rejected_dynamic_edges: int


@dataclass
class ControllerPlanningFunction:
    """Reusable map/planner object with a simple function-like ``plan`` method."""

    planner: TemporalRoadmapPlanner
    dynamic_obstacles: list[DynamicCircleObstacle]
    config: MethodRunConfig
    build_time_s: float
    annotation_time_s: float

    def plan(self, start: Pose2D, goal: Pose2D, *, start_time_s: float = 0.0) -> ControllerRoute:
        """Plan one start-goal query and return controller-trackable route data."""

        result = self.planner.plan(
            start,
            goal,
            self.dynamic_obstacles,
            start_time_s=start_time_s,
            clearance=self.config.clearance,
        )
        if not result.path:
            return ControllerRoute(
                success=False,
                message=result.message,
                waypoints=[],
                timed_waypoints=[],
                segments=[],
                commands=[],
                traversal_time_s=None,
                path_cost=None,
                expanded_labels=result.expanded_labels,
                rejected_dynamic_edges=result.rejected_dynamic_edges,
            )

        waypoints: list[Pose2D] = []
        timed_waypoints: list[TimedPose2D] = []
        current_time = start_time_s
        for segment_index, segment in enumerate(result.path.segments):
            samples = segment.samples if segment_index == 0 else segment.samples[1:]
            waypoints.extend(samples)
            timed = annotate_trajectory_samples(segment, current_time)
            timed_waypoints.extend(timed if segment_index == 0 else timed[1:])
            current_time += segment.duration_s

        commands = [
            ControllerCommandSegment(
                duration_s=segment.duration_s,
                action_name=segment.action_name,
                command=segment.command,
                rpm_l=segment.rpm_l,
                rpm_r=segment.rpm_r,
                steering_angle_deg=segment.steering_angle_deg,
                drive_rpm=segment.drive_rpm,
            )
            for segment in result.path.segments
        ]
        return ControllerRoute(
            success=result.success,
            message=result.message,
            waypoints=waypoints,
            timed_waypoints=timed_waypoints,
            segments=result.path.segments,
            commands=commands,
            traversal_time_s=result.path.total_traversal_time,
            path_cost=result.path.total_cost,
            expanded_labels=result.expanded_labels,
            rejected_dynamic_edges=result.rejected_dynamic_edges,
        )


def build_controller_planning_function(
    dynamic_obstacles: list[DynamicCircleObstacle],
    config: MethodRunConfig,
    *,
    mode: str = "actea",
) -> ControllerPlanningFunction:
    """Build the reusable roadmap and return a controller-facing planning function."""

    planner, build_time, annotation_time = build_sampled_temporal_planner(
        mode,
        Pose2D(0.0, 0.0, 0.0),
        dynamic_obstacles,
        config,
    )
    return ControllerPlanningFunction(
        planner=planner,
        dynamic_obstacles=dynamic_obstacles,
        config=config,
        build_time_s=build_time,
        annotation_time_s=annotation_time,
    )

