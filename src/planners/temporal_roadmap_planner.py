"""First working nonholonomic temporal roadmap query planner."""

from __future__ import annotations

from dataclasses import dataclass, field
import heapq
import math

from src.configs.defaults import CollisionParams, PlannerConfig, VehicleParams
from src.core.angles import normalize_angle
from src.core.costs import euclidean_distance_xy
from src.core.rollout import simulate_primitive
from src.core.static_collision import is_pose_static_valid, is_trajectory_static_valid
from src.core.temporal_cache import CachedTemporalValidator, TemporalValidationStats
from src.core.temporal_validation import temporal_collision_free
from src.models.labels import TemporalSearchLabel
from src.models.obstacles import DynamicCircleObstacle, StaticWorld
from src.models.primitives import MotionPrimitive
from src.models.roadmap import Roadmap, RoadmapEdge, RoadmapNode
from src.models.state import Pose2D, TrajectorySegment


@dataclass(frozen=True)
class TemporalRoadmapPlannerConfig:
    """Configuration for the first temporal roadmap query search."""

    max_arrival_time_s: float = 60.0
    time_bin_size_s: float = 0.5
    goal_tolerance_m: float = 0.20
    goal_heading_tolerance_rad: float = math.radians(25.0)
    connection_position_tolerance_m: float = 0.20
    connection_heading_tolerance_rad: float = math.radians(35.0)
    timestamp_mode: str = "normalized"
    max_expanded_labels: int = 20000
    use_temporal_cache: bool = False
    use_temporal_intervals: bool = False
    cache_time_slack_s: float = 0.0


@dataclass(frozen=True)
class TemporalRoadmapPath:
    """Reconstructed temporal path."""

    node_ids: list[int]
    poses: list[Pose2D]
    edge_ids: list[int]
    actions: list[str | None]
    arrival_times: list[float]
    segments: list[TrajectorySegment]
    total_cost: float
    total_traversal_time: float


@dataclass(frozen=True)
class TemporalRoadmapResult:
    """Planner result for a temporal roadmap query."""

    success: bool
    message: str
    path: TemporalRoadmapPath | None
    expanded_labels: int
    generated_labels: int
    rejected_dynamic_edges: int
    pruned_labels: int
    roadmap_node_count: int
    roadmap_edge_count: int
    start_node_id: int | None = None
    goal_node_id: int | None = None
    debug: dict[str, object] = field(default_factory=dict)


class TemporalRoadmapPlanner:
    """Temporal A* over static-valid nonholonomic roadmap edges."""

    def __init__(
        self,
        roadmap: Roadmap,
        static_world: StaticWorld,
        collision: CollisionParams,
        vehicle_params: VehicleParams,
        primitives: list[MotionPrimitive],
        planner_config: PlannerConfig,
        config: TemporalRoadmapPlannerConfig | None = None,
    ) -> None:
        self.roadmap = roadmap
        self.static_world = static_world
        self.collision = collision
        self.vehicle_params = vehicle_params
        self.primitives = primitives
        self.planner_config = planner_config
        self.config = config or TemporalRoadmapPlannerConfig()
        self.cached_validator: CachedTemporalValidator | None = None

    def _make_cached_validator(self, clearance: float) -> CachedTemporalValidator:
        if (
            self.cached_validator is None
            or self.cached_validator.clearance != clearance
            or self.cached_validator.time_bin_size_s != self.config.time_bin_size_s
            or self.cached_validator.timestamp_mode != self.config.timestamp_mode
        ):
            self.cached_validator = CachedTemporalValidator(
                static_world=self.static_world,
                collision=self.collision,
                clearance=clearance,
                time_bin_size_s=self.config.time_bin_size_s,
                max_time_slack_s=self.config.cache_time_slack_s,
                timestamp_mode=self.config.timestamp_mode,
                use_interval_lookup=self.config.use_temporal_intervals,
            )
        self.cached_validator.use_interval_lookup = self.config.use_temporal_intervals
        return self.cached_validator

    def annotate_temporal_intervals(
        self,
        dynamic_obstacles: list[DynamicCircleObstacle],
        *,
        start_time_s: float = 0.0,
        end_time_s: float | None = None,
        step_s: float | None = None,
        clearance: float = 0.0,
        max_edges: int | None = None,
    ) -> dict[int, object]:
        """Precompute sampled temporal validity intervals for roadmap edges."""

        validator = self._make_cached_validator(clearance)
        interval_end = self.config.max_arrival_time_s if end_time_s is None else end_time_s
        edge_items = list(self.roadmap.edges.items())
        if max_edges is not None:
            edge_items = edge_items[:max_edges]
        return {
            edge_id: validator.annotate_edge_intervals(
                edge,
                dynamic_obstacles,
                start_time_s=start_time_s,
                end_time_s=interval_end,
                step_s=step_s or self.config.time_bin_size_s,
            )
            for edge_id, edge in edge_items
        }

    def annotate_edge_obstacle_interactions(
        self,
        dynamic_obstacles: list[DynamicCircleObstacle],
        *,
        start_time_s: float = 0.0,
        end_time_s: float | None = None,
        clearance: float = 0.0,
        max_edges: int | None = None,
    ) -> dict[int, tuple[int, ...]]:
        """Precompute edge-to-obstacle candidate interaction lists."""

        validator = self._make_cached_validator(clearance)
        interaction_end = self.config.max_arrival_time_s if end_time_s is None else end_time_s
        edge_items = list(self.roadmap.edges.items())
        if max_edges is not None:
            edge_items = edge_items[:max_edges]
        return {
            edge_id: validator.annotate_edge_obstacle_candidates(
                edge,
                dynamic_obstacles,
                start_time_s=start_time_s,
                end_time_s=interaction_end,
            )
            for edge_id, edge in edge_items
        }

    def initial_label(self, node_id: int, arrival_time: float = 0.0) -> TemporalSearchLabel:
        return TemporalSearchLabel(
            node_id=node_id,
            arrival_time=arrival_time,
            cost_to_come=0.0,
            parent_label_id=None,
            incoming_edge_id=None,
        )

    def _time_bin(self, arrival_time: float) -> int:
        return int(round(arrival_time / self.config.time_bin_size_s))

    def _clone_roadmap(self) -> Roadmap:
        query_graph = Roadmap()
        for node_id in sorted(self.roadmap.nodes):
            node = self.roadmap.nodes[node_id]
            added = query_graph.add_node(node.pose)
            if added.node_id != node.node_id:
                raise ValueError("Roadmap node IDs must be dense and zero-based for query cloning.")
        for edge_id in sorted(self.roadmap.edges):
            edge = self.roadmap.edges[edge_id]
            added_edge = query_graph.add_edge(
                source_id=edge.source_id,
                target_id=edge.target_id,
                primitive_label=edge.primitive_label,
                trajectory=edge.trajectory,
                duration_s=edge.duration_s,
                static_valid=edge.static_valid,
                geometric_cost=edge.geometric_cost,
            )
            if added_edge.edge_id != edge.edge_id:
                raise ValueError("Roadmap edge IDs must be dense and zero-based for query cloning.")
        return query_graph

    def _heading_error(self, a: float, b: float) -> float:
        return abs(normalize_angle(a - b))

    def _nearest_rollout_target(
        self,
        rollout_end: Pose2D,
        targets: list[RoadmapNode],
        source_id: int,
    ) -> RoadmapNode | None:
        candidates = [
            node
            for node in targets
            if node.node_id != source_id
            and euclidean_distance_xy(rollout_end, node.pose) <= self.config.connection_position_tolerance_m
            and self._heading_error(rollout_end.theta, node.pose.theta) <= self.config.connection_heading_tolerance_rad
        ]
        if not candidates:
            return None
        return min(candidates, key=lambda node: euclidean_distance_xy(rollout_end, node.pose))

    def _connect_sources_to_targets(
        self,
        query_graph: Roadmap,
        source_ids: list[int],
        target_ids: list[int],
        clearance: float,
    ) -> int:
        targets = [query_graph.nodes[target_id] for target_id in target_ids]
        added_count = 0
        existing_pairs = {
            (edge.source_id, edge.target_id, edge.primitive_label)
            for edge in query_graph.edges.values()
        }
        for source_id in source_ids:
            source = query_graph.nodes[source_id]
            for primitive in self.primitives:
                trajectory = simulate_primitive(
                    source.pose,
                    primitive,
                    self.vehicle_params,
                    self.planner_config.action_duration_s,
                    self.planner_config.integration_dt_s,
                )
                if trajectory.cost <= 1e-12:
                    continue
                target = self._nearest_rollout_target(trajectory.end, targets, source_id)
                if target is None:
                    continue
                key = (source_id, target.node_id, primitive.label)
                if key in existing_pairs:
                    continue
                if not is_trajectory_static_valid(trajectory, self.collision, clearance, self.static_world):
                    continue
                query_graph.add_edge(
                    source_id=source_id,
                    target_id=target.node_id,
                    primitive_label=primitive.label,
                    trajectory=trajectory,
                    duration_s=trajectory.duration_s,
                    static_valid=True,
                )
                existing_pairs.add(key)
                added_count += 1
        return added_count

    def _pose_satisfies_goal(self, pose: Pose2D, goal_pose: Pose2D) -> bool:
        if euclidean_distance_xy(pose, goal_pose) > self.config.goal_tolerance_m:
            return False
        return self._heading_error(pose.theta, goal_pose.theta) <= self.config.goal_heading_tolerance_rad

    def _prepare_query_graph(
        self,
        start_pose: Pose2D,
        goal_pose: Pose2D,
        clearance: float,
    ) -> tuple[Roadmap | None, int | None, int | None, dict[str, object]]:
        if not is_pose_static_valid(start_pose, self.collision, clearance, self.static_world):
            return None, None, None, {"failure": "start pose is not statically valid"}
        if not is_pose_static_valid(goal_pose, self.collision, clearance, self.static_world):
            return None, None, None, {"failure": "goal pose is not statically valid"}

        query_graph = self._clone_roadmap()
        original_node_ids = list(query_graph.nodes)
        start_node = query_graph.add_node(start_pose)
        goal_node = query_graph.add_node(goal_pose)

        start_edges = self._connect_sources_to_targets(
            query_graph,
            source_ids=[start_node.node_id],
            target_ids=original_node_ids + [goal_node.node_id],
            clearance=clearance,
        )
        goal_edges = self._connect_sources_to_targets(
            query_graph,
            source_ids=original_node_ids + [start_node.node_id],
            target_ids=[goal_node.node_id],
            clearance=clearance,
        )

        debug = {
            "original_node_count": len(original_node_ids),
            "start_connection_edges": start_edges,
            "goal_connection_edges": goal_edges,
        }
        if start_edges == 0 and not self._pose_satisfies_goal(start_pose, goal_pose):
            debug["failure"] = "start could not be connected to roadmap"
            return None, start_node.node_id, goal_node.node_id, debug
        return query_graph, start_node.node_id, goal_node.node_id, debug

    def _reconstruct_path(
        self,
        query_graph: Roadmap,
        labels: list[TemporalSearchLabel],
        goal_label_id: int,
        start_time_s: float,
    ) -> TemporalRoadmapPath:
        label_ids: list[int] = []
        cursor: int | None = goal_label_id
        while cursor is not None:
            label_ids.append(cursor)
            cursor = labels[cursor].parent_label_id
        label_ids.reverse()

        node_ids = [labels[label_id].node_id for label_id in label_ids]
        poses = [query_graph.nodes[node_id].pose for node_id in node_ids]
        arrival_times = [labels[label_id].arrival_time for label_id in label_ids]
        edge_ids = [
            labels[label_id].incoming_edge_id
            for label_id in label_ids[1:]
            if labels[label_id].incoming_edge_id is not None
        ]
        edges = [query_graph.edges[edge_id] for edge_id in edge_ids]
        return TemporalRoadmapPath(
            node_ids=node_ids,
            poses=poses,
            edge_ids=edge_ids,
            actions=[edge.primitive_label for edge in edges],
            arrival_times=arrival_times,
            segments=[edge.trajectory for edge in edges],
            total_cost=labels[goal_label_id].cost_to_come,
            total_traversal_time=labels[goal_label_id].arrival_time - start_time_s,
        )

    def plan(
        self,
        start_pose: Pose2D,
        goal_pose: Pose2D,
        dynamic_obstacles: list[DynamicCircleObstacle],
        *,
        clearance: float = 0.0,
        start_time_s: float = 0.0,
    ) -> TemporalRoadmapResult:
        """Run query-time temporal A* over a static-valid primitive roadmap."""

        query_graph, start_node_id, goal_node_id, debug = self._prepare_query_graph(start_pose, goal_pose, clearance)
        if query_graph is None or start_node_id is None or goal_node_id is None:
            return TemporalRoadmapResult(
                success=False,
                message=str(debug.get("failure", "failed to prepare query graph")),
                path=None,
                expanded_labels=0,
                generated_labels=0,
                rejected_dynamic_edges=0,
                pruned_labels=0,
                roadmap_node_count=len(self.roadmap.nodes),
                roadmap_edge_count=len(self.roadmap.edges),
                start_node_id=start_node_id,
                goal_node_id=goal_node_id,
                debug=debug,
            )

        labels: list[TemporalSearchLabel] = [self.initial_label(start_node_id, start_time_s)]
        open_heap: list[tuple[float, float, int, int]] = []
        best_cost: dict[tuple[int, int], float] = {
            (start_node_id, self._time_bin(start_time_s)): 0.0,
        }
        start_h = euclidean_distance_xy(start_pose, goal_pose)
        heapq.heappush(open_heap, (start_h, start_h, 0, 0))

        expanded_labels = 0
        generated_labels = 1
        rejected_dynamic_edges = 0
        pruned_labels = 0
        tie_breaker = 0
        validator = self._make_cached_validator(clearance) if self.config.use_temporal_cache else None
        stats_before = validator.stats.as_dict() if validator is not None else TemporalValidationStats().as_dict()

        while open_heap and expanded_labels < self.config.max_expanded_labels:
            _, _, _, label_id = heapq.heappop(open_heap)
            label = labels[label_id]
            label_key = (label.node_id, self._time_bin(label.arrival_time))
            if label.cost_to_come > best_cost.get(label_key, math.inf) + 1e-12:
                continue

            current_pose = query_graph.nodes[label.node_id].pose
            expanded_labels += 1

            if label.node_id == goal_node_id or self._pose_satisfies_goal(current_pose, goal_pose):
                path = self._reconstruct_path(query_graph, labels, label_id, start_time_s)
                return TemporalRoadmapResult(
                    success=True,
                    message="Temporal roadmap path found.",
                    path=path,
                    expanded_labels=expanded_labels,
                    generated_labels=generated_labels,
                    rejected_dynamic_edges=rejected_dynamic_edges,
                    pruned_labels=pruned_labels,
                    roadmap_node_count=len(query_graph.nodes),
                    roadmap_edge_count=len(query_graph.edges),
                    start_node_id=start_node_id,
                    goal_node_id=goal_node_id,
                    debug=self._with_cache_debug(debug, validator, stats_before),
                )

            for edge_id in query_graph.outgoing_edges.get(label.node_id, []):
                edge: RoadmapEdge = query_graph.edges[edge_id]
                arrival_time = label.arrival_time + edge.duration_s
                if arrival_time > self.config.max_arrival_time_s:
                    continue
                if validator is not None:
                    edge_is_temporally_valid = validator.validate_edge(edge, label.arrival_time, dynamic_obstacles)
                else:
                    edge_is_temporally_valid = temporal_collision_free(
                        edge.trajectory,
                        label.arrival_time,
                        self.static_world,
                        dynamic_obstacles,
                        self.collision,
                        clearance=clearance,
                        timestamp_mode=self.config.timestamp_mode,
                    )
                if not edge_is_temporally_valid:
                    rejected_dynamic_edges += 1
                    continue

                child_cost = label.cost_to_come + edge.geometric_cost
                child_key = (edge.target_id, self._time_bin(arrival_time))
                if child_cost >= best_cost.get(child_key, math.inf):
                    pruned_labels += 1
                    continue
                best_cost[child_key] = child_cost
                child_label = TemporalSearchLabel(
                    node_id=edge.target_id,
                    arrival_time=arrival_time,
                    cost_to_come=child_cost,
                    parent_label_id=label_id,
                    incoming_edge_id=edge.edge_id,
                )
                labels.append(child_label)
                child_label_id = len(labels) - 1
                generated_labels += 1
                heuristic = euclidean_distance_xy(query_graph.nodes[edge.target_id].pose, goal_pose)
                tie_breaker += 1
                heapq.heappush(open_heap, (child_cost + heuristic, heuristic, tie_breaker, child_label_id))

        message = "No temporal roadmap path found."
        if expanded_labels >= self.config.max_expanded_labels:
            message = f"No path found before max_expanded_labels={self.config.max_expanded_labels}."
        return TemporalRoadmapResult(
            success=False,
            message=message,
            path=None,
            expanded_labels=expanded_labels,
            generated_labels=generated_labels,
            rejected_dynamic_edges=rejected_dynamic_edges,
            pruned_labels=pruned_labels,
            roadmap_node_count=len(query_graph.nodes),
            roadmap_edge_count=len(query_graph.edges),
            start_node_id=start_node_id,
            goal_node_id=goal_node_id,
            debug=self._with_cache_debug(debug, validator, stats_before),
        )

    def _with_cache_debug(
        self,
        debug: dict[str, object],
        validator: CachedTemporalValidator | None,
        stats_before: dict[str, int],
    ) -> dict[str, object]:
        if validator is None:
            return debug
        stats_after = validator.stats.as_dict()
        query_stats = {
            key: stats_after.get(key, 0) - stats_before.get(key, 0)
            for key in stats_after
        }
        merged = dict(debug)
        merged["temporal_cache_enabled"] = True
        merged["temporal_interval_lookup_enabled"] = self.config.use_temporal_intervals
        merged["temporal_cache_stats"] = query_stats
        merged["temporal_cache_total_stats"] = stats_after
        merged["temporal_bin_cache_entries"] = len(validator.bin_cache)
        merged["temporal_interval_cache_entries"] = len(validator.interval_cache)
        merged["edge_obstacle_interaction_cache_entries"] = len(validator.interaction_cache)
        return merged
