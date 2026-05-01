"""Cached and analytic temporal edge validation for roadmap queries."""

from __future__ import annotations

from dataclasses import dataclass, field
import math

from src.configs.defaults import CollisionParams
from src.core.time_parameterization import normalized_sample_times
from src.core.temporal_validation import temporal_collision_free
from src.models.obstacles import DynamicCircleObstacle, StaticWorld
from src.models.roadmap import RoadmapEdge
from src.models.state import Pose2D, TrajectorySegment


Interval = tuple[float, float]
ObstacleSignature = tuple[tuple[str, float, float, float, float, float], ...]
EPS = 1e-12


@dataclass(frozen=True)
class AxisAlignedBounds:
    """2D axis-aligned bounds."""

    min_x: float
    max_x: float
    min_y: float
    max_y: float

    def expanded(self, margin: float) -> "AxisAlignedBounds":
        return AxisAlignedBounds(
            self.min_x - margin,
            self.max_x + margin,
            self.min_y - margin,
            self.max_y + margin,
        )

    def intersects(self, other: "AxisAlignedBounds") -> bool:
        return not (
            self.max_x < other.min_x
            or other.max_x < self.min_x
            or self.max_y < other.min_y
            or other.max_y < self.min_y
        )


@dataclass(frozen=True)
class TemporalValidityInterval:
    """Departure-time interval for an edge."""

    edge_id: int
    start_time_s: float
    end_time_s: float
    valid: bool


@dataclass(frozen=True)
class EdgeTemporalAnnotation:
    """First-class ACTEA metadata for one roadmap edge and obstacle scenario."""

    edge_id: int
    obstacle_signature: ObstacleSignature
    blocked_intervals_exact: list[Interval]
    valid_intervals_exact: list[Interval]
    temporal_horizon_s: tuple[float, float]
    temporal_annotation_mode: str
    blocked_intervals_by_obstacle: dict[str, list[Interval]] | None = None

    def status_at(self, departure_time_s: float) -> bool | None:
        """Return edge validity at departure time, or None outside the annotated horizon."""

        horizon_start, horizon_end = self.temporal_horizon_s
        if departure_time_s < horizon_start - EPS or departure_time_s > horizon_end + EPS:
            return None
        for start, end in self.blocked_intervals_exact:
            if start <= departure_time_s <= end:
                return False
        for start, end in self.valid_intervals_exact:
            if start <= departure_time_s <= end:
                return True
        return None

    def to_validity_intervals(self) -> list[TemporalValidityInterval]:
        """Return legacy interval objects used by the existing cache lookup path."""

        intervals = [
            *(TemporalValidityInterval(self.edge_id, start, end, False) for start, end in self.blocked_intervals_exact),
            *(TemporalValidityInterval(self.edge_id, start, end, True) for start, end in self.valid_intervals_exact),
        ]
        intervals.sort(key=lambda item: (item.start_time_s, item.valid, item.end_time_s))
        return intervals


@dataclass
class EdgeTemporalAnnotationStore:
    """Parallel edge-annotation table keyed by edge id and dynamic-obstacle scenario."""

    annotations: dict[tuple[int, ObstacleSignature], EdgeTemporalAnnotation] = field(default_factory=dict)

    def set(self, annotation: EdgeTemporalAnnotation) -> None:
        self.annotations[(annotation.edge_id, annotation.obstacle_signature)] = annotation

    def get_by_signature(self, edge_id: int, signature: ObstacleSignature) -> EdgeTemporalAnnotation | None:
        return self.annotations.get((edge_id, signature))

    def get(self, edge_id: int, dynamic_obstacles: list[DynamicCircleObstacle]) -> EdgeTemporalAnnotation | None:
        return self.get_by_signature(edge_id, obstacle_signature(dynamic_obstacles))

    def __len__(self) -> int:
        return len(self.annotations)


@dataclass
class TemporalValidationStats:
    """Cache and validation counters for one or more queries."""

    exact_checks: int = 0
    dynamic_collision_rejections: int = 0
    candidate_filter_skips: int = 0
    temporal_cache_hits_free: int = 0
    temporal_cache_hits_blocked: int = 0
    temporal_cache_misses: int = 0
    interval_cache_hits_free: int = 0
    interval_cache_hits_blocked: int = 0
    interaction_cache_hits: int = 0
    analytic_interval_annotations: int = 0

    def as_dict(self) -> dict[str, int]:
        return {
            "exact_checks": self.exact_checks,
            "dynamic_collision_rejections": self.dynamic_collision_rejections,
            "candidate_filter_skips": self.candidate_filter_skips,
            "temporal_cache_hits_free": self.temporal_cache_hits_free,
            "temporal_cache_hits_blocked": self.temporal_cache_hits_blocked,
            "temporal_cache_misses": self.temporal_cache_misses,
            "interval_cache_hits_free": self.interval_cache_hits_free,
            "interval_cache_hits_blocked": self.interval_cache_hits_blocked,
            "interaction_cache_hits": self.interaction_cache_hits,
            "analytic_interval_annotations": self.analytic_interval_annotations,
        }


def trajectory_bounds(trajectory: TrajectorySegment) -> AxisAlignedBounds:
    """Return XY bounds of a trajectory."""

    xs = [sample.x for sample in trajectory.samples]
    ys = [sample.y for sample in trajectory.samples]
    return AxisAlignedBounds(min(xs), max(xs), min(ys), max(ys))


def _clip_interval(interval: Interval, lower: float, upper: float) -> Interval | None:
    start = max(interval[0], lower)
    end = min(interval[1], upper)
    if end < start - EPS:
        return None
    return (start, end)


def merge_intervals(intervals: list[Interval]) -> list[Interval]:
    """Merge overlapping closed intervals."""

    if not intervals:
        return []
    sorted_intervals = sorted(intervals)
    merged = [sorted_intervals[0]]
    for start, end in sorted_intervals[1:]:
        prev_start, prev_end = merged[-1]
        if start <= prev_end + EPS:
            merged[-1] = (prev_start, max(prev_end, end))
        else:
            merged.append((start, end))
    return merged


def complement_intervals(blocked: list[Interval], horizon_start: float, horizon_end: float) -> list[Interval]:
    """Return available intervals inside a finite horizon."""

    available: list[Interval] = []
    cursor = horizon_start
    for start, end in merge_intervals(blocked):
        clipped = _clip_interval((start, end), horizon_start, horizon_end)
        if clipped is None:
            continue
        start, end = clipped
        if start > cursor + EPS:
            available.append((cursor, start))
        cursor = max(cursor, end)
    if cursor < horizon_end - EPS:
        available.append((cursor, horizon_end))
    return available


def _intersect_interval_lists(a: list[Interval], b: list[Interval]) -> list[Interval]:
    out: list[Interval] = []
    i = 0
    j = 0
    a_merged = merge_intervals(a)
    b_merged = merge_intervals(b)
    while i < len(a_merged) and j < len(b_merged):
        start = max(a_merged[i][0], b_merged[j][0])
        end = min(a_merged[i][1], b_merged[j][1])
        if start <= end + EPS:
            out.append((start, end))
        if a_merged[i][1] < b_merged[j][1]:
            i += 1
        else:
            j += 1
    return merge_intervals(out)


def _solve_quadratic_leq(a: float, b: float, c: float) -> list[Interval]:
    """Solve a*d^2 + b*d + c <= 0 over the real line."""

    if abs(a) <= EPS:
        if abs(b) <= EPS:
            return [(-math.inf, math.inf)] if c <= EPS else []
        root = -c / b
        if b > 0.0:
            return [(-math.inf, root)]
        return [(root, math.inf)]

    discriminant = (b * b) - (4.0 * a * c)
    if discriminant < -EPS:
        return [(-math.inf, math.inf)] if a < 0.0 else []
    if abs(discriminant) <= EPS:
        root = -b / (2.0 * a)
        if a > 0.0:
            return [(root, root)]
        return [(-math.inf, math.inf)]

    sqrt_disc = math.sqrt(max(discriminant, 0.0))
    r1 = (-b - sqrt_disc) / (2.0 * a)
    r2 = (-b + sqrt_disc) / (2.0 * a)
    low, high = min(r1, r2), max(r1, r2)
    if a > 0.0:
        return [(low, high)]
    return [(-math.inf, low), (high, math.inf)]


def _solve_linear_range(alpha: float, beta: float, lower: float, upper: float) -> list[Interval]:
    """Solve lower <= alpha*d + beta <= upper."""

    if abs(alpha) <= EPS:
        return [(-math.inf, math.inf)] if lower - EPS <= beta <= upper + EPS else []
    a = (lower - beta) / alpha
    b = (upper - beta) / alpha
    return [(min(a, b), max(a, b))]


def _quadratic_norm_leq_intervals(
    q0: tuple[float, float],
    q1: tuple[float, float],
    radius: float,
) -> list[Interval]:
    """Solve ||q0 + q1*d||^2 <= radius^2 for departure time d."""

    a = (q1[0] * q1[0]) + (q1[1] * q1[1])
    b = 2.0 * ((q0[0] * q1[0]) + (q0[1] * q1[1]))
    c = (q0[0] * q0[0]) + (q0[1] * q0[1]) - (radius * radius)
    return _solve_quadratic_leq(a, b, c)


def dynamic_obstacle_swept_bounds(
    obstacle: DynamicCircleObstacle,
    start_time_s: float,
    end_time_s: float,
) -> AxisAlignedBounds:
    """Return AABB swept by a constant-velocity circle over a time range."""

    start_x = obstacle.initial_x + obstacle.velocity_x * start_time_s
    start_y = obstacle.initial_y + obstacle.velocity_y * start_time_s
    end_x = obstacle.initial_x + obstacle.velocity_x * end_time_s
    end_y = obstacle.initial_y + obstacle.velocity_y * end_time_s
    return AxisAlignedBounds(
        min(start_x, end_x) - obstacle.radius,
        max(start_x, end_x) + obstacle.radius,
        min(start_y, end_y) - obstacle.radius,
        max(start_y, end_y) + obstacle.radius,
    )


def _pose_segment_departure_blocked_intervals(
    pose_a: Pose2D,
    pose_b: Pose2D,
    rel_time_a: float,
    rel_time_b: float,
    obstacle: DynamicCircleObstacle,
    collision_radius: float,
    horizon_start_s: float,
    horizon_end_s: float,
) -> list[Interval]:
    """Compute exact blocked departure-time intervals for one path segment.

    The robot center is linearly interpolated between two trajectory samples
    over edge-relative time s in [rel_time_a, rel_time_b]. The obstacle follows
    constant velocity in absolute time d + s, where d is edge departure time.
    We project the resulting two-variable quadratic collision condition onto d.
    """

    dt = rel_time_b - rel_time_a
    if dt <= EPS:
        return []

    robot_vx = (pose_b.x - pose_a.x) / dt
    robot_vy = (pose_b.y - pose_a.y) / dt
    # Robot position r(s) = robot_c + robot_v * s.
    robot_cx = pose_a.x - robot_vx * rel_time_a
    robot_cy = pose_a.y - robot_vy * rel_time_a

    # Difference r(s) - o(d+s) = A + B*s + C*d.
    ax = robot_cx - obstacle.initial_x
    ay = robot_cy - obstacle.initial_y
    bx = robot_vx - obstacle.velocity_x
    by = robot_vy - obstacle.velocity_y
    cx = -obstacle.velocity_x
    cy = -obstacle.velocity_y
    radius = collision_radius + obstacle.radius

    blocked: list[Interval] = []

    # Endpoint cases: s = rel_time_a and s = rel_time_b.
    for rel_time in (rel_time_a, rel_time_b):
        q0 = (ax + bx * rel_time, ay + by * rel_time)
        q1 = (cx, cy)
        for interval in _quadratic_norm_leq_intervals(q0, q1, radius):
            clipped = _clip_interval(interval, horizon_start_s, horizon_end_s)
            if clipped is not None:
                blocked.append(clipped)

    # Interior closest-point case on the segment.
    bb = (bx * bx) + (by * by)
    if bb <= EPS:
        # Distance does not change with s; endpoint intervals already cover it.
        return merge_intervals(blocked)

    # s*(d) = -(B dot (A + C*d)) / ||B||^2 = alpha*d + beta.
    alpha = -((bx * cx) + (by * cy)) / bb
    beta = -((bx * ax) + (by * ay)) / bb
    s_range = _solve_linear_range(alpha, beta, rel_time_a, rel_time_b)

    # Perpendicular residual P(A + C*d), where P projects orthogonal to B.
    # This is q0 + q1*d in the 2D plane.
    def project_perp(vx: float, vy: float) -> tuple[float, float]:
        scale = ((vx * bx) + (vy * by)) / bb
        return (vx - scale * bx, vy - scale * by)

    q0_perp = project_perp(ax, ay)
    q1_perp = project_perp(cx, cy)
    distance_ok = _quadratic_norm_leq_intervals(q0_perp, q1_perp, radius)
    for interval in _intersect_interval_lists(distance_ok, s_range):
        clipped = _clip_interval(interval, horizon_start_s, horizon_end_s)
        if clipped is not None:
            blocked.append(clipped)

    return merge_intervals(blocked)


def edge_obstacle_blocked_departure_intervals(
    edge: RoadmapEdge,
    obstacle: DynamicCircleObstacle,
    collision: CollisionParams,
    clearance: float,
    horizon_start_s: float,
    horizon_end_s: float,
) -> list[Interval]:
    """Compute exact blocked departure-time intervals for one edge-obstacle pair."""

    samples = edge.trajectory.samples
    if len(samples) < 2:
        return []
    rel_times = normalized_sample_times(len(samples), 0.0, edge.duration_s)
    radius = collision.radius_m + clearance
    blocked: list[Interval] = []
    for index in range(1, len(samples)):
        blocked.extend(
            _pose_segment_departure_blocked_intervals(
                samples[index - 1],
                samples[index],
                rel_times[index - 1],
                rel_times[index],
                obstacle,
                radius,
                horizon_start_s,
                horizon_end_s,
            )
        )
    return merge_intervals(blocked)


def edge_blocked_departure_intervals(
    edge: RoadmapEdge,
    dynamic_obstacles: list[DynamicCircleObstacle],
    collision: CollisionParams,
    clearance: float,
    horizon_start_s: float,
    horizon_end_s: float,
) -> list[Interval]:
    """Compute exact blocked departure-time intervals for one edge."""

    blocked: list[Interval] = []
    for obstacle in dynamic_obstacles:
        blocked.extend(
            edge_obstacle_blocked_departure_intervals(
                edge,
                obstacle,
                collision,
                clearance,
                horizon_start_s,
                horizon_end_s,
            )
        )
    return merge_intervals(blocked)


def edge_valid_departure_intervals(
    edge: RoadmapEdge,
    dynamic_obstacles: list[DynamicCircleObstacle],
    collision: CollisionParams,
    clearance: float,
    horizon_start_s: float,
    horizon_end_s: float,
) -> list[Interval]:
    """Compute exact valid departure-time intervals for one edge."""

    return complement_intervals(
        edge_blocked_departure_intervals(edge, dynamic_obstacles, collision, clearance, horizon_start_s, horizon_end_s),
        horizon_start_s,
        horizon_end_s,
    )


def obstacle_signature(obstacles: list[DynamicCircleObstacle]) -> tuple[tuple[str, float, float, float, float, float], ...]:
    """Stable signature for a dynamic-obstacle scenario."""

    return tuple(
        sorted(
            (
                obstacle.label,
                round(obstacle.initial_x, 9),
                round(obstacle.initial_y, 9),
                round(obstacle.velocity_x, 9),
                round(obstacle.velocity_y, 9),
                round(obstacle.radius, 9),
            )
            for obstacle in obstacles
        )
    )


def candidate_dynamic_obstacles_for_edge(
    edge: RoadmapEdge,
    departure_time_s: float,
    max_time_slack_s: float,
    dynamic_obstacles: list[DynamicCircleObstacle],
    collision: CollisionParams,
    clearance: float,
) -> list[DynamicCircleObstacle]:
    """Filter obstacles that can geometrically interact with an edge."""

    edge_bounds = trajectory_bounds(edge.trajectory).expanded(collision.radius_m + clearance)
    end_time_s = departure_time_s + edge.duration_s + max(0.0, max_time_slack_s)
    return [
        obstacle
        for obstacle in dynamic_obstacles
        if edge_bounds.intersects(dynamic_obstacle_swept_bounds(obstacle, departure_time_s, end_time_s))
    ]


@dataclass
class CachedTemporalValidator:
    """Edge-obstacle filtering, time-bin cache, and ACTEA metadata."""

    static_world: StaticWorld
    collision: CollisionParams
    clearance: float
    time_bin_size_s: float
    max_time_slack_s: float = 0.0
    timestamp_mode: str = "normalized"
    use_interval_lookup: bool = True
    stats: TemporalValidationStats = field(default_factory=TemporalValidationStats)
    annotation_store: EdgeTemporalAnnotationStore = field(default_factory=EdgeTemporalAnnotationStore)
    bin_cache: dict[tuple[int, int, tuple[tuple[str, float, float, float, float, float], ...]], bool] = field(
        default_factory=dict
    )
    interval_cache: dict[
        tuple[int, tuple[tuple[str, float, float, float, float, float], ...]],
        list[TemporalValidityInterval],
    ] = field(default_factory=dict)
    interaction_cache: dict[
        tuple[int, tuple[tuple[str, float, float, float, float, float], ...]],
        tuple[int, ...],
    ] = field(default_factory=dict)

    def time_bin(self, time_s: float) -> int:
        return int(round(time_s / self.time_bin_size_s))

    def _bin_cache_key(
        self,
        edge_id: int,
        departure_time_s: float,
        dynamic_obstacles: list[DynamicCircleObstacle],
    ) -> tuple[int, int, tuple[tuple[str, float, float, float, float, float], ...]]:
        return (edge_id, self.time_bin(departure_time_s), obstacle_signature(dynamic_obstacles))

    def _interval_key(
        self,
        edge_id: int,
        dynamic_obstacles: list[DynamicCircleObstacle],
    ) -> tuple[int, tuple[tuple[str, float, float, float, float, float], ...]]:
        return (edge_id, obstacle_signature(dynamic_obstacles))

    def _interaction_key(
        self,
        edge_id: int,
        dynamic_obstacles: list[DynamicCircleObstacle],
    ) -> tuple[int, tuple[tuple[str, float, float, float, float, float], ...]]:
        return (edge_id, obstacle_signature(dynamic_obstacles))

    def annotate_edge_obstacle_candidates(
        self,
        edge: RoadmapEdge,
        dynamic_obstacles: list[DynamicCircleObstacle],
        *,
        start_time_s: float,
        end_time_s: float,
    ) -> tuple[int, ...]:
        """Precompute obstacles that could ever interact with an edge over a horizon."""

        edge_bounds = trajectory_bounds(edge.trajectory).expanded(self.collision.radius_m + self.clearance)
        candidate_indices = tuple(
            index
            for index, obstacle in enumerate(dynamic_obstacles)
            if edge_bounds.intersects(dynamic_obstacle_swept_bounds(obstacle, start_time_s, end_time_s))
        )
        self.interaction_cache[self._interaction_key(edge.edge_id, dynamic_obstacles)] = candidate_indices
        return candidate_indices

    def _interval_status(
        self,
        edge_id: int,
        departure_time_s: float,
        dynamic_obstacles: list[DynamicCircleObstacle],
    ) -> bool | None:
        if not self.use_interval_lookup:
            return None
        annotation = self.annotation_store.get(edge_id, dynamic_obstacles)
        if annotation is not None:
            status = annotation.status_at(departure_time_s)
            if status is not None:
                if status:
                    self.stats.interval_cache_hits_free += 1
                else:
                    self.stats.interval_cache_hits_blocked += 1
                return status
        intervals = self.interval_cache.get(self._interval_key(edge_id, dynamic_obstacles))
        if not intervals:
            return None
        for interval in intervals:
            if not interval.valid and interval.start_time_s <= departure_time_s <= interval.end_time_s:
                self.stats.interval_cache_hits_blocked += 1
                return False
        for interval in intervals:
            if interval.valid and interval.start_time_s <= departure_time_s <= interval.end_time_s:
                self.stats.interval_cache_hits_free += 1
                return True
        return None

    def validate_edge(
        self,
        edge: RoadmapEdge,
        departure_time_s: float,
        dynamic_obstacles: list[DynamicCircleObstacle],
    ) -> bool:
        """Validate one edge at one departure time with cache support."""

        interval_status = self._interval_status(edge.edge_id, departure_time_s, dynamic_obstacles)
        if interval_status is not None:
            return interval_status

        cache_key = self._bin_cache_key(edge.edge_id, departure_time_s, dynamic_obstacles)
        cached = self.bin_cache.get(cache_key)
        if cached is not None:
            if cached:
                self.stats.temporal_cache_hits_free += 1
            else:
                self.stats.temporal_cache_hits_blocked += 1
            return cached

        interaction_key = self._interaction_key(edge.edge_id, dynamic_obstacles)
        if interaction_key in self.interaction_cache:
            self.stats.interaction_cache_hits += 1
            candidate_pool = [dynamic_obstacles[index] for index in self.interaction_cache[interaction_key]]
        else:
            candidate_pool = dynamic_obstacles

        candidates = candidate_dynamic_obstacles_for_edge(
            edge,
            departure_time_s,
            self.max_time_slack_s,
            candidate_pool,
            self.collision,
            self.clearance,
        )
        if dynamic_obstacles and not candidates:
            self.stats.candidate_filter_skips += 1
            self.bin_cache[cache_key] = True
            return True

        self.stats.temporal_cache_misses += 1
        self.stats.exact_checks += 1
        valid = temporal_collision_free(
            edge.trajectory,
            departure_time_s,
            self.static_world,
            candidates,
            self.collision,
            clearance=self.clearance,
            timestamp_mode=self.timestamp_mode,
        )
        if not valid:
            self.stats.dynamic_collision_rejections += 1
        self.bin_cache[cache_key] = valid
        return valid

    def edge_is_valid_at_time(
        self,
        edge: RoadmapEdge,
        departure_time_s: float,
        dynamic_obstacles: list[DynamicCircleObstacle],
    ) -> bool:
        """Return whether an edge is valid when departed at a given time."""

        return self.validate_edge(edge, departure_time_s, dynamic_obstacles)

    def edge_cost_at_time(
        self,
        edge: RoadmapEdge,
        departure_time_s: float,
        dynamic_obstacles: list[DynamicCircleObstacle],
    ) -> float:
        """Return geometric edge cost at valid times and infinity when blocked."""

        if not self.edge_is_valid_at_time(edge, departure_time_s, dynamic_obstacles):
            return math.inf
        return edge.geometric_cost

    def annotate_edge_intervals(
        self,
        edge: RoadmapEdge,
        dynamic_obstacles: list[DynamicCircleObstacle],
        *,
        start_time_s: float,
        end_time_s: float,
        step_s: float | None = None,
    ) -> list[TemporalValidityInterval]:
        """Compute analytic edge validity intervals over departure time.

        ``step_s`` is accepted for backward compatibility but is not used by
        the analytic solver.
        """

        annotation = self.annotate_edge_temporal_annotation(
            edge,
            dynamic_obstacles,
            start_time_s=start_time_s,
            end_time_s=end_time_s,
            step_s=step_s,
        )
        return annotation.to_validity_intervals()

    def annotate_edge_temporal_annotation(
        self,
        edge: RoadmapEdge,
        dynamic_obstacles: list[DynamicCircleObstacle],
        *,
        start_time_s: float,
        end_time_s: float,
        step_s: float | None = None,
    ) -> EdgeTemporalAnnotation:
        """Compute and store first-class ACTEA edge temporal annotation."""

        _ = step_s
        blocked_by_obstacle: dict[str, list[Interval]] = {}
        for index, obstacle in enumerate(dynamic_obstacles):
            obstacle_key = f"{index}:{obstacle.label}"
            blocked_by_obstacle[obstacle_key] = edge_obstacle_blocked_departure_intervals(
                edge,
                obstacle,
                self.collision,
                self.clearance,
                start_time_s,
                end_time_s,
            )
        blocked = merge_intervals([interval for intervals in blocked_by_obstacle.values() for interval in intervals])
        valid = complement_intervals(blocked, start_time_s, end_time_s)
        annotation = EdgeTemporalAnnotation(
            edge_id=edge.edge_id,
            obstacle_signature=obstacle_signature(dynamic_obstacles),
            blocked_intervals_exact=blocked,
            valid_intervals_exact=valid,
            temporal_horizon_s=(start_time_s, end_time_s),
            temporal_annotation_mode="actea",
            blocked_intervals_by_obstacle=blocked_by_obstacle,
        )
        self.stats.analytic_interval_annotations += 1
        self.annotation_store.set(annotation)
        self.interval_cache[self._interval_key(edge.edge_id, dynamic_obstacles)] = annotation.to_validity_intervals()
        return annotation


def pose_list_bounds(poses: list[Pose2D]) -> AxisAlignedBounds:
    """Return bounds for arbitrary poses."""

    return AxisAlignedBounds(
        min(pose.x for pose in poses),
        max(pose.x for pose in poses),
        min(pose.y for pose in poses),
        max(pose.y for pose in poses),
    )
