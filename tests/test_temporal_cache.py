from __future__ import annotations

import unittest

from src.configs.defaults import CollisionParams
from src.core.temporal_cache import (
    CachedTemporalValidator,
    candidate_dynamic_obstacles_for_edge,
    edge_blocked_departure_intervals,
    edge_valid_departure_intervals,
)
from src.models.obstacles import DynamicCircleObstacle, StaticWorld
from src.models.roadmap import RoadmapEdge
from src.models.state import Pose2D, TrajectorySegment


def _edge() -> RoadmapEdge:
    start = Pose2D(0.0, 0.0, 0.0)
    mid = Pose2D(0.5, 0.0, 0.0)
    end = Pose2D(1.0, 0.0, 0.0)
    trajectory = TrajectorySegment(start, end, [start, mid, end], cost=1.0, duration_s=1.0)
    return RoadmapEdge(
        edge_id=0,
        source_id=0,
        target_id=1,
        primitive_label="straight",
        trajectory=trajectory,
        duration_s=1.0,
        geometric_cost=1.0,
        static_valid=True,
    )


class TemporalCacheTests(unittest.TestCase):
    def test_candidate_filter_removes_far_obstacle(self) -> None:
        edge = _edge()
        collision = CollisionParams(radius_m=0.05, source="test")
        far = DynamicCircleObstacle(10.0, 10.0, 0.0, 0.0, 0.1)

        candidates = candidate_dynamic_obstacles_for_edge(edge, 0.0, 0.0, [far], collision, 0.0)

        self.assertEqual(candidates, [])

    def test_time_bin_cache_reuses_blocked_result(self) -> None:
        edge = _edge()
        validator = CachedTemporalValidator(
            static_world=StaticWorld(bounds=(-1.0, 2.0, -1.0, 1.0), obstacles=[]),
            collision=CollisionParams(radius_m=0.05, source="test"),
            clearance=0.0,
            time_bin_size_s=0.5,
        )
        obstacle = DynamicCircleObstacle(0.5, 0.0, 0.0, 0.0, 0.1)

        self.assertFalse(validator.validate_edge(edge, 0.0, [obstacle]))
        self.assertFalse(validator.validate_edge(edge, 0.0, [obstacle]))

        self.assertEqual(validator.stats.exact_checks, 1)
        self.assertEqual(validator.stats.temporal_cache_hits_blocked, 1)

    def test_analytic_interval_annotation_is_reused(self) -> None:
        edge = _edge()
        validator = CachedTemporalValidator(
            static_world=StaticWorld(bounds=(-1.0, 2.0, -1.0, 1.0), obstacles=[]),
            collision=CollisionParams(radius_m=0.05, source="test"),
            clearance=0.0,
            time_bin_size_s=0.5,
            use_interval_lookup=True,
        )
        obstacle = DynamicCircleObstacle(0.5, 0.0, 0.0, 0.0, 0.1)

        intervals = validator.annotate_edge_intervals(edge, [obstacle], start_time_s=0.0, end_time_s=1.0, step_s=0.5)
        self.assertGreaterEqual(len(intervals), 1)
        self.assertFalse(validator.validate_edge(edge, 0.0, [obstacle]))
        self.assertGreaterEqual(validator.stats.interval_cache_hits_blocked, 1)
        self.assertEqual(validator.stats.exact_checks, 0)
        self.assertGreaterEqual(validator.stats.analytic_interval_annotations, 1)

    def test_analytic_edge_intervals_match_crossing_obstacle(self) -> None:
        edge = _edge()
        collision = CollisionParams(radius_m=0.05, source="test")
        obstacle = DynamicCircleObstacle(0.5, -1.0, 0.0, 1.0, 0.05)

        blocked = edge_blocked_departure_intervals(edge, [obstacle], collision, 0.0, 0.0, 2.0)
        valid = edge_valid_departure_intervals(edge, [obstacle], collision, 0.0, 0.0, 2.0)

        self.assertEqual(len(blocked), 1)
        self.assertAlmostEqual(blocked[0][0], 0.35857864376269044)
        self.assertAlmostEqual(blocked[0][1], 0.6414213562373096)
        self.assertEqual(len(valid), 2)
        self.assertAlmostEqual(valid[0][1], blocked[0][0])
        self.assertAlmostEqual(valid[1][0], blocked[0][1])

    def test_edge_obstacle_interaction_cache_is_used(self) -> None:
        edge = _edge()
        validator = CachedTemporalValidator(
            static_world=StaticWorld(bounds=(-1.0, 2.0, -1.0, 1.0), obstacles=[]),
            collision=CollisionParams(radius_m=0.05, source="test"),
            clearance=0.0,
            time_bin_size_s=0.5,
        )
        obstacles = [
            DynamicCircleObstacle(0.5, 0.0, 0.0, 0.0, 0.1),
            DynamicCircleObstacle(10.0, 10.0, 0.0, 0.0, 0.1),
        ]

        candidate_indices = validator.annotate_edge_obstacle_candidates(
            edge,
            obstacles,
            start_time_s=0.0,
            end_time_s=2.0,
        )
        self.assertEqual(candidate_indices, (0,))
        self.assertFalse(validator.validate_edge(edge, 0.0, obstacles))
        self.assertEqual(validator.stats.interaction_cache_hits, 1)


if __name__ == "__main__":
    unittest.main()
