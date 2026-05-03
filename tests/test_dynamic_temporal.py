from __future__ import annotations

import unittest

from src.configs.defaults import CollisionParams
from src.core.dynamic_collision import obstacle_position_at_time
from src.core.temporal_validation import temporal_collision_free
from src.models.obstacles import DynamicCircleObstacle, StaticWorld
from src.models.state import Pose2D, TrajectorySegment


class DynamicTemporalTests(unittest.TestCase):
    def test_obstacle_position_at_time(self) -> None:
        obstacle = DynamicCircleObstacle(1.0, 2.0, 0.5, -0.25, 0.1)
        self.assertEqual(obstacle_position_at_time(obstacle, 2.0), (2.0, 1.5))

    def test_temporal_trajectory_validation_detects_dynamic_collision(self) -> None:
        start = Pose2D(0.0, 0.0, 0.0)
        end = Pose2D(1.0, 0.0, 0.0)
        trajectory = TrajectorySegment(start, end, [start, end], 1.0, 1.0)
        world = StaticWorld(bounds=(-1.0, 2.0, -1.0, 1.0), obstacles=[])
        collision = CollisionParams(radius_m=0.1)
        dynamic = [DynamicCircleObstacle(1.0, 0.0, 0.0, 0.0, 0.1)]

        self.assertFalse(temporal_collision_free(trajectory, 0.0, world, dynamic, collision))
        self.assertTrue(temporal_collision_free(trajectory, 0.0, world, [], collision))

    def test_time_gated_obstacle_is_ignored_outside_active_window(self) -> None:
        start = Pose2D(0.0, 0.0, 0.0)
        end = Pose2D(1.0, 0.0, 0.0)
        trajectory = TrajectorySegment(start, end, [start, end], 1.0, 1.0)
        world = StaticWorld(bounds=(-1.0, 2.0, -1.0, 1.0), obstacles=[])
        collision = CollisionParams(radius_m=0.1)
        dynamic = [
            DynamicCircleObstacle(
                1.0,
                0.0,
                0.0,
                0.0,
                0.1,
                active_start_time_s=2.0,
                active_end_time_s=3.0,
            )
        ]

        self.assertTrue(temporal_collision_free(trajectory, 0.0, world, dynamic, collision))
        self.assertFalse(temporal_collision_free(trajectory, 2.0, world, dynamic, collision))


if __name__ == "__main__":
    unittest.main()
