from __future__ import annotations

import unittest

from src.configs.defaults import CollisionParams
from src.core.static_collision import is_pose_static_valid, is_trajectory_static_valid
from src.models.obstacles import StaticWorld
from src.models.state import OrientedBox, Pose2D, TrajectorySegment


class StaticCollisionTests(unittest.TestCase):
    def test_static_trajectory_collision_checking(self) -> None:
        world = StaticWorld(
            bounds=(0.0, 2.0, 0.0, 2.0),
            obstacles=[OrientedBox("block", 1.0, 1.0, 0.2, 0.2)],
        )
        collision = CollisionParams(radius_m=0.05)
        valid_pose = Pose2D(0.2, 0.2, 0.0)
        invalid_pose = Pose2D(1.0, 1.0, 0.0)
        trajectory = TrajectorySegment(valid_pose, invalid_pose, [valid_pose, invalid_pose], 1.0, 1.0)

        self.assertTrue(is_pose_static_valid(valid_pose, collision, 0.0, world))
        self.assertFalse(is_pose_static_valid(invalid_pose, collision, 0.0, world))
        self.assertFalse(is_trajectory_static_valid(trajectory, collision, 0.0, world))


if __name__ == "__main__":
    unittest.main()
