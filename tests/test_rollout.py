from __future__ import annotations

import unittest

from src.configs.defaults import builtin_vehicle_params
from src.core.rollout import simulate_primitive
from src.models.primitives import MotionPrimitive
from src.models.state import Pose2D


class RolloutTests(unittest.TestCase):
    def test_equal_rpm_produces_straight_turtlebot_motion(self) -> None:
        segment = simulate_primitive(
            Pose2D(0.0, 0.0, 0.0),
            MotionPrimitive(60.0, 60.0),
            builtin_vehicle_params("turtlebot"),
            action_duration_s=1.0,
            integration_dt_s=0.05,
        )

        self.assertGreater(segment.end.x, 0.0)
        self.assertAlmostEqual(segment.end.y, 0.0, places=6)
        self.assertAlmostEqual(segment.end.theta, 0.0, places=6)
        self.assertAlmostEqual(segment.duration_s, 1.0)


if __name__ == "__main__":
    unittest.main()
