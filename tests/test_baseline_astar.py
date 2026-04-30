from __future__ import annotations

import unittest

from src.configs.defaults import CollisionParams, PlannerConfig, builtin_vehicle_params
from src.models.obstacles import StaticWorld
from src.models.state import Pose2D
from src.planners.baseline_nonholonomic_astar import plan


class BaselineAStarTests(unittest.TestCase):
    def test_original_static_baseline_still_runs(self) -> None:
        config = PlannerConfig(
            world_width_m=1.0,
            world_height_m=1.0,
            xy_resolution_m=0.05,
            theta_bins=16,
            action_duration_s=1.0,
            integration_dt_s=0.1,
            goal_tolerance_m=0.10,
            max_iterations=1000,
        )
        result = plan(
            start_pose=Pose2D(0.2, 0.2, 0.0),
            goal_xy=(0.8, 0.2),
            rpm1=60.0,
            rpm2=90.0,
            clearance=0.0,
            vehicle_params=builtin_vehicle_params("turtlebot"),
            collision_params=CollisionParams(radius_m=0.03, source="test"),
            static_world=StaticWorld(bounds=config.bounds, obstacles=[]),
            config=config,
        )

        self.assertTrue(result.success, result.message)
        self.assertGreater(len(result.path_segments), 0)


if __name__ == "__main__":
    unittest.main()
