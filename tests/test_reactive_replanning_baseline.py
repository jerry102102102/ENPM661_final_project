from __future__ import annotations

import unittest

from src.configs.defaults import CollisionParams, PlannerConfig, VehicleParams
from src.models.obstacles import DynamicCircleObstacle, StaticWorld
from src.models.state import Pose2D
from src.planners.reactive_replanning_baseline import ReactiveReplanningBaseline, ReactiveReplanningConfig


class ReactiveReplanningBaselineTests(unittest.TestCase):
    def _make_simple_problem(self):
        config = PlannerConfig(
            world_width_m=2.0,
            world_height_m=2.0,
            xy_resolution_m=0.05,
            theta_bins=32,
            action_duration_s=1.0,
            integration_dt_s=0.1,
            goal_tolerance_m=0.15,
            max_iterations=10000,
        )
        static_world = StaticWorld(config.bounds, [])
        vehicle = VehicleParams(0.1, 0.4, 0.4, "differential_drive")
        collision = CollisionParams(radius_m=0.05, source="test")
        return config, static_world, vehicle, collision

    def test_reactive_replanning_runs_static_path_without_replans(self) -> None:
        config, static_world, vehicle, collision = self._make_simple_problem()
        planner = ReactiveReplanningBaseline()

        result = planner.plan(
            Pose2D(0.2, 0.2, 0.0),
            (1.2, 0.2),
            [],
            rpm1=30.0,
            rpm2=60.0,
            clearance=0.0,
            vehicle_params=vehicle,
            collision_params=collision,
            static_world=static_world,
            planner_config=config,
        )

        self.assertTrue(result.success, result.message)
        self.assertEqual(result.number_of_replans, 0)
        self.assertEqual(result.dynamic_collision_failures, 0)
        self.assertGreater(len(result.path_segments), 0)

    def test_reactive_replanning_waits_when_first_segment_is_temporally_blocked(self) -> None:
        config, static_world, vehicle, collision = self._make_simple_problem()
        planner = ReactiveReplanningBaseline(
            ReactiveReplanningConfig(
                replan_period_s=1.0,
                max_replans=5,
                max_total_time_s=20.0,
            )
        )
        dynamic_obstacles = [
            DynamicCircleObstacle(
                initial_x=0.5,
                initial_y=0.2,
                velocity_x=0.0,
                velocity_y=0.2,
                radius=0.10,
                label="moving_blocker",
            )
        ]

        result = planner.plan(
            Pose2D(0.2, 0.2, 0.0),
            (1.2, 0.2),
            dynamic_obstacles,
            rpm1=30.0,
            rpm2=60.0,
            clearance=0.0,
            vehicle_params=vehicle,
            collision_params=collision,
            static_world=static_world,
            planner_config=config,
        )

        self.assertTrue(result.success, result.message)
        self.assertGreaterEqual(result.number_of_replans, 1)
        self.assertGreaterEqual(result.dynamic_collision_failures, 1)
        self.assertGreater(result.total_traversal_time, len(result.path_segments) * config.action_duration_s - 1e-9)


if __name__ == "__main__":
    unittest.main()
