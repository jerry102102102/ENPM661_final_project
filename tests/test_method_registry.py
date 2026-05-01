from __future__ import annotations

import unittest

from src.configs.defaults import CollisionParams, PlannerConfig, VehicleParams
from src.experiments.method_registry import MethodRunConfig, plan_with_method, registered_method_names
from src.models.obstacles import DynamicCircleObstacle, StaticWorld
from src.models.state import Pose2D


class MethodRegistryTests(unittest.TestCase):
    def _config(self) -> MethodRunConfig:
        planner_config = PlannerConfig(
            world_width_m=2.0,
            world_height_m=2.0,
            xy_resolution_m=0.05,
            theta_bins=32,
            action_duration_s=1.0,
            integration_dt_s=0.1,
            goal_tolerance_m=0.15,
            max_iterations=10000,
        )
        return MethodRunConfig(
            static_world=StaticWorld(planner_config.bounds, []),
            vehicle_params=VehicleParams(0.1, 0.4, 0.4, "differential_drive"),
            collision_params=CollisionParams(radius_m=0.05, source="test"),
            planner_config=planner_config,
            xy_sample_count=100,
            headings_rad=(0.0, 1.5707963267948966, -1.5707963267948966, 3.141592653589793),
            grid_spacing_m=0.2,
        )

    def test_registry_exposes_expected_methods(self) -> None:
        self.assertEqual(
            set(registered_method_names()),
            {
                "static_astar",
                "expansion_temporal",
                "sampled_temporal_online",
                "sampled_temporal_actea",
                "sampled_temporal_bin_cache",
                "reactive_replanning",
            },
        )

    def test_registered_static_and_reactive_methods_return_common_result(self) -> None:
        config = self._config()
        start = Pose2D(0.2, 0.2, 0.0)
        goal = Pose2D(1.2, 0.2, 0.0)

        static_result = plan_with_method("static_astar", start, goal, [], config)
        reactive_result = plan_with_method("reactive_replanning", start, goal, [], config)

        self.assertTrue(static_result.success, static_result.message)
        self.assertTrue(reactive_result.success, reactive_result.message)
        self.assertEqual(static_result.method_name, "static_astar")
        self.assertEqual(reactive_result.method_name, "reactive_replanning")
        self.assertIsNotNone(static_result.path_length)
        self.assertIsNotNone(reactive_result.traversal_time)

    def test_registered_sampled_actea_method_uses_cache_stats(self) -> None:
        config = self._config()
        start = Pose2D(0.2, 0.2, 0.0)
        goal = Pose2D(1.6, 0.2, 0.0)
        obstacles = [DynamicCircleObstacle(0.8, 0.2, 0.0, 0.0, 0.08, "blocker")]

        result = plan_with_method("sampled_temporal_actea", start, goal, obstacles, config)

        self.assertTrue(result.success, result.message)
        self.assertEqual(result.method_name, "sampled_temporal_actea")
        self.assertIsNotNone(result.build_time_sec)
        self.assertIsNotNone(result.annotation_time_sec)
        self.assertIsNotNone(result.cache_stats)
        self.assertGreater(result.roadmap_edges or 0, 0)


if __name__ == "__main__":
    unittest.main()
