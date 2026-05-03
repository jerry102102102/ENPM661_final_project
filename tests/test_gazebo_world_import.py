from __future__ import annotations

import unittest
from pathlib import Path

from src.integrations.gazebo_world import (
    import_gazebo_world_config,
    method_run_config_from_gazebo_import,
)


class GazeboWorldImportTests(unittest.TestCase):
    def test_mbgazworld_import_expands_looping_waypoints(self) -> None:
        imported = import_gazebo_world_config(
            Path("mbgazworld/world_config.json"),
            annotation_horizon_s=20.0,
        )

        self.assertEqual(imported.world_name, "competition_environment")
        self.assertEqual(len(imported.static_world.obstacles), 4)
        self.assertGreaterEqual(len(imported.dynamic_obstacles), 10)
        self.assertTrue(all(obstacle.active_start_time_s is not None for obstacle in imported.dynamic_obstacles))
        self.assertTrue(all(obstacle.active_end_time_s is not None for obstacle in imported.dynamic_obstacles))
        self.assertEqual(imported.static_world.bounds[0], 0.0)
        self.assertEqual(imported.static_world.bounds[2], 0.0)

    def test_import_can_build_method_run_config(self) -> None:
        imported = import_gazebo_world_config(
            Path("mbgazworld/world_config.json"),
            annotation_horizon_s=12.0,
        )
        config = method_run_config_from_gazebo_import(imported, xy_sample_count=80)

        self.assertIs(config.static_world, imported.static_world)
        self.assertEqual(config.temporal_annotation_end_time_s, 12.0)
        self.assertEqual(config.xy_sample_count, 80)


if __name__ == "__main__":
    unittest.main()

