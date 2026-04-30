from __future__ import annotations

import unittest

import math

from src.builders.roadmap_builder import (
    build_sampled_nonholonomic_roadmap,
    connect_nodes_with_primitives,
    sample_xy_heading_roadmap_nodes,
)
from src.configs.defaults import PlannerConfig, builtin_collision_params, builtin_vehicle_params
from src.core.rollout import simulate_primitive
from src.models.obstacles import StaticWorld
from src.models.primitives import MotionPrimitive
from src.models.roadmap import Roadmap
from src.models.state import Pose2D


class RoadmapBuilderTests(unittest.TestCase):
    def test_roadmap_edge_created_from_primitive_rollout(self) -> None:
        config = PlannerConfig(world_width_m=2.0, world_height_m=2.0, integration_dt_s=0.1)
        vehicle = builtin_vehicle_params("turtlebot")
        collision = builtin_collision_params("turtlebot_circle")
        primitive = MotionPrimitive(60.0, 60.0, "straight")
        start = Pose2D(0.4, 0.4, 0.0)
        rollout = simulate_primitive(start, primitive, vehicle, config.action_duration_s, config.integration_dt_s)

        roadmap = Roadmap()
        roadmap.add_node(start)
        roadmap.add_node(rollout.end)
        connect_nodes_with_primitives(
            roadmap,
            [primitive],
            vehicle,
            collision,
            clearance=0.0,
            static_world=StaticWorld((0.0, 2.0, 0.0, 2.0), []),
            config=config,
            position_tolerance_m=1e-9,
            heading_tolerance_rad=1e-9,
        )

        self.assertEqual(len(roadmap.edges), 1)
        edge = next(iter(roadmap.edges.values()))
        self.assertTrue(edge.static_valid)
        self.assertEqual(edge.source_id, 0)
        self.assertEqual(edge.target_id, 1)

    def test_sampled_xy_heading_nodes_are_query_independent(self) -> None:
        config = PlannerConfig(world_width_m=2.0, world_height_m=2.0, xy_resolution_m=0.05, theta_bins=32)
        poses = sample_xy_heading_roadmap_nodes(
            xy_count=4,
            static_world=StaticWorld((0.0, 2.0, 0.0, 2.0), []),
            collision=builtin_collision_params("turtlebot_circle"),
            clearance=0.0,
            config=config,
            headings_rad=[0.0, math.pi / 2.0],
            sampling_mode="grid",
            grid_spacing_m=0.5,
        )

        self.assertEqual(len(poses), 8)
        self.assertEqual({round(pose.theta, 6) for pose in poses}, {0.0, round(math.pi / 2.0, 6)})

    def test_sampled_nonholonomic_roadmap_has_static_primitive_edges(self) -> None:
        config = PlannerConfig(
            world_width_m=2.0,
            world_height_m=2.0,
            xy_resolution_m=0.05,
            theta_bins=32,
            action_duration_s=1.0,
            integration_dt_s=0.1,
        )
        vehicle = builtin_vehicle_params("turtlebot")
        collision = builtin_collision_params("turtlebot_circle")
        roadmap = build_sampled_nonholonomic_roadmap(
            xy_sample_count=100,
            primitives=[MotionPrimitive(60.0, 60.0, "straight")],
            vehicle_params=vehicle,
            collision=collision,
            clearance=0.0,
            static_world=StaticWorld((0.0, 2.0, 0.0, 2.0), []),
            config=config,
            headings_rad=[0.0],
            sampling_mode="grid",
            grid_spacing_m=0.2,
            position_tolerance_m=0.15,
            heading_tolerance_rad=0.1,
        )

        self.assertGreater(len(roadmap.nodes), 0)
        self.assertGreater(len(roadmap.edges), 0)
        self.assertTrue(all(edge.static_valid for edge in roadmap.edges.values()))
        self.assertTrue(all(edge.geometric_cost > 0.0 for edge in roadmap.edges.values()))


if __name__ == "__main__":
    unittest.main()
