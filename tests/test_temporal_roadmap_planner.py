from __future__ import annotations

import math
import unittest

from src.builders.roadmap_builder import build_sampled_nonholonomic_roadmap
from src.configs.defaults import CollisionParams, PlannerConfig, VehicleParams
from src.models.obstacles import DynamicCircleObstacle, StaticWorld
from src.models.primitives import get_action_set
from src.models.state import Pose2D
from src.planners.temporal_roadmap_planner import TemporalRoadmapPlanner, TemporalRoadmapPlannerConfig


class TemporalRoadmapPlannerTests(unittest.TestCase):
    def test_temporal_annotation_mode_normalizes_legacy_flags(self) -> None:
        online = TemporalRoadmapPlannerConfig()
        sampled = TemporalRoadmapPlannerConfig(use_temporal_cache=True)
        exact = TemporalRoadmapPlannerConfig(use_temporal_intervals=True)
        explicit = TemporalRoadmapPlannerConfig(temporal_annotation_mode="actea")

        self.assertEqual(online.temporal_annotation_mode, "online")
        self.assertFalse(online.use_temporal_cache)
        self.assertEqual(sampled.temporal_annotation_mode, "bin_cache")
        self.assertTrue(sampled.use_temporal_cache)
        self.assertFalse(sampled.use_temporal_intervals)
        self.assertEqual(exact.temporal_annotation_mode, "actea")
        self.assertTrue(exact.use_temporal_cache)
        self.assertTrue(exact.use_temporal_intervals)
        self.assertEqual(explicit.temporal_annotation_mode, "actea")
        self.assertTrue(explicit.use_temporal_cache)
        self.assertTrue(explicit.use_temporal_intervals)

    def test_temporal_roadmap_finds_path_and_rejects_dynamic_edges(self) -> None:
        config = PlannerConfig(
            world_width_m=4.0,
            world_height_m=2.0,
            xy_resolution_m=0.08,
            theta_bins=32,
            action_duration_s=0.8,
            integration_dt_s=0.1,
        )
        static_world = StaticWorld(config.bounds, [])
        vehicle = VehicleParams(
            wheel_radius_m=0.1,
            track_width_m=0.4,
            wheelbase_m=0.4,
            motion_model="differential_drive",
        )
        collision = CollisionParams(radius_m=0.05, source="test")
        primitives = get_action_set(30.0, 60.0, vehicle)
        start = Pose2D(0.2, 0.2, 0.0)
        goal = Pose2D(1.6, 0.2, 0.0)

        roadmap = build_sampled_nonholonomic_roadmap(
            xy_sample_count=100,
            primitives=primitives,
            vehicle_params=vehicle,
            collision=collision,
            clearance=0.0,
            static_world=static_world,
            config=config,
            headings_rad=[0.0, math.pi / 2.0, -math.pi / 2.0, math.pi],
            sampling_mode="grid",
            grid_spacing_m=0.2,
            position_tolerance_m=0.15,
            heading_tolerance_rad=0.4,
        )
        planner = TemporalRoadmapPlanner(
            roadmap,
            static_world,
            collision,
            vehicle,
            primitives,
            config,
            TemporalRoadmapPlannerConfig(
                max_arrival_time_s=30.0,
                time_bin_size_s=0.25,
                goal_tolerance_m=0.15,
                goal_heading_tolerance_rad=math.radians(30.0),
                connection_position_tolerance_m=0.15,
                connection_heading_tolerance_rad=math.radians(30.0),
            ),
        )
        result = planner.plan(
            start,
            goal,
            [DynamicCircleObstacle(0.8, 0.2, 0.0, 0.0, 0.08)],
            clearance=0.0,
        )

        self.assertTrue(result.success, result.message)
        self.assertIsNotNone(result.path)
        self.assertGreater(result.rejected_dynamic_edges, 0)
        self.assertGreater(int(result.debug["start_connection_edges"]), 0)
        self.assertGreater(result.roadmap_edge_count, len(roadmap.edges))

    def test_sampled_roadmap_is_reused_without_query_mutation(self) -> None:
        config = PlannerConfig(
            world_width_m=2.0,
            world_height_m=2.0,
            xy_resolution_m=0.05,
            theta_bins=32,
            action_duration_s=1.0,
            integration_dt_s=0.1,
        )
        static_world = StaticWorld(config.bounds, [])
        vehicle = VehicleParams(0.1, 0.4, 0.4, "differential_drive")
        collision = CollisionParams(radius_m=0.05, source="test")
        primitives = get_action_set(30.0, 60.0, vehicle)
        roadmap = build_sampled_nonholonomic_roadmap(
            xy_sample_count=100,
            primitives=primitives,
            vehicle_params=vehicle,
            collision=collision,
            clearance=0.0,
            static_world=static_world,
            config=config,
            headings_rad=[0.0, math.pi / 2.0, -math.pi / 2.0, math.pi],
            sampling_mode="grid",
            grid_spacing_m=0.2,
            position_tolerance_m=0.15,
            heading_tolerance_rad=0.4,
        )
        base_node_count = len(roadmap.nodes)
        base_edge_count = len(roadmap.edges)
        planner = TemporalRoadmapPlanner(
            roadmap,
            static_world,
            collision,
            vehicle,
            primitives,
            config,
            TemporalRoadmapPlannerConfig(
                max_arrival_time_s=30.0,
                time_bin_size_s=0.25,
                goal_tolerance_m=0.15,
                goal_heading_tolerance_rad=math.radians(30.0),
                connection_position_tolerance_m=0.15,
                connection_heading_tolerance_rad=math.radians(30.0),
            ),
        )

        first = planner.plan(Pose2D(0.2, 0.2, 0.0), Pose2D(1.6, 0.2, 0.0), [])
        second = planner.plan(Pose2D(0.2, 1.0, 0.0), Pose2D(1.6, 1.0, 0.0), [])

        self.assertTrue(first.success, first.message)
        self.assertTrue(second.success, second.message)
        self.assertEqual(len(roadmap.nodes), base_node_count)
        self.assertEqual(len(roadmap.edges), base_edge_count)
        self.assertEqual(int(first.debug["original_node_count"]), base_node_count)
        self.assertEqual(int(second.debug["original_node_count"]), base_node_count)

    def test_temporal_cache_reduces_repeated_query_exact_checks(self) -> None:
        config = PlannerConfig(
            world_width_m=2.0,
            world_height_m=2.0,
            xy_resolution_m=0.05,
            theta_bins=32,
            action_duration_s=1.0,
            integration_dt_s=0.1,
        )
        static_world = StaticWorld(config.bounds, [])
        vehicle = VehicleParams(0.1, 0.4, 0.4, "differential_drive")
        collision = CollisionParams(radius_m=0.05, source="test")
        primitives = get_action_set(30.0, 60.0, vehicle)
        roadmap = build_sampled_nonholonomic_roadmap(
            xy_sample_count=100,
            primitives=primitives,
            vehicle_params=vehicle,
            collision=collision,
            clearance=0.0,
            static_world=static_world,
            config=config,
            headings_rad=[0.0, math.pi / 2.0, -math.pi / 2.0, math.pi],
            sampling_mode="grid",
            grid_spacing_m=0.2,
            position_tolerance_m=0.15,
            heading_tolerance_rad=0.4,
        )
        planner = TemporalRoadmapPlanner(
            roadmap,
            static_world,
            collision,
            vehicle,
            primitives,
            config,
            TemporalRoadmapPlannerConfig(
                max_arrival_time_s=30.0,
                time_bin_size_s=0.25,
                goal_tolerance_m=0.15,
                goal_heading_tolerance_rad=math.radians(30.0),
                connection_position_tolerance_m=0.15,
                connection_heading_tolerance_rad=math.radians(30.0),
                use_temporal_cache=True,
            ),
        )
        obstacle = [DynamicCircleObstacle(0.8, 0.2, 0.0, 0.0, 0.08)]

        first = planner.plan(Pose2D(0.2, 0.2, 0.0), Pose2D(1.6, 0.2, 0.0), obstacle)
        second = planner.plan(Pose2D(0.2, 0.2, 0.0), Pose2D(1.6, 0.2, 0.0), obstacle)

        self.assertTrue(first.success, first.message)
        self.assertTrue(second.success, second.message)
        first_checks = first.debug["temporal_cache_stats"]["exact_checks"]  # type: ignore[index]
        second_checks = second.debug["temporal_cache_stats"]["exact_checks"]  # type: ignore[index]
        second_hits = second.debug["temporal_cache_stats"]["temporal_cache_hits_free"]  # type: ignore[index]
        self.assertGreater(first_checks, 0)
        self.assertLess(second_checks, first_checks)
        self.assertGreater(second_hits, 0)

    def test_actea_annotations_are_first_class_planner_metadata(self) -> None:
        config = PlannerConfig(
            world_width_m=2.0,
            world_height_m=2.0,
            xy_resolution_m=0.05,
            theta_bins=32,
            action_duration_s=1.0,
            integration_dt_s=0.1,
        )
        static_world = StaticWorld(config.bounds, [])
        vehicle = VehicleParams(0.1, 0.4, 0.4, "differential_drive")
        collision = CollisionParams(radius_m=0.05, source="test")
        primitives = get_action_set(30.0, 60.0, vehicle)
        roadmap = build_sampled_nonholonomic_roadmap(
            xy_sample_count=100,
            primitives=primitives,
            vehicle_params=vehicle,
            collision=collision,
            clearance=0.0,
            static_world=static_world,
            config=config,
            headings_rad=[0.0, math.pi / 2.0, -math.pi / 2.0, math.pi],
            sampling_mode="grid",
            grid_spacing_m=0.2,
            position_tolerance_m=0.15,
            heading_tolerance_rad=0.4,
        )
        obstacle = [DynamicCircleObstacle(0.8, 0.2, 0.0, 0.0, 0.08, "blocker")]
        planner = TemporalRoadmapPlanner(
            roadmap,
            static_world,
            collision,
            vehicle,
            primitives,
            config,
            TemporalRoadmapPlannerConfig(
                temporal_annotation_mode="actea",
                max_arrival_time_s=30.0,
                time_bin_size_s=0.25,
                goal_tolerance_m=0.15,
                goal_heading_tolerance_rad=math.radians(30.0),
                connection_position_tolerance_m=0.15,
                connection_heading_tolerance_rad=math.radians(30.0),
            ),
        )

        annotations = planner.annotate_temporal_intervals(obstacle, start_time_s=0.0, end_time_s=8.0)
        result = planner.plan(Pose2D(0.2, 0.2, 0.0), Pose2D(1.6, 0.2, 0.0), obstacle)

        self.assertEqual(len(annotations), len(roadmap.edges))
        self.assertEqual(len(planner.temporal_annotation_store), len(roadmap.edges))
        self.assertTrue(result.success, result.message)
        self.assertEqual(result.debug["temporal_annotation_mode"], "actea")
        self.assertEqual(result.debug["edge_temporal_annotation_entries"], len(roadmap.edges))
        stats = result.debug["temporal_cache_stats"]  # type: ignore[assignment]
        self.assertGreater(stats["interval_cache_hits_free"], 0)  # type: ignore[index]


if __name__ == "__main__":
    unittest.main()
