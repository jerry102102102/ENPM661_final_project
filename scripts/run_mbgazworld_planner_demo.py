"""Plan one ACTEA route on the imported mbgazworld map."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[1]
if REPO_ROOT.as_posix() not in sys.path:
    sys.path.insert(0, REPO_ROOT.as_posix())

from src.integrations.gazebo_world import import_gazebo_world_config, method_run_config_from_gazebo_import
from src.integrations.planner_api import build_controller_planning_function
from src.models.state import Pose2D


def main() -> None:
    parser = argparse.ArgumentParser(description="Import mbgazworld and run one ACTEA planning query.")
    parser.add_argument("--config", type=Path, default=Path("mbgazworld/world_config.json"))
    parser.add_argument("--output", type=Path, default=Path("outputs/gazebo_integration/mbgazworld_route.json"))
    parser.add_argument("--horizon", type=float, default=30.0)
    parser.add_argument("--xy-samples", type=int, default=360)
    parser.add_argument("--grid-spacing", type=float, default=0.25)
    parser.add_argument("--start-time", type=float, default=6.0)
    parser.add_argument("--start", type=float, nargs=3, default=(0.35, 0.35, 0.0), metavar=("X", "Y", "THETA_RAD"))
    parser.add_argument("--goal", type=float, nargs=3, default=(3.65, 1.65, 0.0), metavar=("X", "Y", "THETA_RAD"))
    args = parser.parse_args()

    imported = import_gazebo_world_config(args.config, annotation_horizon_s=args.horizon, margin_m=0.35)
    config = method_run_config_from_gazebo_import(
        imported,
        xy_sample_count=args.xy_samples,
        grid_spacing_m=args.grid_spacing,
    )
    planning_function = build_controller_planning_function(imported.dynamic_obstacles, config, mode="actea")

    start_gazebo = Pose2D(*args.start)
    goal_gazebo = Pose2D(*args.goal)
    start = imported.transform_pose(start_gazebo)
    goal = imported.transform_pose(goal_gazebo)
    route = planning_function.plan(start, goal, start_time_s=args.start_time)
    gazebo_waypoints = [imported.inverse_transform_pose(pose) for pose in route.waypoints]
    gazebo_timed_waypoints = [
        {
            "time_s": item.time_s,
            "x": item.pose.x - imported.origin_offset_xy[0],
            "y": item.pose.y - imported.origin_offset_xy[1],
            "theta": item.pose.theta,
        }
        for item in route.timed_waypoints
    ]

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(
            {
                "world_name": imported.world_name,
                "success": route.success,
                "message": route.message,
                "planned_start_time_s": args.start_time,
                "start_pose_gazebo": {"x": start_gazebo.x, "y": start_gazebo.y, "theta": start_gazebo.theta},
                "goal_pose_gazebo": {"x": goal_gazebo.x, "y": goal_gazebo.y, "theta": goal_gazebo.theta},
                "build_time_s": planning_function.build_time_s,
                "annotation_time_s": planning_function.annotation_time_s,
                "traversal_time_s": route.traversal_time_s,
                "path_cost": route.path_cost,
                "expanded_labels": route.expanded_labels,
                "rejected_dynamic_edges": route.rejected_dynamic_edges,
                "origin_offset_xy": imported.origin_offset_xy,
                "dynamic_obstacle_segments": len(imported.dynamic_obstacles),
                "waypoints": [
                    {"x": pose.x, "y": pose.y, "theta": pose.theta}
                    for pose in route.waypoints
                ],
                "gazebo_waypoints": [
                    {"x": pose.x, "y": pose.y, "theta": pose.theta}
                    for pose in gazebo_waypoints
                ],
                "timed_waypoints": [
                    {"time_s": item.time_s, "x": item.pose.x, "y": item.pose.y, "theta": item.pose.theta}
                    for item in route.timed_waypoints
                ],
                "gazebo_timed_waypoints": gazebo_timed_waypoints,
                "commands": [
                    {
                        "duration_s": command.duration_s,
                        "action_name": command.action_name,
                        "command": command.command,
                        "rpm_l": command.rpm_l,
                        "rpm_r": command.rpm_r,
                        "steering_angle_deg": command.steering_angle_deg,
                        "drive_rpm": command.drive_rpm,
                    }
                    for command in route.commands
                ],
            },
            indent=2,
        )
    )
    print(f"Wrote {args.output}")


if __name__ == "__main__":
    main()
