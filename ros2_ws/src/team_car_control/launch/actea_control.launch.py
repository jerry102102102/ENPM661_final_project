#!/usr/bin/env python3
"""Launch only the ACTEA route follower controller."""

from __future__ import annotations

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    control_share = get_package_share_directory("team_car_control")
    project_root = Path(os.environ.get("ACTEA_PROJECT_ROOT", Path.cwd())).resolve()
    generated_route_file = project_root / "outputs" / "gazebo_integration" / "mbgazworld_route.json"
    packaged_route_file = Path(control_share) / "routes" / "mbgazworld_route.json"
    default_route_file = generated_route_file if generated_route_file.exists() else packaged_route_file
    default_log_path = project_root / "outputs" / "gazebo_integration" / "actea_gazebo_execution_{stamp}.csv"

    return LaunchDescription(
        [
            DeclareLaunchArgument("world_name", default_value="competition_environment"),
            DeclareLaunchArgument("model_name", default_value="team_car"),
            DeclareLaunchArgument("route_file", default_value=str(default_route_file)),
            DeclareLaunchArgument("execution_log_path", default_value=str(default_log_path)),
            DeclareLaunchArgument("drive_rpm", default_value="24.0"),
            DeclareLaunchArgument("slow_drive_rpm", default_value="16.0"),
            DeclareLaunchArgument("lookahead_m", default_value="0.22"),
            DeclareLaunchArgument("goal_tolerance_m", default_value="0.16"),
            DeclareLaunchArgument("planned_start_time_s", default_value="-1.0"),
            Node(
                package="team_car_control",
                executable="actea_route_follower",
                parameters=[
                    {"use_sim_time": True},
                    {"route_file": LaunchConfiguration("route_file")},
                    {"world_name": LaunchConfiguration("world_name")},
                    {"model_name": LaunchConfiguration("model_name")},
                    {"pose_info_topic": ["/world/", LaunchConfiguration("world_name"), "/pose/info"]},
                    {"drive_rpm": LaunchConfiguration("drive_rpm")},
                    {"slow_drive_rpm": LaunchConfiguration("slow_drive_rpm")},
                    {"lookahead_m": LaunchConfiguration("lookahead_m")},
                    {"goal_tolerance_m": LaunchConfiguration("goal_tolerance_m")},
                    {"planned_start_time_s": LaunchConfiguration("planned_start_time_s")},
                    {"execution_log_path": LaunchConfiguration("execution_log_path")},
                ],
                output="screen",
            ),
        ]
    )

