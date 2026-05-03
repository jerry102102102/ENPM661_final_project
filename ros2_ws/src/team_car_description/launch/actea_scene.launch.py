#!/usr/bin/env python3
"""Launch only the ACTEA Gazebo scene and team-car model."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    description_share = get_package_share_directory("team_car_description")
    gazebo_launch = os.path.join(description_share, "launch", "gazebo.launch.py")
    world_path = os.path.join(description_share, "world", "mbgazworld_actea.sdf")

    return LaunchDescription(
        [
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("world_name", default_value="competition_environment"),
            DeclareLaunchArgument("model_name", default_value="team_car"),
            DeclareLaunchArgument("x", default_value="0.35"),
            DeclareLaunchArgument("y", default_value="0.35"),
            DeclareLaunchArgument("z", default_value="0.0"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gazebo_launch),
                launch_arguments={
                    "headless": LaunchConfiguration("headless"),
                    "world_name": LaunchConfiguration("world_name"),
                    "model_name": LaunchConfiguration("model_name"),
                    "world_path": world_path,
                    "x": LaunchConfiguration("x"),
                    "y": LaunchConfiguration("y"),
                    "z": LaunchConfiguration("z"),
                    "yaw": LaunchConfiguration("yaw"),
                }.items(),
            ),
        ]
    )

