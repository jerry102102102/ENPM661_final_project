#!/usr/bin/env python3
"""Launch only the planner/executor action server."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("world_name", default_value="phase2_course"),
            DeclareLaunchArgument("model_name", default_value="team_car"),
            DeclareLaunchArgument("reference_z", default_value="0.0"),
            DeclareLaunchArgument("drive_command_topic", default_value="/rear_wheel_velocity_controller/commands"),
            DeclareLaunchArgument("steering_command_topic", default_value="/steering_position_controller/commands"),
            DeclareLaunchArgument("joint_states_topic", default_value="/joint_states"),
            DeclareLaunchArgument("max_steering_command_rad", default_value="0.7854"),
            DeclareLaunchArgument("pose_info_topic", default_value="/world/phase2_course/pose/info"),
            DeclareLaunchArgument("execution_log_path", default_value="/tmp/team_car_execution_debug_{stamp}.csv"),
            DeclareLaunchArgument("planner_log_path", default_value="/tmp/team_car_planner_debug_{stamp}.json"),
            DeclareLaunchArgument("tracking_lookahead_m", default_value="0.26"),
            DeclareLaunchArgument("tracking_goal_tolerance_m", default_value="0.12"),
            DeclareLaunchArgument("tracking_progress_timeout_sec", default_value="4.0"),
            DeclareLaunchArgument("tracking_max_runtime_sec", default_value="70.0"),
            DeclareLaunchArgument("tracking_fast_steering_threshold_rad", default_value="0.34"),
            DeclareLaunchArgument("tracking_medium_steering_threshold_rad", default_value="0.52"),
            DeclareLaunchArgument("tracking_medium_speed_scale", default_value="0.95"),
            DeclareLaunchArgument("tracking_slow_speed_scale", default_value="0.80"),
            Node(
                package="team_car_control",
                executable="navigate_action_server",
                parameters=[
                    {"use_sim_time": True},
                    {"world_name": LaunchConfiguration("world_name")},
                    {"model_name": LaunchConfiguration("model_name")},
                    {"reference_z": LaunchConfiguration("reference_z")},
                    {"drive_command_topic": LaunchConfiguration("drive_command_topic")},
                    {"steering_command_topic": LaunchConfiguration("steering_command_topic")},
                    {"joint_states_topic": LaunchConfiguration("joint_states_topic")},
                    {"max_steering_command_rad": LaunchConfiguration("max_steering_command_rad")},
                    {"pose_info_topic": LaunchConfiguration("pose_info_topic")},
                    {"execution_log_path": LaunchConfiguration("execution_log_path")},
                    {"planner_log_path": LaunchConfiguration("planner_log_path")},
                    {"tracking_lookahead_m": LaunchConfiguration("tracking_lookahead_m")},
                    {"tracking_goal_tolerance_m": LaunchConfiguration("tracking_goal_tolerance_m")},
                    {"tracking_progress_timeout_sec": LaunchConfiguration("tracking_progress_timeout_sec")},
                    {"tracking_max_runtime_sec": LaunchConfiguration("tracking_max_runtime_sec")},
                    {"tracking_fast_steering_threshold_rad": LaunchConfiguration("tracking_fast_steering_threshold_rad")},
                    {"tracking_medium_steering_threshold_rad": LaunchConfiguration("tracking_medium_steering_threshold_rad")},
                    {"tracking_medium_speed_scale": LaunchConfiguration("tracking_medium_speed_scale")},
                    {"tracking_slow_speed_scale": LaunchConfiguration("tracking_slow_speed_scale")},
                ],
                output="screen",
            ),
        ]
    )
