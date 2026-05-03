#!/usr/bin/env python3
"""Launch Gazebo Sim with the Phase 2 course world and spawn the team car."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = "team_car_description"
    pkg_share = get_package_share_directory(pkg_name)
    install_root = os.path.dirname(pkg_share)

    urdf_path = os.path.join(pkg_share, "urdf", "ENPM661_Group4_CAD_V2.SLDASM.urdf")
    default_world_path = os.path.join(pkg_share, "world", "mbgazworld_actea.sdf")
    controllers_path = os.path.join(pkg_share, "config", "ros2_controllers.yaml")

    with open(urdf_path, "r", encoding="utf-8") as urdf_file:
        robot_description = urdf_file.read()

    set_gz_resource_path = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        os.pathsep.join([pkg_share, install_root]),
    )
    set_ign_resource_path = SetEnvironmentVariable(
        "IGN_GAZEBO_RESOURCE_PATH",
        os.pathsep.join([pkg_share, install_root]),
    )
    set_session_type = SetEnvironmentVariable("XDG_SESSION_TYPE", "x11")
    set_current_desktop = SetEnvironmentVariable("XDG_CURRENT_DESKTOP", "XFCE")
    set_desktop_session = SetEnvironmentVariable("DESKTOP_SESSION", "xfce")
    set_qt_platform = SetEnvironmentVariable("QT_QPA_PLATFORM", "xcb")
    set_qt_no_mitshm = SetEnvironmentVariable("QT_X11_NO_MITSHM", "1")
    set_runtime_dir = SetEnvironmentVariable("XDG_RUNTIME_DIR", "/tmp/runtime-root")
    set_software_gl = SetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE", "1")
    set_no_audio = SetEnvironmentVariable("GAZEBO_AUDIO", "0")
    gz_share = get_package_share_directory("ros_gz_sim")
    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz_share, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args": LaunchConfiguration("gui_gz_args"),
            "debug_env": LaunchConfiguration("debug_env"),
        }.items(),
        condition=UnlessCondition(LaunchConfiguration("headless")),
    )
    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz_share, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args": LaunchConfiguration("headless_gz_args"),
            "debug_env": LaunchConfiguration("debug_env"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("headless")),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"use_sim_time": True, "robot_description": robot_description}],
        output="screen",
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            LaunchConfiguration("model_name"),
            "-topic",
            "robot_description",
            "-x",
            LaunchConfiguration("x"),
            "-y",
            LaunchConfiguration("y"),
            "-z",
            LaunchConfiguration("z"),
            "-Y",
            LaunchConfiguration("yaw"),
        ],
        output="screen",
    )

    def controller_spawner(controller_name):
        # Spawner exits non-zero if a controller is already loaded by an older
        # Gazebo instance. Treat an already-active controller as success so
        # repeated launches do not fail during tuning sessions.
        script = f"""
set -e
if ros2 control list_controllers --controller-manager /controller_manager 2>/dev/null \\
  | sed -r 's/\\x1B\\[[0-9;]*[mK]//g' \\
  | awk '$1 == "{controller_name}" && $3 == "active" {{ found=1 }} END {{ exit found ? 0 : 1 }}'; then
  echo "{controller_name} already active; skipping spawner."
  exit 0
fi
ros2 run controller_manager spawner {controller_name} \\
  --controller-manager /controller_manager \\
  --param-file "{controllers_path}" \\
  --switch-timeout 30
"""
        return ExecuteProcess(cmd=["/bin/bash", "-lc", script], output="screen")

    joint_state_broadcaster = controller_spawner("joint_state_broadcaster")
    drive_velocity_controller = controller_spawner("rear_wheel_velocity_controller")
    steering_position_controller = controller_spawner("steering_position_controller")

    spawn_then_joint_state = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[TimerAction(period=2.0, actions=[joint_state_broadcaster])],
        )
    )

    joint_state_then_drive = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[TimerAction(period=2.0, actions=[drive_velocity_controller])],
        )
    )

    drive_then_steering = RegisterEventHandler(
        OnProcessExit(
            target_action=drive_velocity_controller,
            on_exit=[TimerAction(period=2.0, actions=[steering_position_controller])],
        )
    )

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            [
                "/world/",
                LaunchConfiguration("world_name"),
                "/pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
            ],
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Use server-only Gazebo Sim mode (-s) without GUI.",
            ),
            DeclareLaunchArgument(
                "debug_env",
                default_value="false",
                description="Print Gazebo environment variables from ros_gz_sim launcher.",
            ),
            DeclareLaunchArgument(
                "world_path",
                default_value=default_world_path,
                description="SDF world file used by Gazebo Sim.",
            ),
            DeclareLaunchArgument(
                "gui_gz_args",
                default_value=["-r -v 2 ", LaunchConfiguration("world_path")],
                description="Arguments passed to gz sim when GUI mode is enabled.",
            ),
            DeclareLaunchArgument(
                "headless_gz_args",
                default_value=["-s -r -v 2 ", LaunchConfiguration("world_path")],
                description="Arguments passed to gz sim when headless mode is enabled.",
            ),
            DeclareLaunchArgument("model_name", default_value="team_car"),
            DeclareLaunchArgument("world_name", default_value="competition_environment"),
            DeclareLaunchArgument("x", default_value="0.35"),
            DeclareLaunchArgument("y", default_value="0.35"),
            DeclareLaunchArgument("z", default_value="0.0"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            set_gz_resource_path,
            set_ign_resource_path,
            set_session_type,
            set_current_desktop,
            set_desktop_session,
            set_qt_platform,
            set_qt_no_mitshm,
            set_runtime_dir,
            set_software_gl,
            set_no_audio,
            gz_sim_gui,
            gz_sim_headless,
            robot_state_publisher,
            spawn_robot,
            spawn_then_joint_state,
            joint_state_then_drive,
            drive_then_steering,
            gz_bridge,
        ]
    )
