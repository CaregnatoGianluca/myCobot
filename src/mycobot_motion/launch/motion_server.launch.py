#!/usr/bin/env python3
"""
All-in-one launcher: Gazebo simulation + MoveIt2 + motion action server.

Usage (full stack — default):
  export LIBGL_ALWAYS_SOFTWARE=1 MESA_GL_VERSION_OVERRIDE=4.5
  ros2 launch mycobot_motion motion_server.launch.py

Motion-server only (sim already running in another terminal):
  ros2 launch mycobot_motion motion_server.launch.py with_sim:=false

Real robot (no Gazebo) — start real.launch.py first, then:
  ros2 launch mycobot_motion motion_server.launch.py with_sim:=false
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup = get_package_share_directory('mycobot_bringup')
    pkg_motion  = get_package_share_directory('mycobot_motion')

    params_file = os.path.join(pkg_motion, 'config', 'motion_params.yaml')

    args = [
        DeclareLaunchArgument(
            'with_sim', default_value='true', choices=['true', 'false'],
            description='Also launch Gazebo + MoveIt2 simulation'),
        DeclareLaunchArgument(
            'robot_name', default_value='mycobot_280'),
        DeclareLaunchArgument(
            'use_camera', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument(
            'world_file', default_value='pick_and_place_demo.world',
            description='Gazebo world file (only used when with_sim:=true)'),
    ]

    with_sim = LaunchConfiguration('with_sim')

    # ── Simulation stack ───────────────────────────────────────────────────────
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'sim.launch.py')),
        launch_arguments={
            'robot_name': LaunchConfiguration('robot_name'),
            'use_camera': LaunchConfiguration('use_camera'),
            'world_file': LaunchConfiguration('world_file'),
        }.items(),
        condition=IfCondition(with_sim),
    )

    # ── Motion server — delayed when sim is starting up ────────────────────────
    # Gazebo + move_group take ~15-20 s on WSL2 with software rendering.
    # The 25 s delay gives the full stack time to reach a ready state before
    # MoveGroupInterface tries to connect.
    motion_server_delayed = TimerAction(
        period=25.0,
        actions=[
            LogInfo(msg='[motion_server] Starting motion action servers...'),
            Node(
                package='mycobot_motion',
                executable='motion_server_node',
                name='motion_server',
                output='screen',
                parameters=[params_file],
            ),
        ],
        condition=IfCondition(with_sim),
    )

    # When the sim is already running, start immediately.
    motion_server_immediate = Node(
        package='mycobot_motion',
        executable='motion_server_node',
        name='motion_server',
        output='screen',
        parameters=[params_file],
        condition=UnlessCondition(with_sim),
    )

    ld = LaunchDescription(args)
    ld.add_action(sim_launch)
    ld.add_action(motion_server_delayed)
    ld.add_action(motion_server_immediate)
    return ld
