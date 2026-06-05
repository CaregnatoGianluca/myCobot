#!/usr/bin/env python3
"""Simulation entry point for the myCobot 280."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = [
        DeclareLaunchArgument('robot_name', default_value='mycobot_280'),
        DeclareLaunchArgument('use_camera', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('use_rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('world_file', default_value='pick_and_place_demo.world',
                              description='World file inside mycobot_gazebo/worlds/'),
    ]
    pkg_gazebo = FindPackageShare('mycobot_gazebo')
    pkg_moveit = FindPackageShare('mycobot_moveit_config')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([pkg_gazebo, '/launch/mycobot.gazebo.launch.py']),
        launch_arguments={
            'robot_name':       LaunchConfiguration('robot_name'),
            'use_camera':       LaunchConfiguration('use_camera'),
            'use_gazebo':       'true',
            'use_sim_time':     'true',
            'use_rviz':         'false',
            'load_controllers': 'true',
            'world_file':       LaunchConfiguration('world_file'),
        }.items(),
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([pkg_moveit, '/launch/move_group.launch.py']),
        launch_arguments={
            'robot_name':   LaunchConfiguration('robot_name'),
            'use_sim_time': 'true',
            'use_rviz':     'true',
        }.items(),
    )

    ld = LaunchDescription(args)
    ld.add_action(gazebo_launch)
    ld.add_action(moveit_launch)
    return ld
