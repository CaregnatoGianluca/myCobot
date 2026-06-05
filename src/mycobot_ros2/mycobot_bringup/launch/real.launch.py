#!/usr/bin/env python3
"""Real robot entry point for the myCobot 280 Raspberry Pi."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = [
        DeclareLaunchArgument('robot_name', default_value='mycobot_280'),
        DeclareLaunchArgument('use_mock_hardware', default_value='false',
                              choices=['true', 'false'],
                              description='true = mock hardware for offline dev; false = real serial'),
        DeclareLaunchArgument('use_rviz', default_value='true', choices=['true', 'false']),
    ]
    pkg_description = FindPackageShare('mycobot_description')
    pkg_moveit = FindPackageShare('mycobot_moveit_config')

    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([pkg_description, '/launch/robot_state_publisher.launch.py']),
        launch_arguments={
            'robot_name':        LaunchConfiguration('robot_name'),
            'use_gazebo':        'false',
            'use_mock_hardware': LaunchConfiguration('use_mock_hardware'),
            'use_sim_time':      'false',
            'use_rviz':          'false',
            'jsp_gui':           'false',
        }.items(),
    )

    controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([pkg_moveit, '/launch/load_ros2_controllers.launch.py']),
        launch_arguments={'use_sim_time': 'false'}.items(),
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([pkg_moveit, '/launch/move_group.launch.py']),
        launch_arguments={
            'robot_name':   LaunchConfiguration('robot_name'),
            'use_sim_time': 'false',
            'use_rviz':     'true',
        }.items(),
    )

    ld = LaunchDescription(args)
    ld.add_action(rsp_launch)
    ld.add_action(controllers_launch)
    ld.add_action(moveit_launch)
    return ld
