#!/usr/bin/env python3
"""Real robot entry point for the myCobot 280 Pi.

Unlike sim.launch.py (Gazebo + ros2_control), the real path does NOT use
ros2_control. Instead, mycobot_driver_node bridges MoveIt2 to the physical arm:
it serves the FollowJointTrajectory action that move_group calls and publishes
/joint_states, talking over TCP to mycobot_tcp_bridge.py on the robot's Pi.

Prerequisite: mycobot_tcp_bridge.py must be running on the robot's Raspberry Pi
(see mycobot_driver/scripts/). Start it there first, then launch this.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = [
        DeclareLaunchArgument('robot_name', default_value='mycobot_280'),
        DeclareLaunchArgument('use_rviz', default_value='true', choices=['true', 'false']),
    ]
    pkg_description = FindPackageShare('mycobot_description')
    pkg_moveit = FindPackageShare('mycobot_moveit_config')
    pkg_driver = FindPackageShare('mycobot_driver')

    # robot_state_publisher (no joint_state_publisher — the driver publishes
    # /joint_states from the real arm). use_gazebo/use_mock_hardware default false.
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([pkg_description, '/launch/robot_state_publisher.launch.py']),
        launch_arguments={
            'robot_name':        LaunchConfiguration('robot_name'),
            'use_gazebo':        'false',
            'use_mock_hardware': 'false',
            'use_sim_time':      'false',
            'use_rviz':          'false',
            'jsp_gui':           'false',
        }.items(),
    )

    # Real-robot driver: FollowJointTrajectory action + /joint_states, over TCP
    # to the Pi bridge. Replaces ros2_control_node + controllers for the real robot.
    driver_params = PathJoinSubstitution([pkg_driver, 'config', 'driver_params.yaml'])
    driver_node = Node(
        package='mycobot_driver',
        executable='mycobot_driver_node',
        name='mycobot_driver',
        output='screen',
        parameters=[driver_params],
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
    ld.add_action(driver_node)
    ld.add_action(moveit_launch)
    return ld
