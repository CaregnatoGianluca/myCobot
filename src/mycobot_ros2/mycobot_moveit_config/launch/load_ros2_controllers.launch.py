#!/usr/bin/env python3
"""
Launch ROS 2 controllers for the robot using spawner nodes.

Spawner nodes wait for the controller_manager to become available (up to
--controller-manager-timeout seconds) before activating, which makes this
robust to slow Gazebo startup in WSL2 / software rendering environments.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
        ],
        output='screen',
    )

    arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
        ],
        output='screen',
    )

    gripper_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'gripper_action_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '30',
        ],
        output='screen',
    )

    return LaunchDescription([
        joint_state_broadcaster,
        arm_controller,
        gripper_controller,
    ])
