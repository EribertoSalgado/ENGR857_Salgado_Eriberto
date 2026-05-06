"""Teleop mode — manual driving."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_fake_robot = LaunchConfiguration('use_fake_robot')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_fake_robot',
            default_value='true',
            description='Launch the fake QBot.'
        ),

        Node(
            package='fake_qbot',
            executable='fake_qbot',
            name='fake_qbot',
            output='screen',
            condition=IfCondition(use_fake_robot),
        ),
    ])
