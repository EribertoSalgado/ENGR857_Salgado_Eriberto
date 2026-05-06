"""Bringup for the QBot Delivery Bot system."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('delivery_bot_bringup')
    default_config = os.path.join(pkg_share, 'config', 'qbot_params.yaml')

    use_fake_robot = LaunchConfiguration('use_fake_robot')
    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_fake_robot',
            default_value='true',
            description='Launch the fake QBot.'
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Path to parameter YAML.'
        ),

        Node(
            package='fake_qbot',
            executable='fake_qbot',
            name='fake_qbot',
            output='screen',
            condition=IfCondition(use_fake_robot),
        ),
        Node(
            package='delivery_bot',
            executable='state_machine',
            name='delivery_bot_state_machine',
            output='screen',
            parameters=[config_file],
        ),
        Node(
            package='delivery_bot_intent',
            executable='intent_communication',
            name='intent_communication',
            output='screen',
            parameters=[config_file],
        ),
    ])
