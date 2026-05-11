# This is the launch file that starts up the basic QBot Platform nodes,
# plus the TF node. Then start the April Tag detection and localization.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, DeclareLaunchArgument)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (PathJoinSubstitution, LaunchConfiguration)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim = LaunchConfiguration('use_sim')

    qbot_platform_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('teleop'), 'launch', 'qbot_platform_launch.py')]
        )
    )

    qbot_platform_to_lidar_tf_node = Node(
            package='teleop',
            executable='fixed_lidar_frame',
            name='fixed_lidar_frame')

    # USB Camera node
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        parameters=[{
            'video_device': '/dev/video0',
            'image_width': 640,
            'image_height': 480,
            'pixel_format': 'yuyv',
            'camera_frame_id': 'camera_link',
            'io_method': 'mmap'
        }]
    )

    # AprilTag Pose Publisher node
    apriltag_pose_publisher_node = Node(
        package='teleop',
        executable='apriltag_pose_publisher',
        name='apriltag_pose_publisher'
    )

    # Robot Localization node to fuse odometry and April Tag poses
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[{
            'frequency': 30.0,
            'sensor_timeout': 0.1,
            'two_d_mode': True,
            'transform_time_offset': 0.0,
            'transform_timeout': 0.0,
            'print_diagnostics': True,
            'debug': False,
            'debug_out_file': '',
            'publish_tf': True,
            'publish_acceleration': False,
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_link_frame': 'base_link',
            'world_frame': 'odom',
            'odom0': '/odom',
            'odom0_config': [False, False, False,
                             False, False, False,
                             True, True, False,
                             False, False, True,
                             False, False, False],
            'odom0_queue_size': 10,
            'odom0_nodelay': False,
            'odom0_differential': False,
            'odom0_relative': False,
            'odom0_pose_rejection_threshold': 5.0,
            'odom0_twist_rejection_threshold': 1.0,
            'pose0': '/apriltag_bundle_poses',
            'pose0_config': [True, True, False,
                             False, False, False,
                             False, False, False,
                             False, False, False,
                             False, False, False],
            'pose0_queue_size': 10,
            'pose0_nodelay': False,
            'pose0_differential': False,
            'pose0_relative': False,
            'pose0_pose_rejection_threshold': 2.0,
            'pose0_twist_rejection_threshold': 0.5,
            'use_sim_time': use_sim
        }]
    )

    use_sim_la = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Start robot in Gazebo simulation')

    return LaunchDescription([
        qbot_platform_launch,
        use_sim_la,
        usb_cam_node,
        apriltag_node,
        apriltag_pose_publisher_node,
        robot_localization_node,
        qbot_platform_to_lidar_tf_node,
    ])