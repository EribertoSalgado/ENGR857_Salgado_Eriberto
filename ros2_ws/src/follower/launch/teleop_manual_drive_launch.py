# This is the launch file that starts up the basic QBot Platform nodes
import subprocess

from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, RegisterEventHandler, OpaqueFunction, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def start_lidar_node():
    # Assumes qbot_platform package is built in the workspace.
    return Node(
        package='qbot_platform',
        executable='lidar',
        name='lidar',
        output='screen'
    )


def exit_driver_cb(context):
    subprocess.run(
        'quarc_run -q -t tcpip://localhost:17000 qbot_platform_driver_physical',
        shell=True,
        capture_output=True
    )


def generate_launch_description():

    driver_model_rt_executable = PathJoinSubstitution(
        [FindPackageShare('follower'), 'rt_models',
         'qbot_platform_driver_physical.rt-linux_qbot_platform']
    )

    rt_model_start = ExecuteProcess(
        cmd=[
            'quarc_run',
            '-r -t tcpip://localhost:17000',
            driver_model_rt_executable,
            '-d %d -uri tcpip://%m:17001'
        ],
        name='QBotPlatformDriverModelStart',
        shell=True
    )

    driver_node = Node(
        package='follower',
        executable='qbot_platform_driver_interface',
        name='QBotPlatformDriver',
        parameters=[{'arm_robot': True}],
    )

    joystick_node = Node(
        package='follower',
        executable='command',
        name='JoystickCommands'
    )

    return LaunchDescription([
        rt_model_start,
        start_lidar_node(),
        joystick_node,

        RegisterEventHandler(
            OnProcessExit(
                target_action=driver_node,
                on_exit=[
                    OpaqueFunction(function=exit_driver_cb),
                    LogInfo(msg='Driver exiting. Stopping QUARC model.')
                ]
            )
        ),

        RegisterEventHandler(
            OnProcessStart(
                target_action=rt_model_start,
                on_start=[
                    LogInfo(msg='RT model started. Waiting 2 sec before starting driver...'),
                    TimerAction(
                        period=2.0,
                        actions=[driver_node]
                    )
                ]
            )
        )
    ])
