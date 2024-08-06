import os, sys
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import GroupAction, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import signal

def signal_handler(sig, frame):
    print("Keyboard Interrupt (SIGINT) received. Terminating nodes...")
    sys.exit(0)

def generate_launch_description():
    package_name = 'demorobot_servergui'
    package_dir = get_package_share_directory(package_name)

    msgserver_launch_dir = get_package_share_directory('demorobot_msgserver')

    return LaunchDescription({
        GroupAction(
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        msgserver_launch_dir + '/launch/msg_server.launch.py'
                    )
                ),
                Node(
                    package=package_name,
                    executable='server_gui',
                )
            ]
        )
    })