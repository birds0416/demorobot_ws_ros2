from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
import os
from ament_index_python.packages import get_package_share_directory

import os, sys
import signal

def signal_handler(sig, frame):
    print("Keyboard Interrupt (SIGINT) received. Terminating nodes...")
    sys.exit(0)

def generate_launch_description():

    package_name = 'demorobot_msgserver'
    package_dir = get_package_share_directory(package_name)

    config = os.path.join(
        package_dir, 'config', 'mqtt_params.yaml'
    )

    signal.signal(signal.SIGINT, signal_handler)

    return LaunchDescription([
        # Node(
        #     package=package_name,
        #     executable="action_server",
        #     output="screen",
        #     emulate_tty=True,
        #     parameters=[
        #         {"use_sim_time": True},
        #         {"is_stamped": True}
        #     ]
        # ),

        Node(
            package=package_name,
            executable="msg_sender",
            parameters=[
                config
            ]
        ),
        Node(
            package=package_name,
            executable="msg_receiver",
            parameters=[
                config
            ]
        ),
        
        # Node(
        #     package=package_name,
        #     executable="video_stream",
        #     output="screen",
        # )

        ExecuteProcess(
            cmd=["ros2", "run", "demorobot_msgserver", "video_stream"], output="screen"
        )
    ])