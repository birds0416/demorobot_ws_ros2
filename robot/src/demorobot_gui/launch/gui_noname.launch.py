from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import GroupAction, TimerAction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os, sys
import signal

def signal_handler(sig, frame):
    print("Keyboard Interrupt (SIGINT) received. Terminating nodes...")
    sys.exit(0)

def generate_launch_description():
    package_name = 'demorobot_gui'
    package_dir = get_package_share_directory(package_name)
    
    signal.signal(signal.SIGINT, signal_handler)

    return LaunchDescription({
        GroupAction(
            actions=[
                Node(
                    package='demorobot_nav',
                    executable='robot_navigator',
                ),
                # Node(
                #     package='demorobot_batterystatus',
                #     executable='battery_status',
                #     parameters=[
                #         {"use_sim_time": True},
                #         {"is_stamped": True},
                #     ]
                # ),
                Node(
                    package=package_name,
                    executable='normal_mode',
                    parameters=[
                        {"use_sim_time": True},
                        {"is_stamped": True},
                    ]
                )
            ]
        )
        # TimerAction(
        #     period=5.0,
        #     actions=[
        #         Node(
        #             package='demorobot_nav',
        #             executable='robot_navigator',
        #         ),
        #         Node(
        #             package='demorobot_nav',
        #             executable='test_navigator'
        #         )
        #     ]
        # )
    })
