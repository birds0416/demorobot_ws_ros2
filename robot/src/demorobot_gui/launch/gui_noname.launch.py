import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import GroupAction, TimerAction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    package_name = 'demorobot_gui'
    package_dir = get_package_share_directory(package_name)

    return LaunchDescription({
        GroupAction(
            actions=[
                Node(
                    package='demorobot_nav',
                    executable='robot_navigator',
                    parameters=[
                        {"use_sim_time": True},
                        {"is_stamped": True},
                    ]
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
    })
