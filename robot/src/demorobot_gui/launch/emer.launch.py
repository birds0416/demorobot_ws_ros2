import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import GroupAction, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    package_name = 'demorobot_gui'
    package_dir = get_package_share_directory(package_name)
    
    camera_launch_dir = get_package_share_directory('demorobot_depth')

    namespace = LaunchConfiguration('namespace', default='device00')

    return LaunchDescription({
        GroupAction(
            actions=[
                PushRosNamespace(namespace),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        camera_launch_dir + '/launch/camera.launch.py'
                    )
                ),
                Node(
                    package=package_name,
                    executable='emergency_mode',
                )
            ]
        )
    })
