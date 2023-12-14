from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    package_name = 'demorobot_msgserver'
    package_dir = get_package_share_directory(package_name)

    config = os.path.join(
        package_dir, 'config', 'mqtt_params.yaml'
    )

    return LaunchDescription([

        Node(
            package=package_name,
            executable="msg_sender",
            emulate_tty=True,
            parameters=[
                {"use_sim_time": True},
                {"is_stamped": True},
                config
            ]
        )
    ])