import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import GroupAction

def generate_launch_description():
    package_name = 'demorobot_depth'
    package_dir = get_package_share_directory(package_name)

    # library_ws = '/home/jetson/cornersdev/yahboomcar_ros2_ws/software/library_ws'
    # astra_camera_launch = os.path.join(library_ws, 'install', 'astra_camera', 'share', 'astra_camera', 'launch', 'astro_pro_plus.launch.xml')

    posedetect_dir = get_package_share_directory('demorobot_posedetect')

    _ns = LaunchConfiguration('_ns', default='device00')

    # _ns_launch_arg = DeclareLaunchArgument(
    #     '_ns',
    #     default_value='device00'
    # )

    return LaunchDescription({

        GroupAction(
            actions=[
                PushRosNamespace(_ns),
                IncludeLaunchDescription(
                    XMLLaunchDescriptionSource(
                        # astra_camera_launch
                        os.path.join(get_package_share_directory('astra_camera'), 'launch/astro_pro_plus.launch.xml')
                    )
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        posedetect_dir + '/launch/posedetect.launch.py'
                    )
                )
            ]
        )
    })