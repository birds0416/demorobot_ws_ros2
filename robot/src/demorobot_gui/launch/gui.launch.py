import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import GroupAction

def generate_launch_description():
    package_name = 'demorobot_gui'
    package_dir = get_package_share_directory(package_name)

    _ns = LaunchConfiguration('_ns', default='device00')
    config = os.path.join(
        package_dir, 'config', 'location.yaml'
    )

    return LaunchDescription({
        GroupAction(
            actions=[
                PushRosNamespace(_ns),
                # Node(
                #     package=package_name,
                #     executable='robot_main',
                #     emulate_tty=True,
                #     parameters=[
                #         {"use_sim_time": True},
                #         {"is_stamped": True},
                #         {"namespace": _ns}
                #     ]
                # )
                Node(
                    package=package_name,
                    executable='normal_mode',
                    parameters=[
                        {"use_sim_time": True},
                        {"is_stamped": True},
                        {"namespace": _ns},
                        config
                    ]
                )
            ]
        )
    })