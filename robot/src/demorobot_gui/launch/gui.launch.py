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

    yahboomcar_ws = '/home/jetson/cornersdev/yahboomcar_ros2_ws/yahboomcar_ws'
    laser_bringup_launch = os.path.join(yahboomcar_ws, 
        'install', 
        'yahboomcar_nav', 
        'share', 
        'yahboomcar_nav', 
        'launch', 
        'laser_bringup_launch.py'
    )

    display_nav_launch = os.path.join(yahboomcar_ws,
        'install', 
        'yahboomcar_nav', 
        'share', 
        'yahboomcar_nav', 
        'launch',
        'display_nav_launch.py'
    )

    navigation_dwa_launch = os.path.join(yahboomcar_ws, 
        'install', 
        'yahboomcar_nav', 
        'share', 
        'yahboomcar_nav', 
        'launch', 
        'navigation_dwa_launch.py'
    )

    _ns = LaunchConfiguration('_ns', default='device00')
    # config = os.path.join(
    #     package_dir, 'config', 'location.yaml'
    # )

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
                TimerAction(
                    # 초 단위 설정
                    period = 1.0,
                    actions=[
                        IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(
                                laser_bringup_launch
                            )
                        ),
                        # IncludeLaunchDescription(
                        #     PythonLaunchDescriptionSource(
                        #         display_nav_launch
                        #     )
                        # ),
                        IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(
                                navigation_dwa_launch
                            )
                        ),
                        Node(
                            package='demorobot_nav',
                            executable='robot_navigator',
                            parameters=[
                                {"use_sim_time": True},
                                {"is_stamped": True},
                                {"namespace": _ns},
                            ]
                        )
                    ]
                ),
                Node(
                    package=package_name,
                    executable='normal_mode',
                    parameters=[
                        {"use_sim_time": True},
                        {"is_stamped": True},
                        {"namespace": _ns},
                    ]
                )
            ]
        )
    })