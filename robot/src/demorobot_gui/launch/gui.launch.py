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
    
    namespace = LaunchConfiguration('namespace', default='device00')
    
    #region yahboom launch descriptions
    # yahboomcar_ws = '/home/jetson/cornersdev/yahboomcar_ros2_ws/yahboomcar_ws'
    # laser_bringup_launch = os.path.join(yahboomcar_ws, 
    #     'install', 
    #     'yahboomcar_nav', 
    #     'share', 
    #     'yahboomcar_nav', 
    #     'launch', 
    #     'laser_bringup_launch.py'
    # )

    # display_nav_launch = os.path.join(yahboomcar_ws,
    #     'install', 
    #     'yahboomcar_nav', 
    #     'share', 
    #     'yahboomcar_nav', 
    #     'launch',
    #     'display_nav_launch.py'
    # )

    # navigation_dwa_launch = os.path.join(yahboomcar_ws, 
    #     'install', 
    #     'yahboomcar_nav', 
    #     'share', 
    #     'yahboomcar_nav', 
    #     'launch', 
    #     'navigation_dwa_launch.py'
    # )
    
    # return LaunchDescription({
    #     GroupAction(
    #         actions=[
    #             # PushRosNamespace(namespace),
    #             TimerAction(
    #                 # 초 단위 설정
    #                 period = 1.0,
    #                 actions = [
    #                     # PushRosNamespace(namespace),
    #                     IncludeLaunchDescription(
    #                         PythonLaunchDescriptionSource(
    #                             laser_bringup_launch
    #                         ),
    #                         launch_arguments={'namespace': namespace}.items(),
    #                     )
    #                 ]
    #             ),
    #             # TimerAction(
    #             #     # 초 단위 설정
    #             #     period = 1.0,
    #             #     actions=[
    #             #         PushRosNamespace(namespace),
    #             #         IncludeLaunchDescription(
    #             #             PythonLaunchDescriptionSource(
    #             #                 display_nav_launch
    #             #             )
    #             #         )
    #             #     ]
    #             # ),
    #             TimerAction(
    #                 # 초 단위 설정
    #                 period = 2.0,
    #                 actions = [
    #                     # PushRosNamespace(namespace),
    #                     IncludeLaunchDescription(
    #                         PythonLaunchDescriptionSource(
    #                             navigation_dwa_launch
    #                         ),
    #                         launch_arguments={'namespace': namespace}.items(),
    #                     )
    #                 ]
    #             ),
    #             TimerAction(
    #                 period = 1.0,
    #                 actions = [
    #                     # PushRosNamespace(namespace),
    #                     Node(
    #                         package='demorobot_nav',
    #                         executable='robot_navigator',
    #                         parameters=[
    #                             {"use_sim_time": True},
    #                             {"is_stamped": True},
    #                             {"namespace": namespace}
    #                         ]
    #                     )
    #                 ]
    #             ),
    #             TimerAction(
    #                 period = 1.0,
    #                 actions = [
    #                     # PushRosNamespace(namespace),
    #                     Node(
    #                         package=package_name,
    #                         executable='normal_mode',
    #                         parameters=[
    #                             {"use_sim_time": True},
    #                             {"is_stamped": True},
    #                             {"namespace": namespace}
    #                         ]
    #                     )
    #                 ]
    #             ),
    #         ]
    #     )
    # })
    #endregion

    return LaunchDescription({
        GroupAction(
            actions=[
                PushRosNamespace(namespace),
                Node(
                    package='demorobot_nav',
                    executable='robot_navigator',
                ),
                Node(
                    package=package_name,
                    executable='normal_mode',
                )
            ]
        )
    })
