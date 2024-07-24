from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

import os, sys
import signal

def signal_handler(sig, frame):
    print("Keyboard Interrupt (SIGINT) received. Terminating nodes...")
    sys.exit(0)

def generate_launch_description():
    package_name = 'demorobot_ekf'
    package_dir = get_package_share_directory(package_name)
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    robot_localization_file_dir = os.path.join(package_dir, 'config', 'ekf.yaml')
    
    signal.signal(signal.SIGINT, signal_handler)
    
    return LaunchDescription({
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                robot_localization_file_dir, 
                {'use_sim_time': use_sim_time}
            ]
        ),
    })