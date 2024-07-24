import rclpy
from rclpy.node import Node
from rclpy.qos import QosProfile

from std_msgs.msg import Int8, String
from sensor_msgs.msg import Image
from demorobot_msg.msg import BatteryStat
from .src.main_gui import *
from .src.emer_gui import *
from .src.parent_gui import *

import normal as ROS_Normal
import emergency as ROS_Emergency

import sys
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
import subprocess
import threading

import numpy as np
import cv2
from cv_bridge import CvBridge

bridge = CvBridge()

class RobotThread(threading.Thread):
    def __init__(self, robot_node):
        super().__init__()
        self.robot_node = robot_node
    
    def run(self):
        rclpy.spin(self.robot_node)

class LaunchThread(threading.Thread):
    def __init__(self, package, launch_file, namespace):
        super().__init__()
        self.package = package
        self.launch_file = launch_file
        self.namespace = namespace
    
    def run(self):
        subprocess.run(["ros2", "launch", self.package, self.launch_file, 'namespace:=' + self.namespace])

class RobotMain(Node):
    def __init__(self):
        super().__init__('robot_main')
        self.NAMESPACE = self.get_namespace()
        if self.NAMESPACE == "/":
            self.NAMESPACE = ""

        #region robot_mode
        self.sub_robot_mode = self.create_subscription(Int8, self.NAMESPACE + '/server_msg/drive_mode', self._callback, 10)
        # 0: normal, 1: emergency
        self.robot_mode = 0
        self.robot_sub_mode = 0
        #endregion robot_mode

        #region battery status
        self.sub_robot_battery = self.create_subscription(BatteryStat, self.NAMESPACE + '/battery_stat', self.bat_stat_callback, 10)
        self.robot_battery = 0
        self.robot_battery_mode = False
        #endregion battery status
        
        #region gui mode
        self.sub_gui_mode = self.create_subscription(Int8, self.NAMESPACE + '/server_msg/drive_mode', self.gui_mode_callback, 10)
        
        #endregion gui mode
        
        # main, emer gui 통합
        self.app = QApplication(sys.argv)
        self.gui = ParentGUI()
    
    def _callback(self, msg):
        self.robot_mode = msg.data

    def bat_stat_callback(self, msg):
        self.robot_battery = msg.vol
        self.robot_battery_mode = msg.mode
        
    def gui_mode_callback(self, msg):
        pass

def main(args=None):
    rclpy.init(args=args)
    robot_main = RobotMain()
    robot_thread = RobotThread(robot_main)
    robot_thread.start()

    '''서버로 디바이스 ID 전송'''
    # ns_action_thread  = LaunchThread("demorobot_datahandler", "datahandler.launch.py", robot_main.NAMESPACE)
    # ns_action_thread.start()

    normal_mode = ROS_Normal.NormalMode()
    emergency_mode = ROS_Emergency.EmergencyMode()
    
    sys.exit(robot_main.app.exec_())

if __name__ == '__main__':
    main()