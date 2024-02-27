import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, String
from sensor_msgs.msg import Image
from demorobot_msg.msg import BatteryStat
from scripts.main_gui import *
from scripts.emer_gui import *

from normal import *
from emergency import *

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
        subprocess.run(["ros2", "launch", self.package, self.launch_file, '_ns:=' + self.namespace])

class RobotMain(Node):
    def __init__(self):
        super().__init__('robot_main')
        self.NAMESPACE = self.get_namespace()

        #region robot_mode
        self.sub_robot_mode = self.create_subscription(Int8, '/server_msg/drive_mode', self._callback, 10)
        # 0: normal, 1: emergency
        self.robot_mode = 0
        self.robot_sub_mode = 0
        #endregion robot_mode

        #region battery status
        self.sub_robot_battery = self.create_subscription(BatteryStat, self.NAMESPACE + '/battery_stat', self.bat_stat_callback, 10)
        self.robot_battery = 0
        self.robot_battery_mode = False
        #endregion battery status

        #region pose detect img
        self.pose_image = np.empty(shape=[1])
        self.sub_pose_detect = self.create_subscription(Image, self.NAMESPACE + '/pose_detect/detect_img', self.pose_detect_callback, 10)
        #endregion pose detect img
    
    def _callback(self, msg):
        self.robot_mode = msg.data.vols

    def bat_stat_callback(self, msg):
        self.robot_battery = msg.vol
        self.robot_battery_mode = msg.mode

    def pose_detect_callback(self, msg):
        self.pose_image = bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.encoding = msg.encoding

def main(args=None):
    rclpy.init(args=args)
    robot_main = RobotMain()
    robot_thread = RobotThread(robot_main)
    robot_thread.start()

    '''서버로 디바이스 ID 전송'''
    ns_action_thread  = LaunchThread("demorobot_datahandler", "datahandler.launch.py", robot_main.NAMESPACE)
    ns_action_thread.start()

    # normal_mode = NormalMode()
    # emergency_mode = EmergencyMode()

    # app = QApplication(sys.argv)
    # gui = MainWindow()
    # gui.bat_percent = robot_main.robot_battery

    # if robot_main.robot_mode == 0:
    #     # TODO
    #     gui = MainWindow()
    #     # nav_launch_thread = LaunchThread("demorobot_nav", "nav.launch.py", robot_main.NAMESPACE)
    #     # nav_launch_thread.start()
    # elif robot_main.robot_mode == 1:
    #     # initial thread is posedetect fall
    #     gui = EmerWindow()
    #     pose_launch_thread = LaunchThread("demorobot_posedetect", "posedetect.launch.py", robot_main.NAMESPACE)
    #     pose_launch_thread.start()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()