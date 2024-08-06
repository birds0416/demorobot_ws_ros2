import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from rclpy.qos import qos_profile_services_default

from std_msgs.msg import Int8, String, Bool
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Pose, PoseWithCovarianceStamped, PoseWithCovariance, Point

from .src.main_gui import *
import json

import numpy as np
import time
import cv2
from cv_bridge import CvBridge

bridge = CvBridge()

class ROS2Worker(QThread):
    def __init__(self, node):
        super().__init__()
        self.node = node

    def run(self):
        rclpy.spin(self.node)  # Run ROS2 spin in this thread
        rclpy.shutdown()  # Shutdown ROS2 node when thread finishes

class ServerGUI(Node):
    def __init__(self):
        super().__init__('server_gui')

        self.ros2_worker = ROS2Worker(self)
        self.ros2_worker.start()

        #region pose detect img
        self.pose_image = np.empty(shape=[1])
        self.sub_pose_detect = self.create_subscription(Image, '/pose_detect/detect_img', self.pose_detect_callback, 10)
        self.sub_depth_frame = self.create_subscription(Image, '/pose_detect/depth_frame', self.depth_frame_callback, 10)
        #endregion pose detect img

        #region EVT subscribe
        # self.sub_fall_evt = self.create_subscription(String, '/pose_detect/event', self.evt_callback, 10)
        #endregion EVT subscribe

        self.app = QApplication(sys.argv)
        self.gui = MainWindow()
        self.gui.show()

    def pose_detect_callback(self, msg):
        self.pose_image = bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.gui.show_rgb_img(self.pose_image)

    def depth_frame_callback(self, msg):
        self.depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.gui.show_depth_img(self.depth_image)

def main(args=None):
    rclpy.init(args=args)
    server_gui = ServerGUI()
    sys.exit(server_gui.app.exec_())

if __name__ == '__main__':
    main()