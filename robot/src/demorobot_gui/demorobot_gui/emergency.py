import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, String
from sensor_msgs.msg import Image
from demorobot_msg.msg import BatteryStat

from scripts.emer_gui import *

import numpy as np
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

class EmergencyMode(Node):
    def __init__(self):
        super().__init__('emergency_mode')
        self.NAMESPACE = self.get_namespace()

        self.ros2_worker = ROS2Worker(self)
        self.ros2_worker.start()

        #region pose detect img
        self.pose_image = np.empty(shape=[1])
        # self.sub_pose_detect = self.create_subscription(Image, self.NAMESPACE + '/pose_detect/detect_img', self.pose_detect_callback, 10)
        self.sub_pose_detect = self.create_subscription(Image, '/device01/pose_detect/detect_img', self.pose_detect_callback, 10)
        #endregion pose detect img

        self.app = QApplication(sys.argv)
        self.gui = EmerWindow()
        # self.detect_thread.change_pixmap_signal.connect(self.gui.update_img)
        self.show_image(self.pose_image)

    def show_image(self, img):
        # self.gui.detection_img_btn.clicked.connect(lambda: self.get_cv_img(None))
        # if img == None:
        #     self.gui.detection_img_btn.clicked.connect(lambda: self.get_cv_img(None))
        # else:
        self.detect_thread = VideoThread(img)
        self.gui.detection_img_btn.clicked.connect(lambda: self.get_cv_img(img))
        self.detect_thread.start()
        self.gui.show()

    def get_cv_img(self, img):
        # if img == None:
        #     QMessageBox.warning(self.gui, 'Detect Image Error', "감지 이미지 접속 불가")
        # else:
        self.detect_thread.change_pixmap_signal.connect(lambda: self.gui.update_img(img))

    def pose_detect_callback(self, msg):
        self.pose_image = bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.encoding = msg.encoding

def main(args=None):
    rclpy.init(args=args)
    emergency_mode = EmergencyMode()
    sys.exit(emergency_mode.app.exec_())

if __name__ == '__main__':
    main()