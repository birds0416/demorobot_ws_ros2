import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, String
from sensor_msgs.msg import Image
from demorobot_msg.msg import BatteryStat

from scripts.emer_gui import *

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

class EmergencyMode(Node):
    def __init__(self):
        super().__init__('emergency_mode')
        self.NAMESPACE = self.get_namespace()

        self.ros2_worker = ROS2Worker(self)
        self.ros2_worker.start()

        self.rgb_video_playing = False
        self.depth_video_playing = False

        #region pose detect img
        self.pose_image = np.empty(shape=[1])
        self.sub_pose_detect = self.create_subscription(Image, self.NAMESPACE + '/pose_detect/detect_img', self.pose_detect_callback, 10)
        self.sub_depth_frame = self.create_subscription(Image, self.NAMESPACE + '/pose_detect/depth_frame', self.depth_frame_callback, 10)
        #endregion pose detect img

        #region EVT subscribe
        self.sub_fall_evt = self.create_subscription(String, self.NAMESPACE + '/pose_detect/event', self.evt_callback, 10)
        #endregion EVT subscribe

        self.app = QApplication(sys.argv)
        self.gui = EmerWindow()
        self.gui.detection_rgb_img_btn.clicked.connect(lambda: self.toggle_video_feed(1))
        self.gui.detection_depth_img_btn.clicked.connect(lambda: self.toggle_video_feed(2))
        self.gui.show()

    def pose_detect_callback(self, msg):
        if self.rgb_video_playing:
            self.pose_image = bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.gui.show_rgb_img(self.pose_image)

    def depth_frame_callback(self, msg):
        if self.depth_video_playing:
            self.depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.gui.show_depth_img(self.depth_image)

    def evt_callback(self, msg):
        if msg.data == "EVT-001" or msg.data == "EVT-002":
            self.gui.person_detect_msg.setText("요구조자가 발견되었습니다.")
            time.sleep(1)
        self.gui.person_detect_msg.setText("요구조자가 위치를 전달중입니다.")
        # TODO - send person location to server

    def toggle_video_feed(self, num):
        ''' rgb, depth 이미지 '''
        if num == 1:
            if self.rgb_video_playing:
                self.gui.rgb_img_label.hide()
                self.gui.detection_rgb_img_btn.setText('감지 이미지 보기')
            else:
                self.gui.rgb_img_label.show()
                self.gui.detection_rgb_img_btn.setText('Stop')
                # self.gui.detection_depth_img_btn.setText('Depth 이미지 보기' if self.depth_video_playing else 'Stop')
                if self.depth_video_playing:
                    self.gui.depth_img_label.hide()
                    self.gui.detection_depth_img_btn.setText('Depth 이미지 보기')
                    self.depth_video_playing = not self.depth_video_playing
                    
            self.rgb_video_playing = not self.rgb_video_playing
        
        elif num == 2:
            if self.depth_video_playing:
                self.gui.depth_img_label.hide()
                self.gui.detection_depth_img_btn.setText('Depth 이미지 보기')
            else:
                self.gui.depth_img_label.show()
                self.gui.detection_depth_img_btn.setText('Stop')
                # self.gui.detection_rgb_img_btn.setText('감지 이미지 보기' if self.rgb_video_playing else 'Stop')
                if self.rgb_video_playing:
                    self.gui.rgb_img_label.hide()
                    self.gui.detection_rgb_img_btn.setText('감지 이미지 보기')
                    self.rgb_video_playing = not self.rgb_video_playing
            
            self.depth_video_playing = not self.depth_video_playing

def main(args=None):
    rclpy.init(args=args)
    emergency_mode = EmergencyMode()
    sys.exit(emergency_mode.app.exec_())

if __name__ == '__main__':
    main()