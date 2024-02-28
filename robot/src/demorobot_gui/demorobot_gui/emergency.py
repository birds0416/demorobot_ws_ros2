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
        # self.pose_image = np.empty(shape=[1/])
        # self.sub_pose_detect = self.create_subscription(Image, self.NAMESPACE + '/pose_detect/detect_img', self.pose_detect_callback, 10)
        self.sub_pose_detect = self.create_subscription(Image, '/device01/pose_detect/detect_img', self.pose_detect_callback, 10)
        # self.sub_depth_frame = self.create_subscription(Image, self.NAMESPACE + '/pose_detect/detect_img', self.depth_frame_callback, 10)
        self.sub_depth_frame = self.create_subscription(Image, '/device01/pose_detect/depth_frame', self.depth_frame_callback, 10)
        #endregion pose detect img

        self.app = QApplication(sys.argv)
        self.gui = EmerWindow()
        self.gui.detection_rgb_img_btn.clicked.connect(lambda: self.toggle_video_feed(1))
        self.gui.detection_depth_img_btn.clicked.connect(lambda: self.toggle_video_feed(2))
        self.gui.show()

        self.rgb_video_playing = False
        self.depth_video_playing = False

    def pose_detect_callback(self, msg):
        if self.rgb_video_playing:
            self.pose_image = bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.gui.show_rgb_img(self.pose_image)

    def depth_frame_callback(self, msg):
        if self.depth_video_playing:
            self.depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.gui.show_depth_img(self.depth_image)

    def toggle_video_feed(self, num):
        # if self.pose_image == None:
        #     QMessageBox.warning(self.gui, 'Detect Image Error', "감지 이미지 접속 불가")
        
        if num == 1:
            # 아무것도 재생중이지 않을 때
            if self.rgb_video_playing == False and self.depth_video_playing == False:
                self.gui.rgb_img_label.show()
                self.gui.detection_rgb_img_btn.setText('Stop')

            # rgb 재생중일때 끄기
            elif self.rgb_video_playing and self.depth_video_playing == False:
                self.gui.rgb_img_label.hide()
                self.gui.detection_rgb_img_btn.setText('감지 이미지 보기')
            
            # depth 재생중일때 끄고 rgb 재생
            elif self.rgb_video_playing == False and self.depth_video_playing:
                self.gui.depth_img_label.hide()
                self.gui.detection_depth_img_btn.setText('Depth 이미지 보기')
                self.gui.rgb_img_label.show()
                self.gui.detection_rgb_img_btn.setText('Stop')
                self.depth_video_playing = not self.depth_video_playing

            self.rgb_video_playing = not self.rgb_video_playing
        
        elif num == 2:
            # 아무것도 재생중이지 않을 때
            if self.rgb_video_playing == False and self.depth_video_playing == False:
                self.gui.depth_img_label.show()
                self.gui.detection_depth_img_btn.setText('Stop')
            
            # depth 재생중일때 끄기
            elif self.rgb_video_playing == False and self.depth_video_playing:
                self.gui.depth_img_label.hide()
                self.gui.detection_depth_img_btn.setText('Depth 이미지 보기')

            # rgb 재생중일때 끄고 depth 재생
            elif self.rgb_video_playing and self.depth_video_playing == False:
                self.gui.rgb_img_label.hide()
                self.gui.detection_rgb_img_btn.setText('감지 이미지 보기')
                self.gui.depth_img_label.show()
                self.gui.detection_depth_img_btn.setText('Stop')
                self.rgb_video_playing = not self.rgb_video_playing
            
            self.depth_video_playing = not self.depth_video_playing

def main(args=None):
    rclpy.init(args=args)
    emergency_mode = EmergencyMode()
    sys.exit(emergency_mode.app.exec_())

if __name__ == '__main__':
    main()