import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

from cv_bridge import CvBridge
import cv2

# 로봇 본체로부터 이벤트 이미지 수신 노드
class MediaReceiver(Node):
    def __init__(self):
        super().__init__('MediaReceiver')

        self.sub_evtimg = self.create_subscription(Image, 'save_img', self.evtimg_callback, 10)
        self.sub_evtimg_empty = self.create_subscription(Image, 'save_img', self.evtimg_empty_callback, 10)
    
    def evtimg_callback(self, msg):
        bridge = CvBridge()
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 이미지 번호를 시간으로 입력하게끔 변경
        cv2.imwrite('./evt_imgs/event_img_{}.jpg'.format(0), cv_img)
        self.get_logger().info("Event Image {} Save SUCCESS".format('event_img_{}.jpg'.format(0)))

    def evtimg_empty_callback(self, msg):
        bridge = CvBridge()
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 이미지 번호를 시간으로 입력하게끔 변경
        cv2.imwrite('./evt_imgs/event_img_empty_{}.jpg'.format(0), cv_img)
        self.get_logger().info("Event Image {} Save SUCCESS".format('event_img_empty_{}.jpg'.format(0)))

def main(args=None):
    rclpy.init(args=args)
    media_receiver = MediaReceiver()
    rclpy.spin(media_receiver)
    media_receiver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()