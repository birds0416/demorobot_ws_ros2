import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

from cv_bridge import CvBridge
import cv2

class MediaSender(Node):
    def __init__(self):
        super().__init__('MediaSender')

def main(args=None):
    rclpy.init(args=args)
    mediasender = MediaSender()
    rclpy.spin(mediasender)
    mediasender.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
