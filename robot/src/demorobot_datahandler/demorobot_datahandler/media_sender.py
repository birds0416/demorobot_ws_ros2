import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

from cv_bridge import CvBridge
import cv2

class MediaSender(Node):
    def __init__(self):
        super().__init__('MediaSender')

        