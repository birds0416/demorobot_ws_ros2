import rclpy
from rclpy.node import Node
from rclpy.subscription import Subscription

from sensor_msgs.msg import Image
from std_msgs.msg import Int32, Int32MultiArray

from cv_bridge import CvBridge, CvBridgeError
import cv2

def gen_namespace(num):
    if num < 10:
        return 'device0' + str(num)
    else:
        return 'device' + str(num)

class VideoStream(Node):
    def __init__(self):
        super().__init__('VideoStream')

        self.sub_current_device = self.create_subscription(Int32MultiArray, '/current_device_nums', self.device_list_callback, 10)
        self.image_subscribers = {}

        self.bridge = CvBridge()
        # self.sub_image_raw = self.create_subscription(Image, '/camera/color/image_raw', self.stream_callback, 10)
    
    def device_list_callback(self, msg):
        for num in msg.data:
            namespace = gen_namespace(num)

            if namespace not in self.image_subscribers:
                self.image_subscribers[namespace] = self.create_subscription(
                    Image,
                    f"/{namespace}/camera/color/image_raw",
                    lambda msg, topic_name=f"/{namespace}/camera/color/image_raw": self.stream_callback(msg, topic_name),
                    10
                )
    
    def stream_callback(self, msg, topic_name):
        namespace = topic_name.split('/')[1]
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imshow(f'Image Window - {namespace}', cv_img)
            cv2.waitKey(1)

        except CvBridgeError as e:
            self.get_logger().error(e)

def main(args=None):
    rclpy.init(args=args)
    stream_node = VideoStream()
    rclpy.spin(stream_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()