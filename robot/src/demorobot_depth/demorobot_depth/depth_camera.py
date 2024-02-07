import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
from std_msgs.msg import Float32
from demorobot_msg.msg import Detect

from cv_bridge import CvBridge
import datetime
import numpy as np
import math

class DepthCamera(Node):
    def __init__(self):
        super().__init__('DepthCamera')

        self.NAMESPACE = self.get_namespace()
        self.get_logger().info("Namespace: {}".format(self.NAMESPACE))

        self.sub_depth_img = self.create_subscription(Image, self.NAMESPACE + '/camera/depth/image_raw', self.depth_image_callback, 10)
        self.sub_detect_box = self.create_subscription(Detect, self.NAMESPACE + '/pose_detect/detect_points', self.detect_points_callback, 10)
        # self.sub_depth_points = self.create_subscription(PointCloud2, self.NAMESPACE + '/camera/depth/points', self.depth_points_callback, 10)

        self.pub_center_d = self.create_publisher(Float32, self.NAMESPACE + '/depth/center_distance', 10)
        self.pub_box_d = self.create_publisher(Float32, self.NAMESPACE + '/depth/box_distance', 10)
        self.img_values = []

        self.bridge = CvBridge()
    
    def depth_image_callback(self, msg):
        # Version 1 -> get center pixel value
        depths = msg.data

        ''' Version 3'''
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        h, w = cv_img.shape
        self.img_values = np.zeros((h, w), dtype=np.uint16)
        self.img_values = cv_img.astype(np.uint16)

        # depth_pix = self.img_values

        # now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        # distances = open("./src/demorobot_depth/distances/{}_distances.txt".format(now), 'w')
        # distances.write(now + ":\n")

    def detect_points_callback(self, msg):
        detect_box = msg.box
        distance = self.cal_distance(detect_box)
        
        box_d = Float32()
        box_d.data = distance
        self.pub_box_d.publish(box_d)

    def cal_distance(self, box):
        box_mid_x = int((box[0] + box[2]) / 2)
        box_mid_y = int((box[1] + box[3]) / 2)

        # unit = meter
        d_raw = float(self.img_values[box_mid_y][box_mid_x]) / 1000
        d_hor = float(self.img_values[240][box_mid_x]) / 1000

        # unit = meter
        return d_hor

def main(args=None):
    rclpy.init(args=args)
    depth_camera = DepthCamera()
    rclpy.spin(depth_camera)
    depth_camera.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()