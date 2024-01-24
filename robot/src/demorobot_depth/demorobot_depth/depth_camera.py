import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge

class DepthCamera(Node):
    def __init__(self):
        super().__init__('DepthCamera')

        self.NAMESPACE = self.get_namespace()
        self.get_logger().info("Namespace: {}".format(self.NAMESPACE))

        self.sub_depth_img = self.create_subscription(Image, self.NAMESPACE + '/camera/depth/image_raw', self.depth_image_callback, 10)
        # self.sub_depth_points = self.create_subscription(PointCloud2, self.NAMESPACE + '/camera/depth/points', self.depth_points_callback, 10)

        self.pub_center_d = self.create_publisher(Float32, self.NAMESPACE + '/camera/depth/center_distance', 10)
        self.center_d = Float32()

        self.bridge = CvBridge()
    
    def depth_image_callback(self, msg):
        # Version 1 -> get center pixel value
        depths = msg.data

        ''' Version 3'''
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        h, w = cv_img.shape
        img = []
        for j in range(h):
            temp = []
            for i in range(w):
                temp.append(str(cv_img[j, i]))
            img.append(temp)
        center = float(img[240][320]) / 1000
        self.center_d.data = center
        self.pub_center_d.publish(self.center_d)
        # self.get_logger().info("Center value : {} m".format(self.center_d.data))

    def depth_points_callback(self, msg):
        pass

def main(args=None):
    rclpy.init(args=args)
    depth_camera = DepthCamera()
    rclpy.spin(depth_camera)
    depth_camera.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()