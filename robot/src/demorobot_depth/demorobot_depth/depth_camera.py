import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image

class DepthCamera(Node):
    def __init__(self):
        super().__init__('DepthCamera')

        self.NAMESPACE = self.get_namespace()
        self.get_logger().info("Namespace: {}".format(self.NAMESPACE))

        self.sub_depth_points = self.create_subscription(PointCloud2, self.NAMESPACE + '/camera/depth/points', self.depth_points_callback, 10)
        self.sub_depth_img = self.create_subscription(Image, self.NAMESPACE + '/camera/depth/image_raw', self.depth_image_callback, 10)
    
    def depth_points_callback(self, msg):
        pass
    
    def depth_image_callback(self, msg):
        pass

def main(args=None):
    rclpy.init(args=args)
    depth_camera = DepthCamera()
    rclpy.spin(depth_camera)
    depth_camera.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()