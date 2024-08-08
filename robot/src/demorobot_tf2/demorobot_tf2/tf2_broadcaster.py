import math

import numpy as np

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Pose

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

class FramePublisher(Node):
    def __init__(self):
        super().__init__('tf2_frame_publisher')
        
        self.NAMESPACE = self.get_namespace()
        if self.NAMESPACE == "/":
            self.NAMESPACE = ""
        
        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pub_tf = self.create_publisher(TransformStamped, self.NAMESPACE + '/demorobot/tf', 10)
        
        self._subscription = self.create_subscription(
            Pose,
            '/pose',
            self.pose_callback,
            1
        )
        # prevent unused variable warning
        self._subscription
        
    def pose_callback(self, msg):
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = "odom"

        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0

        q = quaternion_from_euler(0, 0, msg.theta)
        self.get_logger().info("msg.theta: {}".format(msg.theta))
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)
        self.pub_tf.publish(t)
        
        
def main():
    rclpy.init()
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()