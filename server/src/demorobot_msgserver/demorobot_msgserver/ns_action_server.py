import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from std_msgs.msg import Int32, Int32MultiArray
from demorobot_action_interfaces.action import RobotNamespace

class NamespaceActionServer(Node):
    def __init__(self):
        super().__init__('namespace_action_server')
        self._action_server = ActionServer(
            self,
            RobotNamespace,
            'robot_namespace',
            self.execute_callback
        )

        self.device_list = []
        self.pub_device_list = self.create_publisher(Int32MultiArray, '/current_device_nums', 10)
        self.device_list_msg = Int32MultiArray()

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        # Execute action here
        # feedback_msg = RobotNamespace.Feedback()
        temp_device_list = []
        self.device_list.append(goal_handle.request.device_num)

        self.get_logger().info("self.device_list: {}".format(self.device_list))

        for device_num in self.device_list:
            temp_device_list.append(device_num)
        
        goal_handle.succeed()

        result = RobotNamespace.Result()
        result.device_list = temp_device_list
        self.device_list_msg.data = result.device_list
        self.pub_device_list.publish(self.device_list_msg)
        return result

def main(args=None):
    rclpy.init(args=args)
    namespace_action_server = NamespaceActionServer()
    rclpy.spin(namespace_action_server)

if __name__ == '__main__':
    main()


        