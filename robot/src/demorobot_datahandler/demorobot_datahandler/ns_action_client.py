import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from std_msgs.msg import Int32
from demorobot_action_interfaces.action import RobotNamespace

class NamespaceActionClient(Node):
    def __init__(self):
        super().__init__('namespace_action_client')
        self._action_client = ActionClient(self, RobotNamespace, '/robot_namespace')
        
        self.NAMESPACE = self.get_namespace()
        if self.NAMESPACE == "/":
            self.NAMESPACE = ""
            
        self.DEVICE_N = Int32()
        self.DEVICE_N.data = int(self.NAMESPACE[-2:])
        self.get_logger().info("self.DEVICE_N: {}".format(self.DEVICE_N.data))
        self.send_goal()

    def send_goal(self):
        goal_msg = RobotNamespace.Goal()
        goal_msg.device_num = self.DEVICE_N.data
        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(goal_msg)
    
def main(args=None):
    rclpy.init(args=args)
    action_client = NamespaceActionClient()
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()