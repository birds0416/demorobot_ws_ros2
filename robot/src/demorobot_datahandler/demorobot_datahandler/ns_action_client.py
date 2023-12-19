import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from std_msgs.msg import Int32
from demorobot_action_interfaces.action import RobotNamespace

class NamespaceActionClient(Node):
    def __init__(self):
        super().__init__('namespace_action_client')
        self._action_client = ActionClient(self, RobotNamespace, 'robot_namespace')
        
        self.NAMESPACE = self.get_namespace()
        self.DEVICE_N = Int32()
        self.DEVICE_N.data = int(self.NAMESPACE[-2:])
        self.get_logger().info("self.DEVICE_N: {}".format(self.DEVICE_N.data))

    def send_goal(self):
        goal_msg = RobotNamespace.Goal()
        goal_msg.device_num = self.DEVICE_N.data
        self._action_client.wait_for_server()
        
        # self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        # self._send_goal_future.add_done_callback(self.goal_response_callback)

        return self._action_client.send_goal_async(goal_msg)
    
    # def goal_response_callback(self, future):
    #     goal_handle = future.result()
    #     if not goal_handle.accepted:
    #         self.get_logger().info("Goal rejected : (")
    #         return
        
    #     self.get_logger().info('Goal accepted :)')

    #     self._get_result_future = goal_handle.get_result_async()
    #     self._get_result_future.add_done_callback(self.get_result_callback)
    
    # def get_result_callback(self, future):
    #     result = future.result().result
    #     self.get_logger().info('Result: {0}'.format(result.device_list))
    #     rclpy.shutdown()
    
def main(args=None):
    rclpy.init(args=args)
    action_client = NamespaceActionClient()
    action_client.send_goal()
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()