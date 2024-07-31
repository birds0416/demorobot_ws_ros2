import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration

import time

class GoalSequenceNavigator(Node):
    def __init__(self):
        super().__init__('goal_sequence_navigator')
        self.goal_poses = [
            (0.0, 0.0),
            (0.0, -1.0),
            (0.0, -2.0),
            (0.0, -3.0),
            (1.0, 0.0),
            (1.0, -1.0),
            (1.0, -2.0),
            (1.0, -3.0),
            (2.0, 0.0),
            (2.0, -1.0),
            (2.0, -2.0),
            (2.0, -3.0),
            (3.0, 0.0),
            (3.0, -1.0),
            (3.0, -2.0),
            (3.0, -3.0),
            (4.0, 0.0),
            (4.0, -1.0),
            (4.0, -2.0),
            (4.0, -3.0),
            (5.0, 0.0),
            (5.0, -1.0),
            (5.0, -2.0),
            (5.0, -3.0)
        ]
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, x, y):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0  # Neutral orientation (facing forward)

        self.action_client.wait_for_server()
        self._send_goal_future = self.action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result:
            self.get_logger().info('Goal succeeded!')
        else:
            self.get_logger().info('Goal failed!')

        if self.goal_poses:
            next_goal = self.goal_poses.pop(0)
            time.sleep(3)
            self.send_goal(next_goal[0], next_goal[1])
        else:
            self.get_logger().info('All goals reached!')
            rclpy.shutdown()

    def run(self):
        self.send_goal(*self.goal_poses.pop(0))

def main(args=None):
    rclpy.init(args=args)
    navigator = GoalSequenceNavigator()
    navigator.run()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()