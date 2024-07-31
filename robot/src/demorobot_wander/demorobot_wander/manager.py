from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from demorobot_action_interfaces.action import Wander
from nav2_msgs.action import NavigateToPose

from std_msgs.msg import Float32
from visualization_msgs.msg import MarkerArray

import rclpy
import math
from rclpy.action import ActionClient
from rclpy.node import Node

from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from std_msgs.msg import Bool

#ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {stamp: {sec: 0}, frame_id: 'map'}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
#ros2 param get /controller_server goal_checker.xy_goal_tolerance

import os, sys
import signal
import time

class Manager(Node):
    def __init__(self):
        super().__init__('manager')
        self._action_client_wanderer = ActionClient(self, Wander, 'wander')
        self.navigation_client = NavigationClient()
        timer_period = 5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.start_time=self.get_clock().now()
        
        self._goal_handle = None
        
        self.sub_wander_stop = self.create_subscription(Bool, '/wander/stop', self.wander_stop_callback, 10)
        self.sub_wander_stop # prevent unused variable error
        self.is_wander_stop = Bool()
        
    def print_feedback(self):
        try:
            self.map_explored="{:.2f}".format(self.map_explored) #Crop to 2 decimals
            time_now=self.get_clock().now()
            duration=str(int((time_now.nanoseconds-self.start_time.nanoseconds)/(10**9)))
        except:
            pass

    def timer_callback(self):
        #Print feedback in terminal acording to timer_period
        if not self.map_finished:
            self.print_feedback()
    
    def wander_stop_callback(self, msg):
        self.is_wander_stop.data = msg.data
        if self.is_wander_stop.data:
            self.cancel_goal()

    def goal_response_callback_wanderer(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Exploration goal rejected')
            return

        self.get_logger().info('Exploration goal accepted')
        self._goal_handle = goal_handle

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback_wanderer)

    def feedback_callback_wanderer(self, feedback):
        self.get_logger().info('Received feedback: {0}'.format(feedback.feedback.sequence))

    def get_result_callback_wanderer(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.map_finished=True
            self.get_logger().info('MAP SUCCESSFULLY EXPLORED')
            self.print_feedback()
            #Return to home
            self.navigation_client.send_goal()
        else:
            self.get_logger().info('Goal failed with status: {0}'.format(status))

    def send_goal_wanderer(self):
        self.get_logger().info('Waiting for action server...')
        self._action_client_wanderer.wait_for_server()

        goal_msg = Wander.Goal()
        goal_msg.map_completed_thres = 0.9
        self.get_logger().info('Sending wanderer goal request...')

        self._send_goal_future = self._action_client_wanderer.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback_wanderer)

        self._send_goal_future.add_done_callback(self.goal_response_callback_wanderer)
    
    def cancel_goal(self):
        if self._goal_handle is not None and self.is_wander_stop.data:
            self.get_logger().info("Attempting to cancel goal")
            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)
        else:
            self.get_logger().info("No goal to cancel or cancel request not received")
    
    def cancel_done_callback(self, future):
        cancel_response = future.result()
        # if cancel_response.return_code == GoalStatus.STATUS_CANCELED:
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal Successfully canceled')
        else:
            self.get_logger().info(f'Goal failed to cancel with return code: {cancel_response.return_code}')
        rclpy.shutdown()
    
class NavigationClient(Node):

    def __init__(self):
        super().__init__('navigation_client')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Exploration goal rejected')
            return

        self.get_logger().info('Navigation goal accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Arrived to home position')
        else:
            self.get_logger().info('Goal failed with status: {0}'.format(status))
            
    def send_goal(self):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.pose.orientation.w=1.0 #Home position

        self.get_logger().info('Returning to base...')

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.goal_response_callback)


def main(args=None):
    rclpy.init(args=args)
    manager = Manager()
    manager.send_goal_wanderer()
    rclpy.spin(manager)

if __name__ == '__main__':
    main()