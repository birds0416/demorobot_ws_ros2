import time
from enum import Enum
import subprocess
import threading

from action_msgs.msg import GoalStatus
from std_msgs.msg import Int8
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose, FollowWaypoints, ComputePathToPose
from nav2_msgs.srv import LoadMap, ClearEntireCostmap, ManageLifecycleNodes, GetCostmap
from std_srvs.srv import SetBool

import rclpy

from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

from rclpy.duration import Duration

class LaunchThread(threading.Thread):
    def __init__(self, package, launch_file, namespace):
        super().__init__()
        self.package = package
        self.launch_file = launch_file
        self.namespace = namespace
    
    def run(self):
        # subprocess.run(["ros2", "launch", self.package, self.launch_file, 'namespace:=' + self.namespace])
        subprocess.run(["ros2", "launch", self.package, self.launch_file])

class RunThread(threading.Thread):
    def __init__(self, package, run_file):
        super().__init__()
        self.package = package
        self.run_file = run_file
    
    def run(self):
        subprocess.run(["ros2", "run", self.package, self.run_file])

class NavigationResult(Enum):
    UNKNOWN = 0
    SUCCEEDED = 1
    CANCELED = 2
    FAILED = 3

class RobotNavigator(Node):
    def __init__(self):
        super().__init__('robot_navigator')
        self.NAMESPACE = self.get_namespace()
        if self.NAMESPACE == "/":
            self.NAMESPACE = ""

        self.goal_handle = None
        self.result_future = None
        self.feedback = None
        self.status = None

        amcl_pose_qos = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=1)

        self.initial_pose_received = False
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.follow_waypoints_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self.compute_path_to_pose_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        
        ''' pose related variables '''
        #region
        self.sub_initial_pose = self.create_subscription(PoseWithCovarianceStamped, 'initialpose', self._initial_pose_callback, 10)
        # self.initial_pose_pub = self.create_publisher(
        #     PoseWithCovarianceStamped,
        #     'initialpose',
        #     10)
        
        self.initial_pose = PoseStamped()
        # self.initial_pose.header.frame_id = 'map'
        # self.initial_pose.pose.position.x = 0.0
        # self.initial_pose.pose.position.y = 0.0
        # self.initial_pose.pose.orientation.z = 0.0
        # self.initial_pose.pose.orientation.w = 1.0
        
        # TODO localization_pose_sub is not correct
        ''' 
        self.locatlization_pose_sub는 로봇이 처음 켜졌을때 
        자신의 위치를 /pose 토픽으로 보내는데 그 정보를 구독하고, 
        callback 함수를 통해 initialpose로 전달한다. 
        즉 정확한 초기 위치 정보가 아닐 가능성이 높음. 
        초기 위치를 어떻게 파악할지에 대한 정확한 알고리즘 수정이 필요 
        '''
        self.robot_current_pose = Odometry()
        self.localization_pose_sub = self.create_subscription(Odometry,
                                                              '/odom',
                                                              self.currentPoseCallback,
                                                              10)

        self.is_pose_mqtt_received = False
        
        ''' 서버에서 보내는 위치 정보 '''
        #region
        self.is_pose_server = False
        self.sub_pose_server = self.create_subscription(
            PoseStamped,
            self.NAMESPACE + '/server_msg/pose_from_server',
            self.pose_server_callback,
            10
        )
        self.pose_msg_server = PoseStamped()
        #endregion
        
        ''' UI버튼에서 보내는 위치 정보 '''
        #region
        self.is_pose_ui = False
        self.sub_pose_ui = self.create_subscription(
            PoseStamped,
            self.NAMESPACE + '/location_pose_navigation',
            self.pose_ui_callback,
            10
        )
        self.pose_msg_ui = PoseStamped()
        #endregion
        
        self.sub_drive_mode = self.create_subscription(
            Int8,
            self.NAMESPACE + '/server_msg/drive_mode',
            self.drive_mode_callback,
            10
        )
        self.drive_mode = Int8()
        
        #endregion
        
        ''' navigation related variables '''
        #region
        # 처음 initial pose로 갈 때는 navigation result 안보내게
        self.nav_cnt = 0
        self.pub_navigation_result = self.create_publisher(Int8, self.NAMESPACE + '/navigation_result', 10)
        self.cancel_navigation_srv = self.create_service(SetBool, self.NAMESPACE + '/robot_navigator/cancel_navigation', self.cancel_navigation_callback)
        #endregion
        
        ''' costmap related variables '''
        #region
        self.change_maps_srv = self.create_client(LoadMap, 'map_server/load_map')
        self.clear_costmap_global_srv = self.create_client(
            ClearEntireCostmap, 
            'global_costmap/clear_entirely_global_costmap'
        )
        
        self.clear_costmap_local_srv = self.create_client(
            ClearEntireCostmap, 
            'local_costmap/clear_entirely_local_costmap'
        )
        
        self.get_costmap_global_srv = self.create_client(GetCostmap, 'global_costmap/get_costmap')
        self.get_costmap_local_srv = self.create_client(GetCostmap, 'local_costmap/get_costmap')
        #endregion

    # My Method
    def _initial_pose_callback(self, msg):
        self.initial_pose.pose.position.x = msg.pose.pose.position.x
        self.initial_pose.pose.position.y = msg.pose.pose.position.y
        self.initial_pose.pose.orientation.z = msg.pose.pose.orientation.z
        self.initial_pose.pose.orientation.w = msg.pose.pose.orientation.w
    
    def cancel_navigation_callback(self, request, response):
        self.cancelNav()
        response.success = True
        return response

    def drive_mode_callback(self, msg):
        self.drive_mode = msg.data

    def pose_ui_callback(self, data):
        self.pose_msg_ui.pose.position.x = data.pose.position.x
        self.pose_msg_ui.pose.position.y = data.pose.position.y
        self.is_pose_ui = True

    def pose_server_callback(self, data):
        self.pose_msg_server.pose.position.x = data.pose.position.x
        self.pose_msg_server.pose.position.y = data.pose.position.y
        # self.pose_msg_server.pose.orientation.z = data.pose.orientation.z
        # self.pose_msg_server.pose.orientation.w = data.pose.orientation.w
        self.is_pose_server = True
        self.is_pose_mqtt_received = True
        # self.received_pose_msg.pose.position.z = data.pose.position.z

        # self.received_pose_msg.pose.orientation.x = data.pose.orientation.x
        # self.received_pose_msg.pose.orientation.y = data.pose.orientation.y
        # self.received_pose_msg.pose.orientation.z = data.pose.orientation.z
        # self.received_pose_msg.pose.orientation.w = data.pose.orientation.w
        
    def currentPoseCallback(self, msg):
        self.robot_current_pose.pose.pose.position.x = msg.pose.pose.position.x
        self.robot_current_pose.pose.pose.position.y = msg.pose.pose.position.y
        self.robot_current_pose.pose.pose.orientation.z = msg.pose.pose.orientation.z
        self.robot_current_pose.pose.pose.orientation.w = msg.pose.pose.orientation.w
    
    # def initialPoseCallback(self, msg):
    #     self.initial_pose_received = True
    #     self.debug('Received initial pose')
    #     self.initial_pose.pose.position.x = msg.pose.pose.position.x
    #     self.initial_pose.pose.position.y = msg.pose.pose.position.y
    #     self.initial_pose.pose.position.z = msg.pose.pose.position.z

    #     self.initial_pose.pose.orientation.x = msg.pose.pose.orientation.x
    #     self.initial_pose.pose.orientation.y = msg.pose.pose.orientation.y
    #     self.initial_pose.pose.orientation.z = msg.pose.pose.orientation.z
    #     self.initial_pose.pose.orientation.w = msg.pose.pose.orientation.w
    
    # Built in Methods
    def goToPose(self, pose):
        # Sends a `NavToPose` action request
        self.debug("Waiting for 'NavigateToPose' action server")
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.info("'NavigateToPose' action server not available, waiting...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.info('Navigating to goal: ' + str(pose.pose.position.x) + ' ' +
                      str(pose.pose.position.y) + '...')
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg,
                                                                   self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Goal to ' + str(pose.pose.position.x) + ' ' +
                           str(pose.pose.position.y) + ' was rejected!')
            return False

        self.is_pose_mqtt_received = False
        self.result_future = self.goal_handle.get_result_async()
        return True

    def followWaypoints(self, poses):
        # Sends a `FollowWaypoints` action request
        self.debug("Waiting for 'FollowWaypoints' action server")
        while not self.follow_waypoints_client.wait_for_server(timeout_sec=1.0):
            self.info("'FollowWaypoints' action server not available, waiting...")

        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = poses

        self.info('Following ' + str(len(goal_msg.poses)) + ' goals.' + '...')
        send_goal_future = self.follow_waypoints_client.send_goal_async(goal_msg,
                                                                        self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Following ' + str(len(poses)) + ' waypoints request was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def cancelNav(self):
        self.info('Canceling current goal.')
        if self.result_future:
            future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future)
        return

    def isNavComplete(self):
        if not self.result_future:
            # task was cancelled or completed
            return True
        rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.10)
        if self.result_future.result():
            self.status = self.result_future.result().status
            self.info("status: {}".format(self.status))
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                self.debug('Goal with failed with status code: {0}'.format(self.status))
                return True
        else:
            # Timed out, still processing, not complete yet
            return False

        self.debug('Goal succeeded!')
        return True

    def getFeedback(self):
        return self.feedback

    def getResult(self):
        if self.status == GoalStatus.STATUS_SUCCEEDED:
            return NavigationResult.SUCCEEDED
        elif self.status == GoalStatus.STATUS_ABORTED:
            return NavigationResult.FAILED
        elif self.status == GoalStatus.STATUS_CANCELED:
            return NavigationResult.CANCELED
        else:
            return NavigationResult.UNKNOWN

    def waitUntilNav2Active(self):
        # self._waitForNodeToActivate('amcl')
        # self._waitForInitialPose()
        self._waitForNodeToActivate('bt_navigator')
        self.info('Nav2 is ready for use!')
        return

    def getPath(self, start, goal):
        # Sends a `NavToPose` action request
        self.debug("Waiting for 'ComputePathToPose' action server")
        while not self.compute_path_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.info("'ComputePathToPose' action server not available, waiting...")

        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = goal
        goal_msg.start = start

        self.info('Getting path...')
        send_goal_future = self.compute_path_to_pose_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Get path was rejected!')
            return None

        self.result_future = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, self.result_future)
        self.status = self.result_future.result().status
        if self.status != GoalStatus.STATUS_SUCCEEDED:
            self.warn('Getting path failed with status code: {0}'.format(self.status))
            return None

        return self.result_future.result().result.path

    def changeMap(self, map_filepath):
        while not self.change_maps_srv.wait_for_service(timeout_sec=1.0):
            self.info('change map service not available, waiting...')
        req = LoadMap.Request()
        req.map_url = map_filepath
        future = self.change_maps_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        status = future.result().result
        if status != LoadMap.Response().RESULT_SUCCESS:
            self.error('Change map request failed!')
        else:
            self.info('Change map request was successful!')
        return

    def clearAllCostmaps(self):
        self.clearLocalCostmap()
        self.clearGlobalCostmap()
        return

    def clearLocalCostmap(self):
        while not self.clear_costmap_local_srv.wait_for_service(timeout_sec=1.0):
            self.info('Clear local costmaps service not available, waiting...')
        req = ClearEntireCostmap.Request()
        future = self.clear_costmap_local_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return

    def clearGlobalCostmap(self):
        while not self.clear_costmap_global_srv.wait_for_service(timeout_sec=1.0):
            self.info('Clear global costmaps service not available, waiting...')
        req = ClearEntireCostmap.Request()
        future = self.clear_costmap_global_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return

    def getGlobalCostmap(self):
        while not self.get_costmap_global_srv.wait_for_service(timeout_sec=1.0):
            self.info('Get global costmaps service not available, waiting...')
        req = GetCostmap.Request()
        future = self.get_costmap_global_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().map

    def getLocalCostmap(self):
        while not self.get_costmap_local_srv.wait_for_service(timeout_sec=1.0):
            self.info('Get local costmaps service not available, waiting...')
        req = GetCostmap.Request()
        future = self.get_costmap_local_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().map

    def lifecycleStartup(self):
        self.info('Starting up lifecycle nodes based on lifecycle_manager.')
        srvs = self.get_service_names_and_types()
        for srv in srvs:
            if srv[1][0] == 'nav2_msgs/srv/ManageLifecycleNodes':
                srv_name = srv[0]
                self.info('Starting up ' + srv_name)
                mgr_client = self.create_client(ManageLifecycleNodes, srv_name)
                while not mgr_client.wait_for_service(timeout_sec=1.0):
                    self.info(srv_name + ' service not available, waiting...')
                req = ManageLifecycleNodes.Request()
                req.command = ManageLifecycleNodes.Request().STARTUP
                future = mgr_client.call_async(req)

                # starting up requires a full map->odom->base_link TF tree
                # so if we're not successful, try forwarding the initial pose
                while True:
                    rclpy.spin_until_future_complete(self, future, timeout_sec=0.10)
                    if not future:
                        self._waitForInitialPose()
                    else:
                        break
        self.info('Nav2 is ready for use!')
        return

    def lifecycleShutdown(self):
        self.info('Shutting down lifecycle nodes based on lifecycle_manager.')
        srvs = self.get_service_names_and_types()
        for srv in srvs:
            if srv[1][0] == 'nav2_msgs/srv/ManageLifecycleNodes':
                srv_name = srv[0]
                self.info('Shutting down ' + srv_name)
                mgr_client = self.create_client(ManageLifecycleNodes, srv_name)
                while not mgr_client.wait_for_service(timeout_sec=1.0):
                    self.info(srv_name + ' service not available, waiting...')
                req = ManageLifecycleNodes.Request()
                req.command = ManageLifecycleNodes.Request().SHUTDOWN
                future = mgr_client.call_async(req)
                rclpy.spin_until_future_complete(self, future)
                future.result()
        return

    def _waitForNodeToActivate(self, node_name):
        # Waits for the node within the tester namespace to become active
        self.debug('Waiting for ' + node_name + ' to become active..')
        node_service = node_name + '/get_state'
        state_client = self.create_client(GetState, node_service)
        while not state_client.wait_for_service(timeout_sec=1.0):
            self.info(node_service + ' service not available, waiting...')

        req = GetState.Request()
        state = 'unknown'
        while (state != 'active'):
            self.debug('Getting ' + node_name + ' state...')
            future = state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                state = future.result().current_state.label
                self.debug('Result of get_state: %s' % state)
            time.sleep(2)
        return

    def _waitForInitialPose(self):
        while not self.initial_pose_received:
            self.info('Setting initial pose')
            self._setInitialPose()
            self.info('Waiting for amcl_pose to be received')
            rclpy.spin_once(self, timeout_sec=1.0)
        return


    def _feedbackCallback(self, msg):
        self.debug('Received action feedback message')
        self.feedback = msg.feedback
        return

    def _setInitialPose(self):
        # msg = PoseWithCovarianceStamped()
        # msg.pose.pose = self.initial_pose.pose
        # msg.header.frame_id = self.initial_pose.header.frame_id
        # msg.header.stamp = self.initial_pose.header.stamp
        # self.info('Publishing Initial Pose')
        # self.initial_pose_pub.publish(msg)
        return

    def info(self, msg):
        self.get_logger().info(msg)
        return

    def warn(self, msg):
        self.get_logger().warn(msg)
        return

    def error(self, msg):
        self.get_logger().error(msg)
        return

    def debug(self, msg):
        self.get_logger().debug(msg)
        return

'''Start of Main'''
def main(args=None):
    rclpy.init(args=args)
    navigator = RobotNavigator()
    
    ''' yahboomcar_nav navigation 시작 '''
    laser_bringup  = LaunchThread("yahboomcar_nav", "laser_bringup_launch.py", navigator.NAMESPACE)
    laser_bringup.start()
    time.sleep(1)
    # display_nav  = LaunchThread("yahboomcar_nav", "display_nav_launch.py", navigator.NAMESPACE)
    # display_nav.start()
    # ros2 run yahboomcar_laser laser_Avoidance_a1_X3
    # laser_avoid = RunThread("yahboomcar_laser", "laser_Avoidance_a1_X3")
    # laser_avoid.start()
    # time.sleep(1)
    navigation_dwa  = LaunchThread("yahboomcar_nav", "navigation_dwa_launch.py", navigator.NAMESPACE)
    navigation_dwa.start()
    # navigation_teb  = LaunchThread("yahboomcar_nav", "navigation_teb_launch.py", navigator.NAMESPACE)
    # navigation_teb.start()
    time.sleep(5)
    
    # Set our demo's initial pose
    # navigator._setInitialPose()
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = navigator.initial_pose.pose.position.x
    initial_pose.pose.position.y = navigator.initial_pose.pose.position.y
    initial_pose.pose.orientation.z = navigator.initial_pose.pose.orientation.z
    initial_pose.pose.orientation.w = navigator.initial_pose.pose.orientation.w
    navigator.waitUntilNav2Active()
    navigator.goToPose(initial_pose)

    # Go to our demos first goal pose
    new_goal_pose = PoseStamped()
    new_goal_pose.header.frame_id = 'map'
    # new_goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    
    prev_goal_pose = PoseStamped()

    try:
        while rclpy.ok():
            rclpy.spin_once(navigator)
            if navigator.is_pose_server:
                new_goal_pose.pose.position.x = navigator.pose_msg_server.pose.position.x
                new_goal_pose.pose.position.y = navigator.pose_msg_server.pose.position.y
                new_goal_pose.pose.orientation.z = navigator.pose_msg_server.pose.orientation.z
                new_goal_pose.pose.orientation.w = navigator.pose_msg_server.pose.orientation.w
                navigator.get_logger().info("goal_pose: {}".format(new_goal_pose.pose))
                navigator.is_pose_server = False
            
            if navigator.is_pose_ui:
                new_goal_pose.pose.position.x = navigator.pose_msg_ui.poyse.position.x
                new_goal_pose.pose.position.y = navigator.pose_msg_ui.pose.position.y
                new_goal_pose.pose.orientation.z = navigator.pose_msg_server.pose.orientation.z
                new_goal_pose.pose.orientation.w = navigator.pose_msg_ui.pose.orientation.w
                navigator.get_logger().info("goal_pose: {}".format(new_goal_pose.pose))
                navigator.is_pose_ui = False

            '''TODO
            메시지 받으면 지금 네비게이션 동작중인지 확인하고, cancelNav를 호출
            goToPose 다음에 while not navigator.isNavComplete(): 이 구문이 있으니, 아까 예상대로 완료가 되어야지만 다음 로직 수행.
            '''
            prev_goal_pose = new_goal_pose
            if new_goal_pose == prev_goal_pose:
                pass
            else:
                navigator.goToPose(new_goal_pose)

            i = 0
            while not navigator.isNavComplete():
                # Do something with the feedback
                i = i + 1
                feedback = navigator.getFeedback()

                if navigator.is_pose_mqtt_received:
                    navigator.cancelNav()
                    navigator.status = GoalStatus.STATUS_CANCELED

            # Do something depending on the return code
            result = navigator.getResult()
            nav_result = Int8()
            if result == NavigationResult.SUCCEEDED:
                navigator.info('Goal succeeded')
                nav_result.data = 1
                if navigator.nav_cnt != 0:
                    navigator.pub_navigation_result.publish(nav_result)
                    navigator.nav_cnt += 1
            elif result == NavigationResult.CANCELED:
                navigator.info('Goal canceled')
                nav_result.data = 2
                navigator.pub_navigation_result.publish(nav_result)
            elif result == NavigationResult.FAILED:
                navigator.info('Goal failed')
                nav_result.data = 3
                navigator.pub_navigation_result.publish(nav_result)
            else:
                navigator.info('Goal invalid return status!')

            # navigator.lifecycleShutdown()
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("KeyboardInterrupt program terminate")
    
    finally:
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

'''End of Main'''
