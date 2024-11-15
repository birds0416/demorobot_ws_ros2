import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from rclpy.qos import qos_profile_services_default

from std_msgs.msg import Int8, String
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Pose, PoseWithCovarianceStamped, PoseWithCovariance, Point
from demorobot_msg.msg import BatteryStat
from std_srvs.srv import SetBool

from .src.main_gui import *
import json

class ROS2Worker(QThread):
    def __init__(self, node):
        super().__init__()
        self.node = node

    def run(self):
        rclpy.spin(self.node)  # Run ROS2 spin in this thread
        rclpy.shutdown()  # Shutdown ROS2 node when thread finishes

class NormalMode(Node):
    def __init__(self):
        super().__init__('normal_mode')
        self.NAMESPACE = self.get_namespace()
        if self.NAMESPACE == "/":
            self.NAMESPACE = ""

        ''' 위치 parameters '''
        #region
        location_path = "/home/jetson/cornersdev/demorobot_ws/src/demorobot_gui/config/location.json"
        with open(location_path, 'r') as rf:
            location_data = json.load(rf)

        self.LOC_1 = location_data["location_1"]
        self.LOC_2 = location_data["location_2"]
        self.LOC_3 = location_data["location_3"]
        self.LOC_4 = location_data["location_4"]
        self.LOC_5 = location_data["location_5"]
        self.LOC_6 = location_data["location_6"]
        #endregion

        self.ros2_worker = ROS2Worker(self)
        self.ros2_worker.start()

        self.pub_location = self.create_publisher(PoseStamped, self.NAMESPACE + '/location_pose_navigation', 10)
        self.sub_goal_result = self.create_subscription(Int8, self.NAMESPACE + '/navigation_result', self.nav_result_callback, 10)
        self.cancel_navigation_client = self.create_client(SetBool, self.NAMESPACE + '/robot_navigator/cancel_navigation')
        
        self.is_nav = False

        self.bat_stat = BatteryStat()
        
        self.current_location = 0

        self.app = QApplication(sys.argv)
        self.gui = MainWindow()
        
        self.gui.bat_stat.setText(str(self.bat_stat))
        
        self.init_navigation_btns()
        self.gui.show()

    def init_navigation_Tab(self, num):
        self.gui.navigationTab.setupUI(self.gui, num)

        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = self.get_clock().now().to_msg()

        if num == 1:
            pose_msg.pose.position = Point(x = self.LOC_1[0], y = self.LOC_1[1], z = 0.0)
        elif num == 2:
            pose_msg.pose.position = Point(x = self.LOC_2[0], y = self.LOC_2[1], z = 0.0)
        elif num == 3:
            pose_msg.pose.position = Point(x = self.LOC_3[0], y = self.LOC_3[1], z = 0.0)
        elif num == 4:
            pose_msg.pose.position = Point(x = self.LOC_4[0], y = self.LOC_4[1], z = 0.0)
        elif num == 5:
            pose_msg.pose.position = Point(x = self.LOC_5[0], y = self.LOC_5[1], z = 0.0)
        elif num == 6:
            pose_msg.pose.position = Point(x = self.LOC_6[0], y = self.LOC_6[1], z = 0.0)
            
        self.is_nav = True
        
        self.current_location = num
        # self.get_logger().info("Pose: {}".format(pose_msg))
        self.pub_location.publish(pose_msg)

        self.gui.navigationTab.backBTN.clicked.connect(self.init_navigation_btns)
        self.gui.show()

    def init_navigation_btns(self):
        self.gui.navigationWindow.setupUI(self.gui)
        
        if self.is_nav:
            self.cancel_navigation()
            self.is_nav = False
        
        self.gui.navigationWindow.location_btn1.clicked.connect(lambda: self.init_navigation_Tab(1))
        self.gui.navigationWindow.location_btn2.clicked.connect(lambda: self.init_navigation_Tab(2))
        self.gui.navigationWindow.location_btn3.clicked.connect(lambda: self.init_navigation_Tab(3))
        self.gui.navigationWindow.location_btn4.clicked.connect(lambda: self.init_navigation_Tab(4))
        self.gui.navigationWindow.location_btn5.clicked.connect(lambda: self.init_navigation_Tab(5))
        self.gui.navigationWindow.location_btn6.clicked.connect(lambda: self.init_navigation_Tab(6))

    def nav_result_callback(self, msg):
        # UNKNOWN = 0
        # SUCCEEDED = 1
        # CANCELED = 2
        # FAILED = 3
        if msg.data == 1:
            self.gui.navigationTab.location_msg.setText(('위치 {}에\n도착하였습니다.').format(self.current_location))
            
    def bat_stat_callback(self, msg):
        self.bat_stat.vol = msg.vol

    def cancel_navigation(self):
        while not self.cancel_navigation_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting.....')

        request = SetBool.Request()
        future = self.cancel_navigation_client.call_async(request)
        self.get_logger().info('Navigation cancellation request sent (non-blocking)')

        if future.result() != None and future.result().success:
            self.get_logger().info('Navigation cancellation request sent successfully')
        else:
            self.get_logger().info('Failed to send navigation cancellation request')

def main(args=None):
    rclpy.init(args=args)
    normal_mode = NormalMode()
    sys.exit(normal_mode.app.exec_())

if __name__ == '__main__':
    main()
