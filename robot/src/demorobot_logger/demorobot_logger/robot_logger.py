import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Pose, PoseWithCovarianceStamped, PoseWithCovariance, Transform
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener 
from nav_msgs.msg import Odometry
from demorobot_msg.msg import CustomPose, Detect, DetectArray

from datetime import datetime
import math

class RobotLogger(Node):
    def __init__(self):
        super().__init__('robot_logger')
        
        # logging info
        now = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.robot_log = open("./cornersdev/demorobot_ws/src/demorobot_logger/logs/{}_log.txt".format(now), 'w+')
        
        # subscription
        self.sub_robot_pose = self.create_subscription(Odometry, '/odom', self.robot_pose_callback, 10)
        self.sub_robot_pose
        # self.robot_pose = PoseStamped()
        self.robot_pose = Odometry()
        
        self.sub_robot_tf = self.create_subscription(TFMessage, '/tf', self.robot_tf_callback, 10)
        self.sub_robot_tf
        self.robot_tf = Transform()
        
        self.sub_detect_data = self.create_subscription(DetectArray, '/pose_detect/detect_data', self.detect_data_callback, 10)
        self.sub_detect_data
        self.detect_data_arr = []
        
        self.declare_parameter('map', 'base_footprint')
        self.target_frame = self.get_parameter('map').get_parameter_value().string_value
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.pub_map_base = self.create_publisher(CustomPose, '/map_base_info', 10)
        
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
    def timer_callback(self):
        
        from_frame_rel = self.target_frame
        to_frame_rel = 'map'
        trans = None
        
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                        to_frame_rel,
                        from_frame_rel,
                        now)
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return
        
        now2 = datetime.now().strftime("%H-%M-%S")
        self.get_logger().info("Logging.......")
        self.robot_log.write(now2 + " Robot Odom Topic Log: {\n")
        # self.robot_log.write("\ttf x: {}\n".format(self.robot_tf.translation.x))
        # self.robot_log.write("\tpose x: {}\n".format(self.robot_pose.pose.pose.position.x))
        # self.robot_log.write("\ttf y: {}\n".format(self.robot_tf.translation.y))
        # self.robot_log.write("\tpose y : {}\n\n".format(self.robot_pose.pose.pose.position.y))
        
        self.current_x = trans.transform.translation.x
        self.current_y = trans.transform.translation.y    
        roll, pitch, yaw = self.euler_from_quaternion(
            trans.transform.rotation.x,
            trans.transform.rotation.y,
            trans.transform.rotation.z,
            trans.transform.rotation.w
        )
        self.current_yaw = yaw
        
        map_base_msg = CustomPose()
        map_base_msg.cur_x = self.current_x
        map_base_msg.cur_y = self.current_y
        map_base_msg.cur_yaw = self.current_yaw
        
        self.pub_map_base.publish(map_base_msg)
        
        # temp_x, temp_y = self.robot_pose.pose.pose.position.x, self.robot_pose.pose.pose.position.y
        # if temp_x != 0 and temp_y != 0.0:
        #     r = math.sqrt(temp_x ** 2 + temp_y ** 2)
        #     theta2 = math.asin(temp_x / r)
        #     theta1 = math.pi / 2 - self.robot_tf.rotation.w - theta2
        #     tf_pose_x = r * math.cos(theta1) + abs(self.robot_tf.translation.x)
        #     tf_pose_y = r * math.sin(theta1) + abs(self.robot_tf.translation.y)
        #     # tf_pose_x = temp_x * math.cos(self.robot_tf.rotation.w) - temp_y * math.sin(self.robot_tf.rotation.w)
        #     # tf_pose_y = temp_x * math.sin(self.robot_tf.rotation.w) + temp_y * math.cos(self.robot_tf.rotation.w)
        #     self.robot_log.write("\ttf_pose x : {}\n".format(tf_pose_x))
        #     self.robot_log.write("\ttf_pose y : {}\n\n".format(tf_pose_y))
        self.robot_log.write("\tcur_pose x : {}\n".format(self.current_x))
        self.robot_log.write("\tcur_pose y : {}\n\n".format(self.current_y))
        self.robot_log.write("\tcur_yaw : {}\n\n".format(self.current_yaw))
        
        self.robot_log.write("\tpose.pose.orientation.x: {}\n".format(self.robot_pose.pose.pose.orientation.x))
        self.robot_log.write("\tpose.pose.orientation.y: {}\n".format(self.robot_pose.pose.pose.orientation.y))
        self.robot_log.write("\tpose.pose.orientation.z: {}\n".format(self.robot_pose.pose.pose.orientation.z))
        self.robot_log.write("\tpose.pose.orientation.w: {}\n\n".format(self.robot_pose.pose.pose.orientation.w))
        
        self.robot_log.write("\ttf.rotation.x: {}\n".format(self.robot_tf.rotation.x))
        self.robot_log.write("\ttf.rotation.y: {}\n".format(self.robot_tf.rotation.y))
        self.robot_log.write("\ttf.rotation.z: {}\n".format(self.robot_tf.rotation.z))
        self.robot_log.write("\ttf.rotation.w: {}\n\n".format(self.robot_tf.rotation.w))
        
        self.robot_log.write("\tdetect_data_arr: {}\n\n".format(self.detect_data_arr))
        
        self.robot_log.write("} Robot Odom Topic Log End\n\n")
        
    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
        
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        
        return roll_x, pitch_y, yaw_z # in radians
        
    def robot_pose_callback(self, data):
        if data != None:
            ''' PoseStamped '''
            ''' Odometry '''
            self.robot_pose.pose.pose.position.x = data.pose.pose.position.x
            self.robot_pose.pose.pose.position.y = data.pose.pose.position.y
            self.robot_pose.pose.pose.orientation.x = data.pose.pose.orientation.x
            self.robot_pose.pose.pose.orientation.y = data.pose.pose.orientation.y
            self.robot_pose.pose.pose.orientation.z = data.pose.pose.orientation.z
            self.robot_pose.pose.pose.orientation.w = data.pose.pose.orientation.w
            
    def robot_tf_callback(self, data):
        for d in data.transforms:
            if d.header.frame_id == "map" and d.child_frame_id == "odom":
                self.robot_tf.translation.x = d.transform.translation.x
                self.robot_tf.translation.y = d.transform.translation.y
                self.robot_tf.rotation.x = d.transform.rotation.x
                self.robot_tf.rotation.y = d.transform.rotation.y
                self.robot_tf.rotation.z = d.transform.rotation.z
                self.robot_tf.rotation.w = d.transform.rotation.w
                
    def detect_data_callback(self, msg):
        # self.detect_data.box_mid_x = data.box_mid_x
        # self.detect_data.box_mid_y = data.box_mid_y
        # self.detect_data.distance = data.distance
        if msg.data != []:
            self.detect_data_arr.clear()
            for item in msg.data:
                temp_data = {
                    "id" : item.id,
                    "data" : {
                        "isFall": item.fall,
                        "box_mid_x" : item.box_mid_x,
                        "box_mid_y" : item.box_mid_y,
                        # "detect_key" : item.keypnt_data,
                        "depth_val" : item.distance
                    }
                }
                self.detect_data_arr.append(temp_data)
        
def main(args=None):
    rclpy.init(args=args)
    robot_logger = RobotLogger()
    rclpy.spin(robot_logger)
    robot_logger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
            
