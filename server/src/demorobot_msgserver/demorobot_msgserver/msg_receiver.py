import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_system_default
from rclpy.qos import qos_profile_services_default

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose, PoseWithCovarianceStamped, PoseWithCovariance, Transform
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from demorobot_msg.msg import BatteryStat, Detect, DetectArray
from demorobot_action_interfaces.action import RobotNamespace
from tf2_ros import transform_listener

import paho.mqtt.client as mqtt
import json

import random

class MsgReceiver(Node):
    def __init__(self):
        super().__init__('MsgReceiver')

        '''mqtt settings'''
        self.declare_parameter("~broker_ip_address", '')
        # self.declare_parameter("~broker_ip_route", 9999)
        self.declare_parameter("~mqtt_pub_topic", '')
        self.declare_parameter("~mqtt_username", '')
        self.declare_parameter("~mqtt_password", '')

        self.broker_address = self.get_parameter("~broker_ip_address").get_parameter_value().string_value
        # self.broker_route = self.get_parameter("~broker_ip_route").get_parameter_value().integer_value
        self.MQTT_PUB_TOPIC = self.get_parameter("~mqtt_pub_topic").get_parameter_value().string_value
        self.MQTT_USERNAME = self.get_parameter("~mqtt_username").get_parameter_value().string_value
        self.MQTT_PASSWORD = self.get_parameter("~mqtt_password").get_parameter_value().string_value

        try:
            # 관제 서버 전송 mqtt 설정
            self.mqttclient = mqtt.Client("robot_server_msg_receiver")
            self.mqttclient.username_pw_set(self.MQTT_USERNAME, self.MQTT_PASSWORD)

            self.mqttclient.connect(self.broker_address)
            # self.mqttclient.connect(self.broker_address, self.broker_route)
            self.mqttclient.loop_start()

            # 토픽 구독 변수 생성
            self.sub_battery_stat = self.create_subscription(BatteryStat, '/battery_stat', self.battery_callback, 10)
            self.sub_detect_data = self.create_subscription(DetectArray, '/pose_detect/detect_data', self.detect_data_callback, 10)
            self.sub_robot_pose = self.create_subscription(Odometry, '/odom', self.robot_pose_callback, 10)
            self.sub_robot_tf = self.create_subscription(TFMessage, '/tf', self.robot_tf_callback, 10)
            # unused variable resolve
            self.sub_battery_stat
            self.sub_detect_data
            self.sub_robot_pose
            self.sub_robot_tf
            
            self.detect_data_arr = []
            self.robot_pose = PoseStamped()
            self.robot_tf = Transform()
            
            self.battery_mode = None
            self.battery_vol = None

            timer_period = 0.05  # seconds
            self.mqtt_timer = self.create_timer(timer_period, self.mqtt_timer_callback)

        except TimeoutError:
            self.get_logger().error("Cannot connect to mqtt broker {}".format(self.broker_address))
    
    '''MQTT 메시지 전송'''
    def mqtt_timer_callback(self):
        self.publish_mqtt()

    def publish_mqtt(self):
        Dictionary ={
            # robot info start
            'pos_x':str(self.robot_pose.pose.position.x),
            'pos_y':str(self.robot_pose.pose.position.y),
            'ori_x':str(self.robot_pose.pose.orientation.x),
            'ori_y':str(self.robot_pose.pose.orientation.y),
            'ori_z':str(self.robot_pose.pose.orientation.z),
            'ori_w':str(self.robot_pose.pose.orientation.w),
            'tf_pos_x':self.robot_tf.translation.x,
            'tf_pos_y':self.robot_tf.translation.y,
            'tf_ori_x':self.robot_tf.rotation.x,
            'tf_ori_y':self.robot_tf.rotation.y,
            'tf_ori_z':self.robot_tf.rotation.z,
            'tf_ori_w':self.robot_tf.rotation.w,
            # robot info end
            # detect info - multiple object info start
            'detect_data_arr':self.detect_data_arr
            # detect info - multiple object info end
        }
        self.mqttclient.publish(self.MQTT_PUB_TOPIC, json.dumps(Dictionary).encode(), qos=0, retain=False)
        # if self.detect_data_arr != None:
        #     self.detect_data_arr.clear()
    
    '''배터리 정보 callback'''
    def battery_callback(self, data):
        self.battery_mode = data.mode
        self.battery_vol = data.vol

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

    '''로봇 좌표 callback'''
    def robot_pose_callback(self, data):
        if data != None:
            self.robot_pose.pose.position.x = data.pose.pose.position.x
            self.robot_pose.pose.position.y = data.pose.pose.position.y * -1
            # self.robot_pose.pose.position.z = data.pose.pose.position.z

            self.robot_pose.pose.orientation.x = data.pose.pose.orientation.x
            self.robot_pose.pose.orientation.y = data.pose.pose.orientation.y
            self.robot_pose.pose.orientation.z = data.pose.pose.orientation.z * -1
            self.robot_pose.pose.orientation.w = data.pose.pose.orientation.w
    
    def robot_tf_callback(self, data):
        for d in data.transforms:
            if d.header.frame_id == "map" and d.child_frame_id == "odom":
                self.robot_tf.translation.x = d.transform.translation.x
                self.robot_tf.translation.y = d.transform.translation.y * -1
                self.robot_tf.rotation.x = d.transform.rotation.x
                self.robot_tf.rotation.y = d.transform.rotation.y
                self.robot_tf.rotation.z = d.transform.rotation.z * -1
                self.robot_tf.rotation.w = d.transform.rotation.w

def main(args=None):
    rclpy.init(args=args)
    msg_sender = MsgReceiver()
    rclpy.spin(msg_sender)
    msg_sender.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()