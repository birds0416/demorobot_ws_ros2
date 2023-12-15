import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_system_default
from rclpy.qos import qos_profile_services_default

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose, PoseWithCovarianceStamped, PoseWithCovariance
from demorobot_msg.msg import BatteryStat
from demorobot_action_interfaces.action import RobotNamespace
from tf2_ros import transform_listener

import paho.mqtt.client as mqtt
import json

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
            self.mqttclient.on_message = self.on_message
            self.mqttclient.username_pw_set(self.MQTT_USERNAME, self.MQTT_PASSWORD)

            self.mqttclient.connect(self.broker_address)
            # self.mqttclient.connect(self.broker_address, self.broker_route)
            self.mqttclient.loop_start()

            # 토픽 구독 변수 생성
            self.sub_battery_stat = self.create_subscription(BatteryStat, '/battery_stat', self.battery_callback, 10)
            self.sub_robot_pose = self.create_subscription(PoseWithCovarianceStamped, '/pose', self.robot_pose_callback, 10)
            
            self.robot_pose = PoseStamped()
            
            self.battery_mode = None
            self.battery_vol = None

        except TimeoutError:
            self.get_logger().error("Cannot connect to mqtt broker {}".format(self.broker_address))
    
    '''배터리 정보 callback'''
    def battery_callback(self, data):
        self.battery_mode = data.mode
        self.battery_vol = data.vol

    '''로봇 좌표 callback'''
    def robot_pose_callback(self, data):
        if data != None:
            self.robot_pose.pose.position.x = data.pose.pose.position.x
            self.robot_pose.pose.position.y = data.pose.pose.position.y
            # self.robot_pose.pose.position.z = data.pose.pose.position.z

            # self.robot_pose.pose.orientation.x = data.pose.pose.orientation.x
            # self.robot_pose.pose.orientation.y = data.pose.pose.orientation.y
            self.robot_pose.pose.orientation.z = data.pose.pose.orientation.z
            self.robot_pose.pose.orientation.w = data.pose.pose.orientation.w

            Dictionary ={
                'pos_x':str(self.robot_pose.pose.position.x),
                'pos_y':str(self.robot_pose.pose.position.y),
                'ori_z':str(self.robot_pose.pose.orientation.z),
                'ori_w':str(self.robot_pose.pose.orientation.w)}
            self.mqttclient.publish(self.MQTT_PUB_TOPIC, json.dumps(Dictionary).encode(),qos=0, retain=False)
            self.get_logger().info("Robot Pose MQTT Sent to {}:\npos_x = {},\nposy: {},\nori_z = {},\nori_w = {}".format(
                                                                self.MQTT_PUB_TOPIC,
                                                                self.robot_pose.pose.position.x,
                                                                self.robot_pose.pose.position.y,
                                                                self.robot_pose.pose.orientation.z,
                                                                self.robot_pose.pose.orientation.w))
    
def main(args=None):
    rclpy.init(args=args)
    msg_sender = MsgReceiver()
    rclpy.spin(msg_sender)
    msg_sender.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()