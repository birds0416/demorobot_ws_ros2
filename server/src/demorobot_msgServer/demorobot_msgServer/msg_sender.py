import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32, Int32MultiArray
from geometry_msgs.msg import PoseStamped, Pose, PoseWithCovarianceStamped, PoseWithCovariance
from tf2_ros import transform_listener

import paho.mqtt.client as mqtt
import json

class MsgSender(Node):
    def __init__(self):
        super().__init__('MsgReceiver')

        self.declare_parameter("~broker_ip_address", '')
        # self.declare_parameter("~broker_ip_route", 9999)
        # self.declare_parameter("~mqtt_pub_topic", '')
        self.declare_parameter("~mqtt_sub_topic", '')
        self.declare_parameter("~mqtt_username", '')
        self.declare_parameter("~mqtt_password", '')

        self.broker_address = self.get_parameter("~broker_ip_address").get_parameter_value().string_value
        # self.broker_route = self.get_parameter("~broker_ip_route").get_parameter_value().integer_value
        # self.MQTT_PUB_TOPIC = self.get_parameter("~mqtt_pub_topic").get_parameter_value().string_value
        self.MQTT_SUB_TOPIC_1 = self.get_parameter("~mqtt_sub_topic_1").get_parameter_value().string_value
        self.MQTT_SUB_TOPIC_2 = self.get_parameter("~mqtt_sub_topic_2").get_parameter_value().string_value
        self.MQTT_SUB_TOPIC_3 = self.get_parameter("~mqtt_sub_topic_3").get_parameter_value().string_value
        self.MQTT_USERNAME = self.get_parameter("~mqtt_username").get_parameter_value().string_value
        self.MQTT_PASSWORD = self.get_parameter("~mqtt_password").get_parameter_value().string_value

        try:

            # mqtt settings
            self.mqttclient = mqtt.Client("robot_server_msg_receiver")
            self.mqttclient.on_message = self.on_message
            self.mqttclient.username_pw_set(self.MQTT_USERNAME, self.MQTT_PASSWORD)

            self.mqttclient.connect(self.broker_address)
            # self.mqttclient.connect(self.broker_address, self.broker_route)
            self.mqttclient.subscribe(self.MQTT_SUB_TOPIC_1)
            self.mqttclient.loop_start()
            
            # ROS2 publisher
            self.pose_from_server = self.create_publisher(PoseStamped, '/server_msg/pose_from_server', 10)
            self.robot_drive_mode = self.create_publisher(Int32, '/server_msg/drive_mode', 10)
            # self.map_info = self.create_publisher(Int32MultiArray, '/server_msg/map_info', 10)
            
            self.get_logger().info('ROS2 MQTT Broker:: START...')
            self.get_logger().info('ROS2 MQTT Broker:: broker_address = {}'.format(self.broker_address))
            # self.get_logger().info('ROS2 MQTT Broker:: broker_address = {}:{}'.format(self.broker_address, self.broker_route))
            self.get_logger().info('ROS2 MQTT Broker:: MQTT_PUB_TOPIC = {}'.format(self.MQTT_PUB_TOPIC))
            self.get_logger().info('ROS2 MQTT Broker:: MQTT_SUB_TOPIC = {}'.format(self.MQTT_SUB_TOPIC))
        
        except TimeoutError:
            self.get_logger().error("Cannot connect to mqtt broker {}".format(self.broker_address))

    def on_message(self, client, userdata, msg):
        msg_topic = msg.topic
        msg = str(msg.payload.decode("utf-8"))
        msg_dict = json.loads(msg)
        
        if msg_dict is not None:

            if msg_topic == self.MQTT_SUB_TOPIC_1:
                self.pub_msg_topic_1(msg_dict)
            elif msg_topic == self.MQTT_SUB_TOPIC_2:
                self.pub_msg_topic_2(msg_dict)
            elif msg_topic == self.MQTT_SUB_TOPIC_3:
                self.pub_msg_topic_3(msg_dict)
    
    def pub_msg_topic_1(self, msg_dict):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'robot_pose_from_server'

        pose_msg.pose.position.x = float(msg_dict['pos_x'])
        pose_msg.pose.position.y = float(msg_dict['pos_y'])
        pose_msg.pose.orientation.z = float(msg_dict['ori_z'])
        pose_msg.pose.orientation.w = float(msg_dict['ori_w'])

        self.pose_from_server.publish(pose_msg)
        self.get_logger().info("Pose published to topic {}:\npose_x: {},\npose_y: {}".format('/server_msg/pose_from_server', 
                                                                                pose_msg.pose.position.x,
                                                                                pose_msg.pose.position.y))

    def pub_msg_topic_2(self, msg_dict):
        drive_mode = Int32()
        drive_mode.data = int(msg_dict['mode'])
        
        self.robot_drive_mode.publish(drive_mode)
        self.get_logger().info("Drive mode published to topic {}: {}".format('/server_msg/drive_mode', drive_mode.data))
    
    def pub_msg_topic_3(self, msg_dict):
        pass

def main(args=None):
    rclpy.init(args=args)
    rpose_subscriber = MsgSender()
    rclpy.spin(rpose_subscriber)
    rpose_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()