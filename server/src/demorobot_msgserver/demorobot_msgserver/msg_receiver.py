import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from rclpy.qos import qos_profile_services_default

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose, PoseWithCovarianceStamped, PoseWithCovariance
from demorobot_msg.msg import BatteryStat
from tf2_ros import transform_listener

import paho.mqtt.client as mqtt
import json

class MsgReceiver(Node):
    def __init__(self):
        super().__init__('MsgReceiver')

        self.battery_stat_sub = self.create_subscription(BatteryStat, '/device01/battery_stat', self.battery_callback, 10)
        
        self.battery_mode = None
        self.battery_vol = None
    
    def battery_callback(self, data):
        self.battery_mode = data.mode
        self.battery_vol = data.vol

def main(args=None):
    rclpy.init(args=args)
    msg_sender = MsgReceiver()
    rclpy.spin(msg_sender)
    msg_sender.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()