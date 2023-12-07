import rclpy
from rclpy.node import Node

from demorobot_msg.msg import BatteryStat
from Rosmaster_Lib import Rosmaster

class BatteryStat(Node):
    def __init__(self):
        super().__init__('battery_status')

        self.bat_stat_pub = self.create_publisher(BatteryStat, '/device01/battery_stat', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.battery_vol_callback)

        self.g_debug = False
        self.robot_hw = Rosmaster(debug=self.g_debug)


    def battery_vol_callback(self):
        msg = BatteryStat()

        vol = int(self.robot_hw.get_battery_voltage() * 10) % 256
        if vol < 0:
            vol = 0
        # checknum = (T_CARTYPE + T_FUNC + T_LEN + vol) % 256
        # data = "$%02x%02x%02x%02x%02x#" % (T_CARTYPE, T_FUNC, T_LEN, vol, checknum)
        # tcp.send(data.encode(encoding="utf-8"))
        if self.g_debug:
            print("voltage:", vol / 10.0)
            # print("tcp send:", data)

        msg.vol = vol / 10.0
        self.bat_stat_pub.publish(msg)
        self.get_logger().info("Publishing battery_stat: {}".format(msg))