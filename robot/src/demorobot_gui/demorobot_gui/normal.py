import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, String
from sensor_msgs.msg import Image
from demorobot_msg.msg import BatteryStat

from scripts.main_gui import *

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

        self.ros2_worker = ROS2Worker(self)
        self.ros2_worker.start()

        self.app = QApplication(sys.argv)
        self.gui = MainWindow()
        self.gui.show()

def main(args=None):
    rclpy.init(args=args)
    normal_mode = NormalMode()
    sys.exit(normal_mode.app.exec_())

if __name__ == '__main__':
    main()