import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State, VFR_HUD
from sensor_msgs.msg import FluidPressure
import csv
from datetime import datetime

class LoggerNode(Node):
    def __init__(self):
        super().__init__('logger_node')
        filename = f"ros2_pixhawk_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.csv_file = open(filename, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(['Timestamp', 'Mode', 'Altitude', 'Heading'])

        self.create_subscription(State, '/mavros/state', self.state_callback, 10)
        self.create_subscription(VFR_HUD, '/mavros/vfr_hud', self.hud_callback, 10)
        
        self.current_state = ""
        self.get_logger().info(f"📝 Logging started: {filename}")

    def state_callback(self, msg):
        self.current_state = msg.mode

    def hud_callback(self, msg):
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.writer.writerow([timestamp, self.current_state, msg.altitude, msg.heading])
        # Print live like the original logger.py
        self.get_logger().info(f"[{timestamp}] Mode: {self.current_state} | Alt: {msg.altitude}")

    def __del__(self):
        self.csv_file.close()

def main(args=None):
    rclpy.init(args=args)
    node = LoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()