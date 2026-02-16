import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from mavros_msgs.msg import ManualControl
from mavros_msgs.srv import CommandBool, SetMode
import time

class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_node')
        
        # Publisher for movements
        self.control_pub = self.create_publisher(ManualControl, '/mavros/manual_control/send', 10)
        
        # Service clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for MAVROS services...')

    def set_mode(self, mode_str):
        req = SetMode.Request()
        req.custom_mode = mode_str
        self.set_mode_client.call_async(req)
        self.get_logger().info(f'🔄 Mode set to {mode_str}')

    def arm(self, status):
        req = CommandBool.Request()
        req.value = status
        self.arming_client.call_async(req)
        self.get_logger().info(f'🚀 Vehicle {"Armed" if status else "Disarmed"}')

    def send_control(self, x, y, z, r):
        msg = ManualControl()
        msg.x = float(x) # Forward/Backward
        msg.y = float(y) # Lateral
        msg.z = float(z) # Vertical (Thrust)
        msg.r = float(r) # Yaw
        self.control_pub.publish(msg)

    def run_mission(self):
        # 1. ARM
        self.arm(True)
        time.sleep(2)

        # 2. PHASE 1: DESCEND (MANUAL)
        self.set_mode("MANUAL")
        self.get_logger().info("⬇️ Descending for 5 seconds...")
        for _ in range(50): # 10Hz loop
            self.send_control(0, 0, 300, 0) # MAVROS z scales differently; 500 is neutral in many setups, check params
            time.sleep(0.1)

        # 3. PHASE 2: DEPTH HOLD
        self.set_mode("ALT_HOLD")
        self.get_logger().info("🤿 Holding depth for 5 seconds...")
        for _ in range(50):
            self.send_control(0, 0, 500, 0) # Neutral z
            time.sleep(0.1)

        # 4. PHASE 3: MOVE FORWARD
        self.get_logger().info("➡️ Moving forward for 10 seconds...")
        for _ in range(100):
            self.send_control(400, 0, 500, 0)
            time.sleep(0.1)

        # 5. PHASE 4: ASCEND
        self.set_mode("MANUAL")
        self.get_logger().info("⬆️ Ascending for 5 seconds...")
        for _ in range(50):
            self.send_control(0, 0, 700, 0) # Upward thrust
            time.sleep(0.1)

        # 6. STOP & DISARM
        self.send_control(0, 0, 500, 0)
        self.arm(False)
        self.get_logger().info("✅ Mission complete.")

def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    node.run_mission()
    rclpy.shutdown()