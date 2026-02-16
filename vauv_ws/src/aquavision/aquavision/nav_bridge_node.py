"""
Navigation Bridge Node for Aquavision 2.0.

Pure pymavlink bridge between ROS 2 and Pixhawk (ArduSub).
Handles MAVLink connection, arming, manual control, heartbeat,
and telemetry — all non-blocking via ROS timers.

Reference: Tested pymavlink patterns from the team's working control.py.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Quaternion
from aquavision_msgs.msg import GateData

from pymavlink import mavutil
from typing import Optional
import time as _time


class NavBridgeNode(Node):
    """ROS 2 ↔ Pixhawk pymavlink bridge.

    All MAVLink I/O is driven by ROS timers (no blocking loops).
    Uses the proven manual_control_send pattern from the team's
    tested control.py.

    Parameters:
        mavlink_connection (str): Pymavlink connection string.
        mavlink_baud (int): Serial baud rate.
        heartbeat_rate (float): Heartbeat send/check rate in Hz.
        telemetry_rate (float): Telemetry poll rate in Hz.
        control_rate (float): Manual control send rate in Hz.
    """

    def __init__(self) -> None:
        super().__init__('nav_bridge_node')

        # ── Parameters ──────────────────────────────────────────────────
        self.declare_parameter('mavlink_connection', '/dev/ttyACM0')
        self.declare_parameter('mavlink_baud', 115200)
        self.declare_parameter('heartbeat_rate', 1.0)
        self.declare_parameter('telemetry_rate', 10.0)
        self.declare_parameter('control_rate', 20.0)

        conn_str: str = (
            self.get_parameter('mavlink_connection')
            .get_parameter_value().string_value
        )
        baud: int = (
            self.get_parameter('mavlink_baud')
            .get_parameter_value().integer_value
        )
        hb_rate: float = (
            self.get_parameter('heartbeat_rate')
            .get_parameter_value().double_value
        )
        telem_rate: float = (
            self.get_parameter('telemetry_rate')
            .get_parameter_value().double_value
        )
        ctrl_rate: float = (
            self.get_parameter('control_rate')
            .get_parameter_value().double_value
        )

        # ── MAVLink connection ──────────────────────────────────────────
        self._conn_str: str = conn_str
        self._baud: int = baud
        self._master: Optional[mavutil.mavlink_connection] = None
        self._connected: bool = False
        self._armed: bool = False
        self._last_heartbeat_time: float = 0.0
        self._depth: float = 0.0

        self.get_logger().info(
            f'🔌 Connecting to Pixhawk: {conn_str} @ {baud}'
        )
        self._attempt_connection()

        # ── Current control commands ────────────────────────────────────
        # ArduSub manual_control_send ranges:
        #   x, y, r: -1000 to 1000
        #   z: 0 to 1000 (500 = neutral, <500 = up, >500 = down)
        self._cmd_x: int = 0       # forward/backward
        self._cmd_y: int = 0       # lateral
        self._cmd_z: int = 500     # vertical (500 = neutral)
        self._cmd_r: int = 0       # yaw
        self._cmd_buttons: int = 0

        # ── Publishers ──────────────────────────────────────────────────
        self._state_pub = self.create_publisher(
            String, '/vauv/mav_state', 10
        )

        # ── Subscribers ─────────────────────────────────────────────────
        self.create_subscription(
            Quaternion, '/vauv/cmd_manual',
            self._cmd_manual_callback, 10
        )
        self.create_subscription(
            Bool, '/vauv/arm_cmd',
            self._arm_callback, 10
        )

        # ── Timers (non-blocking) ───────────────────────────────────────
        self._hb_timer = self.create_timer(
            1.0 / max(hb_rate, 0.1), self._heartbeat_tick
        )
        self._telem_timer = self.create_timer(
            1.0 / max(telem_rate, 1.0), self._telemetry_tick
        )
        self._ctrl_timer = self.create_timer(
            1.0 / max(ctrl_rate, 1.0), self._control_tick
        )

        # ── Reconnect timer (active only when disconnected) ─────────────
        self._reconnect_timer: Optional[object] = None
        if self._master is None:
            self._start_reconnect_timer()

        self.get_logger().info(
            f'🚀 NavBridge started | HB={hb_rate}Hz | '
            f'Telem={telem_rate}Hz | Ctrl={ctrl_rate}Hz'
        )

    # ── Connection management ────────────────────────────────────────────

    def _attempt_connection(self) -> None:
        """Try to establish MAVLink serial connection."""
        try:
            self._master = mavutil.mavlink_connection(
                self._conn_str, baud=self._baud
            )
            self.get_logger().info('📡 MAVLink connection object created')
        except Exception as e:
            self.get_logger().warn(
                f'⚠️ MAVLink connection failed: {e} — will retry'
            )
            self._master = None

    def _start_reconnect_timer(self) -> None:
        """Start a 2-second timer to retry MAVLink connection."""
        if self._reconnect_timer is not None:
            return  # Already running
        self._reconnect_timer = self.create_timer(
            2.0, self._reconnect_tick
        )
        self.get_logger().warn(
            '🔄 MAVLink reconnect timer started (every 2s)'
        )

    def _reconnect_tick(self) -> None:
        """Attempt to reconnect to MAVLink. Cancels itself on success."""
        if self._master is not None:
            # Already connected — cancel timer
            if self._reconnect_timer is not None:
                self._reconnect_timer.cancel()
                self._reconnect_timer = None
            return

        self.get_logger().warn(
            f'⚠️ MAVLink: retrying connection to {self._conn_str}...'
        )
        self._attempt_connection()

        if self._master is not None:
            self.get_logger().info(
                '✅ MAVLink reconnected successfully'
            )
            if self._reconnect_timer is not None:
                self._reconnect_timer.cancel()
                self._reconnect_timer = None

    # ── Subscriber Callbacks ─────────────────────────────────────────────

    def _cmd_manual_callback(self, msg: Quaternion) -> None:
        """Receive manual control commands.

        Quaternion fields mapped as:
            x → forward/backward (-1000 to 1000)
            y → lateral (-1000 to 1000)
            z → vertical (0 to 1000, 500=neutral)
            w → yaw (-1000 to 1000)
        """
        self._cmd_x = self._clamp_symmetric(int(msg.x))
        self._cmd_y = self._clamp_symmetric(int(msg.y))
        self._cmd_z = self._clamp_vertical(int(msg.z))
        self._cmd_r = self._clamp_symmetric(int(msg.w))

    def _arm_callback(self, msg: Bool) -> None:
        """Arm or disarm the vehicle."""
        if self._master is None:
            self.get_logger().warn('Cannot arm — no MAVLink connection')
            return

        arm_value: int = 1 if msg.data else 0
        action: str = 'Arming' if msg.data else 'Disarming'
        self.get_logger().info(f'🔑 {action} vehicle...')

        try:
            self._master.mav.command_long_send(
                self._master.target_system,
                self._master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,           # confirmation
                arm_value,   # param1: 1=arm, 0=disarm
                0, 0, 0, 0, 0, 0
            )
            self._armed = msg.data
            self.get_logger().info(
                f'{"🚀 Armed" if msg.data else "🛑 Disarmed"}'
            )
        except Exception as e:
            self.get_logger().error(f'Arm/disarm failed: {e}')

    # ── Timer Callbacks (non-blocking) ───────────────────────────────────

    def _heartbeat_tick(self) -> None:
        """Send heartbeat and check for incoming heartbeat."""
        if self._master is None:
            return

        # Send GCS heartbeat to keep connection alive
        try:
            self._master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0
            )
        except Exception as e:
            self.get_logger().warn(f'Heartbeat send failed: {e}')

        # Check connection health
        now: float = _time.monotonic()
        if self._connected and (now - self._last_heartbeat_time > 5.0):
            self.get_logger().warn('⚠️ Heartbeat timeout — connection lost')
            self._connected = False

        # Publish state
        state_msg = String()
        state_msg.data = (
            f'connected={self._connected},armed={self._armed}'
        )
        self._state_pub.publish(state_msg)

    def _telemetry_tick(self) -> None:
        """Poll MAVLink for incoming messages (non-blocking)."""
        if self._master is None:
            return

        # Drain all available messages without blocking
        while True:
            msg = self._master.recv_match(blocking=False)
            if msg is None:
                break

            msg_type: str = msg.get_type()

            if msg_type == 'HEARTBEAT':
                self._last_heartbeat_time = _time.monotonic()
                if not self._connected:
                    self.get_logger().info('💓 Heartbeat received — connected')
                    self._connected = True

            elif msg_type == 'VFR_HUD':
                # ArduSub reports depth via VFR_HUD alt field
                self._depth = getattr(msg, 'alt', 0.0)

            elif msg_type == 'ATTITUDE':
                # Available for future use (roll, pitch, yaw)
                pass

    def _control_tick(self) -> None:
        """Send manual_control command at configured rate."""
        if self._master is None or not self._connected:
            return

        try:
            self._master.mav.manual_control_send(
                self._master.target_system,
                self._cmd_x,     # forward/backward
                self._cmd_y,     # lateral
                self._cmd_z,     # vertical (0-1000, 500=neutral)
                self._cmd_r,     # yaw
                self._cmd_buttons
            )
        except Exception as e:
            self.get_logger().warn(f'Control send failed: {e}')

    # ── Property accessors ──────────────────────────────────────────────

    @property
    def depth(self) -> float:
        """Current depth reading from VFR_HUD."""
        return self._depth

    @property
    def connected(self) -> bool:
        """Whether MAVLink heartbeat is active."""
        return self._connected

    @property
    def armed(self) -> bool:
        """Whether vehicle is armed."""
        return self._armed

    # ── Helpers ──────────────────────────────────────────────────────────

    @staticmethod
    def _clamp_symmetric(value: int) -> int:
        """Clamp value to [-1000, 1000]."""
        return max(-1000, min(1000, value))

    @staticmethod
    def _clamp_vertical(value: int) -> int:
        """Clamp vertical value to [0, 1000] (ArduSub z-axis range)."""
        return max(0, min(1000, value))


def main(args=None) -> None:
    """Entry point for the navigation bridge node."""
    rclpy.init(args=args)
    node = NavBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
