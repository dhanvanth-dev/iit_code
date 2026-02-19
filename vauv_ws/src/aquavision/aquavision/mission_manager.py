"""
Mission Manager for Aquavision 2.0.

ROS 2 interface that drives the deterministic state machine at 30 Hz.
All mission logic lives in pure Python state classes — this node only
handles ROS I/O (subscriptions, publishers, timers, logging).

Compliant with architecture.md: single mission node, internal modularity,
no ROS inside states.
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Quaternion
from aquavision_msgs.msg import GateData

from aquavision.states import (
    BaseState, InitState, DescendState, StabilizeState,
    SearchState, AlignState, ThrustState, BlindPassState,
    DoneState, FailsafeState,
)
from aquavision.utils.csv_logger import CSVLogger

from typing import Dict, Optional
import time as _time


# State name → state class mapping
STATE_CLASSES: Dict[str, type] = {
    'INIT': InitState,
    'DESCEND': DescendState,
    'STABILIZE': StabilizeState,
    'SEARCH': SearchState,
    'ALIGN': AlignState,
    'THRUST': ThrustState,
    'BLIND_PASS': BlindPassState,
    'DONE': DoneState,
    'FAILSAFE': FailsafeState,
}


class MissionManager(Node):
    """ROS 2 node that drives the mission state machine.

    Owns state instances, calls update() each tick, handles ROS I/O.
    The state classes contain no ROS code — they receive perception
    dicts and return control command dicts.

    Parameters:
        tick_rate (float): State machine update rate in Hz.
        descend_duration/thrust/stabilize_duration: Timed state params.
        search_yaw_speed: Yaw rate during search.
        align_threshold_x/y, align_stable_count: Alignment params.
        thrust_forward_speed, thrust_width_threshold: Approach params.
        blind_pass_duration/speed: Blind pass params.
        pid_yaw_kp/ki/kd, pid_vertical_kp/ki/kd: PID gains.
        neutral_z: Neutral vertical thrust (500).
        perception_timeout (float): Seconds without gate data before
            commanding neutral thrust and logging a warning.
    """

    def __init__(self) -> None:
        super().__init__('mission_manager')

        # ── Declare all parameters ──────────────────────────────────────
        self.declare_parameter('tick_rate', 30.0)
        self.declare_parameter('descend_duration', 5.0)
        self.declare_parameter('descend_thrust', 700)
        self.declare_parameter('stabilize_duration', 3.0)
        self.declare_parameter('search_yaw_speed', 200)
        self.declare_parameter('align_threshold_x', 0.05)
        self.declare_parameter('align_threshold_y', 0.05)
        self.declare_parameter('align_stable_count', 10)
        self.declare_parameter('thrust_forward_speed', 400)
        self.declare_parameter('thrust_width_threshold', 0.8)
        self.declare_parameter('blind_pass_duration', 5.0)
        self.declare_parameter('blind_pass_speed', 500)
        self.declare_parameter('pid_yaw_kp', 400.0)
        self.declare_parameter('pid_yaw_ki', 0.0)
        self.declare_parameter('pid_yaw_kd', 50.0)
        self.declare_parameter('pid_vertical_kp', 300.0)
        self.declare_parameter('pid_vertical_ki', 0.0)
        self.declare_parameter('pid_vertical_kd', 30.0)
        self.declare_parameter('neutral_z', 500)
        self.declare_parameter('perception_timeout', 2.0)

        # Build config dict for state classes (no ROS objects)
        self._config: dict = {
            'tick_rate': self._get_double('tick_rate'),
            'descend_duration': self._get_double('descend_duration'),
            'descend_thrust': self._get_int('descend_thrust'),
            'stabilize_duration': self._get_double('stabilize_duration'),
            'search_yaw_speed': self._get_int('search_yaw_speed'),
            'align_threshold_x': self._get_double('align_threshold_x'),
            'align_threshold_y': self._get_double('align_threshold_y'),
            'align_stable_count': self._get_int('align_stable_count'),
            'thrust_forward_speed': self._get_int('thrust_forward_speed'),
            'thrust_width_threshold': self._get_double('thrust_width_threshold'),
            'blind_pass_duration': self._get_double('blind_pass_duration'),
            'blind_pass_speed': self._get_int('blind_pass_speed'),
            'pid_yaw_kp': self._get_double('pid_yaw_kp'),
            'pid_yaw_ki': self._get_double('pid_yaw_ki'),
            'pid_yaw_kd': self._get_double('pid_yaw_kd'),
            'pid_vertical_kp': self._get_double('pid_vertical_kp'),
            'pid_vertical_ki': self._get_double('pid_vertical_ki'),
            'pid_vertical_kd': self._get_double('pid_vertical_kd'),
            'neutral_z': self._get_int('neutral_z'),
        }
        self._perception_timeout: float = self._get_double('perception_timeout')

        # ── Instantiate all states ──────────────────────────────────────
        self._states: Dict[str, BaseState] = {
            name: cls(self._config) for name, cls in STATE_CLASSES.items()
        }
        self._current_state_name: str = 'INIT'
        self._current_state: BaseState = self._states['INIT']
        self._current_state.on_enter()

        # ── Perception data (updated by subscriber) ─────────────────────
        self._perception: dict = {
            'error_x': 0.0,
            'error_y': 0.0,
            'width_ratio': 0.0,
            'gate_visible': False,
            'mav_connected': False,
            'mav_armed': False,
        }
        self._last_gate_data_time: float = _time.monotonic()
        self._perception_warned: bool = False

        # ── CSV Logger ──────────────────────────────────────────────────
        self._logger = CSVLogger()
        self.get_logger().info(f'📝 Logging to: {self._logger.filepath}')

        # ── Publishers ──────────────────────────────────────────────────
        self._cmd_pub = self.create_publisher(
            Quaternion, '/vauv/cmd_manual', 10
        )
        self._arm_pub = self.create_publisher(
            Bool, '/vauv/arm_cmd', 10
        )

        # ── Subscribers ─────────────────────────────────────────────────
        self.create_subscription(
            GateData, '/vauv/gate_data',
            self._gate_data_callback, 10
        )
        self.create_subscription(
            String, '/vauv/mav_state',
            self._mav_state_callback, 10
        )

        # ── Main tick timer (30 Hz default) ─────────────────────────────
        tick_rate: float = self._config['tick_rate']
        self._last_tick_time: Optional[Time] = None
        self._tick_timer = self.create_timer(
            1.0 / max(tick_rate, 1.0), self._tick
        )

        self.get_logger().info(
            f'🧠 MissionManager started | rate={tick_rate}Hz | '
            f'state=INIT'
        )

    # ── Parameter helpers ────────────────────────────────────────────────

    def _get_double(self, name: str) -> float:
        return self.get_parameter(name).get_parameter_value().double_value

    def _get_int(self, name: str) -> int:
        return self.get_parameter(name).get_parameter_value().integer_value

    # ── Subscriber callbacks ─────────────────────────────────────────────

    def _gate_data_callback(self, msg: GateData) -> None:
        """Update perception data from vision node."""
        self._perception['error_x'] = msg.error_x
        self._perception['error_y'] = msg.error_y
        self._perception['width_ratio'] = msg.width_ratio
        self._perception['gate_visible'] = msg.gate_visible
        self._last_gate_data_time = _time.monotonic()
        self._perception_warned = False

    def _mav_state_callback(self, msg: String) -> None:
        """Parse MAVLink state from nav_bridge_node."""
        try:
            for part in msg.data.split(','):
                key, val = part.strip().split('=')
                if key == 'connected':
                    self._perception['mav_connected'] = val == 'True'
                elif key == 'armed':
                    self._perception['mav_armed'] = val == 'True'
        except Exception:
            pass

    # ── Main tick ────────────────────────────────────────────────────────

    def _tick(self) -> None:
        """Execute one state machine tick."""
        now: Time = self.get_clock().now()

        # Compute dt
        dt: float = 1.0 / self._config['tick_rate']  # default
        if self._last_tick_time is not None:
            dt = max(
                (now - self._last_tick_time).nanoseconds / 1e9, 0.001
            )
        self._last_tick_time = now

        # ── Perception timeout check ────────────────────────────────
        time_since_gate: float = (
            _time.monotonic() - self._last_gate_data_time
        )
        if time_since_gate > self._perception_timeout:
            if not self._perception_warned:
                self.get_logger().warn(
                    f'⚠️ No gate_data for {time_since_gate:.1f}s '
                    f'(timeout={self._perception_timeout}s)'
                )
                self._perception_warned = True
            # Mark gate as not visible during timeout
            self._perception['gate_visible'] = False

        # ── Failsafe check: MAVLink loss ────────────────────────────
        if (self._current_state_name not in ('INIT', 'DONE', 'FAILSAFE')
                and not self._perception.get('mav_connected', False)):
            self.get_logger().error(
                '🚨 MAVLink lost — entering FAILSAFE'
            )
            self._transition_to('FAILSAFE')

        # ── Run current state ───────────────────────────────────────
        try:
            cmd: dict = self._current_state.update(self._perception, dt)
        except Exception as e:
            self.get_logger().error(
                f'❌ State {self._current_state_name} exception: {e} '
                f'— commanding neutral'
            )
            nz: int = self._config.get('neutral_z', 500)
            cmd = {'x': 0, 'y': 0, 'z': nz, 'r': 0, 'arm': -1}

        # ── Publish control command ─────────────────────────────────
        self._publish_control(cmd)

        # ── Handle arm/disarm requests from state ───────────────────
        arm_val = cmd.get('arm', -1)
        if arm_val == 1:
            arm_msg = Bool()
            arm_msg.data = True
            self._arm_pub.publish(arm_msg)
        elif arm_val == 0:
            arm_msg = Bool()
            arm_msg.data = False
            self._arm_pub.publish(arm_msg)

        # ── CSV logging ─────────────────────────────────────────────
        self._logger.log_row(
            state=self._current_state_name,
            error_x=self._perception['error_x'],
            error_y=self._perception['error_y'],
            width_ratio=self._perception['width_ratio'],
            thrust_x=float(cmd.get('x', 0)),
            thrust_y=float(cmd.get('y', 0)),
            thrust_z=float(cmd.get('z', 500)),
            thrust_r=float(cmd.get('r', 0)),
        )

        # ── Check for state transition ──────────────────────────────
        next_name: Optional[str] = self._current_state.next_state
        if next_name is not None:
            self._transition_to(next_name)

    # ── Helpers ──────────────────────────────────────────────────────────

    def _transition_to(self, new_state_name: str) -> None:
        """Transition to a new state by name."""
        if new_state_name not in self._states:
            self.get_logger().error(
                f'❌ Unknown state: {new_state_name}'
            )
            return

        old_name: str = self._current_state_name
        self._current_state.on_exit()
        self._current_state_name = new_state_name
        self._current_state = self._states[new_state_name]
        self._current_state.on_enter()

        self.get_logger().info(f'📍 {old_name} → {new_state_name}')

    def _publish_control(self, cmd: dict) -> None:
        """Publish control command as Quaternion (x, y, z, w=r)."""
        msg = Quaternion()
        msg.x = float(cmd.get('x', 0))
        msg.y = float(cmd.get('y', 0))
        msg.z = float(cmd.get('z', self._config.get('neutral_z', 500)))
        msg.w = float(cmd.get('r', 0))
        self._cmd_pub.publish(msg)

    def destroy_node(self) -> None:
        """Clean up on shutdown."""
        self._logger.close()
        super().destroy_node()


def main(args=None) -> None:
    """Entry point for mission_manager node."""
    rclpy.init(args=args)
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
