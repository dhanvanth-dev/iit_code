"""
Mission Node for Aquavision 2.0.

Deterministic state machine for autonomous gate traversal.
All timing via ROS timers and clock comparisons — no blocking loops
or time.sleep calls.

States: INIT → DESCEND → STABILIZE → SEARCH → ALIGN → THRUST → BLIND_PASS → DONE
        Any state → FAILSAFE (on heartbeat loss)
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Quaternion
from aquavision_msgs.msg import GateData

from aquavision.utils.pid import PIDController
from aquavision.utils.csv_logger import CSVLogger

from enum import Enum
from typing import Optional


class MissionState(Enum):
    """Mission state machine states."""
    INIT = 'INIT'
    DESCEND = 'DESCEND'
    STABILIZE = 'STABILIZE'
    SEARCH = 'SEARCH'
    ALIGN = 'ALIGN'
    THRUST = 'THRUST'
    BLIND_PASS = 'BLIND_PASS'
    DONE = 'DONE'
    FAILSAFE = 'FAILSAFE'


class MissionNode(Node):
    """ROS 2 deterministic state machine for gate traversal.

    Drives the AUV through the full mission sequence using timer-based
    state transitions. Publishes control commands to nav_bridge_node
    and subscribes to vision data from vision_node.

    Parameters:
        tick_rate (float): State machine tick rate in Hz.
        descend_duration (float): Seconds to descend.
        descend_thrust (int): Z thrust during descent (>500 = down).
        stabilize_duration (float): Seconds to stabilize.
        search_yaw_speed (int): Yaw rate during search rotation.
        align_threshold_x (float): Max |error_x| to consider aligned.
        align_threshold_y (float): Max |error_y| to consider aligned.
        align_stable_count (int): Consecutive aligned ticks required.
        thrust_forward_speed (int): Forward speed during thrust phase.
        thrust_width_threshold (float): width_ratio to trigger blind pass.
        blind_pass_duration (float): Seconds of blind forward thrust.
        blind_pass_speed (int): Forward speed during blind pass.
        pid_yaw_kp/ki/kd (float): PID gains for yaw correction.
        pid_vertical_kp/ki/kd (float): PID gains for vertical correction.
    """

    def __init__(self) -> None:
        super().__init__('mission_node')

        # ── Declare all parameters ──────────────────────────────────────
        self.declare_parameter('tick_rate', 20.0)
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

        # Read parameters
        self._tick_rate: float = self._get_double('tick_rate')
        self._descend_dur: float = self._get_double('descend_duration')
        self._descend_thrust: int = self._get_int('descend_thrust')
        self._stabilize_dur: float = self._get_double('stabilize_duration')
        self._search_yaw: int = self._get_int('search_yaw_speed')
        self._align_thresh_x: float = self._get_double('align_threshold_x')
        self._align_thresh_y: float = self._get_double('align_threshold_y')
        self._align_stable_needed: int = self._get_int('align_stable_count')
        self._thrust_fwd: int = self._get_int('thrust_forward_speed')
        self._thrust_width_thresh: float = self._get_double('thrust_width_threshold')
        self._blind_dur: float = self._get_double('blind_pass_duration')
        self._blind_speed: int = self._get_int('blind_pass_speed')
        self._neutral_z: int = self._get_int('neutral_z')

        # ── PID controllers ─────────────────────────────────────────────
        self._yaw_pid = PIDController(
            kp=self._get_double('pid_yaw_kp'),
            ki=self._get_double('pid_yaw_ki'),
            kd=self._get_double('pid_yaw_kd'),
            output_min=-1000.0,
            output_max=1000.0,
            deadband=0.02,
        )
        self._vert_pid = PIDController(
            kp=self._get_double('pid_vertical_kp'),
            ki=self._get_double('pid_vertical_ki'),
            kd=self._get_double('pid_vertical_kd'),
            output_min=-500.0,  # Maps to z offset from neutral
            output_max=500.0,
            deadband=0.02,
        )

        # ── State machine ──────────────────────────────────────────────
        self._state: MissionState = MissionState.INIT
        self._state_entry_time: Optional[Time] = None
        self._align_stable_counter: int = 0

        # ── Latest gate data ────────────────────────────────────────────
        self._gate_error_x: float = 0.0
        self._gate_error_y: float = 0.0
        self._gate_width_ratio: float = 0.0
        self._gate_visible: bool = False

        # ── MAVLink state ───────────────────────────────────────────────
        self._mav_connected: bool = False
        self._mav_armed: bool = False

        # ── Current commands being sent ─────────────────────────────────
        self._cur_x: int = 0
        self._cur_y: int = 0
        self._cur_z: int = self._neutral_z
        self._cur_r: int = 0

        # ── CSV Logger ──────────────────────────────────────────────────
        self.csv_logger = CSVLogger()
        self.get_logger().info(
            f'📝 Logging to: {self.csv_logger.filepath}'
        )

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

        # ── Main tick timer ─────────────────────────────────────────────
        self._last_tick_time: Optional[Time] = None
        self._tick_timer = self.create_timer(
            1.0 / max(self._tick_rate, 1.0), self._tick
        )

        self.get_logger().info(
            f'🧠 Mission node started | rate={self._tick_rate}Hz | '
            f'state={self._state.value}'
        )

    # ── Parameter helpers ────────────────────────────────────────────────

    def _get_double(self, name: str) -> float:
        return self.get_parameter(name).get_parameter_value().double_value

    def _get_int(self, name: str) -> int:
        return self.get_parameter(name).get_parameter_value().integer_value

    # ── Subscriber callbacks ─────────────────────────────────────────────

    def _gate_data_callback(self, msg: GateData) -> None:
        """Update latest gate detection data."""
        self._gate_error_x = msg.error_x
        self._gate_error_y = msg.error_y
        self._gate_width_ratio = msg.width_ratio
        self._gate_visible = msg.gate_visible

    def _mav_state_callback(self, msg: String) -> None:
        """Parse MAVLink state string from nav_bridge_node."""
        # Format: "connected=True,armed=False"
        try:
            parts = msg.data.split(',')
            for part in parts:
                key, val = part.strip().split('=')
                if key == 'connected':
                    self._mav_connected = val == 'True'
                elif key == 'armed':
                    self._mav_armed = val == 'True'
        except Exception:
            pass

    # ── Main state machine tick ──────────────────────────────────────────

    def _tick(self) -> None:
        """Execute one state machine tick. Called by timer."""
        now: Time = self.get_clock().now()

        # Compute dt
        dt: float = 0.05  # default
        if self._last_tick_time is not None:
            dt = max(
                (now - self._last_tick_time).nanoseconds / 1e9, 0.001
            )
        self._last_tick_time = now

        # Time in current state
        state_elapsed: float = 0.0
        if self._state_entry_time is not None:
            state_elapsed = (
                (now - self._state_entry_time).nanoseconds / 1e9
            )

        # Dispatch to state handler
        state_handler = {
            MissionState.INIT: self._state_init,
            MissionState.DESCEND: self._state_descend,
            MissionState.STABILIZE: self._state_stabilize,
            MissionState.SEARCH: self._state_search,
            MissionState.ALIGN: self._state_align,
            MissionState.THRUST: self._state_thrust,
            MissionState.BLIND_PASS: self._state_blind_pass,
            MissionState.DONE: self._state_done,
            MissionState.FAILSAFE: self._state_failsafe,
        }

        handler = state_handler.get(self._state)
        if handler:
            handler(dt, state_elapsed)

        # Publish control command
        self._publish_control()

        # CSV logging
        self.csv_logger.log_row(
            state=self._state.value,
            error_x=self._gate_error_x,
            error_y=self._gate_error_y,
            width_ratio=self._gate_width_ratio,
            thrust_x=float(self._cur_x),
            thrust_y=float(self._cur_y),
            thrust_z=float(self._cur_z),
            thrust_r=float(self._cur_r),
        )

    # ── State handlers ───────────────────────────────────────────────────

    def _state_init(self, dt: float, elapsed: float) -> None:
        """INIT: Wait for MAVLink connection, then arm."""
        self._set_neutral()

        if not self._mav_connected:
            return

        # Connection established — arm the vehicle
        if not self._mav_armed:
            self.get_logger().info('🔑 MAVLink connected — sending arm command')
            arm_msg = Bool()
            arm_msg.data = True
            self._arm_pub.publish(arm_msg)
            return

        # Armed and connected — start mission
        self.get_logger().info('🚀 Armed — starting descent')
        self._transition_to(MissionState.DESCEND)

    def _state_descend(self, dt: float, elapsed: float) -> None:
        """DESCEND: Apply downward thrust for configured duration."""
        self._cur_x = 0
        self._cur_y = 0
        self._cur_z = self._descend_thrust  # >500 = down
        self._cur_r = 0

        if elapsed >= self._descend_dur:
            self.get_logger().info('⬇️ Descent complete — stabilizing')
            self._transition_to(MissionState.STABILIZE)

    def _state_stabilize(self, dt: float, elapsed: float) -> None:
        """STABILIZE: Neutral thrust to settle."""
        self._set_neutral()

        if elapsed >= self._stabilize_dur:
            self.get_logger().info('⚖️ Stabilized — searching for gate')
            self._transition_to(MissionState.SEARCH)

    def _state_search(self, dt: float, elapsed: float) -> None:
        """SEARCH: Rotate slowly until gate is visible."""
        self._cur_x = 0
        self._cur_y = 0
        self._cur_z = self._neutral_z
        self._cur_r = self._search_yaw  # slow rotation

        if self._gate_visible:
            self.get_logger().info('👁️ Gate detected — aligning')
            self._yaw_pid.reset()
            self._vert_pid.reset()
            self._align_stable_counter = 0
            self._transition_to(MissionState.ALIGN)

    def _state_align(self, dt: float, elapsed: float) -> None:
        """ALIGN: PID-controlled yaw and vertical to center the gate."""
        if not self._gate_visible:
            # Lost gate during alignment — go back to search
            self.get_logger().warn('⚠️ Gate lost during alignment — searching')
            self._transition_to(MissionState.SEARCH)
            return

        # PID corrections
        yaw_output: float = self._yaw_pid.compute(self._gate_error_x, dt)
        vert_output: float = self._vert_pid.compute(self._gate_error_y, dt)

        self._cur_x = 0
        self._cur_y = 0
        self._cur_r = int(yaw_output)
        # Vertical: positive vert_output means gate is below → need to go down
        # z > 500 = down, z < 500 = up
        self._cur_z = self._neutral_z + int(vert_output)
        self._cur_z = max(0, min(1000, self._cur_z))

        # Check alignment
        x_aligned: bool = abs(self._gate_error_x) < self._align_thresh_x
        y_aligned: bool = abs(self._gate_error_y) < self._align_thresh_y

        if x_aligned and y_aligned:
            self._align_stable_counter += 1
        else:
            self._align_stable_counter = 0

        if self._align_stable_counter >= self._align_stable_needed:
            self.get_logger().info(
                f'✅ Aligned (stable for {self._align_stable_needed} ticks) '
                f'— thrusting forward'
            )
            self._transition_to(MissionState.THRUST)

    def _state_thrust(self, dt: float, elapsed: float) -> None:
        """THRUST: Move forward while maintaining vertical PID correction."""
        if not self._gate_visible:
            # Gate probably very close and partially out of frame
            # — continue forward anyway
            self._cur_x = self._thrust_fwd
            self._cur_y = 0
            self._cur_z = self._neutral_z
            self._cur_r = 0
        else:
            # Maintain vertical correction while moving forward
            vert_output: float = self._vert_pid.compute(
                self._gate_error_y, dt
            )
            yaw_output: float = self._yaw_pid.compute(
                self._gate_error_x, dt
            )

            self._cur_x = self._thrust_fwd
            self._cur_y = 0
            self._cur_r = int(yaw_output)
            self._cur_z = self._neutral_z + int(vert_output)
            self._cur_z = max(0, min(1000, self._cur_z))

        if self._gate_width_ratio > self._thrust_width_thresh:
            self.get_logger().info(
                f'🚪 Gate width ratio {self._gate_width_ratio:.2f} > '
                f'{self._thrust_width_thresh} — blind pass'
            )
            self._transition_to(MissionState.BLIND_PASS)

    def _state_blind_pass(self, dt: float, elapsed: float) -> None:
        """BLIND_PASS: Full forward thrust, no vision correction."""
        self._cur_x = self._blind_speed
        self._cur_y = 0
        self._cur_z = self._neutral_z
        self._cur_r = 0

        if elapsed >= self._blind_dur:
            self.get_logger().info('🏁 Blind pass complete — mission DONE')
            self._transition_to(MissionState.DONE)

    def _state_done(self, dt: float, elapsed: float) -> None:
        """DONE: Neutral thrust, disarm."""
        self._set_neutral()

        if elapsed < 0.5:
            # Disarm once
            arm_msg = Bool()
            arm_msg.data = False
            self._arm_pub.publish(arm_msg)
            self.get_logger().info('✅ Mission complete — disarmed')

    def _state_failsafe(self, dt: float, elapsed: float) -> None:
        """FAILSAFE: Neutral thrust. Waiting for manual intervention."""
        self._set_neutral()

        # Attempt disarm
        if elapsed < 0.5:
            arm_msg = Bool()
            arm_msg.data = False
            self._arm_pub.publish(arm_msg)
            self.get_logger().error('🚨 FAILSAFE — disarmed, awaiting operator')

    # ── Helpers ──────────────────────────────────────────────────────────

    def _transition_to(self, new_state: MissionState) -> None:
        """Transition to a new state, recording entry time."""
        self.get_logger().info(
            f'📍 {self._state.value} → {new_state.value}'
        )
        self._state = new_state
        self._state_entry_time = self.get_clock().now()

    def _set_neutral(self) -> None:
        """Set all commands to neutral (no movement)."""
        self._cur_x = 0
        self._cur_y = 0
        self._cur_z = self._neutral_z
        self._cur_r = 0

    def _publish_control(self) -> None:
        """Publish current command values via Quaternion message."""
        msg = Quaternion()
        msg.x = float(self._cur_x)
        msg.y = float(self._cur_y)
        msg.z = float(self._cur_z)
        msg.w = float(self._cur_r)
        self._cmd_pub.publish(msg)

    def destroy_node(self) -> None:
        """Clean up resources on shutdown."""
        self.csv_logger.close()
        super().destroy_node()


def main(args=None) -> None:
    """Entry point for the mission node."""
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
