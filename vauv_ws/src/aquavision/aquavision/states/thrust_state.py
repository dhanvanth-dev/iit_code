"""
THRUST state — move forward while maintaining PID correction.

Transitions to BLIND_PASS when gate fills most of the frame.
Pure Python, no ROS dependencies.
"""

from aquavision.states.base_state import BaseState, ControlCmd, neutral_cmd
from aquavision.behaviors.yaw_control import compute_yaw
from aquavision.behaviors.vertical_control import compute_vertical
from aquavision.behaviors.forward_motion import compute_forward
from aquavision.utils.pid import PIDController


class ThrustState(BaseState):
    """Forward thrust with PID correction. Transitions to BLIND_PASS."""

    def __init__(self, config: dict) -> None:
        super().__init__(config)
        self._yaw_pid = PIDController(
            kp=config.get('pid_yaw_kp', 400.0),
            ki=config.get('pid_yaw_ki', 0.0),
            kd=config.get('pid_yaw_kd', 50.0),
            output_min=-1000.0,
            output_max=1000.0,
            deadband=0.02,
        )
        self._vert_pid = PIDController(
            kp=config.get('pid_vertical_kp', 300.0),
            ki=config.get('pid_vertical_ki', 0.0),
            kd=config.get('pid_vertical_kd', 30.0),
            output_min=-500.0,
            output_max=500.0,
            deadband=0.02,
        )

    def on_enter(self) -> None:
        super().on_enter()
        self._yaw_pid.reset()
        self._vert_pid.reset()

    def update(self, perception: dict, dt: float) -> ControlCmd:
        self._elapsed += dt
        nz: int = self._config.get('neutral_z', 500)
        fwd_speed: int = self._config.get('thrust_forward_speed', 400)
        cmd = neutral_cmd(nz)

        cmd['x'] = compute_forward(fwd_speed)

        if perception.get('gate_visible', False):
            # Maintain PID correction while approaching
            cmd['r'] = compute_yaw(
                perception.get('error_x', 0.0), dt, self._yaw_pid
            )
            cmd['z'] = compute_vertical(
                perception.get('error_y', 0.0), dt, self._vert_pid, nz
            )

        # Transition when gate fills frame
        width_thresh: float = self._config.get('thrust_width_threshold', 0.8)
        if perception.get('width_ratio', 0.0) > width_thresh:
            self._next_state = 'BLIND_PASS'

        return cmd
