"""
ALIGN state — PID-controlled alignment to gate center.

Uses yaw_control and vertical_control behaviors for reusable PID.
Pure Python, no ROS dependencies.
"""

from aquavision.states.base_state import BaseState, ControlCmd, neutral_cmd
from aquavision.behaviors.yaw_control import compute_yaw
from aquavision.behaviors.vertical_control import compute_vertical
from aquavision.utils.pid import PIDController


class AlignState(BaseState):
    """PID alignment on gate center. Transitions to THRUST when stable."""

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
        self._stable_counter: int = 0

    def on_enter(self) -> None:
        super().on_enter()
        self._yaw_pid.reset()
        self._vert_pid.reset()
        self._stable_counter = 0

    def update(self, perception: dict, dt: float) -> ControlCmd:
        self._elapsed += dt
        nz: int = self._config.get('neutral_z', 500)
        cmd = neutral_cmd(nz)

        if not perception.get('gate_visible', False):
            # Gate lost during alignment — fall back to search
            self._next_state = 'SEARCH'
            return cmd

        error_x: float = perception.get('error_x', 0.0)
        error_y: float = perception.get('error_y', 0.0)

        # Use behavior modules for PID computation
        cmd['r'] = compute_yaw(error_x, dt, self._yaw_pid)
        cmd['z'] = compute_vertical(error_y, dt, self._vert_pid, nz)

        # Check alignment stability
        thresh_x: float = self._config.get('align_threshold_x', 0.05)
        thresh_y: float = self._config.get('align_threshold_y', 0.05)

        if abs(error_x) < thresh_x and abs(error_y) < thresh_y:
            self._stable_counter += 1
        else:
            self._stable_counter = 0

        stable_needed: int = self._config.get('align_stable_count', 10)
        if self._stable_counter >= stable_needed:
            self._next_state = 'THRUST'

        return cmd
