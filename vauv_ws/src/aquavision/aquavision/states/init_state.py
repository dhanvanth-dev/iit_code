"""
INIT state — wait for MAVLink connection and arm the vehicle.

Pure Python, no ROS dependencies.
"""

from aquavision.states.base_state import BaseState, ControlCmd, neutral_cmd


class InitState(BaseState):
    """Wait for MAVLink connection and armed confirmation.

    Sends arm command (arm=1) once connected.
    Transitions to DESCEND once connected AND armed.
    """

    def update(self, perception: dict, dt: float) -> ControlCmd:
        self._elapsed += dt
        cmd = neutral_cmd(self._config.get('neutral_z', 500))

        if not perception.get('mav_connected', False):
            return cmd

        if not perception.get('mav_armed', False):
            # Request arming
            cmd['arm'] = 1
            return cmd

        # Connected and armed — transition
        self._next_state = 'DESCEND'
        return cmd
