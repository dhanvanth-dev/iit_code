"""
FAILSAFE state — neutral thrust, disarm, log error.

Entered on MAVLink heartbeat loss or unrecoverable error.
Pure Python, no ROS dependencies.
"""

from aquavision.states.base_state import BaseState, ControlCmd, neutral_cmd


class FailsafeState(BaseState):
    """Emergency neutral + disarm. Terminal state."""

    def __init__(self, config: dict) -> None:
        super().__init__(config)
        self._disarm_sent: bool = False

    def on_enter(self) -> None:
        super().on_enter()
        self._disarm_sent = False

    def update(self, perception: dict, dt: float) -> ControlCmd:
        self._elapsed += dt
        cmd = neutral_cmd(self._config.get('neutral_z', 500))

        if not self._disarm_sent:
            cmd['arm'] = 0  # Disarm immediately
            self._disarm_sent = True

        return cmd
