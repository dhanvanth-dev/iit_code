"""
DONE state — mission complete, neutral thrust and disarm.

Pure Python, no ROS dependencies.
"""

from aquavision.states.base_state import BaseState, ControlCmd, neutral_cmd


class DoneState(BaseState):
    """Neutral thrust and send disarm command. Terminal state."""

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
            cmd['arm'] = 0  # Disarm
            self._disarm_sent = True

        return cmd
