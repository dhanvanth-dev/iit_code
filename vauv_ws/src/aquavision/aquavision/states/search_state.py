"""
SEARCH state — rotate slowly until the gate becomes visible.

Pure Python, no ROS dependencies.
"""

from aquavision.states.base_state import BaseState, ControlCmd, neutral_cmd


class SearchState(BaseState):
    """Yaw rotate to find the gate. Transitions to ALIGN when visible."""

    def update(self, perception: dict, dt: float) -> ControlCmd:
        self._elapsed += dt
        cmd = neutral_cmd(self._config.get('neutral_z', 500))

        cmd['r'] = self._config.get('search_yaw_speed', 200)

        if perception.get('gate_visible', False):
            self._next_state = 'ALIGN'

        return cmd
