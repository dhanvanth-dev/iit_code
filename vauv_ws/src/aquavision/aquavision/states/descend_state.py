"""
DESCEND state — apply downward thrust for a configured duration.

Pure Python, no ROS dependencies.
"""

from aquavision.states.base_state import BaseState, ControlCmd, neutral_cmd


class DescendState(BaseState):
    """Descend for a fixed duration then transition to STABILIZE."""

    def update(self, perception: dict, dt: float) -> ControlCmd:
        self._elapsed += dt
        nz: int = self._config.get('neutral_z', 500)
        cmd = neutral_cmd(nz)

        cmd['z'] = self._config.get('descend_thrust', 700)

        if self._elapsed >= self._config.get('descend_duration', 5.0):
            self._next_state = 'STABILIZE'

        return cmd
