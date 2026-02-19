"""
STABILIZE state — hold neutral thrust to settle after descent.

Pure Python, no ROS dependencies.
"""

from aquavision.states.base_state import BaseState, ControlCmd, neutral_cmd


class StabilizeState(BaseState):
    """Neutral thrust for a fixed duration then transition to SEARCH."""

    def update(self, perception: dict, dt: float) -> ControlCmd:
        self._elapsed += dt
        cmd = neutral_cmd(self._config.get('neutral_z', 500))

        if self._elapsed >= self._config.get('stabilize_duration', 3.0):
            self._next_state = 'SEARCH'

        return cmd
