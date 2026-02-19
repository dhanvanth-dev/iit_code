"""
BLIND_PASS state — full forward thrust with no vision correction.

Timed pass through the gate when too close for reliable detection.
Pure Python, no ROS dependencies.
"""

from aquavision.states.base_state import BaseState, ControlCmd, neutral_cmd
from aquavision.behaviors.forward_motion import compute_forward


class BlindPassState(BaseState):
    """Blind forward thrust for a fixed duration. Transitions to DONE."""

    def update(self, perception: dict, dt: float) -> ControlCmd:
        self._elapsed += dt
        cmd = neutral_cmd(self._config.get('neutral_z', 500))

        cmd['x'] = compute_forward(
            self._config.get('blind_pass_speed', 500)
        )

        if self._elapsed >= self._config.get('blind_pass_duration', 5.0):
            self._next_state = 'DONE'

        return cmd
