"""
Base state interface for Aquavision mission states.

All mission states inherit from BaseState and implement the
on_enter/update/on_exit contract. States are pure Python —
no ROS imports allowed.
"""

from typing import Dict, Optional


# Type alias for the control command dict returned by states
ControlCmd = Dict[str, float]


def neutral_cmd(neutral_z: int = 500) -> ControlCmd:
    """Return a neutral (no movement) control command.

    Args:
        neutral_z: Neutral vertical thrust value.

    Returns:
        Control command dict with all axes neutral.
    """
    return {'x': 0, 'y': 0, 'z': neutral_z, 'r': 0, 'arm': -1}


class BaseState:
    """Abstract base class for all mission states.

    States are pure Python classes with no ROS dependencies.
    Each state receives perception data and returns control commands.

    The state lifecycle is:
        1. on_enter() — called once when transitioning into this state
        2. update(perception, dt) — called every tick while active
        3. on_exit() — called once when transitioning out of this state

    The `next_state` property signals when a transition should occur.
    Return None to stay in the current state, or a state name string
    to transition.
    """

    def __init__(self, config: dict) -> None:
        """Initialize with configuration parameters.

        Args:
            config: Dict of all mission parameters (PID gains,
                    thresholds, durations, thrust values, etc.).
        """
        self._config: dict = config
        self._next_state: Optional[str] = None
        self._elapsed: float = 0.0

    def on_enter(self) -> None:
        """Called once when entering this state. Override to initialize."""
        self._next_state = None
        self._elapsed = 0.0

    def update(self, perception: dict, dt: float) -> ControlCmd:
        """Called every tick. Must return a control command dict.

        Args:
            perception: Dict with keys:
                - error_x (float): Normalized horizontal error [-1, 1]
                - error_y (float): Normalized vertical error [-1, 1]
                - width_ratio (float): Gate bbox width as fraction of frame
                - gate_visible (bool): Whether gate is detected
                - mav_connected (bool): MAVLink heartbeat active
                - mav_armed (bool): Vehicle armed state
            dt: Time since last update in seconds.

        Returns:
            ControlCmd dict with keys: x, y, z, r, arm
            arm: 1=arm, 0=disarm, -1=no change
        """
        self._elapsed += dt
        return neutral_cmd(self._config.get('neutral_z', 500))

    def on_exit(self) -> None:
        """Called once when leaving this state. Override to clean up."""
        pass

    @property
    def next_state(self) -> Optional[str]:
        """State name to transition to, or None to stay."""
        return self._next_state

    @property
    def name(self) -> str:
        """Human-readable state name."""
        return self.__class__.__name__.replace('State', '').upper()
