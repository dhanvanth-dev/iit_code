"""
Vertical control behavior — reusable depth/vertical correction primitive.

Pure Python, no ROS dependencies.
"""

from aquavision.utils.pid import PIDController


def compute_vertical(
    error_y: float,
    dt: float,
    pid: PIDController,
    neutral_z: int = 500,
) -> int:
    """Compute vertical thrust from vertical error using PID.

    Positive error_y means gate is below center → need to descend (z > 500).
    Negative error_y means gate is above center → need to ascend (z < 500).

    Args:
        error_y: Normalized vertical error [-1.0, 1.0].
        dt: Time step in seconds.
        pid: PID controller instance for vertical axis.
        neutral_z: Neutral vertical thrust value (500 for ArduSub).

    Returns:
        Clamped z thrust value in [0, 1000].
    """
    vert_output: float = pid.compute(error_y, dt)
    z: int = neutral_z + int(vert_output)
    return max(0, min(1000, z))
