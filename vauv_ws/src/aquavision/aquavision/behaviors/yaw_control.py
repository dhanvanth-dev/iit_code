"""
Yaw control behavior — reusable heading correction primitive.

Pure Python, no ROS dependencies.
"""

from aquavision.utils.pid import PIDController


def compute_yaw(
    error_x: float,
    dt: float,
    pid: PIDController,
) -> int:
    """Compute yaw thrust from horizontal error using PID.

    Positive error_x means gate is to the right → yaw right (positive r).
    Negative error_x means gate is to the left → yaw left (negative r).

    Args:
        error_x: Normalized horizontal error [-1.0, 1.0].
        dt: Time step in seconds.
        pid: PID controller instance for yaw axis.

    Returns:
        Clamped yaw value in [-1000, 1000].
    """
    yaw_output: float = pid.compute(error_x, dt)
    return max(-1000, min(1000, int(yaw_output)))
