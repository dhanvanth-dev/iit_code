"""
Forward motion behavior — reusable forward thrust primitive.

Pure Python, no ROS dependencies.
"""


def compute_forward(speed: int) -> int:
    """Compute forward thrust value.

    Args:
        speed: Desired forward speed in [-1000, 1000].
               Positive = forward, negative = backward.

    Returns:
        Clamped forward thrust in [-1000, 1000].
    """
    return max(-1000, min(1000, speed))
