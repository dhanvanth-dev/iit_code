"""Behaviors subpackage — reusable control primitives (no ROS)."""

from aquavision.behaviors.vertical_control import compute_vertical
from aquavision.behaviors.yaw_control import compute_yaw
from aquavision.behaviors.forward_motion import compute_forward

__all__ = ['compute_vertical', 'compute_yaw', 'compute_forward']
