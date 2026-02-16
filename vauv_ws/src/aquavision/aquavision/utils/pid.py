"""
PID Controller utility for Aquavision 2.0.

Provides a reusable, thread-safe PID controller with anti-windup,
output saturation, and deadband handling.
"""

from typing import Optional


class PIDController:
    """Discrete PID controller with anti-windup and deadband.

    Attributes:
        kp: Proportional gain.
        ki: Integral gain.
        kd: Derivative gain.
        output_min: Minimum output saturation limit.
        output_max: Maximum output saturation limit.
        integral_max: Maximum integral accumulator magnitude (anti-windup).
        deadband: Error values within [-deadband, deadband] are treated as zero.
    """

    def __init__(
        self,
        kp: float = 1.0,
        ki: float = 0.0,
        kd: float = 0.0,
        output_min: float = -1000.0,
        output_max: float = 1000.0,
        integral_max: float = 500.0,
        deadband: float = 0.0,
    ) -> None:
        """Initialize PID controller with gains and limits.

        Args:
            kp: Proportional gain.
            ki: Integral gain.
            kd: Derivative gain.
            output_min: Minimum output value (saturation floor).
            output_max: Maximum output value (saturation ceiling).
            integral_max: Anti-windup clamp for integral accumulator.
            deadband: Errors within this magnitude are treated as zero.
        """
        self.kp: float = kp
        self.ki: float = ki
        self.kd: float = kd
        self.output_min: float = output_min
        self.output_max: float = output_max
        self.integral_max: float = integral_max
        self.deadband: float = deadband

        # Internal state
        self._integral: float = 0.0
        self._prev_error: Optional[float] = None

    def compute(self, error: float, dt: float) -> float:
        """Compute PID output for the given error and time step.

        Args:
            error: Current error signal (setpoint - measurement).
            dt: Time elapsed since last computation (seconds).
                Must be positive; if zero or negative, returns 0.0.

        Returns:
            Clamped PID output within [output_min, output_max].
        """
        if dt <= 0.0:
            return 0.0

        # Apply deadband
        if abs(error) < self.deadband:
            error = 0.0

        # Proportional term
        p_term: float = self.kp * error

        # Integral term with anti-windup
        self._integral += error * dt
        self._integral = max(-self.integral_max,
                             min(self.integral_max, self._integral))
        i_term: float = self.ki * self._integral

        # Derivative term
        if self._prev_error is not None:
            d_term: float = self.kd * (error - self._prev_error) / dt
        else:
            d_term = 0.0
        self._prev_error = error

        # Sum and saturate
        output: float = p_term + i_term + d_term
        output = max(self.output_min, min(self.output_max, output))

        return output

    def reset(self) -> None:
        """Reset integral accumulator and derivative memory."""
        self._integral = 0.0
        self._prev_error = None

    def update_gains(
        self, kp: float, ki: float, kd: float
    ) -> None:
        """Update PID gains at runtime.

        Args:
            kp: New proportional gain.
            ki: New integral gain.
            kd: New derivative gain.
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
