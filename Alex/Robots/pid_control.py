import time
from math import pi, isnan

class PID:
    """
    A class for PID control. Modified from openMV library
    """
    _kp = _ki = _kd = _integrator = _imax = 0
    _last_error = _last_derivative = _last_t = 0
    _RC = 1 / (2 * pi * 20)

    def __init__(self, p=0, i=0, d=0, imax=0):
        """
        Initialise the Tuning object with given PID parameters.

        Args:
            p (float): Proportional gain.
            i (float): Integral gain.
            d (float): Derivative gain.
            imax (float): Maximum Integral error.
        """
        self._kp = float(p)
        self._ki = float(i)
        self._kd = float(d)
        self._imax = abs(imax)
        self._last_derivative = float("nan")


    def get_pid(self, error, scaler):
        """
        Calculates next PID command based on given error

        Args:
            error (float): Error between current state and desired state
            scaler (float): Multiplier for proportional and derivative control

        Returns:
            output (float): Control command
        """
        tnow = time.ticks_ms()
        dt = time.ticks_diff(tnow, self._last_t)
        output = 0
        if self._last_t == 0 or dt > 1000:
            dt = 0
            self.reset_I()
        self._last_t = tnow
        delta_time = float(dt) / float(1000)
        output += error * self._kp
        if abs(self._kd) > 0 and dt > 0:
            if isnan(self._last_derivative):
                derivative = 0
                self._last_derivative = 0
            else:
                derivative = (error - self._last_error) / delta_time
            derivative = self._last_derivative + (
                (delta_time / (self._RC + delta_time)) * (derivative - self._last_derivative)
            )
            self._last_error = error
            self._last_derivative = derivative
            output += self._kd * derivative
        output *= scaler
        if abs(self._ki) > 0 and dt > 0:
            self._integrator += (error * self._ki) * scaler * delta_time
            if self._integrator < -self._imax:
                self._integrator = -self._imax
            elif self._integrator > self._imax:
                self._integrator = self._imax
            output += self._integrator
        return output


    def reset_I(self):
        """
        Resets integral error
        """
        self._integrator = 0
        self._last_derivative = float("nan")
