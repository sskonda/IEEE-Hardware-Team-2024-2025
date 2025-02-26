import time
import math
from typing import Optional

class PID:
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, kff=0.0, setpoint=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.kff = kff

        self.setpoint = setpoint
        self.prev_error = None
        self.integral = 0.0
        self.last_time = None

    def update(self, current_value: float) -> float:
        """ Calculate the PID output value for the given setpoint and current value. """
        error = self.setpoint - current_value
        current_time = time.time()

        if self.last_time is None or self.prev_error is None:
            self.prev_error = error
            self.last_time = current_time
            return 0

        dt = current_time - self.last_time
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt

        self.prev_error = error
        self.last_time = current_time
        return self.kp * error + self.ki * self.integral + self.kd * derivative + math.copysign(self.kff, error)

    def error(self):
        return self.prev_error or float('inf')

    def reset(self):
        self.prev_error = None
        self.integral = 0.0
        self.last_time = None
