import time
import math

import numpy as np

class PID:
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, kff=0.0, setpoint: np.ndarray = np.array([0.0], dtype=np.float64)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.kff = kff

        self.setpoint = setpoint
        self.prev_error = None
        self.integral = 0.0
        self.last_time = None

    def update(self, current_value: np.ndarray) -> np.ndarray:
        """ Calculate the PID output value for the given setpoint and current value. """
        error = self.setpoint - current_value
        current_time = time.time()

        if self.last_time is None or self.prev_error is None:
            self.prev_error = error
            self.last_time = current_time
            return np.zeros_like(error)

        dt = current_time - self.last_time
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt

        self.prev_error = error
        self.last_time = current_time
        return self.kp * error + self.ki * self.integral + self.kd * derivative + math.copysign(self.kff, error)

    def error(self):
        return self.prev_error 

    def reset(self):
        self.prev_error = None
        self.integral = 0.0
        self.last_time = None
