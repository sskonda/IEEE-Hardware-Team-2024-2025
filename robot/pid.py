import time

class PID:
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, setpoint=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.prev_error = 0
        self.integral = 0
        self.last_time = None

    def update(self, current_value):
        """ Calculate the PID output value for the given setpoint and current value. """
        error = self.setpoint - current_value
        current_time = time.time()

        if self.last_time is None:
            self.prev_error = error
            self.last_time = current_time
            return 0

        dt = current_time - self.last_time
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt

        self.prev_error = error
        self.last_time = current_time
        return self.kp * error + self.ki * self.integral + self.kd * derivative

    def reset(self):
        self.prev_error = 0
        self.integral = 0
        self.last_time = None