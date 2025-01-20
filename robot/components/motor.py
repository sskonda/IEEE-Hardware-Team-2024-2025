from . import Component 
from .encoder import Encoder

from robot import PI
import pigpio

GEAR_RATIO = 12#:1
WHEEL_DIAMETER = 8 # inches
WHEEL_SEPARATION = 8 # inches
ENCODER_RESOLUTION = 64 * GEAR_RATIO

class SpeedMotor(Encoder, Component):
    def set_speed(self, speed):
        """Sets the desired speed of the motor in RPM

        Parameters
        ----------
        speed : float
            The desired speed of the motor in RPM (negative for backwards)
        """
        raise NotImplementedError()


class DutyMotor(Component):
    def __init__(self, forward_pin: int, reverse_pin: int):
        self.forward_pin = forward_pin
        self.reverse_pin = reverse_pin
    
    def init(self):
        PI.set_mode(self.forward_pin, pigpio.OUTPUT)
        PI.set_mode(self.reverse_pin, pigpio.OUTPUT)

        PI.set_PWM_dutycycle(self.forward_pin, 0)
        PI.set_PWM_dutycycle(self.reverse_pin, 0)
        PI.set_PWM_range(self.forward_pin, 255)
        PI.set_PWM_range(self.reverse_pin, 255)
        PI.set_PWM_frequency(self.forward_pin, 100)
        PI.set_PWM_frequency(self.reverse_pin, 100)
    
    def set_duty(self, duty: float):
        """Set the duty cycle of the motor (negative for reverse)

        Parameters
        ----------
        duty : float
            The desired duty cycle of the motor
        """
        duty = int(255*duty)
        PI.set_PWM_dutycycle(self.forward_pin, max(0, duty))
        PI.set_PWM_dutycycle(self.reverse_pin, -min(0, duty))
    
    def stop(self):
        PI.set_PWM_dutycycle(self.forward_pin, 0)
        PI.set_PWM_dutycycle(self.reverse_pin, 0)
        PI.write(self.forward_pin, 0)
        PI.write(self.reverse_pin, 0)
    
    def release(self):
        self.stop()
   

class ServoMotor(Component):
    def __init__(self, pin: int, period=20.0, min_pulse=0.8, max_pulse=2.2):
        self.pin = pin
        self.period = period
        self.resolution = 1000
        self.min_pulse = min_pulse
        self.max_pulse = max_pulse
    
    def init(self):
        PI.set_mode(self.pin, pigpio.OUTPUT)
        
        PI.set_PWM_dutycycle(self.pin, 0)
        PI.set_PWM_range(self.pin, int(self.resolution * self.period))
        PI.set_PWM_frequency(self.pin, int(1000 / self.period))

    def set_position(self, position):
        """Sets the servo to the given position

        Parameters
        ----------
        position : float
            The desired position (0 being one-side and 1 being the other)
        """
        duty = ((self.max_pulse - self.min_pulse) * position + self.min_pulse)
        PI.set_PWM_dutycycle(self.pin, int(self.resolution * duty + 0.5))

    def release(self):
        PI.set_PWM_dutycycle(self.pin, 0)
        PI.write(self.pin, 0)