from robot import PI
from . import Component 
from .encoder import Encoder
import pigpio

GEAR_RATIO = 12#:1
WHEEL_DIAMETER = 8 # inches
WHEEL_SEPARATION = 8 # inches
ENCODER_RESOLUTION = 64 * GEAR_RATIO

 
class DutyMotor(Component):
    def __init__(self, forward_pin, reverse_pin, encoder=None):
        self.forward_pin = forward_pin
        self.reverse_pin = reverse_pin
        self.encoder = encoder
    
    def init(self):
        PI.set_mode(self.forward_pin, pigpio.OUTPUT)
        PI.set_mode(self.reverse_pin, pigpio.OUTPUT)

        PI.set_PWM_dutycycle(self.forward_pin, 0)
        PI.set_PWM_dutycycle(self.reverse_pin, 0)
        PI.set_PWM_range(self.forward_pin, 255)
        PI.set_PWM_range(self.reverse_pin, 255)
        PI.set_PWM_frequency(self.forward_pin, 100)
        PI.set_PWM_frequency(self.reverse_pin, 100)
        
    def set_speed(self, speed: float):
        """Set the speed of the motor in RPM (negative for reverse)

        Parameters
        ----------
        speed : float
            The desired speed of the motor in RPM
        """
        if self.encoder is None:
            raise NotImplementedError()
    
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
    