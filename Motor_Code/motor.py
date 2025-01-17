from turtle import forward
import RPi.GPIO as GPIO
from time import time
import numpy as np

GEAR_RATIO = 12#:1
WHEEL_DIAMETER = 8 # inches
WHEEL_SEPARATION = 8 # inches
ENCODER_RESOLUTION = 64 * GEAR_RATIO

DRIVE_P = 1.0
DRIVE_I = 0.0
DRIVE_D = 0.0

 
class DutyMotor():
    def __init__(self, forward_pin, reverse_pin, encoder=None):
        self.forward_pin = forward_pin
        self.reverse_pin = reverse_pin
    
    def init(self):
        GPIO.setup(self.forward_pin, GPIO.OUT)
        GPIO.setup(self.reverse_pin, GPIO.OUT)
        self.forward_pwm = GPIO.PWM(self.forward_pin, 100)
        self.reverse_pwm = GPIO.PWM(self.reverse_pin, 100)
        
        self.forward_pwm.start(0)
        self.reverse_pwm.start(0)
        
    def set_speed(self, speed: float):
        """Set the speed of the motor in RPM (negative for reverse)

        Parameters
        ----------
        speed : float
            The desired speed of the motor in RPM
        """
        raise NotImplementedError()
    
    def set_duty(self, duty: float):
        """Set the duty cycle of the motor (negative for reverse)

        Parameters
        ----------
        duty : float
            The desired duty cycle of the motor
        """
        if duty > 0:
            self.forward_pwm.ChangeDutyCycle(100*duty)
            self.reverse_pwm.ChangeDutyCycle(0)
        elif duty < 0:
            self.forward_pwm.ChangeDutyCycle(0)
            self.reverse_pwm.ChangeDutyCycle(100*-duty)
    

   
if __name__ == "__main__":
    GPIO.setmode(GPIO.BCM)

    motor = DutyMotor(17, 18, None)
    motor.init()

    motor.set_duty(0.50)
    input("Press enter to stop")
    motor.set_duty(0)

    GPIO.cleanup()