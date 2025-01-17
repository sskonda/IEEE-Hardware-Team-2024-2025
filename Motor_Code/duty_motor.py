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


class Encoder():
    def init(self):
        """Initializes the encoder

        Raises
        ------
        NotImplementedError
            If the method is not implemented
        """
        raise NotImplementedError()
    
    def get_speed(self) -> float:
        """Returns the current speed of the encoder in RPM

        Returns
        -------
        float
            The current speed of the encoder in RPM

        Raises
        ------
        NotImplementedError
            If the method is not implemented
        """
        raise NotImplementedError()

        

    def get_angle(self) -> float:
        """Returns the current angle of the encoder in degrees

        Returns
        -------
        float
            The current angle of the encoder in degrees

        Raises
        ------
        NotImplementedError
            If the method is not implemented
        """
        raise NotImplementedError()

        
class HallEncoder(Encoder):
    def __init__(self, sin_pin, cos_pin, resolution):
        self.sin_pin = sin_pin
        self.cos_pin = cos_pin
        self.resolution = resolution
        
    def init(self):
        GPIO.setup(self.sin_pin, GPIO.IN)
        GPIO.setup(self.cos_pin, GPIO.IN)
        self.sin_state = GPIO.input(self.sin_pin)
        self.cos_state = GPIO.input(self.cos_pin)
        
    def get_speed(self) -> float:
        """Returns the current speed of the encoder in RPM

        Returns
        -------
        float
            The current speed of the encoder in RPM
        """
        return 0
   

class reader:
   """
   A class to read speedometer pulses and calculate the RPM.
   """
   def __init__(self, pi, gpio, pulses_per_rev=1.0, weighting=0.0, min_RPM=5.0):
      """
      Instantiate with the Pi and gpio of the RPM signal
      to monitor.

      Optionally the number of pulses for a complete revolution
      may be specified.  It defaults to 1.

      Optionally a weighting may be specified.  This is a number
      between 0 and 1 and indicates how much the old reading
      affects the new reading.  It defaults to 0 which means
      the old reading has no effect.  This may be used to
      smooth the data.

      Optionally the minimum RPM may be specified.  This is a
      number between 1 and 1000.  It defaults to 5.  An RPM
      less than the minimum RPM returns 0.0.
      """
      self.pi = pi
      self.gpio = gpio
      self.pulses_per_rev = pulses_per_rev

      if min_RPM > 1000.0:
         min_RPM = 1000.0
      elif min_RPM < 1.0:
         min_RPM = 1.0

      self.min_RPM = min_RPM

      self._watchdog = 200 # Milliseconds.

      if weighting < 0.0:
         weighting = 0.0
      elif weighting > 0.99:
         weighting = 0.99

      self._new = 1.0 - weighting # Weighting for new reading.
      self._old = weighting       # Weighting for old reading.

      self._high_tick = None
      self._period = None

      pi.set_mode(gpio, pigpio.INPUT)

      self._cb = pi.callback(gpio, pigpio.RISING_EDGE, self._cbf)
      pi.set_watchdog(gpio, self._watchdog)
   def _cbf(self, gpio, level, tick):

      if level == 1: # Rising edge.

         if self._high_tick is not None:
            t = pigpio.tickDiff(self._high_tick, tick)

            if self._period is not None:
               self._period = (self._old * self._period) + (self._new * t)
            else:
               self._period = t

         self._high_tick = tick

      elif level == 2: # Watchdog timeout.

         if self._period is not None:
            if self._period < 2000000000:
               self._period += (self._watchdog * 1000)

   def RPM(self):
      """
      Returns the RPM.
      """
      RPM = 0.0
      if self._period is not None:
         RPM = 60000000.0 / (self._period * self.pulses_per_rev)
         if RPM < self.min_RPM:
            RPM = 0.0

      return RPM

   def cancel(self):
      """
      Cancels the reader and releases resources.
      """
      self.pi.set_watchdog(self.gpio, 0) # cancel watchdog
      self._cb.cancel()
 
class DriveMotor():
    def __init__(self, forward_pin, reverse_pin, encoder):
        self.forward_pin = forward_pin
        self.reverse_pin = reverse_pin
        GPIO.setup(forward_pin, GPIO.OUT)
        GPIO.setup(reverse_pin, GPIO.OUT)
        self.forward_pwm = GPIO.PWM(forward_pin, 100)
        self.reverse_pwm = GPIO.PWM(reverse_pin, 100)

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
            self.forward_pwm.ChangeDutyCycle(duty)
            self.reverse_pwm.ChangeDutyCycle(0)
        elif duty < 0:
            self.forward_pwm.ChangeDutyCycle(0)
            self.reverse_pwm.ChangeDutyCycle(-duty)
    

   
if __name__ == "__main__":
    GPIO.setmode(GPIO.BCM)
    motor = DriveMotor(17, 18, None)
    motor.set_duty(50)
    input("Press enter to stop")
    motor.set_duty(0)
    GPIO.cleanup()