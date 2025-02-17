from . import Component
from .. import PID
from .encoder import Encoder


import pigpio


class SpeedMotor(Encoder, Component):    
    def __init__(self):
        super().__init__()
        self._desired_speed = 0.0

    def set_speed(self, speed):
        """Sets the desired speed of the motor in RPM

        Parameters
        ----------
        speed : float
            The desired speed of the motor in RPM (negative for backwards)
        """
        raise NotImplementedError()
    
    def get_desired_speed(self):
        self._desired_speed


class DutyMotor(Component):
    def __init__(self):
        super().__init__()
        self._duty = 0.0
        
    def set_duty(self, duty: float):
        """Set the duty cycle of the motor (negative for reverse)

        Parameters
        ----------
        duty : float
            The desired duty cycle of the motor
        """
        raise NotImplementedError()
    
    def get_duty(self) -> float:
        return self._duty


class PositionMotor(Component):
    def __init__(self):
        super().__init__()
        self._desired_position = 0.0
    
    def set_position(self, position: float):
        """Sets the motor to the given position

        Parameters
        ----------
        position : float
            The desired position (0 being one-side and 1 being the other)
        """
        raise NotImplementedError()
    
    def get_desired_position(self) -> float:
        return self._desired_position

    def move(self, delta: float):
        self.set_position(self.get_desired_position() + delta)    


class BrushedMotor(DutyMotor, Component):
    def __init__(self, forward_pin: int, reverse_pin: int):
        super().__init__()
        self._forward_pin = forward_pin
        self._reverse_pin = reverse_pin
        self.speed = 0
    
    def init(self, pi):
        self.pi = pi
        self.pi.set_mode(self._forward_pin, pigpio.OUTPUT)
        self.pi.set_mode(self._reverse_pin, pigpio.OUTPUT)

        self.pi.set_PWM_dutycycle(self._forward_pin, 0)
        self.pi.set_PWM_dutycycle(self._reverse_pin, 0)
        self.pi.set_PWM_range(self._forward_pin, 255)
        self.pi.set_PWM_range(self._reverse_pin, 255)
        self.pi.set_PWM_frequency(self._forward_pin, 100)
        self.pi.set_PWM_frequency(self._reverse_pin, 100)
    
    def set_duty(self, duty: float):
        self._duty = int(255*duty)
        self.pi.set_PWM_dutycycle(self._forward_pin, max(0, self._duty))
        self.pi.set_PWM_dutycycle(self._reverse_pin, -min(0, self._duty))
    
    def stop(self):
        self.pi.set_PWM_dutycycle(self._forward_pin, 0)
        self.pi.set_PWM_dutycycle(self._reverse_pin, 0)
        self.pi.write(self._forward_pin, 0)
        self.pi.write(self._reverse_pin, 0)
    
    def release(self):
        self.stop()
        self.pi.set_mode(self._forward_pin, pigpio.INPUT)
        self.pi.set_mode(self._reverse_pin, pigpio.INPUT)
    

class DualBrushedMotor:
    """Class to control two brushed motors for movement operations."""
    
    def __init__(self, left_motor: BrushedMotor, right_motor: BrushedMotor):
        self.left_motor = left_motor
        self.right_motor = right_motor
        self.speed = 0.5  # Default movement speed (50% duty cycle)

    def set_speed(self, speed: float):
        """Sets the movement speed (0 to 1)."""
        self.speed = max(0, min(1, speed))  # Clamp between 0 and 1

    def move_forward(self):
        """Moves the lawnmower forward."""
        self.left_motor.set_duty(self.speed)
        self.right_motor.set_duty(self.speed)

    def move_backward(self):
        """Moves the lawnmower backward."""
        self.left_motor.set_duty(-self.speed)
        self.right_motor.set_duty(-self.speed)

    def rotate_left_90(self):
        """Rotates the lawnmower 90 degrees to the left."""
        self.left_motor.set_duty(-self.speed)
        self.right_motor.set_duty(self.speed)
        self._wait_for_rotation(90)

    def rotate_right_90(self):
        """Rotates the lawnmower 90 degrees to the right."""
        self.left_motor.set_duty(self.speed)
        self.right_motor.set_duty(-self.speed)
        self._wait_for_rotation(90)

    def rotate_left_180(self):
        """Rotates the lawnmower 180 degrees to the left."""
        self.left_motor.set_duty(-self.speed)
        self.right_motor.set_duty(self.speed)
        self._wait_for_rotation(180)

    def rotate_right_180(self):
        """Rotates the lawnmower 180 degrees to the right."""
        self.left_motor.set_duty(self.speed)
        self.right_motor.set_duty(-self.speed)
        self._wait_for_rotation(180)

    def stop(self):
        """Stops both motors."""
        self.left_motor.stop()
        self.right_motor.stop()

    def _wait_for_rotation(self, degrees):
        """
        Simulates waiting for a rotation to complete.
        The actual implementation should use sensors like an IMU or encoder feedback.
        """
        import time
        time.sleep(degrees / 90 * 0.5)  # Adjust timing as needed

   

class ServoMotor(PositionMotor, Component):
    def __init__(self, pin: int, period=20.0, min_pulse=0.8, max_pulse=2.2):
        super().__init__()
        self.pin = pin
        self.period = period
        self.resolution = 1000
        self.min_pulse = min_pulse
        self.max_pulse = max_pulse
    
    def init(self, pi):
        self.pi = pi
        self.pi.set_mode(self.pin, pigpio.OUTPUT)
        
        self.pi.set_PWM_dutycycle(self.pin, 0)
        self.pi.set_PWM_range(self.pin, int(self.resolution * self.period))
        self.pi.set_PWM_frequency(self.pin, int(1000 / self.period))

    def set_position(self, position):
        """Moves to the desired position 0 being one side 1 being the other

        Parameters
        ----------
        position : float
            The desired position from 0 to 1
        """
        self._desired_position = position
        duty = ((self.max_pulse - self.min_pulse) * position + self.min_pulse)
        self.pi.set_PWM_dutycycle(self.pin, int(self.resolution * duty + 0.5))

    def release(self):
        self.pi.set_PWM_dutycycle(self.pin, 0)
        self.pi.write(self.pin, 0)
        self.pi.set_mode(self.pin, pigpio.INPUT)


class StepperMotor(PositionMotor, Component):
    def __init__(self, step_pin, direction_pin):
        super().__init__()
        self._step_pin = step_pin
        self._direction_pin = direction_pin
        self._step_wave = None
        self._forward_wave = None
        self._backward_wave = None
        
    def init(self, pi):
        self.pi = pi
        self.pi.set_mode(self._step_pin, pigpio.OUTPUT)
        self.pi.set_mode(self._direction_pin, pigpio.OUTPUT)

        step_pulse = [
            pigpio.pulse(1 << self._step_pin, 0, 1000),    # 1ms high pulse
            pigpio.pulse(0, 1 << self._step_pin, 1000),    # 1ms low pulse
        ]

        forward_pulse = [
            pigpio.pulse(0, 1 << self._direction_pin, 1)
        ]
        
        backward_pulse = [
            pigpio.pulse(1 << self._direction_pin, 0, 1)
        ]

        self.pi.wave_add_new()
        self.pi.wave_add_generic(step_pulse)
        self._step_wave = self.pi.wave_create()
        
        self.pi.wave_add_generic(forward_pulse)
        self._forward_wave = self.pi.wave_create()
        
        self.pi.wave_add_generic(backward_pulse)
        self._backward_wave = self.pi.wave_create()
        
    def set_position(self, position: float):
        delta =  int(int(position) - self._desired_position)
        self._desired_position = int(position)

        remaining_steps = abs(delta)
        while remaining_steps != 0:
            # Wait if there is a waveform being transmitted
            while self.pi.wave_tx_busy():
                pass
            
            steps = min(2**16-1, remaining_steps)
            self.pi.wave_chain([
                self._forward_wave if delta > 0 else self._backward_wave,
                255, 0,
                    self._step_wave,
                255, 1, steps % 2**8, steps // 2**8
            ])
            remaining_steps -= steps
            
    def release(self):
        self.pi.wave_tx_stop()
        self.pi.write(self._step_pin, 0)
        self.pi.write(self._direction_pin, 0)
        self.pi.set_mode(self._step_pin, pigpio.INPUT)
        self.pi.set_mode(self._direction_pin, pigpio.INPUT)
        self.pi.wave_delete(self._forward_wave)
        self.pi.wave_delete(self._backward_wave)

class PIDMotor(PositionMotor, SpeedMotor, Component):
    def __init__(self, duty_motor: DutyMotor, encoder: Encoder, pid: PID):
        self.duty_motor = duty_motor
        self.encoder = encoder
        
        
        