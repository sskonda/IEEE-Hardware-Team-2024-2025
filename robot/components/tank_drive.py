import time
from .motor_driver import DualBrushedMotor

class TankDrive:
    """
    Controls a robot with tank drive movement using two brushed motors.
    """

    def __init__(self, left_motor, right_motor):
        """
        Initializes the tank drive system.

        Parameters
        ----------
        left_motor : BrushedMotor
            The motor controlling the left side of the robot.
        right_motor : BrushedMotor
            The motor controlling the right side of the robot.
        """
        self.drive = DualBrushedMotor(left_motor, right_motor)
        self.rotation_speed = 0.5  # Default rotation speed (50% duty cycle)

    def set_speed(self, speed: float):
        """
        Sets the movement speed for forward and backward motion.

        Parameters
        ----------
        speed : float
            Speed value between 0 and 1.
        """
        self.drive.set_speed(speed)

    def set_rotation_speed(self, speed: float):
        """
        Sets the rotation speed for turning movements.

        Parameters
        ----------
        speed : float
            Speed value between 0 and 1.
        """
        self.rotation_speed = max(0, min(1, speed))  # Clamp between 0 and 1

    def move_forward(self):
        """Moves the robot forward."""
        self.drive.move_forward()

    def move_backward(self):
        """Moves the robot backward."""
        self.drive.move_backward()

    def rotate(self, angle: float, direction: str = "right"):
        """
        Rotates the robot by a specified angle.

        Parameters
        ----------
        angle : float
            The desired angle of rotation in degrees.
        direction : str, optional
            Rotation direction, "right" or "left". Default is "right".
        """
        if direction.lower() == "right":
            self.drive.left_motor.set_duty(self.rotation_speed)
            self.drive.right_motor.set_duty(-self.rotation_speed)
        elif direction.lower() == "left":
            self.drive.left_motor.set_duty(-self.rotation_speed)
            self.drive.right_motor.set_duty(self.rotation_speed)
        else:
            raise ValueError("Invalid direction! Use 'right' or 'left'.")

        self._wait_for_rotation(angle)
        self.drive.stop()

    def _wait_for_rotation(self, degrees):
        """
        Simulates waiting for a rotation to complete.
        The actual implementation should use encoders or an IMU.

        Parameters
        ----------
        degrees : float
            The number of degrees to rotate.
        """
        time.sleep(degrees / 90 * 0.5)  # Adjust timing as needed

    def stop(self):
        """Stops both motors."""
        self.drive.stop()
