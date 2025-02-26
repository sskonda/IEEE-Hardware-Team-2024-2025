import time
import numpy as np

from ..pid import PID
from .. import DRIVE_WHEEL_DIAMETER, DRIVE_WHEEL_SPACING
from .motor import PIDMotor

class TankDrive():
    """
    Controls a robot with tank drive movement using two brushed motors.
    """

    def __init__(self, left_motor: PIDMotor, right_motor: PIDMotor):
        """
        Initializes the tank drive system.

        Parameters
        ----------
        left_motor : BrushedMotor
            The motor controlling the left side of the robot.
        right_motor : BrushedMotor
            The motor controlling the right side of the robot.
        """
        self.left_motor = left_motor
        self.right_motor = right_motor

        self.position_pid = PID(1, 0, 0.1, 0.1)
        self.heading_pid = PID(1, 0, 0.1, 0.1)

        self.current_pose = np.array([0, 0, 0])
        self.target_pose = np.array([0, 0, 0])

    def to_robot(self):
        """
        Converts a vector from the global frame to the robot frame.

        Parameters
        ----------
        vector : numpy.ndarray
            The vector to convert.
        direction : bool, optional
            If True, the vector is a direction vector. Default is False.

        Returns
        -------
        numpy.ndarray
            The vector in the robot frame.
        """
        angle = self.current_pose[2]
        return np.linalg.inv(self.to_world())
    
    def to_world(self):
        """
        Converts a vector from the robot frame to the global frame.

        Parameters
        ----------
        vector : numpy.ndarray
            The vector to convert.
        direction : bool, optional
            If True, the vector is a direction vector. Default is False.

        Returns
        -------
        numpy.ndarray
            The vector in the global frame.
        """
        angle = -self.current_pose[2]
        return np.array([[np.cos(angle), np.sin(angle), self.current_pose[0]], [-np.sin(angle), np.cos(angle), self.current_pose[1]], [0, 0, 1]])
    
    def update_position(self, pose: np.ndarray, weight: float = 0.1):
        self.current_pose[:2] = weight * pose[:2] + (1 - weight) * self.current_pose[:2]

    def update_heading(self, heading: float, weight: float = 0.1):
        self.current_pose[2] = weight * heading + (1 - weight) * self.current_pose[2]

    def set_target(self, pose: np.ndarray):
        """Sets the target pose for the robot to drive to.

        Parameters
        ----------
        pose : np.ndarray
            [x, y, theta] pose to drive to. [in, in, deg]
        """
        self.position_pid.reset()
        self.heading_pid.reset()

        self.target_pose = pose

    def update(self):
        # Do rotation
        self.heading_pid.setpoint = self.target_pose[2]
        
        if abs(self.heading_pid.error()) > 3:
            wheel_angle = (self.right_motor.encoder.get_angle() + -self.left_motor.encoder.get_angle()) / 2
            current_heading = np.cos(24.73 / 180 * np.pi) * DRIVE_WHEEL_DIAMETER / DRIVE_WHEEL_SPACING * wheel_angle
            control = self.heading_pid.update(current_heading)
            print(control)
            self.right_motor.set_speed(control)
            self.left_motor.set_speed(-control)
            
        self.right_motor.update()
        self.left_motor.update()

    def stop(self):
        """Stops both motors."""
        self.right_motor.stop()
        self.left_motor.stop()
