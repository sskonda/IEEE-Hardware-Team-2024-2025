import math
import numpy as np

from robot.components import Component

from ..pid import PID
from ..constants import DRIVE_WHEEL_DIAMETER, DRIVE_WHEEL_OFFTANGENT, DRIVE_WHEEL_SPACING
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
        left_motor : PIDMotor
            The motor controlling the left side of the robot.
        right_motor : PIDMotor
            The motor controlling the right side of the robot.
        """
        self.left_motor = left_motor
        self.right_motor = right_motor

        self.position_pid = PID(1, 0, 0.1, 0.0)
        self.heading_pid = PID(10.0, 0, 0.1, 0.0)

        self.current_pose = np.array([0.0, 0.0, 0.0])
        self.target_pose = np.array([0.0, 0.0, 0.0])

    def init(self, pi):
        self.left_motor.init(pi)
        self.right_motor.init(pi)
        
        self.__last_wheel_angles = np.array([
            self.right_motor.encoder.get_angle(),
            self.left_motor.encoder.get_angle()
        ])

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
        self.target_pose = pose

    def update(self):
        error = (self.to_robot() @ np.append(self.target_pose[:2], [1,]))[:2]
        
        # Do translation
        if error[0] > 5:
            self.position_pid.setpoint = np.linalg.norm(error).item()
            self.position_pid.update(np.linalg.norm(error).item())
            return

        # Do rotation
        self.heading_pid.setpoint = self.target_pose[2]
        
        heading_output = self.heading_pid.update(self.current_pose[2])

        self.right_motor.set_speed(heading_output)
        self.left_motor.set_speed(-heading_output)
       
        # Do inverse kinematics
        angles = np.array([
            self.right_motor.encoder.get_angle(),
            self.left_motor.encoder.get_angle()
        ])
        
        d_angles = self.__last_wheel_angles - angles
        
        self.__last_wheel_angles = angles
        
        d_wheel_distance = (DRIVE_WHEEL_DIAMETER * np.pi) * d_angles / 360
        
        d_wheel_arc_length = d_wheel_distance * np.cos(math.radians(DRIVE_WHEEL_OFFTANGENT))
        
        A, B = d_wheel_arc_length
        d_theta = math.degrees((B - A) / DRIVE_WHEEL_SPACING)

        self.current_pose[:2] += d_wheel_distance * np.array([np.cos(self.current_pose[2]), np.sin(self.current_pose[2])])
        self.current_pose[2] += d_theta
           
        self.right_motor.update()
        self.left_motor.update()

    def stop(self):
        """Stops both motors."""
        self.right_motor.stop()
        self.left_motor.stop()
