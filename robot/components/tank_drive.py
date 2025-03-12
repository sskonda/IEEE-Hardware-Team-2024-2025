import math
import numpy as np

from robot.components import Component

from ..pid import PID
from ..constants import DRIVE_WHEEL_DIAMETER, DRIVE_WHEEL_OFFTANGENT, DRIVE_WHEEL_SPACING
from .motor import PIDMotor

class TankDrive(Component):
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

        self.pose_pid = PID(1.0, 0.0, 0.1, 0.0)

        self.current_pose = np.array([0.0, 0.0, 0.0])
        self.target_pose = np.array([0.0, 0.0, 0.0])

    def _init(self, pi):
        success = self.left_motor.init(pi) and self.right_motor.init(pi)
        
        self.__last_wheel_angles = np.array([
            self.right_motor.encoder.get_angle(),
            self.left_motor.encoder.get_angle()
        ])

        return success


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
        angle = -math.radians(self.current_pose[2])
        return np.array([[np.cos(angle), np.sin(angle), self.current_pose[0]], [-np.sin(angle), np.cos(angle), self.current_pose[1]], [0, 0, 1]])
    
    def set_target(self, pose: np.ndarray):
        """Sets the target pose for the robot to drive to.

        Parameters
        ----------
        pose : np.ndarray
            [x, y, theta] pose to drive to. [in, in, deg]
        """
        self.target_pose = pose
        self.pose_pid.setpoint = pose

    def update(self):
        error = (self.to_robot() @ np.append(self.target_pose[:2], [1,]))[:2]
        
        # Do translation
        s = ""
        # print("Error", error)
        if np.linalg.norm(error) > 1:  # precision in inches
            phi = math.degrees(math.atan2(error[1], error[0]))
            if abs((phi + 360) % 360 - 180) < 15.0:
                s += "Reversing\n"
                drive_cmd = -30.0
                rotate_cmd = (phi + 360) % 360 - 180
            elif abs(phi) < 15.0:  # precision in degrees
                s += "Driving\n"
                drive_cmd = 30.0
                rotate_cmd = phi
            else:
                s += "Turning\n"
                drive_cmd = 0.0
                rotate_cmd = phi
        else:
            s += "Done\n"
            drive_cmd = 0.0
            rotate_cmd = (self.target_pose[2] - self.current_pose[2]) 

        s += f"Drive: {drive_cmd}\n"
        s += f"Rotate: {rotate_cmd}"
        print(s)
        self.right_motor.set_speed(drive_cmd + rotate_cmd)
        self.left_motor.set_speed(drive_cmd - rotate_cmd)
        
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

        self.current_pose[:2] += -np.mean(d_wheel_distance) * np.array([np.cos(math.radians(self.current_pose[2])), np.sin(math.radians(self.current_pose[2]))])
        self.current_pose[2] += d_theta
            
        self.right_motor.update()
        self.left_motor.update()

    def at_target(self, position_tolerance=0.1, heading_tolerance=1.0):
        """
        Returns True if the robot is at the target pose.

        Returns
        -------
        bool
            True if the robot is at the target pose.
        """
        return np.linalg.norm(self.current_pose[:2] - self.target_pose[:2]) < position_tolerance and abs(self.current_pose[2] - self.target_pose[2]) < heading_tolerance
    
    def release(self):
        """Releases the motors."""
        self.left_motor.release()
        self.right_motor.release()

    def reset(self):    
        self.pose_pid.reset()
        self.right_motor.pid().reset()
        self.left_motor.pid().reset()

    def stop(self):
        """Stops both motors."""
        self.right_motor.stop()
        self.left_motor.stop()
