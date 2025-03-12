import math
import numpy as np

from robot.components import Component

from ..pid import PID
from ..constants import DRIVE_WHEEL_DIAMETER, DRIVE_WHEEL_OFFTANGENT, DRIVE_WHEEL_SPACING
from .motor import PIDMotor

def pos_neg(angle) -> float:
    return (angle + 180.0) % 360.0 - 180.0

def full_pos(angle) -> float:
    return angle % 360.0

def dist(a, b) -> float:
    return abs(pos_neg(a - b))

def clamp(value, low, high):
    return max(low, min(high, value))

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
        super().__init__()

        self.left_motor = left_motor
        self.right_motor = right_motor

        self.current_position = np.array([0.0, 0.0])
        self.current_heading = np.array([0.0])

        self.target_position = None
        self.target_heading = None
        self.position_tolerance = 1.0
        self.heading_tolerance = 5.0

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
        angle = -math.radians(self.current_heading)
        return np.array([[np.cos(angle), np.sin(angle), self.current_position[0]], [-np.sin(angle), np.cos(angle), self.current_position[1]], [0, 0, 1]])
    
    def drive_to_position(self, position: np.ndarray):
        """Sets the target pose for the robot to drive to.

        Parameters
        ----------
        pose : np.ndarray
            [x, y, theta] pose to drive to. [in, in, deg]
        """
        self.target_position = position
        self.target_heading = None
    
    def drive_to_heading(self, heading: float):
        self.target_position = None
        self.target_heading = heading

    def update(self):
        # Do translation

        drive_cmd = 0.0
        rotate_cmd = 0.0
        if not self.at_target():
            s = ""
            if self.target_position is not None:
                error = (self.to_robot() @ np.append(self.target_position, [1,]))[:2]
                angle_error = math.degrees(math.atan2(error[1], error[0]))

                drive_cmd = error[0]
                if drive_cmd > 0:
                    rotate_cmd = pos_neg(angle_error)
                else:
                    rotate_cmd = pos_neg(angle_error - 180.0)
            elif self.target_heading is not None:
                angle_error = pos_neg(self.target_heading - self.current_heading)
                drive_cmd = 0.0
                rotate_cmd = float(angle_error / 2.0)
            drive_cmd = clamp(drive_cmd, -40.0, 40.0)
            rotate_cmd = clamp(rotate_cmd, -40.0, 40.0)
            s += f"Drive: {drive_cmd}\n"
            s += f"Rotate: {rotate_cmd}"
            print(s)

            right_speed = (drive_cmd + rotate_cmd)
            right_speed += math.copysign(15.0, right_speed)
            left_speed = (drive_cmd - rotate_cmd)
            left_speed += math.copysign(15.0, left_speed)

            self.right_motor.set_speed(right_speed)
            self.left_motor.set_speed(left_speed)
        else:
            self.right_motor.set_speed(0.0)
            self.left_motor.set_speed(0.0)
        
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

        # TODO: Implement arc-based position update
        self.current_position += -np.mean(d_wheel_distance) * np.array([np.cos(math.radians(self.current_heading)), np.sin(math.radians(self.current_heading))])
        self.current_heading += d_theta
            
        self.right_motor.update()
        self.left_motor.update()

    def at_target(self):
        """
        Returns True if the robot is at the target pose.

        Returns
        -------
        bool
            True if the robot is at the target pose.
        """
        position_done = self.target_position is None or np.linalg.norm(self.current_position - self.target_position) < self.position_tolerance
        heading_done = self.target_heading is None or dist(self.current_heading, self.target_heading) < self.heading_tolerance
        return position_done and heading_done
    
    def _release(self):
        """Releases the motors."""
        self.left_motor.release()
        self.right_motor.release()

    def reset(self):    
        self.right_motor.pid().reset()
        self.left_motor.pid().reset()

    def stop(self):
        """Stops both motors."""
        self.right_motor.stop()
        self.left_motor.stop()
