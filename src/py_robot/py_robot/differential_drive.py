import rclpy
import numpy as np 
import pigpio
from geometry_msgs import Twist

from .pid import PID
from .motor import BrushedMotor, PIDMotor
from .encoder import HallEncoder
from .constants import DRIVE_SPEED, DRIVE_WHEEL_DIAMETER, DRIVE_WHEEL_OFFTANGENT, DRIVE_WHEEL_SPACING


RIGHT_DRIVE_MOTOR = BrushedMotor(12, 18)  # Right Drive Motor
RIGHT_DRIVE_ENCODER = HallEncoder(24, 23, 1200)  # Right Drive Encoder

LEFT_DRIVE_MOTOR = BrushedMotor(19, 13)  # Left Drive Motor
LEFT_DRIVE_ENCODER = HallEncoder(14, 15, 1200)  # Left Drive Encoder

RIGHT = (RIGHT_DRIVE_MOTOR, RIGHT_DRIVE_ENCODER)
LEFT = (LEFT_DRIVE_MOTOR, LEFT_DRIVE_ENCODER)

RIGHT_PID_MOTOR = PIDMotor(
    *RIGHT,
    position_pid=PID(1 / 60, 0, 0.1, 0.1),
    velocity_pid=PID(0.005, 0.015, 0.0, 0.0),
    smoothing=0.1,
    max_duty=DRIVE_SPEED
)
LEFT_PID_MOTOR = PIDMotor(
    *LEFT,
    position_pid=PID(1 / 60, 0, 0.1, 0.1),
    velocity_pid=PID(0.005, 0.015, 0.0, 0.0),
    smoothing=0.1,
    max_duty=DRIVE_SPEED
)


class DifferentialDrive(rclpy.Node):
    """
    Controls the robot with tank drive movement the two brushed motors.
    """

    def __init__(self):
        """
        Initializes the tank drive system.
        """
        super().__init__('drive_action_server')

        self.pi = pigpio.pi()

        self.left_motor = LEFT_PID_MOTOR
        self.right_motor = RIGHT_PID_MOTOR

        self.current_position = np.array([0.0, 0.0])
        self.current_heading = np.array([0.0])

        self.left_motor.init(self.pi)
        self.right_motor.init(self.pi)
        
        self.__last_wheel_angles = np.array([
            self.right_motor.encoder.get_angle(),
            self.left_motor.encoder.get_angle()
        ])

        self.cmd_vel = self.create_subscription()

    def _periodic(self):
        # Do inverse kinematics
        angles = np.array([
            self.right_motor.encoder.get_angle(),
            self.left_motor.encoder.get_angle()
        ])
        
        d_angles = self.__last_wheel_angles - angles
        
        self.__last_wheel_angles = angles
        
        d_wheel_distance = (DRIVE_WHEEL_DIAMETER * np.pi) * d_angles / 360
        
        d_wheel_arc_length = d_wheel_distance * np.cos(np.radians(DRIVE_WHEEL_OFFTANGENT))
        
        A, B = d_wheel_arc_length
        d_theta = np.degrees((B - A) / DRIVE_WHEEL_SPACING)

        # TODO: Implement arc-based position update
        self.current_position += -np.mean(d_wheel_distance) * np.array([np.cos(np.radians(self.current_heading)), np.sin(np.radians(self.current_heading))])
        self.current_heading += d_theta
            
        self.right_motor.update()
        self.left_motor.update()

    def _release(self):
        """Releases the motors."""
        self.left_motor.release()
        self.right_motor.release()
        self.pi.stop()


def main():
    print('Hi from py_robot.')


if __name__ == '__main__':
    main()
