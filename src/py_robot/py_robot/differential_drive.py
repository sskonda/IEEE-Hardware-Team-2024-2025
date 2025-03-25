import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Duration
import numpy as np 
import pigpio

from std_msgs.msg import Header
from geometry_msgs.msg import Vector3, Point, Quaternion, Twist, Pose, PoseWithCovariance, TwistWithCovariance
from nav_msgs.msg import Odometry

from .pid import PID
from .motor import BrushedMotor, PIDMotor
from .encoder import HallEncoder
from .constants import DRIVE_SPEED, DRIVE_WHEEL_DIAMETER, DRIVE_WHEEL_OFFTANGENT, DRIVE_WHEEL_SPACING

DRIVE_EFFECTIVE_SPACING = DRIVE_WHEEL_SPACING / np.cos(np.radians(DRIVE_WHEEL_OFFTANGENT))

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
    max_duty=DRIVE_SPEED
)
LEFT_PID_MOTOR = PIDMotor(
    *LEFT,
    position_pid=PID(1 / 60, 0, 0.1, 0.1),
    velocity_pid=PID(0.005, 0.015, 0.0, 0.0),
    max_duty=DRIVE_SPEED
)

KINEMATIC = (DRIVE_WHEEL_DIAMETER / 2.0) * np.array([
            [0.5, 0.5],
            [0.0, 0.0],
            [1.0 / DRIVE_EFFECTIVE_SPACING, -1.0 / DRIVE_EFFECTIVE_SPACING]
])

class DifferentialDrive(rclpy.Node):
    """
    Controls the robot with tank drive movement the two brushed motors.
    """

    def __init__(self):
        """
        Initializes the tank drive system.
        """
        super().__init__('differential_drive')
        
        self.timer_period = self.declare_parameter('timer_period', 0.01).get_parameter_value().double_value

        timeout = self.declare_parameter('cmd_vel_lifetime', 0.1).get_parameter_value().double_value
        self.cmd_vel_lifetime = Duration(sec=timeout)

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
        
        self._cmd_vel = Twist()
        self.cmd_vel_subscription = self.create_subscription(Twist, "/drive/cmd_vel", self._cmd_vel_received, qos_profile=1)
        self.odometry_publisher = self.create_publisher(Odometry, '/drive/odometry', qos_profile=qos_profile_sensor_data)
        self.timer = self.create_timer(self.timer_period, self._periodic)
        
    def _cmd_vel_received(self, msg):
        print(type(msg))
        print(msg.data)
        self._cmd_vel_stamp = self.get_clock().now()
        self._cmd_vel = msg.data

    def _periodic(self):
        # Do kinematics
        wheel_states = np.array([
            [ self.right_motor.get_position(), self.right_motor.get_speed() ],
            [ self.left_motor.get_position(), self.left_motor.get_speed() ]
        ])
        wheel_states[:, 0] -= self.__last_wheel_angles
        self.__last_wheel_angles = wheel_states[:, 0]
        
        
        local_twist = KINEMATIC @ wheel_states
        global_twist = np.array([
            [ np.cos(self.current_heading), np.cos(self.current_heading), 0.0],
            [ np.sin(self.current_heading), np.sin(self.current_heading), 0.0],
            [                          0.0,                          0.0, 1.0],
        ]) @ local_twist

        # TODO: Implement arc-based position update
        self.current_position += global_twist[:, [0, 1]]
        self.current_heading += global_twist[:, [2]]

        odom = Odometry(
            header=Header(
                stamp=self.get_clock().now().to_msg(),
                frame_id="odom"
            ),
            child_frame_id="base_link",
            pose=PoseWithCovariance(
                pose=Pose(
                    position=Point(
                        x=self.current_position[0],
                        y=self.current_position[1],
                        z=0.0,
                    ),
                    orientation=Quaternion(
                        x=0.0,
                        y=0.0,
                        z=np.sin(self.current_heading),
                        w=np.cos(self.current_heading),
                    )
                )
            ),
            twist=TwistWithCovariance(
                twist=Twist(
                    linear=Vector3(
                        x=local_twist[0],
                        y=local_twist[1],
                        z=0.0
                    ),
                    angular=Vector3(
                        x=0.0,
                        y=0.0,
                        z=local_twist[2]
                    )
                )
            )
        )

        # Do inverse kinematics
        if (self.get_clock().now() - self._cmd_vel_stamp) > self.cmd_vel_lifetime:
            self._cmd_vel = Twist()
        V = self._cmd_vel.linear.x
        w = self._cmd_vel.angular.z
        v_R = (V + w * DRIVE_EFFECTIVE_SPACING)
        v_L = (V - w * DRIVE_EFFECTIVE_SPACING)

        self.right_motor.set_speed(v_R)
        self.left_motor.set_speed(v_L)

        self.right_motor.update()
        self.left_motor.update()

    def release(self):
        """Stops and releases the motors."""
        self.left_motor.stop()
        self.right_motor.stop()
        self.left_motor.release()
        self.right_motor.release()
        self.pi.stop()


def main():
    print('Hi from py_robot.')


if __name__ == '__main__':
    main()
