import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_best_available
from rclpy.time import Duration
import numpy as np 
import pigpio

from std_msgs.msg import Header, Bool
from geometry_msgs.msg import Vector3, Point, Quaternion, Twist, Pose, PoseWithCovariance, TwistWithCovariance
from nav_msgs.msg import Odometry

from .pid import PID
from .motor import BrushedMotor, PIDMotor
from .encoder import HallEncoder
from .constants import DRIVE_SPEED, DRIVE_WHEEL_DIAMETER, DRIVE_WHEEL_OFFTANGENT, DRIVE_WHEEL_SPACING
from .constants import DRIVE_P, DRIVE_I, DRIVE_D, DRIVE_FF

DRIVE_EFFECTIVE_SPACING = DRIVE_WHEEL_SPACING / np.cos(np.radians(DRIVE_WHEEL_OFFTANGENT))

RIGHT_DRIVE_MOTOR = BrushedMotor(12, 18)  # Right Drive Motor
RIGHT_DRIVE_ENCODER = HallEncoder(24, 23, 1200)  # Right Drive Encoder

LEFT_DRIVE_MOTOR = BrushedMotor(19, 13)  # Left Drive Motor
LEFT_DRIVE_ENCODER = HallEncoder(14, 15, 1200)  # Left Drive Encoder

RIGHT = (RIGHT_DRIVE_MOTOR, RIGHT_DRIVE_ENCODER)
LEFT = (LEFT_DRIVE_MOTOR, LEFT_DRIVE_ENCODER)

RIGHT_PID_MOTOR = PIDMotor(
    *RIGHT,
    # position_pid=PID(1 / 60 / 0.0254, 0, 0.1 / 0.0254, 0.1 / 0.0254),
    velocity_pid=PID(DRIVE_P, DRIVE_I, DRIVE_D, DRIVE_FF),
    max_duty=DRIVE_SPEED
)
LEFT_PID_MOTOR = PIDMotor(
    *LEFT,
    # position_pid=PID(1 / 60 / 0.0254, 0, 0.1 / 0.0254, 0.1 / 0.0254),
    velocity_pid=PID(DRIVE_P, DRIVE_I, DRIVE_D, DRIVE_FF),
    max_duty=DRIVE_SPEED
)

KINEMATIC = (DRIVE_WHEEL_DIAMETER / 2.0) * np.array([
            [0.5, 0.5],
            [0.0, 0.0],
            [1.0 / DRIVE_EFFECTIVE_SPACING, -1.0 / DRIVE_EFFECTIVE_SPACING]
])

class DifferentialDrive(Node):
    """
    Controls the robot with tank drive movement the two brushed motors.
    """

    def __init__(self):
        """
        Initializes the tank drive system.
        """
        super().__init__('differential_drive')
        
        self.timer_period = self.declare_parameter('timer_period', 0.01).get_parameter_value().double_value

        self.logger = self.get_logger()
        timeout = self.declare_parameter('cmd_vel_lifetime', 0.5).get_parameter_value().double_value
        self.cmd_vel_lifetime = Duration(seconds=timeout)

        self.pi = pigpio.pi()

        self.current_position = np.array([0.0, 0.0])
        self.current_heading = np.array([0.0])

        self.__last_wheel_angles = np.array([
            RIGHT_PID_MOTOR.encoder.get_angle(),
            LEFT_PID_MOTOR.encoder.get_angle()
        ])
        
        self._cmd_vel = Twist()
        self._cmd_vel_stamp = None
        self.cmd_vel_subscription = self.create_subscription(Twist, "/cmd_vel", self._cmd_vel_received, qos_profile=qos_profile_sensor_data)
        self.enable_sub = self.create_subscription(Bool, '/drive/enable', self._enable_callback, qos_profile=qos_profile_best_available)
        self.odometry_publisher = self.create_publisher(Odometry, '/drive/odometry', qos_profile=qos_profile_sensor_data)
        self.timer = self.create_timer(self.timer_period, self._periodic)
        self.enabled = False

        LEFT_PID_MOTOR.init(self.pi)
        RIGHT_PID_MOTOR.init(self.pi)
        
    def _enable_callback(self, msg: Bool):
        self.enabled = msg.data
        
    def _cmd_vel_received(self, msg: Twist):
        self._cmd_vel_stamp = self.get_clock().now()
        self._cmd_vel = msg

    def _periodic(self):
        # Do kinematics
        wheel_states = np.array([
            [ RIGHT_PID_MOTOR.get_position(), RIGHT_PID_MOTOR.get_speed() ],
            [ LEFT_PID_MOTOR.get_position(), LEFT_PID_MOTOR.get_speed() ]
        ])
        wheel_angles = np.array(wheel_states[:, 0])
        wheel_states[:, 0] -= self.__last_wheel_angles

        self.__last_wheel_angles = wheel_angles
        
        local_twist = (KINEMATIC @ wheel_states).T
        global_twist = (np.array([
            [ np.cos(self.current_heading.item()), np.cos(self.current_heading.item()), 0.0],
            [ np.sin(self.current_heading.item()), np.sin(self.current_heading.item()), 0.0],
            [                          0.0,                          0.0, 1.0],
        ]) @ local_twist.T).T

        # TODO: Implement arc-based position update
        self.current_position += global_twist[0, [0, 1]]
        self.current_heading += global_twist[0, [2]]

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
                        z=np.sin(self.current_heading.item()),
                        w=np.cos(self.current_heading.item()),
                    )
                )
            ),
            twist=TwistWithCovariance(
                twist=Twist(
                    linear=Vector3(
                        x=local_twist[1, 0],
                        y=local_twist[1, 1],
                        z=0.0
                    ),
                    angular=Vector3(
                        x=0.0,
                        y=0.0,
                        z=local_twist[1, 2]
                    )
                )
            )
        )
        self.odometry_publisher.publish(odom)


        # Do inverse kinematics
        self.get_logger().info("Enabled (DRIVE): " + str(self.enabled))
        if self.enabled:
            if self._cmd_vel_stamp is None or (self.get_clock().now() - self._cmd_vel_stamp) > self.cmd_vel_lifetime:
                self._cmd_vel = Twist()
            V = self._cmd_vel.linear.x
            w = self._cmd_vel.angular.z

            v_R = (V + w * DRIVE_EFFECTIVE_SPACING)
            v_L = (V - w * DRIVE_EFFECTIVE_SPACING)

            RIGHT_PID_MOTOR.set_speed(2 * v_R / DRIVE_WHEEL_DIAMETER)
            LEFT_PID_MOTOR.set_speed(2 * v_L / DRIVE_WHEEL_DIAMETER)

            RIGHT_PID_MOTOR.update()
            LEFT_PID_MOTOR.update()
        else:
            RIGHT_PID_MOTOR.duty_motor.set_duty(0.0)
            LEFT_PID_MOTOR.duty_motor.set_duty(0.0)
            RIGHT_PID_MOTOR.velocity_pid.reset()
            LEFT_PID_MOTOR.velocity_pid.reset()

    def release(self):
        """Stops and releases the motors."""
        LEFT_PID_MOTOR.stop()
        RIGHT_PID_MOTOR.stop()
        LEFT_PID_MOTOR.release()
        RIGHT_PID_MOTOR.release()
        self.pi.stop()


def main(args=None):
    rclpy.init(args=args)
    drive = DifferentialDrive()
    
    try:
        rclpy.spin(drive)
    finally:
        drive.release()
        drive.destroy_node()

    rclpy.shutdown()

    


if __name__ == '__main__':
    main()
