import rclpy
from rclpy.node import Node
import pigpio
import math
import time

from std_msgs.msg import Float64, Bool
from sensor_msgs.msg import JointState

from .motor import BrushedMotor, LinearActuator, ServoMotor
from .encoder import HallEncoder
from .constants import CLAMP_ROD_PITCH

INTAKE_MOTOR = BrushedMotor(26, 11)
INTAKE_ENCODER = HallEncoder(14, 15, 1200)

CLAMP_MOTOR = BrushedMotor(10, 9)
CLAMP_ENCODER = HallEncoder(25, 8, 1200)

LIFT_ACTUATOR = LinearActuator(20, 21)

BEACON_SERVO = ServoMotor(5)

CLAMP_RATIO = 2 * math.pi / CLAMP_ROD_PITCH  # rad/mm

class HardwareInterface(Node):
    """
    Controls the misc. actuators and sensors on the robot.
    """

    def __init__(self):
        super().__init__('hardware_interface')
        
        self._timer_period = self.declare_parameter('timer_period', 0.01).get_parameter_value().double_value

        self.pi = pigpio.pi()
        
        CLAMP_MOTOR.init(self.pi)
        CLAMP_ENCODER.init(self.pi)

        INTAKE_MOTOR.init(self.pi)
        INTAKE_ENCODER.init(self.pi)

        BEACON_SERVO.init(self.pi)
        LIFT_ACTUATOR.init(self.pi)

        self._pub_clamp_finished = self.create_publisher(Bool, '/clamp/finished', 1)
        self._pub_clamp_joint = self.create_publisher(JointState, '/clamp/joint_state', 1)
        self._sub_clamp = self.create_subscription(Float64, '/clamp', self._clamp_callback, 1)
        self._sub_intake = self.create_subscription(Float64, '/intake', self._intake_callback, 1)
        self._pub_intake_joint = self.create_publisher(JointState, '/intake/joint_state', 1)
        self._sub_beacon = self.create_subscription(Float64, '/beacon', self._beacon_callback, 1)
        self._sub_lift = self.create_subscription(Bool, '/lift', self._lift_callback, 1)

        self._timer = self.create_timer(self._timer_period, self._periodic)

    def _periodic(self):
        self._pub_clamp_joint.publish(
            JointState(
                name=['clamp'],
                position=[CLAMP_ENCODER.get_angle() / CLAMP_RATIO],
                velocity=[CLAMP_ENCODER.get_speed() / CLAMP_RATIO],
                effort=[CLAMP_MOTOR._duty]
            )
        )

        self._pub_intake_joint.publish(
            JointState(
                name=['intake'],
                position=[INTAKE_ENCODER.get_angle()],
                velocity=[INTAKE_ENCODER.get_speed()],
                effort=[INTAKE_MOTOR._duty]
            )
        )


    def _clamp_callback(self, msg: Float64):
        self._pub_clamp_finished.publish(Bool(data=False))
        target_rad = msg.data * CLAMP_RATIO

        if target_rad > CLAMP_ENCODER.get_angle():
            duty = 1.0
        else:
            duty = -1.0 
        
        CLAMP_MOTOR.set_duty(duty)
        time.sleep(0.75)  # Let motor spin up

        # make sure motor is still moving and position has not been reached
        while duty * (target_rad - CLAMP_ENCODER.get_angle()) > math.pi and abs(CLAMP_ENCODER.get_speed()) > 2 * math.pi:
            time.sleep(0.001)

        CLAMP_MOTOR.set_duty(0.0)
        self._pub_clamp_finished.publish(Bool(data=True))
    
    def _intake_callback(self, msg: Float64):
        INTAKE_MOTOR.set_duty(msg.data)

    def _beacon_callback(self, msg: Float64):
        BEACON_SERVO.set_position(msg.data)

    def _lift_callback(self, msg: Bool):
        LIFT_ACTUATOR.set_position(1 if msg.data else 0)
    
    def release(self):
        CLAMP_MOTOR.set_duty(0.0)
        INTAKE_MOTOR.set_duty(0.0)
        BEACON_SERVO.set_position(0.0)
        LIFT_ACTUATOR.set_position(0)

        CLAMP_MOTOR.release()
        CLAMP_ENCODER.release()

        INTAKE_MOTOR.release()
        INTAKE_ENCODER.release()

        BEACON_SERVO.release()
        LIFT_ACTUATOR.release()

def main(args=None):
    rclpy.init(args=args)
    hardware_interface = HardwareInterface()
    try:
        rclpy.spin(hardware_interface)
    finally:
        hardware_interface.release()
        hardware_interface.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
