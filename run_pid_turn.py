import math
import pigpio
from robot import DRIVE_WHEEL_DIAMETER, RIGHT_PID_MOTOR, LEFT_PID_MOTOR
from robot.pid import PID

PI = pigpio.pi()
PI.wave_clear()

RIGHT_PID_MOTOR.init(PI)
LEFT_PID_MOTOR.init(PI)

HEADING_PID = PID(1, 0, 0.1, 0.1)

try:
    while True:
        RIGHT_PID_MOTOR.stop()
        RIGHT_PID_MOTOR.pid().reset()
        LEFT_PID_MOTOR.stop()
        LEFT_PID_MOTOR.pid().reset()
        HEADING_PID.reset()

        new_position = float(input("New Heading: "))
        HEADING_PID.setpoint = new_position
        current_heading = 0

        try:
            while abs(HEADING_PID.error() or float('inf')) > 3:
                wheel_angle = (RIGHT_PID_MOTOR.encoder.get_angle() + -LEFT_PID_MOTOR.encoder.get_angle()) / 2
                current_heading = math.cos(24.73 / 180 * math.pi) * DRIVE_WHEEL_DIAMETER / 12.5 * wheel_angle
                control = HEADING_PID.update(current_heading)
                print(control)
                RIGHT_PID_MOTOR.set_speed(control)
                LEFT_PID_MOTOR.set_speed(-control)

                RIGHT_PID_MOTOR.update()
                print("RIGHT: ", RIGHT_PID_MOTOR.pid().error())
                LEFT_PID_MOTOR.update()
                print("LEFT: ", LEFT_PID_MOTOR.pid().error())
                print("HEADING: ", HEADING_PID.error())
        except KeyboardInterrupt:
            pass
except KeyboardInterrupt:
    RIGHT_PID_MOTOR.stop()
    LEFT_PID_MOTOR.stop()
