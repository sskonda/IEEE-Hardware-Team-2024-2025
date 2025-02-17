import math
import pigpio
from robot import DRIVE_WHEEL_DIAMETER, RIGHT_PID_MOTOR, LEFT_PID_MOTOR
from robot.pid import PID
from robot.components import BrushedMotor, HallEncoder, PIDMotor

PI = pigpio.pi()
PI.wave_clear()

RIGHT_PID_MOTOR.init(PI)
LEFT_PID_MOTOR.init(PI)

try:
    while True:
        RIGHT_PID_MOTOR.stop()
        RIGHT_PID_MOTOR.pid().reset()
        LEFT_PID_MOTOR.stop()
        LEFT_PID_MOTOR.pid().reset()

        new_position = float(input("New Position: ")) / (DRIVE_WHEEL_DIAMETER * math.pi) * 360
        RIGHT_PID_MOTOR.set_position(new_position)
        LEFT_PID_MOTOR.set_position(new_position)

        try:
            while abs(RIGHT_PID_MOTOR.pid().error() or float('inf')) > 3 or abs(LEFT_PID_MOTOR.pid().error() or float('inf')) > 3:
                RIGHT_PID_MOTOR.update()
                print("RIGHT: ", RIGHT_PID_MOTOR.pid().error())
                LEFT_PID_MOTOR.update()
                print("LEFT: ", LEFT_PID_MOTOR.pid().error())
        except KeyboardInterrupt:
            pass
except KeyboardInterrupt:
    RIGHT_PID_MOTOR.stop()
    LEFT_PID_MOTOR.stop()
