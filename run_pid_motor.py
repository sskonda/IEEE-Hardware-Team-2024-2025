import pigpio
from robot.pid import PID
from robot.components import BrushedMotor, HallEncoder, PIDMotor

PI = pigpio.pi()
PI.wave_clear()

RIGHT_DRIVE_MOTOR = BrushedMotor(18, 12)  # Right Drive Motor
RIGHT_DRIVE_ENCODER = HallEncoder(23, 24, 1200)  # Right Drive Encoder

LEFT_DRIVE_MOTOR = BrushedMotor(13, 19)  # Left Drive Motor
LEFT_DRIVE_ENCODER = HallEncoder(15, 14, 1200)  # Left Drive Encoder

for component in (RIGHT_DRIVE_MOTOR, RIGHT_DRIVE_ENCODER, LEFT_DRIVE_MOTOR, LEFT_DRIVE_ENCODER):
    component.init(PI)

RIGHT = (RIGHT_DRIVE_MOTOR, RIGHT_DRIVE_ENCODER)
LEFT = (LEFT_DRIVE_MOTOR, LEFT_DRIVE_ENCODER)

RIGHT_PID_MOTOR = PIDMotor(*RIGHT, PID(1/360, 0, 0.1))
LEFT_PID_MOTOR = PIDMotor(*LEFT, PID(1/360, 0, 0.1))

try:
    while True:
        try:
            while True:
                for pid_motor in (RIGHT_PID_MOTOR, LEFT_PID_MOTOR):
                    pid_motor.update()
        except KeyboardInterrupt:
            new_position = float(input("New Position: "))
            RIGHT_PID_MOTOR.set_position(new_position)
            LEFT_PID_MOTOR.set_position(new_position)
except KeyboardInterrupt:
    pass