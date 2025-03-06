import time
import pigpio
from .pid import PID
from .constants import *
from .components.motor import BrushedMotor, StepperMotor, ServoMotor, PIDMotor
from .components.encoder import HallEncoder
from .components.ultrasonic import Ultrasonic

PI = pigpio.pi()
PI.wave_clear()

RIGHT_DRIVE_MOTOR = BrushedMotor(12, 18)  # Right Drive Motor
RIGHT_DRIVE_ENCODER = HallEncoder(24, 23, 1200)  # Right Drive Encoder

LEFT_DRIVE_MOTOR = BrushedMotor(19, 13)  # Left Drive Motor
LEFT_DRIVE_ENCODER = HallEncoder(14, 15, 1200)  # Left Drive Encoder

RIGHT = (RIGHT_DRIVE_MOTOR, RIGHT_DRIVE_ENCODER)
LEFT = (LEFT_DRIVE_MOTOR, LEFT_DRIVE_ENCODER)

RIGHT_PID_MOTOR = PIDMotor(*RIGHT, position_pid=PID(1/60, 0, 0.1, 0.1), velocity_pid=PID(0.005, 0.015, 0.0, 0.0), smoothing=0.1, max_duty=DRIVE_SPEED)
LEFT_PID_MOTOR = PIDMotor(*LEFT, position_pid=PID(1/60, 0, 0.1, 0.1), velocity_pid=PID(0.005, 0.015, 0.0, 0.0), smoothing=0.1, max_duty=DRIVE_SPEED)

INTAKE_MOTOR = BrushedMotor(26, 11)  # Intake Motor
INTAKE_ENCODER = HallEncoder(14, 15)  # Intake Encoder

CLAMP_MOTOR = BrushedMotor(10, 9)  # Clamp Motor
CLAMP_ENCODER = HallEncoder(25, 8)  # Clamp Encoder

BIN_LIFT_STEPPER = StepperMotor(20, 21)  # Bin Lift Stepper

BEACON_SERVO = ServoMotor(5)  # Beacon Servo
MISC_SERVO = ServoMotor(6)  # Misc Servo

PORT_ULTRASONIC = Ultrasonic(2, 3)  # Port-Side Ultrasonic
STARBOARD_ULTRASONIC = Ultrasonic(4, 17)  # Starboard-Side Ultrasonic
AFT_ULTRASONIC = Ultrasonic(27, 22)  # Aft-Side Ultrasonic

ROBOT = {
    "RIGHT_DRIVE_MOTOR": RIGHT_DRIVE_MOTOR,
    "RIGHT_DRIVE_ENCODER": RIGHT_DRIVE_ENCODER,
    "LEFT_DRIVE_MOTOR": LEFT_DRIVE_MOTOR,
    "LEFT_DRIVE_ENCODER": LEFT_DRIVE_ENCODER,
    "INTAKE_MOTOR": INTAKE_MOTOR,
    "INTAKE_ENCODER": INTAKE_ENCODER,
    "CLAMP_MOTOR": CLAMP_MOTOR,
    "CLAMP_ENCODER": CLAMP_ENCODER,
    "BIN_LIFT_STEPPER": BIN_LIFT_STEPPER,
    "BEACON_SERVO": BEACON_SERVO,
    "MISC_SERVO": MISC_SERVO,
    "PORT_ULTRASONIC": PORT_ULTRASONIC,
    "STARBOARD_ULTRASONIC": STARBOARD_ULTRASONIC,
    "AFT_ULTRASONIC": AFT_ULTRASONIC
}


def main():
    import tty
    import sys
    import termios
    
    for component in ROBOT:
        print("Initializing", component)
        ROBOT[component].init(PI)

    orig_settings = termios.tcgetattr(sys.stdin)

    tty.setcbreak(sys.stdin)
    x = 0
    try:
        while x != chr(27): # ESC
            x=sys.stdin.read(1)[0]
            print("You pressed", ord(x))
            match x:
                case 'i': 
                    print("Run Intake")
                    INTAKE_MOTOR.set_duty(1)
                case 'o':
                    print("Spit Intake")
                    INTAKE_MOTOR.set_duty(-1)
                case '[':
                    print("Lift Boxes")
                    BIN_LIFT_STEPPER.set_position(2600)
                case ']':
                    print("Lower Boxes")
                    BIN_LIFT_STEPPER.set_position(0)
                case 'w':
                    print("Forward")
                    RIGHT_DRIVE_MOTOR.set_duty(DRIVE_SPEED)
                    LEFT_DRIVE_MOTOR.set_duty(DRIVE_SPEED)
                case 'a':
                    print("Left")
                    RIGHT_DRIVE_MOTOR.set_duty(DRIVE_SPEED)
                    LEFT_DRIVE_MOTOR.set_duty(-DRIVE_SPEED)
                case 'd':
                    print("Right")
                    RIGHT_DRIVE_MOTOR.set_duty(-DRIVE_SPEED)
                    LEFT_DRIVE_MOTOR.set_duty(DRIVE_SPEED)
                case 's':
                    print("Backward")
                    RIGHT_DRIVE_MOTOR.set_duty(-DRIVE_SPEED)
                    LEFT_DRIVE_MOTOR.set_duty(-DRIVE_SPEED)
                case 'j':
                    print("Clamp Tighten")
                    CLAMP_MOTOR.set_duty(-1)
                case 'k':
                    print("Clamp Release")
                    CLAMP_MOTOR.set_duty(1)
                case ' ':
                    print("Stop")
                    for motor in [RIGHT_DRIVE_MOTOR, LEFT_DRIVE_MOTOR, CLAMP_MOTOR, INTAKE_MOTOR]:
                        motor.stop()
    except KeyboardInterrupt as e:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, orig_settings)   

        for component in ROBOT.values():
            component.release()
        PI.stop()