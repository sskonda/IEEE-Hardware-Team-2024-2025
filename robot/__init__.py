import time
import pigpio
from .pid import PID
from .components.motor import BrushedMotor, StepperMotor, ServoMotor, PIDMotor
from .components.encoder import HallEncoder
from .components.ultrasonic import Ultrasonic

DRIVE_WHEEL_DIAMETER = 3.14961
DRIVE_TICKS_PER_REV = 1200
DRIVE_SPEED = 0.8

DRIVE_P = 1/60
DRIVE_I = 0
DRIVE_D = 0.1
DRIVE_FF = 0.1


PI = pigpio.pi()
PI.wave_clear()

RIGHT_DRIVE_MOTOR = BrushedMotor(12, 18)  # Right Drive Motor
RIGHT_DRIVE_ENCODER = HallEncoder(24, 23, 1200)  # Right Drive Encoder

LEFT_DRIVE_MOTOR = BrushedMotor(19, 13)  # Left Drive Motor
LEFT_DRIVE_ENCODER = HallEncoder(14, 15, 1200)  # Left Drive Encoder

RIGHT = (RIGHT_DRIVE_MOTOR, RIGHT_DRIVE_ENCODER)
LEFT = (LEFT_DRIVE_MOTOR, LEFT_DRIVE_ENCODER)

RIGHT_PID_MOTOR = PIDMotor(*RIGHT, position_pid=PID(1/60, 0, 0.1, 0.1), velocity_pid=PID(0.0, 1/30, 0.0, 0.1), smoothing=0.1, max_duty=DRIVE_SPEED)
LEFT_PID_MOTOR = PIDMotor(*LEFT, position_pid=PID(1/60, 0, 0.1, 0.1), velocity_pid=PID(0.0, 1/30, 0.0, 0.1), smoothing=0.1, max_duty=DRIVE_SPEED)

# INTAKE_MOTOR = BrushedMotor(11, None)  # Intake Motor
# INTAKE_ENCODER = HallEncoder(14, 15)  # Intake Encoder

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
    # "INTAKE_MOTOR": INTAKE_MOTOR,
    # "INTAKE_ENCODER": INTAKE_ENCODER,
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
            print("You pressed", x)
            match x:
                case 'w':
                    print("Forward")
                    RIGHT_DRIVE_MOTOR.set_duty(-DRIVE_SPEED)
                    LEFT_DRIVE_MOTOR.set_duty(DRIVE_SPEED)
                    
                    
                    # DISTANCE = 20

                    # start_angle = RIGHT_DRIVE_ENCODER.get_angle()
                    # while RIGHT_DRIVE_ENCODER.get_angle() > start_angle - DISTANCE / 9.5 * 360:
                    #     RIGHT_DRIVE_MOTOR.set_duty(DRIVE_SPEED)
                    #     LEFT_DRIVE_MOTOR.set_duty(DRIVE_SPEED)
                    
                    # RIGHT_DRIVE_MOTOR.stop()
                    # LEFT_DRIVE_MOTOR.stop()
                case 'a':
                    print("Left")
                    RIGHT_DRIVE_MOTOR.set_duty(-DRIVE_SPEED)
                    LEFT_DRIVE_MOTOR.set_duty(-DRIVE_SPEED)
                case 'd':
                    print("Right")
                    RIGHT_DRIVE_MOTOR.set_duty(DRIVE_SPEED)
                    LEFT_DRIVE_MOTOR.set_duty(DRIVE_SPEED)
                case 's':
                    print("Backward")
                    RIGHT_DRIVE_MOTOR.set_duty(DRIVE_SPEED)
                    LEFT_DRIVE_MOTOR.set_duty(-DRIVE_SPEED)
                case 'l':
                    print("Clamp Tighten")
                    CLAMP_MOTOR.set_duty(1)
                case 'v':
                    print("Clamp Release")
                    CLAMP_MOTOR.set_duty(-1)
                case 'q':
                    print("Left wheel F")
                    LEFT_DRIVE_MOTOR.set_duty(DRIVE_SPEED)
                case 'e':
                    print("Left wheel B")
                    LEFT_DRIVE_MOTOR.set_duty(-DRIVE_SPEED)
                case ']':
                    print("Right wheel F")
                    RIGHT_DRIVE_MOTOR.set_duty(DRIVE_SPEED)
                case '[':
                    print("Right wheel B")
                    RIGHT_DRIVE_MOTOR.set_duty(-DRIVE_SPEED)
                case ',':
                    print("Rotate Left")
                    start_angle = RIGHT_DRIVE_ENCODER.get_angle()
                    # t = 0
                    # last_time = time.time()
                    ANGLE = 72
                    while RIGHT_DRIVE_ENCODER.get_angle() > start_angle - 5 * ANGLE:
                        RIGHT_DRIVE_MOTOR.set_duty(DRIVE_SPEED)
                        LEFT_DRIVE_MOTOR.set_duty(-DRIVE_SPEED)
                        # curr_time = time.time()
                        # t += min(1, 0.1 * curr_time - last_time)
                        # last_time = curr_time
                    RIGHT_DRIVE_MOTOR.stop()
                    LEFT_DRIVE_MOTOR.stop()
                case ' ':
                    print("Stop")
                    for motor in [RIGHT_DRIVE_MOTOR, LEFT_DRIVE_MOTOR, CLAMP_MOTOR]:
                        motor.stop()
    except KeyboardInterrupt as e:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, orig_settings)   

        for component in ROBOT.values():
            component.release()
        PI.stop()