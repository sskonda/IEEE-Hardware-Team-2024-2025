import numpy as np
import pigpio

from .pid import PID
from .constants import *
from .components import *

RIGHT_DRIVE_MOTOR = BrushedMotor(12, 18)  # Right Drive Motor
RIGHT_DRIVE_ENCODER = HallEncoder(24, 23, 1200)  # Right Drive Encoder

LEFT_DRIVE_MOTOR = BrushedMotor(19, 13)  # Left Drive Motor
LEFT_DRIVE_ENCODER = HallEncoder(14, 15, 1200)  # Left Drive Encoder

RIGHT = (RIGHT_DRIVE_MOTOR, RIGHT_DRIVE_ENCODER)
LEFT = (LEFT_DRIVE_MOTOR, LEFT_DRIVE_ENCODER)

RIGHT_PID_MOTOR = PIDMotor(*RIGHT, position_pid=PID(1/60, 0, 0.1, 0.1), velocity_pid=PID(0.005, 0.015, 0.0, 0.0), smoothing=0.1, max_duty=DRIVE_SPEED)
LEFT_PID_MOTOR = PIDMotor(*LEFT, position_pid=PID(1/60, 0, 0.1, 0.1), velocity_pid=PID(0.005, 0.015, 0.0, 0.0), smoothing=0.1, max_duty=DRIVE_SPEED)

CAMERA = Camera()
IMU = I2C_IMU()
DRIVE = TankDrive(LEFT_PID_MOTOR, RIGHT_PID_MOTOR)

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
    # "INTAKE_MOTOR": INTAKE_MOTOR,
    # "INTAKE_ENCODER": INTAKE_ENCODER,
    # "CLAMP_MOTOR": CLAMP_MOTOR,
    # "CLAMP_ENCODER": CLAMP_ENCODER,
    # "BIN_LIFT_STEPPER": BIN_LIFT_STEPPER,
    # "BEACON_SERVO": BEACON_SERVO,
    # "MISC_SERVO": MISC_SERVO,
    # "PORT_ULTRASONIC": PORT_ULTRASONIC,
    # "STARBOARD_ULTRASONIC": STARBOARD_ULTRASONIC,
    # "AFT_ULTRASONIC": AFT_ULTRASONIC,
    # "CAMERA": CAMERA,
    "DRIVE": DRIVE,
    "IMU": IMU,
}

class Action:
    def __init__(self, start, repeated, is_finished):
        self.start = start
        self.repeated = repeated
        self.is_finished = is_finished
        
    def __call__(self):
        if self.start:
            self.start()
            self.start = None
        if self.repeated:
            self.repeated()
        if self.is_finished:
            return self.is_finished()
        return False

START = (31.25, 4.625, -90.0)
WAYPOINTS = (
    Action(lambda: DRIVE.drive_to_position(np.array([31.25, 19.5])), None, DRIVE.at_target),  # Back out of start
    Action(lambda: DRIVE.drive_to_heading(0.0), None, DRIVE.at_target),  # Turn to face bins
    Action(lambda: DRIVE.drive_to_position(np.array([52.0, 19.5])), None, DRIVE.at_target),  # Drive to first bin

    ## DO ARM STUFF TO GRAB BIN
    
    Action(lambda: DRIVE.drive_to_position(np.array([22.63, 22.88])), None, DRIVE.at_target),  # Drive to second bin
    Action(lambda: DRIVE.drive_to_heading(-90.0), None, DRIVE.at_target),  # Turn to face wall
    
    ## Drive into the wall
    ## Reset position and heading to (22.63, 34.375, -90.0)
    ## Close the clamp

    # Lawnmower pattern
)

def main():
    PI = pigpio.pi()
    PI.wave_clear()

    try:
        # Wait for start signal
        input("Press Enter to start...")
        
        for component in ROBOT:
            print("Initializing", component)
            ROBOT[component].init(PI)

        if not DRIVE.initialized:
            raise Exception("Drive failed to initialize, exiting...")

        if CAMERA.initialized: 
            CAMERA.poll_for_light()
        else:
            input("Press Enter to start driving...")
            pass


        current_position = START[:2]
        current_heading = START[2]

        DRIVE.current_position = current_position
        DRIVE.current_heading = current_heading
        IMU.current_position = current_position
        IMU.current_heading = current_heading

        state = 0

        while True:
            # Update loop
            for component in ROBOT:
                if not ROBOT[component].initialized:
                    continue
                ROBOT[component].update()
            
            # Update pose
            if IMU.initialized:
                current_heading = IMU.get_orientation()
            else:
                # Fallback to odometry
                current_heading = DRIVE.current_heading

            current_position = DRIVE.current_position
            DRIVE.current_heading = current_heading

            # Execute action
            if state < len(WAYPOINTS):
                if WAYPOINTS[state]():
                    # Move to next state
                    state += 1
            else:
                print("Finished!")
                break
            
    except Exception as e:
        pass
    finally:
        for component in ROBOT:
            try:
                print("Releasing", component)
                ROBOT[component].release()
            except Exception as e:
                print("Error releasing", component, e)
            finally:
                PI.stop()
        
        
