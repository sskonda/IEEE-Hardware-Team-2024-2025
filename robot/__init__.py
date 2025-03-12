import pigpio

import random
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
    "DRIVE": DRIVE,
    "IMU": IMU,
    "INTAKE_MOTOR": INTAKE_MOTOR,
    "INTAKE_ENCODER": INTAKE_ENCODER,
    "CLAMP_MOTOR": CLAMP_MOTOR,
    "CLAMP_ENCODER": CLAMP_ENCODER,
    "BIN_LIFT_STEPPER": BIN_LIFT_STEPPER,
    "BEACON_SERVO": BEACON_SERVO,
    "MISC_SERVO": MISC_SERVO,
    "PORT_ULTRASONIC": PORT_ULTRASONIC,
    "STARBOARD_ULTRASONIC": STARBOARD_ULTRASONIC,
    "AFT_ULTRASONIC": AFT_ULTRASONIC,
    "CAMERA": CAMERA,
    "DRIVE": DRIVE,
}

def main():
    import numpy as np
    
    PI = pigpio.pi()
    PI.wave_clear()

    try:
        # Wait for start signal
        # input("Press Enter to start...")
        
        for component in ROBOT:
            print("Initializing", component)
            ROBOT[component].init(PI)

        if CAMERA.initialized: 
            CAMERA.poll_for_light()
        else:
            # input("Press Enter to start driving...")
            pass

        current_heading = 0.0
        state = 0
        while True:
            # Update loop
            for component in ROBOT:
                if not ROBOT[component].initialized:
                    continue
                ROBOT[component].update()
            
            # Update heading
            if IMU.initialized:
                print(IMU.get_orientation(), DRIVE.current_pose[2])

            # Drive in squares
            match state:
                case 0:
                    print("NORTH")
                    DRIVE.set_target(np.array([0.0, 0.0, 0.0]))
                    if DRIVE.at_target():
                        state = random.randint(0, 3)
                case 1:
                    print("WEST")
                    DRIVE.set_target(np.array([0.0, 0.0, 90.0]))
                    if DRIVE.at_target():
                        state = random.randint(0, 3)
                case 2:
                    print("SOUTH")
                    DRIVE.set_target(np.array([0.0, 0.0, 180.0]))
                    if DRIVE.at_target():
                        state = random.randint(0, 3)
                case 3:
                    print("EAST")
                    DRIVE.set_target(np.array([0.0, 0.0, 270.0]))
                    if DRIVE.at_target():
                        state = random.randint(0, 3)
                

                

                
            
            
        
            
    except KeyboardInterrupt:
        pass
    finally:
        for component in ROBOT:
            try:
                print("Releasing", component)
                ROBOT[component].release()
            except Exception as e:
                print("Error releasing", component, e)
                pass
        PI.stop()
        
        
