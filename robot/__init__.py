import pigpio
import numpy as np

from robot.actions import * 
from ..src.py_robot.py_robot.pid import PID
from .constants import *
from .components import *

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
    smoothing=0.1,
    max_duty=DRIVE_SPEED
)
LEFT_PID_MOTOR = PIDMotor(
    *LEFT,
    position_pid=PID(1 / 60, 0, 0.1, 0.1),
    velocity_pid=PID(0.005, 0.015, 0.0, 0.0),
    smoothing=0.1,
    max_duty=DRIVE_SPEED
)

IMU = I2C_IMU()
DRIVE = TankDrive(LEFT_PID_MOTOR, RIGHT_PID_MOTOR)

INTAKE_MOTOR = BrushedMotor(26, 11)  # Intake Motor
INTAKE_ENCODER = HallEncoder(14, 15)  # Intake Encoder

CLAMP_MOTOR = BrushedMotor(10, 9)  # Clamp Motor
CLAMP_ENCODER = HallEncoder(25, 8)  # Clamp Encoder

BIN_LIFT_STEPPER = StepperMotor(20, 21)  # Bin Lift Stepper

BEACON_SERVO = ServoMotor(5)  # Beacon Servo
MISC_SERVO = ServoMotor(6)  # Misc Servo

ROBOT = {
    "INTAKE_MOTOR": INTAKE_MOTOR,
    # "INTAKE_ENCODER": INTAKE_ENCODER,
    "CLAMP_MOTOR": CLAMP_MOTOR,
    "CLAMP_ENCODER": CLAMP_ENCODER,
    "BIN_LIFT_STEPPER": BIN_LIFT_STEPPER,
    # "BEACON_SERVO": BEACON_SERVO,
    # "MISC_SERVO": MISC_SERVO,
    # "PORT_ULTRASONIC": PORT_ULTRASONIC,
    # "STARBOARD_ULTRASONIC": STARBOARD_ULTRASONIC,
    # "AFT_ULTRASONIC": AFT_ULTRASONIC,
    "DRIVE": DRIVE,
    "IMU": IMU,
}


BIN_PICKUP = (
    DriveToPosition(DRIVE, [31.25, 10.0]),  # Back out of start
    DriveToHeading(DRIVE, 180.0),
    DriveToPosition(DRIVE, [46.25, 8.0], 0.5),  # Drive to first bin
    DriveToHeading(DRIVE, 180.0, 0.5),
    ClampRelease(CLAMP_MOTOR, CLAMP_ENCODER),  # Pin bin to wall
    ## LOWER ARM TO GRAB BIN
    # ClampTighten(CLAMP_MOTOR, CLAMP_ENCODER),  # Tighten clamp
    ## DO ARM STUFF TO GRAB BIN
    DriveToPosition(DRIVE, [22.63, 22.88]),  # Drive to second bin
    DriveToHeading(DRIVE, -90.0),  # Turn to face wall
    DriveTimeout(DRIVE, [0.0, 10.0], 2.0, end=lambda: DRIVE.set_current_pose([22.63, 34.375], 90.0)),  # Drive to wall
)

BEACON = (
    DriveToPosition(DRIVE, [31.25, 20.0], 5.0),  # Back out of start
    DriveToHeading(DRIVE, 180.0, 5.0),
    DriveToPosition(DRIVE, [5.0, 23.5], 0.5),
    DriveToHeading(DRIVE, 180.0, 0.5),
    DriveTimeout(DRIVE, [-30.0, 0.0], 2),
    WithTimeout(1.0, start=lambda: BEACON_SERVO.set_position(0.75)),
    WithTimeout(1.0, start=lambda: BEACON_SERVO.set_position(0.0)),
)

CLAMP_TEST = (
    # WithTimeout(10.0),
    ClampTighten(CLAMP_MOTOR, CLAMP_ENCODER),
    # WithTimeout(10.0),
    # ClampRelease(CLAMP_MOTOR, CLAMP_ENCODER),
)

AUTO = BIN_PICKUP

def main():
    PI = pigpio.pi()
    PI.wave_clear()

    drive_poses = None
    imu_poses = None

    try:
        # Wait for start signal
        input("Press Enter to start...")

        for component in ROBOT:
            print("Initializing", component)
            ROBOT[component].init(PI)

        if not DRIVE.initialized:
            raise Exception("Drive failed to initialize, exiting...")

        if BEACON_SERVO.initialized:
            BEACON_SERVO.set_position(0.0)

        input("Press Enter to start driving...")

        current_position = np.array(START[:2])
        current_heading = np.array([START[2]])

        DRIVE.current_position[:] = current_position
        DRIVE.current_heading[:] = current_heading
        IMU.current_position[:] = current_position
        IMU.current_heading[:] = current_heading

        state = 0

        if IMU.initialized:
            imu_poses = []
        drive_poses = []
        last_pose_time = time.time()


        while True:
            if time.time() - last_pose_time > 0.5:
                last_pose_time = time.time()
                if IMU.initialized and imu_poses is not None:
                    imu_poses.append({"time": last_pose_time, "position": IMU.current_position, "heading": IMU.current_heading}) 
                drive_poses.append({"time": last_pose_time, "position": DRIVE.current_position, "heading": DRIVE.current_heading})

            # Update loop
            for component in ROBOT:
                if not ROBOT[component].initialized:
                    continue
                ROBOT[component].update()

            # Update pose
            if IMU.initialized:
                current_heading[:] = IMU.current_heading[:]
            else:
                # Fallback to odometry
                current_heading[:] = DRIVE.current_heading[:]

            current_position[:] = DRIVE.current_position
            DRIVE.current_heading[:] = current_heading

            # Execute action
            if state < len(AUTO):
                if AUTO[state]():
                    DRIVE.stop()
                    print(DRIVE.current_position, DRIVE.current_heading)
                    # input("Press enter to continue...")
                    # Move to next state
                    state += 1
            else:
                print("Finished!")
                break

    except Exception as e:
        pass
    finally:
        t = time.asctime()
        if imu_poses is not None:
            np.save(f"{t}_imu_poses.npy", imu_poses)
        if drive_poses is not None:
            np.save(f"{t}_drive_poses.npy", drive_poses)

        DRIVE.stop()
        for component in ROBOT:
            try:
                print("Releasing", component)
                ROBOT[component].release()
            except Exception as e:
                print("Error releasing", component, e)
        PI.stop()
