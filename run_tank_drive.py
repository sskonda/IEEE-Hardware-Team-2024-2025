import math
import time
import numpy as np
import pigpio
from robot.components import TankDrive
from robot import DRIVE_WHEEL_DIAMETER, RIGHT_PID_MOTOR, LEFT_PID_MOTOR

PI = pigpio.pi()
PI.wave_clear()

drive = TankDrive(LEFT_PID_MOTOR, RIGHT_PID_MOTOR)
drive.init(PI)

drive.left_motor.stop()
drive.right_motor.stop()
drive.position_pid.reset()
drive.heading_pid.reset()

start_time = time.time()
try:
    while True:
        drive.heading_pid.reset()
        drive.set_target(np.array([0.0, 0.0, float(input("Enter heading: "))]))
        while True:
            print("Left Speed", drive.left_motor.encoder.get_speed())
            print("Right Speed", drive.right_motor.encoder.get_speed())
            print("Rotations: ", drive.right_motor.encoder.get_angle() // 360)
            print("Actual RPM: ", drive.right_motor.encoder.get_angle() / 360 / (time.time() - start_time) * 60)
            drive.update()
            
            print(f"Heading: {drive.current_pose[2]}")
            print(f"Position: {drive.current_pose[:2]}")
            
            if abs(drive.heading_pid.error()) < 3:
                drive.stop()
                break
except KeyboardInterrupt:
    drive.stop()
