import math
import cv2
import time
import numpy as np
import pigpio
from visualizer import draw_indicator, SERVER, OUTPUT
from robot.components import TankDrive
from robot import DRIVE_WHEEL_DIAMETER, RIGHT_PID_MOTOR, LEFT_PID_MOTOR

PI = pigpio.pi()
SERVER.start()
PI.wave_clear()

drive = TankDrive(LEFT_PID_MOTOR, RIGHT_PID_MOTOR)
drive.init(PI)

drive.left_motor.stop()
drive.right_motor.stop()

drive.pose_pid.reset()

start_time = time.time()
try:
    while True:
        drive.reset()
        drive.set_target(np.array([float(val) for val in input("Enter x y heading: ").split()]))
        drive.current_pose = np.array([0.0, 0.0, 0.0])
        try:
            while True:
                drive.update()

                bg = np.zeros((500, 500, 3), np.uint8)

                robot = draw_indicator(bg, drive.current_pose[:2], drive.current_pose[2], color=(255, 0, 0))
                target = draw_indicator(robot, drive.target_pose[:2], drive.target_pose[2], color=(0, 255, 0))
                error = (drive.to_robot() @ np.append(drive.target_pose[:2], [1,]))[:2]
                error = draw_indicator(target, drive.current_pose[:2], math.degrees(np.arctan2(error[1], error[0])) + drive.current_pose[2], color=(0, 0, 255))

                OUTPUT.write(target)
        except KeyboardInterrupt:
            drive.stop()
            print()
except KeyboardInterrupt:
    drive.stop()
finally:
    drive.stop()
    PI.stop()