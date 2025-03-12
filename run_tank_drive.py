import numpy as np
import pigpio
from visualizer import draw_indicator, SERVER, OUTPUT
from robot import DRIVE

PI = pigpio.pi()
SERVER.start()
PI.wave_clear()

DRIVE.init(PI)

DRIVE.left_motor.stop()
DRIVE.right_motor.stop()

START = (31.25, 4.625, -90.0)
current_position = START[:2]
current_heading = START[2]

DRIVE.current_position[:] = current_position
DRIVE.current_heading[:] = current_heading

try:
    while True:
        DRIVE.reset()

        x, y, heading = (float(val) for val in input("Enter x y heading: ").split())
        DRIVE.drive_to_position(np.array([x, y]))

        try:
            while True:
                DRIVE.update()

                bg = np.zeros((500, 500, 3), np.uint8)
                robot = draw_indicator(bg, DRIVE.current_position, DRIVE.current_heading, color=(255, 0, 0))
                target = draw_indicator(
                    robot,
                    np.array([x, y]),
                    heading,
                    color=(0, 255, 0)
                )

                OUTPUT.write(target)

                if DRIVE.at_target():
                    if DRIVE.target_position is not None:
                        print("At target position.")
                        DRIVE.drive_to_heading(heading)
                    else:
                        print("At target heading.")
                        DRIVE.stop()
                        break
        except KeyboardInterrupt:
            DRIVE.stop()
            print()
except KeyboardInterrupt:
    DRIVE.stop()
finally:
    DRIVE.stop()
    PI.stop()