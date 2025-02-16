#!/bin/python3

from robot.components import HallEncoder
from collections import deque
import time
import pigpio


def main():
    pi = pigpio.pi()
    RIGHT_DRIVE_ENCODER = HallEncoder(23, 24, 1200)  # Right Drive Encoder
    LEFT_DRIVE_ENCODER = HallEncoder(14, 15, 1200)  # Left Drive Encoder
    encoder = RIGHT_DRIVE_ENCODER
    encoder.init(pi)
    q = deque(maxlen=25)

    try:
        while True:
            q.append(encoder.get_speed())
            print("Speed:", sum(q) / len(q) / 60)
            print("Ticks:", encoder.get_ticks())
            print("Deg:", encoder.get_angle())

            time.sleep(0.02)

    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
    