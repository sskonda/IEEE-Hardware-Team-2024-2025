#!/bin/python3

import pigpio
from robot import BEACON_SERVO

def main():
    BEACON_SERVO.init(pi=pigpio.pi())

    try:
        while True:
            BEACON_SERVO.set_position(float(input("Position: ")))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()