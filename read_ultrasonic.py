#!/bin/python3

import pigpio
from robot.components import Ultrasonic
import time

def main():
    ultrasonic = Ultrasonic(17, 23)
    ultrasonic.init(pi=pigpio.pi())

    try:
        while True:
            ultrasonic.ping()
            time.sleep(0.060)  # Wait for ping
            print("Range (in):", ultrasonic.get_range())
            input("Press enter to continue...")

    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
    