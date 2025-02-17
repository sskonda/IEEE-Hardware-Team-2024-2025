#!/bin/python3

import pigpio
from robot.components import ServoMotor

def main():
    motor = ServoMotor(17)
    motor.init(pi=pigpio.pi())

    try:
        while True:
            motor.set_position(float(input("Position: ")))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()