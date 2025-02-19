#!/bin/python3

from robot import PI, BIN_LIFT_STEPPER
from robot.components import StepperMotor

def main():
    motor = BIN_LIFT_STEPPER
    motor.init(PI)

    try:
        while True:
            motor.set_position(int(input("Position: ")))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()