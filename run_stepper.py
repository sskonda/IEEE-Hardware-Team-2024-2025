#!/bin/python3

from robot.components import StepperMotor

def main():
    motor = StepperMotor(20, 21)
    motor.init()

    try:
        while True:
            motor.set_position(int(input("Position: ")))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()