#!/bin/python3

from robot.components import BrushedMotor

def main():
    motor = BrushedMotor(17, 18)
    motor.init()

    try:
        while True:
            motor.set_duty(float(input("Duty Cyle: ")))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()