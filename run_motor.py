#!/bin/python3
import pigpio
from robot.components import BrushedMotor

def main():
    motor = BrushedMotor(17, 18)
    motor.init(pi=pigpio.pi())

    try:
        while True:
            motor.set_duty(float(input("Duty Cyle: ")))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()