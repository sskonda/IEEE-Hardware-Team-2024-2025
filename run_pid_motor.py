#!/bin/python3
import pigpio
from robot import RIGHT_PID_MOTOR, LEFT_PID_MOTOR

def main():
    LEFT_PID_MOTOR.init(pi=pigpio.pi())
    RIGHT_PID_MOTOR.init(pi=pigpio.pi())

    try:
        while True:
            LEFT_PID_MOTOR.stop()
            RIGHT_PID_MOTOR.stop()
            rpm = float(input("RPM: "))
            LEFT_PID_MOTOR.set_speed(rpm)
            RIGHT_PID_MOTOR.set_speed(rpm)
            LEFT_PID_MOTOR.pid().reset()
            RIGHT_PID_MOTOR.pid().reset()
            try:
                while True:
                        LEFT_PID_MOTOR.update()
                        RIGHT_PID_MOTOR.update()
            except KeyboardInterrupt:
                pass
    except KeyboardInterrupt:
        pass
    finally:
        LEFT_PID_MOTOR.stop()
        RIGHT_PID_MOTOR.stop()
         

if __name__ == "__main__":
    main()