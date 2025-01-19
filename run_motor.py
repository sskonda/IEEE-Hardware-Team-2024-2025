from robot.components import DutyMotor

def main():
    motor = DutyMotor(17, 18, None)
    motor.init()

    try:
        while True:
            motor.set_duty(float(input("Duty Cyle: ")))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
    