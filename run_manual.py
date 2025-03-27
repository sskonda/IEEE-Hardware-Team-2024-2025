from robot import main

if __name__ == "__main__":
    import tty
    import sys
    import termios
    from robot import *
    
    pi = pigpio.pi()
    
    for component in ROBOT:
        print("Initializing", component)
        ROBOT[component].init(pi)

    orig_settings = termios.tcgetattr(sys.stdin)

    tty.setcbreak(sys.stdin)
    x = 0
    try:
        while x != chr(27): # ESC
            x=sys.stdin.read(1)[0]
            print("You pressed", ord(x))
            match x:
                case 'i': 
                    print("Run Intake")
                    INTAKE_MOTOR.set_duty(1)
                case 'o':
                    print("Spit Intake")
                    INTAKE_MOTOR.set_duty(-1)
                case '0':
                    BIN_LIFT_STEPPER.set_position(0)
                case '1':
                    BIN_LIFT_STEPPER.set_position(1)
                case '2':
                    BIN_LIFT_STEPPER.set_position(2)
                case '3':
                    BIN_LIFT_STEPPER.set_position(3)
                case 'w':
                    print("Forward")
                    RIGHT_DRIVE_MOTOR.set_duty(DRIVE_SPEED)
                    LEFT_DRIVE_MOTOR.set_duty(DRIVE_SPEED)
                case 'a':
                    print("Left")
                    RIGHT_DRIVE_MOTOR.set_duty(DRIVE_SPEED)
                    LEFT_DRIVE_MOTOR.set_duty(-DRIVE_SPEED)
                case 'd':
                    print("Right")
                    RIGHT_DRIVE_MOTOR.set_duty(-DRIVE_SPEED)
                    LEFT_DRIVE_MOTOR.set_duty(DRIVE_SPEED)
                case 's':
                    print("Backward")
                    RIGHT_DRIVE_MOTOR.set_duty(-DRIVE_SPEED)
                    LEFT_DRIVE_MOTOR.set_duty(-DRIVE_SPEED)
                case 'j':
                    print("Clamp Tighten")
                    CLAMP_MOTOR.set_duty(-1)
                    print(CLAMP_ENCODER.get_speed())
                case 'k':
                    print("Clamp Release")
                    CLAMP_MOTOR.set_duty(1)
                    print(CLAMP_ENCODER.get_speed())
                case 'b':
                    print("Drop Beacon")
                    if BEACON_SERVO.get_desired_position() < 0.75:
                        BEACON_SERVO.set_position(0.75)
                    else:
                        BEACON_SERVO.set_position(0.0)
                case ' ':
                    print("Stop")
                    for component in ROBOT.values():
                        if component.initialized and hasattr(component, "stop"):
                            component.stop()
    except KeyboardInterrupt as e:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, orig_settings)   

        for component in ROBOT.values():
            component.release()
        pi.stop()
