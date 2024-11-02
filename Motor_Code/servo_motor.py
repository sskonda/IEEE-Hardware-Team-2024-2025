import RPi.GPIO as GPIO
from time import sleep

servoPin = 12
GPIO.setmode(GPIO.BOARD)
GPIO.setup(servoPin, GPIO.OUT)
servo_motor = GPIO.PWM(servoPin, 50)
servo_motor.start(0)

def servo_move(pin, position): 
    duty = position*(7/100) + 4
    GPIO.setwarnings(False)
    pin.ChangeDutyCycle(duty)
    
while True: 
    for duty in range(0, 101, 1): 
        servo_move(servo_motor, duty)
        sleep(0.05)
    for duty in range(100, 0, -1): 
        servo_move(servo_motor, duty)
        sleep(0.05)