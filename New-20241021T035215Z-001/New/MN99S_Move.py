import RPi.GPIO as GPIO
from time import sleep
from gpiozero import AngularServo
import subprocess
import threading
import bluetooth  

# Pin configuration
in1 = 23
in2 = 24
en = 13  # Speed control pin for motor
myServoPin = 18
SERVO_DELAY_SEC = 0.001

# Set up servo (for steering - bánh trước)
servo = AngularServo(myServoPin, initial_angle=90, min_angle=0, max_angle=180)

temp1 = 1  # 1 = forward, 0 = backward

# Set up motor pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(in1, GPIO.OUT)
GPIO.setup(in2, GPIO.OUT)
GPIO.setup(en, GPIO.OUT)

# Initialize PWM for speed control

p = GPIO.PWM(en, 1000)
p.start(50)  # Default speed 25%

print("\n")
print("The default speed & direction of motor is LOW & Forward.....")
print("r-run s-stop f-forward b-backward l-left r-right low-medium-high e-exit")
print("\n")

while True:

    x = input()

    if x == 'run':  # run
        print("run")
        if temp1 == 1:
            GPIO.output(in1, GPIO.HIGH)
            GPIO.output(in2, GPIO.LOW)
            print("forward")
        else:
            GPIO.output(in1, GPIO.LOW)
            GPIO.output(in2, GPIO.HIGH)
            print("backward")

    elif x == 's':  # stop
        print("stop")
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.LOW)

    elif x == 'f':  # forward
        print("forward")
        GPIO.output(in1, GPIO.HIGH)
        GPIO.output(in2, GPIO.LOW)
        temp1 = 1

    elif x == 'b':  # backward
        print("backward")
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.HIGH)
        temp1 = 0

    elif x == 'l':  # left (turn the servo to the left)
        print("turn left")
        servo.angle = 30  # Adjust the servo to turn left (angle can be adjusted based on your setup)
        sleep(SERVO_DELAY_SEC)

    elif x == 'r':  # right (turn the servo to the right)
        print("turn right")
        servo.angle = 150  # Adjust the servo to turn right (angle can be adjusted based on your setup)
        sleep(SERVO_DELAY_SEC)

    elif x == 'low':  # low speed
        print("low speed")
        p.ChangeDutyCycle(10)  # Set PWM duty cycle for low speed

    elif x == 'medium':  # medium speed
        print("medium speed")
        p.ChangeDutyCycle(50)  # Set PWM duty cycle for medium speed

    elif x == 'high':  # high speed
        print("high speed")
        p.ChangeDutyCycle(75)  # Set PWM duty cycle for high speed

    elif x == 'e':  # exit the program and clean up GPIO
        GPIO.cleanup()
        print("GPIO Clean up")
        break

    else:
        print("<<< wrong data >>>")
        print("please enter the defined data to continue.....")
    
sleep(0.5)

