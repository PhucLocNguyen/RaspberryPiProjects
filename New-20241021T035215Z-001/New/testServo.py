import RPi.GPIO as GPIO
import time
from gpiozero import Robot

GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT)
# Set pin for Motor L9110s
motorA_1A = 12  # Motor A
motorA_1B = 13  # Motor A
motorB_1A = 11  # Motor B
motorB_1B = 10  # Motor B
# Initialize robot
robot = Robot(right=(motorA_1A, motorA_1B), left=(motorB_1A, motorB_1B))

pwm = GPIO.PWM(18, 50)

pwm.start(0)

def set_servo_angle(angle):
    duty = 2 + (angle / 18)
    GPIO.output(18, True)
    pwm.ChangeDutyCycle(duty)
    time.sleep(1)
    GPIO.output(18, False)
    pwm.ChangeDutyCycle(0)

try:
    #robot.forward()
    while True:
        set_servo_angle(0)
        time.sleep(2)
        set_servo_angle(90)
        time.sleep(1)
        set_servo_angle(180)
        time.sleep(1)

except KeyboardInterrupt:
    print("Ending")

finally:
    #robot.stop()
    pwm.stop() 
    GPIO.cleanup() 
