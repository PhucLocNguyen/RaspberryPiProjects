import RPi.GPIO as GPIO
import time

# Setup GPIO mode
GPIO.setmode(GPIO.BCM)

# Define motor control pins
# Motor A (left)
motorA_1A = 13  # Connect to A-1A on L9110S (Motor A)
motorA_1B = 12  # Connect to A-1B on L9110S (Motor A)

# Motor B (right)
motorB_1A = 11  # Connect to B-1A on L9110S (Motor B)
motorB_1B = 10  # Connect to B-1B on L9110S (Motor B)

# Setup GPIO pins as output
GPIO.setup(motorA_1A, GPIO.OUT)
GPIO.setup(motorA_1B, GPIO.OUT)
GPIO.setup(motorB_1A, GPIO.OUT)
GPIO.setup(motorB_1B, GPIO.OUT)

# Functions to control motor directions
def motor_left_forward():
    GPIO.output(motorA_1A, GPIO.HIGH)
    GPIO.output(motorA_1B, GPIO.LOW)

def motor_left_backward():
    GPIO.output(motorA_1A, GPIO.LOW)
    GPIO.output(motorA_1B, GPIO.HIGH)

def motor_right_forward():
    GPIO.output(motorB_1A, GPIO.HIGH)
    GPIO.output(motorB_1B, GPIO.LOW)

def motor_right_backward():
    GPIO.output(motorB_1A, GPIO.LOW)
    GPIO.output(motorB_1B, GPIO.HIGH)

def stop_motors():
    GPIO.output(motorA_1A, GPIO.LOW)
    GPIO.output(motorA_1B, GPIO.LOW)
    GPIO.output(motorB_1A, GPIO.LOW)
    GPIO.output(motorB_1B, GPIO.LOW)

# Test the motors
try:
    while True:
        print("Forward")
        motor_left_forward()
        time.sleep(0.2)
        motor_right_forward()
        time.sleep(2)

        print("Stop")
        stop_motors()
        time.sleep(2)

except KeyboardInterrupt:
    # Cleanup GPIO when program is interrupted
    GPIO.cleanup()
    print("error")