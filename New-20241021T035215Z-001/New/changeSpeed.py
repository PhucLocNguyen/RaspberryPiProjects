import RPi.GPIO as GPIO
import time

# Pin Definitions for Motor A and Motor B
motorA_1A = 12  # GPIO pin for Motor A IA
motorA_1B = 13  # GPIO pin for Motor A IB
motorB_1A = 11  # GPIO pin for Motor B IA
motorB_1B = 10  # GPIO pin for Motor B IB

# Setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(motorA_1A, GPIO.OUT)
GPIO.setup(motorA_1B, GPIO.OUT)
GPIO.setup(motorB_1A, GPIO.OUT)
GPIO.setup(motorB_1B, GPIO.OUT)

# Set up PWM for both motors (1 kHz frequency)
pwm_motorA_1A = GPIO.PWM(motorA_1A, 1000)
pwm_motorA_1B = GPIO.PWM(motorA_1B, 1000)
pwm_motorB_1A = GPIO.PWM(motorB_1A, 1000)
pwm_motorB_1B = GPIO.PWM(motorB_1B, 1000)

# Start PWM with 0% duty cycle (motors off)
pwm_motorA_1A.start(0)
pwm_motorA_1B.start(0)
pwm_motorB_1A.start(0)
pwm_motorB_1B.start(0)

# Test with increasing speed
try:
    for speed in range(0, 101, 20):  # From 0 to 100% duty cycle, step by 20
        print(f"Setting speed to {speed}%")
        pwm_motorA_1A.ChangeDutyCycle(speed)
        pwm_motorA_1B.ChangeDutyCycle(0)
        pwm_motorB_1A.ChangeDutyCycle(speed)
        pwm_motorB_1B.ChangeDutyCycle(0)
        time.sleep(2)
except KeyboardInterrupt:
    pass

# Stop the motors
pwm_motorA_1A.ChangeDutyCycle(0)
pwm_motorA_1B.ChangeDutyCycle(0)
pwm_motorB_1A.ChangeDutyCycle(0)
pwm_motorB_1B.ChangeDutyCycle(0)

# Cleanup
pwm_motorA_1A.stop()
pwm_motorA_1B.stop()
pwm_motorB_1A.stop()
pwm_motorB_1B.stop()
GPIO.cleanup()
pwm_motorB_1B.stop()
GPIO.cleanup()
