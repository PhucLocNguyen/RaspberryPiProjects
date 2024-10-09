import RPi.GPIO as GPIO
import time

# Use BCM pin numbering
GPIO.setmode(GPIO.BCM)

# Set up pin 18 for PWM
pwm_pin = 18
GPIO.setup(pwm_pin, GPIO.OUT)

# Set frequency to 1Hz
pwm = GPIO.PWM(pwm_pin, 1)

# Start PWM with a duty cycle of 50%
pwm.start(50)