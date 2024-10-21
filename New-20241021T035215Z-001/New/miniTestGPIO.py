import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT)

try:
    GPIO.output(18, GPIO.HIGH)
    time.sleep(2)
    GPIO.output(18, GPIO.LOW)
finally:
    GPIO.cleanup()
