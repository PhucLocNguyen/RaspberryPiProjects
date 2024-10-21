from gpiozero import DistanceSensor
from time import sleep
import RPi.GPIO as GPIO
trigPin = 18
echoPin = 24
buzzer_pin = 17

sensor = DistanceSensor(echo=echoPin, trigger=trigPin, max_distance=3)

GPIO.setmode(GPIO.BCM)  # Use Broadcom GPIO numbering
GPIO.setup(buzzer_pin, GPIO.OUT)

# Set up PWM at 1000Hz frequency
pwm = GPIO.PWM(buzzer_pin, 2000)  # Use GPIO 17
pwm.start(0)  # Start PWM with 50% duty cycle (volume control)

def loop():
    while True:
        valueSensor =sensor.distance * 100
        print('Distance: ', sensor.distance * 100, 'cm')
        if(valueSensor<=10):
            pwm.ChangeDutyCycle(50)
        else:
            print("Buzzer off")
            pwm.ChangeDutyCycle(0)
        sleep(1)
        
        

if __name__ == '__main__':  # Program entrance
    print('Program is starting...')
    try:
        loop()
    except KeyboardInterrupt:  # Press ctrl-c to end the program.
        sensor.close()
        print("Ending program")