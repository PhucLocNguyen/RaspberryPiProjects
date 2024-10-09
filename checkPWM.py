import RPi.GPIO as GPIO
import time
import threading

GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT)

pwm = GPIO.PWM(18, 50)

duty_cycle = 0

def set_pwm():
    global duty_cycle
    pwm.start(0)
    try:
        while True:
            for dc in range(0, 101, 5):
                pwm.ChangeDutyCycle(dc)
                duty_cycle = dc
                print(f"Setting duty cycle: {dc}%")
                time.sleep(0.5)
            for dc in range(100, -1, -5):
                pwm.ChangeDutyCycle(dc)
                duty_cycle = dc
                print(f"Setting duty cycle: {dc}%")
                time.sleep(0.5)
    except KeyboardInterrupt:
        pass

def read_pwm():
    global duty_cycle
    try:
        while True:
            print(f"Current PWM duty cycle: {duty_cycle}%")
            time.sleep(1)
    except KeyboardInterrupt:
        pass

thread1 = threading.Thread(target=set_pwm)
thread2 = threading.Thread(target=read_pwm)

thread1.start()
thread2.start()

thread1.join()
thread2.join()

pwm.stop()
GPIO.cleanup()