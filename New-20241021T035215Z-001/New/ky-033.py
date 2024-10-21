import RPi.GPIO as GPIO
import time
from gpiozero import AngularServo, DistanceSensor, Robot
import threading
import subprocess

# Setup GPIO pins
myServoPin = 18
SERVO_DELAY_SEC = 0.001
servo = AngularServo(myServoPin, initial_angle=100, min_angle=0,max_angle=180)

# Set pin for Ultrasonic sensor
trigPin = 5
echoPin = 24
buzzer_pin = 17
#set pin for RGB LED
redPin = 25
greenPin = 22
bluePin = 27
# Set pin for Motor L9110s
motorA_1A = 12  # Motor A
motorA_1B = 13  # Motor A
motorB_1A = 10  # Motor B
motorB_1B = 11  # Motor B

FlagRunningIR_tracking = True
ir_center = 21 # Cảm biến IR bên trái
ir_right = 26  # Cảm biến IR bên phải
GPIO.setmode(GPIO.BCM)  # Use BCM numbering for all pins
GPIO.setup(ir_center, GPIO.IN)  # Cảm biến bên trái là đầu vào
GPIO.setup(ir_right, GPIO.IN)  # Cảm biến bên phải là đầu vào
sensor = DistanceSensor(echo=echoPin, trigger=trigPin, max_distance=3)

robot = Robot(right=(12, 13), left=(11, 10))


# Set up motor pins as output
GPIO.setup(buzzer_pin, GPIO.OUT)
pwm = GPIO.PWM(buzzer_pin, 1000)  # 1000Hz frequency
pwm.start(0)  # Start PWM with 0% duty cycle (off)

# Set up RGB pins as output
GPIO.setup(redPin, GPIO.OUT)
GPIO.setup(greenPin, GPIO.OUT)
GPIO.setup(bluePin, GPIO.OUT)

try:
    while True:
        # Read sensor values
        center_status = GPIO.input(ir_center)  # Đọc giá trị cảm biến trái
        right_status = GPIO.input(ir_right)
        print(f"Left: {center_status}")
        print(f"Right: {right_status}")
        if center_status == 1 and right_status == 0:
            # Cả hai cảm biến trên vạch đen -> Tiến về phía trước
            robot.forward()
            time.sleep(0.05)
            print("Move Forward")
        elif center_status == 1 and right_status == 1:
            # Cảm biến trái trên vạch trắng, cảm biến phải trên vạch đen -> Rẽ phải
            robot.right ()
            time.sleep(0.04)
            print("Move Right")

        elif center_status == 0 and right_status == 0:
            # Cảm biến phải trên vạch trắng, cảm biến trái trên vạch đen -> Rẽ trái
            #robot.backward()
            #time.sleep(0.03)
            robot.left()
            time.sleep(0.04)
            #robot.forward()
            #time.sleep(0.05)
            print("Move Left")

        else:
            # Cả hai cảm biến trên vạch trắng -> Dừng lại
            robot.forward()
            time.sleep(0.04)
            robot.right()
            time.sleep(0.05)
            print("All White -> Stop")
        robot.stop()
        time.sleep(0.5)

except KeyboardInterrupt:
    pass
finally:
    GPIO.cleanup()  # Clean up GPIO when exiting
