import RPi.GPIO as GPIO
import time
from gpiozero import Robot

# GPIO setup
left_sensor = 17  
center_sensor = 27  
right_sensor = 22  

# Set pin for Motor L9110s
motorA_1A = 12  # Motor A
motorA_1B = 13  # Motor A
motorB_1A = 10  # Motor B
motorB_1B = 11  # Motor B

GPIO.setmode(GPIO.BCM)  # Use BCM numbering for all pins
GPIO.setup(left_sensor, GPIO.IN)  
GPIO.setup(right_sensor, GPIO.IN)  
GPIO.setup(center_sensor, GPIO.IN)

#robot = Robot(right=(12, 13), left=(11, 10))
robot = Robot(right=(10, 11), left=(12, 13))

#Kp = 2.95
Kp = 1.569
Kd = 0
#Kd = 0.1
#Ki = 0.1268
Ki = 0
error = 0
pre_error = 0
sum_error = 0


base_speed = 0.4
max_speed = 0.9
min_speed = 0.1

def set_robot_speed(left_motor, right_motot):
    robot.value = (left_motor, right_motot)

def calculate_error(left_value, right_value, center_value):
    global error
    if left_value == GPIO.LOW and center_value == GPIO.HIGH  and right_value == GPIO.LOW:
        error = 0
    elif left_value == GPIO.HIGH and center_value == GPIO.HIGH  and right_value == GPIO.LOW:
        error = -1
        '''
    elif left_value == GPIO.HIGH and center_value == GPIO.LOW  and right_value == GPIO.LOW:
        error = -2
        '''
    elif left_value == GPIO.LOW and center_value == GPIO.HIGH  and right_value == GPIO.HIGH:
        error = 1
        '''
    elif left_value == GPIO.LOW and center_value == GPIO.LOW  and right_value == GPIO.HIGH:
        error = 2
        '''
    
def calculate_pid():
    global error, pre_error, sum_error

    sum_error += error
    sum_error = max(min(sum_error, 10), -10) 

    pid_output = Kp * error + Kd * (error - pre_error) + Ki * sum_error

    pid_output = max(min(pid_output, 0.5), -0.5)

    pre_error = error
    return pid_output

def set_motor_speed(error):
    global pre_error, sum_error

    motor_speed = calculate_pid()

    left_speed = base_speed + motor_speed
    right_speed = base_speed - motor_speed

    left_speed = max(min(left_speed, max_speed), min_speed)
    right_speed = max(min(right_speed, max_speed), min_speed)
	
    robot.value = (left_speed, right_speed)
    pre_error = error

try:
    time.sleep(2)
    while True:
        left_value = GPIO.input(left_sensor)
        right_value = GPIO.input(right_sensor)
        center_value = GPIO.input(center_sensor)

        calculate_error(left_value, right_value, center_value)
        set_motor_speed(error)
        time.sleep(0.008)
except KeyboardInterrupt:
    robot.stop()
    GPIO.cleanup()
