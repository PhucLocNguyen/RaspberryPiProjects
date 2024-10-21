import RPi.GPIO as GPIO
import time
from gpiozero import Robot

# GPIO setup
left_sensor = 17 # GPIO pin for left sensor
right_sensor = 22  # GPIO pin for right sensor
center_sensor = 27 #GPIO pin for center sensor

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
robot = Robot(right=(11, 10), left=(13, 12))

Kp = 1
Kd = 0.1
Ki = 0.01

error = 0
pre_error = 0
sum_error = 0


base_speed = 0.5 
max_speed = 1
min_speed = 0

def set_robot_speed(left_motor, right_motot):
    robot.value = (left_motor, right_motot)

def calculate_error(left_value, right_value, center_value):
    global error
    if left_value == GPIO.LOW and center_value == GPIO.HIGH  and right_value == GPIO.LOW:
        error = 0
    elif left_value == GPIO.HIGH and center_value == GPIO.HIGH  and right_value == GPIO.LOW:
        error = -1
    elif left_value == GPIO.HIGH and center_value == GPIO.LOW  and right_value == GPIO.LOW:
        error = -2
    elif left_value == GPIO.LOW and center_value == GPIO.HIGH  and right_value == GPIO.HIGH:
        error = 1
    elif left_value == GPIO.LOW and center_value == GPIO.LOW  and right_value == GPIO.HIGH:
        error = 2
    
def calculate_pid():
    global error, pre_error, sum_error
    return Kp*error + Kd*(error - pre_error) + Ki*(sum_error)

def set_motor_speed(error):
    global pre_error, sum_error

    motor_speed = calculate_pid()

    left_speed = base_speed + motor_speed
    right_speed = base_speed - motor_speed


    if(left_speed > max_speed):
        left_speed = max_speed
    if(right_speed > max_speed):
        right_speed = max_speed

    if(left_speed < -max_speed):
        left_speed = -max_speed
    if(right_speed < -max_speed):
        right_speed = -max_speed


    robot.value = (left_speed,right_speed)
    pre_error = error
    sum_error += error

try:
    pre_straight = "inative"
    pre_turn = "inative"
    robot.value = (0,0)
    while True:
        left_value = GPIO.input(left_sensor)
        right_value = GPIO.input(right_sensor)
        center_value = GPIO.input(center_sensor)

        if left_value == GPIO.LOW and center_value == GPIO.HIGH  and right_value == GPIO.LOW:
            robot.forward()
            pre_straight = "forward"
            print("Trang - Den - Trang")
        elif left_value == GPIO.HIGH and center_value == GPIO.HIGH  and right_value == GPIO.LOW:
            calculate_error(left_value, right_value, center_value)
            set_motor_speed(error)
            pre_turn = "left"
            print("Den - Den - Trang")
        elif left_value == GPIO.HIGH and center_value == GPIO.LOW  and right_value == GPIO.LOW:
            calculate_error(left_value, right_value, center_value)
            set_motor_speed(error)
            pre_turn = "left"
            print("Den - Trang - Trang")
        elif left_value == GPIO.LOW and center_value == GPIO.HIGH  and right_value == GPIO.HIGH:
            calculate_error(left_value, right_value, center_value)
            set_motor_speed(error)
            pre_turn = "right"
            print("Trang - Den - Den")
        elif left_value == GPIO.LOW and center_value == GPIO.LOW  and right_value == GPIO.HIGH:
            calculate_error(left_value, right_value, center_value)
            set_motor_speed(error)
            pre_turn = "right"
            print("Trang - Trang - Den")
        elif left_value == GPIO.HIGH and center_value == GPIO.HIGH  and right_value == GPIO.HIGH:  
            print("Den - Den - Den")
            if pre_turn == "right" :
                set_motor_speed(1)
                pre_turn = "right" 
            elif pre_turn == "left":
                set_motor_speed(-1)
                pre_turn = "right" 
        elif left_value == GPIO.LOW and center_value == GPIO.LOW  and right_value == GPIO.LOW:  
            print("Trang - Trang - Trang")
            if pre_straight == "foward":
                robot.forward()
                pre_straight = "inative"
            else:
                robot.value = (0.85,-0.85)
                time.sleep(0.671)
                robot.stop()
                time.sleep(0.4)
                robot.forward()
                time.sleep(0.3)
                
        time.sleep(0.2)

except KeyboardInterrupt:

    set_motor_speed(0, 0)
    GPIO.cleanup()
