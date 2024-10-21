from gpiozero import Robot
import RPi.GPIO as GPIO
import time

sensor_left = 21
sensor_right = 26

robot = Robot(left=(11, 10), right=(12, 13))

GPIO.setmode(GPIO.BCM)
GPIO.setup(sensor_left, GPIO.IN)
GPIO.setup(sensor_right, GPIO.IN)

Kp = 1.5 
Ki = 0.0 
Kd = 0.2

previous_error = 0
integral = 0

def set_robot_speed(output):
    base_speed = 0.3 

    left_motor_speed = base_speed + output
    right_motor_speed = base_speed - output

    left_motor_speed = max(0, min(1, left_motor_speed))
    right_motor_speed = max(0, min(1, right_motor_speed))

    robot.value = (left_motor_speed, right_motor_speed)

try:
    while True:
        left_state = GPIO.input(sensor_left)
        right_state = GPIO.input(sensor_right)

        error = 0
        if left_state == 1 and right_state == 0:
            error = -1 
        elif left_state == 0 and right_state == 1:
            error = 1  
        elif left_state == 1 and right_state == 1:
            error = 0  
        proportional = error
        integral += error * 0.1 
        derivative = error - previous_error

        output = Kp * proportional + Ki * integral + Kd * derivative

        set_robot_speed(output)

        previous_error = error

        time.sleep(0.1)

except KeyboardInterrupt:
    robot.stop() 
    GPIO.cleanup()
