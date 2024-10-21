import RPi.GPIO as GPIO
import time
from gpiozero import Robot


GPIO.setmode(GPIO.BCM)  # Use BCM numbering for all pins
GPIO.setwarnings(False)
# GPIO setup
left_sensor_pin = 17  
center_sensor_pin = 27  
right_sensor_pin = 22  
# Set pin for Motor L9110s
motorA_1A = 12  # Motor A
motorA_1B = 13  # Motor A
motorB_1A = 10  # Motor B
motorB_1B = 11  # Motor B


GPIO.setup(left_sensor_pin, GPIO.IN)  # Cảm biến bên trái là đầu vào
GPIO.setup(right_sensor_pin, GPIO.IN)  # Cảm biến bên phải là đầu vào
GPIO.setup(center_sensor_pin, GPIO.IN)

robot = Robot(right=(11, 10), left=(13, 12))

Kp = 1.5  # Proportional
Ki = 1.0  # Integral
Kd = 1.0 # Derivative

previous_error = 0
integral = 0

def read_sensors():
    left = GPIO.input(left_sensor_pin)
    center = GPIO.input(center_sensor_pin)
    right = GPIO.input(right_sensor_pin)
    return left, center, right

def calculate_error(left, center, right):
    if left == 1 and center == 0 and right == 0:
        return 1
    elif left == 0 and center == 0 and right == 1:
        return -1
    elif center == 1:
        return 0 
    return 0 

def pid_control(error):
    global previous_error, integral
    
    # Proportional
    proportional = error
    
    # Integral
    integral += error
    
    # Derivative
    derivative = error - previous_error
    
    # PID Output
    output = Kp * proportional + Ki * integral + Kd * derivative
    
    previous_error = error
    
    return output

def set_motor_speed(pid_output):
    base_speed = 0.5 

    left_motor_speed = base_speed - pid_output / 100
    right_motor_speed = base_speed + pid_output / 100

    left_motor_speed = max(min(left_motor_speed, 1), 0)
    right_motor_speed = max(min(right_motor_speed, 1), 0)
    
    print(f"Left: {left_motor_speed}, Right: {right_motor_speed}")
    
    robot.left_motor.value = left_motor_speed
    robot.right_motor.value = right_motor_speed

    
def main():
    try:
        while True:
            left, center, right = read_sensors()
        
            error = calculate_error(left, center, right)
        
            pid_output = pid_control(error)
        
            set_motor_speed(pid_output)
        
            time.sleep(0.05)
        '''
        robot.left()
        time.sleep(0.5)
        robot.stop()
        '''
        
    except KeyboardInterrupt:
        print("Program interrupted")
    finally:
        robot.stop()
        GPIO.cleanup()  # Ensure cleanup happens on exit
        print("GPIO cleanup")

if __name__ == "__main__":
    main()

