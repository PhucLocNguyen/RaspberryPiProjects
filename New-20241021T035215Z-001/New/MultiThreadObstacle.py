import RPi.GPIO as GPIO
import time
from gpiozero import AngularServo, Robot
import threading
import subprocess
import math

# Setup GPIO pins
GPIO.setmode(GPIO.BCM)
myServoPin = 18
SERVO_DELAY_SEC = 0.001
FlagRunningObstacle = True

# Setup pin for servo motor
GPIO.setup(myServoPin, GPIO.OUT)
pwm = GPIO.PWM(myServoPin, 50)

pwm.start(0)

# Set pin for Motor L9110s
motorA_1A = 12  # Motor A
motorA_1B = 13  # Motor A
motorB_1A = 11  # Motor B
motorB_1B = 10  # Motor B

# Initialize robot
robot = Robot(right=(motorA_1A, motorA_1B), left=(motorB_1A, motorB_1B))

# Set pin for Ultrasonic sensor
trigPin = 26
echoPin = 24
GPIO.setup(trigPin, GPIO.OUT)
GPIO.setup(echoPin, GPIO.IN)

GPIO.output(trigPin, False)
time.sleep(0.1)

OBSTACLE_DISTANCE = 45
OBSTACLE_DISTANCE_LOW = 25

def set_servo_angle(angle):
    duty = 2 + (angle / 18)
    GPIO.output(myServoPin, True)
    pwm.ChangeDutyCycle(duty)
    time.sleep(1)
    GPIO.output(myServoPin, False)
    pwm.ChangeDutyCycle(0)

'''
def GetDistance():
    GPIO.output(trigPin, True)
    time.sleep(0.00001)
    GPIO.output(trigPin, False)

    while GPIO.input(echoPin) == 0:
        start = time.time()
    while GPIO.input(echoPin) == 1:
        stop = time.time()

    elapsed = stop - start
    distanceUltrasonic = elapsed * 34600 / 2
    return distanceUltrasonic
'''

def GetDistance():
    GPIO.output(trigPin, False)
    time.sleep(0.5)

    GPIO.output(trigPin, True)
    time.sleep(0.00001)
    GPIO.output(trigPin, False)

    while GPIO.input(echoPin) == 0:
        pulse_start = time.time()

    while GPIO.input(echoPin) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start

    distance = pulse_duration * 17150
    distance = round(distance, 2)

    return distance

def Scan():
    scanAngle = [135, 80, 45]
    distance = [0] * 3

    for i in range(3):
        set_servo_angle(scanAngle[i])
        time.sleep(0.5)
        distance[i] = GetDistance()  # Sửa thành GetDistance()
        time.sleep(0.2)
    set_servo_angle(80)
    print(f"Distance: {distance}")
    return distance


def listen_ir():
    global FlagRunningObstacle  # Thêm global
    while True:
        print("Đang lắng nghe tín hiệu IR...")
        time.sleep(1)
        process = subprocess.Popen(['irw'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        output = process.stdout.readline()
        if output:
            decoded_output = output.decode('utf-8').strip()
            parts = decoded_output.split()
            if len(parts) >= 4:
                ir_signal = parts[2]
                print(ir_signal)
                if ir_signal == "KEY_DOWN":
                    FlagRunningObstacle = False
                    print("Nhấn Keydown đã được phát hiện. Dừng xe...")
                    break


def RunObstacle():
    global FlagRunningObstacle
    time.sleep(5)
    while FlagRunningObstacle:
        distance_front = GetDistance()
        print(f"Distance: {distance_front}")
        time.sleep(0.1)

        if distance_front < OBSTACLE_DISTANCE:
            robot.backward()
            time.sleep(0.2)
            robot.stop()
            time.sleep(0.2)
            distance = Scan()
 
            if distance[0] >= OBSTACLE_DISTANCE:
                robot.left()
                time.sleep(0.3)
            elif distance[2] >= OBSTACLE_DISTANCE:
                robot.right()
                time.sleep(0.3)
            else:
                robot.backward()
                time.sleep(0.3)
                robot.stop()
                time.sleep(0.1)
                robot.left()
                time.sleep(0.6)
            robot.stop()
        else:
            #timeSleepForward = math.floor(((distance_front - OBSTACLE_DISTANCE)) / 0.708) / 100
            robot.forward()
            time.sleep(0.1)


def main():
    try:
        robot.forward()
        '''
        ir_thread = threading.Thread(target=listen_ir)
        control_thread = threading.Thread(target=RunObstacle)

        # Khởi động các luồng
        ir_thread.start()
        control_thread.start()

        ir_thread.join()
        control_thread.join()
        print("Chương trình kết thúc.")
        robot.stop()
        '''
        '''
        while True:
            print("start")
            distance_front = GetDistance()
            print(f"Distance: {distance_front}")
            time.sleep(0.2)
        
        distance_front = GetDistance()
        print(f"Distance: {distance_front}")
        '''

    except KeyboardInterrupt:
        print("Program interrupted")
        robot.stop()
        GPIO.cleanup()
    finally:
        robot.stop()
        GPIO.cleanup() 
        print("GPIO cleanup")


if __name__ == "__main__":
    main()
