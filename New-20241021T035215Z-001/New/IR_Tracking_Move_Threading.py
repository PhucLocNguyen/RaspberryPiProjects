import RPi.GPIO as GPIO
import time
from gpiozero import AngularServo, DistanceSensor, Robot
import threading
import subprocess

GPIO.setmode(GPIO.BCM)
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
ir_left = 26 # Cảm biến IR bên trái
ir_right = 21  # Cảm biến IR bên phải
GPIO.setmode(GPIO.BCM)  # Use BCM numbering for all pins
GPIO.setup(ir_left, GPIO.IN)  # Cảm biến bên trái là đầu vào
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

global leftToRight, servoAngle,lastServoAngle,OBSTACLE_DISTANCE,OBSTACLE_DISTANCE_LOW,speedOffset,SONIC_TIMEOUT
# Recover data from servo
leftToRight = 0
servoAngle = 0
lastServoAngle = 0
OBSTACLE_DISTANCE = 30
OBSTACLE_DISTANCE_LOW = 25
speedOffset = 0
SONIC_TIMEOUT = 20


def blink(pin):
    GPIO.output(pin, GPIO.HIGH)

def turnOff(pin):
    GPIO.output(pin, GPIO.LOW)

def clearColor():
    turnOff(redPin)
    turnOff(bluePin)
    turnOff(greenPin)

def LightOff():
    turnOff(redPin)
    turnOff(greenPin)
    turnOff(bluePin)

def GetSonar():

    distanceUltrasonic = sensor.distance * 100  # From meter to Centimeter
    return distanceUltrasonic

def Scan():
    global servoAngle, lastServoAngle, OBSTACLE_DISTANCE, OBSTACLE_DISTANCE_LOW, speedOffset, SONIC_TIMEOUT  # Declare global variables
    scanAngle = [180, 100, 30]

    # Initialize variables
    distance = [0] * 3
    # Main logic
    for i in range(3):
        time.sleep(1)
        servo.angle = scanAngle[i]
        time.sleep(0.5)

        distance[i] = GetSonar()
        print(f"Distance initial : {distance[i]}")
        time.sleep(1)
        distance[i] = GetSonar()
        print(f"Distance after 1s: {distance[i]}")

    print(f"distance[0]: { distance[0]}, {scanAngle[0]}")
    print(f"distance[1]: { distance[1]}, {scanAngle[1]}")
    print(f"distance[2]: { distance[2]}, {scanAngle[2]}")
    
    servo.angle = 100
    time.sleep(0.5)
    servo.detach()
    time.sleep(1)
    
    return distance

def ObstacleAvoidance():
    global OBSTACLE_DISTANCE, OBSTACLE_DISTANCE_LOW
    for _ in range(5):
        distance_front = GetSonar()
        time.sleep(0.2)

        if distance_front < OBSTACLE_DISTANCE:
            print(f"Gap vat can: {distance_front}")
            robot.backward()
            time.sleep(0.3)
            robot.stop()

            distances = Scan()

            if distances[0] > OBSTACLE_DISTANCE:
                print("Re trai")
                robot.left()
                time.sleep(0.4)
            elif distances[2] > OBSTACLE_DISTANCE:
                print("Re phai")
                robot.right()
                time.sleep(0.3)
            else:
                print("Quay")
                robot.backward()
                time.sleep(0.5)
                robot.stop()
                robot.left() 
                time.sleep(0.8)
                
            robot.stop()
            time.sleep(1)
        else:
            print("Khong co vat can")
            robot.forward()
            time.sleep(0.2)
def listen_ir():
    global FlagRunningIR_tracking  # Thêm global
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
                if ir_signal == "KEY_DOWN":
                    FlagRunningIR_tracking = False
                    print("Nhấn Keydown đã được phát hiện. Dừng xe...")
                    break


def LineTracking():
    global FlagRunningIR_tracking
    while FlagRunningIR_tracking:
        left_status = GPIO.input(ir_left)  # Đọc giá trị cảm biến trái
        right_status = GPIO.input(ir_right)
        print(f"Left: {left_status}")
        print(f"Right: {right_status}")
        if left_status == 1 and right_status == 1:
            # Cả hai cảm biến trên vạch đen -> Tiến về phía trước
            robot.forward()
            print("Move Forward")
        elif left_status == 0 and right_status == 1:
            # Cảm biến trái trên vạch trắng, cảm biến phải trên vạch đen -> Rẽ phải
            robot.right()
            print("Move Right")

        elif left_status == 1 and right_status == 0:
            # Cảm biến phải trên vạch trắng, cảm biến trái trên vạch đen -> Rẽ trái
            robot.left()
            print("Move Left")

        else:
            # Cả hai cảm biến trên vạch trắng -> Dừng lại
            robot.stop()
            print("All White -> Stop")
        time.sleep(0.1)
        robot.stop()
        time.sleep(1)

def main():
    
    try:
        ir_thread = threading.Thread(target=listen_ir)
        control_thread = threading.Thread(target=LineTracking)
        ir_thread.start()
        control_thread.start()
        ir_thread.join()
        control_thread.join()
        print("Chương trình kết thúc.")
        robot.stop()
        
    except KeyboardInterrupt:
        robot.stop()
        pass
    finally:
        GPIO.cleanup()  # Ensure cleanup happens on exit

if __name__ == "__main__":
    main()
