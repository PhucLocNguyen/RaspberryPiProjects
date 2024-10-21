import RPi.GPIO as GPIO
import time
from gpiozero import AngularServo, Robot
import subprocess
import threading 

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)  

# Setup GPIO pins
'''myServoPin = 18
SERVO_DELAY_SEC = 0.001
servo = AngularServo(myServoPin, initial_angle=100, min_angle=0, max_angle=180)

# Set pin for Ultrasonic sensor
trigPin = 5
echoPin = 24
buzzer_pin = 17
'''
# Set pin for RGB LED
redPin = 25
greenPin = 22
bluePin = 27

# Set pin for Motor L9110s
motorA_1A = 12  # Motor A
motorA_1B = 13  # Motor A
motorB_1A = 11  # Motor B
motorB_1B = 10  # Motor B

# Set pin for IR Receiver and Line Tracking Sensor
#tracking_pinL = 23  
ir_pin = 16  # IR Receiver

# Setup GPIO pins
GPIO.setup(ir_pin, GPIO.IN)
#GPIO.setup(trigPin, GPIO.OUT)
#GPIO.setup(echoPin, GPIO.IN)
#GPIO.setup(tracking_pinL, GPIO.IN)

# Setup Motor pins as output
'''GPIO.setup(buzzer_pin, GPIO.OUT)
pwm = GPIO.PWM(buzzer_pin, 1000)  # 1000Hz frequency
pwm.start(0) ''' # Start PWM with 0% duty cycle (off)

# Setup RGB pins as output
GPIO.setup(redPin, GPIO.OUT)
GPIO.setup(greenPin, GPIO.OUT)
GPIO.setup(bluePin, GPIO.OUT)

# Initialize robot
robot = Robot(right=(motorA_1A, motorA_1B), left=(motorB_1A, motorB_1B))

# Global variables
leftToRight = 0
servoAngle = 0
lastServoAngle = 0
OBSTACLE_DISTANCE = 30
OBSTACLE_DISTANCE_LOW = 25
speedOffset = 0
SONIC_TIMEOUT = 20
IR_RUNNING=True
def blink(pin):
    GPIO.output(pin, GPIO.HIGH)

def turnOff(pin):
    GPIO.output(pin, GPIO.LOW)

def clearColor():
    turnOff(redPin)
    turnOff(bluePin)
    turnOff(greenPin)

'''def GetSonar():
    GPIO.output(trigPin, True)
    time.sleep(0.00001)
    GPIO.output(trigPin, False)
    start = time.time()
    while GPIO.input(echoPin) == 0:
        start = time.time()
    while GPIO.input(echoPin) == 1:
        stop = time.time()
    elapsed = stop - start
    distanceUltrasonic = elapsed * 34000 / 2
    return distanceUltrasonic

def Scan():
    global servoAngle, lastServoAngle, OBSTACLE_DISTANCE, OBSTACLE_DISTANCE_LOW, speedOffset, SONIC_TIMEOUT
    scanAngle = [180, 100, 30]
    distance = [0] * 3

    for i in range(3):
        servo.angle = scanAngle[i]
        time.sleep(0.5)
        distance[i] = GetSonar()
        print(f"Distance initial : {distance[i]}")
        distance[i] = GetSonar()
        print(f"Distance after 1s: {distance[i]}")

    print(f"distance[0]: { distance[0]}, {scanAngle[0]}")
    print(f"distance[1]: { distance[1]}, {scanAngle[1]}")
    print(f"distance[2]: { distance[2]}, {scanAngle[2]}")
    
    servo.angle = 100
    time.sleep(0.5)
    
    return distance

def ObstacleAvoidance():
    global OBSTACLE_DISTANCE, OBSTACLE_DISTANCE_LOW
    for _ in range(12):
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
            print(f"Khong co vat can :{distance_front}")
            robot.forward()
            time.sleep(0.2)
'''

def read_ir_signal():
    process = subprocess.Popen(['irw'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    output = process.stdout.readline()
    if output:
        decoded_output = output.decode('utf-8').strip()
        parts = decoded_output.split()
        if len(parts) >= 4:
            ir_signal = parts[2]
            print(f"Received IR signal: {ir_signal}")
            return ir_signal

def handle_movement():
    global IR_RUNNING
    while IR_RUNNING:
        ir_signal=read_ir_signal()
        print(f"Handling movement for signal: {ir_signal}")
        if ir_signal == "KEY_UP":
            print("Moving Forward")
            robot.forward()
            time.sleep(0.5)
            robot.stop()
        elif ir_signal == "KEY_DOWN":
            print("Moving Backward")
            robot.backward()
            time.sleep(0.5)
            robot.stop()
        elif ir_signal == "KEY_RIGHT":
            print("Turning Right")
            robot.right()
            time.sleep(0.2)
            robot.stop()

        elif ir_signal == "KEY_LEFT":
            print("Turning Left")
            robot.left()
            time.sleep(0.2)
            robot.stop()
        elif ir_signal == "KEY_F1":
            IR_RUNNING=False
            robot.stop()
            
        
        
    time.sleep(0.2)

'''def LineTracking():
    flag = True
    robot.forward()
    
    while flag:
        time.sleep(0.04)
        left_status = GPIO.input(tracking_pinL)
        print(f"Left: {left_status}")
        
        if left_status == 1:
            robot.stop()
            flag = False

        time.sleep(0.1)
'''
def main():
    try:
        handle_movement()
        time.sleep(0.1)
        #robot.forward()
        #time.sleep(1)
        robot.stop()
        
        print("Ending program")
        
    except KeyboardInterrupt:
        print("Program interrupted")
    finally:
        robot.stop()
        GPIO.cleanup()  # Ensure cleanup happens on exit
        print("GPIO cleanup")

if __name__ == "__main__":
    main()
