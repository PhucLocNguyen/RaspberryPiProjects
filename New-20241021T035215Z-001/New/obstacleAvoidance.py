import RPi.GPIO as GPIO
import time
from gpiozero import AngularServo, DistanceSensor, Robot

# Setup GPIO pins
myServoPin = 18
SERVO_DELAY_SEC = 0.001
servo = AngularServo(myServoPin, initial_angle=90, min_angle=0,max_angle=180)

# Set pin for Ultrasonic sensor
trigPin = 26
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

sensor = DistanceSensor(echo=echoPin, trigger=trigPin, max_distance=3)

robot = Robot(right=(12, 13), left=(11, 10))

GPIO.setmode(GPIO.BCM)  # Use BCM numbering for all pins
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
OBSTACLE_DISTANCE = 25
OBSTACLE_DISTANCE_LOW = 20
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
    scanAngle = [150, 90, 30]

    # Initialize variables
    distance = [0] * 3
    # Main logic
    for i in range(3):
        servo.angle = scanAngle[i]
        time.sleep(1)

        distance[i] = GetSonar()
        print(f"Distance initial : {distance[i]}")
        #time.sleep(1)
        #distance[i] = GetSonar()
        #print(f"Distance after 1s: {distance[i]}")

    print(f"distance[0]: { distance[0]}, {scanAngle[0]}")
    print(f"distance[1]: { distance[1]}, {scanAngle[1]}")
    print(f"distance[2]: { distance[2]}, {scanAngle[2]}")
    
    servo.angle = 90
    time.sleep(1)
    servo.angle = None
    time.sleep(1)
    
    return distance

def ObstacleAvoidance():
    global OBSTACLE_DISTANCE, OBSTACLE_DISTANCE_LOW
    servo.angle = 90
    time.sleep(1)
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
            print("Khong co vat can")
            robot.forward()
            time.sleep(0.2)
def main():
    try:
#          scanAngle = [180,90,30]
#          for i in range(3):
#              print("Quay")
#              time.sleep(1)
#              servo.angle = scanAngle[i]
#              time.sleep(0.5)
#          servo.angle = 90
#          servo.detach()
        #distances = Scan()
        #time.sleep(1)
        #distances = Scan()
        ObstacleAvoidance()
        #robot.forward()
        #time.sleep(1)
        robot.stop()
        
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()
        robot.stop()# Ensure cleanup happens on exit

if __name__ == "__main__":
    main()
