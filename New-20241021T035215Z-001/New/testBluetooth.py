import bluetooth                       #-------> import bluetooth module
import RPi.GPIO as GPIO
import RPi.GPIO as GPIO
import time
from gpiozero import AngularServo, Robot
import subprocess
import threading

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Setup GPIO pins
#myServoPin = 18
#SERVO_DELAY_SEC = 0.001
#servo = AngularServo(myServoPin, initial_angle=100, min_angle=0, max_angle=180)

# Set pin for Ultrasonic sensor
trigPin = 5
echoPin = 24
buzzer_pin = 17

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
tracking_pinL = 23
ir_pin = 16  # IR Receiver

# Setup GPIO pins
#GPIO.setup(ir_pin, GPIO.IN)
GPIO.setup(trigPin, GPIO.OUT)
GPIO.setup(echoPin, GPIO.IN)
GPIO.setup(tracking_pinL, GPIO.IN)

# Setup Motor pins as output
GPIO.setup(buzzer_pin, GPIO.OUT)
pwm = GPIO.PWM(buzzer_pin, 1000)  # 1000Hz frequency
pwm.start(0)  # Start PWM with 0% duty cycle (off)

# Setup RGB pins as output
GPIO.setup(redPin, GPIO.OUT)
GPIO.setup(greenPin, GPIO.OUT)
GPIO.setup(bluePin, GPIO.OUT)

# Initialize robot
#robot = Robot(right=(motorA_1A, motorA_1B), left=(motorB_1A, motorB_1B))

robot = Robot(right=(11, 10), left=(13, 12))



# Global variables
leftToRight = 0
servoAngle = 0
lastServoAngle = 0
OBSTACLE_DISTANCE = 30
OBSTACLE_DISTANCE_LOW = 25
speedOffset = 0
SONIC_TIMEOUT = 20

server_socket=bluetooth.BluetoothSocket(bluetooth.RFCOMM)              #-----> declaring our bluetooth server socket
port = 1                                                               #-----> a variable to store value of port
server_socket.bind(("",port))                                          #----> bindind port to our sever socket
server_socket.listen(1)                                                #------>make our bluetooth sever to listen for 1 connection at a time
client_socket,address = server_socket.accept()                         #----> accept connection from client and get the address
print ("Accepted connection from ",address)                            #------> print the bluetooth address of the connected client or the device 

data=""
def blink(pin):
    GPIO.output(pin, GPIO.HIGH)

def turnOff(pin):
    GPIO.output(pin, GPIO.LOW)

def clearColor():
    turnOff(redPin)
    turnOff(bluePin)
    turnOff(greenPin)
try:
    stoppingFlag = True
    while stoppingFlag:  # ------> run the below functions in loop
        data = client_socket.recv(1024)  # -----> declaring variable "data" as the data received from the client
        data = data.decode('UTF-8')  # -----> the data recieved will be in the form of byes
        print("Received: ", data)  # so we will convert it into strings

        if (data.startswith("up")):
            print("Moving forward")
            robot.forward()
            time.sleep(0.5)
        elif (data.startswith("left")):  # ----> if the data is L
            robot.left()  # then function left
            time.sleep(0.1)
        elif (data.startswith("right")):  # ----> if the data is R
            robot.right()  # then function right
            time.sleep(0.1)
        elif (data.startswith("down")):  # ----> if the data is B
            robot.backward()  # then function back
            time.sleep(0.5)
        elif (data.startswith("0")):  # ----> if the data is s
            robot.stop()  # then function stop
        elif (data == "R" or data == "r"):
            clearColor()
            blink(redPin)
        elif (data == "G" or data == "g"):
            clearColor()
            blink(greenPin)
        elif (data == "B" or data == "b"):
            clearColor()
            blink(bluePin)
        elif (data == "clear"):
            clearColor()
        elif (data == "Bluzzer"):
            pwm.ChangeDutyCycle(80)
        elif (data == "bluzzer"):
            pwm.ChangeDutyCycle(0)    
        elif (data == "stop"):
            stoppingFlag = False
        else:
            print("No signal")
        robot.stop()
except KeyboardInterrupt:
    print("Program interrupted")
finally:
    GPIO.cleanup()  # Ensure cleanup happens on exit
    print("GPIO cleanup")
