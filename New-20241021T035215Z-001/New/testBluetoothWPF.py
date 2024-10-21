import bluetooth                       #-------> import bluetooth module
import RPi.GPIO as GPIO
import time
from gpiozero import AngularServo, Robot
import subprocess
import threading
import socket

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Setup GPIO pins
myServoPin = 18
SERVO_DELAY_SEC = 0.001
servo = AngularServo(myServoPin, initial_angle=100, min_angle=0, max_angle=180)

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
GPIO.setup(ir_pin, GPIO.IN)
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
robot = Robot(right=(motorA_1A, motorA_1B), left=(motorB_1A, motorB_1B))

# Global variables
leftToRight = 0
servoAngle = 0
lastServoAngle = 0
OBSTACLE_DISTANCE = 30
OBSTACLE_DISTANCE_LOW = 25
speedOffset = 0
SONIC_TIMEOUT = 20

stoppingFlag = True
servo.angle = None
def bluetooth_listener():
    global stoppingFlag
    server_sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    server_sock.bind(("", bluetooth.PORT_ANY))
    server_sock.listen(1)
    client_sock, address = server_sock.accept()
    print(f"Bluetooth connection accepted from {address}")

    while stoppingFlag:
        data = client_sock.recv(1024).decode('UTF-8')
        print("Received from Bluetooth: ", data)
        
        if data.startswith("up"):
            print("Moving forward")
            robot.forward()
            time.sleep(1)
        elif data.startswith("left"):
            robot.left()
            time.sleep(0.5)
        elif data.startswith("right"):
            robot.right()
            time.sleep(0.5)
        elif data.startswith("down"):
            robot.backward()
            time.sleep(1)
        elif data.startswith("0"):
            robot.stop()
        else:
            print("No signal")
    
    client_sock.close()
    server_sock.close()

def socket_listener():
    global stoppingFlag
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(('0.0.0.0', 8080))  
    server_socket.listen(1)
    client_socket, address = server_socket.accept()
    print(f"Socket connection accepted from {address}")

    while stoppingFlag:
        data = client_socket.recv(1024).decode('UTF-8')
        if data == "stop":
            print("Stopping the robot")
            stoppingFlag = False
            robot.stop()

    client_socket.close()
    server_socket.close()


try:
    bt_thread = threading.Thread(target=bluetooth_listener)
    socket_thread = threading.Thread(target=socket_listener)

    bt_thread.start()
    socket_thread.start()  

    bt_thread.join()  
    socket_thread.join()
except KeyboardInterrupt:
    print("Program interrupted")
finally:
    GPIO.cleanup()  # Ensure cleanup happens on exit
    print("GPIO cleanup")
