import socket
import RPi.GPIO as GPIO
from time import sleep
from gpiozero import AngularServo

# Pin configuration
in1 = 23
in2 = 24
en = 13  # Speed control pin for motor
myServoPin = 18
SERVO_DELAY_SEC = 0.001

# Set up servo (for steering - bánh trước)
servo = AngularServo(myServoPin, initial_angle=90, min_angle=0, max_angle=180)

temp1 = 1  # 1 = forward, 0 = backward

# Set up motor pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(in1, GPIO.OUT)
GPIO.setup(in2, GPIO.OUT)
GPIO.setup(en, GPIO.OUT)
p=GPIO.PWM(en,1000)

p.start(100)
# Set up the server
server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_socket.bind(('0.0.0.0', 5005))  # Bind to all interfaces on port 5005

print("Listening for commands...")

try:
    
    while True:
        data, addr = server_socket.recvfrom(1024)  # Buffer size is 1024 bytes
        command = data.decode('utf-8')
        print(f"Received command: {command}")

        # Here, you would translate the command into actions for your car
        # For example:
        
        if command == 'FORWARD':
            # Code to move the car forward
             GPIO.output(in1,GPIO.HIGH)
             GPIO.output(in2,GPIO.LOW)
             print("forward")
            
        elif command == 'BACKWARD':
            # Code to move the car backward
            print("backward")
            GPIO.output(in1,GPIO.LOW)
            GPIO.output(in2,GPIO.HIGH)
            
        elif command == 'LEFT':
            # Code to turn the car left
            print("turn left")
            servo.angle = 170  # Adjust the servo to turn left (angle can be adjusted based on your setup)
            sleep(SERVO_DELAY_SEC)
            
        elif command == 'RIGHT':
            # Code to turn the car right
            print("turn right")
            servo.angle = 10  # Adjust the servo to turn left (angle can be adjusted based on your setup)
            sleep(SERVO_DELAY_SEC)
            
        elif command == 'STOP':
            # Code to stop the car
            print("Stop")
            servo.angle=90
            sleep(SERVO_DELAY_SEC)
            GPIO.output(in1,GPIO.LOW)
            GPIO.output(in2,GPIO.LOW)
                    
except:
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.LOW)
    GPIO.cleanup()
    print("End")
    
    
