import socket
import time
import RPi.GPIO as GPIO
from gpiozero import AngularServo, Robot
import subprocess
import threading 
from time import sleep 
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
ir_pin = 16  # IR Receiver

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
GPIO.setup(ir_pin, GPIO.IN)  
p = GPIO.PWM(en, 1000)
p.start(50)  # Default speed 25%
# Set up the UDP client
UDP_IP = "192.168.1.58"  # Replace this with the IP address of the machine running Unity
UDP_PORT = 5005  # Port should match Unity's listening port

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def Move_Forward():
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in2, GPIO.LOW)
def Move_Backward():
    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.HIGH)
def Turn_Left():
    servo.angle = 10  # Adjust the servo to turn left (angle can be adjusted based on your setup)
    sleep(SERVO_DELAY_SEC)
def Turn_Right():
    servo.angle = 170  # Adjust the servo to turn left (angle can be adjusted based on your setup)
    sleep(SERVO_DELAY_SEC)
def Brake_Stop():
    servo.angle=90
    sleep(SERVO_DELAY_SEC)
    GPIO.output(in1,GPIO.LOW)
    GPIO.output(in2,GPIO.LOW)
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
            
# Function to send a message
def send_command(command):
    sock.sendto(command.encode(), (UDP_IP, UDP_PORT))
    print(f"Sent: {command}")


# Example of sending commands periodically or based on input
try:
    while True:
        # Example command sequence (manual control can be added here)
        #ir_signal=read_ir_signal()
        ir_signal="empty"
        x = input()
        print(f"Handling movement for signal: {ir_signal}")
        if ir_signal == "KEY_UP" or x=="f" :
            print("Moving Forward")
            send_command("FORWARD 1.0")
            Move_Forward()
            time.sleep(0.5)
            
        elif ir_signal == "KEY_DOWN" or x=="b":
            print("Moving Backward")
            send_command("BACKWARD 0.5")
            Move_Backward()
            time.sleep(0.5)
            
        elif ir_signal == "KEY_RIGHT" or x=="r":
            print("Turning Right")
            send_command("RIGHT")
            Turn_Right()
            time.sleep(0.2)

        elif ir_signal == "KEY_LEFT" or x=="l":
            print("Turning Left")
            send_command("LEFT")
            Turn_Left()
            time.sleep(0.2)
        elif ir_signal == "KEY_F1"or x=="s":
            send_command("STOP")
            IR_RUNNING=False
            Brake_Stop()
            

except KeyboardInterrupt:
    print("Control interrupted, exiting...")
    GPIO.cleanup()
    sock.close()
