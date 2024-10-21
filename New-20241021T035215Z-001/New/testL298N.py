import RPi.GPIO as GPIO
import time
from gpiozero import AngularServo
import bluetooth

# Set the GPIO mode
GPIO.setmode(GPIO.BCM)

# Define the GPIO pins for the motor control
IN1 = 23  # Pin connected to IN1 on L298N
IN2 = 24  # Pin connected to IN2 on L298N
ENA = 13  # Pin connected to ENA on L298N (PWM for speed control)

myServoPin = 18
SERVO_DELAY_SEC = 0.001

# Set up servo (for steering - bánh trước)
servo = AngularServo(myServoPin, initial_angle=90, min_angle=0, max_angle=180)

# Set up the GPIO pins as output
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)

# Set up PWM on the enable pin for speed control
pwm = GPIO.PWM(ENA, 1000)  # Set PWM frequency to 1kHz
pwm.start(55)  # Start PWM with 0% duty cycle (motor stopped)


server_socket=bluetooth.BluetoothSocket(bluetooth.RFCOMM)              #-----> declaring our bluetooth server socket
port = 1                                                               #-----> a variable to store value of port
server_socket.bind(("",port))                                          #----> bindind port to our sever socket
server_socket.listen(1)                                                #------>make our bluetooth sever to listen for 1 connection at a time
client_socket,address = server_socket.accept()                         #----> accept connection from client and get the address
print ("Accepted connection from ",address)                            #------> print the bluetooth address of the connected client or the device 

# Function to rotate the motor in one direction with adjustable speed
def motor_backward(speed):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    pwm.ChangeDutyCycle(speed)  # Set speed (0 to 100%)

# Function to rotate the motor in the opposite direction with adjustable speed
def motor_forward(speed):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    pwm.ChangeDutyCycle(speed)  # Set speed (0 to 100%)

# Function to stop the motor
def motor_stop():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    pwm.ChangeDutyCycle(0)  # Set duty cycle to 0 (motor stopped)

# Main loop to test motor speed control
try:
    stoppingFlag = True
    while stoppingFlag:  # ------> run the below functions in loop
        data = client_socket.recv(1024)  # -----> declaring variable "data" as the data received from the client
        data = data.decode('UTF-8')  # -----> the data recieved will be in the form of byes
        print("Received: ", data)  # so we will convert it into strings

        if (data.startswith("up")):
            print("Moving forward")
            motor_forward(100)
            time.sleep(0.5)
            motor_stop()
        elif (data.startswith("left")):  # ----> if the data is L
            servo.angle = 150  # Adjust the servo to turn left (angle can be adjusted based on your setup)
            time.sleep(SERVO_DELAY_SEC)
            motor_forward(100)
            time.sleep(0.2)
            motor_stop()
            servo.angle = 90
            time.sleep(SERVO_DELAY_SEC)
        elif (data.startswith("right")):  # ----> if the data is R
            servo.angle = 30  # Adjust the servo to turn left (angle can be adjusted based on your setup)
            time.sleep(SERVO_DELAY_SEC)
            motor_forward(100)
            time.sleep(0.2)
            motor_stop()
            servo.angle = 90
            time.sleep(SERVO_DELAY_SEC)
        elif (data.startswith("down")):  # ----> if the data is B
            motor_backward(100)
            time.sleep(0.5)
            motor_stop()
        elif (data.startswith("0")):  # ----> if the data is s
            motor_stop()  # then function stop
        elif (data == "stop"):
            stoppingFlag = False
        else:
            print("No signal")
        motor_stop()

except KeyboardInterrupt:
    pwm.stop()
    GPIO.cleanup()
    pass  # Exit the loop on a keyboard interrupt

# Cleanup GPIO after the program ends
pwm.stop()
GPIO.cleanup()
