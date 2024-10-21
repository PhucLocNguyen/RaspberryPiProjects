import socket
import time

UDP_IP = "192.168.1.58"  # Replace with the actual IP of your Unity machine
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

while True:
    command = "FORWARD 0.5"
    sock.sendto(command.encode(), (UDP_IP, UDP_PORT))
    print(f"Sent: {command}")  # Log the sent command
    time.sleep(1)  # Wait for a second before sending again