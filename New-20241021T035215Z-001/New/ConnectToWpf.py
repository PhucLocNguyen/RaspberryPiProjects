import socket
import subprocess

HOST = '192.168.1.176'
PORT = 30802

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((HOST, PORT))
server_socket.listen(1)

print(f'Server listening on {PORT}...')

conn, addr = server_socket.accept()
print(f'Connected from {addr}')
try:
    while True:
        data = conn.recv(1024)
        if not data:
            break

        command = data.decode()
        print(f'Received command: {command}')

        if command.startswith("WRITE_FILE;"):

            content = command[len("WRITE_FILE;"):]
            print(content)

            with open('uploaded_code.py', 'w') as f:
                f.write(content)
            response = 'File written successfully.'
            
            try:
                result = subprocess.run(['python3', 'uploaded_code.py'], capture_output=True, text=True)
                response += f"\nCode ran successfully: {result.stdout}"
            except Exception as e:
                response += f"\nError executing code: {str(e)}"

            conn.sendall(response.encode())
        
        else:
            response = 'Unknown command'
            conn.sendall(response.encode())

    conn.close()

except KeyboardInterrupt:
    print("Program interrupted")
finally:
    print("GPIO cleanup")
