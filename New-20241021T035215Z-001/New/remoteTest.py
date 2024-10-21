import subprocess

def listen_for_ir():
    # Start the irw process to listen for remote control input
    process = subprocess.Popen(['irw'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    print("Listening for remote control signals. Press any key on the remote.")

    try:
        while True:
            output = process.stdout.readline()
            if output:
                # Decode the byte output to string and split to get the button name and hex code
                decoded_output = output.decode('utf-8').strip()
                parts = decoded_output.split()
                
                if len(parts) >= 4:
                    button = parts[2] 
                    hex_code = parts[1] 
                    print(f"Button pressed: {button}, Hex code: {hex_code}")
            else:
                break
        process.terminate()
    except KeyboardInterrupt:
        print("\nExiting.")
        process.terminate()

if __name__ == "__main__":
    listen_for_ir()
