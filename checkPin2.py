import RPi.GPIO as GPIO
import time
import threading

# Thiết lập GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Cài đặt chân 12 là đầu ra
GPIO.setup(12, GPIO.OUT)

# Biến để điều khiển trạng thái LED
led_state = False

def toggle_led():
    global led_state
    while True:
        led_state = not led_state  # Đảo trạng thái
        GPIO.output(12, led_state)  # Cập nhật chân 12
        time.sleep(2)  # Thay đổi trạng thái mỗi 2 giây

def read_led_state():
    while True:
        if GPIO.input(12) == GPIO.HIGH:
            print("LED đang bật")
        else:
            print("LED đang tắt")
        time.sleep(1)

# Tạo và khởi động các luồng
toggle_thread = threading.Thread(target=toggle_led)
read_thread = threading.Thread(target=read_led_state)

toggle_thread.start()
read_thread.start()

try:
    while True:
        time.sleep(1)  # Giữ chương trình chạy

except KeyboardInterrupt:
    print("Dừng chương trình")

finally:
    GPIO.cleanup()
