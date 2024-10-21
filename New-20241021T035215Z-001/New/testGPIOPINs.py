import RPi.GPIO as GPIO

# Sử dụng hệ thống đánh số vật lý (hoặc BCM tùy chọn)
GPIO.setmode(GPIO.BCM)

# Danh sách các chân GPIO bạn muốn kiểm tra
pins = [2, 3, 4, 17, 27, 22, 10, 9, 11, 5, 6, 13, 19, 26, 14, 15, 18, 23, 24, 25, 8, 7, 12, 16, 20, 21]

# Khởi tạo các chân như là INPUT và kiểm tra trạng thái
for pin in pins:
    GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_OFF)
    if GPIO.input(pin) == GPIO.LOW:
        print(f"Pin {pin} đang được sử dụng (LOW)")
    else:
        print(f"Pin {pin} đang được sử dụng (HIGH)")

# Đặt lại tất cả các chân GPIO về trạng thái ban đầu
GPIO.cleanup()
