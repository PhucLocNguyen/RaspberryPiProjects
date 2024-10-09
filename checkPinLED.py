from gpiozero import PWMLED
from time import sleep

# Define a PWM LED object on GPIO pin 18
pwm_led = PWMLED(18)

# Set PWM to 30% duty cycle
pwm_led.value = 0.3
print("PWM set to 30% duty cycle")

# Simulate reading back the value
# (Direct readback isn't possible on real GPIO; this is for simulation)
read_value = pwm_led.value * 100  # Convert to percentage
print(f"Read back value: {read_value:.1f}%")

sleep(2)

# Clean up
pwm_led.close()
