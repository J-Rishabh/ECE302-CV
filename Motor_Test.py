# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

"""Simple test for using adafruit_motorkit with a DC motor and reading encoder signals"""
import time
import board
import RPi.GPIO as GPIO
from adafruit_motorkit import MotorKit

# Motor setup
kit = MotorKit(i2c=board.I2C())

# GPIO setup for encoder
ENCODER_A = 16  # GPIO pin for A-phase
ENCODER_B = 12  # GPIO pin for B-phase

GPIO.setmode(GPIO.BCM)
GPIO.setup(ENCODER_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(ENCODER_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Global variables to track encoder state
encoder_count = 0
last_a = 0
last_b = 0

def read_encoder():
    """Reads the encoder signals and updates the count."""
    global encoder_count, last_a, last_b

    # Read current states
    current_a = GPIO.input(ENCODER_A)
    current_b = GPIO.input(ENCODER_B)

    # Detect edges on A-phase
    if current_a != last_a:  # Rising or falling edge on A
        if current_a == GPIO.HIGH:
            if current_b == GPIO.LOW:
                encoder_count += 1  # Forward
            else:
                encoder_count -= 1  # Reverse
        else:
            if current_b == GPIO.HIGH:
                encoder_count += 1  # Forward
            else:
                encoder_count -= 1  # Reverse

    # Update last states
    last_a = current_a
    last_b = current_b

try:
    # Start motor
    print("Motor running forward")
    kit.motor1.throttle = 1.0  # Full speed forward

    start_time = time.time()
    while time.time() - start_time < 2:  # Run for 2 seconds
        read_encoder()  # Continuously read encoder signals
        print(f"Encoder count: {encoder_count}")
        time.sleep(0.05)  # Small delay to avoid CPU overload

    # Stop motor
    print("Stopping motor")
    kit.motor1.throttle = 0

except KeyboardInterrupt:
    print("Exiting...")

finally:
    # Cleanup GPIO
    GPIO.cleanup()
    print(f"Final encoder count: {encoder_count}")
