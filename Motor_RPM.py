# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

"""Run a single motor at full throttle and calculate RPM using encoder signals"""
import time
import board
import RPi.GPIO as GPIO
from adafruit_motorkit import MotorKit

# Constants for Motor 1
ENCODER_A = 25  # GPIO pin for A-phase
ENCODER_B = 24  # GPIO pin for B-phase
CPR = 320  # Counts per revolution of the encoder (adjust if necessary)

# Initialize MotorKit
kit = MotorKit(i2c=board.I2C())

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(ENCODER_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(ENCODER_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Encoder tracking
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


def calculate_rpm(counts, duration):
    """Calculates RPM from encoder counts and duration."""
    revolutions = counts / CPR
    rpm = (revolutions / duration) * 60
    return rpm


try:
    # Start the motor at full throttle
    print("Running motor at full throttle...")
    kit.motor1.throttle = 1.0

    # Run for a specified duration
    duration = 5  # seconds
    start_time = time.time()
    while time.time() - start_time < duration:
        read_encoder()  # Update encoder count
        time.sleep(0.05)  # Small delay to prevent high CPU usage

    # Calculate and print RPM
    elapsed_time = time.time() - start_time
    rpm = calculate_rpm(encoder_count, elapsed_time)
    print(f"Encoder count: {encoder_count}")
    print(f"RPM: {rpm:.2f}")

    # Stop the motor
    print("Stopping motor...")
    kit.motor1.throttle = 0

except KeyboardInterrupt:
    print("Exiting...")

finally:
    # Cleanup GPIO
    GPIO.cleanup()
    print(f"Final encoder count: {encoder_count}")
