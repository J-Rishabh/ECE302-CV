# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

"""Control a 3-wheeled omnidirectional robot using a joystick controller."""
import time
import pygame
import board
from adafruit_motorkit import MotorKit

# Initialize Pygame
pygame.init()

# Initialize the joystick
pygame.joystick.init()
if pygame.joystick.get_count() == 0:
    print("No joystick detected!")
    pygame.quit()
    exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"Joystick Name: {joystick.get_name()}")

# Motor setup
kit = MotorKit(i2c=board.I2C())

# Scaling factor for joystick input (adjust if needed)
MAX_THROTTLE = 1.0

# Main loop
try:
    print("Use the joystick to control the car.")
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.JOYBUTTONDOWN:
                if event.button == 0:  # Example: Button 0 stops the motors
                    print("Emergency stop!")
                    kit.motor1.throttle = 0
                    kit.motor2.throttle = 0
                    kit.motor3.throttle = 0
                elif event.button == 1:  # Example: Button 1 exits the program
                    print("Exiting...")
                    running = False

        # Get joystick input (left stick)
        axis_x = joystick.get_axis(0)  # Horizontal axis
        axis_y = -joystick.get_axis(1)  # Vertical axis (invert to match forward)

        # Calculate motor throttles based on joystick input
        # Assuming motors are at 120° intervals:
        motor1_throttle = -0.866 * axis_y - 0.5 * axis_x  # Motor1: 240°
        motor2_throttle = axis_x                          # Motor2: 0°
        motor3_throttle = 0.866 * axis_y - 0.5 * axis_x   # Motor3: 120°

        # Scale throttles to MAX_THROTTLE
        motor1_throttle = max(-MAX_THROTTLE, min(MAX_THROTTLE, motor1_throttle))
        motor2_throttle = max(-MAX_THROTTLE, min(MAX_THROTTLE, motor2_throttle))
        motor3_throttle = max(-MAX_THROTTLE, min(MAX_THROTTLE, motor3_throttle))

        # Set motor throttles
        kit.motor1.throttle = motor1_throttle
        kit.motor2.throttle = motor2_throttle
        kit.motor3.throttle = motor3_throttle

        # Debugging: Print current motor values
        print(f"Motor1: {motor1_throttle:.2f}, Motor2: {motor2_throttle:.2f}, Motor3: {motor3_throttle:.2f}")

        # Small delay to prevent excessive CPU usage
        time.sleep(0.05)

except KeyboardInterrupt:
    print("Exiting...")

finally:
    # Ensure all motors are stopped
    kit.motor1.throttle = 0
    kit.motor2.throttle = 0
    kit.motor3.throttle = 0
    print("Program finished.")
    pygame.quit()
