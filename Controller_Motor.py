# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

"""Control a 3-wheeled omnidirectional robot like a car in GTA."""
import time
import pygame
import board
from adafruit_motorkit import MotorKit
import math

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

# Scaling factors
MAX_THROTTLE = 1.0  # Full speed
TURN_SENSITIVITY = 1.0  # Adjust how strongly the car turns
DEAD_ZONE = 0.1  # Ignore small joystick inputs

spin_mode = False

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
                elif event.button == 6:  # Spin clockwise
                    print("Spinning clockwise!")
                    kit.motor1.throttle = MAX_THROTTLE
                    kit.motor2.throttle = MAX_THROTTLE#-MAX_THROTTLE / 4
                    kit.motor3.throttle = MAX_THROTTLE#MAX_THROTTLE / 4
                    spin_mode = True
                elif event.button == 5:  # Spin counterclockwise
                    print("Spinning counterclockwise!")
                    kit.motor1.throttle = -MAX_THROTTLE
                    kit.motor2.throttle = -MAX_THROTTLE#-MAX_THROTTLE / 4
                    kit.motor3.throttle = -MAX_THROTTLE#MAX_THROTTLE / 4
                    spin_mode = True

            elif event.type == pygame.JOYBUTTONUP:
                if event.button == 6 or event.button == 5:  # Stop spinning when the button is released
                    print("Stopping spin!")
                    kit.motor1.throttle = 0
                    kit.motor2.throttle = 0
                    kit.motor3.throttle = 0
                    spin_mode = False
        if spin_mode:
            continue

        # Get joystick input
        #throttle = MAX_THROTTLE if joystick.get_button(8) else 0  # RT (Button 8) for forward throttle
        throttle = MAX_THROTTLE if joystick.get_button(8) else (-MAX_THROTTLE if joystick.get_button(7) else 0)
        axis_x = -joystick.get_axis(0)  # Left stick X-axis for turning
        axis_y = joystick.get_axis(1)  # Left stick Y-axis for forward/backward

        # Apply dead zone to Left Stick inputs
        if abs(axis_x) < DEAD_ZONE:
            axis_x = 0
        if abs(axis_y) < DEAD_ZONE:
            axis_y = 0

        # Determine direction
        if axis_x == 0 and axis_y == 0:  # No Left Stick input
            if throttle > 0 or throttle < 0:  # RT is pressed, move forward
                # Explicitly set motor throttles for forward movement
                motor1_throttle = 0.0
                motor2_throttle = -throttle
                motor3_throttle = throttle
            else:
                motor1_throttle = 0
                motor2_throttle = 0
                motor3_throttle = 0
        else:
            # Use Left Stick input for direction
            direction_angle = math.atan2(axis_y, axis_x)

            # Scale throttle based on joystick input
            scaled_throttle = throttle

            # Calculate motor throttles for omnidirectional motion
            motor1_throttle = scaled_throttle * math.sin(direction_angle - (2 * math.pi / 3))  # Motor1: 240°
            motor2_throttle = scaled_throttle * math.sin(direction_angle)  # Motor2: 0°
            motor3_throttle = scaled_throttle * math.sin(direction_angle + (2 * math.pi / 3))  # Motor3: 120°

            # Clamp motor throttles
            motor1_throttle = max(-MAX_THROTTLE, min(MAX_THROTTLE, motor1_throttle))
            motor2_throttle = max(-MAX_THROTTLE, min(MAX_THROTTLE, motor2_throttle))
            motor3_throttle = max(-MAX_THROTTLE, min(MAX_THROTTLE, motor3_throttle))

        # Set motor throttles
        kit.motor1.throttle = motor1_throttle
        kit.motor2.throttle = motor2_throttle
        kit.motor3.throttle = motor3_throttle

        # Debugging: Print current motor values
        print(f"Throttle: {throttle:.2f}")
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
