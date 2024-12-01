# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

"""Control a 3-wheeled omnidirectional robot to move forward."""
import time
import board
from adafruit_motorkit import MotorKit

# Motor setup
kit = MotorKit(i2c=board.I2C())

try:
    # Move the car forward
    print("Car moving forward")
    kit.motor1.throttle = 0.0  # Motor 1 (reverse) #1
    kit.motor2.throttle = -1.0   # Motor 2 (forward) # 0
    kit.motor3.throttle = 1.0  # Motor 3 (reverse) # -1

    time.sleep(2)  # Move forward for 2 seconds

    # Stop the motors
    print("Stopping motors")
    kit.motor1.throttle = 0
    kit.motor2.throttle = 0
    kit.motor3.throttle = 0

except KeyboardInterrupt:
    print("Exiting...")

finally:
    # Ensure all motors are stopped
    kit.motor1.throttle = 0
    kit.motor2.throttle = 0
    kit.motor3.throttle = 0
    print("Program finished.")

