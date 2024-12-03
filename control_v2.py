import cv2
import cv2.aruco as aruco
import numpy as np
import time
import pygame
import board
from adafruit_motorkit import MotorKit
import math

# ArUco marker setup 
ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
PARAMS = cv2.aruco.DetectorParameters_create()
CAMERA_MATRIX = np.load("camera_matrix2.npy")
DIST_COEFFS = np.load("dist_coeffs2.npy")
MARKER_SIZE = 4.35/100#18.796 / 100 #4.35 / 100  # in meters

# Motor setup
kit = MotorKit(i2c=board.I2C())

# Controller setup
pygame.init()
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

# Constants
MAX_THROTTLE = 1.0
DEAD_ZONE = 0.1
TARGET_DISTANCE = 1.0  # Target distance in meters
KP, KI, KD = 0.5, 0.1, 0.05  # PID constants

# PID variables
prev_error = 0
integral = 0

spin_mode = False

def pid_control(current_distance):
    global prev_error, integral
    error = TARGET_DISTANCE - current_distance
    integral += error
    derivative = error - prev_error
    prev_error = error
    #return KP * error + KI * integral + KD * derivative
    return KP * error + KI * integral

def autonomous_mode(cap):
    global prev_error, integral
    ret, frame = cap.read()
    if not ret:
        return

    frame = cv2.rotate(frame, cv2.ROTATE_180)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = cv2.aruco.detectMarkers(gray, ARUCO_DICT, parameters=PARAMS)

    if ids is not None:
        distances = []  # List to store distances to all detected markers
        marker_positions = []

        for corner, marker_id in zip(corners, ids.flatten()):
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corner, MARKER_SIZE, CAMERA_MATRIX, DIST_COEFFS)
            distance = np.linalg.norm(tvec[0][0])  # Compute distance to the marker
            distances.append(distance)
            marker_center = int(np.mean(corner[0][:, 0]))  # Horizontal center of the marker
            marker_positions.append(marker_center)

        # Calculate the average distance
        if distances:
            average_distance = sum(distances) / len(distances)
            print(f"Average Distance: {average_distance:.2f} meters")

            # Calculate the average marker position
            frame_center = frame.shape[1] // 2
            average_marker_position = sum(marker_positions) / len(marker_positions)
            offset = (average_marker_position - frame_center) / frame_center  # Normalized offset (-1 to 1)
            print(f"Marker Offset: {offset:.2f}")

            # PID control for distance
            control_signal = pid_control(average_distance)
            control_signal = max(-1, min(1, control_signal))  # Clamp to [-1, 1]

            # Determine if turning is needed
            if abs(offset) < 0.1:  # Markers are relatively centered
                print("Markers centered. Moving forward/backward only.")
                turn_throttle = 0
            else:  # Markers are off-center
                print("Markers off-center. Adjusting heading.")
                turn_throttle = offset * 0.5  # Scale turning signal

            # Forward/backward movement
            if abs(average_distance - TARGET_DISTANCE) <= 0.1:  # Tolerance for stopping
                print("Target distance reached. Stopping the car.")
                stop_motors()
                return
            elif average_distance > TARGET_DISTANCE:  # Too far
                forward_throttle = control_signal
            elif average_distance < TARGET_DISTANCE:  # Too close
                forward_throttle = -control_signal

            # Combine forward/backward and turning control
            motor1_throttle = turn_throttle  # Motor 1 for turning
            motor2_throttle = forward_throttle - turn_throttle
            motor3_throttle = -forward_throttle - turn_throttle

            # Clamp motor throttles to [-1, 1]
            motor1_throttle = max(-1, min(1, motor1_throttle))
            motor2_throttle = max(-1, min(1, motor2_throttle))
            motor3_throttle = max(-1, min(1, motor3_throttle))

            # Set motor throttles
            kit.motor1.throttle = motor1_throttle
            kit.motor2.throttle = motor2_throttle
            kit.motor3.throttle = motor3_throttle

            print(f"Motor Throttles -> Motor1: {motor1_throttle:.2f}, Motor2: {motor2_throttle:.2f}, Motor3: {motor3_throttle:.2f}")
    else:
         # Rotate until marker is found
        print("No markers detected. Rotating to find markers.")
        set_motor_rotation("clockwise")



def are_markers_centered(frame, corners):
    """Check if all markers are centered in the frame."""
    if corners is None:
        return False
    frame_center = frame.shape[1] // 2
    marker_positions = [int(np.mean(corner[0][:, 0])) for corner in corners]
    return all(frame_center * 0.4 <= x <= frame_center * 1.6 for x in marker_positions)


def set_motor_rotation(direction):
    """Set motor rotation direction."""
    if direction == "clockwise":
        kit.motor1.throttle = 0.3
        kit.motor2.throttle = 0.3
        kit.motor3.throttle = 0.3
    elif direction == "counterclockwise":
        kit.motor1.throttle = -0.3
        kit.motor2.throttle = -0.3
        kit.motor3.throttle = -0.3


def stop_motors():
    """Stop all motors."""
    kit.motor1.throttle = 0
    kit.motor2.throttle = 0
    kit.motor3.throttle = 0




def manual_mode():
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

def main():
    global spin_mode
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 120)

    mode = "manual"

    try:
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    raise KeyboardInterrupt
                elif event.type == pygame.JOYBUTTONDOWN:
                    if event.button == 3:  # Switch mode
                        #cap.release()
                        mode = "autonomous" if mode == "manual" else "manual"
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
            if mode == "manual":
                print("Manual Starting")
                manual_mode()
            else:
                #cap.rel
                autonomous_mode(cap)
                #cap.release()

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("Exiting...")

    finally:
        cap.release()
        cv2.destroyAllWindows()
        kit.motor1.throttle = 0
        kit.motor2.throttle = 0
        kit.motor3.throttle = 0
        pygame.quit()

if __name__ == "__main__":
    main()