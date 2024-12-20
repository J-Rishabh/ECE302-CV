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
MARKER_SIZE = 18.796 / 100 # in meters

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

spin_mode = False # boolean used for rotating logic when 'RB' or 'LB' buttons are pressed 

def pid_control(current_distance):
    global prev_error, integral
    error = TARGET_DISTANCE - current_distance
    integral += error
    derivative = error - prev_error
    prev_error = error
    return KP * error + KI * integral + KD * derivative

def autonomous_mode(cap):
    global prev_error, integral
    SAFE_DISTANCE = 0.7  # Minimum distance in meters for the car to start back up
    ret, frame = cap.read()
    if not ret:
        return

    frame = cv2.rotate(frame, cv2.ROTATE_180)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = cv2.aruco.detectMarkers(gray, ARUCO_DICT, parameters=PARAMS)

    while ids is None or not are_markers_centered(frame, corners):  # Rotate until markers are centered
        if ids is not None:
            marker_positions = [int(np.mean(corner[0][:, 0])) for corner in corners]
            frame_center = frame.shape[1] // 2  # Horizontal center of the frame

            # Check if markers are to the left or right
            if any(x < frame_center * 0.8 for x in marker_positions):  # Markers too far left
                print("Markers moving out of frame to the left. Rotating counterclockwise.")
                set_motor_rotation("counterclockwise")
            elif any(x > frame_center * 1.2 for x in marker_positions):  # Markers too far right
                print("Markers moving out of frame to the right. Rotating clockwise.")
                set_motor_rotation("clockwise")
        else:
            print("No markers detected. Rotating to find markers.")
            set_motor_rotation("clockwise") # Rotates until finds marker

        # Capture new frame and update detected markers
        ret, frame = cap.read()
        if not ret:
            return
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, ARUCO_DICT, parameters=PARAMS)

    stop_motors() # Stop motors once the ArUco marker is centers in camera's frame

    
        
    distances = []  # List to store distances to all detected markers. In our case we only have one marker but this code will work for x number of markers.
    for corner, marker_id in zip(corners, ids.flatten()):
        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corner, MARKER_SIZE, CAMERA_MATRIX, DIST_COEFFS) # Get pose estimation for ArUco marker (rotation and translation vector)
        distance = np.linalg.norm(tvec[0][0])  # Compute distance to the marker 
        distances.append(distance) # Add distance to distances array
        print(f"Marker ID: {marker_id}, Distance: {distance:.2f} meters")

    if distances:
        average_distance = sum(distances) / len(distances) # Calculate average distance to markers
        print(f"Average Distance: {average_distance:.2f} meters")

        # Backup if markers too close
        if average_distance <= SAFE_DISTANCE:
            print("Markers are too close! Backing up.")
            kit.motor1.throttle = 0
            kit.motor2.throttle = 1  
            kit.motor3.throttle = -1  
            return

        # Stop once target distance roughly reached
        if average_distance <= TARGET_DISTANCE:
            print("Target distance reached. Stopping the car.")
            stop_motors()
            return

        # Logic to determine control_signal for when ArUco markers are further than 1 m target distance
        control_signal = pid_control(average_distance)
        control_signal = max(-1, min(1, control_signal))

        motor1_throttle = 0
        motor2_throttle = control_signal
        motor3_throttle = -control_signal

        motor2_throttle = max(-1, min(1, motor2_throttle))
        motor3_throttle = max(-1, min(1, motor3_throttle))

        kit.motor1.throttle = motor1_throttle
        kit.motor2.throttle = motor2_throttle
        kit.motor3.throttle = motor3_throttle

        print(f"Autonomous Control -> Motor1: {motor1_throttle}, Motor2: {motor2_throttle}, Motor3: {motor3_throttle}")


def are_markers_centered(frame, corners):
    # Check if all markers are centered in the frame
    if corners is None:
        return False
    frame_center = frame.shape[1] // 2
    marker_positions = [int(np.mean(corner[0][:, 0])) for corner in corners]
    return all(frame_center * 0.8 <= x <= frame_center * 1.2 for x in marker_positions)


def set_motor_rotation(direction):
    # Set motor rotation direction
    # Note that the discrepancy in magnitudes (0.5 vs 0.3) has to do with a 'weaker' motor 1 for the same throttle value applied
    # Don't set throttle values to max magnitude (1) or the car will spin too fast and miss the ArUco marker
    if direction == "clockwise":
        kit.motor1.throttle = 0.5
        kit.motor2.throttle = 0.3
        kit.motor3.throttle = 0.3
    elif direction == "counterclockwise":
        kit.motor1.throttle = -0.5
        kit.motor2.throttle = -0.3
        kit.motor3.throttle = -0.3


def stop_motors():
    # Stop all motors
    kit.motor1.throttle = 0
    kit.motor2.throttle = 0
    kit.motor3.throttle = 0




def manual_mode():
    # Get joystick input
    # Go forward if 'RT' button is pressed. Go backwards if 'LT' button is pressed. Else, do nothing.
    throttle = MAX_THROTTLE if joystick.get_button(8) else (-MAX_THROTTLE if joystick.get_button(7) else 0)
    axis_x = -joystick.get_axis(0)  # Left stick X-axis
    axis_y = joystick.get_axis(1)  # Left stick Y-axis

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

        # Calculate motor throttles for omnidirectional motion
        motor1_throttle = throttle * math.sin(direction_angle - (2 * math.pi / 3)) 
        motor2_throttle = throttle * math.sin(direction_angle)  
        motor3_throttle = throttle * math.sin(direction_angle + (2 * math.pi / 3)) 

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

    # Set video format to MJPG (Arducam provides MJPG by default)
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

    # Set up low resolution for computational efficiency
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # Set fps to 120
    cap.set(cv2.CAP_PROP_FPS, 120)

    # Start off code in 'manual' mode, giving user manual control
    mode = "manual"

    try:
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    raise KeyboardInterrupt
                
                # If a button is pressed
                elif event.type == pygame.JOYBUTTONDOWN:
                    if event.button == 3:  # Button 'Y' - Switch mode
                        mode = "autonomous" if mode == "manual" else "manual"
                    elif event.button == 6:  # Button 'RB' - Spin clockwise
                        print("Spinning clockwise!")
                        kit.motor1.throttle = MAX_THROTTLE
                        kit.motor2.throttle = MAX_THROTTLE
                        kit.motor3.throttle = MAX_THROTTLE
                        spin_mode = True
                    elif event.button == 5:  # Button 'LB' - Spin counterclockwise
                        print("Spinning counterclockwise!")
                        kit.motor1.throttle = -MAX_THROTTLE
                        kit.motor2.throttle = -MAX_THROTTLE
                        kit.motor3.throttle = -MAX_THROTTLE
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

            # Transition of modes depending on if Button 'Y' pressed
            if mode == "manual":
                print("Manual Starting")
                manual_mode()
            else:
                autonomous_mode(cap)

    except KeyboardInterrupt:
        print("Exiting...")

    finally:
        # Release cap when done, destroy all windows, set all throttle values to '0', and quit 'pygame'
        cap.release()
        cv2.destroyAllWindows()
        kit.motor1.throttle = 0
        kit.motor2.throttle = 0
        kit.motor3.throttle = 0
        pygame.quit()

if __name__ == "__main__":
    main()