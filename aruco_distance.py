import cv2
import cv2.aruco as aruco
import numpy as np

# Define ArUco marker dictionary and parameters
ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
PARAMS = cv2.aruco.DetectorParameters_create()

# Load calibration results from npy files
CAMERA_MATRIX = np.load("camera_matrix2.npy")
DIST_COEFFS = np.load("dist_coeffs2.npy")


# Define the size of the ArUco marker in meters
MARKER_SIZE = 4.35/100 # Marker size in meters (e.g., 0.1 meters or 10 cm)

def main():
    # Initialize the camera
    # add param  cv2.CAP_V4L2 and change to 0q
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    # Set camera settings

    # add rotation code 

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 800)
    cap.set(cv2.CAP_PROP_FPS, 120)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Could not read frame.")
            break

        # Rotate the image 180 degrees clockwise
        frame = cv2.rotate(frame, cv2.ROTATE_180)


        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Detect ArUco markers
        corners, ids, _ = cv2.aruco.detectMarkers(gray, ARUCO_DICT, parameters=PARAMS)

        if ids is not None:
            # Draw detected markers
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            # Pose estimation for each marker
            for corner, marker_id in zip(corners, ids.flatten()):
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corner, MARKER_SIZE, CAMERA_MATRIX, DIST_COEFFS)

                # Calculate distance
                distance = np.linalg.norm(tvec[0][0])
                print(f"Marker ID: {marker_id}, Distance: {distance:.2f} meters")

                # Use custom function to draw axes
                # draw_axes(frame, CAMERA_MATRIX, DIST_COEFFS, rvec[0], tvec[0], MARKER_SIZE)

                # Display distance on the frame
                cv2.putText(frame, f"ID: {marker_id} Dist: {distance:.2f}m",
                            (int(corner[0][0][0]), int(corner[0][0][1]) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Show the frame
        cv2.imshow("ArUco Marker Detection", frame)

        # Exit on 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
