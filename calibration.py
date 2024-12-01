import cv2
import numpy as np
import os

# Define the ArUco dictionary and board configuration
ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
BOARD_ROWS = 5  # Number of markers in the vertical direction
BOARD_COLS = 4  # Number of markers in the horizontal direction
MARKER_LENGTH = 4.35/100  # Marker side length in meters (e.g., 3 cm)
MARKER_SEPARATION = 0.45/100  # Marker separation in meters (e.g., 1 cm)

# Create the ArUco board
board = cv2.aruco.GridBoard(
    size=(BOARD_COLS, BOARD_ROWS),  # Tuple (columns, rows)
    markerLength=MARKER_LENGTH,    # Marker side length in meters
    markerSeparation=MARKER_SEPARATION,  # Separation in meters
    dictionary=ARUCO_DICT          # ArUco dictionary
)

# Initialize variables for calibration
all_corners_concatenated = []  # Flattened list of all marker corners
all_ids_concatenated = []      # Flattened list of all marker IDs
marker_counter_per_frame = []  # Number of markers detected per frame
image_size = None

# Load the calibration images
CALIBRATION_IMAGES = "calibration_images"  # Folder containing your calibration images
image_files = [os.path.join(CALIBRATION_IMAGES, f) for f in os.listdir(CALIBRATION_IMAGES) if f.endswith('.jpg')]

# Process each calibration image
for image_file in image_files:
    image = cv2.imread(image_file)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Detect ArUco markers in the image
    corners, ids, _ = cv2.aruco.detectMarkers(gray, ARUCO_DICT)

    if ids is not None:
        # Append detected corners and IDs for this frame
        all_corners_concatenated.extend(corners)
        all_ids_concatenated.extend(ids.flatten())  # Flatten IDs into a single list
        marker_counter_per_frame.append(len(ids))  # Count the markers in this frame

        # Draw detected markers for visualization (optional)
        cv2.aruco.drawDetectedMarkers(image, corners, ids)
        cv2.imshow("Detected Markers", image)
        cv2.waitKey(500)

        # Update the image size (once)
        if image_size is None:
            image_size = gray.shape[::-1]

cv2.destroyAllWindows()

# Check that markers were detected
if len(all_corners_concatenated) == 0 or len(all_ids_concatenated) == 0:
    print("Error: No markers detected. Check your calibration images or board setup.")
    exit()

# Perform camera calibration using the detected markers
camera_matrix = np.zeros((3, 3))  # Initialize the camera matrix
dist_coeffs = np.zeros((5,))      # Initialize distortion coefficients
calibration_flags = 0             # Set calibration flags (e.g., cv2.CALIB_ZERO_TANGENT_DIST)

# Convert IDs and marker counters to NumPy arrays
all_ids_concatenated = np.array(all_ids_concatenated, dtype=np.int32)
marker_counter_per_frame = np.array(marker_counter_per_frame, dtype=np.int32)

# Perform calibration
ret_error = cv2.aruco.calibrateCameraAruco(
    all_corners_concatenated,     # Flattened corners
    all_ids_concatenated,         # Flattened IDs
    marker_counter_per_frame,     # Number of markers per frame
    board,                        # ArUco board
    image_size,                   # Image dimensions (width, height)
    camera_matrix,                # Output: Camera matrix
    dist_coeffs,                  # Output: Distortion coefficients
    None,                         # Output: Rotation vectors
    None,                         # Output: Translation vectors
    calibration_flags             # Calibration flags
)



# Output calibration results
print("Reprojection Error:", ret_error)
print("\nCamera matrix:")
print(camera_matrix)
print("\nDistortion coefficients:")
print(dist_coeffs)

# Save calibration results for future use
np.save("camera_matrix3.npy", camera_matrix)
np.save("dist_coeffs3.npy", dist_coeffs)

output_path = "processed_images"
os.makedirs(output_path, exist_ok=True)
for i, image_file in enumerate(image_files):
    image = cv2.imread(image_file)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = cv2.aruco.detectMarkers(gray, ARUCO_DICT)

    if ids is not None:
        cv2.aruco.drawDetectedMarkers(image, corners, ids)
        cv2.imwrite(f"{output_path}/processed_{i}.jpg", image)
