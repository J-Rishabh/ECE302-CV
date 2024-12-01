import cv2
import os

# Folder to save captured images
SAVE_FOLDER = "calibration_images"

# Create the folder if it doesn't exist
if not os.path.exists(SAVE_FOLDER):
    os.makedirs(SAVE_FOLDER)
    print(f"Folder '{SAVE_FOLDER}' created.")

# Initialize the camera (index set to 1 as per your preference)
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

# Set camera resolution to 1280x720
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

# Verify the resolution
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(f"Camera resolution set to: {width}x{height}")

print("Press 'c' to capture an image and save it.")
print("Press 'q' to quit.")

image_count = 0

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Could not read frame.")
        break

    # Rotate the image 180 degrees clockwise
    frame = cv2.rotate(frame, cv2.ROTATE_180)

    # Display the live feed
    cv2.imshow("Camera Calibration - Press 'c' to Capture", frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('c'):  # Capture and save the image
        image_path = os.path.join(SAVE_FOLDER, f"calibration_image_{image_count}.jpg")
        cv2.imwrite(image_path, frame)
        print(f"Image saved: {image_path}")
        image_count += 1

    elif key == ord('q'):  # Quit the script
        break

# Release the camera and close all windows
cap.release()
cv2.destroyAllWindows()

print(f"Total images captured: {image_count}")
print(f"Images saved in folder: '{SAVE_FOLDER}'")
