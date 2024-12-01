import cv2
import torch
import numpy as np
import time
from torchvision.transforms import Compose, Resize, ToTensor, Normalize
from PIL import Image

# Load MiDaS model and transforms
def load_midas_model():
    model_type = "MiDaS_small"  # Lightweight MiDaS model for faster inference
    model = torch.hub.load("intel-isl/MiDaS", model_type)
    model.eval()  # Use full precision (FP32)

    # Transform for preprocessing input images
    transform = Compose([
        Resize((128, 128)),  # Reduced input resolution for speed
        ToTensor(),
        Normalize(mean=[0.5], std=[0.5])  # Single channel normalization
    ])

    return model, transform

# Perform depth estimation on a frame
def estimate_depth(frame, model, transform):
    # Mono camera compatibility: Expand single channel to pseudo-RGB
    if len(frame.shape) == 2 or frame.shape[2] == 1:  # If grayscale
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_GRAY2RGB)
    else:
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Convert the NumPy array to a PIL Image
    frame_pil = Image.fromarray(frame_rgb)

    # Preprocess the frame
    input_batch = transform(frame_pil).unsqueeze(0)  # Add batch dimension

    # Run MiDaS model
    with torch.no_grad():
        prediction = model(input_batch)

    # Resize the output to match the input frame size
    depth_map = torch.nn.functional.interpolate(
        prediction.unsqueeze(1),
        size=frame.shape[:2],
        mode="bicubic",
        align_corners=False
    ).squeeze().cpu().numpy()

    # Ensure depth_map validity
    if depth_map is None or depth_map.size == 0 or not np.isfinite(depth_map).all():
        return np.zeros(frame.shape[:2], dtype=np.float32)  # Black placeholder for invalid maps

    return depth_map

# Main function to run the camera and depth estimation
def main():
    # Initialize the camera
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))  # Use MJPG format
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)  # Reduce resolution
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    cap.set(cv2.CAP_PROP_FPS, 60)  # Lower FPS to reduce CPU load

    # Load the MiDaS model
    print("Loading MiDaS_small model...")
    model, transform = load_midas_model()
    print("Model loaded successfully! Press 'q' to exit.")

    # FPS calculation
    frame_count = 0
    start_time = time.time()

    # Define ROI coordinates
    x_start, y_start = 100, 50  # Example: Top-left corner of ROI
    roi_width, roi_height = 60, 60
    x_end, y_end = x_start + roi_width, y_start + roi_height

    while True:
        # Capture a frame from the camera
        ret, frame = cap.read()
        if not ret:
            print("Error: Could not read frame.")
            break

        # Estimate depth for the current frame
        depth_map = estimate_depth(frame, model, transform)

        # Calculate the minimum depth in the ROI
        roi = depth_map[y_start:y_end, x_start:x_end]
        roi_min_depth = np.min(roi) if roi.size > 0 else None
        if roi_min_depth is not None:
            print(f"Minimum depth in the ROI: {roi_min_depth:.2f}")

        # Normalize depth map for visualization
        if depth_map is not None and depth_map.size > 0:
            depth_map_normalized = cv2.normalize(depth_map, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        else:
            depth_map_normalized = np.zeros(frame.shape[:2], dtype=np.uint8)  # Black placeholder

        # Visualize the ROI on the original frame
        cv2.rectangle(frame, (x_start, y_start), (x_end, y_end), (0, 255, 0), 2)  # Draw rectangle around ROI
        cv2.imshow("Depth Map", depth_map_normalized)
        cv2.imshow("Original Frame with ROI", frame)

        # FPS calculation
        frame_count += 1
        if frame_count >= 30:  # Calculate FPS every 30 frames
            elapsed_time = time.time() - start_time
            fps = frame_count / elapsed_time
            print(f"FPS: {fps:.2f}")
            frame_count = 0
            start_time = time.time()

        # Exit the loop when 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release resources
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
