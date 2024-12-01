import cv2
import torch
import numpy as np
from torchvision.transforms import Compose, Resize, ToTensor, Normalize
from PIL import Image  # Import PIL for converting NumPy arrays

# Load MiDaS model and transforms
def load_midas_model():
    model_type = "MiDaS_small"  # Using the lightweight MiDaS model
    model = torch.hub.load("intel-isl/MiDaS", model_type)
    model.eval()

    # Correct transforms for input preprocessing
    transform = Compose([
        Resize((256, 256)),  # Resize to match MiDaS_small input dimensions
        ToTensor(),          # Convert image to tensor
        Normalize(mean=[0.5, 0.5, 0.5], std=[0.5, 0.5, 0.5])  # Normalize to [-1, 1]
    ])

    return model, transform

# Perform depth estimation on a frame
def estimate_depth(frame, model, transform):
    # Convert frame to RGB (OpenCV gives BGR by default)
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Convert the NumPy array to a PIL Image
    frame_pil = Image.fromarray(frame_rgb)

    # Preprocess the frame
    input_batch = transform(frame_pil).unsqueeze(0)  # Add batch dimension

    # Debugging: Ensure the input shape is correct
    print("Input shape to model:", input_batch.shape)  # Should be [1, 3, height, width]

    # Run MiDaS model
    with torch.no_grad():
        prediction = model(input_batch)

        # Resize the output to match the input frame size
        depth_map = torch.nn.functional.interpolate(
            prediction.unsqueeze(1),
            size=frame.shape[:2],
            mode="bicubic",
            align_corners=False
        ).squeeze()

        return depth_map.cpu().numpy()

# Main function to run the camera and depth estimation
def main():
    # Initialize the camera
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))


    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    cap.set(cv2.CAP_PROP_FPS, 120)



    # Load the MiDaS model
    print("Loading MiDaS_small model...")
    model, transform = load_midas_model()
    print("Model loaded successfully! Press 'q' to exit.")

    while True:
        # Capture a frame from the camera
        ret, frame = cap.read()
        if not ret:
            print("Error: Could not read frame.")
            break

        # Estimate depth for the current frame
        depth_map = estimate_depth(frame, model, transform)

        # Normalize depth map for visualization
        depth_map_normalized = cv2.normalize(depth_map, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)

        # Display the depth map and original frame
        cv2.imshow("Depth Map", depth_map_normalized)
        cv2.imshow("Original Frame", frame)

        # Exit the loop when 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release resources
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
