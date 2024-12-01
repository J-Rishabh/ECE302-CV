import cv2

def main():
    # Open the Arducam video feed (typically at /dev/video0)
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)  # Change '0' to the correct camera ID if needed
    
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 800)
    cap.set(cv2.CAP_PROP_FPS, 120)

    

    # Check if the camera is opened
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    print("Press 'q' to exit.")

    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()

        if not ret:
            print("Error: Could not read frame.")
            break

        # Display the resulting frame
        cv2.imshow('Arducam Live Feed', frame)

        # Exit the loop when 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the capture and close the window
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()