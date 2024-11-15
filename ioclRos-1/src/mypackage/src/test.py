import cv2
import subprocess

def print_camera_device_path():
    # Get the device path using v4l2-ctl
    command = f"v4l2-ctl --list-devices"
    devices = subprocess.check_output(command, shell=True).decode('utf-8')
    print("Connected camera devices:")
    print(devices)

def main():
    # Change the camera index if necessary (0 for the first camera, 1 for the second, etc.)
    camera_index = 0  # or 1

    # Print the camera device path
    print_camera_device_path()

    # Open the webcam
    cap = cv2.VideoCapture(camera_index)

    if not cap.isOpened():
        print("Error: Could not open video device.")
    else:
        print("Webcam is opened successfully!")

        while True:
            # Capture frame-by-frame
            ret, frame = cap.read()
            if not ret:
                print("Error: Failed to capture frame.")
                break

            # Display the resulting frame
            cv2.imshow('Webcam', frame)

            # Break the loop on 'q' key press
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        print("Exiting webcam test...")

    # When everything is done, release the capture
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
