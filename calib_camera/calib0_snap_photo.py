import cv2
import os
from djitellopy import Tello

TELLO_NO: str = 'D'

# Create a folder to save captured images
output_folder = "calib_camera/calibration_images"
os.makedirs(output_folder, exist_ok=True)

# Initialize the Tello drone
tello = Tello()
tello.connect()
print("Battery:", tello.get_battery(), "%")

# Start video stream
tello.streamon()

# OpenCV window setup
cv2.namedWindow("Tello Stream", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Tello Stream", 800, 600)

frame_count = 0

try:
    while True:
        # Get the video frame from Tello
        frame = tello.get_frame_read().frame

        # Display the video stream
        cv2.imshow("Tello Stream", frame)

        # Check for keyboard inputs
        key = cv2.waitKey(1) & 0xFF

        if key == 32:  # Spacebar to save image
            frame_count += 1
            filename = os.path.join(output_folder, f"tello{TELLO_NO}_img{frame_count:03d}.jpg")
            cv2.imwrite(filename, frame)
            print(f"Saved: {filename}")

        elif key == 27:  # Escape key to exit
            print("Exiting...")
            break

except KeyboardInterrupt:
    print("\nInterrupted by user.")

finally:
    # Cleanup
    tello.streamoff()
    cv2.destroyAllWindows()
    print("Done.")
