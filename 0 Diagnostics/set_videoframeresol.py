from djitellopy import Tello
import cv2
import time

tello = Tello()
tello.connect()
print("Battery:", tello.get_battery())

# Ensure the stream is properly started
tello.streamoff()  # Ensure no previous stream is running
time.sleep(1)
tello.streamon()
time.sleep(2)  # Give time for the stream to initialize

tello.send_command_with_return("downvision 0")
tello.set_video_fps(Tello.FPS_15)

frame_reader = tello.get_frame_read()

while True:
    frame = frame_reader.frame
    if frame is None:
        print("No frame received")
        continue
    
    img = cv2.resize(frame, (360, 240))
    cv2.imshow('Tello Forward Cam', img)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

tello.streamoff()
cv2.destroyAllWindows()
