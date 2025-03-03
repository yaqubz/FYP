from shared_utils.customtello import MockTello
import time

# Initialize MockTello with UWB simulation
tello = MockTello(uwb_sim=True, uwb_ip='127.0.0.1', uwb_port=5000, tag_id=0)

# Simulate drone commands
tello.takeoff()
tello.move_forward(200)  # Move forward 200 cm (no need for time.sleep)
tello.rotate_clockwise(80)  # Rotate 90 degrees clockwise (no need for time.sleep)
tello.send_rc_control(-50, 0, 0, 0)  # Move forward (x=50), no left/right (y=0), no up/down (z=0), no yaw (yaw=0)
time.sleep(2)
tello.send_rc_control(50, 0, 0, 0)  # Move forward (x=50), no left/right (y=0), no up/down (z=0), no yaw (yaw=0)
time.sleep(2)
tello.rotate_counter_clockwise(80)  # Rotate 90 degrees clockwise (no need for time.sleep)
tello.move_forward(200)  # Move forward 200 cm (no need for time.sleep)
tello.land()
tello.end()