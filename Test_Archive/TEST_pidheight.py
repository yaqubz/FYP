"""
Works 18 Feb
python -m Test_Archive.TEST_pidheight
""" 

from shared_utils.dronecontroller2 import DroneController 
import time


NETWORK_CONFIG = {
    # 'host': '192.168.0.117',  # if connected through RPi 17
    'host': '192.168.10.1',     # if connected directly through WiFi
    'control_port': 8889,
    'state_port': 8890,
    'video_port': 11111
}

controller = DroneController(NETWORK_CONFIG, drone_id=1, load_midas=False)
controller.drone.takeoff()
print(f"Start Height: {controller.drone.get_height()}, Downward ToF: {controller.drone.get_distance_tof()}")
controller.drone.go_to_height_PID(140)
time.sleep(2)
controller.drone.go_to_height_PID(60)
time.sleep(2)

print(f"End Height: {controller.drone.get_height()}, Downward ToF: {controller.drone.get_distance_tof()}")
controller.drone.end()