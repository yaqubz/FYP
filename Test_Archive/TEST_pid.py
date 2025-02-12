from combine import DroneController # works 6 Feb but need to copy to main directory to run (TBC why) 
import time


NETWORK_CONFIG = {
    # 'host': '192.168.0.117',  # if connected through RPi 17
    'host': '192.168.10.1',     # if connected directly through WiFi
    'control_port': 8889,
    'state_port': 8890,
    'video_port': 11111
}

controller = DroneController(NETWORK_CONFIG)
controller.drone.takeoff()
print(f"Start Height: {controller.drone.get_height()}, Downward ToF: {controller.drone.get_distance_tof()}")
controller.drone.go_to_height_PID(140)
time.sleep(2)
controller.drone.go_to_height_PID(130)
time.sleep(2)

print(f"End Height: {controller.drone.get_height()}, Downward ToF: {controller.drone.get_distance_tof()}")
controller.drone.end()