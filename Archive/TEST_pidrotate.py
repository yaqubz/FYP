# TESTED OK 18 FEB

import time, logging
from djitellopy import Tello

file_handler = logging.FileHandler("logTESTPID.log", mode='w')  # Log to a file (overwrite each run)
console_handler = logging.StreamHandler()  # Log to the terminal

formatter = logging.Formatter("%(levelname)s - %(asctime)s - %(message)s")
file_handler.setFormatter(formatter)
console_handler.setFormatter(formatter)

logging.basicConfig(level=logging.ERROR, handlers=[file_handler, console_handler])

def scan_for_marker_left(drone:Tello, yaw_rate:int=35, scan_deg:int = 90):
    # Store the initial heading at the start of the rotation
    scan_deg = min(scan_deg, 135)  # Ensures scan_deg is always < 135. For ease of returning back to original heading. CANNOT exceed 180.
    start_heading = drone.get_yaw()
    abs_rotation = 0
    
    # Rotate counterclockwise and scan for markers
    while abs_rotation < scan_deg:
        # TODO: INSERT your marker detection code here, and check if markers are found
        # If a marker is detected, stop rotation and land, if needed.
        
        drone.send_rc_control(0, 0, 0, -yaw_rate)
        current_heading = drone.get_yaw()
        total_rotation = normalize_angle(current_heading - start_heading)
        abs_rotation = abs(total_rotation)
        logging.error(f"Current heading: {current_heading} Rotation so far: {total_rotation}")
    
    drone.send_rc_control(0, 0, 0, 0)   # Stop the rotation
    current_heading = drone.get_yaw()
    actual_rotation_diff = normalize_angle(current_heading - start_heading)
    logging.error(f"Final heading: {current_heading}. Total rotation: {actual_rotation_diff}")

    drone.rotate_clockwise(abs_rotation)
    current_heading = drone.get_yaw()
    logging.error(f"Back to heading: {current_heading}.")
    time.sleep(1)


def normalize_angle(angle: float) -> float:
    """
    Normalizes an angle to the range [-180, 180).
    """
    angle = angle % 360  # Ensure the angle is within [0, 360)
    if angle > 180:
        angle -= 360  # Shift down to [-180, 180)
    return angle


drone = Tello()
drone.connect()
logging.error(drone.get_battery())
time.sleep(2)
drone.takeoff()
scan_for_marker_left(drone, scan_deg=170)
drone.end()