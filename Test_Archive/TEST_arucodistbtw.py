"""
python -m Test_Archive.TEST_arucodistbtw
"""

import cv2
from cv2 import aruco
import numpy as np
from djitellopy import Tello
from threading import Lock
import time
import os
import logging  # in decreasing log level: debug > info > warning > error > critical

from UnknownArea_v2.dronecontroller import DroneController
from UnknownArea_v2.customtello import MockTello
from UnknownArea_v2.utils import *

NO_FLY = True     # indicate NO_FLY = True so that the drone doesn't fly but the video feed still appears
LAPTOP_ONLY = True

# Logging handlers and format
file_handler = logging.FileHandler("log_arucodetector.log", mode='w')  # Log to a file (overwrite each run)
console_handler = logging.StreamHandler()  # Log to the terminal

formatter = logging.Formatter("%(levelname)s - %(asctime)s - %(message)s")
file_handler.setFormatter(formatter)
console_handler.setFormatter(formatter)

logging.basicConfig(level=logging.DEBUG, handlers=[file_handler, console_handler])

# Define network configuration constants
NETWORK_CONFIG = {
    # 'host': '192.168.0.117',  # if connected through RPi 17
    'host': '192.168.10.1',     # if connected directly through WiFi
    'control_port': 8889,
    'state_port': 8890,
    'video_port': 11111
}

def navigation_thread(controller):
    logging.info("Starting navigation with depth mapping...")
    
    # Create single window for combined view
    cv2.namedWindow("Drone Navigation", cv2.WINDOW_NORMAL)
    
    # Ensure frame reader is initialized
    frame_reader = controller.drone.get_frame_read()
    logging.info("Initializing Camera...")
    time.sleep(2)  # Give time for camera to initialize
    
    
    while True:  # Main loop continues until marker found or battery low
        try:
            # Get frame with retry mechanism
            retry_count = 0
            frame = None
            while frame is None and retry_count < 3:
                frame = frame_reader.frame
                if frame is None:
                    logging.warning("Frame capture failed, retrying...")
                    time.sleep(0.1)
                    retry_count += 1
            
            if frame is None:
                logging.warning("Failed to capture frame after retries")
                continue
                
            # Check for markers
            marker_found, corners, marker_id, rvecs, tvecs = controller.detect_markers(frame)   # detects all markers; returns details of ONE valid (and land-able) marker, approved by the server
            
            if marker_found:  
                # Draw marker detection and pose information on the ONE detected valid marker
                frame = draw_pose_axes(frame, corners, [marker_id], rvecs, tvecs)
                logging.info(f"Obtained Marker Status from Server: {controller.marker_client.marker_status}")
            
            # Display combined view
            cv2.imshow("Drone Navigation", frame)
            
            # If 'q' is pressed, just continue searching
            if cv2.waitKey(1) & 0xFF == ord('q'):
                logging.info("Exiting. Drone landing.")
                # controller.drone.land()
                break
                
        except Exception as e:
            logging.warning(f"Error in navigation: {e}. Continuing navigation.")
            # Don't cleanup or break - continue searching
            continue

def main():
    controller = DroneController(NETWORK_CONFIG, laptop_only=LAPTOP_ONLY)
    navigation_thread(controller) 

    logging.info("Actually landing for real.")
    controller.drone.end()
    cv2.destroyAllWindows()
    cv2.waitKey(1)

if __name__ == "__main__":
    main()