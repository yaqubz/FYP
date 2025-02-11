# WORKS 23 JAN: Reads waypoint JSON file and executes it, then changes over to navigation_thread to use depth-mapping and lands on valid marker.
# IMPT: Must be connected to drone to work!
# Testing 6 Feb modularised
# Testing 11 Feb markerserverclient

"""
Change Log:
# 23 Jan: Works even without ToF; Can run without flying using NO_FLY = True; Press q to exit and land
# 24 Jan: Can stream and work over RPi17; TODO: video stream thread, ToF doesn't work well over RPi
# 29 Jan: Rework 2D and 3D distance mapping, Calibration
# 30 Jan: Distance calculation should be quite precise now, no need for hard-coded offsets. Will not work for victims not on the floor (TBC if need)
# 6 Feb: BIG Changes
    - 
    -
    -
    - (IMPT) Run in terminal from main workspace as "python -m UnknownArea_v2.main"
        python -m UnknownArea_v2.main
Takes up 23% of CPU per nav_thread (Gab's computer)
"""
import logging  # in decreasing log level: debug > info > warning > error > critical

# Logging handlers and format (IMPT: Must be before importing other modules!?)
file_handler = logging.FileHandler("log_UnknownSearchArea_main.log", mode='w')  # Log to a file (overwrite each run)
console_handler = logging.StreamHandler()  # Log to the terminal
formatter = logging.Formatter("%(levelname)s - %(name)s - %(asctime)s - %(message)s")
file_handler.setFormatter(formatter)
console_handler.setFormatter(formatter)
logging.basicConfig(level=logging.DEBUG, handlers=[file_handler, console_handler])

# IDK TESTING 11 FEB
# logger = logging.getLogger(__name__)
logger = logging.getLogger("ABC")
logger.addHandler(file_handler)
logger.info("TESTING NEW LOGGER")

from PPFLY2.main import execute_waypoints

from .dronecontroller import DroneController
from .utils import *

import torch
import cv2
from cv2 import aruco
import numpy as np
from djitellopy import Tello
import threading
from threading import Lock
import time
import os

LAPTOP_ONLY = True # indicate LAPTOP_ONLY = True to use MockTello() and laptop webcam instead
NO_FLY = True     # indicate NO_FLY = True to connect to the drone, but ensure it doesn't fly while the video feed still appears

EXTRA_HEIGHT = 0   # cm; for demo purpose if obstacle is higher than the ground 

# Define network configuration constants
NETWORK_CONFIG = {
    # 'host': '192.168.0.103',  # if connected through RPi
    # 'control_port': 9003,
    # 'state_port': 8003,
    # 'video_port': 11103
    'host': '192.168.10.1',     # if connected directly through WiFi
    'control_port': 8889,
    'state_port': 8890,
    'video_port': 11111    
}

def draw_pose_axes(frame, corners, ids, rvecs, tvecs):
    """Draw pose estimation axes and information on frame"""
    logging.debug(f"ID {ids} found. Drawing axes.")
    marker_center = np.mean(corners[0], axis=0)
    cv2.circle(frame, 
                (int(marker_center[0]), int(marker_center[1])), 
                10, (0, 255, 0), -1)
    cv2.polylines(frame, 
                [corners[0].astype(np.int32)], 
                True, (0, 255, 0), 2)
    cv2.putText(frame, 
                f"ID: {ids}", 
                (int(marker_center[0]), int(marker_center[1] - 20)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
       
    cv2.drawFrameAxes(
        frame, CAMERA_MATRIX, DIST_COEFF, rvecs, tvecs, 10
    )
    
    rot_matrix = cv2.Rodrigues(rvecs)[0]
    euler_angles = cv2.RQDecomp3x3(rot_matrix)[0]
    x, y, z = tvecs[0]
    roll, pitch, yaw = euler_angles
    
    euclidean_distance = np.sqrt(x*x + y*y + z*z)
    
    position_text = f"Pos (cm): X:{x:.1f} Y:{y:.1f} Z:{z:.1f}"
    rotation_text = f"Rot (deg): R:{roll:.1f} P:{pitch:.1f} Y:{yaw:.1f}"
    distance_text = f"3D Distance: {euclidean_distance:.1f} cm"
    
    cv2.putText(frame, position_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    cv2.putText(frame, rotation_text, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    cv2.putText(frame, distance_text, (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    
    return frame

def tof_update_thread(controller, Hz: float = 2):
    """
    TBC not exactly working, 6 Feb
    Subthread for updating forward ToF reading
    Implemented 3 Feb, works but may block other responses from drone
    May work better with RPi? 
    Possible solution: Find optimal refresh rate? Use both subthread AND main thread?
    
    """
    while True:
        with controller.forward_tof_lock:
            controller.forward_tof_dist = controller.drone.get_ext_tof()
        time.sleep(1/Hz)
        logging.debug(f"controller.forward_tof_dist: {controller.forward_tof_dist}")

def navigation_thread(controller):
    """Main navigation thread combining depth mapping and marker detection"""
    logging.info("Starting navigation with depth mapping...")
    
    # Create single window for combined view
    cv2.namedWindow("Drone Navigation", cv2.WINDOW_NORMAL)
    
    # Ensure frame reader is initialized
    frame_reader = controller.drone.get_frame_read()
    logging.info("Initializing Camera...")
    time.sleep(2)  # Give time for camera to initialize
    
    # Initial movement
    logging.info("Moving to initial altitude...")
    if not NO_FLY:
        controller.drone.go_to_height_PID(120)
        time.sleep(1)
    
    # Approach sequence state
    approach_complete = False
    centering_complete = False
    centering_threshold = 30    # in px
    logging.debug(f"Centering Threshold: {centering_threshold}")
    
    while True:  # Main loop continues until marker found or battery low
        try:
            frame = capture_frame(frame_reader)
            display_frame = frame.copy()    # Create a copy of frame for visualization
            
            # Get depth color map
            depth_colormap = controller.generate_color_depth_map(frame) # TBC 6 Feb can shift under "else" since no need to generate when markers found (10 Feb Ans: Not if you want to visualize)
            controller.process_depth_color_map(depth_colormap)
            
            # Get ToF distance
            tof_dist = controller.drone.get_ext_tof()      
            # dist = controller.forward_tof_dist    # if using tof_thread (commented out 3 Feb)
            logging.debug(f"ToF Dist: {tof_dist}")
            cv2.putText(display_frame, f"ToF: {tof_dist}mm", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            marker_found = False
            
            # Check for markers
            marker_found, corners, marker_id, rvecs, tvecs = controller.detect_markers(frame)   # detects all markers; returns details of ONE valid (and land-able) marker, approved by the server
            if marker_found:  
                # Draw marker detection and pose information on the ONE detected valid marker
                display_frame = draw_pose_axes(display_frame, corners, [marker_id], rvecs, tvecs)
                logging.info(f"Obtained Marker Status from Server: {controller.marker_client.marker_status}")
                logging.debug(f"Marker detected: {marker_id}. Available: {controller.marker_client.is_marker_available(marker_id)}. Currently locked on: {controller.markernum_lockedon}")

                # Difficult logic, discussed between Yaqub and Gab 11 Feb
                if controller.marker_client.is_marker_available(marker_id) or marker_id == controller.markernum_lockedon:
                    if controller.markernum_lockedon is None or marker_id == controller.markernum_lockedon: # first time detecting an available marker, or subsequent time detecting a marker locked on by it (but shown as no longer available)
                        controller.markernum_lockedon = marker_id
                        controller.marker_client.send_update(marker_id, detected=True) 
                        logging.info(f"Valid marker {marker_id} available and locked onto {controller.markernum_lockedon}! Switching to approach sequence...")
                        goto_approach_sequence = True
                        cv2.putText(display_frame, f"LOCKED ON: {controller.markernum_lockedon}", (250, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)


                    else:   # detected a valid and available marker, but it's different from the one locked onto previously
                        logging.debug(f"Switching lock-on from {controller.markernum_lockedon} to {marker_id}.")
                        controller.marker_client.send_update(controller.markernum_lockedon, detected=False) 
                        controller.markernum_lockedon = marker_id   # locking onto the new one
                        controller.marker_client.send_update(controller.markernum_lockedon, detected=True)
                        goto_approach_sequence = False
                    
                else: # marker detected is NOT available
                    logging.info(f"Valid marker {marker_id} is NOT available (and not already locked-on previously).")
                    goto_approach_sequence = False
                
                if goto_approach_sequence is True and not NO_FLY:                  
                    logging.info(f"Still locked on and approaching marker {controller.markernum_lockedon}...")
                    
                    # Center on the marker
                    frame_center = frame.shape[1] / 2
                    marker_center = np.mean(corners[0], axis=0)
                    x_error = marker_center[0] - frame_center
                    
                    if not centering_complete:
                        if abs(x_error) > centering_threshold:
                            # Calculate yaw speed based on error
                            yaw_speed = int(np.clip(x_error / 10, -20, 20))
                            controller.drone.send_rc_control(0, 0, 0, yaw_speed)
                            logging.info(f"Centering: error = {x_error:.1f}, yaw_speed = {yaw_speed}")
                        else:
                            logging.info("Marker centered! Starting approach...")
                            controller.drone.send_rc_control(0, 0, 0, 0)  # Stop rotation
                            time.sleep(1)  # Stabilize
                            centering_complete = True
                    
                    elif not approach_complete:
                        current_distance_3D = controller.get_distance()
                        if current_distance_3D is None:
                            logging.info("Lost marker during approach...")
                            time.sleep(0.1)
                            continue

                        current_height = controller.drone.get_distance_tof() - EXTRA_HEIGHT
                        current_distance_2D = np.sqrt(current_distance_3D**2 - current_height**2)

                        logging.info(f"3D distance to marker: {current_distance_3D:.1f}cm \n Drone height: {current_height:.1f}cm \n 2D distance to marker: {current_distance_2D:.1f}cm")

                        if current_distance_2D >= 300:
                            step_dist:int = 150
                            logging.info(f"Distance too large. Stepping forward {step_dist}cm to approach marker...")
                            controller.drone.send_rc_control(0, 0, 0, 0)
                            controller.drone.move_forward(step_dist)

                            time.sleep(1)  # Wait for movement to complete
                            new_distance = controller.get_distance()
                            if new_distance is not None:
                                logging.info(f"New 3D distance to marker: {new_distance:.1f}cm. Recentering...")
                                centering_complete = False
                                centering_threshold -= 3    # reduce threshold for better accuracy (TBC 6 Feb)
                            else:
                                logging.info("Lost marker during approach step...")
                                centering_threshold = 30
                                centering_complete = False

                        elif current_distance_2D > 0 and current_distance_2D < 300:
                            logging.info(f"Final Approach: Moving forward {int(current_distance_2D)}cm to marker.")
                            controller.drone.move_forward(int(current_distance_2D))
                            logging.info("Approach complete!")
                            controller.drone.send_rc_control(0, 0, 0, 0)
                            approach_complete = True
                            controller.marker_client.send_update(marker_id, landed=True)
                            time.sleep(1)
                            break

                elif goto_approach_sequence is True and NO_FLY == True: # if simulating
                    logging.info("Marker found, but not landing since not flying. Program continues.")

                else:   #goto_approach_sequence is False
                    logging.debug("Marker found, but not approaching yet.")
            
            elif controller.exit_detected: # This condition is placed after marker_detected but before depth mapping 
                ## MTD 1: Simple U-turn when detected
                if not NO_FLY and controller.exit_distance_3D < 300:
                    logging.info(f"Avoiding exit {controller.exit_distance_3D:.0f}cm away. Turning 180 degrees.")
                    controller.drone.rotate_clockwise(180)

                elif not NO_FLY:
                    logging.info(f"Exit {controller.exit_distance_3D:.0f}cm away. More than 3m away, no action taken.")

                controller.exit_detected = False    # resets, then always needs redetection
                controller.exit_distance_3D = None
                
                ## MTD 2: TESTING "FORCE-FIELD METHOD"
                # if controller.target_yaw is not None:
                #     # Get current drone yaw (e.g., from IMU)
                #     current_yaw = controller.drone.get_yaw() 
                #     logging.debug(f"Current yaw: {current_yaw:.2f}°")

                #     # Calculate shortest turn angle
                #     delta_yaw = controller.target_yaw - current_yaw
                #     delta_yaw = (delta_yaw + 180) % 360 - 180  # Normalize to [-180, 180]
                    
                #     # Turn the drone (TBC direction)
                #     if not NO_FLY:
                #         if delta_yaw > 0:
                #             controller.drone.rotate_clockwise(int(delta_yaw))
                #         else:
                #             controller.drone.rotate_counter_clockwise(-int(delta_yaw))
                #     logging.info(f"Turning {delta_yaw:.2f}° to align with exit marker")
                    
                #     # Reset exit state
                #     controller.exit_detected = False
                #     controller.target_yaw = None
                # else:
                #     logging.warning("Exit detected but no target yaw!")

            else: # Navigation logic using depth map if neither victim nor exit detected.
                  # NOTE 4 Feb: Check ToF after depth map should enable it to enter tighter spaces. To be more conservative, can consider checking ToF before depth map.)
                
                # Publish that its lost track of target. Then resets its locked_on number.
                logging.debug(f"Nothing detected. Resetting markernum_lockedon from {controller.markernum_lockedon} to None")
                controller.marker_client.send_update(controller.markernum_lockedon, detected=False)
                controller.markernum_lockedon = None

                if controller.depth_map_colors["red"]["center"] > controller.depth_map_colors["blue"]["center"]:
                    # Obstacle ahead - turn towards more open space
                    if controller.depth_map_colors["blue"]["left"] > controller.depth_map_colors["blue"]["right"]:
                        controller.drone.send_rc_control(0, 0, 0, -controller.yaw_speed)
                        cv2.putText(display_frame, "Turning Left", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                        logging.info("Turning Left")
                    else:
                        controller.drone.send_rc_control(0, 0, 0, controller.yaw_speed)
                        cv2.putText(display_frame, "Turning Right", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                        logging.info("Turning Right")
                else:
                    # Corner condition where center appears further than sides
                    if controller.depth_map_colors["blue"]["center"] > controller.depth_map_colors["red"]["center"] and tof_dist <= 600:
                        if not NO_FLY:
                            controller.drone.rotate_clockwise(135)
                        cv2.putText(display_frame, "Avoiding Corner", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                        logging.info("Avoiding Corner")
                    
                    elif tof_dist <= 500:   # Head-on condition
                        if not NO_FLY:
                            controller.drone.rotate_clockwise(180)
                        cv2.putText(display_frame, "Avoiding Obstacle", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                        logging.info("Avoiding Obstacle Ahead")

                    else:   # Nothing doing - go forward
                        controller.drone.send_rc_control(0, controller.move_speed, 0, 0)
                        cv2.putText(display_frame, "Moving Forward", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                        logging.info("Moving Forward")
            
            # Resize depth_colormap to match frame dimensions
            depth_colormap_resized = cv2.resize(depth_colormap, (display_frame.shape[1]//2, display_frame.shape[0]))
            
            # Create combined view with original frame and depth map side by side
            combined_view = np.hstack((display_frame, depth_colormap_resized))
            
            # Add labels and display combined view
            cv2.putText(combined_view, "Live Feed", (10, combined_view.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(combined_view, "Depth Map", (display_frame.shape[1] + 10, combined_view.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.imshow("Drone Navigation", combined_view)
            
            # If 'q' is pressed, just continue searching
            if cv2.waitKey(1) & 0xFF == ord('q'):
                logging.info("Exiting. Drone landing.")
                break
                
        except Exception as e:
            logging.error(f"Error in navigation: {e}. Continuing navigation.")
            # Don't cleanup or break - continue searching
            continue

def main():
    global CAMERA_MATRIX, DIST_COEFF

    controller = DroneController(NETWORK_CONFIG, simulate=LAPTOP_ONLY)
    try:
        logging.info("Taking off...")
        if not NO_FLY:    
            controller.drone.takeoff()
            controller.drone.go_to_height_PID(100)
            controller.drone.send_rc_control(0, 0, 0, 0)
            # execute_waypoints("waypoints_samplesmall.json", controller.drone, NO_FLY)
            # execute_waypoints("waypoint20x20.json", controller.drone, NO_FLY)
        else:
            logging.info("Simulating takeoff. Drone will NOT fly.")
            # execute_waypoints("waypoints_samplesmall.json", controller.drone, NO_FLY)
        time.sleep(2)
        
        # tof_thread = threading.Thread(target=tof_update_thread, args=(controller,2))
        # tof_thread.daemon = True     ## Daemon threads run in the background, and are killed automatically when your program quits. 
        # tof_thread.start()

        navigation_thread(controller)

        logging.info(f"Actually landing for real. End Battery Level: {controller.drone.get_battery()}%")
        controller.drone.end()
        cv2.destroyAllWindows()
        cv2.waitKey(1)
    
    except Exception as e:
        logging.error(f"Error in main: {e}")

if __name__ == "__main__":
    main()