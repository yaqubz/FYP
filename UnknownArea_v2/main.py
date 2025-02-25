"""
(IMPT) Run in terminal from main workspace as:

    python -m UnknownArea_v2.main
    python -m UnknownArea_v2.main shared_params.params

TODO 11 Feb: 
- ToF (and Streaming to some extent - fail to obtain frame) doesn't work well over RPi

Takes up 23% of CPU per nav_thread. Running two nav_threads already takes up 100% CPU. (Gab's computer)
"""
import logging  # in decreasing log level: debug > info > warning > error > critical

from PPFLY2.main import execute_waypoints

from shared_utils.dronecontroller2 import DroneController
from shared_utils.shared_utils import *

import cv2
import numpy as np
import time

params = load_params()

def tof_update_thread(controller:DroneController, Hz: float = 2):
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

def check_marker_server_and_lockon(controller:DroneController, marker_id:int, display_frame) -> bool:
    """
    Logic discussed and settled between Yaqub and Gab
    Also works without a server, since .is_marker_available will return True
    Can be tested remotely; Has worked well caa 14 Feb, 25 Feb
    """

    if controller.marker_client.is_marker_available(marker_id) or marker_id == controller.markernum_lockedon:

        if controller.markernum_lockedon is None or marker_id == controller.markernum_lockedon: # first time detecting an available marker, or subsequent time detecting a marker locked on by it (but shown as no longer available)
            controller.markernum_lockedon = marker_id
            controller.marker_client.send_update('marker', marker_id=marker_id, detected=True) 
            logging.info(f"Locked onto {controller.markernum_lockedon}! Switching to approach sequence...")
            cv2.putText(display_frame, f"LOCKED ON: {controller.markernum_lockedon}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            return True


        elif centering_complete and marker_id != controller.markernum_lockedon:   
            # centering complete, but lost detection and detected another valid and available marker
            # Do NOT switch lock-on anymore! Go back to the start of the code, and hopefully restore lock-on.
            logging.debug(f"Lost locked-on marker during centering. Finding marker again.")
            centering_complete = False  
            return False        # 25 Feb TBC Gab
        
        else:   # centering incomplete, but lost detection halfway and detected something else. Switch lock-on.
            logging.debug(f"Switching lock-on from {controller.markernum_lockedon} to {marker_id}...")
            controller.marker_client.send_update('marker', marker_id=controller.markernum_lockedon, detected=False) 
            controller.markernum_lockedon = marker_id   # locking onto the new one
            controller.marker_client.send_update('marker', marker_id=controller.markernum_lockedon, detected=True)
            return False

    else: # marker detected is NOT available
        logging.info(f"Valid marker {marker_id} is NOT available (and not already locked-on previously).")
        return False
    

def nav_with_depthmap_tof(controller:DroneController, tof_dist:int, display_frame):
    """
    Navigation logic using depth map and ToF data
    First checks depth map, then ToF
    If no flags, move forward via rc control
    """
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
        if controller.depth_map_colors["blue"]["center"] > controller.depth_map_colors["red"]["center"] and tof_dist <= 700: #usually 600
            if not params.NO_FLY:
                time.sleep(0.5) 
                success = False
                retries = 3  # Increased retries for reliability

                for attempt in range(1, retries + 1):
                    response = controller.drone.send_command_with_return('cw 135', timeout=3)  # Attempt rotation
                    if response == "ok":
                        success = True
                        logging.info(f"Rotation successful on attempt {attempt}.")
                        break
                    else:
                        logging.warning(f"Rotation attempt {attempt} failed. Retrying...")
                    time.sleep(1)  # Small delay before retrying

                if not success:
                    logging.warning("Rotation command failed after multiple attempts. Executing fallback maneuver.")
                    controller.drone.move_back(30)  # Move back 30cm instead of 20cm for more clearance

                    # Secondary attempt to turn using a smaller angle as an alternative
                    time.sleep(1)
                    response = controller.drone.send_command_with_return('cw 90', timeout=5)
                    if response == "ok":
                        logging.info("Alternative 90-degree rotation successful.")
                    else:
                        logging.error("Both rotation attempts failed. Consider manual intervention or emergency stop.")

                cv2.putText(display_frame, "Avoiding Corner", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                logging.info("Avoiding Corner")
        
        elif tof_dist <= 500:   # Head-on condition
            if not params.NO_FLY:
                controller.drone.rotate_clockwise(180)
            cv2.putText(display_frame, "Avoiding Obstacle", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            logging.info("Avoiding Obstacle Ahead")

        else:   # Nothing doing - go forward
            controller.drone.send_rc_control(0, controller.move_speed, 0, 0)
            cv2.putText(display_frame, "Moving Forward", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            logging.info("Moving Forward")

def navigation_thread(controller:DroneController):
    """Main navigation thread combining depth mapping and marker detection"""
    logging.info("Starting navigation with depth mapping...")
    
    # Create single window for combined view (COMMENTED OUT 25 FEB - video thread implemented in dronecontroller)
    # cv2.namedWindow(f"Drone {controller.drone_id} Navigation", cv2.WINDOW_NORMAL)
    
    # # Ensure frame reader is initialized
    # frame_reader = controller.drone.get_frame_read()
    # logging.info("Initializing Camera...")
    # time.sleep(2)  # Give time for camera to initialize
    
    # Initial movement
    logging.info("Moving to initial altitude...")
    if not params.NO_FLY:
        controller.drone.go_to_height_PID(120)
        time.sleep(1)
    
    # Approach sequence state
    approach_complete = False
    centering_complete = False
    centering_threshold = 30    # in px
    start_time = 0
    
    while controller.is_running:  # Main loop continues until marker found or battery low
                
        try:
            next_time = time.time()
            if start_time != 0:
                refresh_rate = 1/(next_time-start_time)
                logging.debug(f"Refresh rate (Hz): {refresh_rate:.1f}")     # 25 Feb improvement from 1-2 Hz / 4 Hz, to 2.5 Hz / 4 Hz
            start_time = next_time
            
            frame = controller.get_current_frame()
            display_frame = frame.copy()
            
            # Get depth color map
            depth_colormap = controller.generate_color_depth_map(frame) # TBC 6 Feb can shift under "else" since no need to generate when markers found (10 Feb Ans: Not if you want to visualize)
            controller.process_depth_color_map(depth_colormap)
            
            # Get ToF distance
            # controller.drone.send_rc_control(0, 0, 0, 0)    # stop before getting Tof reading since it takes 0.5-7 seconds to read ToF
            tof_dist = controller.drone.get_ext_tof()      
            # dist = controller.forward_tof_dist    # if using tof_thread (commented out 3 Feb)
            logging.debug(f"ToF Dist: {tof_dist}")
            cv2.putText(display_frame, f"ToF: {tof_dist}mm", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            marker_found = False
            
            # Check for markers
            marker_found, corners, marker_id, rvecs, tvecs = controller.detect_markers(frame, display_frame)   # detects all markers; returns details of ONE valid (and land-able) marker, approved by the server
            if marker_found:  
                # Draw marker detection and pose information on the ONE detected valid marker
                display_frame = draw_pose_axes(display_frame, corners, [marker_id], rvecs, tvecs)
                logging.info(f"Obtained Marker Status from Server: {controller.marker_client.marker_status}")
                logging.debug(f"Marker detected: {marker_id}. Available: {controller.marker_client.is_marker_available(marker_id)}. Currently locked on: {controller.markernum_lockedon}")

                # PART 1: DETECTION AND LOCK-ON LOGIC
                goto_approach_sequence:bool = check_marker_server_and_lockon(controller, marker_id, display_frame)
                
                # PART 2: CENTERING AND APPROACH LOGIC
                # Still work in progress caa 20 Feb. Can only be tested properly IRL
                if goto_approach_sequence is True and not params.NO_FLY:                  
                    logging.info(f"Locked on: {controller.markernum_lockedon}. Centering/approaching...")
                    controller.marker_client.send_update('status', f"Locked on: {controller.markernum_lockedon}. Centering/approaching...")
                    
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
                            time.sleep(0.5)  # Stabilize
                            centering_complete = True
                    
                    # PART 2A: APPROACH (I.E. CENTERING COMPLETE)
                    elif not approach_complete:
                        current_distance_3D = controller.get_distance()
                        if current_distance_3D is None:
                            logging.info("Lost marker during approach...")  # should not reach here! caa 13 Feb
                            time.sleep(0.1)
                            continue

                        current_height = controller.drone.get_distance_tof() - params.EXTRA_HEIGHT
                        current_distance_2D = np.sqrt(current_distance_3D**2 - current_height**2)

                        logging.info(f"3D distance to marker: {current_distance_3D:.1f}cm \n Drone height: {current_height:.1f}cm \n 2D distance to marker: {current_distance_2D:.1f}cm")
                        cv2.putText(display_frame, "Centered. Approaching...", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 3, (255, 255, 255), 3)

                        if current_distance_2D >= 500:
                            step_dist:int = 100
                            logging.info(f"{current_distance_2D:.2f}cm distance too large. Stepping forward {step_dist}cm to approach marker...")
                            # controller.drone.send_rc_control(0, 0, 0, 0)
                            controller.drone.move_forward(step_dist)
                            time.sleep(0.5)  # Wait for movement to complete
                            centering_complete = False
                            continue        # re-enter the loop; need to re-detect marker and re-measure distance. 

                        elif current_distance_2D > 0 and current_distance_2D < 500:
                            logging.info(f"Final Approach: Moving forward {int(current_distance_2D)}cm to marker.")
                            controller.drone.move_forward(int(current_distance_2D))
                            logging.info("Approach complete!")
                            controller.drone.send_rc_control(0, 0, 0, 0)
                            approach_complete = True
                            controller.marker_client.send_update('marker', marker_id=marker_id, landed=True)
                            time.sleep(1)
                            break

                elif goto_approach_sequence is True and params.NO_FLY == True: # if simulating
                    logging.info("Marker found, but not landing since not flying. Program continues.")

                else:   #goto_approach_sequence is False
                    logging.debug("Marker found, but not approaching yet.")
            
            elif controller.exit_detected: # This condition is placed after marker_detected but before depth mapping 
                exit_text:str = f"Exit detected {controller.exit_distance_3D:.0f}cm away."
                logging.info("No valid markers detected." + exit_text)
                cv2.putText(display_frame, exit_text, (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                if not params.NO_FLY and controller.exit_distance_3D < 300:
                    logging.info(f"Avoiding exit. Turning 90 degrees.")
                    controller.drone.rotate_clockwise(90)

                elif not params.NO_FLY:
                    logging.info(f"Exit more than 3m away, no action taken.")

            else: # Navigation logic using depth map if neither victim nor exit detected.
                  # NOTE 4 Feb: Check ToF after depth map should enable it to enter tighter spaces. To be more conservative, can consider checking ToF before depth map.)
                
                logging.debug(f"Nothing detected.")
                
                if not controller.markernum_lockedon is None:        # Publish that its lost track of target. Then resets its locked_on number.
                    logging.debug(f"Resetting markernum_lockedon from {controller.markernum_lockedon} to None")
                    controller.marker_client.send_update('marker', marker_id=controller.markernum_lockedon, detected=False)
                    controller.markernum_lockedon = None

                nav_with_depthmap_tof(controller, tof_dist, display_frame)      # logic for depth map and ToF
            
            # Resize depth_colormap to match frame dimensions, create combined view side-by-side
            depth_colormap_resized = cv2.resize(depth_colormap, (display_frame.shape[1]//2, display_frame.shape[0]))
            combined_view = np.hstack((display_frame, depth_colormap_resized))
            
            # Add labels and display combined view
            cv2.putText(combined_view, "Live Feed", (10, combined_view.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(combined_view, "Depth Map", (display_frame.shape[1] + 10, combined_view.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            # cv2.imshow(f"Drone {controller.drone_id} Navigation", combined_view)      # DO NOT SHOW - already displaying in dronecontroller
            controller.set_display_frame(combined_view)
            
            # # Keyboard shortcuts! (ok 14 Feb)
            # key = cv2.waitKey(1)        # can print key to debug
            # if key == ord('q'):
            #     break
            # elif key == ord('l'):
            #     logging.debug("DEBUG LOGPOINT")
                
        except Exception as e:
            logging.error(f"Error in navigation: {e}. Continuing navigation.")      # 13 Feb: "ERROR - libav.h264 - error while decoding MB 57 29, bytestream -10" does not come here
            # Don't cleanup or break - continue searching
            continue           

def main():     
    global DIST_COEFF
   
    # Setup logging
    logger = setup_logging(params, "UnknownSearchArea")
    logger.info(f"Starting unknown area main with drone_id: {params.PI_ID}")
    controller = DroneController(params.NETWORK_CONFIG, drone_id=params.PI_ID, laptop_only=params.LAPTOP_ONLY, load_midas=True)
    try:
        if not params.NO_FLY:    
            controller.drone.takeoff()
            controller.marker_client.send_update('status', status_message='Waiting for takeoff')
            # controller.takeoff_simul([11,12,15])
            logging.info("Taking off for real...")
            controller.marker_client.send_update('status', status_message='Flying')
            # controller.drone.go_to_height_PID(100)
            controller.drone.send_rc_control(0, 0, 0, 0)
            # execute_waypoints(params.WAYPOINTS_JSON, controller.drone, params.NO_FLY)
        else:
            logging.info("Simulating takeoff. Drone will NOT fly.")
            # execute_waypoints("waypoints_samplesmall.json", controller.drone, params.NO_FLY)
        time.sleep(2)
        
        # tof_thread = threading.Thread(target=tof_update_thread, args=(controller,2))
        # tof_thread.daemon = True     ## Daemon threads run in the background, and are killed automatically when your program quits. 
        # tof_thread.start()

        navigation_thread(controller)

    except Exception as e:
        logging.error(f"Error in main: {e}. Landing.")

    finally:
        controller.marker_client.send_update('status', status_message='Landing')
        logging.info(f"Actually landing for real. End Battery Level: {controller.drone.get_battery()}%")
        controller.drone.end()
        controller.marker_client.send_update('status', status_message='Landed')
        cv2.destroyAllWindows()
        cv2.waitKey(1)

if __name__ == "__main__":
    main()