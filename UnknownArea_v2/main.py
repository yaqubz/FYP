"""
(IMPT) Run in terminal from main workspace as:

    python -m UnknownArea_v2.main shared_params.paramsXX

TODO 26 Feb: 
- Improve reliability and repeatability
- Decide whether ToF and video threads are worth it
- Victim centering + Danger avoiding
- Tune parameters for centering yaw etc.

Takes up 23% of CPU per nav_thread. Running two nav_threads already takes up 100% CPU. (Gab's computer)
"""
import logging  # in decreasing log level: debug > info > warning > error > critical

from PPFLY2.main import execute_waypoints

from shared_utils.dronecontroller2 import DroneController
from shared_utils.shared_utils import *

import cv2
import numpy as np
import time
import threading 

params = load_params()

def check_marker_server_and_lockon(controller:DroneController, marker_id:int, display_frame) -> bool:
    """
    Logic discussed and settled between Yaqub and Gab
    Also works without a server, since .is_marker_available will return True
    Can be tested remotely; Has worked well caa 14 Feb, 25 Feb
    """

    if controller.marker_client.is_marker_available(marker_id) or marker_id == controller.markernum_lockedon:

        if controller.markernum_lockedon is None or marker_id == controller.markernum_lockedon: # first time detecting an available marker, or subsequent time detecting a marker locked on by it (but shown as no longer available)
            controller.markernum_lockedon = marker_id
            controller.marker_client.send_update('marker', marker_id=marker_id, detected=True)  # NOTE 26 Feb - this itself takes 60ms!
            logger.info(f"Locked onto {controller.markernum_lockedon}! Switching to approach sequence...")
            cv2.putText(display_frame, f"LOCKED ON: {controller.markernum_lockedon}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            return True


        elif centering_complete and marker_id != controller.markernum_lockedon:   
            # centering complete, but lost detection and detected another valid and available marker
            # Do NOT switch lock-on anymore! Go back to the start of the code, and hopefully restore lock-on.
            logger.debug(f"Lost locked-on marker during centering. Finding marker again.")
            centering_complete = False  
            return False        # 25 Feb TBC Gab
        
        else:   # centering incomplete, but lost detection halfway and detected something else. Switch lock-on.
            logger.debug(f"Switching lock-on from {controller.markernum_lockedon} to {marker_id}...")
            controller.marker_client.send_update('marker', marker_id=controller.markernum_lockedon, detected=False) 
            controller.markernum_lockedon = marker_id   # locking onto the new one
            controller.marker_client.send_update('marker', marker_id=controller.markernum_lockedon, detected=True)
            return False

    else: # marker detected is NOT available
        logger.info(f"Valid marker {marker_id} is NOT available (and not already locked-on previously).")
        return False
    

def nav_with_depthmap_tof(controller:DroneController, tof_dist:int, display_frame):
    """
    Navigation logic using depth map and ToF data
    First checks depth map, then ToF
    If no flags, move forward via rc control
    """
    
    def recovery():
        """
        Attempts to recover the drone's movement if it's stuck.
        Tries moving back, then left, then right.
        If all attempts fail, retries from the beginning.
        """
        while True: 
            recovery_moves = [
                ("back", controller.drone.move_back, 30),
                ("left", controller.drone.move_left, 30),
                ("right", controller.drone.move_right, 30),
            ]

            for direction, move_command, distance in recovery_moves:
                try:
                    logging.info(f"Attempting recovery: Moving {direction} {distance}cm")
                    move_command(distance)  # Execute the movement command
                    logging.info(f"Recovery successful: Moved {direction} {distance}cm")
                    return  # Exit if successful
                except Exception as e:
                    logging.warning(f"Recovery attempt failed: Could not move {direction} - {e}")

                time.sleep(1)  # Small delay before next attempt
                
            logging.error("All recovery attempts failed. Restarting recovery process.")
            
    if tof_dist == 8888: # ToF error (while loop)
        controller.drone.send_rc_control(0, 0, 0,0)
        cv2.putText(display_frame, "Pausing, ToF error", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        logger.info("Pausing, ToF error")
        return display_frame

    if controller.depth_map_colors["middle_row"]["middle_center"]["red"] > controller.depth_map_colors["middle_row"]["middle_center"]["blue"]:
        # Obstacle ahead - turn towards more open space
        if controller.depth_map_colors["middle_row"]["middle_left"]["blue"] > controller.depth_map_colors["middle_row"]["middle_right"]["blue"]:
            controller.drone.send_rc_control(0, 0, 0, -controller.yaw_speed)
            cv2.putText(display_frame, "Turning Left", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            logger.info("Turning Left")
        else:
            controller.drone.send_rc_control(0, 0, 0, controller.yaw_speed)
            cv2.putText(display_frame, "Turning Right", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            logger.info("Turning Right")
    else:
        if controller.depth_map_colors["middle_row"]["middle_center"]["blue"] > controller.depth_map_colors["middle_row"]["middle_center"]["red"] and tof_dist <= 900: #OG 700
            cv2.putText(display_frame, "Avoiding Corner", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            logger.info("Avoiding Corner")

            # TBC TODO 26 FEB - ALWAYS AVOIDING CORNER

            if not params.NO_FLY:
                time.sleep(0.5) 
                success = False
                retries = 3  # Increased retries for reliability

                for attempt in range(1, retries + 1):
                    response = controller.drone.send_command_with_return('cw 150', timeout=3)  # Attempt rotation (OG: 135)
                    if response == "ok":
                        success = True
                        logger.info(f"Rotation successful on attempt {attempt}.")
                        break
                    else:
                        logger.warning(f"Rotation attempt {attempt} failed. Retrying...")
                    time.sleep(1)  # Small delay before retrying

                if not success:
                    logger.warning("Rotation command failed after multiple attempts. Executing fallback maneuver.")
                    #controller.drone.move_back(50)  # OG: 50. Move back 30cm instead of 20cm for more clearance
                    recovery()
                    # Secondary attempt to turn using a smaller angle as an alternative
                    #time.sleep(1)
                    #response = controller.drone.send_command_with_return('cw 90', timeout=5)
                    #if response == "ok":
                        #logger.info("Alternative 90-degree rotation successful.")
                    #else:
                        #logger.error("Both rotation attempts failed. Consider manual intervention or emergency stop.")
        
        elif tof_dist <= 500:   # Head-on condition
            if not params.NO_FLY:
                controller.drone.rotate_clockwise(180)
            cv2.putText(display_frame, "Avoiding Obstacle", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            logger.info("Avoiding Obstacle Ahead")

        else:   # Nothing doing - go forward
            controller.drone.send_rc_control(0, controller.move_speed, 0, 0)
            cv2.putText(display_frame, "Moving Forward", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            logger.info("Moving Forward")

    return display_frame

def draw_pose_axes_danger(controller:DroneController, display_frame):
    # TBC 26 Feb - can reuse draw_pose_axes?
    if controller.nearest_danger_id is not None:
        controller.danger_marker_distance = controller.shortest_danger_distance
        logger.info(f"Danger marker {controller.nearest_danger_id} detected. Distance to Marker {controller.markernum_lockedon}: {controller.shortest_danger_distance:.0f}cm. Offset (cm) = {controller.danger_offset}")

        # Compute marker center
        marker_center = tuple(map(int, np.mean(controller.nearest_danger_data["corners"][0], axis=0)))

        # Draw marker annotations
        cv2.circle(display_frame, marker_center, 10, (0, 0, 255), -1)  # Red dot for danger marker

        cv2.polylines(display_frame, 
                    [controller.nearest_danger_data["corners"].astype(np.int32)], 
                    True, (0, 0, 255), 2)  # Red bounding box

        cv2.putText(display_frame, 
                    f"ID: {controller.nearest_danger_id} Dist to {controller.valid_marker_info['id']}: {controller.shortest_danger_distance:.0f}cm, Offset (cm): {controller.danger_offset}", 
                    (marker_center[0], marker_center[1] - 20),  
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # Draw coordinate axes for the danger marker
        cv2.drawFrameAxes(display_frame, CAMERA_MATRIX, DIST_COEFF, 
                        controller.nearest_danger_data["rvecs"], 
                        controller.nearest_danger_data["tvecs"], 
                        10)
        
# Global variables (or add them to your controller if preferred)
hover_mode = False
last_error_time = 0
error_timeout = 0.2  # seconds without an error to exit hover mode

def navigation_thread(controller:DroneController):
    """Main navigation thread combining depth mapping and marker detection"""
    global hover_mode, last_error_time
    logger.info("Starting navigation with depth mapping...")
    
    # Create single window for combined view (COMMENTED OUT 25 FEB - video thread implemented in dronecontroller)
    # cv2.namedWindow(f"Drone {controller.drone_id} Navigation", cv2.WINDOW_NORMAL)
    
    # # Ensure frame reader is initialized
    # frame_reader = controller.drone.get_frame_read()
    # logger.info("Initializing Camera...")
    # time.sleep(2)  # Give time for camera to initialize
    
    # Initial movement
    logger.info("Moving to searching altitude...")
    if not params.NO_FLY:
        with controller.forward_tof_lock:
            controller.drone.go_to_height(params.FLIGHT_HEIGHT_SEARCH)
            # controller.drone.move_up(20)
        time.sleep(1)
    
    # Approach sequence state
    approach_complete = False
    centering_complete = False
    centering_threshold = 15    # in px
    start_time = 0      # to calculate refresh rate
    
    while controller.is_running:  # Main loop continues until marker found or battery low
        #run the stream here
        #try the hover here
        current_time = time.time()
        # Check if we're in hover mode
        if hover_mode:
            # Continue sending hover command until error_timeout expires.
            if current_time - last_error_time < error_timeout:
                controller.drone.send_rc_control(0, 0, 0, 0)
                logger.debug("Hover mode active: sustaining hover command.")
                time.sleep(0.1)  # short sleep before next check
                continue
            else:
                # Clear hover mode after no new error for error_timeout seconds.
                hover_mode = False
                logger.info("Error condition cleared. Resuming normal navigation.")
  
        try:
            next_time = time.time()
            if start_time != 0:
                refresh_rate = 1/(next_time-start_time)
                logger.debug(f"navigation_thread refresh rate (Hz): {refresh_rate:.1f}")     # 25 Feb improvement from 1-2 Hz / 4 Hz, to 2.5 Hz / 4 Hz
            start_time = next_time
            
            frame = controller.get_current_frame()
            display_frame = frame.copy()
            
            # Get depth color map
            depth_colormap = controller.generate_color_depth_map(frame) # TBC 6 Feb can shift under "else" since no need to generate when markers found (10 Feb Ans: Not if you want to visualize)
            controller.process_depth_color_map(depth_colormap)
            
            # Get ToF distance
            # tof_dist = controller.forward_tof_dist
            tof_dist = controller.get_tof_distance()        # TESTING TBC 12 MAR
            logger.debug(f"ToF Dist: {tof_dist}")
            cv2.putText(display_frame, f"ToF: {tof_dist}mm", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            marker_found = False
            controller.nearest_danger_id = None
            
            # Check for markers
            marker_found, corners, marker_id, rvecs, tvecs = controller.detect_markers(frame, display_frame)   # detects all markers; returns details of ONE valid (and land-able) marker, approved by the server
            if marker_found:  
                # Draw marker detection and pose information on the ONE detected valid marker
                display_frame = draw_pose_axes(display_frame, corners, [marker_id], rvecs, tvecs)
                logger.info(f"Obtained Marker Status from Server: {controller.marker_client.marker_status}")
                logger.debug(f"Marker detected: {marker_id}. Available: {controller.marker_client.is_marker_available(marker_id)}. Currently locked on: {controller.markernum_lockedon}")

                # PART 1: DETECTION AND LOCK-ON LOGIC
                goto_approach_sequence:bool = check_marker_server_and_lockon(controller, marker_id, display_frame)
                
                # PART 2: CENTERING AND APPROACH LOGIC
                # 26 Feb: Generally works, but reliability can be improved. E.g. Final forward XX may be executed but not acknowledged, causing it to go again.
                with controller.forward_tof_lock:      
                    if goto_approach_sequence is True and not params.NO_FLY:
                        # PART 2A: CENTERING
                        #include apporach obstacle avoidance here
                        logger.info("Middle center split (left) - Non-blue pixels: %d, Blue pixels: %d",
                                    controller.depth_map_colors["middle_center_split"]["left"]["nonblue"],
                                    controller.depth_map_colors["middle_center_split"]["left"]["blue"])

                        logger.info("Middle center split (right) - Non-blue pixels: %d, Blue pixels: %d",
                                    controller.depth_map_colors["middle_center_split"]["right"]["nonblue"],
                                    controller.depth_map_colors["middle_center_split"]["right"]["blue"])
                        
                        if controller.depth_map_colors["middle_center_split"]["left"]["nonblue"] - 100 > controller.depth_map_colors["middle_center_split"]["left"]["blue"]:
                            #controller.drone.move_right(20)
                            cv2.putText(display_frame, "Moving Right", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                            controller.drone.send_rc_control((controller.move_speed/2), 0, 0, 0)
                            logger.info(f"Uh oh Pillar is blocking the approach, moving right!")
                        elif controller.depth_map_colors["middle_center_split"]["right"]["nonblue"] - 100 > controller.depth_map_colors["middle_center_split"]["right"]["blue"]:
                            #controller.drone.move_left(20)
                            cv2.putText(display_frame, "Moving Left", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                            controller.drone.send_rc_control((-controller.move_speed/2), 0, 0, 0) 
                            logger.info(f"Uh oh Pillar is blocking the approach, moving left!")
                        else:
                            logger.info(f"NVM ALL IS GOOD CAN GO")
                            
                        logger.info(f"Locked on: {controller.markernum_lockedon}. Centering/approaching...")
                        controller.marker_client.send_update('status', f"Locked on: {controller.markernum_lockedon}. Centering/approaching...")
                        
                        # Center on the marker
                        frame_center = frame.shape[1] / 2
                        marker_center = np.mean(corners[0], axis=0)
                        x_error = marker_center[0] - frame_center
                        
                        if not centering_complete:
                            if abs(x_error) > centering_threshold:
                                # Calculate yaw speed based on error
                                    yaw_speed = int(np.clip(x_error / 4, -20, 20))  # OG: / 10
                                    controller.drone.send_rc_control(0, 0, 0, yaw_speed)
                                    logger.info(f"Centering: error = {x_error:.1f}, yaw_speed = {yaw_speed}")
                                # if abs(x_error) < 25:
                                #   logger.info(f"CENTERED MOVE BOX")  
                            else:
                                logger.info(f"Marker centered, x error = {x_error:.0f}px! Starting approach...")
                                controller.drone.send_rc_control(0, 0, 0, 0)  # Stop rotation
                                time.sleep(0.5)  # Stabilize
                                centering_complete = True

                        # PART 2B: APPROACH (I.E. CENTERING COMPLETE)
                        elif not approach_complete:
                            current_distance_3D = controller.get_distance()
                            if current_distance_3D is None:
                                logger.info("Lost marker during approach...")  # should not reach here! caa 13 Feb
                                time.sleep(0.1)
                                continue

                            current_height = controller.drone.get_height() - params.EXTRA_HEIGHT
                            current_distance_2D = np.sqrt(current_distance_3D**2 - current_height**2)

                            logger.info(f"3D distance to marker: {current_distance_3D:.1f}cm \n Drone height: {current_height:.1f}cm \n 2D distance to marker: {current_distance_2D:.1f}cm")
                            cv2.putText(display_frame, "Centered. Approaching...", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 3, (255, 255, 255), 3)

                            if current_distance_2D >= 500:
                                step_dist:int = 100
                                logger.info(f"{current_distance_2D:.2f}cm distance too large. Stepping forward {step_dist}cm to approach marker...")
                                controller.drone.move_forward(step_dist)
                                time.sleep(0.5)  # Wait for movement to complete
                                centering_complete = False
                                continue        # re-enter the loop; need to re-detect marker and re-measure distance. 

                            elif current_distance_2D > 0 and current_distance_2D < 500:
                                logger.info(f"Final Approach: Moving forward {int(current_distance_2D)}cm to marker.")
                                controller.drone.move_forward(int(current_distance_2D))
                                logger.info("Approach complete!")
                                controller.drone.send_rc_control(0, 0, 0, 0)
                                # 26 FEB TODO: ADD DOWNWARD FACING / DANGER AVOIDING / VICTIM CENTERING HERE

                                if controller.danger_offset and controller.shortest_danger_distance < 120:
                                    dx, dy, dz = controller.danger_offset  # Danger offset in (x, y, z)
                                    logger.info(f"Offset measured: x: {dx}, z: {dz} (Victim - Danger, in cv2 coordinates)")
                                    ## NOTE: TELLO XYZ is: x +ve forward, y +ve left, z +ve up
                                    ## NOTE: CV2 XYZ is x +ve right, y +ve down, z +ve forward

                                    # Compute Euclidean distance from danger point
                                    danger_distance = np.sqrt(dx**2 + dz**2)
                                    
                                    if danger_distance > 0:  # Avoid division by zero
                                        scale_factor = 50 / danger_distance  # Inverse relationship ensures smaller danger distance results in larger scale factor
                                        logging.info(f"Movement before clipping: x={dz * scale_factor:.1f} cm, y={-dx * scale_factor} cm")
                                        tello_offset_x = np.clip(dz * scale_factor, -75, 75)  # Move away from danger in cv2's z = Tello's x
                                        tello_offset_y = np.clip(-dx * scale_factor, -75, 75)  # Move away from danger in cv2's x = Tello's -y
                                        
                                    else:
                                        tello_offset_x, tello_offset_y = 0, 0  # No movement if danger distance is 0

                                    logging.info(f"Moving drone by offset: x={tello_offset_x:.1f} cm, y={tello_offset_y:.1f} cm")

                                    # Move the drone safely using go_xyz_speed
                                    controller.drone.go_xyz_speed(int(tello_offset_x), int(tello_offset_y), 0, 20)  # y=0 since we're on the floor
                                    time.sleep(2)  # Wait for the movement to complete

                                approach_complete = True
                                controller.marker_client.send_update('marker', marker_id=marker_id, landed=True)
                                time.sleep(1)
                                controller.is_running = False #added this in cause it wouldn't land! --> ask gabriel
                                break

                    elif goto_approach_sequence is True and params.NO_FLY == True: # if simulating
                        logger.info("Marker found, but not landing since not flying. Program continues.")

                    else:   #goto_approach_sequence is False
                        logger.debug("Marker found, but not approaching yet.")
                
            elif controller.exit_detected: # This condition is placed after marker_detected but before depth mapping 
                exit_text:str = f"Exit detected {controller.exit_distance_3D:.0f}cm away."
                logger.info("No valid markers detected. " + exit_text)
                cv2.putText(display_frame, exit_text, (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                if not params.NO_FLY and controller.exit_distance_3D < 300:
                    logger.info(f"Avoiding exit. Turning 90 degrees.")
                    with controller.forward_tof_lock: 
                        controller.drone.rotate_clockwise(90)

                elif not params.NO_FLY:
                    logger.info(f"Exit more than 3m away, no action taken.")
                    display_frame = nav_with_depthmap_tof(controller, tof_dist, display_frame)      # logic for depth map and ToF

            else: # Navigation logic using depth map if neither victim nor exit detected. Simulates well without drone
                # NOTE 4 Feb: Check ToF after depth map should enable it to enter tighter spaces. To be more conservative, can consider checking ToF before depth map.)
                
                logger.debug(f"Nothing detected.")
                
                if not controller.markernum_lockedon is None:        # Publish that its lost track of target. Then resets its locked_on number.
                    logger.debug(f"Resetting markernum_lockedon from {controller.markernum_lockedon} to None")
                    controller.marker_client.send_update('marker', marker_id=controller.markernum_lockedon, detected=False)
                    controller.markernum_lockedon = None
                
                with controller.forward_tof_lock:
                    logger.debug(f"Executing nav_with_depthmap_tof.") 
                    display_frame = nav_with_depthmap_tof(controller, tof_dist, display_frame)      # logic for depth map and ToF
            
            if controller.nearest_danger_id is not None:
                draw_pose_axes_danger(controller, display_frame)

            # Resize depth_colormap to match frame dimensions, create combined view side-by-side
            depth_colormap_resized = cv2.resize(depth_colormap, (display_frame.shape[1]//2, display_frame.shape[0]))
            if not params.LAPTOP_ONLY:
                display_frame = cv2.cvtColor(display_frame, cv2.COLOR_BGR2RGB)
            combined_view = np.hstack((display_frame, depth_colormap_resized))
            
            # Add labels and display combined view
            cv2.putText(combined_view, "Live Feed", (10, combined_view.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(combined_view, "Depth Map", (display_frame.shape[1] + 10, combined_view.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            controller.set_display_frame(combined_view)

            #cv2.imshow(f"Drone {controller.drone_id} Navigation", combined_view)      # 26 FEB DO NOT SHOW - already displaying in dronecontroller
                
        except Exception as e:
            logger.error(f"Error in navigation: {e}. Continuing navigation.")      # 13 Feb: "ERROR - libav.h264 - error while decoding MB 57 29, bytestream -10" does not come here
            # Don't cleanup or break - continue searching
            continue           

class HoverOnErrorHandler(logging.Handler):
    def __init__(self, drone, error_keywords, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.drone = drone
        self.error_keywords = error_keywords
        self.last_hover_time = 0  # to throttle hover commands if needed

    # def emit(self, record):
    #     message = record.getMessage()
    #     # Check if the error message contains any of the keywords
    #     if any(keyword in message for keyword in self.error_keywords):
    #         # Optional: Throttle the hover command if it was just issued
    #         current_time = time.time()
    #         if current_time - self.last_hover_time > 5:  # 5-second gap, for example
    #             print("Error detected in video stream. Triggering hover command.")
    #             logging.debug("HAHAHAHA IT WORKED NO MORE GABRIEL")
    #             self.drone.send_rc_control(0, 0, 0, 0)
    #             self.last_hover_time = current_time
                
    def emit(self, record):
        global hover_mode, last_error_time
        message = record.getMessage()
        if any(keyword in message for keyword in self.error_keywords):
            current_time = time.time()
            last_error_time = current_time  # update on error
            if not hover_mode:
                print("HAHAHAver mode activated")
            hover_mode = True
            # # Optionally send an immediate hover command:     # TBC 12 MAR  - TO VERIFY
            # self.drone.send_rc_control(0, 0, 0, 0)          

def display_loop(controller:DroneController):
    window_name = f"Drone View {controller.drone_id}"
    # Explicitly create a named window on the main thread
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    while controller.is_running:
        combined_view = controller.get_display_frame()
        if combined_view is not None:
            cv2.imshow(window_name, combined_view)
        else:
            # Debug message if no frame is available
            logging.debug("Display loop: No frame available yet.")
        if cv2.waitKey(1) & 0xFF == ord('q'):
            controller.is_running = False
            break
        time.sleep(0.03)  # Delay to reduce CPU usage
    cv2.destroyAllWindows()

def custom_tof_navigation_gab(controller: DroneController) -> None:
    """
    NEW 12 MAR: Use ToF to help guide. Start pos - Facing straight ahead (North)
    Ideally, drone's execute_waypoints should end with it facing North, a good distance away from the walls.

    Args:
        controller: An instance of DroneController with ToF and movement capabilities.
    """
    # Constants for movement and rotation
    MOVE_INCREMENT = 50  # Distance to move in cm
    LIST_LENGTH = 3
    MAX_MOVEMENT_COUNT = 6

    logging.debug(f"Step 0/4: Initial ToF readings (North): {controller.get_tof_distances_list(list_length=LIST_LENGTH)}")

    # Step 1: Rotate to face West
    with controller.forward_tof_lock:
        controller.drone.rotate_counter_clockwise(90)
    time.sleep(1)  # Allow time for rotation to complete

    # Step 2: Check ToF readings and move right until path is clear
    tof_dist_list = controller.get_tof_distances_list(list_length=LIST_LENGTH)
    logging.debug(f"Step 2a/4: Initial ToF readings (West): {tof_dist_list}")

    clear_count = 0
    movement_count = 0
    while clear_count < 2:  # Ensure drone checks clear, moves forward, and checks again
        while not controller.tof_check_clear(tof_dist_list) and movement_count < MAX_MOVEMENT_COUNT:
            with controller.forward_tof_lock:
                controller.drone.move_right(MOVE_INCREMENT)
            movement_count += 1
            tof_dist_list = controller.get_tof_distances_list(list_length=LIST_LENGTH)
            logging.debug(f"Step 2b/4: Moving right while facing West; Count: {movement_count}/{MAX_MOVEMENT_COUNT}, ToF readings: {tof_dist_list}")

        with controller.forward_tof_lock:
            controller.drone.move_forward(30)
        tof_dist_list = controller.get_tof_distances_list(list_length=LIST_LENGTH)
        if controller.tof_check_clear(tof_dist_list):
            clear_count += 1
        else:
            clear_count -= 1
        logging.debug(f"Step 2c/4: Moved forward 20cm, clear_count = {clear_count}/2, ToF readings: {tof_dist_list}")

    with controller.forward_tof_lock:
        controller.drone.move_right(50) #   for additional clearance from the wall edge
        controller.drone.rotate_counter_clockwise(90)
        controller.drone.move_right(300)
    logging.debug("Drone now facing south.")

    tof_dist_list = controller.get_tof_distances_list(list_length=LIST_LENGTH)
    logging.debug(f"Step 3a/4: Initial ToF readings (South): {tof_dist_list}")

    clear_count = 0
    movement_count = 0
    while clear_count < 1:  # Ensure drone checks clear, moves forward, and checks again
        while not controller.tof_check_clear(tof_dist_list) and movement_count < MAX_MOVEMENT_COUNT:
            with controller.forward_tof_lock:
                controller.drone.move_right(MOVE_INCREMENT)
            movement_count += 1
            tof_dist_list = controller.get_tof_distances_list(list_length=LIST_LENGTH)
            logging.debug(f"Step 3b/4: Moving right while facing South; Count: {movement_count}/{MAX_MOVEMENT_COUNT}, ToF readings: {tof_dist_list}")

        with controller.forward_tof_lock:
            controller.drone.move_forward(30)
        tof_dist_list = controller.get_tof_distances_list(list_length=LIST_LENGTH)
        if controller.tof_check_clear(tof_dist_list):
            clear_count += 1
        else:
            clear_count -= 1
        logging.debug(f"Step 3c/4: Moved forward 30cm, clear_count = {clear_count}/2, ToF readings: {tof_dist_list}")

    controller.drone.move_right(50) # For clearance

    # Step 5: Log completion
    logging.info("Step 4/4: custom_tof_navigation completed! Drone is ready to enter back entrance.")

def custom_tof_navigation_yz(controller: DroneController) -> None:
    """
    NEW 12 MAR: Use ToF to help guide. Start pos - Facing straight ahead (North)
    Ideally, drone's execute_waypoints should end with it facing North, a good distance away from the walls.

    Args:
        controller: An instance of DroneController with ToF and movement capabilities.
    """
    # Constants for movement and rotation
    MOVE_INCREMENT = 50  # Distance to move in cm
    LIST_LENGTH = 3
    MAX_MOVEMENT_COUNT = 3

    # Step 1: Rotate to face West
    with controller.forward_tof_lock:
        controller.drone.rotate_counter_clockwise(90)
    time.sleep(1)  # Allow time for rotation to complete

    # Step 2: Check ToF readings and move right until path is clear
    tof_dist_list = controller.get_tof_distances_list(list_length=LIST_LENGTH)
    logging.debug(f"Initial ToF readings (West): {tof_dist_list}")

    clear_count = 0
    movement_count = 0
    while clear_count < 2:  # Ensure drone checks clear, moves forward, and checks again
        while not controller.tof_check_clear(tof_dist_list) and movement_count < MAX_MOVEMENT_COUNT:
            with controller.forward_tof_lock:
                controller.drone.move_right(MOVE_INCREMENT)
            movement_count += 1
            tof_dist_list = controller.get_tof_distances_list(list_length=LIST_LENGTH)
            logging.debug(f"Moving right while facing West; Count: {movement_count}/{MAX_MOVEMENT_COUNT}, ToF readings: {tof_dist_list}")

        with controller.forward_tof_lock:
            controller.drone.move_forward(30)
        tof_dist_list = controller.get_tof_distances_list(list_length=LIST_LENGTH)
        if controller.tof_check_clear(tof_dist_list):
            clear_count += 1
        else:
            clear_count -= 1
        logging.debug(f"Step 2c/4: Moved forward 20cm, clear_count = {clear_count}/2, ToF readings: {tof_dist_list}")

    controller.drone.move_right(50) # For clearance

    # Step 5: Log completion
    logging.info("Step 4/4: custom_tof_navigation_yz completed! Drone is facing West and ready to enter front entrance.")

def main():
    global DIST_COEFF, logger
    logger = setup_logging(params, "UnknownArea.main")
    logger.info(f"Starting unknown area main with drone_id: {params.PI_ID}")
    controller = DroneController(params.NETWORK_CONFIG, drone_id=params.PI_ID,
                                 laptop_only=params.LAPTOP_ONLY, load_midas=True)
    
    # Add custom error handler for video errors
    error_keywords = ["libav.h264", "no frame!", "non-existing PPS", "decode_slice_header error", "left block unavailable", "error while decoding"]
    hover_handler = HoverOnErrorHandler(controller.drone, error_keywords)
    logger.addHandler(hover_handler)
    # Get the libav.h264 logger and attach the hover handler so it captures errors from that source ---> 8 march try and stop the libhav errors
    libav_logger = logging.getLogger("libav.h264")
    libav_logger.setLevel(logging.DEBUG)   # adjust as needed ---> change from error to debug??? idk need to check
    libav_logger.propagate = True
    libav_logger.addHandler(hover_handler)
    
    try:
        with controller.forward_tof_lock:    
            controller.marker_client.client_takeoff_simul([99], status_message=f'Waiting for takeoff. {controller.drone.get_battery()}%')    # just holds the drone until released. still needs takeoff() in the next line 
            init_yaw = controller.drone.get_yaw()

            # After takeoff triggered, chill on the ground for a specified delay (for 2nd takeoff)
            if params.PRE_TAKEOFF_DELAY > 0:
                msg = f"Takeoff triggered. Starting countdown: {params.PRE_TAKEOFF_DELAY}s"
                logging.info(msg)
                controller.marker_client.send_update('status', status_message=msg)
                time.sleep(params.PRE_TAKEOFF_DELAY)
            
            if not params.NO_FLY:
                controller.drone.takeoff()
                logging.info("Taking off for real...")
            else:
                logging.info("Simulating taking off for real...")

            controller.marker_client.send_update('status', status_message=f'Taking off. {controller.drone.get_battery()}%')
            controller.drone.send_rc_control(0, 0, 0, 0)
            time.sleep(1)
            post_yaw = controller.drone.get_yaw()
            yaw_back = post_yaw - init_yaw
            logging.debug(f"Init yaw: {init_yaw}, Post yaw: {post_yaw}, Normalize: {normalize_angle(yaw_back)}")

            # After taking off for real, hover for a specified delay
            if params.TAKEOFF_HOVER_DELAY > 0:
                logging.info(f"Starting {params.TAKEOFF_HOVER_DELAY}s hover")
                time.sleep(params.TAKEOFF_HOVER_DELAY)        # Hover in position after takeoff

            # Go to specified FLIGHT_HEIGHT_WAYPOINTS and execute waypoints at that flight level
            if not params.NO_FLY:
                try:
                    logging.info(f"Going to initial flight height {params.FLIGHT_HEIGHT_WAYPOINTS}")  
                    controller.drone.go_to_height(params.FLIGHT_HEIGHT_WAYPOINTS)
                    logging.info(f"Executing waypoints {params.WAYPOINTS_JSON}")
                    execute_waypoints(params.WAYPOINTS_JSON, controller.drone, params.NO_FLY)
                except Exception as e:
                    logging.error(f"Error: {e}. DID NOT COMPLETE WAYPOINTS. Continuing on...")       # can just comment out WAYPOINTS_JSON in params
            else:
                try:
                    logging.info(f"Simulating executing waypoints {params.WAYPOINTS_JSON}")
                except Exception as e:
                    logging.error(f"Error: {e}. DID NOT SIMULATE WAYPOINTS. Continuing on...")
        
        controller.start_tof_thread()
        custom_tof_navigation(controller)       # NEW 13 MAR - TESTING
        with controller.forward_tof_lock:
            controller.drone.move_forward(30)   # to ensure drone is inside Unknown Area, past the Reverse marker
        
        # Setup video stream (this starts the _stream_video thread which only updates frames)
        controller.setup_stream()
        
        # Start the navigation logic in a separate thread
        nav_thread = threading.Thread(target=navigation_thread, args=(controller,))
        nav_thread.start()
        
        # Start the display loop in the main thread
        display_loop(controller)
        
        nav_thread.join()
        
    except KeyboardInterrupt:
        logging.info("Keyboard interrupt received.")
    
    except Exception as e:
        logging.error(f"Error in main: {e}. Landing.")

    finally:
        controller.marker_client.send_update('status', status_message='Landing')
        end_batt = controller.drone.get_battery()
        logger.info(f"Actually landing for real. End Battery Level: {end_batt}%")
        with controller.forward_tof_lock:
            controller.drone.end()
        controller.marker_client.send_update('status', status_message=f'Landed. {end_batt}%')
        controller.is_running = False
        controller.stop_event.set()
        # controller.shutdown()
        cv2.destroyAllWindows()
        cv2.waitKey(1)

if __name__ == "__main__":
    main()
