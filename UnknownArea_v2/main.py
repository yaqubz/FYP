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

from shared_utils.dronecontroller2 import DroneController     # WORKS WELL 1 MAR
from shared_utils.shared_utils import *

import cv2
import numpy as np
import time

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
    if tof_dist == 8888: # ToF error (while loop)
        controller.drone.send_rc_control(0, 0, 0,0)
        cv2.putText(display_frame, "Pausing, ToF error", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        logging.warning("Pausing, ToF error")
        return display_frame

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
        if controller.depth_map_colors["blue"]["center"] > controller.depth_map_colors["red"]["center"] and tof_dist <= 800: #OG 700
            cv2.putText(display_frame, "Avoiding Corner", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            logging.info("Avoiding Corner")

            # TBC TODO 26 FEB - ALWAYS AVOIDING CORNER

            if not params.NO_FLY:
                time.sleep(0.5) 
                success = False
                retries = 3  # Increased retries for reliability

                for attempt in range(1, retries + 1):
                    response = controller.drone.send_command_with_return('cw 150', timeout=3)  # Attempt rotation (OG: 135)
                    if response == "ok":
                        success = True
                        logging.info(f"Rotation successful on attempt {attempt}.")
                        break
                    else:
                        logging.warning(f"Rotation attempt {attempt} failed. Retrying...")
                    time.sleep(1)  # Small delay before retrying

                if not success:
                    logging.warning("Rotation command failed after multiple attempts. Executing recovery maneuver.")
                    with controller.forward_tof_lock:
                        execute_recovery(controller)        # YAQUB NEW 1 MAR 

                    # # Secondary attempt to turn using a smaller angle as an alternative
                    # time.sleep(1)
                    # response = controller.drone.send_command_with_return('cw 90', timeout=5)
                    # if response == "ok":
                    #     logging.info("Alternative 90-degree rotation successful.")
                    # else:
                    #     logging.error("Both rotation attempts failed. Consider manual intervention or emergency stop.")
        
        elif tof_dist <= 500:   # Head-on condition
            # if not params.NO_FLY:
            #     controller.drone.rotate_clockwise(180)
            controller.drone.rotate_clockwise(180)
            cv2.putText(display_frame, "Avoiding Obstacle", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            logging.info("Avoiding Obstacle Ahead")

        else:   # Nothing doing - go forward
            controller.drone.send_rc_control(0, controller.move_speed, 0, 0)
            cv2.putText(display_frame, "Moving Forward", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            logging.info("Moving Forward")

    return display_frame

def draw_pose_axes_danger(controller:DroneController, display_frame):
    # TBC 26 Feb - can reuse draw_pose_axes?
    if controller.nearest_danger_id is not None:
        controller.danger_marker_distance = controller.shortest_danger_distance
        logging.info(f"Danger marker {controller.nearest_danger_id} detected. Distance to Marker {controller.markernum_lockedon}: {controller.shortest_danger_distance:.0f}cm. Offset (cm) = {controller.danger_offset}")

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
        cv2.drawFrameAxes(display_frame, params.CAMERA_MATRIX, params.DIST_COEFF, 
                        controller.nearest_danger_data["rvecs"], 
                        controller.nearest_danger_data["tvecs"], 
                        10)
        
def execute_recovery(controller = DroneController):
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
            ("clockwise", controller.drone.rotate_clockwise, 135),
            ("counter-clockwise", controller.drone.rotate_counter_clockwise, 135)
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
        with controller.forward_tof_lock:
            # controller.drone.go_to_height_PID(params.FLIGHT_HEIGHT) # COMMENTED OUT 27 FEB - waste time?
            controller.drone.go_to_height(params.FLIGHT_HEIGHT) # COMMENTED OUT 27 FEB - waste time?
            # controller.drone.move_up(20)
            time.sleep(2)
        
    
    # Approach sequence state
    approach_complete = False
    centering_complete = False
    centering_threshold = 10    # in px OG: 30 -> 15 
    start_time = 0      # to calculate refresh rate
    
    while controller.is_running:  # Main loop continues until marker found or battery low
                
        try:
            start_time = log_refresh_rate(start_time, 'navigation_thread')
            frame = controller.get_current_frame()
            display_frame = frame.copy()

            controller.depth_colormap = controller.generate_color_depth_map(frame)
            controller.process_depth_color_map(controller.depth_colormap)
            
            # Get latest ToF distance from the thread 
            tof_dist = controller.forward_tof_dist
            logging.info(f"ToF Dist: {tof_dist}")
            cv2.putText(display_frame, f"ToF: {tof_dist}mm", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            marker_found = False
            controller.nearest_danger_id = None

            if controller.empty_frame_received:     # TBC Testing - even with video thread, should probably pause the navigation whenever empty frame is received
                controller.drone.send_rc_control(0,0,0,0)
                logging.info("Empty frame received. Pausing navigation_thread with rc(0,0,0,0)")    # TBC IF EVER TRIGGERED
                time.sleep(0.5)
                continue
            
            # Check for markers
            marker_found, corners, marker_id, rvecs, tvecs = controller.detect_markers(frame, display_frame)   # detects all markers; returns details of ONE valid (and land-able) marker, approved by the server
            # logging.debug(f"CHECKPOINT 2: {round((time.time() - start_time), 2)}")
            if marker_found:  
                # Draw marker detection and pose information on the ONE detected valid marker
                display_frame = draw_pose_axes(display_frame, corners, [marker_id], rvecs, tvecs)
                logging.info(f"Obtained Marker Status from Server: {controller.marker_client.marker_status}")
                logging.debug(f"Marker detected: {marker_id}. Available: {controller.marker_client.is_marker_available(marker_id)}. Currently locked on: {controller.markernum_lockedon}")

                # PART 1: DETECTION AND LOCK-ON LOGIC
                goto_approach_sequence:bool = check_marker_server_and_lockon(controller, marker_id, display_frame)

                # logging.debug(f"CHECKPOINT 3: {round((time.time() - start_time), 2)}")
                
                # PART 2: CENTERING AND APPROACH LOGIC
                # 26 Feb: Generally works, but reliability can be improved. E.g. Final forward XX may be executed but not acknowledged, causing it to go again.
                with controller.forward_tof_lock:      
                    if goto_approach_sequence is True and not params.NO_FLY:
                        # PART 2A: CENTERING
                        logging.info(f"Locked on: {controller.markernum_lockedon}. Centering/approaching...")
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
                                logging.info(f"Centering: error = {x_error:.1f}, yaw_speed = {yaw_speed}")
                            else:
                                logging.info(f"Marker centered, x error = {x_error:.0f}px! Starting approach...")
                                controller.drone.send_rc_control(0, 0, 0, 0)  # Stop rotation
                                time.sleep(0.5)  # Stabilize
                                centering_complete = True
                        
                        # PART 2B: APPROACH (I.E. CENTERING COMPLETE)
                        elif not approach_complete:
                            current_distance_3D = controller.get_distance()
                            if current_distance_3D is None:
                                logging.info("Lost marker during approach...")  # should not reach here! caa 13 Feb
                                time.sleep(0.1)
                                continue

                            current_height = controller.drone.get_height() - params.EXTRA_HEIGHT
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
                                # 26 FEB TODO: ADD DOWNWARD FACING / DANGER AVOIDING / VICTIM CENTERING HERE

                                if controller.danger_offset and controller.shortest_danger_distance < 120:
                                    dx, dy, dz = controller.danger_offset  # Danger offset in (x, y, z)
                                    logging.info(f"Offset measured: x: {dx}, z: {dz} (Victim - Danger, in cv2 coordinates)")
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
                    with controller.forward_tof_lock: 
                        controller.drone.rotate_clockwise(90)

                elif not params.NO_FLY:
                    logging.info(f"Exit more than 3m away, no action taken. Executing nav_with_depthmap_tof.") 
                    display_frame = nav_with_depthmap_tof(controller, tof_dist, display_frame)      # logic for depth map and ToF

            else: # Navigation logic using depth map if neither victim nor exit detected. Simulates well without drone
                # NOTE 4 Feb: Check ToF after depth map should enable it to enter tighter spaces. To be more conservative, can consider checking ToF before depth map.)
                
                logging.debug(f"Nothing detected.")
                
                if not controller.markernum_lockedon is None:        # Publish that its lost track of target. Then resets its locked_on number.
                    logging.debug(f"Resetting markernum_lockedon from {controller.markernum_lockedon} to None")
                    controller.marker_client.send_update('marker', marker_id=controller.markernum_lockedon, detected=False)
                    controller.markernum_lockedon = None
                
                with controller.forward_tof_lock:
                    logging.debug(f"Executing nav_with_depthmap_tof.") 
                    display_frame = nav_with_depthmap_tof(controller, tof_dist, display_frame)      # logic for depth map and ToF
            if controller.nearest_danger_id is not None:
                draw_pose_axes_danger(controller, display_frame)

            # Resize depth_colormap to match frame dimensions, create combined view side-by-side
            depth_colormap_resized = cv2.resize(controller.depth_colormap, (display_frame.shape[1]//2, display_frame.shape[0]))
            if not params.LAPTOP_ONLY:
                display_frame = cv2.cvtColor(display_frame, cv2.COLOR_BGR2RGB)
            combined_view = np.hstack((display_frame, depth_colormap_resized))
            
            # Add labels and display combined view
            cv2.putText(combined_view, "Live Feed", (10, combined_view.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(combined_view, "Depth Map", (display_frame.shape[1] + 10, combined_view.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            controller.set_display_frame(combined_view)

            # cv2.imshow(f"Drone {controller.drone_id} Navigation", combined_view)      # 26 FEB DO NOT SHOW - already displaying in dronecontroller
                
        except Exception as e:
            logging.error(f"Error in navigation: {e}. Continuing navigation.")      # 13 Feb: "ERROR - libav.h264 - error while decoding MB 57 29, bytestream -10" does not come here
            # Don't cleanup or break - continue searching
            continue           

def main():     
    global DIST_COEFF, logger
   
    # Setup logging
    logger = setup_logging(params, "UnknownArea.main")      # local logger - TBC 2 MAR cannot stop duplicates 
    logger.info(f"Starting unknown area main with drone_id: {params.PI_ID}")
    controller = DroneController(params.NETWORK_CONFIG, drone_id=params.PI_ID, laptop_only=params.LAPTOP_ONLY, load_midas=True, imshow=params.IMSHOW)
    try:
        with controller.forward_tof_lock:    
            controller.marker_client.send_update('status', status_message=f'Waiting for takeoff. {controller.drone.get_battery()}%')
            init_yaw = controller.drone.get_yaw()
            controller.takeoff_simul([11,17])  # just holds the drone until released. still needs takeoff() in the next line 
            # controller.marker_client.client_takeoff_simul([99])     # change to marker client
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
            
            time.sleep(params.TAKEOFF_HOVER_DELAY)
            if not params.NO_FLY:
                try:
                    logging.info(f"Executing waypoints {params.WAYPOINTS_JSON}")
                    execute_waypoints(params.WAYPOINTS_JSON, controller.drone, params.NO_FLY)
                except Exception as e:
                    logging.error(f"Error: {e}. DID NOT COMPLETE WAYPOINTS. Continuing on...")       # can just comment out WAYPOINTS_JSON in params
            else:
                try:
                    logging.info(f"Simulating executing waypoints {params.WAYPOINTS_JSON}")
                except Exception as e:
                    logging.error(f"Error: {e}. DID NOT SIMULATE WAYPOINTS. Continuing on...")
                
        # Only start video stream and ToF thread after completing waypoints
        controller.setup_stream()
        controller.start_video_stream()
        controller.start_tof_thread()
        time.sleep(2)

        navigation_thread(controller)

    except KeyboardInterrupt:
        logging.info("Keyboard interrupt received.")
    
    except Exception as e:
        logging.error(f"Error in main: {e}. Landing.")

    finally:
        controller.marker_client.send_update('status', status_message='Landing')
        end_batt = controller.drone.get_battery()
        logging.info(f"Actually landing for real. End Battery Level: {end_batt}%")
        with controller.forward_tof_lock:
            logging.info("Finally, shutting down controller and ending drone in main.")
            controller.shutdown()
            controller.drone.end()
        controller.marker_client.send_update('status', status_message=f'Landed. {end_batt}%')
        # controller.stop_event.set()
        # controller.shutdown()
        # cv2.destroyAllWindows()
        # cv2.waitKey(1)

if __name__ == "__main__":
    main()