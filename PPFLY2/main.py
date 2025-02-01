# 23 Jan WORKS WELL but still In Progress, Integrating UWB
# Under constants, Set SIMULATE = True/False as necessary
# IMPT: This version is only different from PPFLY in the execute_waypoints() function - here, the drone itself is an argument, which is needed to import this function in another script

"""
Typically, drone only rotates and moves forward. It will only do otherwise for error correction using UWB.

CANNOT BE RUN DIRECTLY! Only here to have execute_waypoints imported

Since this is now a module in a package, run using `python -m PPFLY2.main` in the terminal.
"""

from djitellopy import Tello

from .utils import *
from .constants import *

import json, math, time, sys, argparse
from pathlib import Path

# Add workspace root to sys.path (9 Jan: Works but might need a better solution)
workspace_root = Path(__file__).resolve().parent.parent
sys.path.append(str(workspace_root))

from UWB_ReadUDP import get_target_position # own custom library

def check_args():
    """
    To set SIMULATE flag via CLI (optional)
    Otherwise, it will use the SIMULATE option under .constants
    """
    parser = argparse.ArgumentParser(description='Execute or Simulate waypoints for Tello drone')
    parser.add_argument('-sim', '--simulate', type=int, choices=[0, 1], help="Set simulation mode: 1 for True, 0 for False")
    return parser.parse_args()

def execute_waypoints(json_filename, drone, simulate = False):
    """
    Connect, takeoff and landing should be done outside this function.
    """
    global LAND_ID, waypoints
    global start_batt, end_batt
    tello = MockTello() if simulate else drone
    DELAY = 0 if simulate else 2
    
    try:      
        # Read waypoints from file
        with open(json_filename, 'r') as f:
            data = json.load(f)

        # Initialize position and orientation
        start_pos_cm = data['wp'][0]["position_cm"]
        print("INTENDED START POS IS:", start_pos_cm)
        abs_position = {"x_cm": start_pos_cm['x'], "y_cm": start_pos_cm['y']}  # in cm; FOR DEAD RECKONING, updated using save_pos()
        orientation = START_HEADING  # Starting heading in degrees (assuming 180 as the initial heading)


        # At StartPos, Take and record first UWB Measurement. IMPT: For now, these 3 are always done together.
        save_pos(waypoints, orientations, abs_position, orientation, 0)
        lastpos_cm = [int(round(coord*100,0)) for coord in get_target_position(UWBTAG_ID)]
        save_pos_UWB(waypoints_UWB, orientations_UWB, lastpos_cm[0:2])
        save_errors(pos_error_list, orientations_error_list, waypoints[-1], waypoints_UWB[-1], orientation, obtain_orientation(waypoints_UWB))
        print(f"WAYPOINTS UWB: {waypoints_UWB}")
        # printdistance(waypoints_UWB[-2], waypoints_UWB[-1]) # HELLOIMPT: THIS SHOULD HAVE INDEX ERROR (IDK HOW TO CATCH ERROR P) 23 Jan: TBC but code still can run even with error?

        # Execute each waypoint
        for wp_index, wp in enumerate(data['wp']):
            if wp['angle_deg'] != 0:
                # To save the current orientation without executing
                orientation -= wp['angle_deg']  
                if orientation > 180:
                    orientation -= 360
                elif orientation < -180:
                    orientation += 360
            
                # To execute the command
                if wp['angle_deg'] < 0:
                    tello.rotate_clockwise(int(abs(wp['angle_deg'])))
                else:
                    tello.rotate_counter_clockwise(int(abs(wp['angle_deg'])))
                time.sleep(DELAY)  # Wait for rotation to complete
                
                # Update position and record data after rotation
                save_pos(waypoints, orientations, abs_position, orientation, 0)

                lastpos_cm = [int(round(coord*100,0)) for coord in get_target_position(UWBTAG_ID)]
                save_pos_UWB(waypoints_UWB, orientations_UWB, lastpos_cm[0:2])
                save_errors(pos_error_list, orientations_error_list, waypoints[-1], waypoints_UWB[-1], orientation, obtain_orientation(waypoints_UWB))
                printdistance(waypoints_UWB[-2], waypoints_UWB[-1])

                check_mission_pad_id(tello, LAND_ID)

            # Handle forward movement in increments
            distance = wp['dist_cm']
            while distance > INCREMENT_CM:
                if distance - INCREMENT_CM < 20:
                    print("[INFO] Distance fine split. Remaining:", distance)
                    tello.move_forward(50)
                    save_pos(waypoints, orientations, abs_position, orientation, 50)

                    lastpos_cm = [int(round(coord*100,0)) for coord in get_target_position(UWBTAG_ID)]
                    save_pos_UWB(waypoints_UWB, orientations_UWB, lastpos_cm[0:2])
                    save_errors(pos_error_list, orientations_error_list, waypoints[-1], waypoints_UWB[-1], orientation, obtain_orientation(waypoints_UWB))
                    printdistance(waypoints_UWB[-2], waypoints_UWB[-1])   

                    check_mission_pad_id(tello, LAND_ID)
                    distance -= 50
                    time.sleep(DELAY)

                else:
                    print("[INFO] Distance split. Remaining:", distance)                    
                    tello.move_forward(INCREMENT_CM)
                    save_pos(waypoints, orientations, abs_position, orientation, INCREMENT_CM)

                    lastpos_cm = [int(round(coord*100,0)) for coord in get_target_position(UWBTAG_ID)]
                    save_pos_UWB(waypoints_UWB, orientations_UWB, lastpos_cm[0:2])
                    save_errors(pos_error_list, orientations_error_list, waypoints[-1], waypoints_UWB[-1], orientation, obtain_orientation(waypoints_UWB))
                    printdistance(waypoints_UWB[-2], waypoints_UWB[-1])

                    check_mission_pad_id(tello, LAND_ID)
                    distance -= INCREMENT_CM
                    time.sleep(DELAY)
                
            # Move remaining distance (if between 50 and 100 cm)
            if distance != 0:
                print("[INFO] No split required. Remaining:", distance)
                tello.move_forward(distance)
                save_pos(waypoints, orientations, abs_position, orientation, distance)

                lastpos_cm = [int(round(coord*100,0)) for coord in get_target_position(UWBTAG_ID)]
                save_pos_UWB(waypoints_UWB, orientations_UWB, lastpos_cm[0:2])
                save_errors(pos_error_list, orientations_error_list, waypoints[-1], waypoints_UWB[-1], orientation, obtain_orientation(waypoints_UWB))
                printdistance(waypoints_UWB[-2], waypoints_UWB[-1])

                check_mission_pad_id(tello, LAND_ID)
                time.sleep(DELAY)

            else:   # for distance = 0
                pass
                print("[INFO] Distance remaining = 0. Path completed.")

            # NEW 7 JAN - READS UWB DISTANCE AFTER EVERY MAJOR WAYPOINT          
            print(f"\n ------------------ \n[LOG] Waypoint {wp_index+1} of {len(data['wp'])} executed")
            print(f"     {len(waypoints)}x Waypoints:", waypoints )
            print(f"     {len(waypoints_UWB)}x UWB Waypoints:", waypoints_UWB)
            print(f"     {len(pos_error_list)}x Pos Errors:", pos_error_list)
            print("----")
            print(f"     {len(orientations)}x Orientations:", orientations )
            print(f"     {len(orientations_UWB)}x UWB Orientations:", orientations_UWB)
            print(f"     {len(orientations_error_list)}x Orientation Errors:", orientations_error_list)
            print("-------------")
    
    except Exception as e:
        print(f"Error occurred: {e}")
    
    finally:
        # if tello.is_flying:
        #     print("Landing...")
        #     tello.land()
        #     end_batt = tello.get_battery()
        #     end_time = time.time()
        #     print(f"End Battery: {end_batt}%")
        #     print(f"Mission Duration (s): {(end_time-start_time):.2f}, Battery Used (%): {start_batt-end_batt}, %/min: {((start_batt-end_batt)/(end_time-start_time)*60):.1f}")
        #     time.sleep(DELAY)
        print("Mission completed! Not Landing.")
        
        # # Save waypoints to JSON file (DISABLED 23 JAN)
        # with open("waypoints_commanded.json", "w") as f:
        #     json.dump(waypoints, f, indent=4)

if __name__ == "__main__":

    # Check if -sim flag is set via CLI - optional
    args = check_args()
    if args.simulate is not None:
        SIMULATE = bool(args.simulate)  # Convert 0/1 to False/True
        print(f"[INFO] SIMULATE set to {SIMULATE}")

    """
    IMPT: Use cases:
    1. (most likely scenario) main() will NOT be called directly, while execute_waypoints() will be imported
    2. main() can also be called directly by main_run_sequence.bat
    """

    tello = MockTello() if SIMULATE else Tello()
    TAKEOFF_DELAY = 0 if SIMULATE else 3 
    DELAY = 0 if SIMULATE else 2

    tello.connect()
    start_batt = tello.get_battery()
    print(f"Battery: {tello.get_battery()}")
    time.sleep(0.5)
    
    # Take off
    print("Taking off...")
    tello.takeoff()
    time.sleep(TAKEOFF_DELAY)  # Give more time for takeoff to stabilize

    if validate_waypoints(INPUT_JSON):
        print(f"Waypoints validated. Starting execution in {'simulation' if SIMULATE else 'real'} mode...")
        time.sleep(2)
        execute_waypoints(INPUT_JSON, tello, SIMULATE)
        print("Landing Now.")
        tello.end()
    else:
        print("Validation failed. Please check warnings above. Exiting program.")