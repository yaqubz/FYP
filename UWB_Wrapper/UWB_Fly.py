# WORKS, STILL TESTING 8 Jan - need to run with nlink_unpack_COMx_udp.exe in the background

# This is for testing querying the UWB position and comparing it to dead reckoning, while flying. 
# Eventually to be integrated into PPFLY

from UWB_ReadUDP import get_target_position

from djitellopy import tello

import math, time, threading

lastpos = None

def update_target_position():
    global lastpos
    while True:
        try:
            # Fetch target position from UDP socket
            lastpos = get_target_position(target_id=1)
            # print(f"Updated position: {lastpos}")
        except Exception as e:
            print(f"Error in get_target_position thread: {e}")
        time.sleep(0.1)  # Small delay to prevent excessive CPU usage

def calculate_vector(pos1, pos2, dims):
    """
    Calculate the 2D/3D vector from pos1 to pos2.
    :param pos1: List [x1, y1, z1] representing the first position.
    :param pos2: List [x2, y2, z2] representing the second position.
    :return: List representing the vector [dx, dy, dz].
    """
    if len(pos1) != 3 or len(pos2) != 3:
        raise ValueError("Positions must be 3D coordinates [x, y, z].")
    
    vector = [pos2[i] - pos1[i] for i in range(dims)]
    print(f"{dims}D Vector from {pos1} to {pos2}: {vector}")
    return vector

def calculate_distance(pos1, pos2, dims):
    """
    Calculate the distance between pos1 and pos2.
    :param pos1: List [x1, y1, z1] representing the first position.
    :param pos2: List [x2, y2, z2] representing the second position.
    :return: Float representing the Euclidean distance.
    """
    if len(pos1) != 3 or len(pos2) != 3:
        raise ValueError("Positions must be 3D coordinates [x, y, z].")
    
    distance = math.sqrt(sum((pos2[i] - pos1[i])**2 for i in range(dims)))
    print(f"{dims}D Vector from {pos1} to {pos2}: {distance}")
    return distance

def printdistances(lastpos, dims):
    newpos = get_target_position(target_id=1)
    print(newpos)
    calculate_vector(lastpos,newpos,dims)
    calculate_distance(lastpos,newpos,dims)
    return newpos



def main():

    target_thread = threading.Thread(target=update_target_position, daemon=True)
    target_thread.start()

    drone = tello.Tello()
    drone.connect()
    print(drone.get_battery())

    # if not drone.get_battery() < 10:
    #     drone.takeoff()
    drone.takeoff()
    drone.go_xyz_speed(200, 0, 0, 20)
    drone.go_xyz_speed(0, -200, 0, 20)

    # Use the shared variable
    global lastpos
    if lastpos is not None:
        print(f"Using last known position: {lastpos}")
    else:
        print("No target position available yet.")

    drone.end()

if __name__ == "__main__":
    main()