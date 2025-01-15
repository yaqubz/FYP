# 15 Jan Testing - Source: ChatGPT

from UWB_ReadUDP import get_target_position
from djitellopy import tello
import math, time, threading

lastpos = None

def update_target_position():
    global lastpos
    while True:
        try:
            lastpos = get_target_position(target_id=1)
        except Exception as e:
            print(f"Error in get_target_position thread: {e}")
        time.sleep(0.1)

def is_within_boundary(position, boundary):
    """
    Check if the position is within the defined boundary.
    :param position: List [x, y, z] of the drone's position.
    :param boundary: Tuple ((x_min, x_max), (y_min, y_max)).
    :return: Boolean indicating whether the position is within the boundary.
    """
    x_min, x_max = boundary[0]
    y_min, y_max = boundary[1]
    return x_min <= position[0] <= x_max and y_min <= position[1] <= y_max

def calculate_turn_angle():
    """
    Return the recommended turn angle (90°).
    :return: Integer representing the turn angle in degrees.
    """
    return 90

def main():
    # Start UWB position update thread
    target_thread = threading.Thread(target=update_target_position, daemon=True)
    target_thread.start()

    # Drone initialization
    drone = tello.Tello()
    drone.connect()
    print(f"Battery: {drone.get_battery()}%")

    # Safety check before takeoff
    if drone.get_battery() < 10:
        print("Battery too low. Exiting.")
        return

    # Take off
    drone.takeoff()
    print("Drone took off.")

    # Define the 2m x 2m boundary
    boundary = ((2, 4), (2, 4))

    # Flying logic
    try:
        while True:
            if lastpos is not None:
                if not is_within_boundary(lastpos, boundary):
                    print(f"Boundary exceeded at position {lastpos}. Turning...")
                    drone.rotate_clockwise(calculate_turn_angle())
                else:
                    # Fly forward a small distance to explore the square
                    drone.send_rc_control(20, 0, 0, 0)  # Forward motion
                    time.sleep(1)  # Adjust duration as needed

    except KeyboardInterrupt:
        print("Landing...")
        drone.land()
        print("Drone landed safely.")
    finally:
        drone.end()

if __name__ == "__main__":
    main()
