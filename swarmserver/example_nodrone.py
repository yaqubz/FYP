"""
No need to connect to drone!
Run this AFTER running swarmserverclient.py, then observe the GUI.
"""

from swarmserverclient import MarkerClient
import time

status_str = f"Waiting for takeoff. Start Batt %: XXX"

marker_client = MarkerClient(drone_id=1)
marker_client.client_takeoff_simul([99], status_message=status_str) # [99] ensures that takeoff is only triggered by the user. Otherwise, put the list of drone_ids you want to takeoff with.
print("drone.takeoff() here")
time.sleep(2)

print("Simulating waypoint 1 occupied by another drone.")   # NOTE caa 6 Mar: waypoint timeout not yet implemented! server-side implementation
marker_client.send_update('waypoint', 1, detected=True)
time.sleep(1)

try:
    # This while-loop simulates some updates being sent to the server.
    while True:
        marker_client.send_update('marker', 1, detected=True)
        time.sleep(2)
        marker_client.send_update('marker', 1, landed=True)
        time.sleep(2)

        if marker_client.is_waypoint_available(1):
            print("Heading to waypoint 1")
            marker_client.send_update('waypoint', 1, detected=True)
            time.sleep(2)
            print("Waypoint 1 completed.")
            marker_client.send_update('waypoint', 1, detected=False)
        else:
            print("Unable to head to waypoint 1.")
            marker_client.send_update('status', status_message="Unable to head to waypoint 1")

        if marker_client.is_waypoint_available(2):
            print("Heading to waypoint 2")
            marker_client.send_update('waypoint', 2, detected=True)
            time.sleep(2)
            print("Waypoint 2 completed.")
            marker_client.send_update('waypoint', 2, detected=False)
        else:
            print("Unable to head to waypoint 2.")
            marker_client.send_update('status', status_message="Unable to head to waypoint 2")

except KeyboardInterrupt:
    print("User interrupt. Exiting.")
except Exception as e:
    print(f"Error: {e}. Exiting.")

finally:
    print("drone.end() here")
    marker_client.send_update('status', status_message='Landed')