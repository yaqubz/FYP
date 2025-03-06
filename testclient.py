from swarmserver.swarmserverclient import MarkerClient
import time

status_str = f"Waiting for takeoff. Start Batt %: XXX"

marker_client = MarkerClient(drone_id=1)
marker_client.client_takeoff_simul([99], status_message=status_str)
print("drone.takeoff() here")
time.sleep(2)

marker_client.send_update('marker', 1, detected=True)
marker_client.send_update('status', status_message="Detected marker 1")



marker_client.send_update('waypoint', 1, detected=True)
time.sleep(2)
marker_client.send_update('waypoint', 2, detected=True)
time.sleep(2)

if marker_client.is_waypoint_available(1):
    marker_client.send_update('status', status_message="Heading to waypoint 1")
else:
    marker_client.send_update('status', status_message="Unable to head to waypoint 1")

print("Sleeping for 5s...")
time.sleep(5)

print("drone.end() here")
marker_client.send_update('status', status_message='Landed')