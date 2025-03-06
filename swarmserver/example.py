from swarmserverclient import MarkerClient
from djitellopy import Tello
import time

tello = Tello()

tello.connect()
start_batt = tello.get_battery()
status_str = f"Start Batt %: {start_batt}"

marker_client = MarkerClient(drone_id=1)
marker_client.client_takeoff_simul([99], status_message=status_str)
tello.takeoff()

marker_client.send_update('marker', 1, detected=True)
time.sleep(2)
marker_client.send_update('marker', 1, landed=True)
time.sleep(2)

tello.end()
marker_client.send_update('status', status_message='Landed')