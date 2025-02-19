# Tested 11 Feb - launch multiple clients

from swarmserverclient import MarkerClient
import time

marker_client = MarkerClient(drone_id=1)

while True:
    marker_client.send_update(1, detected=True)
    time.sleep(1)
    marker_client.send_update(1, detected=False)
    time.sleep(1)
    marker_client.send_update(1, landed=True)
    time.sleep(1)
    marker_client.send_update(2, detected=True)
    time.sleep(1)
    marker_client.send_update(2, detected=False)
    time.sleep(1)