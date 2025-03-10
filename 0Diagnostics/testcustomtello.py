"""
6 Mar TESTING - trying to see if TAKEOFF_DELAY can be overwritten from 20s to 10s
Use this script for any simple drone testing over different network config!
"""


from shared_utils.customtello import CustomTello

# pi_id = 1
# NETWORK_CONFIG = {'host': f'192.168.0.{100+pi_id}',
#                     'control_port': 9000 + pi_id,
#                     'state_port': 8000 + pi_id,
#                     'video_port': 11100 + pi_id}

# NETWORK_CONFIG = {'host': '192.168.10.1',     # if connected directly through WiFi
#                     'control_port': 8889,
#                     'state_port': 8890,
#                     'video_port': 11111}

NETWORK_CONFIG = {'host': '192.168.0.121',     # if connected directly through WiFi
                    'control_port': 8889,
                    'state_port': 8890,
                    'video_port': 11111}

drone = CustomTello(network_config=NETWORK_CONFIG)

drone.connect()
# drone.set_video_port(11121)
# drone.set_state_port(8021)
print(drone.get_battery())
drone.takeoff()
drone.end()
