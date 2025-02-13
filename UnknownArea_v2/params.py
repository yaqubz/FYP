"""
Default params.py file; for Laptop-only simulations!
"""

PI_ID:int = None

# Define network configuration constants
NETWORK_CONFIG = {
    # 'host': '192.168.0.111',  # if connected through RPi
    # 'control_port': 9011,
    # 'state_port': 8011,
    # 'video_port': 11111
    'host': '192.168.10.1',     # if connected directly through WiFi
    'control_port': 8889,
    'state_port': 8890,
    'video_port': 11111    
}

WAYPOINTS_JSON = "pathtotheunknown_3mforward_turnleft.json"
LOGFILE_NAME = f"log_UnknownSearchArea_main{PI_ID if PI_ID else ""}.log"
