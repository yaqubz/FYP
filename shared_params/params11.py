"""
This is to fly through RPi.
"""

LAPTOP_ONLY = False # indicate LAPTOP_ONLY = True to use MockTello() and laptop webcam instead
NO_FLY = False     # indicate NO_FLY = True to connect to the drone, but ensure it doesn't fly while the video feed still appears
IMSHOW = True        # indicate IMSHOW = False to disable video stream display (program still works!)

PI_ID:int = 11
UWBTAG_ID:int = 11

# WAYPOINTS_JSON = "waypoint_fwdtiny.json"
WAYPOINTS_JSON = 'waypoint2mfwd.json'
# WAYPOINTS_JSON = 'waypoint_turnleft.json'




### LESS COMMONLY TOUCHED PARAMS ###

EXTRA_HEIGHT = 0

LOGGING_CONFIG = {
    'filename': f"log_Pi{PI_ID if PI_ID else ''}.log",
    'level': "DEBUG",  # Change this to "INFO", "WARNING", etc.
    'format': "%(levelname)s - %(name)s - %(asctime)s - %(message)s",
    'default_logger_name': "DroneController"
}

def get_network_config(pi_id: int | None = None):
    """
    Returns network configuration based on pi_id.
    Args:
        pi_id: None for direct WiFi connection, or an integer (e.g., 2, 12) for RPi connection
    Returns:
        dict: Network configuration with host and port settings
    """

    if pi_id is None:
        return {
            'host': '192.168.10.1',     # if connected directly through WiFi
            'control_port': 8889,
            'state_port': 8890,
            'video_port': 11111    
        }
    else:
        return {
            'host': f'192.168.0.{pi_id+100}',
            'control_port': 9000 + pi_id,
            'state_port': 8000 + pi_id,
            'video_port': 11100 + pi_id
        }

NETWORK_CONFIG = get_network_config(PI_ID)    # Function defined at bottom of file

if LAPTOP_ONLY:     # just in case
    NO_FLY = True
    PI_ID = None
    get_network_config(PI_ID)     


