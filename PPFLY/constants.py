# Adjust Params Here
LAND_ID = 3 # set to 0 for no land
SIMULATE = True # set to True to initiate MockTello() class (i.e. no fly)
START_HEADING = 0
INCREMENT_CM = 300      # IMPT: maximum go_xyz distance is 500cm; should be set to a value no more than 500
INPUT_JSON = 'waypoint20x20.json'
UWBTAG_ID = 1

# Do not touch Params
FLYING_STATE = False
waypoints = []      # to store executed waypoints and drone's current position
waypoints_UWB = []  # to store UWB recorded waypoints

orientations = []  # to store UWB recorded waypoints
orientations_UWB = []  # to store UWB recorded waypoints

pos_error_list = []     # to store difference in dead reckoning and UWB positions
orientations_error_list = []  # to store UWB recorded waypoints

start_batt, end_batt = 0, 0