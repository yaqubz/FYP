import torch
import cv2
from cv2 import aruco
import numpy as np
from threading import Lock
import logging  # in decreasing log level: debug > info > warning > error > critical

from .customtello import CustomTello, MockTello
from .utils import *
from markerserver.markerserverclient import MarkerClient

class DroneController:
    def __init__(self, network_config, simulate = False):
        # Initialize Tello
        self.drone = MockTello() if simulate else CustomTello(network_config)
        self.drone.connect()
        logging.info(f"Start Battery Level: {self.drone.get_battery()}%")
        self.drone.streamon()
        
        # Initialize MiDaS model
        self.model_type = "MiDaS_small"
        self.midas = torch.hub.load("intel-isl/MiDaS", self.model_type)
        self.device = torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")
        self.midas.to(self.device)
        self.midas.eval()
        
        # Load MiDaS transform
        midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms")
        self.transform = midas_transforms.small_transform

        # Color depth map
        self.depth_map_colors = {}
        
        # Controller state
        self.frame = None
        self.frame_lock = Lock()
        self.distance = None        # 29 Jan Gab: This is the 3D distance - decently accurate
        self.distance_lock = Lock()
        self.is_running = True
        self.marker_detected = False
        self.markernum_lockedon:int = None
        self.is_centered = False
        self.movement_completed = False
        self.valid_ids = set(range(1, 9))   # set because doesn't change
        self.invalid_ids = []               # detected/landed by other drones, updated by MarkerClient
        self.danger_ids = set(range(9, 15)) # set because doesn't change
        self.exit_ids = set(range(50, 100)) # set because doesn't change; for drone to avoid exiting search area
        self.exit_detected = False
        self.exit_distance_3D = None
        self.marker_positions = {}
         
        # Navigation parameters
        self.move_speed = 20
        self.yaw_speed = 50
        self.forward_tof_dist = None # to be updated by subthread
        self.forward_tof_lock = Lock()

        self.target_yaw = None

        # Marker Server/Client (6 Feb new)
        self.marker_client = MarkerClient()
            
    def generate_color_depth_map(self, frame):
        """Process frame through MiDaS to get depth map"""
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        input_batch = self.transform(frame_rgb).to(self.device)
        
        with torch.no_grad():
            prediction = self.midas(input_batch)
            
        depth_map = prediction.squeeze().cpu().numpy()
        depth_map = cv2.normalize(depth_map, None, 0, 1, norm_type=cv2.NORM_MINMAX)
        return cv2.applyColorMap((depth_map * 255).astype(np.uint8), cv2.COLORMAP_JET)
        
    def process_depth_color_map(self, depth_colormap):
        """
        :param: depth_colormap: the actual frame of the depth color map
        :return: None
        Split depth map into regions for navigation. Updates class attributes.
        """
        h, w = depth_colormap.shape[:2]
        left_region = depth_colormap[:, :w//3]
        center_region = depth_colormap[:, w//3:2*w//3]
        right_region = depth_colormap[:, 2*w//3:]
        
        # Draw region divisions on depth map
        cv2.line(depth_colormap, (w//3, 0), (w//3, h), (0, 255, 0), 2)
        cv2.line(depth_colormap, (2*w//3, 0), (2*w//3, h), (0, 255, 0), 2)
        
        # Analyze colors in regions
        red_left = np.sum((left_region[:, :, 2] > 150) & (left_region[:, :, 0] < 50))
        red_center = np.sum((center_region[:, :, 2] > 150) & (center_region[:, :, 0] < 50))
        red_right = np.sum((right_region[:, :, 2] > 150) & (right_region[:, :, 0] < 50))
        
        blue_left = np.sum((left_region[:, :, 0] > 150) & (left_region[:, :, 2] < 50))
        blue_center = np.sum((center_region[:, :, 0] > 150) & (center_region[:, :, 2] < 50))
        blue_right = np.sum((right_region[:, :, 0] > 150) & (right_region[:, :, 2] < 50))

        # Update class attribute
        self.depth_map_colors["red"] = {
            "left": red_left,
            "center": red_center,
            "right": red_right
        }

        self.depth_map_colors["blue"] = {
            "left": blue_left,
            "center": blue_center,
            "right": blue_right
        }

    def get_distance(self):
        with self.distance_lock:
            return self.distance
            
    def set_distance(self, distance):
        with self.distance_lock:
            self.distance = distance

    def get_tof_distance(self):
        with self.forward_tof_lock:
            return self.forward_tof_dist

    def detect_markers(self, frame, marker_size=14.0):
        """
        Detect ArUco markers and estimate pose
        Returns values of the FIRST DETECTED VALID MARKER 
        :return:
            marker_id:int         
        """
        global CAMERA_MATRIX, DIST_COEFF
        aruco_dict = aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
        parameters = aruco.DetectorParameters()
        corners, ids, rejected = aruco.detectMarkers(frame, aruco_dict, parameters=parameters)
        
        # Reset target yaw when no markers detected
        self.target_yaw = None

        self.invalid_ids = self.marker_client.get_invalid_markers(self.valid_ids)   # To deactivate marker detection for markers already detected by others
        logging.debug(f"Invalid markers: {self.invalid_ids}")

        if ids is not None:
            detected_ids = ids.flatten()
            logging.debug(f"Detected IDs: {detected_ids}")

            # Check for exit markers
            if set(detected_ids).intersection(self.exit_ids):
                self.exit_detected = True
                logging.info("Exit marker detected!")

            # Process markers for pose and yaw calculation
            for i, marker_id in enumerate(detected_ids):
                # Estimate pose for ALL markers
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                    corners[i].reshape(1, 4, 2), marker_size, CAMERA_MATRIX, DIST_COEFF
                )

                # Return info if marker_id is valid AND either 
                    #  1. Not invalid (detected by another drone), OR
                    #  2. Already locked onto by itself

                if marker_id in self.valid_ids and (not marker_id in self.invalid_ids or marker_id == self.markernum_lockedon):
                    x, y, z = tvecs[0][0]
                    euclidean_distance = np.sqrt(x*x + y*y + z*z)
                    marker_center = np.mean(corners[i][0], axis=0)
                    
                    # Store basic marker info
                    self.set_distance(euclidean_distance)
                    logging.debug(f"Marker Centre: {marker_center}")

                    return True, corners[i], int(marker_id), rvecs[0], tvecs[0]  # Returns only the FIRST valid marker that fulfils the two criteria above (1, 2)

                # If it's an exit marker, compute its yaw
                elif marker_id in self.exit_ids:
                    x, y, z = tvecs[0][0]
                    euclidean_distance = np.sqrt(x*x + y*y + z*z)
                    self.exit_distance_3D = euclidean_distance
                    # Convert rotation vector to rotation matrix
                    R, _ = cv2.Rodrigues(rvecs[0])
                    # Calculate yaw (rotation around Z-axis)
                    yaw = np.arctan2(R[1, 0], R[0, 0])  # Yaw in radians
                    self.target_yaw = np.degrees(yaw)
                    logging.info(f"Exit marker yaw: {self.target_yaw:.2f}°")

        self.set_distance(None)
        return False, None, None, None, None

