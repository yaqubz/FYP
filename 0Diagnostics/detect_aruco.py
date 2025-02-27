"""
Works 4 Feb - Prints and shows all detected Aruco pose axes 
"""

import cv2
from cv2 import aruco
import numpy as np
from djitellopy import Tello
from threading import Lock
import time
import os
import logging  # in decreasing log level: debug > info > warning > error > critical

NO_FLY = True     # indicate NO_FLY = True so that the drone doesn't fly but the video feed still appears

# Logging handlers and format
file_handler = logging.FileHandler("detect_log.log", mode='w')  # Log to a file (overwrite each run)
console_handler = logging.StreamHandler()  # Log to the terminal

formatter = logging.Formatter("%(levelname)s - %(asctime)s - %(message)s")
file_handler.setFormatter(formatter)
console_handler.setFormatter(formatter)

logging.basicConfig(level=logging.DEBUG, handlers=[file_handler, console_handler])

# Define network configuration constants
NETWORK_CONFIG = {
    # 'host': '192.168.0.117',  # if connected through RPi 17
    'host': '192.168.10.1',     # if connected directly through WiFi
    'control_port': 8889,
    'state_port': 8890,
    'video_port': 11111
}

def get_calibration_parameters(TELLO_NO: str = 'D'): # BEST VERSION 30 JAN, V2.2 (to be tested)
    """
    Retrieves the camera matrix and distortion coefficients for the specified Tello drone.

    Args:
        TELLO_NO (str): Identifier for the Tello drone (e.g., 'D').

    Returns:
        tuple: Camera matrix and distortion coefficients.

    Raises:
        FileNotFoundError: If the calibration files for the specified Tello drone are not found.
    """
    # Construct file paths
    camera_matrix_path = os.path.join("calib_camera", f"camera_matrix_tello{TELLO_NO}.npy")
    dist_coeffs_path = os.path.join("calib_camera", f"dist_coeffs_tello{TELLO_NO}.npy")

    # Ensure files exist before loading
    if not os.path.exists(camera_matrix_path) or not os.path.exists(dist_coeffs_path):
        raise FileNotFoundError(f"Calibration files for Tello {TELLO_NO} not found.")

    # Load calibration parameters
    camera_matrix = np.load(camera_matrix_path)
    dist_coeffs = np.load(dist_coeffs_path)
    
    return camera_matrix, dist_coeffs


class CustomTello(Tello):
    def __init__(self, network_config):
        # Store custom configuration
        self.TELLO_IP = network_config['host']
        self.CONTROL_UDP_PORT = network_config['control_port']
        self.STATE_UDP_PORT = network_config['state_port']
        self.VS_UDP_PORT = network_config['video_port']
        
        Tello.STATE_UDP_PORT = self.STATE_UDP_PORT
        Tello.CONTROL_UDP_PORT = self.CONTROL_UDP_PORT
        
        # Call parent's init with our custom host
        super().__init__(self.TELLO_IP)
        
        # Override the connection parameters
        self.address = (self.TELLO_IP, self.CONTROL_UDP_PORT)
        
        # Override video port
        self.vs_udp_port = self.VS_UDP_PORT

class DroneController:
    def __init__(self, network_config):
        # Initialize Tello
        self.drone = CustomTello(network_config)
        self.drone.connect()
        logging.info(f"Battery Level: {self.drone.get_battery()}%")
        self.drone.streamon()
    
        
        # Controller state
        self.frame = None
        self.frame_lock = Lock()
        self.distance = None        # 29 Jan Gab: This is the 3D distance - decently accurate
        self.distance_lock = Lock()
        self.marker_x = None    # NOT BEING USED? 29 Jan Gab
        self.marker_x_lock = Lock()
        self.is_running = True
        self.marker_detected = False
        self.is_centered = False
        self.movement_completed = False
        self.valid_ids = set(range(1, 100))  # Temporary 29 Jan
        self.invalid_ids = set(range(12, 15))
        self.exit_ids = set(range(50, 100)) # Added 3 Feb - for drone to avoid exiting search area
        self.exit_detected = False
        self.marker_positions = {}
         
        # Navigation parameters
        self.move_speed = 20
        self.yaw_speed = 50
        self.forward_tof_dist = None # to be updated by subthread

        self.target_yaw = None
        
    def get_distance(self):
        with self.distance_lock:
            return self.distance
            
    def set_distance(self, distance):
        with self.distance_lock:
            self.distance = distance
            
    def get_marker_x(self):
        with self.marker_x_lock:
            return self.marker_x
            
    def set_marker_x(self, x):
        with self.marker_x_lock:
            self.marker_x = x

    def detect_markers(self, frame, marker_size=14.0):
        """Detect ArUco markers and estimate pose"""
        camera_matrix, dist_coeffs = get_calibration_parameters()
        aruco_dict = aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
        parameters = aruco.DetectorParameters()
        corners, ids, rejected = aruco.detectMarkers(frame, aruco_dict, parameters=parameters)
        
        # Reset target yaw when no markers detected
        self.target_yaw = None

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
                    corners[i].reshape(1, 4, 2), marker_size, camera_matrix, dist_coeffs
                )

                # If it's a valid marker, store its info
                if marker_id in self.valid_ids:
                    x, y, z = tvecs[0][0]
                    euclidean_distance = np.sqrt(x*x + y*y + z*z)
                    marker_center = np.mean(corners[i][0], axis=0)
                    
                    # Store basic marker info
                    self.set_marker_x(marker_center[0])
                    self.set_distance(euclidean_distance)
                    logging.info(f"Marker Centre: {marker_center}")

                    marker_center = np.mean(corners[i][0], axis=0)
                    cv2.circle(frame, 
                            (int(marker_center[0]), int(marker_center[1])), 
                            10, (0, 255, 0), -1)
                    cv2.polylines(frame, 
                                [corners[i].astype(np.int32)], 
                                True, (0, 255, 0), 2)
                    cv2.putText(frame, 
                            f"ID: {marker_id}", 
                            (int(marker_center[0]), int(marker_center[1] - 20)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    
                    # Draw pose estimation
                    frame = draw_pose_axes(frame, camera_matrix, dist_coeffs, rvecs, tvecs)

                # If it's an exit marker, compute its yaw
                elif marker_id in self.exit_ids:
                    # Convert rotation vector to rotation matrix
                    R, _ = cv2.Rodrigues(rvecs[0])
                    # Calculate yaw (rotation around Z-axis)
                    yaw = np.arctan2(R[1, 0], R[0, 0])  # Yaw in radians
                    self.target_yaw = np.degrees(yaw)
                    logging.info(f"Exit marker yaw: {self.target_yaw:.2f}°")

        self.set_distance(None)
        self.set_marker_x(None)
        return frame

def draw_pose_axes(frame, camera_matrix, dist_coeffs, rvecs, tvecs):
    """Draw pose estimation axes and information on frame"""
    for i in range(len(rvecs)):
        cv2.drawFrameAxes(
            frame, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 10
        )
        
        rot_matrix = cv2.Rodrigues(rvecs[i])[0]
        euler_angles = cv2.RQDecomp3x3(rot_matrix)[0]
        x, y, z = tvecs[i][0]
        roll, pitch, yaw = euler_angles
        
        euclidean_distance = np.sqrt(x*x + y*y + z*z)
        
        position_text = f"Pos (cm): X:{x:.1f} Y:{y:.1f} Z:{z:.1f}"
        rotation_text = f"Rot (deg): R:{roll:.1f} P:{pitch:.1f} Y:{yaw:.1f}"
        distance_text = f"3D Distance: {euclidean_distance:.1f} cm"
        
        cv2.putText(frame, position_text, (10, 60 + i * 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.putText(frame, rotation_text, (10, 90 + i * 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.putText(frame, distance_text, (10, 120 + i * 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    
    return frame


def navigation_thread(controller):
    """Main navigation thread combining depth mapping and marker detection"""
    logging.info("Starting navigation with depth mapping...")
    
    # Create single window for combined view
    cv2.namedWindow("Drone Navigation", cv2.WINDOW_NORMAL)
    
    # Ensure frame reader is initialized
    frame_reader = controller.drone.get_frame_read()
    logging.info("Initializing Camera...")
    time.sleep(2)  # Give time for camera to initialize
    
    
    while True:  # Main loop continues until marker found or battery low
        try:
            # Get frame with retry mechanism
            retry_count = 0
            frame = None
            while frame is None and retry_count < 3:
                frame = frame_reader.frame
                if frame is None:
                    logging.warning("Frame capture failed, retrying...")
                    time.sleep(0.1)
                    retry_count += 1
            
            if frame is None:
                logging.warning("Failed to capture frame after retries")
                continue
                
            # Check for markers
            frame = controller.detect_markers(frame)
            
            # Display combined view
            cv2.imshow("Drone Navigation", frame)
            
            # If 'q' is pressed, just continue searching
            if cv2.waitKey(1) & 0xFF == ord('q'):
                logging.info("Exiting. Drone landing.")
                # controller.drone.land()
                break
                
        except Exception as e:
            logging.warning(f"Error in navigation: {e}. Continuing navigation.")
            # Don't cleanup or break - continue searching
            continue

def main():
    controller = DroneController(NETWORK_CONFIG)
    navigation_thread(controller) 

    logging.info("Actually landing for real.")
    controller.drone.end()
    cv2.destroyAllWindows()
    cv2.waitKey(1)

if __name__ == "__main__":
    main()