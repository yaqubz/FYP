# WORKS 23 JAN: Reads waypoint JSON file and executes it, then changes over to navigation_thread to use depth-mapping and lands on valid marker.
# IMPT: Must be connected to drone to work!

"""
Change Log:
# 23 Jan: Works even without ToF; Can run without flying using NO_FLY = True; Press q to exit and land
# 24 Jan: Can stream and work over RPi17; TODO: video stream thread, ToF doesn't work well over RPi
# 29 Jan: Rework 2D and 3D distance mapping, Calibration
# 30 Jan: Distance calculation should be quite precise now, no need for hard-coded offsets. Will not work for victims not on the floor (TBC if need)
"""

from PPFLY2.main import execute_waypoints


import torch
import cv2
from cv2 import aruco
import numpy as np
from djitellopy import Tello
import threading
from threading import Lock
import time
import os
import logging  # in decreasing log level: debug > info > warning > error > critical

NO_FLY = True     # indicate NO_FLY = True so that the drone doesn't fly but the video feed still appears

# Logging handlers and format
file_handler = logging.FileHandler("log.log", mode='w')  # Log to a file (overwrite each run)
console_handler = logging.StreamHandler()  # Log to the terminal

formatter = logging.Formatter("%(levelname)s - %(asctime)s - %(message)s")
file_handler.setFormatter(formatter)
console_handler.setFormatter(formatter)

logging.basicConfig(level=logging.INFO, handlers=[file_handler, console_handler])

# Define network configuration constants
NETWORK_CONFIG = {
    # 'host': '192.168.0.117',  # if connected through RPi 17
    'host': '192.168.10.1',     # if connected directly through WiFi
    'control_port': 8889,
    'state_port': 8890,
    'video_port': 11111
}

# def get_calibration_parameters(): # Original V1
#     camera_matrix = np.array([
#         [921.170702, 0.000000, 459.904354],
#         [0.000000, 919.018377, 351.238301],
#         [0.000000, 0.000000, 1.000000]
#     ])
#     dist_coeffs = np.array([0.036099, -0.028374, -0.003189, -0.001275, 0.000000])
#     return camera_matrix, dist_coeffs

# def get_calibration_parameters(TELLO_NO: str = 'D'):  # TESTING 29 JAN, V2
#     # Construct file paths
#     camera_matrix_path = os.path.join("calib_camera", f"camera_matrix_tello{TELLO_NO}.npy")
#     dist_coeffs_path = os.path.join("calib_camera", f"dist_coeffs_tello{TELLO_NO}.npy")

#     # Ensure files exist before loading
#     if not os.path.exists(camera_matrix_path) or not os.path.exists(dist_coeffs_path):
#         raise FileNotFoundError(f"Calibration files for Tello {TELLO_NO} not found.")

#     # Load calibration parameters
#     camera_matrix = np.load(camera_matrix_path)
#     dist_coeffs = np.load(dist_coeffs_path)
    
#     return camera_matrix, dist_coeffs

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

# def get_calibration_parameters(calib_data_path = "./Search/calib_data/MultiMatrix.npz"):  # TESTING 29 JAN, V2
#     calib_data = np.load(calib_data_path)
#     cam_mat = calib_data["camMatrix"]
#     dist_coef = calib_data["distCoef"]
#     r_vectors = calib_data["rVector"]
#     t_vectors = calib_data["tVector"]

#     return cam_mat, dist_coef


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
        
        # Initialize MiDaS model
        self.model_type = "MiDaS_small"
        self.midas = torch.hub.load("intel-isl/MiDaS", self.model_type)
        self.device = torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")
        self.midas.to(self.device)
        self.midas.eval()
        
        # Load MiDaS transform
        midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms")
        self.transform = midas_transforms.small_transform
        
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
        self.valid_ids = set(range(1, 12))  # Temporary 29 Jan
        self.invalid_ids = set(range(12, 15))
        self.marker_positions = {}
         
        # Navigation parameters
        self.move_speed = 20
        self.yaw_speed = 50
        self.forward_tof_dist = None # to be updated by subthread

    def get_ext_tof(self) -> int:
        """Get ToF sensor reading"""
        response = self.drone.send_read_command("EXT tof?")
        try:
            return int(response.split()[1])
        except ValueError or IndexError:    # IndexError if using threading and the response is not as intended
            return 8888
            
    def process_depth_map(self, frame):
        """Process frame through MiDaS to get depth map"""
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        input_batch = self.transform(frame_rgb).to(self.device)
        
        with torch.no_grad():
            prediction = self.midas(input_batch)
            
        depth_map = prediction.squeeze().cpu().numpy()
        depth_map = cv2.normalize(depth_map, None, 0, 1, norm_type=cv2.NORM_MINMAX)
        return cv2.applyColorMap((depth_map * 255).astype(np.uint8), cv2.COLORMAP_JET)
        
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

    def detect_markers(self, frame, marker_size=14.0):  # 24 Jan: Might need to recalibrate for 14cm
        """Detect ArUco markers and estimate pose"""
        camera_matrix, dist_coeffs = get_calibration_parameters()
        aruco_dict = aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
        parameters = aruco.DetectorParameters()
        corners, ids, rejected = aruco.detectMarkers(frame, aruco_dict, parameters=parameters)
        
        if ids is not None:
            for i, marker_id in enumerate(ids):
                if marker_id[0] in self.valid_ids:
                    # Get pose estimation
                    rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                        corners[i].reshape(1, 4, 2), marker_size, camera_matrix, dist_coeffs
                    )
                    # Calculate Euclidean distance
                    x, y, z = tvecs[0][0]
                    euclidean_distance = np.sqrt(x*x + y*y + z*z)
                    # Store marker position and distance
                    marker_center = np.mean(corners[i][0], axis=0)
                    self.set_marker_x(marker_center[0])
                    logging.info(f"Marker Centre: {marker_center}")
                    self.set_distance(euclidean_distance)
                    return True, corners[i], marker_id[0], rvecs[0], tvecs[0]
        self.set_distance(None)
        self.set_marker_x(None)
        return False, None, None, None, None

def draw_pose_axes(frame, corners, ids, rvecs, tvecs):
    """Draw pose estimation axes and information on frame"""
    if ids is not None:
        camera_matrix, dist_coeffs = get_calibration_parameters()
        
        cv2.drawFrameAxes(
            frame, camera_matrix, dist_coeffs, rvecs, tvecs, 10
        )
        
        rot_matrix = cv2.Rodrigues(rvecs)[0]
        euler_angles = cv2.RQDecomp3x3(rot_matrix)[0]
        x, y, z = tvecs[0]
        roll, pitch, yaw = euler_angles
        
        euclidean_distance = np.sqrt(x*x + y*y + z*z)
        
        position_text = f"Pos (cm): X:{x:.1f} Y:{y:.1f} Z:{z:.1f}"
        rotation_text = f"Rot (deg): R:{roll:.1f} P:{pitch:.1f} Y:{yaw:.1f}"
        distance_text = f"3D Distance: {euclidean_distance:.1f} cm"
        
        cv2.putText(frame, position_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.putText(frame, rotation_text, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.putText(frame, distance_text, (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    
    return frame

def tof_update_thread(controller, Hz: float = 2):
    """
    Subthread for updating forward ToF reading
    Implemented 3 Feb, works but may block other responses from drone
    May work better with RPi? 
    Possible solution: Find optimal refresh rate? Use both subthread AND main thread?
    
    """
    while True:
        controller.forward_tof_dist = controller.get_ext_tof()
        time.sleep(1/Hz)


def navigation_thread(controller):
    """Main navigation thread combining depth mapping and marker detection"""
    logging.info("Starting navigation with depth mapping...")
    
    # Create single window for combined view
    cv2.namedWindow("Drone Navigation", cv2.WINDOW_NORMAL)
    
    # Ensure frame reader is initialized
    frame_reader = controller.drone.get_frame_read()
    logging.info("Initializing Camera...")
    time.sleep(2)  # Give time for camera to initialize
    
    # Initial movement
    logging.info("Moving to initial altitude...")
    if not NO_FLY:
        # current_height = controller.drone.get_height()
        # desired_height = 80
        # if current_height < desired_height:
        #     controller.drone.move_up(desired_height - current_height)    # TODO 30 Jan: How to go to specific height?
        #     new_height = controller.drone.get_height()
        # new_height = controller.drone.get_height()
        # print(f"Drone moved from {current_height:.0f}cm to {new_height:.0f}cm height.")    
        controller.drone.move_up(20)
    time.sleep(2)
    
    # Approach sequence state
    approach_complete = False
    centering_complete = False
    approach_start_time = None
    
    while True:  # Main loop continues until marker found or battery low
        try:
            # Check battery level
            battery_level = controller.drone.get_battery()
            if battery_level < 10:  # Critical battery threshold
                logging.warning(f"Battery level critical ({battery_level}%)! Initiating landing sequence...")
                break

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
                
            # Create a copy of frame for visualization
            display_frame = frame.copy()
            
            # Get depth map
            depth_colormap = controller.process_depth_map(frame)

            marker_found = False
            
            # Check for markers
            marker_found, corners, marker_id, rvecs, tvecs = controller.detect_markers(frame)
            if marker_found:
                # Draw marker detection and pose information
                marker_center = np.mean(corners[0], axis=0)
                cv2.circle(display_frame, 
                          (int(marker_center[0]), int(marker_center[1])), 
                          10, (0, 255, 0), -1)
                cv2.polylines(display_frame, 
                            [corners[0].astype(np.int32)], 
                            True, (0, 255, 0), 2)
                cv2.putText(display_frame, 
                          f"ID: {marker_id}", 
                          (int(marker_center[0]), int(marker_center[1] - 20)),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                # Draw pose estimation
                display_frame = draw_pose_axes(display_frame, corners, [marker_id], rvecs, tvecs)
                
                logging.info(f"Valid marker {marker_id} detected! Switching to approach sequence...")
                
                # Center on the marker
                frame_center = frame.shape[1] / 2
                x_error = marker_center[0] - frame_center
                
                # Centering threshold
                centering_threshold = 30
                
                if not NO_FLY:
                    if not centering_complete:
                        if abs(x_error) > centering_threshold:
                            # Calculate yaw speed based on error
                            yaw_speed = int(np.clip(x_error / 10, -20, 20))
                            controller.drone.send_rc_control(0, 0, 0, yaw_speed)
                            logging.info(f"Centering: error = {x_error:.1f}, yaw_speed = {yaw_speed}")
                        else:
                            logging.info("Marker centered! Starting approach...")
                            controller.drone.send_rc_control(0, 0, 0, 0)  # Stop rotation
                            time.sleep(1)  # Stabilize
                            centering_complete = True
                    
                    elif not approach_complete:
                        current_distance_3D = controller.get_distance()
                        current_height = controller.drone.get_distance_tof()    # may not work if floor is not even (alt: current_height = controller.drone.get_height())
                        current_distance_2D = np.sqrt(current_distance_3D**2 - current_height**2)
                        if current_distance_2D is None:
                            logging.info("Lost marker distance during approach...")
                            time.sleep(0.1)
                            continue
                        
                        logging.info(f"Current 3D distance to marker: {current_distance_3D:.1f}cm")
                        logging.info(f"Current height: {current_height:.1f}cm")      
                        logging.info(f"Current 2D distance to marker: {current_distance_2D:.1f}cm")

                        if current_distance_2D > 0:
                            logging.info(f"Moving forward {current_distance_2D}cm to approach marker...")
                            controller.drone.send_rc_control(0, 0, 0, 0)  # Stop any existing movement
                            time.sleep(1)  # Stabilize
                            controller.drone.move_forward(current_distance_2D)  # Move exact distance
                            time.sleep(2)  # Wait for movement to complete
                            
                            # Verify new position (TBC 29 Jan not necessary?)
                            new_distance = controller.get_distance()
                            if new_distance is not None:
                                logging.info(f"New 3D distance to marker: {new_distance:.1f}cm")
                        
                        logging.info("Approach complete!")
                        controller.drone.send_rc_control(0, 0, 0, 0)
                        approach_complete = True
                        time.sleep(1)
                        break
                    
                    else:  # Both centering and approach are complete (TBC 23 Jan - does it ever come here?)
                        logging.info("Initiating landing sequence...")
                        break

                else: # if simulating
                    logging.info("Marker Found, but not landing since not flying. Program continues.")
            
            # Split depth map into regions for navigation
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
            
            # Get ToF distance
            dist = controller.get_ext_tof()       ## COMMENTED OUT 3 FEB in favour of threading (TBC: may be good to have both?)
            dist = controller.forward_tof_dist
            
            # Draw navigation info on display frame
            cv2.putText(display_frame, f"ToF: {dist}mm", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # Navigation logic
            if not marker_found:
                if red_center > blue_center:
                    # Obstacle ahead - turn towards more open space
                    if blue_left > blue_right:
                        controller.drone.send_rc_control(0, 0, 0, -controller.yaw_speed)
                        cv2.putText(display_frame, "Turning Left", (10, 60), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                        logging.info("Turning Left")
                    else:
                        controller.drone.send_rc_control(0, 0, 0, controller.yaw_speed)
                        cv2.putText(display_frame, "Turning Right", (10, 60), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                        logging.info("Turning Right")
                else:
                    if blue_center > red_center and dist <= 600:
                        if not NO_FLY:
                            controller.drone.rotate_clockwise(135)
                        cv2.putText(display_frame, "Avoiding Obstacle", (10, 60), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                        logging.info("Avoiding Obstacle")
                        time.sleep(1)  # Stabilize

                    else:
                        controller.drone.send_rc_control(0, controller.move_speed, 0, 0)
                        cv2.putText(display_frame, "Moving Forward", (10, 60), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # Resize depth_colormap to match frame dimensions
            depth_colormap_resized = cv2.resize(depth_colormap, (display_frame.shape[1]//2, display_frame.shape[0]))
            
            # Create combined view with original frame and depth map side by side
            combined_view = np.hstack((display_frame, depth_colormap_resized))
            
            # Add labels
            cv2.putText(combined_view, "Live Feed", (10, combined_view.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(combined_view, "Depth Map", (display_frame.shape[1] + 10, combined_view.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Display combined view
            cv2.imshow("Drone Navigation", combined_view)
            
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
    try:
        logging.info("Taking off...")
        if not NO_FLY:    
            controller.drone.takeoff()
            controller.drone.send_rc_control(0, 0, 0, 0)    # Added 30 Jan for stabilization (TBC)
            execute_waypoints("waypoints_samplesmall.json", controller.drone, NO_FLY)
        else:
            logging.info("Simulating takeoff. Drone will NOT fly.")
            execute_waypoints("waypoints_samplesmall.json", controller.drone, NO_FLY)
        time.sleep(2)
        
        ## MTD 1: Directly call function
        tof_thread = threading.Thread(target=tof_update_thread, args=(controller,2))
        tof_thread.daemon = True     ## Daemon threads run in the background, and are killed automatically when your program quits. 
        tof_thread.start()


        navigation_thread(controller)

        ## MTD 2: Start navigation thread (Gab TBC 23 Jan: Does it really need a thread? More like a thread for video streaming maybe. Same performance.)

        # nav_thread = threading.Thread(target=navigation_thread, args=(controller,))
        # nav_thread.start()
        # nav_thread.join()   # instructs the main thread to wait until the target thread finishes its execution before proceeding. 

        logging.info("Actually landing for real.")
        controller.drone.end()
        cv2.destroyAllWindows()
        cv2.waitKey(1)
    
    except Exception as e:
        logging.error(f"Error in main: {e}")

if __name__ == "__main__":
    main()