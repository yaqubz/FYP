import numpy as np
import time
import os
import logging
import cv2
from typing import Optional


import importlib, sys

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

# def get_calibration_parameters(calib_data_path = "./Search/calib_data/MultiMatrix.npz"):  # TESTING 29 JAN, V2
#     calib_data = np.load(calib_data_path)
#     cam_mat = calib_data["camMatrix"]
#     dist_coef = calib_data["distCoef"]
#     r_vectors = calib_data["rVector"]
#     t_vectors = calib_data["tVector"]

#     return cam_mat, dist_coef


# NEW 13 FEB DOESNT WORK - trying to reduce the frame errors

# def capture_frame(frame_reader, max_retries: int = 3):
#     """
#     Optimized frame capture function to minimize packet loss and decoding errors.
#     """
#     retry_count = 0
#     frame = None

#     while retry_count < max_retries:
#         try:
#             # Get the latest frame
#             frame = frame_reader.frame

#             # Ensure frame is valid
#             if frame is None or frame.size == 0:
#                 logging.warning("Empty or corrupt frame received, retrying...")
#                 retry_count += 1
#                 time.sleep(0.05)  # Reduced delay for better responsiveness
#                 continue

#             # Validate frame using OpenCV (optional, but helps with error detection)
#             if not isinstance(frame, (bytes, bytearray)):
#                 logging.warning("Invalid frame format, retrying...")
#                 frame = None
#                 retry_count += 1
#                 continue

#             break  # Frame successfully captured

#         except Exception as e:
#             logging.error(f"Frame capture error: {e}")
#             retry_count += 1
#             time.sleep(0.05)

#     if frame is None:
#         logging.error(f"Failed to capture frame after {max_retries} retries.")

#     return frame

def capture_frame(frame_reader, max_retries:int = 3):
    # Get frame with retry mechanism
    retry_count = 0
    frame = None
    while frame is None and retry_count < max_retries:
        try:
            frame = frame_reader.frame
            if frame is None:
                logging.debug("Frame capture failed, retrying...")
                time.sleep(0.1)
                retry_count += 1
        except Exception as e:
            logging.warning(f"Frame capture failed: {e}. Returning empty frame.")
            frame = None
              
        if frame is None:
            logging.warning(f"Failed to capture frame after {retry_count} retries")

    return frame

def draw_pose_axes(frame, corners, ids, rvecs, tvecs):
    """Draw pose estimation axes and information on frame"""
    logging.debug(f"ID {ids} found. Drawing axes.")
    marker_center = np.mean(corners[0], axis=0)
    cv2.circle(frame, 
                (int(marker_center[0]), int(marker_center[1])), 
                10, (0, 255, 0), -1)
    cv2.polylines(frame, 
                [corners[0].astype(np.int32)], 
                True, (0, 255, 0), 2)
    cv2.putText(frame, 
                f"ID: {ids}", 
                (int(marker_center[0]), int(marker_center[1] - 20)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
       
    cv2.drawFrameAxes(
        frame, CAMERA_MATRIX, DIST_COEFF, rvecs, tvecs, 10
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



def load_params():
    """
    Loads a params.py file via the command line. Allows multiple drones to run on the same script.
    Example usage: 
        python -m UnknownArea_v2.main UnknownArea_v2.params2

    Returns:
        params: The module containing configuration parameters
    """
    default_params = "UnknownArea_v2.params"
    if len(sys.argv) < 2:
        print("Load Params Usage: python script_name.py <params_module>. Trying default params.py.")  
        params_module = default_params
    else:
        params_module = sys.argv[1]

    try:
        params = importlib.import_module(params_module)
        print(f"Successfully loaded parameters from {params_module}!")
        return params
    except ModuleNotFoundError:
        print(f"Error: Module {params_module} not found. Exiting script.")
        sys.exit(1)

def setup_logging(params: object, logger_name: Optional[str] = None) -> logging.Logger:
    """
    Sets up logging configuration based on parameters from params.py
    
    Args:
        params: Module containing logging configuration
        logger_name: Optional name for the logger. If None, uses default from params
    
    Returns:
        logging.Logger: Configured logger instance
    """
    # Get logging parameters
    config = params.LOGGING_CONFIG
    log_level = getattr(logging, config['level'].upper())
    
    # Create handlers
    file_handler = logging.FileHandler(config['filename'], mode='w')
    console_handler = logging.StreamHandler()
    
    # Create formatter
    formatter = logging.Formatter(config['format'])
    file_handler.setFormatter(formatter)
    console_handler.setFormatter(formatter)
    
    # Configure root logger
    logging.basicConfig(
        level=log_level,
        handlers=[file_handler, console_handler]
    )
    
    # Create and configure specific logger
    logger_name = logger_name or config['default_logger_name']
    logger = logging.getLogger(logger_name)
    logger.addHandler(file_handler)
    
    return logger

CAMERA_MATRIX, DIST_COEFF = get_calibration_parameters()
print("Calibration parameters obtained.")