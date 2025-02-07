import numpy as np
import time
import os
import logging


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

CAMERA_MATRIX, DIST_COEFF = get_calibration_parameters()
logging.debug("Calibration parameters obtained.")