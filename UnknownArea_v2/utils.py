import numpy as np
import time
import os
import logging
import cv2

# TODO: move calib params into UnknownArea_v2 folder?
def get_calibration_parameters(TELLO_NO: str = 'E920EB_480P'): # BEST VERSION 30 JAN, V2.2 (to be tested)
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

CAMERA_MATRIX, DIST_COEFF = get_calibration_parameters()
print("Calibration parameters obtained.")