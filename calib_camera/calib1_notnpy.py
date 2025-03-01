"""
Works well 14 Feb 2025
This script takes the photos captured by calib0.py and produces camera_matrix and dist_coeff variables.

Script Params:
    - TELLO_NO:str
"""

import cv2
import numpy as np
import glob

def calibrate_camera(TELLO_NO: str):
    # Define checkerboard parameters
    CHECKERBOARD = (7, 10)  # Number of inner corners (one less than the number of squares)
    SQUARE_SIZE = 15  # Size of each square in mm

    # Prepare object points (3D points in real-world space)
    objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
    objp *= SQUARE_SIZE  # Scale to real-world square size

    # Arrays to store object points and image points
    objpoints = []  # 3D points
    imgpoints = []  # 2D points

    # Load images in the directory, if they contain the string == TELLO_NO
    images = [img for img in glob.glob("calib_camera/calibration_images/*.jpg") if TELLO_NO in img]

    print(f"Found {len(images)} images")

    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Detect checkerboard corners
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

        if ret:
            print(f"Chessboard detected in {fname}")  # Debugging output
            objpoints.append(objp)
            imgpoints.append(corners)
        else:
            print(f"Failed to detect checkerboard in {fname}")

    cv2.destroyAllWindows()

    # Ensure at least one valid image before calibrating
    if len(objpoints) == 0:
        print("Error: No valid images with detected checkerboards. Calibration aborted.")
        return None, None  # Return None if calibration failed

    # Perform camera calibration
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None
    )
    print("Camera Matrix:\n", camera_matrix)
    print("\nDistortion Coefficients:\n", dist_coeffs)

    return camera_matrix, dist_coeffs

# Example usage:
TELLO_NO = 'E920EB_480P'
camera_matrix, dist_coeffs = calibrate_camera(TELLO_NO)

if camera_matrix is not None and dist_coeffs is not None:
    print("Calibration successful!")
else:
    print("Calibration failed.")