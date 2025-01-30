import cv2
import numpy as np

TELLO_NO: str = 'D'

# Load saved calibration parameters
camera_matrix = np.load(f"calib_camera/camera_matrix_tello{TELLO_NO}.npy")
dist_coeffs = np.load(f"calib_camera/dist_coeffs_tello{TELLO_NO}.npy")

# Load an image to test
img = cv2.imread(f"calib_camera/calibration_images/tello{TELLO_NO}_img004.jpg")  # Replace with an actual test image
h, w = img.shape[:2]

# Undistort the image
new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w, h), 1, (w, h))
undistorted_img = cv2.undistort(img, camera_matrix, dist_coeffs, None, new_camera_matrix)

# Show the original and undistorted images
cv2.imshow("Original Image", img)
cv2.imshow("Undistorted Image", undistorted_img)
cv2.waitKey(0)
cv2.destroyAllWindows()