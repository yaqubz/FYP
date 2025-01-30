# Camera Calibration Package
_(updated 30 Jan 2025)_
This package provides a standalone solution for calibrating the camera of a Tello drone. It is designed to be run from the main directory and supports multiple drones by adjusting filenames based on the drone identifier.

---

## **Steps to Use**

### 1. **Capture Calibration Images**
Run the `calib0` script to capture images while connected to the drone. The images will be saved in a subdirectory called `/calibration_images`.

###  2. Generate Calibration Parameters
Run the `calib1` script to process the captured images and generate the calibration parameters. These parameters will be saved as .npy files, which can be used in the main script.

## File Structure
* `/calibration_images`: Directory where captured images are stored.
* `/calib_camera`: Directory where the generated .npy files (camera matrix and distortion coefficients) are saved.
    * camera_matrix_tello{TELLO_NO}.npy
    * dist_coeffs_tello{TELLO_NO}.npy


## Integration with Main Script
To use the calibration parameters in your main script, call the `get_calibration_parameters` function. This function reads the .npy files based on the Tello drone identifier.

### Example Usage:
```python
import os
import numpy as np

def get_calibration_parameters(TELLO_NO: str = 'D'):
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
```
