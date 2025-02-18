import importlib, sys, logging, os
import numpy as np
from typing import Optional

def load_params():
    """
    Loads a params.py file via the command line. Allows multiple drones to run on the same script.
    Example usage: 
        python -m UnknownArea_v2.main UnknownArea_v2.params2

    Returns:
        params: The module containing configuration parameters
    """
    default_params = "shared_params.params"
    if len(sys.argv) < 2:
        print(f"Load Params Usage: python script_name.py params_module. Trying default {default_params}.")  
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


CAMERA_MATRIX, DIST_COEFF = get_calibration_parameters()
print("Calibration parameters obtained from shared_utils")
