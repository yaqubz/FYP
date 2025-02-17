import importlib, sys, logging
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