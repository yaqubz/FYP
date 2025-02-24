# TESTING OK 14 FEB 


import sys
import importlib

def load_params():
    default_params = "params"       # default params.py
    print(len(sys.argv))
    if len(sys.argv) < 2:
        print("Load Params Usage: python script_name.py <params_module>. Trying default params.py.")
        params_module = default_params
    else:
        params_module = sys.argv[1]

    try:
        params = importlib.import_module(params_module)
        print(f"Loaded parameters from {params_module}:")
        print(params.some_setting)  # Assuming the params file has `some_setting`
    except ModuleNotFoundError:
        print(f"Error: Module {params_module} not found. Exiting script.")
        sys.exit(1)