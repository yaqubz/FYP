# 17 Feb - swarmserver = markerserver_v2
# Need to forcefully close terminal
# must run server BEFORE running clients

"""
python -m markerserver.swarmserver
"""

import logging

# Logging handlers and format (cannot put in main?)
file_handler = logging.FileHandler("log_markerserver.log", mode='w')  # Log to a file (overwrite each run)
console_handler = logging.StreamHandler()  # Log to the terminal
formatter = logging.Formatter("%(levelname)s - %(asctime)s - %(message)s")
file_handler.setFormatter(formatter)
console_handler.setFormatter(formatter)
logging.basicConfig(level=logging.DEBUG, handlers=[file_handler, console_handler])

from .swarmserverclient import MarkerServer

if __name__ == "__main__":
    
    server = MarkerServer()
    server.run()