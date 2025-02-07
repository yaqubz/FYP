# 7 Feb Works - run as 'python -m markerserver.server'
# Need to forcefully close terminal

import logging

# Logging handlers and format (cannot put in main?)
file_handler = logging.FileHandler("serverlog.log", mode='w')  # Log to a file (overwrite each run)
console_handler = logging.StreamHandler()  # Log to the terminal
formatter = logging.Formatter("%(levelname)s - %(asctime)s - %(message)s")
file_handler.setFormatter(formatter)
console_handler.setFormatter(formatter)
logging.basicConfig(level=logging.DEBUG, handlers=[file_handler, console_handler])

from .markerserverclient import MarkerServer

if __name__ == "__main__":
    
    server = MarkerServer()
    server.run()