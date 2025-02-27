import logging

file_handler = logging.FileHandler("log.log", mode='w')  # Log to a file (overwrite each run)
console_handler = logging.StreamHandler()  # Log to the terminal

formatter = logging.Formatter("%(levelname)s - %(asctime)s - %(message)s")
file_handler.setFormatter(formatter)
console_handler.setFormatter(formatter)

logging.basicConfig(level=logging.DEBUG, handlers=[file_handler, console_handler])