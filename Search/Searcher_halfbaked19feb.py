"""
python -m Search.Searcher_halfbaked19feb shared_params.params


Issues 19 Feb:
- Drone does not end when CV2 window closes
- Yet to test IRL

"""

from shared_utils.dronecontroller2 import DroneController
from shared_utils.shared_utils import setup_logging, params
from .utils import *
import cv2
import threading
from threading import Lock, Event
import time
import logging
import sys

class VideoStreamHandler:
    """
    Takes DroneController as an argument! Handles the stream on a thread, and exchanges each frame with DroneController as needed.
    DroneController requests a frame from VideoStreamHandler for processing. 
    DroneController returns a display frame to VideoStreamHandler, which it displays.
    """
    def __init__(self, controller:DroneController, frame_reader):
        self.controller = controller
        self.frame_reader = frame_reader
        self.stop_event = Event()
        self.current_frame = None  # Store the latest frame
        self.display_frame = None  # For CV2 annotations
        self.frame_lock = Lock()   # Thread-safe access to frame
        
        # Add frame access to DroneController
        self.controller.set_video_handler(self)

    def start(self):
        self.thread = threading.Thread(target=self._stream_video)
        self.thread.daemon = True
        self.thread.start()
        
    def stop(self):
        self.stop_event.set()
        if self.thread.is_alive():
            self.thread.join(timeout=2)

    def get_current_frame(self):
        """Thread-safe method to get the latest frame"""
        with self.frame_lock:
            return self.current_frame.copy() if self.current_frame is not None else None
        
    def get_display_frame(self):
        """Thread-safe method to get the display frame"""
        with self.frame_lock:
            return self.display_frame.copy() if self.display_frame is not None else None
        
    def return_display_frame(self, returned_frame):
        """Thread-safe method to get annotated frame back from DroneController"""
        with self.frame_lock:
            if returned_frame is not None:
                logging.debug("NON_EMPTY RETURNED FRAME RECEIVED BY VIDEOSTREAMHANDLER")
            self.display_frame = returned_frame.copy() if returned_frame is not None else None
    
    def _stream_video(self):
        while not self.stop_event.is_set():
            try:
                frame = capture_frame(self.frame_reader)
                if frame is None:
                    continue
                    
                # Store frame thread-safely
                with self.frame_lock:
                    self.current_frame = frame.copy()

                # Get the frame to display
                display_frame = self.get_display_frame()
                if display_frame is not None:
                    cv2.imshow("Drone View", display_frame)

                # Keyboard shortcuts
                key = cv2.waitKey(1)
                if key != -1:  # Only process if a key was actually pressed
                    if key == ord('q'):
                        logging.info("Quit command received")
                        self.stop_event.set()
                        break
                    elif key == ord('l'):
                        logging.debug("DEBUG LOGPOINT")

            except Exception as e:
                logging.error(f"Error in video stream: {e}")
                time.sleep(0.1)
                continue
                
        self._cleanup()
        
    def _cleanup(self):
        cv2.destroyAllWindows()
        logging.info("Destroying all windows.")
        for i in range(4):
            cv2.waitKey(1)

        # TBC - is there a better way to cause the other class to end as well? (TODO 19 FEB)
        # TBC - should videostreamhandler be a daughter class of dronecontroller2?
        self.controller.drone.streamoff()
        self.controller.drone.land()
        self.controller.drone.end()
        sys.exit()

def main():
    logger = setup_logging(params, "UnknownSearchArea")
    logger.info("Starting unknown area main...")
    
    controller = None
    video_handler = None
    
    try:
        controller = DroneController(
            params.NETWORK_CONFIG, 
            drone_id=params.PI_ID, 
            laptop_only=params.LAPTOP_ONLY, 
            load_midas=False
        )
        
        # Initialize video stream
        frame_reader = controller.drone.get_frame_read()
        time.sleep(3)  # Allow time for stream initialization
        
        # Start video stream
        video_handler = VideoStreamHandler(controller, frame_reader)
        video_handler.start()
        
        if not params.NO_FLY:
            controller.takeoff_simul([11,12,13])
            logging.info("Taking off for real...")
            controller.drone.go_to_height_PID(100)
            controller.drone.send_rc_control(0, 0, 0, 0)
        else:
            logging.info("Simulating takeoff. Drone will NOT fly.")
            
        time.sleep(2)
        
        # Execute waypoints
        execute_waypoints_scan_land(controller, params.WAYPOINTS_JSON)
        
        logging.info(f"Landing. Final Battery Level: {controller.drone.get_battery()}%")
        
    except Exception as e:
        logging.error(f"Error in main: {e}")
        raise
        
    finally:    # Cleanup
        logging.info("Cleaning up.")
        if video_handler:
            video_handler.stop()
            
        if controller:
            try:
                controller.drone.streamoff()
                controller.drone.land()
                controller.drone.end()
            except Exception as e:
                logging.error(f"Error during cleanup: {e}")

if __name__ == "__main__":
    main()