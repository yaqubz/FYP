import cv2
import numpy as np
from djitellopy import Tello
import time
import logging  # in decreasing log level: debug > info > warning > error > critical
import threading

class CustomTello(Tello):
    def __init__(self, network_config):
        # Store custom configuration
        self.TELLO_IP = network_config['host']
        self.CONTROL_UDP_PORT = network_config['control_port']
        self.STATE_UDP_PORT = network_config['state_port']
        self.VS_UDP_PORT = network_config['video_port']
        
        Tello.STATE_UDP_PORT = self.STATE_UDP_PORT
        Tello.CONTROL_UDP_PORT = self.CONTROL_UDP_PORT
        
        # Call parent's init with our custom host
        super().__init__(self.TELLO_IP)
        
        # Override the connection parameters
        self.address = (self.TELLO_IP, self.CONTROL_UDP_PORT)
        
        # Override video port
        self.vs_udp_port = self.VS_UDP_PORT
    
    def go_to_height(self, height: int) -> None:
        """
        Custom Tello function to go to exact height, works caa 6 Feb (Gab)
        :param: height: Desired height in cm
        :return: None
        """
        current_height_tof = self.get_distance_tof()
        current_height_ = self.get_height()
        diff = height - current_height_tof
        
        if diff >= 20:
            self.move_up(diff)  # diff is already positive
        elif diff <= -20:  # Changed from abs(diff) <= -20
            self.move_down(abs(diff))  # need abs() since move_down needs positive value
        elif abs(diff) < 5:    # Close enough, accepted
            print(f"Height diff {diff}cm is less than 5cm. Moving on.")
        elif abs(diff) < 20:    # Resets, recurses
            self.move_up(30)
            self.go_to_height(height)
                
        final_height_tof = self.get_distance_tof()
        print(f"go_to_height {height}cm complete. Final height {final_height_tof}cm.")
            
    def go_to_height_PID(self, target_height: int, timeout: float = 10.0) -> bool:      # TESTING 5 FEB
        """
        Control drone height using PID control until target height is reached within tolerance
        
        Args:
            target_height: Desired height in cm
            timeout: Maximum time to attempt height control in seconds
        Returns:
            bool: True if target height was achieved, False if timeout occurred
        """
        # PID constants - these may need tuning
        Kp = 0.3  # Proportional gain
        Ki = 0.1  # Integral gain
        Kd = 0.1  # Derivative gain
        
        # Control parameters
        tolerance = 5  # Acceptable error in cm
        min_speed = 20  # Minimum speed for drone movement
        max_speed = 100  # Maximum speed for drone movement
        dt = 0.1  # Time step in seconds
        
        # Initialize PID variables
        integral = 0
        prev_error = 0
        start_time = time.time()
        
        while True:
            # Check timeout
            if time.time() - start_time > timeout:
                logging.warning(f"Height control timeout after {timeout} seconds")
                return False
                
            # Get current height
            current_height = self.get_distance_tof()
            error = target_height - current_height
            
            # Check if we're within tolerance
            if abs(error) < tolerance:
                logging.info(f"Target height achieved. Current: {current_height}cm, Target: {target_height}cm")
                self.send_rc_control(0,0,0,0)
                return True
                
            # Calculate PID terms
            integral += error * dt
            derivative = (error - prev_error) / dt
            
            # Calculate control output
            output = (Kp * error) + (Ki * integral) + (Kd * derivative)
            
            # Convert output to speed command
            speed = int(abs(output))
            speed = max(min_speed, min(speed, max_speed))  # Clamp speed between min and max
            logging.debug(f"Output: {output}, Speed: {speed}")

            # Apply control
            try:
                if output > 0:
                    if speed >= min_speed:
                        self.send_rc_control(0,0,speed,0)
                else:
                    if speed >= min_speed:
                        self.send_rc_control(0,0,-speed,0)
            except Exception as e:
                logging.error(f"Error during height adjustment: {e}")
                return False
                
            # Update previous error
            prev_error = error
            
            # Small delay to prevent overwhelming the drone
            time.sleep(dt)
            
            # Log progress
            logging.info(f"Current height: {current_height}cm, Error: {error}cm, Speed: {speed}")
    
    def get_ext_tof(self) -> int:   # TBC 5 FEB should put under CustomTello instead of Controller?
        """Get ToF sensor reading"""
        response = self.send_read_command("EXT tof?")
        try:
            return int(response.split()[1])
        except ValueError or IndexError as e:    # IndexError if using threading and the response is not as intended
            logging.debug(f"get_ext_tof raises error: {e}. Returning 8888.")
            return 8888

class MockFrameReader:
    def __init__(self, stream):
        self.stream = stream
        self.frame = None
        self.running = True  # Flag to control the thread
        self.lock = threading.Lock()  # Ensure thread-safe updates

        # Start a background thread to constantly update the frame
        self.thread = threading.Thread(target=self._update_frame, daemon=True)
        self.thread.start()

    def _update_frame(self):
        """Continuously update the latest frame in a separate thread."""
        while self.running:
            ret, frame = self.stream.read()
            if ret:
                with self.lock:  # Ensure thread-safe frame updates
                    self.frame = frame
            else:
                logging.warning("Failed to read frame from webcam.")
            time.sleep(0.03)  # ~30 FPS simulation

    def stop(self):
        """Stop the frame update thread."""
        self.running = False
        self.thread.join()

class MockTello:
    def __init__(self):
        self.stream = None  # Video capture object
        self.stream_on = False  # Stream state

    def connect(self):
        print("Mock: Drone connected.")
    
    def get_battery(self):
        print("Mock: Battery at 100%")
        return 100
    
    def set_mission_pad_detection_direction(self, direction):
        print(f"Mock: Set mission pad detection direction to {direction}")
    
    def takeoff(self):
        print("Mock: Taking off...")
    
    def rotate_clockwise(self, angle):
        print(f"Mock: Rotating clockwise {angle} degrees")
    
    def rotate_counter_clockwise(self, angle):
        print(f"Mock: Rotating counter-clockwise {angle} degrees")
    
    def move_forward(self, distance):
        print(f"Mock: Moving forward {distance} cm")

    def get_mission_pad_id(self):
        return -1   # no pad found
    
    def go_to_height(self, height: int):
        print(f"Mock: Moving up to {height} cm")

    def go_to_height_PID(self, height: int):
        print(f"Mock: Moving up with PID to {height} cm")

    def send_rc_control(*args): # dk how to fix 7 feb
        print(f"Mock: Received rc command {args}")

    def get_distance_tof(self) -> int:
        print("Mock: Downward ToF = 130cm")
        return 130

    def get_ext_tof(self) -> int:
        print("Mock: Ext ToF = 650mm")
        return 650
    
    def streamon(self):
        """Start video stream from laptop webcam."""
        self.stream = cv2.VideoCapture(0)  # Open default camera (0)
        if not self.stream.isOpened():
            raise Exception("Could not open webcam")
        self.stream_on = True
        print("MockTello: Webcam stream started.")

    def streamoff(self):
        """Stop video stream."""
        if hasattr(self, "frame_reader"):  # Stop the frame reading thread
            self.frame_reader.stop()
        if self.stream is not None:
            self.stream.release()
        self.stream_on = False
        print("MockTello: Webcam stream stopped.")

    def get_frame_read(self):
        """Return a frame reader object that mimics Tello's BackgroundFrameRead."""
        if not self.stream_on or self.stream is None:
            raise Exception("Stream is not started. Call streamon() first.")
        
        self.frame_reader = MockFrameReader(self.stream)  # Start the frame reader
        return self.frame_reader  # Return an object with `.frame`

    @property
    def is_flying(self):
        return True
    
    def land(self):
        print("Mock: Landing...")

    def end(self):
        print("Mock: Landing and Ending...")

    def __del__(self):
        """Ensure the webcam is released when the object is deleted."""
        self.streamoff()