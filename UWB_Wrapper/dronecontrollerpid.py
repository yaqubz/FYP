"""
Testing 17 Feb
python -m UWB_Wrapper.dronecontrollerpid2
"""


import threading
import time, math
import logging
from simple_pid import PID
from .UWB_ReadUDP import get_target_position
from UnknownArea_v2.customtello import CustomTello, MockTello

file_handler = logging.FileHandler("log_pid.log", mode='w')  # Log to a file (overwrite each run)
console_handler = logging.StreamHandler()  # Log to the terminal

formatter = logging.Formatter("%(levelname)s - %(asctime)s - %(message)s")
file_handler.setFormatter(formatter)
console_handler.setFormatter(formatter)

logging.basicConfig(level=logging.DEBUG, handlers=[file_handler, console_handler])

NETWORK_CONFIG = {
    'host': '192.168.10.1',  # if connected directly through WiFi
    'control_port': 8889,
    'state_port': 8890,
    'video_port': 11111    
}

class DroneController:
    def __init__(self, network_config, laptop_only=False):
        # Initialize Tello
        self.drone = MockTello() if laptop_only else CustomTello(network_config)
        self.drone.connect()
        logging.info(f"Start Battery Level: {self.drone.get_battery()}%")
        self.drone.streamon()

        self.x, self.y = 0, 0  # UWB Position
        self.target_x, self.target_y = None, None  # Target coordinates

        # Get initial yaw and store it as the offset
        self.yaw_offset = self.drone.get_yaw()
        logging.info(f"Yaw Offset (Initial Yaw): {self.yaw_offset}")

        # PID Controllers
        self.kp_xy = 1.5 
        self.kp_yaw = 2 
        self.max_velocity = 30  

        self.running = True
        self.update_thread = threading.Thread(target=self.update_position)
        self.update_thread.start()

        self.calibrate_initial_yaw()

        # PID Controllers (separate for each axis)
        self.pid_x = PID(Kp=1.5, Ki=0.1, Kd=0.2, setpoint=0)
        self.pid_y = PID(Kp=1.5, Ki=0.1, Kd=0.2, setpoint=0)
        self.pid_yaw = PID(Kp=2.0, Ki=0.1, Kd=0.3, setpoint=0)
        
        # Configure PIDs
        for pid in [self.pid_x, self.pid_y, self.pid_yaw]:
            pid.output_limits = (-self.max_velocity, self.max_velocity)
            pid.sample_time = 0.1  # Matches control loop time

        self.last_uwb_update = time.time()
        self.position_history = []  # For filtering sudden jumps
    
    def calibrate_initial_yaw(self):
        """Improved yaw calibration with multiple measurements."""
        logging.info("Calibrating initial yaw...")
        
        # Collect initial position
        initial_positions = []
        for _ in range(5):
            pos = get_target_position(target_id=1)
            if pos:
                initial_positions.append(pos)
            time.sleep(0.1)
        
        if not initial_positions:
            raise RuntimeError("Cannot calibrate: No UWB data")
            
        x0, y0 = map(lambda x: sum(x)/len(x), zip(*initial_positions))
        
        # Move forward and collect positions
        self.drone.send_rc_control(30, 0, 0, 0)
        time.sleep(2)
        
        final_positions = []
        for _ in range(5):
            pos = get_target_position(target_id=1)
            if pos:
                final_positions.append(pos)
            time.sleep(0.1)
            
        self.drone.send_rc_control(0, 0, 0, 0)
        
        if not final_positions:
            raise RuntimeError("Cannot calibrate: No UWB data after movement")
            
        x1, y1 = map(lambda x: sum(x)/len(x), zip(*final_positions))
        
        # Calculate yaw offset
        dx, dy = x1 - x0, y1 - y0
        self.yaw_offset = math.degrees(math.atan2(dy, dx))
        logging.info(f"Yaw Offset Estimated: {self.yaw_offset}°")


    def update_position(self):
        """Update position with basic filtering."""
        while self.running:
            try:
                uwb_position = get_target_position(target_id=1)
                current_time = time.time()
                
                if uwb_position:
                    # Check for sudden position jumps
                    if self.position_history:
                        last_x, last_y = self.position_history[-1]
                        dx = uwb_position[0] - last_x
                        dy = uwb_position[1] - last_y
                        distance = math.sqrt(dx*dx + dy*dy)
                        
                        if distance > 0.5:  # More than 0.5m change
                            logging.warning(f"Suspicious position jump detected: {distance}m")
                            continue
                    
                    self.x, self.y = uwb_position
                    self.position_history.append((self.x, self.y))
                    self.position_history = self.position_history[-10:]  # Keep last 10 positions
                    self.last_uwb_update = current_time
                    
                elif current_time - self.last_uwb_update > 1.0:  # No UWB data for 1 second
                    logging.error("UWB data timeout")
                    self.drone.send_rc_control(0, 0, 0, 0)  # Emergency stop
                
                # Update yaw with filtering
                raw_yaw = self.drone.get_yaw()
                self.yaw = (raw_yaw - self.yaw_offset + 180) % 360 - 180
                
                time.sleep(0.05)
                
            except Exception as e:
                logging.error(f"Position update error: {e}")
                self.drone.send_rc_control(0, 0, 0, 0)

    def go_to_target_PID(self, target_x, target_y, margin_metres=0.05, timeout=30):
        """Go to target with timeout and better error handling."""
        self.target_x, self.target_y = target_x, target_y
        start_time = time.time()
        
        while True:
            try:
                self.compute_control()
                
                # Check if target reached
                if abs(self.x - target_x) < margin_metres and abs(self.y - target_y) < margin_metres:
                    logging.info("Target reached!")
                    break
                    
                # Check timeout
                if time.time() - start_time > timeout:
                    logging.warning("Target approach timeout")
                    break
                    
                time.sleep(0.1)
                
            except Exception as e:
                logging.error(f"Target approach error: {e}")
                self.drone.send_rc_control(0, 0, 0, 0)
                break

    def compute_control(self):
        """Compute control signals using PID controllers."""
        if self.target_x is None or self.target_y is None:
            return

        try:
            # Calculate errors in global frame
            error_x = self.target_x - self.x
            error_y = self.target_y - self.y
            
            # Convert to drone's local frame
            theta = math.radians(self.yaw)
            local_x = error_x * math.cos(theta) + error_y * math.sin(theta)
            local_y = -error_x * math.sin(theta) + error_y * math.cos(theta)
            
            # Compute desired heading
            desired_yaw = math.degrees(math.atan2(error_y, error_x))
            yaw_error = (desired_yaw - self.yaw + 180) % 360 - 180
            
            # Update PID controllers
            vx = self.pid_x(local_x)
            vy = self.pid_y(local_y)
            yaw_rate = self.pid_yaw(yaw_error)
            
            # Apply commands
            self.drone.send_rc_control(int(vx), int(vy), 0, int(yaw_rate))
            
            logging.debug(f"Control: vx={vx:.2f}, vy={vy:.2f}, yaw_rate={yaw_rate:.2f}")
            
        except Exception as e:
            logging.error(f"Control computation error: {e}")
            self.drone.send_rc_control(0, 0, 0, 0)


    def stop(self):
        """ Stop the controller. """
        self.running = False
        self.update_thread.join()


def main():
    controller = DroneController(NETWORK_CONFIG)

    controller.drone.takeoff()
    time.sleep(2)  # Allow drone to stabilize

    controller.go_to_target_PID(5, 5)  # Move to (5,5)

    controller.drone.land()
    controller.stop()

if __name__ == "__main__":
    main()
