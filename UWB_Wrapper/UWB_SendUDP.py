# WORKS 23 Jan: For testing purposes, run this script to send mock UWB positions for 3 drones via UDP. This can then be visualized on UWBViz.

import socket
import time
import math
import random
import threading

import socket
import time
import math
import random
import threading

"""
Tello local coordinates: x: +ve forward, y: +ve left
UWB global coordinates: x: +right, y: +ve up i.e. forward
"""

import socket
import time
import math
import random
import threading

class UWBPublisher:
    def __init__(self, ip='127.0.0.1', port=5000, tag_id=0):
        """Initialize the UDP publisher for simulated Tello drone position."""
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.target_address = (ip, port)
        self.time_start = time.time()
        self.tag_id = tag_id
        
        # Initialize drone state
        self.position = {'x': 0.0, 'y': 0.0, 'z': 0.0}  # Position in meters
        self.velocity = {'x': 0.0, 'y': 0.0, 'z': 0.0}  # Velocity in m/s
        self.yaw = 0.0  # Rotation in degrees (0 is facing UWB's +Y direction)
        
        # Physics parameters
        self.max_speed = 0.8  # Maximum speed in m/s (80 cm/s)
        self.acceleration = 0.2  # Acceleration in m/s²
        self.deceleration = 0.3  # Deceleration in m/s²
        
        # Thread control
        self.running = False
        self.lock = threading.Lock()
    
    def generate_distances(self):
        """Generate mock distance measurements to anchors."""
        return [random.uniform(100, 500) for _ in range(8)]
    
    def update_position(self, elapsed_time):
        """Update drone position based on current velocity."""
        with self.lock:
            # Update position based on velocity
            self.position['x'] += self.velocity['x'] * elapsed_time
            self.position['y'] += self.velocity['y'] * elapsed_time
            self.position['z'] += self.velocity['z'] * elapsed_time
            
            # Natural deceleration
            for axis in ['x', 'y', 'z']:
                if abs(self.velocity[axis]) > 0:
                    decel = min(self.deceleration * elapsed_time, abs(self.velocity[axis]))
                    self.velocity[axis] *= (1 - decel / abs(self.velocity[axis]))
                    
                    # Stop completely if velocity is very small
                    if abs(self.velocity[axis]) < 0.01:
                        self.velocity[axis] = 0
    
    def create_message(self):
        """Create a message with drone position data in the expected format."""
        with self.lock:
            # Format: id,role,x,y,z,dist1,dist2,dist3,dist4,dist5,dist6,dist7,dist8
            distances = self.generate_distances()
            line = f"{self.tag_id},0,{self.position['x']:.2f},{self.position['y']:.2f},{self.position['z']:.2f}"
            line += ',' + ','.join(f"{d:.2f}" for d in distances)
            return line.encode()
    
    def start_publishing(self, rate=30):
        """Start publishing in a separate thread."""
        self.running = True
        self.publisher_thread = threading.Thread(target=self._publish_loop, args=(rate,), daemon=False)
        self.publisher_thread.start()
        print(f"UWB Publisher started at {rate}Hz")
    
    def _publish_loop(self, rate=30):
        """Internal method to run the publishing loop."""
        period = 1.0 / rate
        last_update_time = time.time()
        
        try:
            while self.running:
                current_time = time.time()
                elapsed_time = current_time - last_update_time
                last_update_time = current_time
                
                self.update_position(elapsed_time)
                message = self.create_message()
                self.sock.sendto(message, self.target_address)
                print(message)
                
                time.sleep(period)
                
        except KeyboardInterrupt:
            print("\nStopping publisher...")
            self.sock.close()          
        except Exception as e:
            print(f"Error in UWB publisher: {e}")
        finally:
            print("UWB Publisher stopped")
    
    def stop_publishing(self):
        """Stop the publisher thread."""
        self.running = False
        if hasattr(self, 'publisher_thread') and self.publisher_thread.is_alive():
            self.publisher_thread.join(timeout=1.0)
    
    # Methods to update state based on drone commands
    def update_takeoff(self, height_cm=100):
        """Update position for takeoff command."""
        with self.lock:
            self.position['z'] = height_cm / 100.0  # Convert to meters
    
    def update_land(self):
        """Update position for land command."""
        with self.lock:
            self.position['z'] = 0.0
    
    def update_move_forward(self, distance_cm):
        """Update velocity for forward movement."""
        with self.lock:
            distance_m = distance_cm / 100.0
            # Calculate velocity components based on yaw
            yaw_rad = math.radians(self.yaw)
            # Forward in Tello's local X is +Y in UWB's global coordinates
            self.velocity['y'] += self.max_speed * math.cos(yaw_rad)
            # Left in Tello's local Y is -X in UWB's global coordinates
            self.velocity['x'] -= self.max_speed * math.sin(yaw_rad)
            # Simulate the time it takes to move the distance
            time.sleep(abs(distance_m) / self.max_speed)
    
    def update_rotate(self, angle):
        """Update yaw for rotation commands."""
        with self.lock:
            self.yaw = (self.yaw - angle) % 360  # Normalize to [0, 360)
            # Simulate the time it takes to rotate
            time.sleep(abs(angle) / 50)  # 50 degrees per second
    
    def update_rc_control(self, x, y, z, yaw):
        """Update velocity based on RC control commands."""
        with self.lock:
            # x: forward/backward (-100~100)
            # y: left/right (-100~100)
            # z: up/down (-100~100)
            # yaw: yaw (-100~100)
            
            # Calculate yaw in radians
            yaw_rad = math.radians(self.yaw)
            
            # Update velocities (scaled by max_speed)
            # Forward/backward component (Y-axis in UWB)
            self.velocity['y'] = (x / 100.0) * self.max_speed * math.cos(yaw_rad)
            # Left/right component (X-axis in UWB)
            self.velocity['x'] = -(x / 100.0) * self.max_speed * math.sin(yaw_rad)
            
            # Left/right component (X-axis in UWB)
            self.velocity['y'] += (y / 100.0) * self.max_speed * math.cos(yaw_rad + math.pi/2)
            self.velocity['x'] -= (y / 100.0) * self.max_speed * math.sin(yaw_rad + math.pi/2)
            
            # Up/down (Z-axis)
            self.velocity['z'] = (z / 100.0) * self.max_speed
            
            # Update yaw (accumulated)
            self.yaw += (yaw / 100.0) * 5  # 5 degrees per unit of yaw at full stick
            self.yaw = (self.yaw + 360) % 360  # Normalize to [0, 360)

    def get_position(self):
        """Get the current position."""
        with self.lock:
            return self.position.copy()
    
    def get_yaw(self):
        """Get the current yaw angle."""
        with self.lock:
            return self.yaw

class UWBPublisherSmurf(UWBPublisher):
    def __init__(self, ip='127.0.0.1', port=5000):
        """Initialize the UDP publisher for random motion."""
        super().__init__(ip, port)
        
        # Configuration for simulated tags
        self.num_tags = 3  # Number of tags to simulate
        self.tag_positions = {}  # Will store current positions of tags
        self.movement_speeds = {}  # Different speeds for each tag
        
        # Initialize tag positions and speeds
        for tag_id in range(self.num_tags):
            self.tag_positions[tag_id] = {
                'x': random.uniform(0, 10),  # Random starting positions
                'y': random.uniform(0, 10),
                'z': 0.0  # Keep z constant for 2D movement
            }
            self.movement_speeds[tag_id] = random.uniform(0.5, 2.0)  # Different speeds
    
    def update_position(self, elapsed_time):
        """Update position of all tags with circular motion."""
        elapsed_time_total = time.time() - self.time_start
        
        for tag_id in range(self.num_tags):
            speed = self.movement_speeds[tag_id]
            radius = 5.0  # Radius of circular motion
            center_x, center_y = 10, 10  # Center of circular motion
            
            # Calculate new position
            angle = (elapsed_time_total * speed) % (2 * math.pi)
            new_x = center_x + radius * math.cos(angle)
            new_y = center_y + radius * math.sin(angle)
            
            self.tag_positions[tag_id].update({
                'x': new_x,
                'y': new_y
            })
    
    def create_message(self):
        """Create a message with all tag data in the expected format."""
        message_lines = []
        
        for tag_id in range(self.num_tags):
            pos = self.tag_positions[tag_id]
            distances = self.generate_distances()
            
            # Format: id,role,x,y,z,dist1,dist2,dist3,dist4,dist5,dist6,dist7,dist8
            line = f"{tag_id},0,{pos['x']:.2f},{pos['y']:.2f},{pos['z']:.2f}"
            line += ',' + ','.join(f"{d:.2f}" for d in distances)
            message_lines.append(line)
        
        return '\n'.join(message_lines).encode()

if __name__ == "__main__":
    # Create and start publisher
    publisher = UWBPublisherSmurf()
    publisher.start_publishing()