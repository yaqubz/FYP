import socket
import time
import math
import random

class UWBPublisher:
    def __init__(self, ip='127.0.0.1', port=5000):
        """Initialize the UDP publisher."""
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.target_address = (ip, port)
        self.time_start = time.time()
        
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
    
    def generate_distances(self):
        """Generate mock distance measurements."""
        return [random.uniform(100, 500) for _ in range(8)]
    
    def update_position(self, tag_id, elapsed_time):
        """Update position of a tag with circular motion."""
        speed = self.movement_speeds[tag_id]
        radius = 5.0  # Radius of circular motion
        center_x, center_y = 10, 10  # Center of circular motion
        
        # Calculate new position
        angle = (elapsed_time * speed) % (2 * math.pi)
        new_x = center_x + radius * math.cos(angle)
        new_y = center_y + radius * math.sin(angle)
        
        self.tag_positions[tag_id].update({
            'x': new_x,
            'y': new_y
        })
    
    def create_message(self):
        """Create a message with all tag data in the expected format."""
        elapsed_time = time.time() - self.time_start
        message_lines = []
        
        for tag_id in range(self.num_tags):
            # Update position for this tag
            self.update_position(tag_id, elapsed_time)
            pos = self.tag_positions[tag_id]
            distances = self.generate_distances()
            
            # Format: id,role,x,y,z,dist1,dist2,dist3,dist4,dist5,dist6,dist7,dist8
            line = f"{tag_id},0,{pos['x']:.2f},{pos['y']:.2f},{pos['z']:.2f}"
            line += ',' + ','.join(f"{d:.2f}" for d in distances)
            message_lines.append(line)
        
        return '\n'.join(message_lines).encode()
    
    def publish(self, rate=30):
        """Publish data at specified rate (Hz)."""
        period = 1.0 / rate
        print(f"Publishing UWB position data at {rate}Hz...")
        
        try:
            while True:
                message = self.create_message()
                self.sock.sendto(message, self.target_address)
                print(message)
                time.sleep(period)
                
        except KeyboardInterrupt:
            print("\nStopping publisher...")
            self.sock.close()

if __name__ == "__main__":
    # Create and start publisher
    publisher = UWBPublisher()
    publisher.publish()