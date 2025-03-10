from djitellopy import Tello
import socket

class CustomTello(Tello):
    def __init__(self, network_config):
        # Store custom configuration
        self.TELLO_IP = network_config['host']
        self.CONTROL_UDP_PORT = network_config['control_port']
        self.STATE_UDP_PORT = network_config['state_port']
        self.VS_UDP_PORT = network_config['video_port']
        
        # Override parent class attributes
        Tello.CONTROL_UDP_PORT = self.CONTROL_UDP_PORT
        Tello.STATE_UDP_PORT = self.STATE_UDP_PORT
        
        # Create and bind the control socket before calling parent's init
        self.socket_control = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket_control.bind(('0.0.0.0', self.CONTROL_UDP_PORT))
        
        # Call parent's init with our custom host
        super().__init__(self.TELLO_IP)
        
        # Replace the parent's control socket with our custom socket
        self.sock = self.socket_control
        
        # Override video port
        self.vs_udp_port = self.VS_UDP_PORT

# Network configuration
NETWORK_CONFIG = {
    'host': '192.168.0.122',     # Drone IP address
    'control_port': 9022,        # Custom control port
    'state_port': 8022,          # Custom state port
    'video_port': 11122          # Custom video port
}

# Initialize and connect to the drone
drone = CustomTello(network_config=NETWORK_CONFIG)
drone.connect()

# Test the connection
print(drone.get_battery())
drone.takeoff()
drone.end()