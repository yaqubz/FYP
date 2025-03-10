import socket
from djitellopy import Tello

class CustomTello(Tello):
    def __init__(self, network_config):
        # Store custom configuration
        self.TELLO_IP = network_config['host']
        self.CONTROL_UDP_PORT = network_config['control_port']
        self.STATE_UDP_PORT = network_config['state_port']
        self.VS_UDP_PORT = network_config['video_port']
        self.LOCAL_CONTROL_PORT = network_config['local_control_port']
        
        # Override parent class attributes
        Tello.CONTROL_UDP_PORT = self.CONTROL_UDP_PORT
        Tello.STATE_UDP_PORT = self.STATE_UDP_PORT
        
        # Create and bind the control socket before calling parent's init
        self.socket_control = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket_control.bind(('0.0.0.0', self.LOCAL_CONTROL_PORT))
        
        # Call parent's init with our custom host
        super().__init__(self.TELLO_IP)
        
        # Replace the parent's control socket with our custom socket
        self.sock = self.socket_control
        
        # Override video port
        self.vs_udp_port = self.VS_UDP_PORT

# Network configurations for two drones
DRONE_1_CONFIG = {
    'host': '192.168.0.121',     # Drone 1 IP address
    'control_port': 8889,        # Drone 1 control port
    'state_port': 8890,          # Drone 1 state port
    'video_port': 11111,         # Drone 1 video port
    'local_control_port': 9021,  # Local control port for Drone 1
    'local_state_port': 8021,    # Local state port for Drone 1
    'local_video_port': 11121    # Local video port for Drone 1
}

DRONE_2_CONFIG = {
    'host': '192.168.0.122',     # Drone 2 IP address
    'control_port': 8889,        # Drone 2 control port
    'state_port': 8890,          # Drone 2 state port
    'video_port': 11111,         # Drone 2 video port
    'local_control_port': 9022,  # Local control port for Drone 2
    'local_state_port': 8022,    # Local state port for Drone 2
    'local_video_port': 11122    # Local video port for Drone 2
}

# Initialize and connect to the drones
# drone1 = CustomTello(network_config=DRONE_1_CONFIG)
drone2 = CustomTello(network_config=DRONE_2_CONFIG)

# drone1.connect()
drone2.connect()

# Take off both drones
# drone1.takeoff()
drone2.takeoff()

# Land both drones
# drone1.land()
drone2.land()

# End the connection
# drone1.end()
drone2.end()