## WORKS 17 Jan: Connects to Tello via RPi, and after setting up port forwarding, will stream a video via FFMPEG
## (see Tab 2 in https://docs.google.com/document/d/1KmyQqbMvM87TkZBuy-VgEdIVLpgY043QIXMpsIDNSzA/edit?usp=sharing)
## For video stream, run: ffplay -f h264 udp://127.0.0.1:11111

from djitellopy import Tello
import time

# Define configuration constants
NETWORK_CONFIG = {
    'host': '192.168.0.117',
    'control_port': 8889,
    'state_port': 8890,
    'video_port': 11111
}

class CustomTello(Tello):
    def __init__(self, network_config):
        # Store custom configuration; Treats RPi as the drone. All communication is with the RPi only
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

def main():
    # Use the centralized network configuration
    controller = CustomTello(NETWORK_CONFIG)
    controller.connect()
    time.sleep(5)
    
    print(f"Battery Level: {controller.get_battery()}%")

    
    controller.streamon()
    time.sleep(5)
    # controller.takeoff()
    # controller.land()
    controller.end() 



if __name__ == "__main__":
    main()