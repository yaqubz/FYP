## WORKS 17 Jan: Connects to Tello via RPi, and after setting up port forwarding, will stream a video via CV2
## Color grading is off, need to edit params (TBC)

from djitellopy import Tello
import cv2, time

# Define configuration constants
# NETWORK_CONFIG = {
#     'host': '192.168.0.117',
#     'control_port': 8889,
#     'state_port': 8890,
#     'video_port': 11111
# }

NETWORK_CONFIG = {
    'host': '192.168.10.1',
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
    tello = CustomTello(NETWORK_CONFIG)
    tello.connect()
    print(tello.get_battery())
    tello.send_command_with_return("downvision 0")
    tello.streamon()
    while True:
        frame = tello.get_frame_read().frame
        # img = cv2.resize(frame,(360,240))       # reduce size to reduce latency
        img = cv2.resize(frame,(648,488))
        cv2.imshow('Tello Forward Cam', img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    tello.end() 



if __name__ == "__main__":
    main()
