# 23 Jan - Purely a battery checker, but can also check over RPi

from djitellopy import Tello

NETWORK_CONFIG = {
    'host': '192.168.10.111',
    # 'host': '192.168.10.117',
    'control_port': 9011,
    'state_port': 8011,
    'video_port': 11111
}

class CustomTello(Tello):
    def __init__(self, network_config):
        """
        Store custom configuration; Treats RPi as the drone. All communication is with the RPi only
        """
        self.TELLO_IP = network_config['host']
        self.CONTROL_UDP_PORT = network_config['control_port']
        self.STATE_UDP_PORT = network_config['state_port']
        self.VS_UDP_PORT = network_config['video_port']
        Tello.STATE_UDP_PORT = self.STATE_UDP_PORT
        Tello.CONTROL_UDP_PORT = self.CONTROL_UDP_PORT
        super().__init__(self.TELLO_IP)
        self.address = (self.TELLO_IP, self.CONTROL_UDP_PORT)
        self.vs_udp_port = self.VS_UDP_PORT


def main():
    tello = CustomTello(NETWORK_CONFIG)
    tello.connect()
    print(tello.get_battery())
    tello.end() 



if __name__ == "__main__":
    main()
