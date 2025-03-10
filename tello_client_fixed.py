import socket
import threading
import time
import cv2
import numpy as np

class ProxyTello:
    """
    Lightweight Tello client that works with the TelloProxy server
    to enable control of multiple drones on the same network.
    """
    
    def __init__(self, proxy_ip='127.0.0.1', proxy_cmd_port=9000, 
                 proxy_state_port=9001, proxy_video_port=9002):
        # Store proxy configuration
        self.proxy_ip = proxy_ip
        self.proxy_cmd_port = proxy_cmd_port
        self.proxy_state_port = proxy_state_port
        self.proxy_video_port = proxy_video_port
        
        # Connection state
        self.connected = False
        self.stream_on = False
        
        # For state data
        self.state_data = {}
        self.state_socket = None
        self.state_thread = None
        
        # For video data
        self.video_socket = None
        self.video_thread = None
        self.frame = None
        self.frame_ready = False
        
        # Command and response handling
        self.cmd_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cmd_socket.bind(('0.0.0.0', 0))  # Bind to any available port
        self.cmd_socket.settimeout(5)  # 5 second timeout for commands
    
    def connect(self):
        """Connect to the drone via the proxy"""
        try:
            # Send the 'command' command to enter SDK mode
            response = self.send_command('command')
            
            if response == 'ok':
                self.connected = True
                print(f"Successfully connected to Tello via proxy at {self.proxy_ip}")
                
                # Register for state updates
                self._register_state()
                
                return True
            else:
                print(f"Failed to connect: {response}")
                return False
                
        except Exception as e:
            print(f"Error connecting to Tello: {str(e)}")
            return False
    
    def send_command(self, command):
        """Send a command to the drone and wait for a response"""
        try:
            # Send the command to the proxy
            self.cmd_socket.sendto(command.encode(), (self.proxy_ip, self.proxy_cmd_port))
            
            # Wait for response
            try:
                response, _ = self.cmd_socket.recvfrom(1024)
                return response.decode('utf-8').strip()
            except socket.timeout:
                return "timeout"
                
        except Exception as e:
            print(f"Error sending command '{command}': {str(e)}")
            return f"error: {str(e)}"
    
    def _register_state(self):
        """Register to receive state updates"""
        if self.state_socket:
            # Already registered
            return
            
        try:
            # Create socket for state data
            self.state_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.state_socket.bind(('0.0.0.0', 0))  # Bind to any available port
            
            # Register with the proxy
            self.state_socket.sendto(b'register', (self.proxy_ip, self.proxy_state_port))
            
            # Wait for confirmation
            self.state_socket.settimeout(5)
            try:
                data, _ = self.state_socket.recvfrom(1024)
                if data == b'registered':
                    print("Successfully registered for state updates")
                else:
                    print(f"Unexpected registration response: {data}")
            except socket.timeout:
                print("Timeout waiting for state registration")
            
            # Reset timeout
            self.state_socket.settimeout(None)
            
            # Start receiver thread
            self.state_thread = threading.Thread(target=self._state_receiver)
            self.state_thread.daemon = True
            self.state_thread.start()
            
        except Exception as e:
            print(f"Error setting up state updates: {str(e)}")
    
    def _state_receiver(self):
        """Thread to receive and parse state data"""
        while self.state_socket:
            try:
                data, _ = self.state_socket.recvfrom(1024)
                
                # Parse state data (format: mid:x;x:y;z:xy;...)
                state_text = data.decode('utf-8').strip()
                state_pairs = state_text.split(';')
                
                for pair in state_pairs:
                    if ':' in pair:
                        key, value = pair.split(':', 1)
                        self.state_data[key] = value
                
            except Exception as e:
                print(f"Error receiving state data: {str(e)}")
                time.sleep(0.1)
    
    def get_battery(self):
        """Get the current battery percentage"""
        if 'bat' in self.state_data:
            try:
                return int(self.state_data['bat'])
            except:
                pass
                
        # If no state data or invalid, query directly
        response = self.send_command('battery?')
        try:
            return int(response)
        except:
            return 0
    
    def takeoff(self):
        """Command the drone to take off"""
        return self.send_command('takeoff') == 'ok'
    
    def land(self):
        """Command the drone to land"""
        return self.send_command('land') == 'ok'
    
    def streamon(self):
        """Enable video streaming"""
        if self.stream_on:
            return True
            
        result = self.send_command('streamon') == 'ok'
        
        if result:
            self.stream_on = True
            self._setup_video()
            
        return result
    
    def streamoff(self):
        """Disable video streaming"""
        if not self.stream_on:
            return True
            
        result = self.send_command('streamoff') == 'ok'
        
        if result:
            self.stream_on = False
            
            # Clean up video resources
            if self.video_socket:
                temp_socket = self.video_socket
                self.video_socket = None
                temp_socket.close()
                
        return result
    
    def _setup_video(self):
        """Set up video streaming"""
        if self.video_socket:
            # Already set up
            return
            
        try:
            # Create socket for video data
            self.video_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.video_socket.bind(('0.0.0.0', 0))  # Bind to any available port
            
            # Register with the proxy
            self.video_socket.sendto(b'register', (self.proxy_ip, self.proxy_video_port))
            
            # Wait for confirmation
            self.video_socket.settimeout(5)
            try:
                data, _ = self.video_socket.recvfrom(1024)
                if data == b'registered':
                    print("Successfully registered for video stream")
                else:
                    print(f"Unexpected video registration response: {data}")
            except socket.timeout:
                print("Timeout waiting for video registration")
            
            # Reset timeout
            self.video_socket.settimeout(None)
            
            # Initialize frame processing
            self.frame = None
            self.frame_ready = False
            
            # Start the video receiver thread
            self.video_thread = threading.Thread(target=self._video_receiver)
            self.video_thread.daemon = True
            self.video_thread.start()
            
        except Exception as e:
            print(f"Error setting up video stream: {str(e)}")
    
    def _video_receiver(self):
        """Thread to receive video frames"""
        # This is a simplified version - in a real implementation, 
        # you would need proper H264 decoding
        import cv2
        
        # Set up video processor
        try:
            # Try to use hardware decoding if available
            proc = cv2.VideoCapture("udp://0.0.0.0:0", cv2.CAP_FFMPEG)
        except:
            print("Warning: Could not initialize hardware decoder, video may not work")
        
        while self.video_socket:
            try:
                # Receive the raw H264 data
                data, _ = self.video_socket.recvfrom(65536)
                
                # In a full implementation, you would need to:
                # 1. Buffer the data
                # 2. Parse H264 frames
                # 3. Decode using OpenCV or another library
                # 4. Set self.frame to the decoded frame
                
                # For now, just log that we received data
                print(f"Received {len(data)} bytes of video data")
                
                # Placeholder for now
                if self.frame is None:
                    # Create a placeholder frame
                    self.frame = np.zeros((720, 960, 3), dtype=np.uint8)
                    cv2.putText(self.frame, "Video stream active", (50, 360), 
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                
                self.frame_ready = True
                
            except Exception as e:
                print(f"Error processing video data: {str(e)}")
                time.sleep(0.01)
    
    def get_frame(self):
        """Get the latest video frame"""
        if not self.stream_on or self.frame is None:
            return None
            
        return self.frame.copy() if self.frame_ready else None
    
    def __del__(self):
        """Clean up resources"""
        try:
            if self.stream_on:
                self.streamoff()
                
            # Close all sockets
            if hasattr(self, 'cmd_socket') and self.cmd_socket:
                self.cmd_socket.close()
                
            if hasattr(self, 'state_socket') and self.state_socket:
                temp_socket = self.state_socket
                self.state_socket = None
                temp_socket.close()
                
            if hasattr(self, 'video_socket') and self.video_socket:
                temp_socket = self.video_socket
                self.video_socket = None
                temp_socket.close()
                
        except:
            pass


# Example usage
if __name__ == "__main__":

    # Control Drone1
    drone1 = ProxyTello(proxy_ip='127.0.0.1', proxy_cmd_port=9000, proxy_state_port=9001, proxy_video_port=9002)
    drone1.connect()
    drone1.takeoff()
    drone1.land()

    print("HERE")

    # Control Drone2
    drone2 = ProxyTello(proxy_ip='127.0.0.1', proxy_cmd_port=9010, proxy_state_port=9011, proxy_video_port=9012)
    drone2.connect()
    drone2.takeoff()
    drone2.land()
    # import argparse
    
    # parser = argparse.ArgumentParser(description='Tello Proxy Client')
    # parser.add_argument('--proxy-ip', type=str, default='127.0.0.1',
    #                     help='IP address of the proxy server')
    # parser.add_argument('--cmd-port', type=int, required=True,
    #                     help='Command port on the proxy server')
    # parser.add_argument('--state-port', type=int, required=True,
    #                     help='State port on the proxy server')
    # parser.add_argument('--video-port', type=int, required=True,
    #                     help='Video port on the proxy server')
    # parser.add_argument('--no-video', action='store_true',
    #                     help='Disable video streaming')
    
    # args = parser.parse_args()
    
    # # Connect to the drone via proxy
    # tello = ProxyTello(
    #     proxy_ip=args.proxy_ip,
    #     proxy_cmd_port=args.cmd_port,
    #     proxy_state_port=args.state_port,
    #     proxy_video_port=args.video_port
    # )
    
    # # Connect and check battery
    # if tello.connect():
    #     print(f"Battery: {tello.get_battery()}%")
        
    #     # Start video if requested
    #     if not args.no_video:
    #         if tello.streamon():
    #             print("Video stream started")
                
    #             # Simple video display loop
    #             try:
    #                 while True:
    #                     frame = tello.get_frame()
    #                     if frame is not None:
    #                         cv2.imshow('Tello Video', frame)
                        
    #                     if cv2.waitKey(1) & 0xFF == ord('q'):
    #                         break
    #             except KeyboardInterrupt:
    #                 print("Interrupted by user")
    #             finally:
    #                 cv2.destroyAllWindows()
    #                 tello.streamoff()
    #         else:
    #             print("Failed to start video stream")
        
    #     # If no video, just takeoff and land as a test
    #     else:
    #         print("Taking off...")
    #         if tello.takeoff():
    #             print("Takeoff successful")
    #             time.sleep(3)
    #             print("Landing...")
    #             tello.land()
    #         else:
    #             print("Takeoff failed")
    # else:
    #     print("Failed to connect to drone")