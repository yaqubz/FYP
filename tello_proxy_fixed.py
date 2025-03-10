import socket
import threading
import time
import sys
import argparse
import logging
from queue import Queue, Empty

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("TelloProxy")

class TelloProxy:
    """
    Proxy server that maps multiple Tello drones with fixed ports to 
    unique port sets that client applications can connect to independently.
    """
    
    # Default Tello ports
    TELLO_CMD_PORT = 8889    # Port to send commands TO drone
    TELLO_STATE_PORT = 8890  # Port drone sends state data FROM
    TELLO_VIDEO_PORT = 11111 # Port drone sends video data FROM
    
    def __init__(self, drone_ip, drone_name, proxy_cmd_port, proxy_state_port, proxy_video_port, shared_state_socket=None, shared_video_socket=None):
        self.drone_ip = drone_ip
        self.drone_name = drone_name
        
        # Ports that client applications will connect to
        self.proxy_cmd_port = proxy_cmd_port
        self.proxy_state_port = proxy_state_port  
        self.proxy_video_port = proxy_video_port
        
        # Command socket (bidirectional)
        self.cmd_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cmd_socket.bind(('0.0.0.0', self.proxy_cmd_port))
        
        # Use shared sockets for state and video if provided (to avoid port conflicts)
        self.state_socket = shared_state_socket
        self.video_socket = shared_video_socket
        
        # Create new sockets if not shared
        if self.state_socket is None:
            self.state_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                self.state_socket.bind(('0.0.0.0', self.TELLO_STATE_PORT))
                self.owns_state_socket = True
            except OSError:
                logger.warning(f"[{drone_name}] Could not bind to state port {self.TELLO_STATE_PORT}, state information will not be received")
                self.state_socket = None
                self.owns_state_socket = False
        else:
            self.owns_state_socket = False
        
        if self.video_socket is None:
            self.video_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                self.video_socket.bind(('0.0.0.0', self.TELLO_VIDEO_PORT))
                self.owns_video_socket = True
            except OSError:
                logger.warning(f"[{drone_name}] Could not bind to video port {self.TELLO_VIDEO_PORT}, video stream will not be received")
                self.video_socket = None
                self.owns_video_socket = False
        else:
            self.owns_video_socket = False
        
        # Client address cache (updated when we receive commands)
        self.cmd_client_addr = None
        self.state_clients = []
        self.video_clients = []
        
        # Running flag
        self.running = False
        
        logger.info(f"Initialized proxy for {drone_name} at {drone_ip}")
        logger.info(f"Command port: {proxy_cmd_port}")
        logger.info(f"State port: {proxy_state_port}")
        logger.info(f"Video port: {proxy_video_port}")
    
    def start(self):
        """Start the proxy server and all its forwarding threads"""
        self.running = True
        
        # Start the command handler (client → drone)
        self.cmd_thread = threading.Thread(target=self._handle_commands)
        self.cmd_thread.daemon = True
        self.cmd_thread.start()
        
        # Start the response handler (drone → client)
        self.resp_thread = threading.Thread(target=self._handle_responses)
        self.resp_thread.daemon = True
        self.resp_thread.start()
        
        # Start the state data handler (drone → client) if socket exists
        if self.state_socket:
            self.state_thread = threading.Thread(target=self._handle_state)
            self.state_thread.daemon = True
            self.state_thread.start()
        
        # Start the video data handler (drone → client) if socket exists
        if self.video_socket:
            self.video_thread = threading.Thread(target=self._handle_video)
            self.video_thread.daemon = True
            self.video_thread.start()
        
        # Start the client registration threads
        self.state_reg_thread = threading.Thread(target=self._register_state_clients)
        self.state_reg_thread.daemon = True
        self.state_reg_thread.start()
        
        self.video_reg_thread = threading.Thread(target=self._register_video_clients)
        self.video_reg_thread.daemon = True
        self.video_reg_thread.start()
        
        logger.info(f"Proxy for {self.drone_name} started")
    
    def stop(self):
        """Stop the proxy server"""
        self.running = False
        
        # Close sockets if we own them
        if hasattr(self, 'cmd_socket'):
            self.cmd_socket.close()
        
        if self.owns_state_socket and self.state_socket:
            self.state_socket.close()
            
        if self.owns_video_socket and self.video_socket:
            self.video_socket.close()
            
        logger.info(f"Proxy for {self.drone_name} stopped")
    
    def _handle_commands(self):
        """Forward commands from client to drone"""
        logger.info(f"[{self.drone_name}] Command handler started")
        
        while self.running:
            try:
                data, client_addr = self.cmd_socket.recvfrom(2048)
                
                # Update client address for responses
                self.cmd_client_addr = client_addr
                
                # Log the command
                try:
                    cmd_str = data.decode('utf-8').strip()
                    logger.info(f"[{self.drone_name}] Command from {client_addr}: {cmd_str}")
                except:
                    logger.info(f"[{self.drone_name}] Binary command from {client_addr}")
                
                # Forward to drone
                self.cmd_socket.sendto(data, (self.drone_ip, self.TELLO_CMD_PORT))
            except Exception as e:
                if self.running:  # Only log if still running
                    logger.error(f"[{self.drone_name}] Command handler error: {str(e)}")
                    time.sleep(0.1)
    
    def _handle_responses(self):
        """Forward command responses from drone to client"""
        logger.info(f"[{self.drone_name}] Response handler started")
        
        # Create a socket to receive responses from the drone
        resp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        resp_socket.bind(('0.0.0.0', 0))  # Bind to any free port
        
        # This is needed because the drone sends responses to the port that sent the command
        resp_port = resp_socket.getsockname()[1]
        logger.info(f"[{self.drone_name}] Listening for responses on port {resp_port}")
        
        while self.running:
            try:
                data, _ = resp_socket.recvfrom(2048)
                
                # Only forward if we have a client
                if self.cmd_client_addr:
                    try:
                        resp_str = data.decode('utf-8').strip()
                        logger.info(f"[{self.drone_name}] Response: {resp_str}")
                    except:
                        logger.info(f"[{self.drone_name}] Binary response")
                    
                    # Forward to the client
                    self.cmd_socket.sendto(data, self.cmd_client_addr)
            except Exception as e:
                if self.running:  # Only log if still running
                    logger.error(f"[{self.drone_name}] Response handler error: {str(e)}")
                    time.sleep(0.1)
    
    def _register_state_clients(self):
        """Listen for clients connecting to the state proxy port"""
        logger.info(f"[{self.drone_name}] State registration handler started")
        
        # Create a socket for state client registration
        state_reg_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        state_reg_socket.bind(('0.0.0.0', self.proxy_state_port))
        
        while self.running:
            try:
                # We don't care about the actual data, just the source address
                _, client_addr = state_reg_socket.recvfrom(128)
                
                if client_addr not in self.state_clients:
                    logger.info(f"[{self.drone_name}] New state client: {client_addr}")
                    self.state_clients.append(client_addr)
                
                # Send a dummy response to confirm registration
                state_reg_socket.sendto(b'registered', client_addr)
            except Exception as e:
                if self.running:  # Only log if still running
                    logger.error(f"[{self.drone_name}] State registration error: {str(e)}")
                    time.sleep(0.1)
    
    def _register_video_clients(self):
        """Listen for clients connecting to the video proxy port"""
        logger.info(f"[{self.drone_name}] Video registration handler started")
        
        # Create a socket for video client registration
        video_reg_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        video_reg_socket.bind(('0.0.0.0', self.proxy_video_port))
        
        while self.running:
            try:
                # We don't care about the actual data, just the source address
                _, client_addr = video_reg_socket.recvfrom(128)
                
                if client_addr not in self.video_clients:
                    logger.info(f"[{self.drone_name}] New video client: {client_addr}")
                    self.video_clients.append(client_addr)
                
                # Send a dummy response to confirm registration
                video_reg_socket.sendto(b'registered', client_addr)
            except Exception as e:
                if self.running:  # Only log if still running
                    logger.error(f"[{self.drone_name}] Video registration error: {str(e)}")
                    time.sleep(0.1)
    
    def _handle_state(self):
        """Forward state data from drone to all registered clients"""
        logger.info(f"[{self.drone_name}] State handler started")
        
        while self.running and self.state_socket:
            try:
                data, _ = self.state_socket.recvfrom(2048)
                
                # Determine if this is for our drone (based on source IP)
                # Assuming the state message comes from our drone's IP
                
                # Forward to all registered clients
                for client_addr in self.state_clients[:]:  # Use a copy of the list
                    try:
                        # Create a temporary socket for sending to avoid binding conflicts
                        tmp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                        tmp_socket.sendto(data, client_addr)
                        tmp_socket.close()
                    except Exception as e:
                        logger.error(f"[{self.drone_name}] Error sending state to {client_addr}: {str(e)}")
                        # Remove failed client
                        if client_addr in self.state_clients:
                            self.state_clients.remove(client_addr)
            except Exception as e:
                if self.running:  # Only log if still running
                    logger.error(f"[{self.drone_name}] State handler error: {str(e)}")
                    time.sleep(0.1)
    
    def _handle_video(self):
        """Forward video data from drone to all registered clients"""
        logger.info(f"[{self.drone_name}] Video handler started")
        
        while self.running and self.video_socket:
            try:
                data, addr = self.video_socket.recvfrom(65536)  # Larger buffer for video
                
                # Forward to all registered clients
                for client_addr in self.video_clients[:]:  # Use a copy of the list
                    try:
                        # Create a temporary socket for sending to avoid binding conflicts
                        tmp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                        tmp_socket.sendto(data, client_addr)
                        tmp_socket.close()
                    except Exception as e:
                        logger.error(f"[{self.drone_name}] Error sending video to {client_addr}: {str(e)}")
                        # Remove failed client
                        if client_addr in self.video_clients:
                            self.video_clients.remove(client_addr)
            except Exception as e:
                if self.running:  # Only log if still running
                    logger.error(f"[{self.drone_name}] Video handler error: {str(e)}")
                    time.sleep(0.1)


class TelloProxyManager:
    """Manager to handle multiple TelloProxy instances"""
    
    def __init__(self):
        self.proxies = {}
        
        # Create shared sockets for state and video (to solve port binding issues)
        try:
            self.shared_state_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.shared_state_socket.bind(('0.0.0.0', TelloProxy.TELLO_STATE_PORT))
            logger.info(f"Created shared state socket on port {TelloProxy.TELLO_STATE_PORT}")
        except OSError:
            logger.warning(f"Could not bind to state port {TelloProxy.TELLO_STATE_PORT}, will try individual bindings")
            self.shared_state_socket = None
        
        try:
            self.shared_video_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.shared_video_socket.bind(('0.0.0.0', TelloProxy.TELLO_VIDEO_PORT))
            logger.info(f"Created shared video socket on port {TelloProxy.TELLO_VIDEO_PORT}")
        except OSError:
            logger.warning(f"Could not bind to video port {TelloProxy.TELLO_VIDEO_PORT}, will try individual bindings")
            self.shared_video_socket = None
    
    def add_drone(self, drone_ip, drone_name, cmd_port, state_port, video_port):
        """Add a new drone proxy"""
        proxy = TelloProxy(
            drone_ip=drone_ip,
            drone_name=drone_name,
            proxy_cmd_port=cmd_port,
            proxy_state_port=state_port,
            proxy_video_port=video_port,
            shared_state_socket=self.shared_state_socket,
            shared_video_socket=self.shared_video_socket
        )
        self.proxies[drone_name] = proxy
        return proxy
    
    def start_all(self):
        """Start all proxies"""
        for name, proxy in self.proxies.items():
            proxy.start()
    
    def stop_all(self):
        """Stop all proxies"""
        for name, proxy in self.proxies.items():
            proxy.stop()
            
        # Close shared sockets
        if hasattr(self, 'shared_state_socket') and self.shared_state_socket:
            self.shared_state_socket.close()
            
        if hasattr(self, 'shared_video_socket') and self.shared_video_socket:
            self.shared_video_socket.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Tello Drone Proxy Server')
    parser.add_argument('--config', type=str, nargs='+', required=True,
                      help='Drone configs in format: name,ip,cmd_port,state_port,video_port')
    
    args = parser.parse_args()
    
    manager = TelloProxyManager()
    
    # Parse drone configs
    for config in args.config:
        parts = config.split(',')
        if len(parts) != 5:
            logger.error(f"Invalid config format: {config}")
            continue
        
        name = parts[0]
        ip = parts[1]
        cmd_port = int(parts[2])
        state_port = int(parts[3])
        video_port = int(parts[4])
        
        manager.add_drone(ip, name, cmd_port, state_port, video_port)
    
    # Start all proxies
    manager.start_all()
    
    try:
        # Keep the main thread alive
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        logger.info("Shutting down...")
        manager.stop_all()
        sys.exit(0)