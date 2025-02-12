# 6 Feb Testing - need to run this in the background as main()? or just put server.run() in the script such that always run when imported?
# Run in the background to reset

import threading, socket, json, time
import logging
from typing import Dict, Set, Any

class MarkerServer:
    def __init__(self, host='0.0.0.0', port=5005, timeout=5):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.marker_status: Dict[str, Dict[str, Any]] = {}
        self.clients: Set[tuple] = set()
        self.lock = threading.Lock()
        self.last_updates: Dict[str, float] = {}  # Track last update time for each marker
        
        # Create main socket for receiving broadcasts
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.sock.bind((self.host, self.port))
        
        # Create separate socket for sending broadcasts
        self.broadcast_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.broadcast_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.broadcast_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        logging.info(f"Marker server started on {self.host}:{self.port}")

    def check_timeouts(self):
        """Check for markers that haven't been updated and clear their detected status"""
        while True:
            try:
                current_time = time.time()
                status_changed = False
                
                with self.lock:
                    for marker_id in list(self.marker_status.keys()):
                        last_update = self.last_updates.get(marker_id, 0)
                        if current_time - last_update > self.timeout:
                            if marker_id in self.marker_status and self.marker_status[marker_id].get("detected", False):
                                logging.info(f"Clearing detected status for marker {marker_id} due to timeout")
                                self.marker_status[marker_id]["detected"] = False
                                status_changed = True
                
                if status_changed:
                    self.broadcast_status()  # Broadcast updates to all clients
                
                time.sleep(1)  # Check every second
            except Exception as e:
                logging.error(f"Error in check_timeouts: {e}")
                time.sleep(1)

    def update_marker_status(self, message):
        """Update marker status and timestamp"""
        with self.lock:
            try:
                marker_id = str(message["marker_id"])
                if marker_id != "-1" and marker_id is not None:
                    # Initialize marker status if not exists
                    if marker_id not in self.marker_status:
                        self.marker_status[marker_id] = {
                            "detected": False,
                            "landed": False
                        }
                    
                    # Update status
                    if "detected" in message:
                        self.marker_status[marker_id]["detected"] = message["detected"]
                        # Only update timestamp if marker is detected
                        if message["detected"]:
                            self.last_updates[marker_id] = time.time()
                    
                    if "landed" in message:
                        self.marker_status[marker_id]["landed"] = message["landed"]
                    
                    logging.debug(f"Updated marker {marker_id} status: {self.marker_status[marker_id]}")
            except Exception as e:
                logging.error(f"Error updating marker status: {e}")

    def handle_messages(self):
        while True:
            try:
                data, addr = self.sock.recvfrom(1024)
                try:
                    message = json.loads(data.decode())
                    logging.debug(f"Received message from {addr}: {message}")
                    
                    # Handle client registration message
                    if message.get("marker_id") == -1:
                        if addr not in self.clients:
                            with self.lock:
                                self.clients.add(addr)
                                logging.info(f"New client connected: {addr}")
                            self.broadcast_status_to_one(addr)
                    else:
                        self.update_marker_status(message)
                        self.broadcast_status()
                        
                except json.JSONDecodeError:
                    logging.warning(f"Received invalid JSON from {addr}: {data}")
                    
            except Exception as e:
                logging.error(f"Error handling message: {e}")

    def broadcast_status_to_one(self, addr):
        try:
            with self.lock:
                status_json = json.dumps(self.marker_status).encode()
                self.broadcast_sock.sendto(status_json, addr)
        except Exception as e:
            logging.error(f"Error broadcasting to {addr}: {e}")

    def broadcast_status(self):
        try:
            with self.lock:
                status_json = json.dumps(self.marker_status).encode()
                
                dead_clients = set()
                for client_addr in self.clients:
                    try:
                        self.broadcast_sock.sendto(status_json, client_addr)
                    except Exception as e:
                        logging.warning(f"Failed to send to client {client_addr}: {e}")
                        dead_clients.add(client_addr)
                
                # Remove dead clients
                self.clients -= dead_clients
                if dead_clients:
                    logging.info(f"Removed dead clients: {dead_clients}")
        except Exception as e:
            logging.error(f"Error in broadcast_status: {e}")

    def run(self):
        try:
            threads = [
                threading.Thread(target=self.handle_messages, daemon=True),
                threading.Thread(target=self.check_timeouts, daemon=True)
            ]
            for thread in threads:
                thread.start()
            
            # Keep main thread alive
            while True:
                time.sleep(1)
        except Exception as e:
            logging.error(f"Error in server run: {e}")


class MarkerClient:
    def __init__(self, server_port=5005, broadcast_ip="255.255.255.255"):
        self.server_port = server_port
        self.broadcast_ip = broadcast_ip
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)  # Enable broadcast
        self.sock.bind(("0.0.0.0", 0))  # Bind to any available port
        self.marker_status = {}
        threading.Thread(target=self.receive_updates, daemon=True).start()

        self.send_update(-1)  # Send an initial message to register with the server
        logging.info(f"Marker client broadcasting on {self.broadcast_ip}:{self.server_port}")
        print(f"Marker client broadcasting on {self.broadcast_ip}:{self.server_port}")

    def send_update(self, marker_id: int, detected=None, landed=None):
        if marker_id is not None:
            message = {"marker_id": marker_id}
            if detected is not None:
                message["detected"] = detected
            if landed is not None:
                message["landed"] = landed
            message_json = json.dumps(message).encode()

            for _ in range(5):  # Send the message 5 times for reliability
                self.sock.sendto(message_json, (self.broadcast_ip, self.server_port))
                time.sleep(0.03)  # Small delay to prevent flooding

            logging.debug(f"MarkerClient sent {message}")

    def receive_updates(self):  # Background thread
        while True:
            try:
                data, _ = self.sock.recvfrom(1024)
                self.marker_status = json.loads(data.decode())
                logging.debug(f"Received marker status from server: {self.marker_status}")
            except socket.timeout:  # Handle timeouts (if a timeout is set)
                pass
            except Exception as e:  # Catch any other exceptions
                logging.error(f"Error receiving updates: {e}")

    def is_marker_available(self, marker_id):
        marker_id = str(marker_id)  # Ensure it's a string to match dictionary keys
        marker_data = self.marker_status.get(marker_id)
        if marker_data is None:
            return True  # Marker has never been seen before -> Available
        detected = marker_data.get("detected", False)
        landed = marker_data.get("landed", False)
        return not (detected or landed)  # If either is True, it's NOT available

    def get_invalid_markers(self, markers_list: list) -> list:
        return [id for id in markers_list if not self.is_marker_available(id)]
    
if __name__ == "__main__":      # Usually should not run directly?
    file_handler = logging.FileHandler("log_markerserver.log", mode='w')  # Log to a file (overwrite each run)
    console_handler = logging.StreamHandler()  # Log to the terminal
    formatter = logging.Formatter("%(levelname)s - %(asctime)s - %(message)s")
    file_handler.setFormatter(formatter)
    console_handler.setFormatter(formatter)
    logging.basicConfig(level=logging.INFO, handlers=[file_handler, console_handler])
    server = MarkerServer()
    server.run()