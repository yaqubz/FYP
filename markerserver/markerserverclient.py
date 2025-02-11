# 6 Feb Testing - need to run this in the background as main()? or just put server.run() in the script such that always run when imported?
# Run in the background to reset

import threading, socket, json, time
import logging  # To take on the logging config of the parent script!

class MarkerServer:
    def __init__(self, host='0.0.0.0', port=5005):
        self.host = host
        self.port = port
        self.marker_status = {}  # {marker_id: {"detected": bool, "landed": bool}}
        self.lock = threading.Lock()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.host, self.port))
        logging.info(f"Marker server started on {self.host}:{self.port}")

    def accept_connections(self):
        while True:
            try:
                data, addr = self.sock.recvfrom(1024)  # Wait for initial data
                if addr not in self.clients:
                    self.clients.add(addr)
                    logging.info(f"New client connected: {addr}")
                    # Immediately broadcast the current status to the new client
                    self.broadcast_status_to_one(addr) #<--- Add this line
            except Exception as e:
                logging.error(f"Error accepting connection: {e}")  # Handle errors

    def handle_messages(self):
        while True:
            data, addr = self.sock.recvfrom(1024)
            message = json.loads(data.decode())
            logging.debug(f"MESSAGE: {message}")
            if message:
                self.update_marker_status(message)
            self.broadcast_status()

    def update_marker_status(self, message):
        """
        Updates internal class attribute.
        """
        with self.lock:
            marker_id = message["marker_id"]
            logging.debug(f"marker_id: {marker_id}")
            if not marker_id == -1 and not marker_id == None: 
                if marker_id not in self.marker_status:
                    # Nested dictionary with key-values pairs (Level 1: Marker ID; Level 2: Statuses)
                    self.marker_status[marker_id] = {"detected": False, "landed": False}
                if "detected" in message:
                    self.marker_status[marker_id]["detected"] = message["detected"]
                if "landed" in message:
                    self.marker_status[marker_id]["landed"] = message["landed"]
                logging.info(f"Marker Statuses: {self.marker_status}")

    def broadcast_status_to_one(self, addr): #<--- Add this function
        with self.lock:
            status_json = json.dumps(self.marker_status)
            self.sock.sendto(status_json.encode(), addr)

    def broadcast_status(self):
        with self.lock:
            status_json = json.dumps(self.marker_status)
            for addr in self.clients:
                self.sock.sendto(status_json.encode(), addr)

    def run(self):
        self.clients = set()
        threading.Thread(target=self.handle_messages, daemon=True).start()

        # Add a separate thread to handle client connections
        threading.Thread(target=self.accept_connections, daemon=True).start()

        # Keep the main thread alive (you might not need this if you have other tasks)
        while True:
            time.sleep(1)


class MarkerClient:
    def __init__(self, server_host='127.0.0.1', server_port=5005):
        self.server_host = server_host
        self.server_port = server_port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', 0))  # Bind to any available port
        self.marker_status = {}
        threading.Thread(target=self.receive_updates, daemon=True).start()

        self.send_update(-1) # Send an initial message to register with the server. -1 means nothing
        logging.info(f"Marker client connected to server {self.server_host}:{self.server_port}")
        print(f"Marker client connected to server  {self.server_host}:{self.server_port}")

    def send_update(self, marker_id:int, detected=None, landed=None):
        if not marker_id is None: 
            message = {"marker_id": marker_id}
            if detected is not None:
                message["detected"] = detected
            if landed is not None:
                message["landed"] = landed
            message_json = json.dumps(message).encode()

            for _ in range(3):  # Send the message 3 times
                self.sock.sendto(message_json, (self.server_host, self.server_port))
                time.sleep(0.05)  # Small delay to prevent flooding

            logging.debug(f"MarkerClient sent {message}")

    def receive_updates(self):  # background thread, does not print directly! DO NOT call this directly.
        while True:
            try:
                data, _ = self.sock.recvfrom(1024)
                self.marker_status = json.loads(data.decode())
                logging.debug(f"Received marker status from server: {self.marker_status}")
            except socket.timeout:  # Handle timeouts (if you set a timeout on the socket)
                pass
            except Exception as e: # Catch any other exceptions
                logging.error(f"Error receiving updates: {e}")

    def is_marker_available(self, marker_id):
        marker_id = str(marker_id)  # Ensure it's a string to match dictionary keys
        marker_data = self.marker_status.get(marker_id)  # Get the marker's data

        if marker_data is None:
            return True  # Marker has never been seen before -> Available

        detected = marker_data.get("detected", False)
        landed = marker_data.get("landed", False)

        return not (detected or landed)  # If either is True, it's NOT available
    
