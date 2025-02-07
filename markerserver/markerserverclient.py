# 6 Feb Testing - need to run this in the background as main()? or just put server.run() in the script such that always run when imported?
# Run in the background to reset

import threading, socket, json
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

    def handle_messages(self):
        while True:
            data, addr = self.sock.recvfrom(1024)
            message = json.loads(data.decode())
            self.update_marker_status(message)
            self.broadcast_status()

    def update_marker_status(self, message):
        with self.lock:
            marker_id = message["marker_id"]
            if marker_id not in self.marker_status:
                # Nested dictionary with key-values pairs (Level 1: Marker ID; Level 2: Statuses)
                self.marker_status[marker_id] = {"detected": False, "landed": False}
                logging.debug(f"{marker_id} not detected, not landed")
            if "detected" in message:
                self.marker_status[marker_id]["detected"] = message["detected"]
                logging.debug(f"{marker_id} detected")
            if "landed" in message:
                self.marker_status[marker_id]["landed"] = message["landed"]
                logging.debug(f"{marker_id} landed")
            logging.info(f"Marker Statuses: {self.marker_status}")

    def broadcast_status(self):
        with self.lock:
            status_json = json.dumps(self.marker_status)
            for addr in self.clients:
                self.sock.sendto(status_json.encode(), addr)

    def run(self):
        self.clients = set()
        threading.Thread(target=self.handle_messages, daemon=True).start()
        while True:
            data, addr = self.sock.recvfrom(1024)
            if addr not in self.clients:
                self.clients.add(addr)
                logging.info(f"New client connected: {addr}")

class MarkerClient:
    def __init__(self, server_host='127.0.0.1', server_port=5005):
        self.server_host = server_host
        self.server_port = server_port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', 0))  # Bind to any available port
        self.marker_status = {}
        threading.Thread(target=self.receive_updates, daemon=True).start()
        logging.info(f"Marker client connected to server {self.server_host}:{self.server_port}")
        print(f"Marker client connected to server  {self.server_host}:{self.server_port}")

    def send_update(self, marker_id:int, detected=None, landed=None):
        message = {"marker_id": marker_id}
        if detected is not None:
            message["detected"] = detected
        if landed is not None:
            message["landed"] = landed
        self.sock.sendto(json.dumps(message).encode(), (self.server_host, self.server_port))
        logging.debug(f"MarkerClient sent {message}")

    def receive_updates(self):  # background thread, does not print directly! DO NOT call this directly.
        while True:
            data, _ = self.sock.recvfrom(1024)
            self.marker_status = json.loads(data.decode())

    def is_marker_available(self, marker_id):
        return not (self.marker_status.get(marker_id, {}).get("detected", False) or
                    self.marker_status.get(marker_id, {}).get("landed", False))
    
