import paramiko
import time
import threading
import json
import tkinter as tk
from tkinter import ttk

def execute_command(client, command):
    """Helper function to execute a command on the remote server."""
    stdin, stdout, stderr = client.exec_command(command)
    output = stdout.read().decode('utf-8')
    error = stderr.read().decode('utf-8')

    if error:
        print(f"Error executing command '{command}': {error}")
    return output


def connect(hostname, username, password, SSID, root, retries=3, delay=2, update_status_callback=None, update_progress_callback=None):
    attempt = 0
    progress_started = False  # Flag to track the progress bar animation
    connected = False

    while attempt < retries:
        try:
            # Create SSH client object
            client = paramiko.SSHClient()
            client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

            # Update GUI status to "Connecting"
            update_status_callback(hostname, "Connecting...")
            update_progress_callback(hostname, 10)

            # Only start the progress bar animation when it's the first attempt to connect
            if not progress_started:
                progress_started = True  # Flag that we’ve started the progress animation

            client.connect(hostname, username=username, password=password, timeout=10)
            update_status_callback(hostname, f"Connected to RPi, searching for {SSID}...")
            if update_progress_callback:
                update_progress_callback(hostname, 30)  # Progress to 30% after connecting

            while not connected:
                
                '''
                try:
                    output = execute_command(client, "iwgetid")
                    if SSID in output:
                        update_status_callback(hostname, f"{SSID} is already connected")
                        connected = True
                        if update_progress_callback:
                            update_progress_callback(hostname, 100)  # Progress to 100%
                    break

                except Exception as e:
                    update_status_callback(hostname, f"Error: {e}")
                '''
                
                commands = [
                    "sudo iwlist wlan0 scan | grep ESSID",  
                    f"sudo nmcli device wifi connect \"{SSID}\""
                ]
                for command in commands:
                    try:
                        output = execute_command(client, command)
                        if SSID in output:
                            update_status_callback(hostname, f"{SSID} found! Connecting...")
                            if update_progress_callback:
                                update_progress_callback(hostname, 60)  # Progress to 60%
                        elif "Device 'wlan0' successfully activated" in output:
                            update_status_callback(hostname, f"Connected to {SSID}")
                            if update_progress_callback:
                                update_progress_callback(hostname, 100)  # Fully connected, 100%
                            connected = True
                    except Exception as e:
                        update_status_callback(hostname, f"Error: {e}")
                        time.sleep(5)

            break
        except Exception as e:
            update_status_callback(hostname, f"Error: {e}")

        attempt += 1
        if attempt < retries:
            update_status_callback(hostname, f"Retrying... ({attempt}/{retries})")
            if update_progress_callback:
                update_progress_callback(hostname, 0)  # Reset progress to 0% for retry
            time.sleep(delay)
        else:
            update_status_callback(hostname, "Max retries reached. Could not connect.")
            if update_progress_callback:
                update_progress_callback(hostname, 0)  # Reset progress to 0% if max retries reached

        client.close()

# Load DRONE_INFO from the drones.json file
def load_drone_info(filename='drones.json'):
    try:
        with open(filename, 'r') as file:
            return json.load(file)
    except FileNotFoundError:
        print(f"{filename} not found!")
        return []
    except json.JSONDecodeError:
        print(f"Error decoding JSON in {filename}!")
        return []

# Tkinter GUI setup
import threading

class DroneConnectionApp:
    def __init__(self, root, drone_info):
        self.root = root
        self.root.title("Drone Connection Status")
        self.drone_info = drone_info
        self.root.geometry("1000x750")
        self.root.config(bg="white")

        self.status_labels = {}
        self.progress_bars = {}
        self.current_progress_values = {drone['TELLO_IP']: 0 for drone in self.drone_info}
        self.failed_drones = []  # Track drones that failed after max retries

        # Header
        header_label = tk.Label(self.root, text="Drone Connection Status", font=("Helvetica", 16), bg="white", fg="black")
        header_label.pack(pady=10)

        # Create a frame for drone statuses
        self.status_frame = ttk.Frame(self.root)
        self.status_frame.pack(pady=20)

        # Create a label and progress bar for each drone
        for drone in self.drone_info:
            drone_frame = ttk.Frame(self.status_frame)
            drone_frame.pack(fill='x', padx=10, pady=5)

            id_label = ttk.Label(drone_frame, text=f"{drone['id']}", width=5, anchor="w", font=("Helvetica", 12))
            id_label.pack(side="left")

            ssid_label = ttk.Label(drone_frame, text=f"{drone['TELLO_SSID']}", width=15, anchor="w", font=("Helvetica", 12))
            ssid_label.pack(side="left")

            label = ttk.Label(drone_frame, text=f"{drone['TELLO_IP']} ", width=12, anchor="w", font=("Helvetica", 12))
            label.pack(side="left")

            status_label = ttk.Label(drone_frame, text="Not Connected", width=40, anchor="w", font=("Helvetica", 12))
            status_label.pack(side="right", padx=10)
            self.status_labels[drone['TELLO_IP']] = status_label

            progress_bar = ttk.Progressbar(drone_frame, length=200, mode="determinate", maximum=100)
            progress_bar.pack(side="right", padx=10)
            self.progress_bars[drone['TELLO_IP']] = progress_bar

        # Retry All button
        self.retry_all_button = ttk.Button(self.root, text="Retry All", command=self.retry_all_drones)
        self.retry_all_button.pack(pady=20)

    
    def update_progress_bar(self, hostname, value):
        """Update the progress bar to a specific percentage."""
        if hostname in self.progress_bars:
            progress_bar = self.progress_bars[hostname]
            
            # Only update progress if value is different from the current progress value
            if self.current_progress_values[hostname] != value:
                self.current_progress_values[hostname] = value
                progress_bar['value'] = value
            
            self.root.update_idletasks()


    def update_status(self, hostname, status):
        """Update the status of the drone in the GUI."""
        if hostname in self.status_labels:
            label = self.status_labels[hostname]
            label.config(text=status)
            self.root.update_idletasks()

            # Trigger animation when updating progress
            if "Connecting" in status:
                self.update_progress_bar(hostname, 10)
            elif "Retrying" in status:
                # Reset progress to 0% when retrying
                self.update_progress_bar(hostname, 0)
            elif "Connected" in status:
                self.update_progress_bar(hostname, 100)

            # Show in failed_drones list if max retries are reached
            if "Max retries reached" in status and hostname not in self.failed_drones:
                self.failed_drones.append(hostname)

    def retry_all_drones(self):
        """Retry all drones that failed after max retries in separate threads."""
        if not self.failed_drones:
            print("No drones need retrying.")
            return
        
        # Create a thread for each drone that failed
        for drone in self.drone_info:
            if drone['TELLO_IP'] in self.failed_drones:
                print(f"Retrying {drone['TELLO_IP']}...")
                # Use threading to retry the connection for each failed drone
                threading.Thread(target=self.start_connection_thread, args=(drone,)).start()
        
        # Clear the failed drones list after retrying
        self.failed_drones.clear()

    def start_connection_thread(self, drone):
        hostname = drone["TELLO_IP"]
        SSID = drone["TELLO_SSID"]
        username = "telloswarm"
        password = "telloswarm"
        connect(hostname, username, password, SSID, self.root, update_status_callback=self.update_status, update_progress_callback=self.update_progress_bar)

# Initialize the Tkinter app
def start_gui(drone_info):
    root = tk.Tk()
    app = DroneConnectionApp(root, drone_info)

    # Set all progress bars to 0 initially to avoid flashing
    for drone in drone_info:
        app.update_progress_bar(drone["TELLO_IP"], 0)

    # Create and start a thread for each drone
    for drone in drone_info:
        threading.Thread(target=app.start_connection_thread, args=(drone,)).start()

    root.mainloop()

if __name__ == "__main__":
    DRONE_INFO = load_drone_info()

    if DRONE_INFO:
        start_gui(DRONE_INFO)
    else:
        print("No drone information available.")
