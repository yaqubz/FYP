import tkinter as tk
from tkinter import ttk
from markerserverclient import MarkerClient
import logging

class MarkerStatusGUI:
    def __init__(self, server_host='255.255.255.255', server_port=5005):
        self.server_host = server_host
        self.server_port = server_port

        # Initialize the MarkerClient to receive updates
        self.client = MarkerClient(broadcast_ip=self.server_host, server_port=self.server_port)

        # Initialize GUI
        self.root = tk.Tk()
        self.root.title("Marker Status Monitor")
        self.root.geometry("400x300")

        # Create a table to display marker statuses
        self.tree = ttk.Treeview(self.root, columns=("Marker ID", "Detected", "Landed"), show="headings")
        self.tree.heading("Marker ID", text="Marker ID")
        self.tree.heading("Detected", text="Detected")
        self.tree.heading("Landed", text="Landed")
        self.tree.pack(fill=tk.BOTH, expand=True)

        # Add a refresh button
        self.refresh_btn = ttk.Button(self.root, text="Refresh", command=self.force_refresh)
        self.refresh_btn.pack(pady=5)

        # Start updating the GUI periodically
        self.update_gui()

        # Start the GUI main loop
        self.root.mainloop()

    def force_refresh(self):
        """Force a manual refresh of the status"""
        self.client.send_update(-1)  # Re-register with server
        self.update_gui()

    def update_gui(self):
        """Update the GUI with the latest marker statuses."""
        try:
            # Clear the current table
            for row in self.tree.get_children():
                self.tree.delete(row)

            # Add rows for each marker status
            for marker_id, status in self.client.marker_status.items():
                if isinstance(status, dict):  # Verify status is a dictionary
                    detected = status.get("detected", False)
                    landed = status.get("landed", False)

                    # Use Unicode green check (✔) and red cross (❌) for clarity
                    detected_str = "✔ True" if detected else "❌ False"
                    landed_str = "✔ True" if landed else "❌ False"

                    self.tree.insert("", "end", values=(marker_id, detected_str, landed_str))

        except Exception as e:
            logging.error(f"Error updating GUI: {e}")

        # Schedule the next update (refresh every 500ms)
        self.root.after(500, self.update_gui)

if __name__ == "__main__":
    file_handler = logging.FileHandler("log_markerserverGUI.log", mode='w')  # Log to a file (overwrite each run)
    console_handler = logging.StreamHandler()  # Log to the terminal
    formatter = logging.Formatter("%(levelname)s - %(asctime)s - %(message)s")
    file_handler.setFormatter(formatter)
    console_handler.setFormatter(formatter)
    logging.basicConfig(level=logging.INFO, handlers=[file_handler, console_handler])
    MarkerStatusGUI(server_host="255.255.255.255", server_port=5005)