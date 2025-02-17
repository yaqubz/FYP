"""
Run directly in terminal to open GUI for markerserver/swarmserver.
Operates as a client that receives updates.
"""

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
        self.root.geometry("100x250")  # IMPT: Size of window upon startup! Height x Width

        # Create a table to display marker statuses
        self.tree = ttk.Treeview(self.root, columns=("Marker ID", "Detected", "Landed", "Drone ID"), show="headings")
        self.tree.heading("Marker ID", text="Marker ID")
        self.tree.heading("Detected", text="Detected")
        self.tree.heading("Landed", text="Landed")
        self.tree.heading("Drone ID", text="Drone ID")

        # Pack the treeview with resizing
        self.tree.pack(fill=tk.BOTH, expand=True)

        # Adjust column width dynamically
        self.root.update_idletasks()  # Ensure layout updates before setting column widths
        self.adjust_column_widths()

        # Bind window resize event
        self.root.bind("<Configure>", lambda event: self.adjust_column_widths())

        # Add a refresh button
        self.refresh_btn = ttk.Button(self.root, text="Refresh", command=self.force_refresh)
        self.refresh_btn.pack(pady=5)

        # Start updating the GUI periodically
        self.update_gui()

        # Start the GUI main loop
        self.root.mainloop()

    def adjust_column_widths(self):
        """Adjust column widths dynamically based on window size."""
        total_width = self.root.winfo_width()
        self.tree.column("Marker ID", width=int(total_width * 0.25))  # 25% of window width
        self.tree.column("Detected", width=int(total_width * 0.25))   # 25%
        self.tree.column("Landed", width=int(total_width * 0.25))     # 25%
        self.tree.column("Drone ID", width=int(total_width * 0.25))   # 25%

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
                    drone_id = status.get("drone_id", 0)

                    # Use Unicode green check (✔) and red cross (❌) for clarity
                    detected_str = "✔ True" if detected else "❌ False"
                    landed_str = "✔ True" if landed else "❌ False"

                    self.tree.insert("", "end", values=(marker_id, detected_str, landed_str, drone_id))

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
