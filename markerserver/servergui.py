import tkinter as tk
from tkinter import ttk
from markerserverclient import MarkerClient

class MarkerStatusGUI:
    def __init__(self, server_host='127.0.0.1', server_port=5005):
        self.server_host = server_host
        self.server_port = server_port

        # Initialize the MarkerClient to receive updates
        self.client = MarkerClient(server_host=self.server_host, server_port=self.server_port)

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

        # Start updating the GUI periodically
        self.update_gui_periodically()

        # Start the GUI main loop
        self.root.mainloop()

    def update_gui_periodically(self):
        """Update the GUI with the latest marker statuses every 0.5 seconds."""
        self.update_gui()
        self.root.after(500, self.update_gui_periodically)  # Schedule the next update

    def update_gui(self):
        """Update the GUI with the latest marker statuses."""
        # Clear the current table
        for row in self.tree.get_children():
            self.tree.delete(row)

        # Add rows for each marker status
        for marker_id, status in self.client.marker_status.items():
            detected = status.get("detected", False)
            landed = status.get("landed", False)

            # Use Unicode green check (✔) and red cross (❌) for clarity
            detected_str = f"✔ True" if detected else f"❌ False"
            landed_str = f"✔ True" if landed else f"❌ False"

            self.tree.insert("", "end", values=(marker_id, detected_str, landed_str))

if __name__ == "__main__":
    MarkerStatusGUI(server_host="127.0.0.1", server_port=5005)
