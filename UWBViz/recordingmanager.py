import time
import json
import pandas as pd
from datetime import datetime

class RecordingManager:
    def __init__(self):
        self.recording = False
        self.recorded_points = []
        self.recording_wall = False
        self.current_wall_id = 0  # To identify different walls
        
    def start_recording(self):
        """Start a new recording session"""
        self.recording = True
        self.recorded_points = []
        print("[INFO] Recording started")
        
    def stop_recording(self):
        """Stop recording and save data"""
        self.recording = False
        self.recording_wall = False
        
        if self.recorded_points:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            # filename = f"uwb_trace_{timestamp}.json"
            filename = f"UWBViz/uwb_trace.json"
            
            with open(filename, 'w') as f:
                json.dump({
                    'points': self.recorded_points,
                    'metadata': {
                        'timestamp': timestamp,
                        'total_points': len(self.recorded_points)
                    }
                }, f, indent=4)
            print(f"[INFO] Recording saved to {filename}")
            
        self.recorded_points = []
        
    def toggle_wall(self, x, y, z):
        """Toggle wall recording state and mark start/end points"""
        if self.recording:
            self.recording_wall = not self.recording_wall
            if self.recording_wall:
                # Start of a new wall
                self.current_wall_id += 1
                self.add_point(x, y, z, point_type='wall_start', wall_id=self.current_wall_id)
                print(f"[INFO] Wall {self.current_wall_id} recording started...")
            else:
                # End of current wall
                self.add_point(x, y, z, point_type='wall_end', wall_id=self.current_wall_id)
                print(f"[INFO] Wall {self.current_wall_id} recording ended")

        else:
            print("[WARNING] Recording not enabled. Press R to enable.")
            
    def add_point(self, x, y, z, point_type='wall', wall_id=None):
        """Add a point to the recording with its type. Internal method; not externally called"""
        if self.recording:
            point = {
                'timestamp': time.time(),
                'x': x,
                'y': y,
                'z': z,
                'type': point_type
            }
            if wall_id is not None:
                point['wall_id'] = wall_id
            self.recorded_points.append(point)
            
    def add_point_obstacle(self, x, y, z):
        """Add a point obstacle"""
        if self.recording:
            self.add_point(x, y, z, point_type='obstacle')
            print("[INFO] Point obstacle recorded")
        else:
            print("[WARNING] Recording not enabled. Press R to enable.")