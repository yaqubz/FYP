# main.py
import pygame
import sys
import time
import threading
import json
from collections import defaultdict
from queue import Queue
import copy
from pathlib import Path

from .constants import *
from .utils import *
import pandas as pd
from datetime import datetime

"""
python -m UWBViz.main

GUI to display live position of all UWB tags in the arena. 

This code requires:
    1. nlink_unpack_COMx_udp.exe to be running on any other device connected to the same network
        - This other device (Windows PC) is connected via USB to the LinkTrack Console and will transmit via UDP
        - nlink_unpack_COMx_udp.exe 's source file is main_udp.c
    2. UWB_ReadUDP.py custom library

Note on UDP:
    Using NTUSecure or RMTT/Tello WiFi is okay for transferring data within the same PC via UDP. However, it will not allow UDP transfer across devices.

Run UWB_SendUDP.py to simulate some drones' locations being broadcast via UDP
"""

# Add workspace root to sys.path (9 Jan: Works but might need a better solution)
workspace_root = Path(__file__).resolve().parent.parent
sys.path.append(str(workspace_root))
from UWB_Wrapper.UWB_ReadUDP import get_all_positions

from .recordingmanager import RecordingManager

class UWBVisualization:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT), pygame.RESIZABLE)
        pygame.display.set_caption("UWB Position Visualization")
        
        self.coord_system = CoordinateSystem()
        self.visualization = Visualization(self.screen, self.coord_system)
        self.background = Background(BGPIC)
        
        self.latest_positions_data = None
        self.data_lock = threading.Lock()
        self.controls_enabled = True
        self.panning = False
        self.last_mouse_pos = None
        self.waypoints = []
        
        # Mouse simulation variables
        self.mouse_simulation = False
        self.simulated_tags = {
            0: {'x': 0, 'y': 0, 'z': 0},
        }
        
        # Recording manager
        self.recording_manager = RecordingManager()

        # Thread control
        self.data_event = threading.Event()
        self.data_event.set()

        # Start data collection thread
        self.data_thread = threading.Thread(target=self.collect_data, daemon=True)
        self.data_thread.start()

        self.marked_positions = {
            'victims': [],
            'dangers': [],
            'pillars': []
        }
        self.current_marker_mode = None

    def save_marked_positions(self):
        """Save marked positions to a JSON file"""
        if self.marked_positions: # only if positions were marked
            try:
                with open('marked_positions.json', 'w') as f:
                    json.dump(self.marked_positions, f, indent=4)
                print("[INFO] Marked positions saved successfully.")
            except Exception as e:
                print(f"[ERROR] Could not save marked positions: {e}")

    def load_marked_positions(self):
        """Load marked positions from JSON file"""
        try:
            with open('marked_positions.json', 'r') as f:
                loaded_positions = json.load(f)
            
            # Clear existing markers
            self.marked_positions = loaded_positions
            print("[INFO] Marked positions loaded successfully.")
        except FileNotFoundError:
            print("[INFO] No saved marked positions found.")
        except Exception as e:
            print(f"[ERROR] Could not load marked positions: {e}")

    def collect_data(self):
        while True:
            self.data_event.wait()  # Block if the event is cleared (paused)
            
            if self.mouse_simulation:
                # Create DataFrame from mouse position
                mouse_x, mouse_y = pygame.mouse.get_pos()
                uwb_x, uwb_y = self.coord_system.uwb_coordinates(mouse_x, mouse_y)
                self.simulated_tags[0]['x'] = uwb_x
                self.simulated_tags[0]['y'] = uwb_y
                
                # Convert simulated data to DataFrame
                rows = []
                for tag_id, pos in self.simulated_tags.items():
                    rows.append({
                        'id': tag_id,
                        'role': 0,
                        'x': pos['x'],
                        'y': pos['y'],
                        'z': pos['z'],
                        'timestamp': time.time()
                    })
                df = pd.DataFrame(rows)
                
                with self.data_lock:
                    self.latest_positions_data = df
            else:
                try:
                    df = get_all_positions()
                    if not df.empty:
                        with self.data_lock:
                            self.latest_positions_data = df.groupby('id').last().reset_index()
                except Exception as e:
                    print(f"Error reading data: {e}")
            
            time.sleep(0.01)

    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT or (
                event.type == pygame.KEYDOWN and 
                event.key == pygame.K_ESCAPE
            ):
                
                if self.recording_manager.recording:
                    self.recording_manager.stop_recording()
                return False
                
            elif event.type == pygame.VIDEORESIZE:
                # Handle window resize
                self.screen = pygame.display.set_mode((event.w, event.h), pygame.RESIZABLE)
                self.visualization.screen = self.screen  # Update visualization's screen reference
                
            elif event.type == pygame.MOUSEWHEEL and self.controls_enabled:
                self.handle_zoom(event.y)
                
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                if self.controls_enabled:
                    self.panning = True
                    self.last_mouse_pos = pygame.mouse.get_pos()
                    
            elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                self.panning = False
                
            elif event.type == pygame.MOUSEMOTION and self.panning and self.controls_enabled:
                current_pos = pygame.mouse.get_pos()
                self.coord_system.offset_x += current_pos[0] - self.last_mouse_pos[0]
                self.coord_system.offset_y += current_pos[1] - self.last_mouse_pos[1]
                self.last_mouse_pos = current_pos
                
            elif event.type == pygame.KEYDOWN:
                self.handle_keypress(event.key)

        return True

    def handle_zoom(self, y):
        mouse_x, mouse_y = pygame.mouse.get_pos()
        pre_zoom_uwb_x, pre_zoom_uwb_y = self.coord_system.uwb_coordinates(mouse_x, mouse_y)
        old_scale = self.coord_system.scale_factor
        self.coord_system.scale_factor = max(10, min(INITIAL_SCALE * 10, self.coord_system.scale_factor + y * 5))
        new_screen_x, new_screen_y = self.coord_system.screen_coordinates(pre_zoom_uwb_x, pre_zoom_uwb_y)
        self.coord_system.offset_x += (mouse_x - new_screen_x)
        self.coord_system.offset_y += (mouse_y - new_screen_y)

    def handle_keypress(self, key):

        current_pos = None
        if self.mouse_simulation:
            mouse_x, mouse_y = pygame.mouse.get_pos()
            uwb_x, uwb_y = self.coord_system.uwb_coordinates(mouse_x, mouse_y)
            current_pos = {'x': uwb_x, 'y': uwb_y, 'z': 0}
        elif self.latest_positions_data is not None and not self.latest_positions_data.empty:
            last_pos = self.latest_positions_data.iloc[-1]
            current_pos = {'x': last_pos['x'], 'y': last_pos['y'], 'z': last_pos.get('z', 0)}

        if key == pygame.K_SPACE:
            self.visualization.persistent_trails = not self.visualization.persistent_trails
            if not self.visualization.persistent_trails:
                self.visualization.position_history.clear()
        elif key == pygame.K_RETURN:
            self.controls_enabled = not self.controls_enabled
            if not self.controls_enabled:
                self.panning = False
        elif key == pygame.K_l:
            self.pause_data_thread()
            self.waypoints = load_waypoints(JSON_READ)
            self.resume_data_thread()
        elif key == pygame.K_v:
            self.current_marker_mode = 'victims'
        elif key == pygame.K_d:
            self.current_marker_mode = 'dangers'
        elif key == pygame.K_p:
            self.current_marker_mode = 'pillars'
        elif key == pygame.K_m:
            self.load_marked_positions()
        elif key == pygame.K_DELETE:
            # Remove marker closest to mouse pointer
            if self.marked_positions:
                mouse_x, mouse_y = pygame.mouse.get_pos()
                uwb_x, uwb_y = self.coord_system.uwb_coordinates(mouse_x, mouse_y)
                mouse_point = {'x': uwb_x, 'y': uwb_y}
                
                closest_marker = None
                closest_type = None
                min_dist = float('inf')
                
                for marker_type in ['victims', 'dangers', 'pillars']:
                    for marker in self.marked_positions[marker_type]:
                        dist = distance(marker, mouse_point)
                        if dist < min_dist:
                            min_dist = dist
                            closest_marker = marker
                            closest_type = marker_type
                
                if closest_marker and min_dist < 50:
                    self.marked_positions[closest_type].remove(closest_marker)
                    print(f"[INFO] Removed {closest_type[:-1]} marker")
        
        elif key == pygame.K_BACKSPACE:
            self.marked_positions = {
                'victims': [],
                'dangers': [],
                'pillars': []
            }
            print("[INFO] All markers cleared")

         # NEW 15 FEB - FOR MOUSE SIMULATION

        elif key == pygame.K_u:  # Toggle mouse simulation
            self.mouse_simulation = not self.mouse_simulation
            if self.mouse_simulation:
                print("[INFO] Mouse simulation enabled")
            else:
                print("[INFO] Mouse simulation disabled")

        # NEW 15 FEB - FOR WALL RECORDING

        elif key == pygame.K_r:  # Toggle recording
            if not self.recording_manager.recording:
                self.recording_manager.start_recording()
            else:
                self.recording_manager.stop_recording()
                
        elif key == pygame.K_w and current_pos:  # Toggle wall recording
            self.recording_manager.toggle_wall(
                current_pos['x'], 
                current_pos['y'], 
                current_pos['z']
            )
            
        elif key == pygame.K_q and current_pos:  # Add point obstacle
            self.recording_manager.add_point_obstacle(
                current_pos['x'], current_pos['y'], current_pos['z']
            )



    def pause_data_thread(self):
        print("[INFO] Pausing data collection thread. Please type input in the terminal.")
        self.data_event.clear()  # Pause the thread

    def resume_data_thread(self):
        print("[INFO] Resuming data collection thread.")
        self.data_event.set()  # Resume the thread

    def update(self):
        with self.data_lock:
            current_positions = copy.deepcopy(self.latest_positions_data)
        
        if current_positions is not None:
            self.visualization.update_positions(current_positions)
            
            # Only record if we're recording a wall
            if self.recording_manager.recording and self.recording_manager.recording_wall and not current_positions.empty:
                last_pos = current_positions.iloc[-1]
                self.recording_manager.add_point(
                    last_pos['x'], 
                    last_pos['y'], 
                    last_pos.get('z', 0),
                    point_type='wall',
                    wall_id=self.recording_manager.current_wall_id
                )

    def draw(self):
        self.screen.fill(BACKGROUND_COLOR)
        
        # Get current screen dimensions
        screen_width, screen_height = self.screen.get_size()
        
        self.background.draw(self.screen, self.coord_system.scale_factor, self.coord_system.screen_coordinates)
        self.visualization.draw_grid()
        self.visualization.draw_rectangle()
        self.visualization.draw_waypoints(self.waypoints)
        self.visualization.draw_positions()
        self.visualization.draw_marked_positions(self.marked_positions)
        
        if self.recording_manager.recording:
            font = pygame.font.Font(None, 36)
            status = f"Recording Wall {self.recording_manager.current_wall_id}..." if self.recording_manager.recording_wall else "Ready to Record"
            text = font.render(status, True, (255, 0, 0))
            self.screen.blit(text, (10, 10))
        
        pygame.display.update()
    

    def run(self):
        clock = pygame.time.Clock()
        print("[INFO] Press ESC to exit, SPACE for trails, ENTER for controls, L to load waypoints")
        print("[INFO] V/D/P to mark Victims/Dangers/Pillars, M to load markers")
        
        running = True
        while running:
            running = self.handle_events()
            self.update()
            self.draw()
            clock.tick(30)
        
        self.save_marked_positions()
        pygame.quit()
        sys.exit()

if __name__ == "__main__":
    app = UWBVisualization()
    app.run()