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

from constants import *
from utils import *

"""
GUI to display live position of all UWB tags in the arena. 

This code requires:
    1. nlink_unpack_COMx_udp.exe to be running on any other device connected to the same network
        - This other device (Windows PC) is connected via USB to the LinkTrack Console and will transmit via UDP
        - nlink_unpack_COMx_udp.exe 's source file is main_udp.c
    2. UWB_ReadUDP.py custom library

Note on UDP:
    Using NTUSecure or RMTT/Tello WiFi is okay for transferring data within the same PC via UDP. However, it will not allow UDP transfer across devices.

"""

# Add workspace root to sys.path (9 Jan: Works but might need a better solution)
workspace_root = Path(__file__).resolve().parent.parent
sys.path.append(str(workspace_root))
from UWB_ReadUDP import get_all_positions

class UWBVisualization:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
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
        

        # Thread control
        self.data_event = threading.Event()
        self.data_event.set()  # Start in a "running" state

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
                return False
                
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

            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 3:  # Right-click
                if self.current_marker_mode:
                    # Convert screen coordinates to UWB coordinates
                    mouse_x, mouse_y = pygame.mouse.get_pos()
                    uwb_x, uwb_y = self.coord_system.uwb_coordinates(mouse_x, mouse_y)
                    
                    # Add marker
                    self.marked_positions[self.current_marker_mode].append({
                        'x': uwb_x, 
                        'y': uwb_y
                    })
                    
                    # Reset marker mode
                    self.current_marker_mode = None
                
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
                
                # Find closest marker across all types
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
                
                # Remove closest marker if within a reasonable threshold
                if closest_marker and min_dist < 50:  # Adjust threshold as needed
                    self.marked_positions[closest_type].remove(closest_marker)
                    print(f"[INFO] Removed {closest_type[:-1]} marker")
        
        elif key == pygame.K_BACKSPACE:
            # Clear all markers
            self.marked_positions = {
                'victims': [],
                'dangers': [],
                'pillars': []
            }
            print("[INFO] All markers cleared")

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

    def draw(self):
        self.screen.fill(BACKGROUND_COLOR)
        self.background.draw(self.screen, self.coord_system.scale_factor, self.coord_system.screen_coordinates)
        self.visualization.draw_grid()
        self.visualization.draw_rectangle()
        self.visualization.draw_waypoints(self.waypoints)
        self.visualization.draw_positions()
        self.visualization.draw_marked_positions(self.marked_positions)
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