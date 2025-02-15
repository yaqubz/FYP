import pygame
import sys
import time
import threading
import json
import copy
from pathlib import Path

from .constants import *
from .utils import *
import pandas as pd

import tkinter as tk
from tkinter import simpledialog

"""
    python -m UWBViz.main
"""

# Add workspace root to sys.path (9 Jan: Works but might need a better solution)
workspace_root = Path(__file__).resolve().parent.parent
sys.path.append(str(workspace_root))
from UWB_Wrapper.UWB_ReadUDP import get_all_positions


# Create a Tkinter root window (but don't show it)
root = tk.Tk()
root.withdraw()  # Hide the main Tkinter window

user_input = ""  # Store the user's input

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
        self.loaded_marked_pos = None

        self.last_cmdline:str = None        # Disabled 15 Feb - too complex to implement printing cmdline on pygame
        
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

    def load_marked_positions(self):
        """Load marked positions from JSON file"""

        user_input = simpledialog.askstring("Load Marked Positions", 
                                        f"Enter JSON filename to load (without .json extension) \nDefault: {MARKEDPOINTS_JSON_DEFAULT}")
        
        filename = MARKEDPOINTS_JSON_DEFAULT if user_input == "" else f"{user_input}.json"

        try:
            with open(filename, 'r') as f:
                self.loaded_marked_pos = json.load(f)
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
            self.waypoints = load_waypoints()
            self.resume_data_thread()
  
         # NEW 15 FEB - FOR MOUSE SIMULATION

        elif key == pygame.K_u:  # Toggle mouse simulation
            self.mouse_simulation = not self.mouse_simulation
            if self.mouse_simulation:
                print("[INFO] Mouse simulation enabled")
            else:
                print("[INFO] Mouse simulation disabled")

        # NEW 15 FEB - FOR WALL RECORDING

        elif key == pygame.K_r:  # Toggle recording

            self.mouse_simulation = True
            print("[INFO] Mouse simulation enabled")
            self.last_cmdline = "[INFO] Mouse simulation enabled"

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
            
        elif key == pygame.K_p and current_pos:  # Add pillar obstacle
            self.recording_manager.add_point(current_pos['x'], current_pos['y'], current_pos['z'], 'pillar')
        elif key == pygame.K_v:
            self.recording_manager.add_point(current_pos['x'], current_pos['y'], current_pos['z'], 'victim')
        elif key == pygame.K_d:
            self.recording_manager.add_point(current_pos['x'], current_pos['y'], current_pos['z'], 'danger')
        elif key == pygame.K_m:
            self.load_marked_positions()


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
            
            # DISABLED 15 Feb - this records ALL points between wall start and end (not yet implemented)
            # if self.recording_manager.recording and self.recording_manager.recording_wall and not current_positions.empty:
            #     last_pos = current_positions.iloc[-1]
            #     self.recording_manager.add_point(
            #         last_pos['x'], 
            #         last_pos['y'], 
            #         last_pos.get('z', 0),
            #         point_type='wall',
            #         wall_id=self.recording_manager.current_wall_id
            #     )

    def draw(self):
        self.screen.fill(BACKGROUND_COLOR)
        
        # Get current screen dimensions
        screen_width, screen_height = self.screen.get_size()
        
        self.background.draw(self.screen, self.coord_system.scale_factor, self.coord_system.screen_coordinates)
        self.visualization.draw_grid()
        self.visualization.draw_rectangle()
        self.visualization.draw_waypoints(self.waypoints)
        self.visualization.draw_positions()
        if self.loaded_marked_pos:
            self.visualization.draw_marked_positions(self.loaded_marked_pos)
        
        if self.recording_manager.recording:
            font = pygame.font.Font(None, 36)
            status = f"Recording Wall {self.recording_manager.current_wall_id}..." if self.recording_manager.recording_wall else "Ready to Record"
            self.last_cmdline = status
            text = font.render(status, True, (255, 0, 0))
            self.screen.blit(text, (10, 10))

        # Instructions
        font = pygame.font.Font(None, 25)
        instructions = "ESC: exit, SPACE: toggle trails, ENTER: toggle pan & zoom, L: load waypoints"
        instructions2 = "R: toggle recording, W: toggle walls, V/D/P: mark Victims/Dangers/Pillars, M: load markers"
        text = font.render(instructions, True, (0, 0, 0))
        text2 = font.render(instructions2, True, (0, 0, 0))
        self.screen.blit(text, (10, 40))
        self.screen.blit(text2, (10, 70))


        # 15 Feb: Last printed commandline (possible but nah too tedious)
        # text3 = font.render(self.last_cmdline, True, (0,0,0))
        # self.screen.blit(text3, (10, 200))
        
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
        
        pygame.quit()
        sys.exit()

if __name__ == "__main__":
    app = UWBVisualization()
    app.run()