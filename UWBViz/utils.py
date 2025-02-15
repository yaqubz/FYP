import pygame
import json
import time
from .constants import *
from typing import List, Dict, Any
import tkinter as tk
from tkinter import simpledialog

class Background:
    def __init__(self, image_path):
        try:
            self.original_image = pygame.image.load(image_path)
            print("Background image loaded successfully")
        except Exception as e:
            print(f"Error loading background image: {e}")
            self.original_image = pygame.Surface((SCREEN_WIDTH, SCREEN_HEIGHT))
            self.original_image.fill(BACKGROUND_COLOR)

    def draw(self, surface, scale_factor, screen_coordinates):
        scaled_width = RECT_WIDTH * scale_factor
        scaled_height = RECT_HEIGHT * scale_factor
        scaled_image = pygame.transform.scale(self.original_image, (scaled_width, scaled_height))
        pos_x, pos_y = screen_coordinates(0, RECT_HEIGHT)
        surface.blit(scaled_image, (pos_x, pos_y))

class CoordinateSystem:
    def __init__(self):
        self.scale_factor = INITIAL_SCALE
        self.offset_x = 0
        self.offset_y = 0

    def screen_coordinates(self, x, y):
        screen_x = SCREEN_WIDTH // 2 + (x * self.scale_factor) + self.offset_x
        screen_y = SCREEN_HEIGHT // 2 - (y * self.scale_factor) + self.offset_y
        return int(screen_x), int(screen_y)

    def uwb_coordinates(self, screen_x, screen_y):
        x = (screen_x - SCREEN_WIDTH // 2 - self.offset_x) / self.scale_factor
        y = (SCREEN_HEIGHT // 2 - screen_y + self.offset_y) / self.scale_factor
        return x, y

def load_waypoints():
    """
    Only loads all the waypoints if the LAST waypoint is provided (i.e. distance,angle = 0,0)
    """
    user_input = simpledialog.askstring("Load Waypoints", 
                                        f"Enter JSON filename to load (without .json extension) \nDefault: {WAYPOINTS_JSON_DEFAULT}")
        
    filename = WAYPOINTS_JSON_DEFAULT if user_input == "" else f"{user_input}.json"
    
    try:
        with open(filename, 'r') as f:
            data = json.load(f)
            waypoints = data.get("wp", [])
            return [(point['position_cm']['x'] / 100, point['position_cm']['y'] / 100) 
                   for point in waypoints]
    except Exception as e:
        print(f"Error loading waypoints: {e}")
        return []

class Visualization:
    def __init__(self, screen, coord_system):
        self.screen = screen
        self.coord_system = coord_system
        self.position_history = {}
        self.id_colors = {}
        self.persistent_trails = False

    def draw_grid(self):
        start_x, start_y = self.coord_system.uwb_coordinates(0, SCREEN_HEIGHT)
        end_x, end_y = self.coord_system.uwb_coordinates(SCREEN_WIDTH, 0)
        font = pygame.font.Font(None, 20)
        
        for x in range(int(start_x) - 1, int(end_x) + 2):
            screen_x, _ = self.coord_system.screen_coordinates(x, 0)
            if 0 <= screen_x <= SCREEN_WIDTH:
                pygame.draw.line(self.screen, GRID_COLOR, (screen_x, 0), (screen_x, SCREEN_HEIGHT))
                if x != 0:
                    label = font.render(f"{x}", True, LABEL_COLOR)
                    self.screen.blit(label, (screen_x - 10, SCREEN_HEIGHT - 20))

        for y in range(int(start_y) - 1, int(end_y) + 2):
            _, screen_y = self.coord_system.screen_coordinates(0, y)
            if 0 <= screen_y <= SCREEN_HEIGHT:
                pygame.draw.line(self.screen, GRID_COLOR, (0, screen_y), (SCREEN_WIDTH, screen_y))
                if y != 0:
                    label = font.render(f"{y}", True, LABEL_COLOR)
                    self.screen.blit(label, (10, screen_y - 10))

    def draw_rectangle(self):
        points = [
            self.coord_system.screen_coordinates(0, 0),
            self.coord_system.screen_coordinates(RECT_WIDTH, 0),
            self.coord_system.screen_coordinates(RECT_WIDTH, RECT_HEIGHT),
            self.coord_system.screen_coordinates(0, RECT_HEIGHT)
        ]
        pygame.draw.polygon(self.screen, RED, points, 2)

    def draw_waypoints(self, waypoints):
        if not waypoints:
            return
        
        screen_coords = [self.coord_system.screen_coordinates(x, y) for x, y in waypoints]
        
        for i, point in enumerate(screen_coords):
            pygame.draw.circle(self.screen, BLUE, point, 5)
            if i > 0:
                pygame.draw.line(self.screen, RED, screen_coords[i-1], point, 2)

    def update_positions(self, positions_df):
        current_time = time.time()
        
        for _, row in positions_df.iterrows():
            tag_id = row['id']
            if tag_id not in self.position_history:
                self.position_history[tag_id] = []
                self.id_colors[tag_id] = TAG_COLORS[int(tag_id) % len(TAG_COLORS)]
            
            self.position_history[tag_id].append((current_time, row['x'], row['y']))
            if len(self.position_history[tag_id]) > MAX_HISTORY and not self.persistent_trails:
                self.position_history[tag_id].pop(0)

    def draw_positions(self):
        current_time = time.time()
        font = pygame.font.Font(None, FONT_SIZE)

        for tag_id, positions in self.position_history.items():
            color = self.id_colors[tag_id]
            
            for i in range(len(positions) - 1):
                timestamp, x1, y1 = positions[i]
                _, x2, y2 = positions[i + 1]
                
                alpha = 255 if self.persistent_trails else \
                       int(255 * (1 - (current_time - timestamp) / TRACE_FADE_TIME))
                
                if alpha <= 0:
                    continue
                    
                trail_color = tuple(c * alpha // 255 for c in color)
                start_pos = self.coord_system.screen_coordinates(x1, y1)
                end_pos = self.coord_system.screen_coordinates(x2, y2)
                pygame.draw.line(self.screen, trail_color, start_pos, end_pos, 2)
            
            # Draw current position
            if positions:
                _, x, y = positions[-1]
                screen_x, screen_y = self.coord_system.screen_coordinates(x, y)
                pygame.draw.circle(self.screen, color, (screen_x, screen_y), POINT_RADIUS)
                label = font.render(f"#{int(tag_id)} ({x:.1f}, {y:.1f})", True, color)
                self.screen.blit(label, (screen_x + 10, screen_y - 10))

    def draw_marked_positions(self, marked_positions:List[Dict[str, Any]]):
        """Draw walls, victims, dangers, and pillars on the screen."""
        
        walls = []
        current_wall_start = None

        # Process walls from wall_start to wall_end
        for point in marked_positions:
            if point["type"] == "wall_start":
                current_wall_start = (point["x"], point["y"])
            elif point["type"] == "wall_end" and current_wall_start:
                walls.append((current_wall_start, (point["x"], point["y"])))
                current_wall_start = None  # Reset for the next segment

        # Draw walls as thick black lines
        for wall in walls:
            start_x, start_y = self.coord_system.screen_coordinates(wall[0][0], wall[0][1])
            end_x, end_y = self.coord_system.screen_coordinates(wall[1][0], wall[1][1])
            pygame.draw.line(self.screen, (100, 100, 200), (start_x, start_y), (end_x, end_y), 5)

        # Draw victims as green triangles
        for victim in (p for p in marked_positions if p["type"] == "victim"):
            screen_x, screen_y = self.coord_system.screen_coordinates(victim["x"], victim["y"])
            pygame.draw.polygon(self.screen, (0, 255, 0), [
                (screen_x, screen_y - 10),
                (screen_x - 10, screen_y + 10),
                (screen_x + 10, screen_y + 10)
            ])

        # Draw dangers as red triangles
        for danger in (p for p in marked_positions if p["type"] == "danger"):
            screen_x, screen_y = self.coord_system.screen_coordinates(danger["x"], danger["y"])
            pygame.draw.polygon(self.screen, (255, 0, 0), [
                (screen_x, screen_y - 10),
                (screen_x - 10, screen_y + 10),
                (screen_x + 10, screen_y + 10)
            ])

        # Draw pillars as black circles
        for pillar in (p for p in marked_positions if p["type"] == "pillar"):
            screen_x, screen_y = self.coord_system.screen_coordinates(pillar["x"], pillar["y"])
            pygame.draw.circle(self.screen, (0, 0, 0), (screen_x, screen_y), 10, 2)

class RecordingManager:
    def __init__(self):
        self.recording:bool = False
        self.recording_wall:bool = False
        self.recorded_points: List[Dict[str, Any]] = [] # Ordered list of points. Each point is a dictionary with {x: y: z: etc.}
        self.current_wall_id:int = 0  # To identify different walls
        
    def start_recording(self):
        """Start a new recording session"""
        self.recording:bool = True
        self.recorded_points = []
        print("[INFO] Ready to record. Hotkeys: W-Walls (toggle on/off) | P-Pillars | V-Victims")
        
    def stop_recording(self):
        """
        Stop recording and save data. Data must be recorded and saved in one setting! Cannot load json and add more data.
        Can manually combine json for separate recordings.
        
        """
        self.recording = False
        self.recording_wall = False
        
        if self.recorded_points:
            user_input = simpledialog.askstring("Save Marked Positions", 
                                        f"Enter JSON filename to save (without .json extension) \nDefault: {MARKEDPOINTS_JSON_DEFAULT}")
        
            filename = MARKEDPOINTS_JSON_DEFAULT if user_input == "" else f"{user_input}.json"
            
    # Count pillars, victims, and danger points *before* creating the JSON
            total_pillars = 0
            total_victims = 0
            total_danger = 0

            for point in self.recorded_points:  # Assuming self.recorded_points is your list of points
                point_type = point.get("type")  # Use .get() to avoid KeyError if "type" is missing
                if point_type == "pillar":  # Adjust the type names as needed
                    total_pillars += 1
                elif point_type == "victim":
                    total_victims += 1
                elif point_type == "danger":
                    total_danger += 1

            with open(filename, 'w') as f:
                json.dump({
                    'points': self.recorded_points,
                    'metadata': {   # for user reference only! not used elsewhere
                        'total_points': len(self.recorded_points),
                        'total_wall_segments': self.current_wall_id,
                        'total_pillars': total_pillars,
                        'total_victims': total_victims,
                        'total_danger': total_danger
                    }
                }, f, indent=4)
            print(f"[INFO] Recording saved to {filename}")
            
        # self.recorded_points = []
        
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
            
    def add_point(self, x:float, y:float, z:float, point_type:str = ('pillar', 'wall', 'wall_start', 'wall_end', 'victim', 'danger'), wall_id=None):
        """Add a point to the recording with its type."""
        x, y, z = round(x,2), round(y,2), round(z,2)
        if self.recording:
            point = {
                'x': x,
                'y': y,
                'z': z,
                'type': point_type
            }
            if point_type == 'wall' and wall_id is not None:
                point['wall_id'] = wall_id
            self.recorded_points.append(point)

            print(f"[INFO] {point_type.capitalize()} recorded at {x},{y}")
        else:
            print("[WARNING] Recording not enabled. Press R to enable.")
            
        

def distance(point1, point2):
    """Calculate Euclidean distance between two points"""
    return ((point1['x'] - point2['x'])**2 + (point1['y'] - point2['y'])**2)**0.5