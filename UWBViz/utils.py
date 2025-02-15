import pygame
import json
import time
from .constants import *

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

def load_waypoints(json_filename):
    """
    Only loads all the waypoints if the LAST waypoint is provided (i.e. distance,angle = 0,0)
    """
    filename = input("Enter JSON filename to load (without .json extension): ")
    filename = json_filename if filename == "" else f"{filename}.json"
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

    def draw_marked_positions(self, marked_positions):
        # Draw victims as green triangles
        for victim in marked_positions['victims']:
            screen_x, screen_y = self.coord_system.screen_coordinates(victim['x'], victim['y'])
            pygame.draw.polygon(self.screen, (0, 255, 0), [
                (screen_x, screen_y - 10),
                (screen_x - 10, screen_y + 10),
                (screen_x + 10, screen_y + 10)
            ])
        
        # Draw dangers as red circles
        for danger in marked_positions['dangers']:
            screen_x, screen_y = self.coord_system.screen_coordinates(danger['x'], danger['y'])
            pygame.draw.circle(self.screen, (255, 0, 0), (screen_x, screen_y), 10, 2)
        
        # Draw pillars as black circles
        for pillar in marked_positions['pillars']:
            screen_x, screen_y = self.coord_system.screen_coordinates(pillar['x'], pillar['y'])
            pygame.draw.circle(self.screen, (0, 0, 0), (screen_x, screen_y), 10, 2)

def distance(point1, point2):
    """Calculate Euclidean distance between two points"""
    return ((point1['x'] - point2['x'])**2 + (point1['y'] - point2['y'])**2)**0.5