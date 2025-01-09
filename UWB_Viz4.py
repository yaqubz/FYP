# WORKS WELL 8 JAN - SOURCE: Claude

"""
GUI to display live position of all UWB tags in the arena. 

This code requires:
    1. nlink_unpack_COMx_udp.exe to be running on any other device connected to the same network
        - This other device (Windows PC) is connected via USB to the LinkTrack Console and will transmit via UDP
        - nlink_unpack_COMx_udp.exe 's source file is main_udp.c
    2. UWB_ReadUDP.py custom library

Note on UDP:
    Using NTUSecure or RMTT/Tello WiFi is okay for transferring data within the same PC via UDP. However, it will not allow UDP transfer across devices.

    
ISSUES/TODO:
    1. Does not load/update when there is no UWB info (resolved caa 8 Jan)
    2. Load waypoints from json on the map (TODO 8 Jan)
"""


import pygame
import pandas as pd
import time
import sys
from collections import defaultdict
from UWB_ReadUDP import get_all_positions, get_target_position

# Initialize Pygame
pygame.init()

# Constants
SCREEN_WIDTH,SCREEN_HEIGHT = 1000, 800
BACKGROUND_COLOR = (255, 255, 255)  # White
GRID_COLOR = (200, 200, 200)       # Lighter gray for better visibility on white
LABEL_COLOR = (100, 100, 100)      # Darker gray for better visibility on white
POINT_RADIUS = 7
FONT_SIZE = 30
INITIAL_SCALE = 40
MAX_HISTORY = 50
BGPIC = "resources/field2025_toscale.PNG"
TRACE_FADE_TIME = 10
RECT_WIDTH, RECT_HEIGHT = 20, 20

# Predefined colors (20 distinct colors that are dark enough to read against white)
TAG_COLORS = [
    (0, 51, 102),      # Navy Blue
    (153, 0, 0),       # Dark Red
    (0, 102, 51),      # Forest Green
    (102, 0, 102),     # Purple
    (153, 76, 0),      # Dark Orange
    (51, 51, 0),       # Olive
    (102, 51, 0),      # Brown
    (51, 0, 51),       # Dark Purple
    (0, 76, 153),      # Royal Blue
    (153, 0, 76),      # Dark Pink
    (0, 102, 102),     # Teal
    (102, 76, 0),      # Bronze
    (76, 0, 153),      # Indigo
    (102, 51, 51),     # Burgundy
    (51, 76, 0),       # Dark Green
    (0, 51, 51),       # Dark Teal
    (153, 51, 0),      # Rust
    (51, 0, 102),      # Deep Purple
    (0, 76, 76),       # Dark Cyan
    (76, 51, 0),       # Dark Brown
]

# Global variables
scale_factor = INITIAL_SCALE
position_history = defaultdict(list)
id_colors = {}
offset_x = 0
offset_y = 0
panning = False
last_mouse_pos = None
persistent_trails = False  # For trail persistence
controls_enabled = True   # For zoom and pan controls

# Set up the display
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption("UWB Position Visualization")

class Background:
    def __init__(self, image_path):
        try:
            # Load original image and store it
            self.original_image = pygame.image.load(image_path)
            print("Background image loaded successfully")
        except Exception as e:
            print(f"Error loading background image: {e}")
            self.original_image = pygame.Surface((SCREEN_WIDTH, SCREEN_HEIGHT))
            self.original_image.fill(BACKGROUND_COLOR)

    def draw(self, surface):
        # Calculate the size that will fit the 20x20 grid
        scaled_width = RECT_WIDTH * scale_factor
        scaled_height = RECT_HEIGHT * scale_factor
        
        # Scale the image to fit exactly in the 20x20 grid
        scaled_image = pygame.transform.scale(self.original_image, (scaled_width, scaled_height))
        
        # Get the screen coordinates for (0,0)
        pos_x, pos_y = screen_coordinates(0, RECT_HEIGHT)  # Use RECT_HEIGHT for Y to flip the image right-side up
        
        # Draw the scaled and positioned image
        surface.blit(scaled_image, (pos_x, pos_y))

def generate_color(tag_id):
    """Assign a color from the predefined list based on tag_id."""
    return TAG_COLORS[int(tag_id) % len(TAG_COLORS)]

def screen_coordinates(x, y):
    """Convert UWB coordinates to screen coordinates."""
    screen_x = SCREEN_WIDTH // 2 + (x * scale_factor) + offset_x
    screen_y = SCREEN_HEIGHT // 2 - (y * scale_factor) + offset_y
    return int(screen_x), int(screen_y)

def uwb_coordinates(screen_x, screen_y):
    """Convert screen coordinates to UWB coordinates."""
    x = ((screen_x - SCREEN_WIDTH // 2 - offset_x) / scale_factor)
    y = ((SCREEN_HEIGHT // 2 - screen_y + offset_y) / scale_factor)
    return x, y

def draw_grid():
    """Draw a grid with 0,0 at the center and measurements."""
    start_x, start_y = uwb_coordinates(0, SCREEN_HEIGHT)
    end_x, end_y = uwb_coordinates(SCREEN_WIDTH, 0)
    font = pygame.font.Font(None, 20)
    
    for x in range(int(start_x) - 1, int(end_x) + 2):
        screen_x, _ = screen_coordinates(x, 0)
        if 0 <= screen_x <= SCREEN_WIDTH:
            pygame.draw.line(screen, GRID_COLOR, (screen_x, 0), (screen_x, SCREEN_HEIGHT))
            if x != 0:
                label = font.render(f"{x}", True, LABEL_COLOR)
                screen.blit(label, (screen_x - 10, SCREEN_HEIGHT - 20))
    
    for y in range(int(start_y) - 1, int(end_y) + 2):
        _, screen_y = screen_coordinates(0, y)
        if 0 <= screen_y <= SCREEN_HEIGHT:
            pygame.draw.line(screen, GRID_COLOR, (0, screen_y), (SCREEN_WIDTH, screen_y))
            if y != 0:
                label = font.render(f"{y}", True, LABEL_COLOR)
                screen.blit(label, (10, screen_y - 10))
    
    origin_x, origin_y = screen_coordinates(0, 0)
    pygame.draw.line(screen, GRID_COLOR, (origin_x, 0), (origin_x, SCREEN_HEIGHT))
    pygame.draw.line(screen, GRID_COLOR, (0, origin_y), (SCREEN_WIDTH, origin_y))
    origin_label = font.render("0", True, LABEL_COLOR)
    screen.blit(origin_label, (origin_x + 5, origin_y + 5))

def draw_rectangle(width, height):
    """Draw a rectangle starting at (0, 0)."""
    rect_color = (255, 0, 0)  # Red
    x1, y1 = screen_coordinates(0, 0)
    x2, y2 = screen_coordinates(width, 0)
    x3, y3 = screen_coordinates(width, height)
    x4, y4 = screen_coordinates(0, height)
    pygame.draw.polygon(screen, rect_color, [(x1, y1), (x2, y2), (x3, y3), (x4, y4)], 2)

def draw_positions(positions_df):
    """Draw all UWB positions and their trails on the screen."""
    current_time = time.time()

    for _, row in positions_df.iterrows():
        tag_id = row['id']
        position_history[tag_id].append((current_time, row['x'], row['y']))
        if len(position_history[tag_id]) > MAX_HISTORY and not persistent_trails:
            position_history[tag_id].pop(0)
        
        if tag_id not in id_colors:
            id_colors[tag_id] = generate_color(tag_id)

    for tag_id, positions in position_history.items():
        color = id_colors[tag_id]
        for i in range(len(positions)):
            timestamp, x, y = positions[i]
            
            # If trails are persistent, draw all points at full opacity
            if persistent_trails:
                alpha = 255
            else:
                elapsed_time = current_time - timestamp
                if elapsed_time > TRACE_FADE_TIME:
                    continue
                alpha = int(255 * (1 - elapsed_time / TRACE_FADE_TIME))
            
            trail_color = (color[0] * alpha // 255, color[1] * alpha // 255, color[2] * alpha // 255)
            
            if i < len(positions) - 1:
                next_x, next_y = positions[i + 1][1], positions[i + 1][2]
                pygame.draw.line(screen, trail_color, screen_coordinates(x, y), screen_coordinates(next_x, next_y), 2)
        
        latest_timestamp, latest_x, latest_y = positions[-1]
        screen_x, screen_y = screen_coordinates(latest_x, latest_y)
        pygame.draw.circle(screen, color, (screen_x, screen_y), POINT_RADIUS)
        
        font = pygame.font.Font(None, FONT_SIZE)
        label = font.render(f"#{int(tag_id)} ({latest_x:.1f}, {latest_y:.1f})", True, color)
        screen.blit(label, (screen_x + 10, screen_y - 10))

def handle_mouse_wheel(y):
    """Handle mouse wheel zoom with cursor as zoom center."""
    if not controls_enabled:
        return
        
    global scale_factor, offset_x, offset_y
    mouse_x, mouse_y = pygame.mouse.get_pos()
    pre_zoom_uwb_x, pre_zoom_uwb_y = uwb_coordinates(mouse_x, mouse_y)
    old_scale = scale_factor
    scale_factor = max(10, min(INITIAL_SCALE * 10, scale_factor + y * 5))
    new_screen_x, new_screen_y = screen_coordinates(pre_zoom_uwb_x, pre_zoom_uwb_y)
    offset_x += (mouse_x - new_screen_x)
    offset_y += (mouse_y - new_screen_y)

def print_status():
    """Print current status of controls and trails."""
    controls_status = "ENABLED" if controls_enabled else "DISABLED"
    trails_status = "PERSISTENT" if persistent_trails else "FADING"
    print(f"[INFO] Controls: {controls_status} | Trails: {trails_status}")

def main():
    global panning, last_mouse_pos, offset_x, offset_y, persistent_trails, controls_enabled
    clock = pygame.time.Clock()
    background = Background(BGPIC)
    
    # Print initial status
    print_status()
    print("[INFO] Press ESC to exit, SPACE for trails, ENTER for controls")
    
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT or (
                event.type == pygame.KEYDOWN and 
                event.key == pygame.K_ESCAPE
            ):
                print("[INFO] Closing program...")
                pygame.quit()
                sys.exit()
            elif event.type == pygame.MOUSEWHEEL:
                handle_mouse_wheel(event.y)
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                if controls_enabled:
                    panning = True
                    last_mouse_pos = pygame.mouse.get_pos()
            elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                panning = False
            elif event.type == pygame.MOUSEMOTION and panning and controls_enabled:
                current_pos = pygame.mouse.get_pos()
                offset_x += current_pos[0] - last_mouse_pos[0]
                offset_y += current_pos[1] - last_mouse_pos[1]
                last_mouse_pos = current_pos
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    persistent_trails = not persistent_trails
                    if not persistent_trails:
                        position_history.clear()
                    print_status()
                elif event.key == pygame.K_RETURN:
                    controls_enabled = not controls_enabled
                    if not controls_enabled:
                        panning = False  # Stop any ongoing panning
                    print_status()
        
        try:
            df = get_all_positions()        # GUI will run but much slower when df is empty 
            screen.fill(BACKGROUND_COLOR)
            background.draw(screen)
            draw_grid()
            draw_rectangle(RECT_WIDTH, RECT_HEIGHT)
            if df.empty is False:
                latest_positions = df.groupby('id').last().reset_index()
                draw_positions(latest_positions)
            
            pygame.display.update()
            
        except Exception as e:
            print(f"Error reading data: {e}")
        
        clock.tick(30)

if __name__ == "__main__":
    main()