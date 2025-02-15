"""
15 FEB - okay but does not fulfil a need right now
Further development - can take in a lot of points, then form straight-line vectors? 
For now, 
            SHELVED; NOT USED CAA 15 FEB

"""

import json
import math

def load_json(filepath):
    """Load JSON data from a file."""
    with open(filepath, "r") as file:
        return json.load(file)

def save_json(data, filepath):
    """Save JSON data to a file."""
    with open(filepath, "w") as file:
        json.dump(data, file, indent=4)

def distance(p1, p2):
    """Calculate Euclidean distance between two points."""
    return math.sqrt((p1["x"] - p2["x"])**2 + (p1["y"] - p2["y"])**2)

def filter_walls(points):
    """Extract walls from UWB trace points based on 'start' and 'end' markers."""
    walls = []
    current_wall_start = None

    for point in points:
        if point["type"] == "wall_start":
            current_wall_start = {"x": point["x"], "y": point["y"]}
        elif point["type"] == "wall_end" and current_wall_start:
            walls.append({"start": current_wall_start, "end": {"x": point["x"], "y": point["y"]}})
            current_wall_start = None  # Reset for the next segment
    
    return walls

def extract_obstacles(points, min_distance=0.3):
    """Extract obstacles, ensuring a minimum clearance between them."""
    obstacles = []
    
    for point in points:
        if point["type"] == "obstacle":
            pos = {"x": point["x"], "y": point["y"]}
            
            # Check if it's sufficiently far from existing obstacles
            if all(distance(pos, obs) > min_distance for obs in obstacles):
                obstacles.append(pos)
    
    return obstacles

def process_uwb_json(input_file="UWBViz/uwb_trace.json", output_file="UWBViz/uwb_filtered_map.json"):
    """Process UWB trace data, extract walls and obstacles, and save the filtered map."""
    data = load_json(input_file)
    
    points = data.get("points", [])
    walls = filter_walls(points)
    obstacles = extract_obstacles(points)
    
    filtered_data = {
        "walls": walls,
        "point_obstacles": obstacles,
        "clearance": 0.3  # Minimum clearance radius around obstacles
    }
    
    save_json(filtered_data, output_file)
    print(f"Filtered map saved to {output_file}")

# Example usage
process_uwb_json()
