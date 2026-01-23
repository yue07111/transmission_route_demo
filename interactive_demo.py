#!/usr/bin/env python
"""Interactive Transmission Line Routing Demo

Features:
- Drag to select start and goal points
- Add strip obstacles (rectangles)
- Add area obstacles (circles)
- Real-time path planning with animation
- Start button to begin planning

Usage:
  python interactive_demo.py --config configs/demo_config.yaml --map configs/demo_map.yaml
"""

from __future__ import annotations

import argparse
import math
import threading
import time
from typing import Dict, List, Optional, Tuple

import numpy as np
import pygame
import yaml
try:
    import matplotlib.cm as cm
    import matplotlib.colors as mcolors
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False

from src.builder import build_from_spec
from src.astar import AStarPlanner
from src.postprocess import place_towers
from src.environment import Environment, Cell, Point


# Color definitions (according to VISUALIZATION_LEGEND.md)
COLOR_BACKGROUND = (250, 255, 250)  # Greener background
COLOR_START = (255, 0, 0)  # Red - start point
COLOR_GOAL = (0, 0, 255)  # Blue - goal point
COLOR_PATH = (0, 255, 0)  # Green - final path
COLOR_TOWER_TURNING = (255, 200, 0)  # Yellow - turning towers (triangle)
COLOR_TOWER_STRAIGHT = (255, 0, 0)  # Red - straight towers (circle)
COLOR_STRIP = (153, 153, 153)  # Gray - strips (0.6 in matplotlib)
COLOR_STRIP_EDGE = (64, 64, 64)  # Dark gray - strip edges (0.25)
COLOR_AREA = (179, 179, 179)  # Light gray - areas (0.7 in matplotlib)
COLOR_AREA_EDGE = (77, 77, 77)  # Dark gray - area edges (0.3)
COLOR_BLOCKED = (0, 0, 0)  # Black - hard obstacles
COLOR_SENSITIVE_HARD = (89, 89, 89)  # Dark gray - sensitive facility hard zone (0.35)
COLOR_SENSITIVE_SOFT = (51, 51, 51)  # Dark gray - sensitive facility soft buffer (0.2)
COLOR_VISITED = (51, 51, 51)  # Dark gray - visited nodes (0.2)
COLOR_CORRIDOR = (255, 165, 0)  # Orange - corridor lines
COLOR_ROAD = (0, 100, 200)  # Blue - roads (tab:blue)
COLOR_TEXT = (235, 235, 235)
COLOR_DRAWING = (250, 240, 180)
COLOR_BUTTON = (70, 130, 180)
COLOR_BUTTON_HOVER = (100, 150, 200)


def load_yaml(path: str):
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def world_to_screen(env: Environment, world_pos: Point, screen_size: Tuple[int, int], 
                  zoom: float = 1.0, center: Optional[Point] = None,
                  pan_offset: Tuple[float, float] = (0.0, 0.0)) -> Tuple[int, int]:
    """Convert world coordinates to screen coordinates with zoom and pan"""
    x, y = world_pos
    
    if center is not None:
        # Zoom around center point
        cx, cy = center
        x = (x - cx) * zoom + cx
        y = (y - cy) * zoom + cy
    
    # Convert to screen coordinates first
    sx = int(x * screen_size[0] / env.width_m)
    sy = int(screen_size[1] - y * screen_size[1] / env.height_m)  # Flip Y axis
    
    # Apply pan offset in screen coordinates (direct pixel offset)
    sx += int(pan_offset[0])
    sy += int(pan_offset[1])
    
    return (sx, sy)


def screen_to_world(env: Environment, screen_pos: Tuple[int, int], screen_size: Tuple[int, int],
                   zoom: float = 1.0, center: Optional[Point] = None,
                   pan_offset: Tuple[float, float] = (0.0, 0.0)) -> Point:
    """Convert screen coordinates to world coordinates with zoom and pan"""
    sx, sy = screen_pos
    
    # Remove pan offset in screen coordinates first
    sx -= int(pan_offset[0])
    sy -= int(pan_offset[1])
    
    # Convert to world coordinates
    x = sx * env.width_m / screen_size[0]
    y = (screen_size[1] - sy) * env.height_m / screen_size[1]  # Flip Y axis
    
    if center is not None:
        # Apply inverse zoom transformation
        cx, cy = center
        x = (x - cx) / zoom + cx
        y = (y - cy) / zoom + cy
    
    return (x, y)


def draw_circle_area(screen: pygame.Surface, env: Environment, center: Point, radius_m: float, 
                     color: Tuple[int, int, int], edge_color: Tuple[int, int, int],
                     screen_size: Tuple[int, int], alpha: int = 255, 
                     zoom: float = 1.0, zoom_center: Optional[Point] = None,
                     pan_offset: Tuple[float, float] = (0.0, 0.0)):
    """Draw circular area"""
    screen_pos = world_to_screen(env, center, screen_size, zoom, zoom_center, pan_offset)
    radius_px = int(radius_m * screen_size[0] / env.width_m * zoom)
    # Create surface for alpha blending
    temp_surf = pygame.Surface((radius_px * 2 + 4, radius_px * 2 + 4), pygame.SRCALPHA)
    pygame.draw.circle(temp_surf, (*color, alpha), (radius_px + 2, radius_px + 2), radius_px, width=0)
    screen.blit(temp_surf, (screen_pos[0] - radius_px - 2, screen_pos[1] - radius_px - 2))
    pygame.draw.circle(screen, edge_color, screen_pos, radius_px, width=2)


def draw_rect_strip(screen: pygame.Surface, env: Environment, center: Point, length_m: float, 
                    width_m: float, angle_deg: float, color: Tuple[int, int, int], 
                    edge_color: Tuple[int, int, int], screen_size: Tuple[int, int], alpha: int = 255,
                    zoom: float = 1.0, zoom_center: Optional[Point] = None,
                    pan_offset: Tuple[float, float] = (0.0, 0.0)):
    """Draw rotated rectangle strip"""
    cx, cy = center
    th = math.radians(angle_deg)
    c, s = math.cos(th), math.sin(th)
    hx, hy = length_m / 2.0, width_m / 2.0
    
    # Local coordinate corners
    corners = [
        (hx, hy),
        (hx, -hy),
        (-hx, -hy),
        (-hx, hy),
    ]
    
    # Rotate and transform to world coordinates
    world_corners = []
    for x, y in corners:
        xr = x * c - y * s
        yr = x * s + y * c
        world_corners.append((cx + xr, cy + yr))
    
    # Transform to screen coordinates
    screen_corners = [world_to_screen(env, p, screen_size, zoom, zoom_center, pan_offset) for p in world_corners]
    # Create surface for alpha blending
    min_x = min(p[0] for p in screen_corners)
    max_x = max(p[0] for p in screen_corners)
    min_y = min(p[1] for p in screen_corners)
    max_y = max(p[1] for p in screen_corners)
    temp_surf = pygame.Surface((int(max_x - min_x + 4), int(max_y - min_y + 4)), pygame.SRCALPHA)
    adjusted_corners = [(p[0] - min_x + 2, p[1] - min_y + 2) for p in screen_corners]
    pygame.draw.polygon(temp_surf, (*color, alpha), adjusted_corners, width=0)
    screen.blit(temp_surf, (int(min_x - 2), int(min_y - 2)))
    pygame.draw.polygon(screen, edge_color, screen_corners, width=2)


def draw_path(screen: pygame.Surface, env: Environment, path_cells: List[Cell], 
              color: Tuple[int, int, int], screen_size: Tuple[int, int], width: int = 3,
              zoom: float = 1.0, zoom_center: Optional[Point] = None,
              pan_offset: Tuple[float, float] = (0.0, 0.0)):
    """Draw path"""
    if len(path_cells) < 2:
        return
    
    screen_points = []
    for cell in path_cells:
        world_pos = env.cell_center_world(cell)
        screen_points.append(world_to_screen(env, world_pos, screen_size, zoom, zoom_center, pan_offset))
    
    for i in range(len(screen_points) - 1):
        pygame.draw.line(screen, color, screen_points[i], screen_points[i + 1], width=width)


def project_point_to_path(tower_pos: Point, path_cells: List[Cell], env: Environment) -> Point:
    """Project a tower position onto the nearest point on the path line"""
    if len(path_cells) < 2:
        return tower_pos
    
    min_dist = float('inf')
    best_point = tower_pos
    
    # Check each segment of the path
    for i in range(len(path_cells) - 1):
        a = env.cell_center_world(path_cells[i])
        b = env.cell_center_world(path_cells[i + 1])
        
        # Vector from a to b
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        seg_len_sq = dx * dx + dy * dy
        
        if seg_len_sq < 1e-6:
            continue
        
        # Vector from a to tower
        tx = tower_pos[0] - a[0]
        ty = tower_pos[1] - a[1]
        
        # Project onto segment
        t = (tx * dx + ty * dy) / seg_len_sq
        t = max(0.0, min(1.0, t))  # Clamp to segment
        
        # Projected point
        proj = (a[0] + t * dx, a[1] + t * dy)
        
        # Distance from tower to projected point
        dist = math.hypot(tower_pos[0] - proj[0], tower_pos[1] - proj[1])
        
        if dist < min_dist:
            min_dist = dist
            best_point = proj
    
    return best_point


def draw_line(screen: pygame.Surface, env: Environment, points: List[Point], 
              color: Tuple[int, int, int], screen_size: Tuple[int, int], 
              width: int = 2, linestyle: str = '-',
              zoom: float = 1.0, zoom_center: Optional[Point] = None,
              pan_offset: Tuple[float, float] = (0.0, 0.0)):
    """Draw line (for corridors and roads)"""
    if len(points) < 2:
        return
    
    screen_points = [world_to_screen(env, p, screen_size, zoom, zoom_center, pan_offset) for p in points]
    
    if linestyle == '--':  # Dashed
        for i in range(len(screen_points) - 1):
            pygame.draw.line(screen, color, screen_points[i], screen_points[i + 1], width=width)
    elif linestyle == ':':  # Dotted
        for i in range(len(screen_points) - 1):
            # Draw as series of dots
            dx = screen_points[i + 1][0] - screen_points[i][0]
            dy = screen_points[i + 1][1] - screen_points[i][1]
            dist = math.hypot(dx, dy)
            if dist > 0:
                steps = int(dist / 5)
                for j in range(steps + 1):
                    t = j / steps if steps > 0 else 0
                    x = int(screen_points[i][0] + dx * t)
                    y = int(screen_points[i][1] + dy * t)
                    pygame.draw.circle(screen, color, (x, y), width)
    elif linestyle == '-.':  # Dash-dot
        for i in range(len(screen_points) - 1):
            pygame.draw.line(screen, color, screen_points[i], screen_points[i + 1], width=width)
    else:  # Solid
        for i in range(len(screen_points) - 1):
            pygame.draw.line(screen, color, screen_points[i], screen_points[i + 1], width=width)


def draw_heatmap(screen: pygame.Surface, env: Environment, screen_size: Tuple[int, int], 
                  alpha: float = 0.55, heatmap_clip_percentile: float = 95.0):
    """Draw cost heatmap matching matplotlib style"""
    if env.costmap is None:
        return
    
    # Create temporary surface for heatmap
    heatmap_surf = pygame.Surface(screen_size, pygame.SRCALPHA)
    
    costmap = env.costmap.copy()
    # Show penalties only (clip negatives so rewards do not confuse the "penalty" heatmap)
    costmap = np.maximum(costmap, 0.0)
    
    # Calculate vmax using percentile (matching matplotlib)
    if costmap.size > 0:
        flat_cm = costmap.flatten()
        vmax = max(1e-6, float(np.percentile(flat_cm, heatmap_clip_percentile)))
    else:
        vmax = 1.0
    
    # Use matplotlib colormap for exact color matching with final.png
    # matplotlib's imshow without cmap uses default (usually 'viridis'), 
    # but for penalty heatmaps, 'hot' or 'YlOrRd' is more appropriate
    # Use 'YlOrRd' (Yellow-Orange-Red) which matches the description in VISUALIZATION_LEGEND.md
    if HAS_MATPLOTLIB:
        try:
            # YlOrRd: Yellow -> Orange -> Red (perfect for penalty heatmap)
            colormap = cm.get_cmap('YlOrRd')
        except:
            try:
                # Fallback: 'hot' colormap (black->red->yellow), reverse for yellow->red
                colormap = cm.get_cmap('hot').reversed()
            except:
                colormap = None
        if colormap:
            norm = mcolors.Normalize(vmin=0.0, vmax=vmax)
    else:
        colormap = None
    
    # Draw heatmap using rectangles for full coverage
    cell_w = screen_size[0] / env.grid_w
    cell_h = screen_size[1] / env.grid_h
    
    for iy in range(env.grid_h):
        for ix in range(env.grid_w):
            cost = float(costmap[iy, ix])
            if cost <= 1e-6:  # Skip very small values
                continue
            
            # Normalize to 0-1
            t = min(1.0, cost / vmax)
            
            if colormap:
                # Use matplotlib colormap for exact color matching
                rgba = colormap(norm(cost))
                r = int(rgba[0] * 255)
                g = int(rgba[1] * 255)
                b = int(rgba[2] * 255)
            else:
                # Fallback: Smooth yellow-red gradient
                t_smooth = t * t * (3.0 - 2.0 * t)  # Smoothstep
                r = 255
                g = int(255 * (1.0 - t_smooth) * (1.0 + 0.3 * t_smooth * (1.0 - t_smooth)))
                g = max(0, min(255, g))
                b = 0
            
            color = (r, g, b, int(255 * alpha))
            
            # Calculate screen position for this cell
            world_pos = env.cell_center_world((ix, iy))
            screen_x = int(world_pos[0] * screen_size[0] / env.width_m)
            screen_y = int(screen_size[1] - world_pos[1] * screen_size[1] / env.height_m)
            
            # Draw rectangle covering the cell
            rect = pygame.Rect(int(screen_x - cell_w/2), int(screen_y - cell_h/2), 
                             int(cell_w) + 1, int(cell_h) + 1)
            pygame.draw.rect(heatmap_surf, color, rect)
    
    screen.blit(heatmap_surf, (0, 0))


class InteractiveDemo:
    def __init__(self, config: Dict, map_data: Dict, screen_size: Tuple[int, int] = (1200, 800)):
        self.config = config
        self.map_data = map_data.copy()  # Mutable copy
        self.screen_size = screen_size
        
        # Initialize environment
        self.env, self.constraint_manager = build_from_spec(config, self.map_data)
        self.planner = AStarPlanner(config.get('planner', {}))
        
        # Start and goal points
        self.start = tuple(map_data['start'])
        self.goal = tuple(map_data['goal'])
        
        # Interactive state
        self.dragging = None  # 'start' | 'goal' | None
        self.dragging_point = None  # Track if we're dragging a point (start/goal)
        self.drawing_strip = False
        self.drawing_area = False
        self.strip_start_pos = None
        self.area_center = None
        self.area_radius = None
        
        # Planning state
        self.planning = False
        self.planning_thread = None
        self.result = None
        self.towers = None
        
        # Animation state
        self.animation_visited = []  # For progressive display
        self.animation_path = []
        self.animation_visited_progress = 0.0  # 0.0 to 1.0 for visited nodes
        self.animation_path_progress = 0.0  # 0.0 to 1.0 for path
        self.show_animation = False
        self.animation_phase = 'visited'  # 'visited' or 'path'
        
        # Button state
        self.start_button_rect = pygame.Rect(10, screen_size[1] - 50, 120, 40)
        self.start_button_hover = False
        self.random_obstacles_button_rect = pygame.Rect(140, screen_size[1] - 50, 200, 40)
        self.random_obstacles_button_hover = False
        
        # Zoom state
        self.zoom = 1.0
        self.zoom_center = None  # World coordinates of zoom center
        
        # Pan state (drag to pan)
        self.pan_offset = (0.0, 0.0)  # (x, y) offset in world coordinates
        self.panning = False
        self.pan_start_pos = None  # Screen coordinates where pan started
        self.pan_start_world = None  # World coordinates when pan started
        
        # Thread lock for thread-safe updates
        self.lock = threading.Lock()
    
    def replan_async(self):
        """Start planning in a separate thread"""
        if self.planning:
            return
        
        self.planning = True
        self.show_animation = True
        self.animation_visited_progress = 0.0
        self.animation_path_progress = 0.0
        self.animation_visited = []  # Will be updated in real-time during planning
        self.animation_path = []
        self.animation_phase = 'visited'
        
        def progress_callback(visited_nodes):
            """Callback to update visited nodes during planning"""
            with self.lock:
                self.animation_visited = visited_nodes.copy()
        
        def plan_thread():
            try:
                start_cell = self.env.world_to_grid(self.start)
                goal_cell = self.env.world_to_grid(self.goal)
                
                # Check if start/goal are in obstacles
                if self.env.is_blocked(start_cell):
                    print(f"Warning: Start point {self.start} is in obstacle")
                if self.env.is_blocked(goal_cell):
                    print(f"Warning: Goal point {self.goal} is in obstacle")
                
                # Plan with progress callback for real-time visualization
                result = self.planner.plan(self.env, self.constraint_manager, start_cell, goal_cell,
                                         progress_callback=progress_callback)
                
                with self.lock:
                    self.result = result
                    
                    if result.success:
                        self.towers = place_towers(self.env, result.path_cells, 
                                                   self.config.get('postprocess', {}))
                        # Prepare animation data - start with visited nodes
                        self.animation_visited = result.visited_order.copy()
                        self.animation_path = result.path_cells.copy()
                        self.animation_phase = 'visited'
                        self.animation_visited_progress = 0.0
                        self.animation_path_progress = 0.0
                    else:
                        self.towers = None
                        print("Path planning failed")
                    
                    self.planning = False
            except Exception as e:
                print(f"Planning error: {e}")
                with self.lock:
                    self.planning = False
        
        self.planning_thread = threading.Thread(target=plan_thread, daemon=True)
        self.planning_thread.start()
    
    def update_obstacles(self):
        """Rebuild environment after updating obstacles"""
        self.env, self.constraint_manager = build_from_spec(self.config, self.map_data)
        # Don't auto-replan, wait for user to click Start
    
    def add_strip(self, center: Point, length_m: float, width_m: float, angle_deg: float):
        """Add strip obstacle"""
        strip_id = f"strip_interactive_{len(self.map_data.get('strips', [])) + 1}"
        strip = {
            'id': strip_id,
            'center': list(center),
            'length_m': length_m,
            'width_m': width_m,
            'angle_deg': angle_deg,
            'is_crossable': True,
            'min_cross_angle_deg': 30.0,
            'cross_cost': 20.0,
            'note': 'Interactive'
        }
        if 'strips' not in self.map_data:
            self.map_data['strips'] = []
        self.map_data['strips'].append(strip)
        self.update_obstacles()
    
    def add_area(self, center: Point, radius_m: float):
        """Add area obstacle"""
        area_id = f"area_interactive_{len(self.map_data.get('areas', [])) + 1}"
        area = {
            'id': area_id,
            'center': list(center),
            'radius_m': radius_m,
            'is_enterable': False,
            'is_crossable': False,
            'damping': 0.0,
            'entry_penalty': 0.0,
            'cross_cost': 0.0,
            'note': 'Interactive'
        }
        if 'areas' not in self.map_data:
            self.map_data['areas'] = []
        self.map_data['areas'].append(area)
        self.update_obstacles()
    
    def get_view_bounds(self) -> Tuple[Point, Point]:
        """Get current view bounds in world coordinates"""
        # Get screen corners in world coordinates
        top_left = screen_to_world(self.env, (0, 0), self.screen_size, 
                                   self.zoom, self.zoom_center, self.pan_offset)
        bottom_right = screen_to_world(self.env, self.screen_size, self.screen_size,
                                       self.zoom, self.zoom_center, self.pan_offset)
        
        # Ensure bounds are within map limits
        min_x = max(0, min(top_left[0], bottom_right[0]))
        max_x = min(self.env.width_m, max(top_left[0], bottom_right[0]))
        min_y = max(0, min(top_left[1], bottom_right[1]))
        max_y = min(self.env.height_m, max(top_left[1], bottom_right[1]))
        
        return ((min_x, min_y), (max_x, max_y))
    
    def add_random_obstacles_in_view(self, num_strips: int = 5, num_areas: int = 5, 
                                     num_sensitive: int = 3, num_corridors: int = 2, 
                                     num_roads: int = 2):
        """Add random obstacles in the current view (removes ALL existing obstacles first)"""
        import random
        
        # Remove ALL existing obstacles and features
        self.map_data['strips'] = []
        self.map_data['areas'] = []
        self.map_data['sensitive_points'] = []
        self.map_data['corridor'] = {}
        self.map_data['roads'] = []
        
        # Rebuild environment after removing obstacles
        self.update_obstacles()
        
        view_min, view_max = self.get_view_bounds()
        view_width = view_max[0] - view_min[0]
        view_height = view_max[1] - view_min[1]
        
        # Reference original config parameters:
        # Strips: length 1500-3000m, width 100-140m
        # Areas: radius 250-400m
        # Sensitive points: radius_hard 180-250m, radius_soft 450-600m
        # Corridors: lines with multiple points
        # Roads: lines with multiple points
        
        # Add random strips (reference original: length 1500-3000m, width 100-140m)
        max_strip_length = min(3000, max(1500, view_width * 0.6))
        min_strip_length = min(1500, max(800, view_width * 0.3))
        
        for _ in range(num_strips):
            center_x = random.uniform(view_min[0], view_max[0])
            center_y = random.uniform(view_min[1], view_max[1])
            center = (center_x, center_y)
            
            length_m = random.uniform(min_strip_length, max_strip_length)
            width_m = random.uniform(100, 140)
            angle_deg = random.uniform(0, 360)
            
            self.add_strip(center, length_m, width_m, angle_deg)
        
        # Add random areas (reference original: radius 250-400m)
        max_area_radius = min(400, max(250, min(view_width, view_height) * 0.15))
        min_area_radius = min(250, max(150, min(view_width, view_height) * 0.08))
        
        for _ in range(num_areas):
            center_x = random.uniform(view_min[0], view_max[0])
            center_y = random.uniform(view_min[1], view_max[1])
            center = (center_x, center_y)
            
            radius_m = random.uniform(min_area_radius, max_area_radius)
            self.add_area(center, radius_m)
        
        # Add random sensitive points (reference original: radius_hard 180-250m, radius_soft 450-600m)
        for i in range(num_sensitive):
            center_x = random.uniform(view_min[0], view_max[0])
            center_y = random.uniform(view_min[1], view_max[1])
            
            radius_hard_m = random.uniform(180, 250)
            radius_soft_m = random.uniform(450, 600)
            soft_penalty = random.uniform(8, 12)
            
            sensitive_id = f"sensitive_random_{i+1}"
            sensitive_point = {
                'id': sensitive_id,
                'center': [center_x, center_y],
                'radius_hard_m': radius_hard_m,
                'radius_soft_m': radius_soft_m,
                'soft_penalty': soft_penalty,
                'note': 'Random'
            }
            if 'sensitive_points' not in self.map_data:
                self.map_data['sensitive_points'] = []
            self.map_data['sensitive_points'].append(sensitive_point)
        
        # Add random corridors (lines crossing the view)
        corridor_others = []
        for i in range(num_corridors):
            # Generate a line that crosses the view
            if random.random() < 0.5:  # Horizontal line
                y = random.uniform(view_min[1], view_max[1])
                num_points = random.randint(3, 5)
                points = []
                for j in range(num_points):
                    x = view_min[0] + (view_max[0] - view_min[0]) * j / (num_points - 1)
                    y_offset = random.uniform(-50, 50)  # Add some variation
                    points.append([x, y + y_offset])
            else:  # Vertical line
                x = random.uniform(view_min[0], view_max[0])
                num_points = random.randint(3, 5)
                points = []
                for j in range(num_points):
                    y = view_min[1] + (view_max[1] - view_min[1]) * j / (num_points - 1)
                    x_offset = random.uniform(-50, 50)  # Add some variation
                    points.append([x + x_offset, y])
            
            corridor_others.append({
                'id': f'corridor_random_{i+1}',
                'points': points
            })
        
        # Set primary corridor (first one) and others
        if corridor_others:
            self.map_data['corridor'] = {
                'primary': corridor_others[0],
                'others': corridor_others[1:] if len(corridor_others) > 1 else []
            }
        else:
            self.map_data['corridor'] = {}
        
        # Add random roads (lines crossing the view)
        for i in range(num_roads):
            # Generate a line that crosses the view
            if random.random() < 0.5:  # Horizontal road
                y = random.uniform(view_min[1], view_max[1])
                points = [
                    [view_min[0], y],
                    [view_max[0], y]
                ]
            else:  # Vertical road
                x = random.uniform(view_min[0], view_max[0])
                points = [
                    [x, view_min[1]],
                    [x, view_max[1]]
                ]
            
            road = {
                'id': f'road_random_{i+1}',
                'points': points,
                'note': 'Random'
            }
            if 'roads' not in self.map_data:
                self.map_data['roads'] = []
            self.map_data['roads'].append(road)
        
        # Rebuild environment with all new features
        self.update_obstacles()
        
        # Clear path when obstacles are added
        with self.lock:
            self.result = None
            self.towers = None
            self.animation_visited = []
            self.animation_path = []
            self.show_animation = False
    
    def handle_mouse_down(self, pos: Tuple[int, int], button: int):
        """Handle mouse down event"""
        # Check button click
        if button == 1 and self.start_button_rect.collidepoint(pos):
            self.replan_async()
            return
        
        if button == 1 and self.random_obstacles_button_rect.collidepoint(pos):
            self.add_random_obstacles_in_view()
            return
        
        world_pos = screen_to_world(self.env, pos, self.screen_size, self.zoom, self.zoom_center, self.pan_offset)
        
        # Left button: check if clicking start/goal point, otherwise pan
        if button == 1:  # Left button
            dist_start = math.hypot(world_pos[0] - self.start[0], world_pos[1] - self.start[1])
            dist_goal = math.hypot(world_pos[0] - self.goal[0], world_pos[1] - self.goal[1])
            
            # Check if clicking start or goal point (within 100 meters)
            if dist_start < 100:
                self.dragging = 'start'
                self.dragging_point = True
                # Clear path when starting to drag
                with self.lock:
                    self.result = None
                    self.towers = None
                    self.animation_visited = []
                    self.animation_path = []
                    self.show_animation = False
                return
            elif dist_goal < 100:
                self.dragging = 'goal'
                self.dragging_point = True
                # Clear path when starting to drag
                with self.lock:
                    self.result = None
                    self.towers = None
                    self.animation_visited = []
                    self.animation_path = []
                    self.show_animation = False
                return
            
            # Not clicking on start/goal, start panning
            self.panning = True
            self.pan_start_pos = pos
            # Get world position at pan start (for reference)
            self.pan_start_world = screen_to_world(self.env, pos, self.screen_size, 
                                                   self.zoom, self.zoom_center, self.pan_offset)
            return
        
        # Right button or middle button for panning (backup)
        if button == 2 or button == 3:  # Middle button (2) or Right button (3)
            self.panning = True
            self.pan_start_pos = pos
            # Get world position at pan start (for reference)
            self.pan_start_world = screen_to_world(self.env, pos, self.screen_size, 
                                                   self.zoom, self.zoom_center, self.pan_offset)
            return
        
        # Start drawing strip or area (for other buttons)
        if self.drawing_strip:
            self.strip_start_pos = world_pos
        elif self.drawing_area:
            self.area_center = world_pos
    
    def handle_mouse_up(self, pos: Tuple[int, int], button: int):
        """Handle mouse up event"""
        # Stop panning on left button release
        if button == 1:
            if self.panning:
                self.panning = False
                self.pan_start_pos = None
                self.pan_start_world = None
            elif self.dragging_point:
                # Stop dragging point
                world_pos = screen_to_world(self.env, pos, self.screen_size, self.zoom, self.zoom_center, self.pan_offset)
                # Limit to boundaries
                world_pos = (
                    max(0, min(self.env.width_m, world_pos[0])),
                    max(0, min(self.env.height_m, world_pos[1]))
                )
                if self.dragging == 'start':
                    self.start = world_pos
                elif self.dragging == 'goal':
                    self.goal = world_pos
                self.dragging = None
                self.dragging_point = False
                # Path is already cleared, user needs to click Start to replan
            return
        
        # Stop panning on right/middle button release
        if button == 2 or button == 3:
            self.panning = False
            self.pan_start_pos = None
            self.pan_start_world = None
            return
        
        # Handle drawing completion
        world_pos = screen_to_world(self.env, pos, self.screen_size, self.zoom, self.zoom_center, self.pan_offset)
        
        if self.drawing_strip and self.strip_start_pos and button == 1:
            # Complete strip drawing
            dx = world_pos[0] - self.strip_start_pos[0]
            dy = world_pos[1] - self.strip_start_pos[1]
            length_m = math.hypot(dx, dy)
            if length_m > 50:  # Minimum length
                angle_deg = math.degrees(math.atan2(dy, dx))
                width_m = 100.0  # Default width
                center = ((self.strip_start_pos[0] + world_pos[0]) / 2,
                         (self.strip_start_pos[1] + world_pos[1]) / 2)
                self.add_strip(center, length_m, width_m, angle_deg)
            self.strip_start_pos = None
            self.drawing_strip = False
        
        elif self.drawing_area and self.area_center and button == 1:
            # Complete area drawing
            if self.area_radius and self.area_radius > 50:  # Minimum radius
                self.add_area(self.area_center, self.area_radius)
            self.area_center = None
            self.area_radius = None
            self.drawing_area = False
    
    def handle_mouse_motion(self, pos: Tuple[int, int]):
        """Handle mouse motion event"""
        # Check button hover
        self.start_button_hover = self.start_button_rect.collidepoint(pos)
        self.random_obstacles_button_hover = self.random_obstacles_button_rect.collidepoint(pos)
        
        # Handle panning (left button drag when not dragging a point)
        if self.panning and self.pan_start_pos:
            # Calculate screen delta (in pixels)
            dx = pos[0] - self.pan_start_pos[0]
            dy = pos[1] - self.pan_start_pos[1]
            
            # Update pan offset directly in screen coordinates (pixels)
            # Add because we want to move the view in the same direction as mouse
            self.pan_offset = (
                self.pan_offset[0] + dx,
                self.pan_offset[1] + dy
            )
            
            # Update pan start position for smooth continuous panning
            self.pan_start_pos = pos
            return
        
        # Handle dragging start/goal point
        if self.dragging_point and self.dragging:
            world_pos = screen_to_world(self.env, pos, self.screen_size, self.zoom, self.zoom_center, self.pan_offset)
            # Limit to boundaries
            world_pos = (
                max(0, min(self.env.width_m, world_pos[0])),
                max(0, min(self.env.height_m, world_pos[1]))
            )
            if self.dragging == 'start':
                self.start = world_pos
            elif self.dragging == 'goal':
                self.goal = world_pos
        
        elif self.drawing_area and self.area_center:
            world_pos = screen_to_world(self.env, pos, self.screen_size, self.zoom, self.zoom_center, self.pan_offset)
            self.area_radius = math.hypot(world_pos[0] - self.area_center[0], 
                                          world_pos[1] - self.area_center[1])
    
    def handle_mouse_wheel(self, pos: Tuple[int, int], y: float):
        """Handle mouse wheel event for zooming"""
        # Zoom factor
        zoom_factor = 1.15
        if y > 0:  # Scroll up - zoom in
            new_zoom = self.zoom * zoom_factor
        elif y < 0:  # Scroll down - zoom out
            new_zoom = self.zoom / zoom_factor
        else:
            return
        
        # Limit zoom range
        new_zoom = max(0.5, min(5.0, new_zoom))
        
        if abs(new_zoom - self.zoom) < 0.01:
            return
        
        # Get mouse position in world coordinates (current zoom)
        mouse_world = screen_to_world(self.env, pos, self.screen_size, self.zoom, self.zoom_center, self.pan_offset)
        
        # Update zoom and set center to mouse position
        self.zoom = new_zoom
        self.zoom_center = mouse_world
    
    def update_animation(self, dt: float):
        """Update animation progress"""
        if not self.show_animation:
            return
        
        with self.lock:
            if not self.planning and self.result and self.result.success:
                # Planning complete, animate the results
                if self.animation_phase == 'visited' and self.animation_visited:
                    # Animate visited nodes (slower speed)
                    self.animation_visited_progress = min(1.0, self.animation_visited_progress + dt * 0.8)
                    n_visited = int(len(self.animation_visited) * self.animation_visited_progress)
                    
                    # Switch to path animation when visited nodes are fully shown
                    if self.animation_visited_progress >= 1.0:
                        self.animation_phase = 'path'
                        self.animation_path_progress = 0.0
                elif self.animation_phase == 'path' and self.animation_path:
                    # Animate path (slower speed)
                    self.animation_path_progress = min(1.0, self.animation_path_progress + dt * 0.6)
    
    def draw(self, screen: pygame.Surface):
        """Draw scene"""
        screen.fill(COLOR_BACKGROUND)
        
        # Heatmap removed for interactive demo (as per user request)
        # draw_heatmap(screen, self.env, self.screen_size, alpha=0.55, heatmap_clip_percentile=95.0)
        
        # Draw hard obstacles overlay (semi-transparent black, matching matplotlib)
        if self.env.hard_mask is not None:
            hard_surf = pygame.Surface(self.screen_size, pygame.SRCALPHA)
            step = max(1, self.env.grid_w // 100)
            for iy in range(0, self.env.grid_h, step):
                for ix in range(0, self.env.grid_w, step):
                    if self.env.hard_mask[iy, ix]:
                        world_pos = self.env.cell_center_world((ix, iy))
                        screen_pos = world_to_screen(self.env, world_pos, self.screen_size, 
                                                    self.zoom, self.zoom_center, self.pan_offset)
                        # Black with alpha=0.25 (matching matplotlib)
                        pygame.draw.circle(hard_surf, (0, 0, 0, 64), screen_pos, 
                                         max(1, int(step // 2 * self.zoom)))
            screen.blit(hard_surf, (0, 0))
        
        # Draw hard obstacles (black semi-transparent)
        if self.env.hard_mask is not None:
            # Simplified: draw as overlay
            pass  # Hard mask visualization can be added if needed
        
        # Draw obstacles
        # Strips (gray rectangles)
        for s in self.map_data.get('strips', []):
            center = tuple(s['center'])
            if s.get('is_crossable', True):
                color = COLOR_STRIP
                edge_color = COLOR_STRIP_EDGE
                alpha = 204  # 0.8 * 255
            else:
                color = COLOR_BLOCKED
                edge_color = COLOR_BLOCKED
                alpha = 128  # Semi-transparent black
            draw_rect_strip(screen, self.env, center, s['length_m'], s['width_m'], 
                          s['angle_deg'], color, edge_color, self.screen_size, alpha=alpha,
                          zoom=self.zoom, zoom_center=self.zoom_center, pan_offset=self.pan_offset)
        
        # Areas (light gray circles)
        for a in self.map_data.get('areas', []):
            center = tuple(a['center'])
            if a.get('is_enterable', True) and a.get('is_crossable', True):
                color = COLOR_AREA
                edge_color = COLOR_AREA_EDGE
                alpha = 128  # 0.5 * 255
            else:
                color = COLOR_BLOCKED
                edge_color = COLOR_BLOCKED
                alpha = 128
            draw_circle_area(screen, self.env, center, a['radius_m'], color, edge_color, 
                           self.screen_size, alpha=alpha, zoom=self.zoom, zoom_center=self.zoom_center,
                           pan_offset=self.pan_offset)
        
        # Sensitive facilities
        for p in self.map_data.get('sensitive_points', []):
            center = tuple(p['center'])
            # Hard zone (inner circle)
            draw_circle_area(screen, self.env, center, p['radius_hard_m'], 
                           COLOR_SENSITIVE_HARD, (26, 26, 26), self.screen_size, alpha=230,
                           zoom=self.zoom, zoom_center=self.zoom_center, pan_offset=self.pan_offset)
            # Soft buffer (outer dashed circle)
            screen_pos = world_to_screen(self.env, center, self.screen_size, 
                                        self.zoom, self.zoom_center, self.pan_offset)
            radius_px = int(p['radius_soft_m'] * self.screen_size[0] / self.env.width_m * self.zoom)
            # Draw dashed circle
            for angle in range(0, 360, 10):
                if angle % 20 < 10:  # Create dash effect
                    rad = math.radians(angle)
                    x1 = int(screen_pos[0] + radius_px * math.cos(rad))
                    y1 = int(screen_pos[1] + radius_px * math.sin(rad))
                    rad2 = math.radians(angle + 10)
                    x2 = int(screen_pos[0] + radius_px * math.cos(rad2))
                    y2 = int(screen_pos[1] + radius_px * math.sin(rad2))
                    pygame.draw.line(screen, COLOR_SENSITIVE_SOFT, (x1, y1), (x2, y2), width=1)
        
        # Corridor lines (orange)
        corridor = self.map_data.get('corridor', {})
        if corridor:
            primary = corridor.get('primary')
            if primary:
                points = [tuple(p) for p in primary['points']]
                draw_line(screen, self.env, points, COLOR_CORRIDOR, self.screen_size, 
                         width=2, linestyle='--', zoom=self.zoom, zoom_center=self.zoom_center,
                         pan_offset=self.pan_offset)
            for other in corridor.get('others', []):
                points = [tuple(p) for p in other['points']]
                draw_line(screen, self.env, points, COLOR_CORRIDOR, self.screen_size, 
                         width=1, linestyle=':', zoom=self.zoom, zoom_center=self.zoom_center,
                         pan_offset=self.pan_offset)
        
        # Roads (blue)
        for r in self.map_data.get('roads', []):
            points = [tuple(p) for p in r['points']]
            draw_line(screen, self.env, points, COLOR_ROAD, self.screen_size, 
                     width=2, linestyle='-.', zoom=self.zoom, zoom_center=self.zoom_center,
                     pan_offset=self.pan_offset)
        
        # Draw visited nodes (real-time during planning or animation)
        with self.lock:
            if self.animation_visited:
                # Show all visited nodes during planning, or animated subset after planning
                if self.planning:
                    # Real-time: show all visited nodes during planning
                    step = max(1, len(self.animation_visited) // 2000)  # Sample for performance
                    for cell in self.animation_visited[::step]:
                        world_pos = self.env.cell_center_world(cell)
                        screen_pos = world_to_screen(self.env, world_pos, self.screen_size,
                                                    self.zoom, self.zoom_center, self.pan_offset)
                        pygame.draw.circle(screen, COLOR_VISITED, screen_pos, max(1, int(1 * self.zoom)))
                elif self.show_animation and self.animation_phase == 'visited':
                    # Animated: show progressive subset
                    n_visited = int(len(self.animation_visited) * self.animation_visited_progress)
                    if n_visited > 0:
                        step = max(1, n_visited // 1000)
                        for cell in self.animation_visited[:n_visited:step]:
                            world_pos = self.env.cell_center_world(cell)
                            screen_pos = world_to_screen(self.env, world_pos, self.screen_size,
                                                        self.zoom, self.zoom_center, self.pan_offset)
                            pygame.draw.circle(screen, COLOR_VISITED, screen_pos, max(1, int(1 * self.zoom)))
            
            # Draw path (animation)
            if self.show_animation and self.animation_path and len(self.animation_path) >= 2:
                if self.animation_phase == 'path':
                    n_path = int(len(self.animation_path) * self.animation_path_progress)
                    if n_path >= 2:
                        draw_path(screen, self.env, self.animation_path[:n_path], COLOR_PATH, 
                                 self.screen_size, width=max(1, int(3 * self.zoom)),
                                 zoom=self.zoom, zoom_center=self.zoom_center, pan_offset=self.pan_offset)
                elif self.animation_phase == 'visited' and self.animation_visited_progress >= 1.0:
                    # Show full path when visited animation completes
                    draw_path(screen, self.env, self.animation_path, COLOR_PATH, 
                             self.screen_size, width=max(1, int(3 * self.zoom)),
                             zoom=self.zoom, zoom_center=self.zoom_center, pan_offset=self.pan_offset)
            
            # Draw final path and towers if animation complete
            if not self.planning and self.result and self.result.success:
                if not self.show_animation or (self.animation_phase == 'path' and 
                                               self.animation_path_progress >= 1.0):
                    draw_path(screen, self.env, self.result.path_cells, COLOR_PATH, 
                             self.screen_size, width=max(1, int(3 * self.zoom)),
                             zoom=self.zoom, zoom_center=self.zoom_center, pan_offset=self.pan_offset)
                    
                    # Draw towers (exclude start and goal)
                    if self.towers:
                        goal_cell = self.env.world_to_grid(self.goal)
                        start_cell = self.env.world_to_grid(self.start)
                        
                        # Turning towers (yellow triangles) - exclude start and goal
                        for cell in self.towers.turning_towers:
                            if cell == goal_cell or cell == start_cell:
                                continue
                            world_pos = self.env.cell_center_world(cell)
                            screen_pos = world_to_screen(self.env, world_pos, self.screen_size,
                                                        self.zoom, self.zoom_center, self.pan_offset)
                            # Draw triangle pointing up
                            size = max(3, int(6 * self.zoom))
                            points = [
                                (screen_pos[0], screen_pos[1] - size),
                                (screen_pos[0] - size, screen_pos[1] + size),
                                (screen_pos[0] + size, screen_pos[1] + size),
                            ]
                            pygame.draw.polygon(screen, COLOR_TOWER_TURNING, points)
                        
                        # Straight towers (red circles) - exclude start and goal
                        for cell in self.towers.straight_towers:
                            if cell == goal_cell or cell == start_cell:
                                continue
                            tower_world_pos = self.env.cell_center_world(cell)
                            # Project onto path to ensure it's on the line
                            projected_pos = project_point_to_path(tower_world_pos, 
                                                                 self.result.path_cells, 
                                                                 self.env)
                            screen_pos = world_to_screen(self.env, projected_pos, self.screen_size,
                                                        self.zoom, self.zoom_center, self.pan_offset)
                            # Draw red circle
                            radius = max(2, int(4 * self.zoom))
                            pygame.draw.circle(screen, COLOR_TOWER_STRAIGHT, screen_pos, radius, width=0)
                            pygame.draw.circle(screen, (255, 255, 255), screen_pos, radius, width=1)
        
        # Draw start and goal points (larger circles)
        start_screen = world_to_screen(self.env, self.start, self.screen_size, 
                                      self.zoom, self.zoom_center, self.pan_offset)
        goal_screen = world_to_screen(self.env, self.goal, self.screen_size, 
                                     self.zoom, self.zoom_center, self.pan_offset)
        
        # Start point (red, smaller size)
        start_radius = max(4, int(8 * self.zoom))
        pygame.draw.circle(screen, COLOR_START, start_screen, start_radius, width=0)
        pygame.draw.circle(screen, (255, 255, 255), start_screen, start_radius, width=max(1, int(1 * self.zoom)))
        
        # Goal point (blue, smaller size)
        goal_radius = max(4, int(8 * self.zoom))
        pygame.draw.circle(screen, COLOR_GOAL, goal_screen, goal_radius, width=0)
        pygame.draw.circle(screen, (255, 255, 255), goal_screen, goal_radius, width=max(1, int(1 * self.zoom)))
        
        # Draw obstacles being drawn
        if self.drawing_strip and self.strip_start_pos:
            start_screen = world_to_screen(self.env, self.strip_start_pos, self.screen_size,
                                          self.zoom, self.zoom_center, self.pan_offset)
            mx, my = pygame.mouse.get_pos()
            pygame.draw.line(screen, COLOR_DRAWING, start_screen, (mx, my), 
                           width=max(1, int(3 * self.zoom)))
        
        if self.drawing_area and self.area_center:
            center_screen = world_to_screen(self.env, self.area_center, self.screen_size,
                                           self.zoom, self.zoom_center, self.pan_offset)
            if self.area_radius:
                radius_px = int(self.area_radius * self.screen_size[0] / self.env.width_m * self.zoom)
            else:
                mx, my = pygame.mouse.get_pos()
                radius_px = int(math.hypot(mx - center_screen[0], my - center_screen[1]))
            pygame.draw.circle(screen, COLOR_DRAWING, center_screen, radius_px, 
                             width=max(1, int(2 * self.zoom)))
        
        # Draw Start button
        button_color = COLOR_BUTTON_HOVER if self.start_button_hover else COLOR_BUTTON
        pygame.draw.rect(screen, button_color, self.start_button_rect)
        pygame.draw.rect(screen, (255, 255, 255), self.start_button_rect, width=2)
        font = pygame.font.Font(None, 28)
        text = font.render("Start" if not self.planning else "Planning...", True, (255, 255, 255))
        text_rect = text.get_rect(center=self.start_button_rect.center)
        screen.blit(text, text_rect)
        
        # Draw Random Obstacles button
        button_color = COLOR_BUTTON_HOVER if self.random_obstacles_button_hover else COLOR_BUTTON
        pygame.draw.rect(screen, button_color, self.random_obstacles_button_rect)
        pygame.draw.rect(screen, (255, 255, 255), self.random_obstacles_button_rect, width=2)
        # Use smaller font for button text to fit better
        button_font = pygame.font.Font(None, 22)
        text = button_font.render("Random Obstacles", True, (255, 255, 255))
        text_rect = text.get_rect(center=self.random_obstacles_button_rect.center)
        screen.blit(text, text_rect)
        
        # Draw info panel (guide box)
        font = pygame.font.Font(None, 18)  # Smaller font size
        info_lines = [
            "Interactive Transmission Line Routing",
            "Left Click Start/Goal: Move Point",
            "Left Drag: Pan View",
            "Mouse Wheel: Zoom In/Out",
            "S Key: Add Strip (click two points)",
            "A Key: Add Area (click center + drag)",
            "Random Obstacles: Add obstacles in view",
            "ESC: Exit",
        ]
        
        if self.drawing_strip:
            info_lines.append("Drawing Mode: Strip (click two points)")
        elif self.drawing_area:
            info_lines.append("Drawing Mode: Area (click center + drag)")
        
        with self.lock:
            if self.result:
                if self.result.success:
                    info_lines.append(f"Path Length: {len(self.result.path_cells)} cells")
                    info_lines.append(f"Total Cost: {self.result.total_cost:.1f}")
                    if self.towers:
                        info_lines.append(f"Towers: {len(self.towers.tower_sequence)}")
                else:
                    info_lines.append("Path Planning Failed")
            elif self.planning:
                info_lines.append("Planning in progress...")
        
        # Add zoom level info
        info_lines.append(f"Zoom: {self.zoom:.2f}x")
        
        # Calculate panel size
        max_width = 0
        for line in info_lines:
            text_surface = font.render(line, True, (0, 0, 0))
            max_width = max(max_width, text_surface.get_width())
        
        panel_width = max_width + 20
        panel_height = len(info_lines) * 20 + 10  # Smaller line spacing
        panel_rect = pygame.Rect(10, 10, panel_width, panel_height)
        
        # Draw semi-transparent background panel
        panel_surface = pygame.Surface((panel_width, panel_height), pygame.SRCALPHA)
        panel_surface.fill((255, 255, 255, 200))  # White with transparency
        screen.blit(panel_surface, panel_rect)
        
        # Draw border
        pygame.draw.rect(screen, (0, 0, 0), panel_rect, width=2)
        
        # Draw text in black
        y_offset = 12
        for line in info_lines:
            text = font.render(line, True, (0, 0, 0))  # Black text
            screen.blit(text, (15, y_offset))
            y_offset += 20  # Smaller line spacing


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--config', type=str, default='configs/demo_config.yaml')
    ap.add_argument('--map', type=str, default='configs/demo_map.yaml')
    ap.add_argument('--size', type=int, nargs=2, default=[1200, 800], 
                    metavar=('WIDTH', 'HEIGHT'), help='Window size')
    args = ap.parse_args()
    
    # Load config
    config = load_yaml(args.config)
    map_data = load_yaml(args.map)
    
    # Initialize pygame
    pygame.init()
    screen = pygame.display.set_mode(tuple(args.size))
    pygame.display.set_caption("Interactive Transmission Line Routing")
    clock = pygame.time.Clock()
    
    # Create demo object
    demo = InteractiveDemo(config, map_data, tuple(args.size))
    
    running = True
    last_time = time.time()
    
    while running:
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time
        
        # Update animation
        demo.update_animation(dt)
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_s:
                    demo.drawing_strip = True
                    demo.drawing_area = False
                    demo.strip_start_pos = None
                elif event.key == pygame.K_a:
                    demo.drawing_area = True
                    demo.drawing_strip = False
                    demo.area_center = None
                    demo.area_radius = None
            elif event.type == pygame.MOUSEBUTTONDOWN:
                demo.handle_mouse_down(event.pos, event.button)
            elif event.type == pygame.MOUSEBUTTONUP:
                demo.handle_mouse_up(event.pos, event.button)
            elif event.type == pygame.MOUSEMOTION:
                demo.handle_mouse_motion(event.pos)
            elif event.type == pygame.MOUSEWHEEL:
                # MOUSEWHEEL event doesn't have pos attribute, use mouse.get_pos() instead
                mouse_pos = pygame.mouse.get_pos()
                demo.handle_mouse_wheel(mouse_pos, event.y)
        
        # Draw
        demo.draw(screen)
        pygame.display.flip()
        clock.tick(60)
    
    pygame.quit()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
