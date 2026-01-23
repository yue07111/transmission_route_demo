#!/usr/bin/env python
"""Web-based Interactive Transmission Line Routing Demo

Features:
- Web interface for interactive path planning
- Real-time visualization
- Drag to select start and goal points
- Add obstacles interactively

Usage:
  python web_demo.py --config configs/demo_config.yaml --map configs/demo_map.yaml
  Then open http://localhost:5000 in your browser
"""

from __future__ import annotations

import argparse
import json
import math
import threading
import time
from typing import Dict, List, Optional, Tuple

import numpy as np
from flask import Flask, render_template, request, jsonify, Response, stream_with_context
import yaml
import queue
import json as json_lib

from src.builder import build_from_spec
from src.astar import AStarPlanner
from src.postprocess import place_towers
from src.environment import Environment, Cell, Point


app = Flask(__name__)

# Global state
config_data = None
map_data = None
env = None
constraint_manager = None
planner = None
planning_result = None
planning_lock = threading.Lock()

# For real-time search visualization
search_progress_queue = None


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


def load_yaml(path: str):
    import os
    if not os.path.isfile(path):
        raise FileNotFoundError(f"YAML file not found: {path}")
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def init_environment(config: Dict, map_spec: Dict):
    """Initialize environment and planner"""
    global env, constraint_manager, planner, config_data, map_data
    env, constraint_manager = build_from_spec(config, map_spec)
    planner = AStarPlanner(config.get('planner', {}))


def extend_map_to_view(view_min: List[float], view_max: List[float], start: Point = None, goal: Point = None):
    """Extend map boundaries to include view bounds and optionally start/goal.
    Returns (adjusted_start, adjusted_goal) if map was extended, None otherwise."""
    global env, constraint_manager, map_data, config_data
    
    # Calculate required bounds from view
    min_x = min(view_min[0], view_max[0])
    max_x = max(view_min[0], view_max[0])
    min_y = min(view_min[1], view_max[1])
    max_y = max(view_min[1], view_max[1])
    
    # Store original start/goal coordinates
    start_x = start[0] if start else None
    start_y = start[1] if start else None
    goal_x = goal[0] if goal else None
    goal_y = goal[1] if goal else None
    
    # Store original start/goal coordinates
    start_x = start[0] if start else None
    start_y = start[1] if start else None
    goal_x = goal[0] if goal else None
    goal_y = goal[1] if goal else None
    
    # Include start/goal if provided
    if start:
        min_x = min(min_x, start[0])
        max_x = max(max_x, start[0])
        min_y = min(min_y, start[1])
        max_y = max(max_y, start[1])
    if goal:
        min_x = min(min_x, goal[0])
        max_x = max(max_x, goal[0])
        min_y = min(min_y, goal[1])
        max_y = max(max_y, goal[1])
    
    # Add padding
    padding = 500.0
    min_x -= padding
    max_x += padding
    min_y -= padding
    max_y += padding
    
    # Check if extension is needed
    need_extend = False
    new_width = env.width_m
    new_height = env.height_m
    offset_x = 0.0
    offset_y = 0.0
    
    if min_x < 0:
        offset_x = -min_x
        new_width = max_x + offset_x
        need_extend = True
    elif max_x > env.width_m:
        new_width = max_x
        need_extend = True
    
    if min_y < 0:
        offset_y = -min_y
        new_height = max_y + offset_y
        need_extend = True
    elif max_y > env.height_m:
        new_height = max_y
        need_extend = True
    
    if need_extend:
        # Update map_data with new dimensions
        map_data['env']['width_m'] = new_width
        map_data['env']['height_m'] = new_height
        
        # Adjust all entity coordinates if we had to shift (only if offset > 0)
        if offset_x > 0 or offset_y > 0:
            # Shift all obstacles and entities
            for area in map_data.get('areas', []):
                area['center'][0] += offset_x
                area['center'][1] += offset_y
            
            for strip in map_data.get('strips', []):
                strip['center'][0] += offset_x
                strip['center'][1] += offset_y
            
            for point in map_data.get('sensitive_points', []):
                point['center'][0] += offset_x
                point['center'][1] += offset_y
            
            for road in map_data.get('roads', []):
                for pt in road.get('points', []):
                    pt[0] += offset_x
                    pt[1] += offset_y
            
            if 'corridor' in map_data:
                if 'primary' in map_data['corridor']:
                    for pt in map_data['corridor']['primary'].get('points', []):
                        pt[0] += offset_x
                        pt[1] += offset_y
                for other in map_data['corridor'].get('others', []):
                    for pt in other.get('points', []):
                        pt[0] += offset_x
                        pt[1] += offset_y
        
        # Rebuild environment with extended bounds
        env, constraint_manager = build_from_spec(config_data, map_data)
        
        # Adjust start/goal coordinates if we shifted
        if offset_x > 0 or offset_y > 0:
            return (start_x + offset_x, start_y + offset_y), (goal_x + offset_x, goal_y + offset_y)
        return start, goal
    
    return None  # No extension needed


@app.route('/')
def index():
    """Main page"""
    return render_template('index.html')


@app.route('/api/init', methods=['GET'])
def api_init():
    """Get initial map data"""
    global map_data, env
    
    # Convert map data to JSON-serializable format
    response = {
        'width_m': env.width_m,
        'height_m': env.height_m,
        'resolution_m': env.resolution_m,
        'start': map_data['start'],
        'goal': map_data['goal'],
        'strips': map_data.get('strips', []),
        'areas': map_data.get('areas', []),
        'sensitive_points': map_data.get('sensitive_points', []),
        'corridor': map_data.get('corridor', {}),
        'roads': map_data.get('roads', []),
    }
    
    return jsonify(response)


@app.route('/api/plan', methods=['POST'])
def api_plan():
    """Plan path from start to goal with real-time progress streaming"""
    global env, constraint_manager, planner, planning_result, search_progress_queue
    
    data = request.json
    start = tuple(data['start'])
    goal = tuple(data['goal'])
    
    # Convert to grid coordinates WITHOUT clamping (for infinite map)
    # This allows coordinates outside the original map bounds
    start_cell = env.world_to_grid(start, clamp=False)
    goal_cell = env.world_to_grid(goal, clamp=False)
    
    # Get the actual world coordinates (should match input now)
    start_actual = env.cell_center_world(start_cell)
    goal_actual = env.cell_center_world(goal_cell)
    
    # Check if start/goal are in obstacles (only if within original map bounds)
    # For infinite map: out-of-bounds cells are not blocked, so we allow them
    if env.in_bounds(start_cell) and env.is_blocked(start_cell):
        return jsonify({'success': False, 'error': 'Start point is in obstacle'}), 400
    if env.in_bounds(goal_cell) and env.is_blocked(goal_cell):
        return jsonify({'success': False, 'error': 'Goal point is in obstacle'}), 400
    
    # Create queue for progress updates
    search_progress_queue = queue.Queue()
    
    def progress_callback(visited_nodes):
        """Callback to send progress updates"""
        try:
            # Convert cells to world coordinates
            visited_world = []
            for cell in visited_nodes:
                world_pos = env.cell_center_world(cell)
                visited_world.append([float(world_pos[0]), float(world_pos[1])])
            search_progress_queue.put({'type': 'progress', 'visited': visited_world})
        except:
            pass  # Ignore errors in callback
    
    def plan_in_thread():
        """Run planning in separate thread"""
        try:
            result = planner.plan(env, constraint_manager, start_cell, goal_cell,
                                progress_callback=progress_callback)
            
            if result.success:
                towers = place_towers(env, result.path_cells, config_data.get('postprocess', {}))
                
                # Convert to JSON-serializable format
                path_world = []
                for cell in result.path_cells:
                    world_pos = env.cell_center_world(cell)
                    path_world.append([float(world_pos[0]), float(world_pos[1])])
                
                turning_towers_world = []
                for cell in towers.turning_towers:
                    world_pos = env.cell_center_world(cell)
                    turning_towers_world.append([float(world_pos[0]), float(world_pos[1])])
                
                straight_towers_world = []
                for cell in towers.straight_towers:
                    tower_world_pos = env.cell_center_world(cell)
                    # Project straight tower onto the path line
                    projected_pos = project_point_to_path(tower_world_pos, result.path_cells, env)
                    straight_towers_world.append([float(projected_pos[0]), float(projected_pos[1])])
                
                final_result = {
                    'type': 'complete',
                    'success': True,
                    'path': path_world,
                    'turning_towers': turning_towers_world,
                    'straight_towers': straight_towers_world,
                    'total_cost': float(result.total_cost),
                    'path_length': len(result.path_cells),
                    'start_actual': [float(start_actual[0]), float(start_actual[1])],
                    'goal_actual': [float(goal_actual[0]), float(goal_actual[1])],
                }
                search_progress_queue.put(final_result)
                
                with planning_lock:
                    planning_result = result
            else:
                search_progress_queue.put({'type': 'complete', 'success': False, 'error': 'Path planning failed'})
        except Exception as e:
            search_progress_queue.put({'type': 'complete', 'success': False, 'error': str(e)})
    
    # Start planning in background thread
    plan_thread = threading.Thread(target=plan_in_thread, daemon=True)
    plan_thread.start()
    
    # Stream results using Server-Sent Events
    def generate():
        while True:
            try:
                item = search_progress_queue.get(timeout=1.0)
                yield f"data: {json_lib.dumps(item)}\n\n"
                if item.get('type') == 'complete':
                    break
            except queue.Empty:
                # Send keepalive
                yield f"data: {json_lib.dumps({'type': 'keepalive'})}\n\n"
                continue
    
    return Response(stream_with_context(generate()), mimetype='text/event-stream')


@app.route('/api/add_strip', methods=['POST'])
def api_add_strip():
    """Add a strip obstacle"""
    global map_data, env, constraint_manager
    
    data = request.json
    strip = {
        'id': f"strip_interactive_{len(map_data.get('strips', [])) + 1}",
        'center': data['center'],
        'length_m': data['length_m'],
        'width_m': data['width_m'],
        'angle_deg': data['angle_deg'],
        'is_crossable': True,
        'min_cross_angle_deg': 30.0,
        'cross_cost': 20.0,
        'note': 'Interactive'
    }
    
    if 'strips' not in map_data:
        map_data['strips'] = []
    map_data['strips'].append(strip)
    
    # Rebuild environment
    env, constraint_manager = build_from_spec(config_data, map_data)
    
    return jsonify({'success': True})


@app.route('/api/add_area', methods=['POST'])
def api_add_area():
    """Add an area obstacle"""
    global map_data, env, constraint_manager
    
    data = request.json
    area = {
        'id': f"area_interactive_{len(map_data.get('areas', [])) + 1}",
        'center': data['center'],
        'radius_m': data['radius_m'],
        'is_enterable': False,
        'is_crossable': False,
        'damping': 0.0,
        'entry_penalty': 0.0,
        'cross_cost': 0.0,
        'note': 'Interactive'
    }
    
    if 'areas' not in map_data:
        map_data['areas'] = []
    map_data['areas'].append(area)
    
    # Rebuild environment
    env, constraint_manager = build_from_spec(config_data, map_data)
    
    return jsonify({'success': True})


@app.route('/api/random_obstacles', methods=['POST'])
def api_random_obstacles():
    """Generate random obstacles in view"""
    global map_data, env, constraint_manager
    import random
    
    data = request.json
    view_bounds = data.get('view_bounds', {})
    num_strips = data.get('num_strips', 5)
    num_areas = data.get('num_areas', 5)
    num_sensitive = data.get('num_sensitive', 3)
    num_corridors = data.get('num_corridors', 2)
    num_roads = data.get('num_roads', 2)
    
    # Remove all existing obstacles
    map_data['strips'] = []
    map_data['areas'] = []
    map_data['sensitive_points'] = []
    map_data['corridor'] = {}
    map_data['roads'] = []
    
    # Rebuild environment
    env, constraint_manager = build_from_spec(config_data, map_data)
    
    # Generate random obstacles in view
    view_min = view_bounds.get('min', [0, 0])
    view_max = view_bounds.get('max', [env.width_m, env.height_m])
    
    view_width = view_max[0] - view_min[0]
    view_height = view_max[1] - view_min[1]
    
    # Add random strips
    for _ in range(num_strips):
        center_x = random.uniform(view_min[0], view_max[0])
        center_y = random.uniform(view_min[1], view_max[1])
        length_m = random.uniform(800, min(3000, view_width * 0.6))
        width_m = random.uniform(100, 140)
        angle_deg = random.uniform(0, 360)
        
        strip = {
            'id': f"strip_random_{len(map_data.get('strips', [])) + 1}",
            'center': [center_x, center_y],
            'length_m': length_m,
            'width_m': width_m,
            'angle_deg': angle_deg,
            'is_crossable': True,
            'min_cross_angle_deg': 30.0,
            'cross_cost': 20.0,
            'note': 'Random'
        }
        if 'strips' not in map_data:
            map_data['strips'] = []
        map_data['strips'].append(strip)
    
    # Add random areas
    for _ in range(num_areas):
        center_x = random.uniform(view_min[0], view_max[0])
        center_y = random.uniform(view_min[1], view_max[1])
        max_radius = min(400, max(250, min(view_width, view_height) * 0.15))
        radius_m = random.uniform(150, max_radius)
        
        area = {
            'id': f"area_random_{len(map_data.get('areas', [])) + 1}",
            'center': [center_x, center_y],
            'radius_m': radius_m,
            'is_enterable': False,
            'is_crossable': False,
            'damping': 0.0,
            'entry_penalty': 0.0,
            'cross_cost': 0.0,
            'note': 'Random'
        }
        if 'areas' not in map_data:
            map_data['areas'] = []
        map_data['areas'].append(area)
    
    # Add random sensitive points
    for i in range(num_sensitive):
        center_x = random.uniform(view_min[0], view_max[0])
        center_y = random.uniform(view_min[1], view_max[1])
        radius_hard = random.uniform(150, 250)
        radius_soft = random.uniform(400, 600)
        soft_penalty = random.uniform(8, 15)
        
        sensitive = {
            'id': f'sensitive_random_{i+1}',
            'center': [center_x, center_y],
            'radius_hard_m': radius_hard,
            'radius_soft_m': radius_soft,
            'soft_penalty': soft_penalty,
            'note': 'Random'
        }
        if 'sensitive_points' not in map_data:
            map_data['sensitive_points'] = []
        map_data['sensitive_points'].append(sensitive)
    
    # Add random corridors (lines crossing the view)
    corridor_others = []
    for i in range(num_corridors):
        # Generate a line that crosses the view
        if random.random() < 0.5:  # Horizontal line
            y = random.uniform(view_min[1], view_max[1])
            num_points = random.randint(3, 5)
            points = []
            for j in range(num_points):
                x = view_min[0] + (view_max[0] - view_min[0]) * j / (num_points - 1) if num_points > 1 else view_min[0]
                y_offset = random.uniform(-50, 50)
                points.append([x, y + y_offset])
        else:  # Vertical line
            x = random.uniform(view_min[0], view_max[0])
            num_points = random.randint(3, 5)
            points = []
            for j in range(num_points):
                y = view_min[1] + (view_max[1] - view_min[1]) * j / (num_points - 1) if num_points > 1 else view_min[1]
                x_offset = random.uniform(-50, 50)
                points.append([x + x_offset, y])
        
        corridor_others.append({
            'id': f'corridor_random_{i+1}',
            'points': points
        })
    
    # Set primary corridor (first one) and others
    if corridor_others:
        map_data['corridor'] = {
            'primary': corridor_others[0],
            'others': corridor_others[1:] if len(corridor_others) > 1 else []
        }
    else:
        map_data['corridor'] = {}
    
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
        if 'roads' not in map_data:
            map_data['roads'] = []
        map_data['roads'].append(road)
    
    # Rebuild environment
    env, constraint_manager = build_from_spec(config_data, map_data)
    
    return jsonify({
        'success': True,
        'strips': map_data.get('strips', []),
        'areas': map_data.get('areas', []),
        'sensitive_points': map_data.get('sensitive_points', []),
        'corridor': map_data.get('corridor', {}),
        'roads': map_data.get('roads', [])
    })


@app.route('/api/clear_obstacles', methods=['POST'])
def api_clear_obstacles():
    """Clear all obstacles"""
    global map_data, env, constraint_manager
    
    # Remove all obstacles
    map_data['strips'] = []
    map_data['areas'] = []
    map_data['sensitive_points'] = []
    map_data['corridor'] = {}
    map_data['roads'] = []
    
    # Rebuild environment
    env, constraint_manager = build_from_spec(config_data, map_data)
    
    return jsonify({'success': True})


def main() -> int:
    global config_data, map_data
    
    ap = argparse.ArgumentParser()
    ap.add_argument('--config', type=str, default='configs/demo_config.yaml')
    ap.add_argument('--map', type=str, default='configs/demo_map.yaml')
    ap.add_argument('--host', type=str, default='0.0.0.0')
    ap.add_argument('--port', type=int, default=5000)
    args = ap.parse_args()
    
    # Load config with error handling
    try:
        config_data = load_yaml(args.config)
    except FileNotFoundError as e:
        print(f"Error: {e}")
        print(f"Please provide a valid config file path, e.g., configs/demo_config.yaml")
        return 1
    except Exception as e:
        print(f"Error loading config file: {e}")
        return 1
    
    try:
        map_data = load_yaml(args.map)
    except FileNotFoundError as e:
        print(f"Error: {e}")
        print(f"Please provide a valid map file path, e.g., configs/demo_map.yaml")
        return 1
    except Exception as e:
        print(f"Error loading map file: {e}")
        return 1
    
    # Initialize environment
    init_environment(config_data, map_data)
    
    print(f"Starting web server at http://{args.host}:{args.port}")
    print("Open this URL in your browser to use the interactive demo")
    
    try:
        app.run(host=args.host, port=args.port, debug=True)
    except OSError as e:
        if "Address already in use" in str(e) or "Port" in str(e):
            print(f"\n错误: 端口 {args.port} 已被占用")
            print(f"请使用 --port 参数指定其他端口，例如：")
            print(f"  python web_demo.py --config {args.config} --map {args.map} --port 5001")
            return 1
        else:
            raise
    return 0


if __name__ == '__main__':
    import sys
    sys.exit(main())

