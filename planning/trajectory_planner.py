import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
from matplotlib.patches import FancyBboxPatch, Circle, Wedge
from matplotlib.collections import LineCollection
import matplotlib.patheffects as path_effects
import math
from collections import deque

class LibraryTrajectoryPlanner:
    def __init__(self):
        # Robot parameters
        self.approach_distance = 0.2
        self.safety_distance = 0.15
        self.robot_radius = 0.08
        
        # Robot initial state
        self.robot_pos = [1.5, 0.5]
        self.robot_orientation = 0
        
        # Path history
        self.path_history = []
        self.confirmed_paths = []
        
        # Library layout setup
        self.setup_library_layout()
        
    def setup_library_layout(self):
        """Setup library shelves with proper orientation"""
        self.shelves = {}
        shelf_id = 1
        
        # Shelf dimensions
        shelf_length = 0.8
        shelf_width = 0.25
        
        # Central aisles (A-C) - reduced to make room
        aisles = ['A', 'B', 'C']
        for i, aisle in enumerate(aisles):
            x_base = 1.5 + i * 2.0
            
            # 6 shelves per aisle (both sides) - continuous shelving
            for j in range(6):
                y_pos = 2.0 + j * shelf_length
                
                # Left shelf - books face inward (toward R shelf)
                self.shelves[f"{aisle}{j+1}L"] = {
                    'id': shelf_id,
                    'position': [x_base - 0.5, y_pos],
                    'size': [shelf_width, shelf_length],
                    'orientation': 0,
                    'access_side': 'west',  # Access from left
                    'book_side': 'east'     # Books face right (toward R)
                }
                shelf_id += 1
                
                # Right shelf - books face inward (toward L shelf)
                self.shelves[f"{aisle}{j+1}R"] = {
                    'id': shelf_id,
                    'position': [x_base + 0.5, y_pos],
                    'size': [shelf_width, shelf_length],
                    'orientation': math.pi,
                    'access_side': 'east',   # Access from right
                    'book_side': 'west'      # Books face left (toward L)
                }
                shelf_id += 1
        
        # Top shelves (T1-T14) - 2 rows of 7 shelves with increased spacing
        row_spacing = 0.8  # Increased row spacing
        shelf_spacing = 0.9  # Spacing between shelves in a row
        
        # Row 1 (T1-T7)
        for i in range(7):
            x_pos = 0.8 + i * shelf_spacing
            y_pos = 7.2
            self.shelves[f"T{i+1}"] = {
                'id': shelf_id,
                'position': [x_pos, y_pos],
                'size': [shelf_length, shelf_width],
                'orientation': 0,
                'access_side': 'south',
                'book_side': 'north'  # Books face north
            }
            shelf_id += 1
        
        # Row 2 (T8-T14)
        for i in range(7):
            x_pos = 0.8 + i * shelf_spacing
            y_pos = 7.2 + row_spacing
            self.shelves[f"T{i+8}"] = {
                'id': shelf_id,
                'position': [x_pos, y_pos],
                'size': [shelf_length, shelf_width],
                'orientation': 0,
                'access_side': 'north',
                'book_side': 'south'  # Books face south
            }
            shelf_id += 1
            
        # Define walls with adjusted size
        self.walls = [
            # Left wall
            {'start': [0, 0], 'end': [0, 8.5]},
            # Top wall
            {'start': [0, 8.5], 'end': [7.5, 8.5]},
            # Right wall
            {'start': [7.5, 8.5], 'end': [7.5, 0]},
            # Bottom wall (with entrance)
            {'start': [0, 0], 'end': [1, 0]},
            {'start': [2.5, 0], 'end': [7.5, 0]}
        ]
            
        print(f"Total {len(self.shelves)} shelves configured")
        
    def get_approach_position(self, shelf_code):
        """Calculate approach position to always face the book side"""
        if shelf_code not in self.shelves:
            print(f"Error: Shelf '{shelf_code}' not found.")
            return None, None
            
        shelf = self.shelves[shelf_code]
        pos = shelf['position']
        size = shelf['size']
        book_side = shelf['book_side']
        
        # Always approach to face the book side (red line)
        if book_side == 'east':
            approach_pos = [pos[0] + size[0]/2 + self.approach_distance, pos[1]]
            approach_orientation = math.pi  # facing west to see books
        elif book_side == 'west':
            approach_pos = [pos[0] - size[0]/2 - self.approach_distance, pos[1]]
            approach_orientation = 0  # facing east to see books
        elif book_side == 'north':
            approach_pos = [pos[0], pos[1] + size[1]/2 + self.approach_distance]
            approach_orientation = -math.pi/2  # facing south to see books
        elif book_side == 'south':
            approach_pos = [pos[0], pos[1] - size[1]/2 - self.approach_distance]
            approach_orientation = math.pi/2  # facing north to see books
            
        return approach_pos, approach_orientation
        
    def is_safe_position(self, pos):
        """Check if position is safe from shelves and walls"""
        # Check shelf collisions
        for shelf_code, shelf in self.shelves.items():
            shelf_pos = shelf['position']
            shelf_size = shelf['size']
            
            min_x = shelf_pos[0] - shelf_size[0]/2 - self.safety_distance
            max_x = shelf_pos[0] + shelf_size[0]/2 + self.safety_distance
            min_y = shelf_pos[1] - shelf_size[1]/2 - self.safety_distance
            max_y = shelf_pos[1] + shelf_size[1]/2 + self.safety_distance
            
            if min_x <= pos[0] <= max_x and min_y <= pos[1] <= max_y:
                return False
                
        # Check wall collisions
        wall_distance = 0.2
        if pos[0] < wall_distance or pos[0] > 7.5 - wall_distance:
            return False
        if pos[1] < wall_distance or pos[1] > 8.5 - wall_distance:
            return False
            
        return True
        
    def plan_path(self, target_shelf_code, start_pos=None, start_orientation=None):
        """Plan path with rotation first, then movement"""
        if start_pos is None:
            start_pos = self.robot_pos
        if start_orientation is None:
            start_orientation = self.robot_orientation
            
        target_pos, target_orientation = self.get_approach_position(target_shelf_code)
        if target_pos is None:
            return None
            
        print(f"\nPath planning: ({start_pos[0]:.2f}, {start_pos[1]:.2f}) -> "
              f"{target_shelf_code} ({target_pos[0]:.2f}, {target_pos[1]:.2f})")
        
        # Calculate orientation difference
        angle_diff = target_orientation - start_orientation
        # Normalize to [-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        print(f"Current orientation: {start_orientation*180/math.pi:.1f}°, "
              f"Target: {target_orientation*180/math.pi:.1f}°, "
              f"Rotation needed: {angle_diff*180/math.pi:.1f}°")
        
        # A* pathfinding
        grid_resolution = 0.05
        
        # Grid bounds
        min_x, max_x = 0, 7.5
        min_y, max_y = 0, 8.5
        
        # Convert to grid coordinates
        start_grid = (
            int((start_pos[0] - min_x) / grid_resolution),
            int((start_pos[1] - min_y) / grid_resolution)
        )
        goal_grid = (
            int((target_pos[0] - min_x) / grid_resolution),
            int((target_pos[1] - min_y) / grid_resolution)
        )
        
        # A* search
        path_grid = self.astar_search(start_grid, goal_grid, 
                                     min_x, max_x, min_y, max_y, grid_resolution)
        
        if path_grid is None:
            print("No path found.")
            return None
            
        # Convert to real coordinates
        path = []
        for grid_pos in path_grid:
            x = min_x + grid_pos[0] * grid_resolution
            y = min_y + grid_pos[1] * grid_resolution
            path.append([x, y])
            
        # Simplify path
        simplified_path = self.simplify_path(path)
        
        # Create path with rotation first, then movement
        path_with_orientation = []
        
        # Step 1: Add rotation phase (if needed)
        if abs(angle_diff) > 0.01:  # If rotation is needed
            # Add multiple rotation steps for smooth visualization
            rotation_steps = 10
            for i in range(rotation_steps + 1):
                t = i / float(rotation_steps)
                orientation = start_orientation + angle_diff * t
                path_with_orientation.append({
                    'position': start_pos.copy(),  # Stay in same position
                    'orientation': orientation,
                    'phase': 'rotation'
                })
        
        # Step 2: Add movement phase (with constant orientation)
        for i in range(len(simplified_path)):
            pos = simplified_path[i]
            path_with_orientation.append({
                'position': pos,
                'orientation': target_orientation,  # Keep target orientation during movement
                'phase': 'movement'
            })
            
        return {
            'target_shelf': target_shelf_code,
            'target_position': target_pos,
            'target_orientation': target_orientation,
            'path': path_with_orientation,
            'has_rotation': abs(angle_diff) > 0.01
        }
        
    def plan_multiple_paths(self, shelf_codes):
        """Plan paths for multiple shelves in sequence"""
        all_paths = []
        current_pos = self.robot_pos.copy()
        current_orientation = self.robot_orientation
        
        for i, shelf_code in enumerate(shelf_codes):
            print(f"\n{'='*50}")
            print(f"Planning path {i+1}/{len(shelf_codes)} to shelf: {shelf_code}")
            
            path_info = self.plan_path(shelf_code, current_pos, current_orientation)
            
            if path_info:
                all_paths.append(path_info)
                # Update position for next path
                current_pos = path_info['target_position']
                current_orientation = path_info['target_orientation']
            else:
                print(f"Failed to plan path to {shelf_code}")
                
        return all_paths
        
    def astar_search(self, start, goal, min_x, max_x, min_y, max_y, resolution):
        """A* algorithm for path search"""
        from heapq import heappush, heappop
        
        def heuristic(a, b):
            return abs(a[0] - b[0]) + abs(a[1] - b[1])
            
        def is_valid(pos):
            x = min_x + pos[0] * resolution
            y = min_y + pos[1] * resolution
            if x < min_x or x > max_x or y < min_y or y > max_y:
                return False
            return self.is_safe_position([x, y])
            
        open_set = []
        heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: heuristic(start, goal)}
        
        # Manhattan distance only - no diagonal movement
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
        
        while open_set:
            current = heappop(open_set)[1]
            
            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1]
                
            for dx, dy in directions:
                neighbor = (current[0] + dx, current[1] + dy)
                
                if not is_valid(neighbor):
                    continue
                    
                # Manhattan moves only (cost = 1)
                tentative_g_score = g_score[current] + 1
                
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, goal)
                    heappush(open_set, (f_score[neighbor], neighbor))
                    
        return None
        
    def simplify_path(self, path):
        """Simplify path to Manhattan distance style (x,y axis aligned)"""
        if len(path) <= 2:
            return path
            
        simplified = [path[0]]
        
        for i in range(1, len(path)-1):
            prev_dx = path[i][0] - path[i-1][0]
            prev_dy = path[i][1] - path[i-1][1]
            next_dx = path[i+1][0] - path[i][0]
            next_dy = path[i+1][1] - path[i][1]
            
            # Add corner points where direction changes
            if (abs(prev_dx) > 0 and abs(next_dy) > 0) or (abs(prev_dy) > 0 and abs(next_dx) > 0):
                simplified.append(path[i])
                
        simplified.append(path[-1])
        return simplified
        
    def execute_path(self, path_info):
        """Execute path with rotation first then movement"""
        if path_info is None:
            return
            
        # Create detailed trajectory
        trajectory = []
        waypoints = path_info['path']
        
        # Add all waypoints to trajectory
        for wp in waypoints:
            trajectory.append({
                'position': wp['position'].copy(),
                'orientation': wp['orientation'],
                'phase': wp.get('phase', 'movement')
            })
        
        # Store confirmed path
        self.confirmed_paths.append({
            'path_info': path_info,
            'trajectory': trajectory
        })
        
        # Update robot position
        final_waypoint = path_info['path'][-1]
        self.robot_pos = final_waypoint['position']
        self.robot_orientation = final_waypoint['orientation']
        
        # Record path
        self.path_history.append(path_info)
        
        print(f"Arrived at: {path_info['target_shelf']} "
              f"Position: ({self.robot_pos[0]:.2f}, {self.robot_pos[1]:.2f}) "
              f"Orientation: {self.robot_orientation*180/math.pi:.0f} degrees")
        
    def execute_multiple_paths(self, all_paths):
        """Execute multiple paths in sequence"""
        for i, path_info in enumerate(all_paths):
            print(f"\nExecuting path {i+1}/{len(all_paths)} to {path_info['target_shelf']}")
            self.execute_path(path_info)
        
    def visualize_library(self, current_paths=None, show_path_history=True):
        """Enhanced visualization with support for multiple paths"""
        fig, ax = plt.subplots(figsize=(14, 12))
        
        # Modern dark theme
        fig.patch.set_facecolor('#1a1a1a')
        ax.set_facecolor('#2d2d2d')
        
        # Library floor with grid
        ax.set_xlim(-0.5, 8)
        ax.set_ylim(-0.5, 9)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.1, linestyle='-', color='#404040', linewidth=0.5)
        
        # Stylized labels
        ax.set_xlabel('X (meters)', fontsize=12, color='#e0e0e0', fontweight='light')
        ax.set_ylabel('Y (meters)', fontsize=12, color='#e0e0e0', fontweight='light')
        ax.set_title('Advanced Library Autonomous Navigation System\nRotate-then-Move Strategy', 
                    fontsize=20, color='#ffffff', fontweight='bold', pad=20)
        
        # Set tick colors
        ax.tick_params(colors='#a0a0a0', which='both')
        for spine in ax.spines.values():
            spine.set_edgecolor('#404040')
            spine.set_linewidth(1)
        
        # Draw walls with gradient effect
        for wall in self.walls:
            ax.plot([wall['start'][0], wall['end'][0]], 
                   [wall['start'][1], wall['end'][1]], 
                   color='#606060', linewidth=4, solid_capstyle='round')
            # Add shadow effect
            ax.plot([wall['start'][0], wall['end'][0]], 
                   [wall['start'][1], wall['end'][1]], 
                   color='#000000', linewidth=6, alpha=0.3, 
                   transform=ax.transData, zorder=-1)
        
        # Draw entrance with glow
        entrance_x = [1, 2.5]
        entrance_y = [0, 0]
        ax.plot(entrance_x, entrance_y, color='#00ff88', linewidth=4, 
               linestyle='--', alpha=0.8, label='Entrance')
        
        # Draw shelves with modern styling
        for shelf_code, shelf in self.shelves.items():
            pos = shelf['position']
            size = shelf['size']
            
            # Shelf shadow
            shadow = FancyBboxPatch(
                (pos[0] - size[0]/2 + 0.02, pos[1] - size[1]/2 - 0.02),
                size[0], size[1],
                boxstyle="round,pad=0.02",
                facecolor='#000000',
                alpha=0.3,
                zorder=1
            )
            ax.add_patch(shadow)
            
            # Shelf body with gradient
            shelf_rect = FancyBboxPatch(
                (pos[0] - size[0]/2, pos[1] - size[1]/2),
                size[0], size[1],
                boxstyle="round,pad=0.01",
                facecolor='#4a4a4a',
                edgecolor='#808080',
                linewidth=1.5,
                zorder=2
            )
            ax.add_patch(shelf_rect)
            
            # Book side indicator with glow
            book_side = shelf['book_side']
            book_color = '#ff4444'
            glow_color = '#ff6666'
            
            if book_side == 'east':
                x_coords = [pos[0] + size[0]/2, pos[0] + size[0]/2]
                y_coords = [pos[1] - size[1]/2, pos[1] + size[1]/2]
            elif book_side == 'west':
                x_coords = [pos[0] - size[0]/2, pos[0] - size[0]/2]
                y_coords = [pos[1] - size[1]/2, pos[1] + size[1]/2]
            elif book_side == 'north':
                x_coords = [pos[0] - size[0]/2, pos[0] + size[0]/2]
                y_coords = [pos[1] + size[1]/2, pos[1] + size[1]/2]
            elif book_side == 'south':
                x_coords = [pos[0] - size[0]/2, pos[0] + size[0]/2]
                y_coords = [pos[1] - size[1]/2, pos[1] - size[1]/2]
            
            # Book side with glow effect
            line = ax.plot(x_coords, y_coords, color=book_color, linewidth=4, 
                          solid_capstyle='round', zorder=4)[0]
            line.set_path_effects([path_effects.withStroke(linewidth=8, 
                                                          foreground=glow_color, 
                                                          alpha=0.5)])
            
            # Shelf label with modern styling
            label_bbox = dict(boxstyle="round,pad=0.3", 
                            facecolor='#ffffff', 
                            edgecolor='none',
                            alpha=0.9)
            ax.text(pos[0], pos[1], shelf_code, 
                   ha='center', va='center', fontsize=9, 
                   fontweight='bold', color='#2d2d2d',
                   bbox=label_bbox, zorder=5)
        
        # Draw path history with fading effect
        if show_path_history:
            for i, confirmed in enumerate(self.confirmed_paths):
                alpha = 0.3 + 0.4 * (i + 1) / len(self.confirmed_paths) if self.confirmed_paths else 0.7
                trajectory = confirmed['trajectory']
                
                if len(trajectory) > 1:
                    # Separate rotation and movement phases
                    rotation_points = [t for t in trajectory if t.get('phase') == 'rotation']
                    movement_points = [t for t in trajectory if t.get('phase') != 'rotation']
                    
                    # Draw rotation phase if exists
                    if rotation_points:
                        center = rotation_points[0]['position']
                        # Draw rotation indicator
                        rotation_circle = Circle(center, 0.15, 
                                               facecolor='none', 
                                               edgecolor='#ffaa00', 
                                               linewidth=2, 
                                               linestyle='--',
                                               alpha=alpha * 0.7, 
                                               zorder=7)
                        ax.add_patch(rotation_circle)
                    
                    # Draw movement phase
                    if movement_points:
                        positions = np.array([t['position'] for t in movement_points])
                        
                        # Create gradient line
                        points = positions.reshape(-1, 1, 2)
                        segments = np.concatenate([points[:-1], points[1:]], axis=1)
                        
                        lc = LineCollection(segments, linewidths=2,
                                          colors=[(0.5, 0.8, 1.0, alpha * (0.5 + 0.5 * j/len(segments))) 
                                                 for j in range(len(segments))])
                        ax.add_collection(lc)
        
        # Draw current paths (multiple paths)
        if current_paths:
            colors = ['#00ffff', '#ff00ff', '#ffff00', '#00ff00', '#ff8800']  # Different colors for each path
            
            for path_idx, path_info in enumerate(current_paths):
                path_color = colors[path_idx % len(colors)]
                waypoints = path_info['path']
                
                # Separate rotation and movement phases
                rotation_points = [wp for wp in waypoints if wp.get('phase') == 'rotation']
                movement_points = [wp for wp in waypoints if wp.get('phase') != 'rotation']
                
                # Draw rotation phase
                if rotation_points:
                    center = rotation_points[0]['position']
                    start_angle = rotation_points[0]['orientation'] * 180 / math.pi
                    end_angle = rotation_points[-1]['orientation'] * 180 / math.pi
                    
                    # Draw rotation arc
                    angle_diff = end_angle - start_angle
                    if angle_diff != 0:
                        wedge = Wedge(center, 0.25, start_angle, end_angle, 
                                    facecolor='none', 
                                    edgecolor=path_color, 
                                    linewidth=3, 
                                    alpha=0.7, 
                                    zorder=10)
                        ax.add_patch(wedge)
                        
                        # Add rotation label
                        ax.text(center[0], center[1] + 0.35, 
                               f'↻ {abs(angle_diff):.0f}°', 
                               ha='center', fontsize=8, 
                               color=path_color, 
                               fontweight='bold',
                               bbox=dict(boxstyle="round,pad=0.2", 
                                       facecolor='#2a2a2a', 
                                       edgecolor=path_color,
                                       alpha=0.8))
                
                # Draw movement phase
                if movement_points:
                    positions = np.array([wp['position'] for wp in movement_points])
                    
                    if len(positions) > 1:
                        # Main path
                        for i in range(len(positions)-1):
                            ax.plot([positions[i][0], positions[i+1][0]], 
                                   [positions[i][1], positions[i+1][1]], 
                                   color=path_color, linewidth=4, alpha=0.8, 
                                   solid_capstyle='round', zorder=10)
                        
                        # Path glow
                        for i in range(len(positions)-1):
                            line = ax.plot([positions[i][0], positions[i+1][0]], 
                                          [positions[i][1], positions[i+1][1]], 
                                          color=path_color, linewidth=3, alpha=0.4, 
                                          zorder=9)[0]
                            line.set_path_effects([path_effects.withStroke(linewidth=10, 
                                                                          foreground=path_color, 
                                                                          alpha=0.2)])
                
                # Draw target marker with path number
                target_pos = waypoints[-1]['position']
                target_marker = Circle(target_pos, 0.1, 
                                     facecolor=path_color, 
                                     edgecolor='#ffffff', 
                                     linewidth=2, 
                                     alpha=0.8,
                                     zorder=15)
                ax.add_patch(target_marker)
                
                # Add path number
                ax.text(target_pos[0], target_pos[1], 
                       str(path_idx + 1), 
                       ha='center', va='center',
                       fontsize=10, color='#ffffff', 
                       fontweight='bold',
                       zorder=16)
                
                # Target shelf label
                ax.text(target_pos[0], target_pos[1] - 0.2, 
                       path_info['target_shelf'], 
                       ha='center', fontsize=8, 
                       color=path_color, 
                       fontweight='bold')
        
        # Current robot with modern design
        # Robot shadow
        shadow = Circle(self.robot_pos, self.robot_radius + 0.02, 
                       facecolor='#000000', alpha=0.3, zorder=14)
        ax.add_patch(shadow)
        
        # Robot body
        robot_circle = Circle(self.robot_pos, self.robot_radius, 
                            facecolor='#0080ff', edgecolor='#ffffff', 
                            linewidth=2, zorder=16)
        ax.add_patch(robot_circle)
        
        # Robot center
        center = Circle(self.robot_pos, 0.02, 
                       facecolor='#ffffff', zorder=17)
        ax.add_patch(center)
        
        # Robot direction indicator
        arrow_length = 0.2
        dx = arrow_length * math.cos(self.robot_orientation)
        dy = arrow_length * math.sin(self.robot_orientation)
        ax.arrow(self.robot_pos[0], self.robot_pos[1], dx, dy,
                head_width=0.08, head_length=0.05, 
                fc='#ffff00', ec='#ffffff', linewidth=1.5, zorder=18)
        
        # Modern legend
        legend_elements = [
            plt.Line2D([0], [0], color='#ff4444', linewidth=4, label='Book Side'),
            plt.Line2D([0], [0], color='#00ffff', linewidth=4, label='Path 1'),
            plt.Line2D([0], [0], color='#ff00ff', linewidth=4, label='Path 2'),
            plt.Line2D([0], [0], color='#ffaa00', linewidth=3, 
                      linestyle='--', label='Rotation Phase'),
            plt.Line2D([0], [0], color='#00ff88', linewidth=3, 
                      linestyle='--', label='Entrance')
        ]
        legend = ax.legend(handles=legend_elements, loc='upper right', 
                          fontsize=10, frameon=True, fancybox=True, 
                          shadow=True, facecolor='#3a3a3a', edgecolor='#606060')
        for text in legend.get_texts():
            text.set_color('#e0e0e0')
        
        # Status panel with modern styling
        info_text = f"Robot Status\n"
        info_text += f"━━━━━━━━━━━━━━━━━━━━\n"
        info_text += f"Position: ({self.robot_pos[0]:.2f}, {self.robot_pos[1]:.2f})\n"
        info_text += f"Orientation: {self.robot_orientation*180/math.pi:.0f}°\n"
        info_text += f"Mode: Rotate-then-Move\n"
        info_text += f"Paths Completed: {len(self.confirmed_paths)}"
        
        info_box = FancyBboxPatch((0.1, 8.7), 2.5, 0.7,
                                 boxstyle="round,pad=0.1",
                                 facecolor='#1a1a1a',
                                 edgecolor='#606060',
                                 linewidth=1.5,
                                 alpha=0.9)
        ax.add_patch(info_box)
        ax.text(0.2, 8.8, info_text, transform=ax.transData, 
               fontsize=10, color='#e0e0e0', fontfamily='monospace',
               verticalalignment='bottom')
        
        plt.tight_layout()
        plt.show()
        
    def get_available_shelves(self):
        """Get list of available shelf codes organized by area"""
        shelves_by_area = {
            'Aisle A': [],
            'Aisle B': [],
            'Aisle C': [],
            'Top Row 1 (T1-T7)': [],
            'Top Row 2 (T8-T14)': []
        }
        
        for code in sorted(self.shelves.keys()):
            if code.startswith('A'):
                shelves_by_area['Aisle A'].append(code)
            elif code.startswith('B'):
                shelves_by_area['Aisle B'].append(code)
            elif code.startswith('C'):
                shelves_by_area['Aisle C'].append(code)
            elif code.startswith('T'):
                shelf_num = int(code[1:])
                if shelf_num <= 7:
                    shelves_by_area['Top Row 1 (T1-T7)'].append(code)
                else:
                    shelves_by_area['Top Row 2 (T8-T14)'].append(code)
                
        return shelves_by_area

def interactive_library_navigation():
    """Interactive library navigation with multiple targets"""
    planner = LibraryTrajectoryPlanner()
    
    print("\n" + "═"*70)
    print(" ADVANCED LIBRARY AUTONOMOUS NAVIGATION SYSTEM ".center(70))
    print(" Rotate-then-Move Strategy with Multi-Path Planning ".center(70))
    print("═"*70)
    print(f"Total {len(planner.shelves)} shelves configured")
    print("─"*70)
    
    # Show initial library
    planner.visualize_library()
    
    while True:
        print(f"\n📍 Current Position: ({planner.robot_pos[0]:.1f}, {planner.robot_pos[1]:.1f})")
        print(f"🧭 Current Orientation: {planner.robot_orientation*180/math.pi:.0f}°")
        print("\n📚 Available Shelves:")
        
        shelves_by_area = planner.get_available_shelves()
        for area, codes in shelves_by_area.items():
            if codes:
                print(f"  {area}: {', '.join(codes)}")
        
        print("\n💡 Options:")
        print("  • Enter single shelf code (e.g., A1L)")
        print("  • Enter multiple shelf codes separated by space (e.g., A1L B3R T5)")
        print("  • Type 'show' to display library map")
        print("  • Type 'q' to quit")
        
        user_input = input("\n🎯 Enter command: ").strip()
        
        if user_input.lower() == 'q':
            print("👋 Exiting system. Goodbye!")
            break
        elif user_input.lower() == 'show':
            planner.visualize_library()
            continue
            
        # Parse shelf codes (multiple targets)
        shelf_codes = user_input.upper().split()
        
        # Validate shelf codes
        valid_codes = []
        for code in shelf_codes:
            if code in planner.shelves:
                valid_codes.append(code)
            else:
                print(f"⚠️  Invalid shelf code: '{code}' (skipped)")
        
        if not valid_codes:
            print("❌ No valid shelf codes entered.")
            continue
            
        # Plan paths for all targets
        print(f"\n🗺️  Planning paths for {len(valid_codes)} target(s): {', '.join(valid_codes)}")
        all_paths = planner.plan_multiple_paths(valid_codes)
        
        if all_paths:
            # Visualize all paths at once
            planner.visualize_library(current_paths=all_paths)
            
            # Execute paths
            execute = input(f"\n🚀 Execute all {len(all_paths)} paths? (y/n): ").strip().lower()
            if execute == 'y':
                planner.execute_multiple_paths(all_paths)
                print("✅ All paths executed successfully!")
                # Show updated map
                planner.visualize_library()
            else:
                print("❌ Path execution cancelled.")
        else:
            print("⚠️  Failed to plan any paths.")

if __name__ == "__main__":
    interactive_library_navigation()