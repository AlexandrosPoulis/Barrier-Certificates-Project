import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle, Circle
import math

class MovingCar:
    def __init__(self, start_point, end_point, obstacle_positions, barrier_distance, speed=0.02):
        self.start_point = np.array(start_point, dtype=float)
        self.end_point = np.array(end_point, dtype=float)
        self.obstacle_positions = [np.array(obs, dtype=float) for obs in obstacle_positions]
        self.barrier_distance = barrier_distance
        self.speed = speed
        
        # Initialize current_position
        self.current_position = self.start_point.copy()
        
        # Calculate path that properly avoids obstacles
        self.path = self.calculate_safe_path()
        self.current_path_index = 0
        self.progress = 0.0
        self.has_reached_end = False
        self.visited_waypoints = []
        
        # Setup the plot
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        self.setup_plot()
        
        # Car dimensions
        self.car_length = 0.8
        self.car_width = 0.4
        
        # Log initial information
        print(f"\n=== PATH PLANNING INFORMATION ===")
        print(f"Start: {self.start_point}")
        print(f"End: {self.end_point}")
        print(f"Barrier distance: {self.barrier_distance}")
        print(f"Obstacles: {self.obstacle_positions}")
        print(f"Planned path: {self.path}")
        print(f"Total waypoints: {len(self.path)}")
        print("================================\n")
        
    def calculate_safe_path(self):
        """Calculate a safe path that avoids all obstacles with the given barrier distance"""
        if not self.obstacle_positions:
            return [self.start_point, self.end_point]
        
        path = [self.start_point]
        current_pos = self.start_point
        
        # Create waypoints that go around obstacles
        for i, obstacle in enumerate(self.obstacle_positions):
            print(f"Planning around obstacle {i+1} at {obstacle}")
            
            # Calculate vector from current position to obstacle
            to_obstacle = obstacle - current_pos
            obstacle_dist = np.linalg.norm(to_obstacle)
            
            # Calculate the main direction (start to end)
            main_direction = self.end_point - self.start_point
            if np.linalg.norm(main_direction) > 0:
                main_direction = main_direction / np.linalg.norm(main_direction)
            
            # Calculate perpendicular direction (choose the one that's more towards the end)
            perp = np.array([-main_direction[1], main_direction[0]])
            
            # Check which side is better (the one that's closer to the end point)
            side1 = obstacle + perp * self.barrier_distance * 1.5
            side2 = obstacle - perp * self.barrier_distance * 1.5
            
            dist1 = np.linalg.norm(self.end_point - side1)
            dist2 = np.linalg.norm(self.end_point - side2)
            
            best_side = side1 if dist1 < dist2 else side2
            
            # Create approach point (before obstacle) and departure point (after obstacle)
            safe_distance = self.barrier_distance * 1.2
            
            # Approach from the side to avoid getting too close
            approach_point = obstacle - main_direction * safe_distance + perp * np.sign(np.dot(perp, best_side - obstacle)) * safe_distance
            
            # Make sure approach point is safe from all obstacles
            approach_point = self.ensure_safe_point(approach_point)
            
            # Add approach point if it's significantly different from current position
            if np.linalg.norm(approach_point - current_pos) > 0.1:
                path.append(approach_point)
                current_pos = approach_point
            
            # Add the main avoidance point
            avoidance_point = best_side
            avoidance_point = self.ensure_safe_point(avoidance_point)
            
            if np.linalg.norm(avoidance_point - current_pos) > 0.1:
                path.append(avoidance_point)
                current_pos = avoidance_point
            
            print(f"  Added waypoint: {approach_point} (approach)")
            print(f"  Added waypoint: {avoidance_point} (avoidance)")
        
        # Add final destination
        final_point = self.ensure_safe_point(self.end_point)
        if np.linalg.norm(final_point - current_pos) > 0.1:
            path.append(final_point)
        
        # Remove any duplicate points
        clean_path = []
        for point in path:
            if len(clean_path) == 0 or np.linalg.norm(point - clean_path[-1]) > 0.1:
                clean_path.append(point)
        
        print(f"Final clean path: {clean_path}")
        return clean_path
    
    def ensure_safe_point(self, point):
        """Ensure a point is safe from all obstacles"""
        safe_point = point.copy()
        
        for obstacle in self.obstacle_positions:
            distance = np.linalg.norm(safe_point - obstacle)
            if distance < self.barrier_distance:
                # Move point away from obstacle
                direction_away = safe_point - obstacle
                if np.linalg.norm(direction_away) > 0:
                    direction_away = direction_away / np.linalg.norm(direction_away)
                safe_point = obstacle + direction_away * self.barrier_distance * 1.1
                print(f"  Adjusted unsafe point {point} to safe point {safe_point}")
        
        return safe_point
    
    def setup_plot(self):
        """Setup the plot with proper limits and labels"""
        # Combine all points to calculate limits
        all_points = [self.start_point, self.end_point] + self.obstacle_positions
        for point in self.path:
            all_points.append(point)
        
        all_x = [point[0] for point in all_points]
        all_y = [point[1] for point in all_points]
        
        min_x, max_x = min(all_x) - 2, max(all_x) + 2
        min_y, max_y = min(all_y) - 2, max(all_y) + 2
        
        self.ax.set_xlim(min_x, max_x)
        self.ax.set_ylim(min_y, max_y)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlabel('X Coordinate')
        self.ax.set_ylabel('Y Coordinate')
        self.ax.set_title(f'Moving Car - Obstacle Avoidance (Barrier: {self.barrier_distance})')
        
        # Plot the route
        for i in range(len(self.path) - 1):
            self.ax.plot([self.path[i][0], self.path[i+1][0]], 
                        [self.path[i][1], self.path[i+1][1]], 
                        'b--', alpha=0.5, linewidth=2, label='Planned Path' if i == 0 else "")
            
            # Mark waypoints
            if i > 0:  # Don't mark start point as waypoint
                self.ax.plot(self.path[i][0], self.path[i][1], 'bo', markersize=6, alpha=0.7)
        
        # Mark start and end points
        self.ax.plot(*self.start_point, 'go', markersize=10, label='Start')
        self.ax.plot(*self.end_point, 'ro', markersize=10, label='End')
        
        # Mark obstacle positions with barrier distance circles
        for i, obstacle in enumerate(self.obstacle_positions):
            self.ax.plot(*obstacle, 's', markersize=12, color='orange', 
                        label=f'Obstacle {i+1}' if i == 0 else "")
            
            # Add obstacle barrier zone circle
            barrier_zone = plt.Circle(obstacle, self.barrier_distance, color='red', alpha=0.2)
            self.ax.add_patch(barrier_zone)
            
            # Add text showing barrier distance
            self.ax.text(obstacle[0], obstacle[1] + self.barrier_distance + 0.3, 
                        f'Barrier: {self.barrier_distance}', 
                        ha='center', va='bottom', fontsize=8, color='red')
        
        self.ax.legend()
    
    def calculate_car_angle(self):
        """Calculate the angle of the car based on current direction"""
        if self.current_path_index >= len(self.path) - 1:
            return 0  # Keep last orientation when stopped
        
        current_target = self.path[self.current_path_index + 1]
        direction = current_target - self.current_position
        if np.linalg.norm(direction) == 0:
            return 0
        return math.atan2(direction[1], direction[0])
    
    def create_car_patches(self, position):
        """Create car body and wheels at given position"""
        angle = self.calculate_car_angle()
        
        # Car body (rectangle)
        car_body = Rectangle((position[0] - self.car_length/2, position[1] - self.car_width/2),
                           self.car_length, self.car_width,
                           angle=np.degrees(angle), rotation_point='center',
                           facecolor='blue', edgecolor='black', alpha=0.8)
        
        # Wheels
        wheel_radius = 0.1
        wheel_positions = [
            (-self.car_length/3, -self.car_width/2 - wheel_radius/2),
            (self.car_length/3, -self.car_width/2 - wheel_radius/2),
            (-self.car_length/3, self.car_width/2 + wheel_radius/2),
            (self.car_length/3, self.car_width/2 + wheel_radius/2)
        ]
        
        wheels = []
        for wx, wy in wheel_positions:
            rotated_x = wx * math.cos(angle) - wy * math.sin(angle)
            rotated_y = wx * math.sin(angle) + wy * math.cos(angle)
            
            wheel = Circle((position[0] + rotated_x, position[1] + rotated_y),
                         wheel_radius, facecolor='black', alpha=0.7)
            wheels.append(wheel)
        
        return car_body, wheels
    
    def is_too_close_to_obstacle(self, position):
        """Check if position is too close to any obstacle and log if it is"""
        for i, obstacle in enumerate(self.obstacle_positions):
            distance = np.linalg.norm(position - obstacle)
            if distance < self.barrier_distance:
                print(f"🚨 BARRIER VIOLATION at position {position}!")
                print(f"   Too close to obstacle {i+1} at {obstacle}")
                print(f"   Distance: {distance:.3f}, Barrier: {self.barrier_distance}")
                return True, i
        return False, -1
    
    def is_waypoint_visited(self, waypoint):
        """Check if a waypoint has been visited (using tuple comparison for numpy arrays)"""
        waypoint_tuple = tuple(waypoint)
        for visited in self.visited_waypoints:
            if np.allclose(waypoint, visited):
                return True
        return False
    
    def log_waypoint_info(self):
        """Log information about current waypoint"""
        if self.current_path_index < len(self.path):
            current_waypoint = self.path[self.current_path_index]
            
            # Check if this waypoint has been visited using proper comparison
            if not self.is_waypoint_visited(current_waypoint):
                self.visited_waypoints.append(current_waypoint.copy())
                print(f"📍 Reached waypoint {self.current_path_index}: {current_waypoint}")
                
                # Check if this waypoint is safe
                is_unsafe, obstacle_idx = self.is_too_close_to_obstacle(current_waypoint)
                if is_unsafe:
                    print(f"   ⚠️  WARNING: Waypoint is inside barrier zone of obstacle {obstacle_idx + 1}")
    
    def update(self, frame):
        """Update function for animation"""
        # Clear previous car
        for patch in self.ax.patches[:]:
            if isinstance(patch, (Rectangle, Circle)):
                patch_color = patch.get_facecolor()
                # Check if it's a car part (blue, green, or black)
                if (len(patch_color) > 0 and 
                    (np.allclose(patch_color, [0., 0., 1., 0.8]) or
                     np.allclose(patch_color, [0., 1., 0., 1.]) or
                     np.allclose(patch_color, [0., 0., 0., 0.7]))):
                    patch.remove()
        
        if not self.has_reached_end:
            # Move along current path segment
            if self.current_path_index < len(self.path) - 1:
                segment_start = self.path[self.current_path_index]
                segment_end = self.path[self.current_path_index + 1]
                
                # Log when we start a new segment (only at the beginning of segment)
                if self.progress == 0:
                    print(f"🛣️  Starting segment {self.current_path_index + 1}: {segment_start} -> {segment_end}")
                    self.log_waypoint_info()
                
                self.progress += self.speed
                
                if self.progress >= 1.0:
                    # Move to next segment
                    self.progress = 0.0
                    self.current_path_index += 1
                    
                    if self.current_path_index >= len(self.path) - 1:
                        self.has_reached_end = True
                        print("🎉 Car has reached the destination!")
                        self.log_waypoint_info()
                    else:
                        print(f"➡️  Moving to next waypoint: {self.path[self.current_path_index + 1]}")
                
                if not self.has_reached_end:
                    self.current_position = segment_start + self.progress * (segment_end - segment_start)
                    
                    # Continuous safety check (but don't spam the console)
                    if frame % 10 == 0:  # Only check every 10 frames to reduce spam
                        is_unsafe, obstacle_idx = self.is_too_close_to_obstacle(self.current_position)
                        if is_unsafe:
                            print(f"🚨 CONTINUOUS CHECK: Car at {self.current_position} is inside barrier of obstacle {obstacle_idx + 1}")
            else:
                self.has_reached_end = True
                self.current_position = self.end_point.copy()
        
        # Create new car
        car_body, wheels = self.create_car_patches(self.current_position)
        
        # Add car to plot
        self.ax.add_patch(car_body)
        for wheel in wheels:
            self.ax.add_patch(wheel)
        
        # Update title with current position and status
        status = "STOPPED" if self.has_reached_end else f"MOVING (Segment {self.current_path_index + 1}/{len(self.path) - 1})"
        is_unsafe, _ = self.is_too_close_to_obstacle(self.current_position)
        obstacle_warning = " - BARRIER VIOLATION!" if is_unsafe else ""
        self.ax.set_title(f'Moving Car - Position: ({self.current_position[0]:.2f}, {self.current_position[1]:.2f}) - {status}{obstacle_warning}')
        
        # Change car color when stopped or when too close to obstacle
        if self.has_reached_end:
            car_body.set_facecolor('green')
            car_body.set_alpha(1.0)
        elif is_unsafe:
            car_body.set_facecolor('red')
        
        return [car_body] + wheels
    
    def animate(self):
        """Start the animation"""
        # Create initial car
        car_body, wheels = self.create_car_patches(self.start_point)
        self.ax.add_patch(car_body)
        for wheel in wheels:
            self.ax.add_patch(wheel)
        
        # Start animation
        self.animation = FuncAnimation(self.fig, self.update, frames=None,
                                     interval=50, blit=False, repeat=True)
        
        plt.show()

def get_user_coordinates():
    """Get start and end coordinates from user"""
    print("Enter coordinates for the start and end points:")
    
    try:
        start_x = float(input("Start point X coordinate: "))
        start_y = float(input("Start point Y coordinate: "))
        end_x = float(input("End point X coordinate: "))
        end_y = float(input("End point Y coordinate: "))
        
        return (start_x, start_y), (end_x, end_y)
    except ValueError:
        print("Please enter valid numbers!")
        return get_user_coordinates()

def get_obstacle_positions():
    """Get obstacle positions from user"""
    try:
        num_obstacles = int(input("How many obstacle positions? "))
        obstacles = []
        
        for i in range(num_obstacles):
            print(f"Obstacle {i+1}:")
            obs_x = float(input("  X coordinate: "))
            obs_y = float(input("  Y coordinate: "))
            obstacles.append((obs_x, obs_y))
        
        return obstacles
    except ValueError:
        print("Please enter valid numbers!")
        return get_obstacle_positions()

def get_barrier_distance():
    """Get barrier distance from user"""
    try:
        barrier_dist = float(input("Enter barrier distance (minimum distance from obstacles, e.g., 1.5): "))
        return max(0.5, barrier_dist)  # Minimum barrier distance of 0.5
    except ValueError:
        print("Please enter a valid number! Using default barrier distance of 1.5")
        return 1.5

def get_speed():
    """Get animation speed from user"""
    try:
        speed = float(input("Enter animation speed (0.01 to 0.1, default 0.02): ") or "0.02")
        return max(0.001, min(0.1, speed))
    except ValueError:
        print("Using default speed 0.02")
        return 0.02

def main():
    """Main function to run the animation"""
    print("Moving Car Animation - Advanced Obstacle Avoidance")
    print("=================================================")
    print("The car will move from start to end while maintaining safe distance from obstacles.")
    print("Detailed logging will show waypoints and barrier violations.")
    print()
    
    # Get coordinates from user
    start_point, end_point = get_user_coordinates()
    
    # Get obstacle positions
    obstacle_positions = get_obstacle_positions()
    
    # Get barrier distance
    barrier_distance = get_barrier_distance()
    
    # Get speed from user
    speed = get_speed()
    
    # Create and run animation
    car_animation = MovingCar(start_point, end_point, obstacle_positions, barrier_distance, speed=speed)
    car_animation.animate()

if __name__ == "__main__":
    main()